package xiangshan.backend.fu

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import utils._
import utility._
import xiangshan._
import xiangshan.TMUOpType._
import xiangshan.cache.mmu._
import xiangshan.backend.fu.PMPRespBundle
import freechips.rocketchip.diplomacy.{IdRange, LazyModule, LazyModuleImp, TransferSizes}
import freechips.rocketchip.tilelink._
import xiangshan.backend.rob.RobPtr
import java.lang.reflect.Parameter


class TmuDataInput(implicit p: Parameters) extends XSBundle {
  val src    = Vec(2, UInt(XLEN.W))
  val imm    = UInt(32.W)
  val func   = FuOpType()
  val robIdx = new RobPtr
}

trait TmuParams extends HasXSParameter {
  val numTmm    : Int = 8
  val numTrows  : Int = 16
  val numTcolsb : Int = 64 // bytes per row

  val tile_idx_w : Int = log2Ceil(numTmm)
  val row_idx_w  : Int = log2Ceil(numTrows)
  val row_data_w : Int = numTcolsb * 8

  val numTcolsw : Int = numTcolsb / 4 // words per row

  val numTileBbuf: Int = 3 // number of tileB buffer
  val tileBbuf_ptr_w : Int = log2Ceil(numTileBbuf)
  def update_tileBbuf_ptr(ptr: UInt): UInt = Mux(ptr === (numTileBbuf-1).U, 0.U, ptr + 1.U)

  // row walk pointer
  case class RowWalkPtr(init: Int = 0) {
    private val ptr = RegInit(init.U(log2Ceil(numTrows + 1).W))
    def value: UInt = ptr(row_idx_w-1, 0)
    def next_value: UInt = value + 1.U // for synchronous read
    def update(): Unit = {
      ptr := Mux(overflow, ptr, ptr + 1.U)
    }
    def overflow: Bool = ptr === numTrows.U
    def ready_go: Bool = ptr === (numTrows-1).U || overflow // ptr = 15 时就可以拉高 out_valid，在下一个上升沿握手
    def reset(): Unit = {
      ptr := init.U
    }
  }

  // stage info stored in registers
  case class TmuStageInfo(in_fire: Bool, out_fire: Bool, in_bits: TmuDataInput, needVaddr: Boolean = false) {
    val valid = RegInit(false.B)
    when(in_fire) {
      valid := true.B
    }.elsewhen(out_fire) {
      valid := false.B
    }

    val regs = RegEnable(in_bits, in_fire)
    def isTileLoad:  Bool = regs.func === TMUOpType.tileload
    def isTileStore: Bool = regs.func === TMUOpType.tilestore
    def isTDP:       Bool = isTdp(regs.func)
    def tmmA_sign:   Bool = regs.func(1) === "b1".U
    def tmmB_sign:   Bool = regs.func(0) === "b1".U
    def func:        UInt = regs.func

    def tmmA: UInt = regs.imm(2, 0)
    def tmmB: UInt = regs.imm(5, 3)
    def tmmC: UInt = regs.imm(8, 6)

    def robIdx: RobPtr = regs.robIdx

    def base_vaddr: UInt = regs.src(0) + ZeroExt(Cat(regs.imm(31, 3), 0.U(3.W)), VAddrBits) // 目标块在内存中的起始虚地址
    def stride    : UInt = regs.src(1)      // 主轴长度
    def row_vaddr_vec: Vec[UInt] = VecInit( // 每行的虚拟地址(实际不会这样使用)
      (0 until numTrows).scanLeft(base_vaddr) { (vaddr, _) => vaddr + stride }
    )
    def mem_op : UInt = Cat(isTileLoad, isTileStore)

    val row_vaddr = Option.when(needVaddr)(Reg(UInt(VAddrBits.W)))
    if (needVaddr) {
      when(in_fire) {
        row_vaddr.get := in_bits.src(0) + ZeroExt(Cat(in_bits.imm(31, 3), 0.U(3.W)), VAddrBits)
      }
    }
    def updateRowVaddr(): Unit = {
      row_vaddr.get := row_vaddr.get + stride
    }
  }

  val tileLSQueue_sz = 32
  val mem_op_typ     = 2  // {load, store}
  // TileLink clinet node params
  val tmuClientParameters = TLMasterPortParameters.v1(
    clients = Seq(TLMasterParameters.v1(
      "tmu",
      sourceId = IdRange(0, tileLSQueue_sz) // [0, tileLSQueue_sz)
    )),
  )
  val sourceIDWidth = log2Ceil(tileLSQueue_sz)

  val numBurst = row_data_w / l1BusDataWidth
  case class TLTransCnt() {
    private val cnt = RegInit(0.U(log2Ceil(numBurst).W))
    def value: UInt = cnt
    def update(): Unit = {
      cnt := cnt + 1.U
    }
    def last: Bool = cnt === (numBurst-1).U
  }
  object TLState {
    val s_idle :: s_wait_tlb :: s_wait_a :: s_wait_d :: s_done :: Nil = Enum(5)
    def apply() = UInt(s_idle.getWidth.W)
  }
}

// Tmm register (a rough implementation)
class TileReg (implicit val p: Parameters) extends Module with TmuParams {
  val io = IO(new Bundle {
    val rrow  = Input(UInt(row_idx_w.W))
    val rdata = Output(UInt(row_data_w.W))
    val wrow  = Input(UInt(row_idx_w.W))
    val wen   = Input(Bool())
    val wdata = Input(UInt(row_data_w.W))
  })

  private val tile = Reg(Vec(numTrows, UInt(row_data_w.W)))
  // read: 1 cycle latency; write: 1 cycle latency
  // no write to read bypass
  io.rdata := tile(GatedRegNext(io.rrow))
  when(io.wen) {
    tile(io.wrow) := io.wdata
  }
}


// Tmu memory access bundle
class TmuMemBus (implicit p: Parameters) extends XSBundle with TmuParams {
  val req = DecoupledIO(new Bundle {
    val source = UInt(sourceIDWidth.W)
    val paddr  = UInt(PAddrBits.W)
    val wdata  = UInt(l1BusDataWidth.W)
    val isWrite = Bool()
  })
  val resp = Flipped(DecoupledIO(new Bundle {
    val source = UInt(sourceIDWidth.W)
    val rdata  = UInt(l1BusDataWidth.W)
    val isWrite = Bool()
  }))


  def ConnectClientNode(node: TLClientNode): Unit = {
    val (bus, edge) = node.out.head
    bus.a.valid := req.valid
    req.ready   := bus.a.ready
    bus.a.bits  := Mux(req.bits.isWrite, 
      edge.Put(fromSource = req.bits.source, toAddress = req.bits.paddr, lgSize = log2Ceil(l1BusDataWidth/8).U, data = req.bits.wdata)._2,
      edge.Get(fromSource = req.bits.source, toAddress = req.bits.paddr, lgSize = log2Ceil(l1BusDataWidth/8).U)._2
    )

    resp.valid  := bus.d.valid
    bus.d.ready := resp.ready
    resp.bits.source  := bus.d.bits.source
    resp.bits.rdata   := bus.d.bits.data
    resp.bits.isWrite := bus.d.bits.opcode === TLMessages.AccessAck
  }
}



// HINT: Tile Matrix Unit
// Tile Matrix Unit is a special functional unit that is used to accelerate matrix operations.
class TmuModule (implicit p: Parameters) extends XSModule with TmuParams {
  val io = IO(new Bundle() {
    // Exu interface
    val in  = Flipped(Decoupled(new TmuDataInput))
    val out = Decoupled(new Bundle{
      val robIdx = new RobPtr
    })
    // Mem interface
    val tlb  = new TlbRequestIO()
    // val pmp  = Flipped(new PMPRespBundle()) // arrive same to tlb now
    val memBus = new TmuMemBus
  })

  ///////////////////////////////////////////////
  // inner implemmentation of Tile Matrix Unit!
  ///////////////////////////////////////////////

  // tile register file
  val tiles = Seq.fill(numTmm)(Module(new TileReg))
  val tiles_rdatas = VecInit(tiles.map(_.io.rdata))

  // registers for tdp operation
  val tileB_buf = Reg(Vec(numTileBbuf, Vec(numTrows, UInt(row_data_w.W)))) // multiple buffer for B
  val tileA_buf = Reg(MixedVec((0 until numTrows).map(i => UInt(((numTcolsw - i)* 32).W))))
  val tileC_buf = Reg(Vec(numTrows, UInt(row_data_w.W)))

  val tileB_words = VecInit(Seq.tabulate(numTileBbuf)(i => 
    VecInit((0 until numTrows).map(r => 
      VecInit((0 until numTcolsw).map(c => tileB_buf(i)(r)(c*32+31, c*32))))
    )
  ))
  val tileA_words = VecInit((0 until numTrows).map(r => tileA_buf(r)(31, 0)))
  val tileC_words = VecInit((0 until numTrows).map(r => 
    VecInit((0 until numTcolsw).map(c => tileC_buf(r)(c*32+31, c*32)))
  ))

  // TMU Load/Store Queue
  private val lsqIO = Module(new TmuLoadStoreQueue).io
  lsqIO.tlb  <> io.tlb
  lsqIO.memBus <> io.memBus


  //////////////////////////////////////
  // Stage 1 for Tile Matrix Unit
  // * for TDP, pump tmmB into tileB_buf
  // * for TILELOADD/STIELSTORED, add load/store ctrl info into lsQueue
  //////////////////////////////////////

  val s1_out_valid = Wire(Bool())
  val s2_in_ready  = Wire(Bool())
  val s1_s2_fire   = s1_out_valid && s2_in_ready // only for TDP
  val s1_lsq_done  = Wire(Bool()) // only for tilelaodd/tilestored
  val s1_stall     = Wire(Bool())

  val s1_regs = TmuStageInfo(io.in.fire, s1_s2_fire || s1_lsq_done, io.in.bits, needVaddr = true)
  when(io.in.fire) {
    when(s1_regs.isTileLoad) {
      printf(p"[TMU] tileloadd tmm${s1_regs.tmmC}, vaddr = 0x${Hexadecimal(s1_regs.base_vaddr)}, stride = 0x${Hexadecimal(s1_regs.stride)}\n")
    }.elsewhen(s1_regs.isTileStore) {
      printf(p"[TMU] tilestored tmm${s1_regs.tmmC}, vaddr = 0x${Hexadecimal(s1_regs.base_vaddr)}, stride = 0x${Hexadecimal(s1_regs.stride)}\n")
    }.elsewhen(s1_regs.isTDP) {
      printf(p"[TMU] tdpb??d tmm${s1_regs.tmmC}, tmm${s1_regs.tmmA}, tmm${s1_regs.tmmB}, sign = ${s1_regs.tmmA_sign}, ${s1_regs.tmmB_sign}\n")
    }.otherwise {
      printf(p"[TMU] unknown tmu op!\n")
    }
  }

  val s1_row_walk_ptr = RowWalkPtr()
  when(s1_s2_fire || s1_lsq_done) {
    s1_row_walk_ptr.reset()
  }.elsewhen(s1_regs.valid && s1_regs.isTDP && !s1_stall) {
    s1_row_walk_ptr.update()
  }.elsewhen(s1_regs.valid && (s1_regs.isTileLoad || s1_regs.isTileStore) && lsqIO.enq.fire){
    s1_row_walk_ptr.update()
  }

  // pump data into tileB_buf
  val s1_tileB_buf_ptr = RegInit(0.U(tileBbuf_ptr_w.W)) // use tileB_buf(0) or tileB_buf(1) or ...
  when(s1_s2_fire) {
    s1_tileB_buf_ptr := update_tileBbuf_ptr(s1_tileB_buf_ptr)
  }
  
  when(s1_regs.valid && s1_regs.isTDP && !s1_row_walk_ptr.overflow) {
    for (r <- 0 until numTrows) {
      if(r == numTrows - 1) { // pump tileB into the last row
        tileB_buf(s1_tileB_buf_ptr)(r) := tiles_rdatas(s1_regs.tmmB)
      }else {
        tileB_buf(s1_tileB_buf_ptr)(r) := tileB_buf(s1_tileB_buf_ptr)(r+1)
      }
    }
  }

  // add load/store ctrl info into lsQueue
  lsqIO.enq.valid := s1_regs.valid && (s1_regs.isTileLoad || s1_regs.isTileStore)
  lsqIO.enq.bits        := DontCare
  lsqIO.enq.bits.tile   := s1_regs.tmmC
  lsqIO.enq.bits.row    := s1_row_walk_ptr.value
  lsqIO.enq.bits.mem_op := s1_regs.mem_op
  lsqIO.enq.bits.vaddr  := s1_regs.row_vaddr.get

  when(lsqIO.enq.fire) {
    s1_regs.updateRowVaddr()
  }

  s1_lsq_done := lsqIO.enq.fire && s1_row_walk_ptr.ready_go
  s1_out_valid := s1_regs.isTDP && s1_regs.valid && s1_row_walk_ptr.overflow
  io.in.ready  := s1_s2_fire || s1_lsq_done || !s1_regs.valid

  /////////////////////////////////////////
  // Stage 2 for Tile Matrix Unit
  // * for TDP, pump tmmC into tileC_buf, pump tmmA into tileA_buf
  //            and matmul computing start
  /////////////////////////////////////////

  val s2_out_valid = Wire(Bool())
  val s3_in_ready  = Wire(Bool())
  val s2_s3_fire   = s2_out_valid && s3_in_ready
  val s2_stall     = Wire(Bool())

  val s2_regs = TmuStageInfo(s1_s2_fire, s2_s3_fire, s1_regs.regs)
  val s2_tileB_buf_ptr = RegEnable(s1_tileB_buf_ptr, s1_s2_fire) // use tileB_buf(0) or tileB_buf(1) or ...
  
  // 16 * 16 DPAUnits
  val DPAMatrix = Seq.fill(numTrows)(Seq.fill(numTcolsw)(DPAUnit("int8")))
  for (i <- 0 until numTrows) {
    for (j <- 0 until numTcolsw) {
      DPAMatrix(i)(j).connect_in(tileA_words(i), tileB_words(s2_tileB_buf_ptr)(i)(j), tileC_words(i)(j), s2_regs.func)
    }
  }

  val s2_row_walk_ptr = RowWalkPtr()
  when(s2_regs.isTDP && s2_s3_fire) {
    s2_row_walk_ptr.reset()
  }.elsewhen(s2_regs.isTDP && s2_regs.valid && !s2_stall) {
    s2_row_walk_ptr.update()
  }


  s2_out_valid := s2_regs.valid && s2_row_walk_ptr.overflow
  s2_in_ready  := s2_s3_fire || !s2_regs.valid

  ////////////////////////////////////
  // Stage 3 for Tile Matrix Unit
  // * only TDP has stage 3
  // * push tmmC out of tileC_buf, push tmmA out of tileA_buf
  // * and store tmmC back to tile register file
  ////////////////////////////////////

  val s3_regs = TmuStageInfo(s2_s3_fire, io.out.fire, s2_regs.regs)
  val s3_tileB_buf_ptr = RegEnable(s2_tileB_buf_ptr, s2_s3_fire) // use tileB_buf(0) or tileB_buf(1) or ...
  
  val s3_row_walk_ptr = RowWalkPtr()
  when(s3_regs.isTDP && io.out.fire) {
    s3_row_walk_ptr.reset()
  }.elsewhen(s3_regs.isTDP && s3_regs.valid) {
    s3_row_walk_ptr.update()
  }

  val s2_move = s2_regs.isTDP && s2_regs.valid && !s2_row_walk_ptr.overflow
  val s3_move = s3_regs.isTDP && s3_regs.valid && !s3_row_walk_ptr.overflow
  val row_move = s2_move || s3_move

  // pump data into tileA_buf and tileC_buf
  when(row_move) {
    for (r <- 0 until numTrows) {
      if(r == 0) {
        tileA_buf(r) := tiles_rdatas(s2_regs.tmmA)
      }else {
        var w = tileA_buf(r-1).getWidth
        tileA_buf(r) := tileA_buf(r-1)(w-1, 32) // move the remaining data in (r-1)_th row to r_th row
      }
    }
  }

  when(row_move) {
    for (r <- 0 until numTrows) {
      if (r == 0) {
        tileC_buf(r) := tiles_rdatas(s2_regs.tmmC)
      }else {
        tileC_buf(r) := VecInit((0 until numTcolsw).map(c => DPAMatrix(r-1)(c).out)).asUInt // move the temp results in (r-1)_th row to r_th row
      }
    }
  }

  s3_in_ready  := io.out.fire || !s3_regs.valid

  val tileLS_ready_go = lsqIO.deq.valid && lsqIO.deq.bits.row === (numTrows-1).U
  lsqIO.deq.ready := Mux(tileLS_ready_go, io.out.ready, true.B)

  io.out.valid       := Mux(tileLS_ready_go, true.B, s3_regs.valid && s3_row_walk_ptr.ready_go)
  io.out.bits.robIdx := Mux(tileLS_ready_go, lsqIO.deq.bits.robIdx, s3_regs.robIdx)


  /////////////////////////////////
  // Manage tiles read/write ports
  /////////////////////////////////
  val s1_ren = VecInit((0 until numTmm).map(i => s1_regs.valid && s1_regs.isTDP && (s1_regs.tmmB === i.U)))
  val s2_ren = VecInit((0 until numTmm).map(i => s2_regs.valid && s2_regs.isTDP && (s2_regs.tmmA === i.U || s2_regs.tmmC === i.U)))
  val s3_wen = VecInit((0 until numTmm).map(i => s3_regs.valid && s3_regs.isTDP && (s3_regs.tmmC === i.U)))
  val lsq_ren = VecInit((0 until numTmm).map(i => lsqIO.tileData.ren && (lsqIO.tileData.rtile === i.U)))
  val lsq_wen = VecInit((0 until numTmm).map(i => lsqIO.tileData.wen && (lsqIO.tileData.wtile === i.U)))


  for (i <- 0 until numTmm) {
    tiles(i).io.rrow  := PriorityMux(Seq(
      lsq_ren(i) -> lsqIO.tileData.rrow,
      s2_ren(i)  -> s2_row_walk_ptr.value,
      s1_ren(i)  -> s1_row_walk_ptr.value
    ))
    tiles(i).io.wrow  := Mux(lsq_wen(i), lsqIO.tileData.wrow, s3_row_walk_ptr.value)
    tiles(i).io.wen   := s3_wen(i) || lsq_wen(i)
    tiles(i).io.wdata := Mux(lsq_wen(i), lsqIO.tileData.wdata, tileC_buf.last) // data pop from the last line
  }

  lsqIO.tileData.rdata := tiles_rdatas(lsqIO.tileData.rtile)

  s1_stall := s1_ren.zip(s2_ren).map(r => r._1 && r._2).reduce(_ || _) || // s1 and s2 read the same tile
              s1_ren.zip(s3_wen).map(r => r._1 && r._2).reduce(_ || _)    // s1 read and s3 write the same tile
  s2_stall := s2_ren.zip(s3_wen).map(r => r._1 && r._2).reduce(_ || _)    // s2 read and s3 write the same tile

}

// Dot-product Accumulate
// very rough implementation!!
class int8DP4A extends Module {
  val io = IO(new Bundle {
    val func  = Input(FuOpType())
    val a_vec = Input(Vec(4, UInt(8.W)))
    val b_vec = Input(Vec(4, UInt(8.W)))
    val c_in  = Input(UInt(32.W))
    val c_out = Output(UInt(32.W))
  })

  val a_sign = io.func(1) === "b1".U
  val b_sign = io.func(0) === "b1".U

  val a_vec_widen = io.a_vec.map(a => Mux(a_sign, SignExt(a, 32), ZeroExt(a, 32)))
  val b_vec_widen = io.b_vec.map(b => Mux(b_sign, SignExt(b, 32), ZeroExt(b, 32)))

  val dp = a_vec_widen.zip(b_vec_widen).map { case(a, b) =>
    LookupTreeDefault(io.func, 0.U, Seq(
      (TMUOpType.tdpbss, (a.asSInt * b.asSInt).asUInt),
      (TMUOpType.tdpbsu, (a.asSInt * b.asUInt).asUInt),
      (TMUOpType.tdpbus, (a.asUInt * b.asSInt).asUInt),
      (TMUOpType.tdpbuu, (a.asUInt * b.asUInt).asUInt))
    )
  }.reduce(_ + _)

  io.c_out := io.c_in + dp
}

case class DPAUnit(data_typ: String = "int8") {
  val dpa = if (data_typ == "int8") Some(Module(new int8DP4A)) else None

  def connect_in(a: UInt, b: UInt, c: UInt, func: UInt): Unit = {
    dpa.get.io.func  := func
    dpa.get.io.a_vec := VecInit((0 until 4).map(i => a(8*i+7, 8*i)))
    dpa.get.io.b_vec := VecInit((0 until 4).map(i => b(8*i+7, 8*i)))
    dpa.get.io.c_in  := c
  }

  def connect_in(a_vec: Vec[UInt], b_vec: Vec[UInt], c: UInt, func: UInt): Unit = {
    dpa.get.io.func  := func
    dpa.get.io.a_vec := a_vec
    dpa.get.io.b_vec := b_vec
    dpa.get.io.c_in  := c
  }

  def out: UInt = dpa.get.io.c_out
}


class TmuLSQueueEntry(implicit p: Parameters) extends XSBundle with TmuParams{
  val tile    = UInt(tile_idx_w.W)
  val row     = UInt(row_idx_w.W)
  val mem_op  = UInt(mem_op_typ.W)
  val vaddr   = UInt(VAddrBits.W)
  val paddr   = UInt(PAddrBits.W)
  val paddr_v = Bool()
  val state   = TLState()
  val robIdx  = new RobPtr

  def isLoad:  Bool = mem_op(1)
  def isStore: Bool = mem_op(0)
  def valid:   Bool = state =/= TLState.s_idle && state =/= TLState.s_done
}

class TmuLSQueueToTiles(implicit val p: Parameters) extends Bundle with TmuParams {
  val rtile = Output(UInt(tile_idx_w.W))
  val ren   = Output(Bool())
  val rrow  = Output(UInt(row_idx_w.W))
  val rdata = Input(UInt(row_data_w.W))
  val wtile = Output(UInt(tile_idx_w.W))
  val wen   = Output(Bool())
  val wrow  = Output(UInt(row_idx_w.W))
  val wdata = Output(UInt(row_data_w.W))
}


// TMU laod/store queue
class TmuLoadStoreQueue (implicit p: Parameters) extends XSModule
with TmuParams with HasCircularQueuePtrHelper {
  val io = IO(new Bundle {
    val enq      = Flipped(Decoupled(new TmuLSQueueEntry))
    val deq      = Decoupled(new TmuLSQueueEntry)
    val tileData = new TmuLSQueueToTiles

    val tlb  = new TlbRequestIO()
    // val pmp  = Flipped(new PMPRespBundle()) // arrive same to tlb now
    val memBus = new TmuMemBus
  })

  class TmuLSQueuePtr extends CircularQueuePtr[TmuLSQueuePtr](p => tileLSQueue_sz) {}
  
  // queues
  val ctrl_queue = Reg(Vec(tileLSQueue_sz, new Bundle{
    val tile    = UInt(tile_idx_w.W)
    val row     = UInt(row_idx_w.W)
    val mem_op  = UInt(mem_op_typ.W)
    val vaddr   = UInt(VAddrBits.W)
    val robIdx  = new RobPtr
  }))
  val paddr_queue = Reg(Vec(tileLSQueue_sz, new Bundle{
    val paddr   = UInt(PAddrBits.W)
    val paddr_v = Bool()
  }))
  val tlState_queue = RegInit(VecInit(Seq.fill(tileLSQueue_sz)(TLState.s_idle)))

  def ToTmuLSQueueEntry(ptr: TmuLSQueuePtr): TmuLSQueueEntry = {
    val entry = Wire(new TmuLSQueueEntry)
    entry.tile    := ctrl_queue(ptr.value).tile
    entry.row     := ctrl_queue(ptr.value).row
    entry.mem_op  := ctrl_queue(ptr.value).mem_op
    entry.vaddr   := ctrl_queue(ptr.value).vaddr
    entry.robIdx  := ctrl_queue(ptr.value).robIdx
    entry.paddr   := paddr_queue(ptr.value).paddr
    entry.paddr_v := paddr_queue(ptr.value).paddr_v
    entry.state   := tlState_queue(ptr.value)
    entry
  }

  val in_ptr     = RegInit(0.U.asTypeOf(new TmuLSQueuePtr))
  val tlb_ptr    = RegInit(0.U.asTypeOf(new TmuLSQueuePtr))
  val tlReq_ptr  = RegInit(0.U.asTypeOf(new TmuLSQueuePtr))
  val tlResp_ptr = RegInit(0.U.asTypeOf(new TmuLSQueuePtr))
  val out_ptr    = RegInit(0.U.asTypeOf(new TmuLSQueuePtr))

  io.enq.ready := !isFull(in_ptr, out_ptr)
  when(io.enq.fire) {
    in_ptr := in_ptr + 1.U
  }

  io.deq.valid := isAfter(out_ptr, tlResp_ptr)
  when(io.deq.fire) {
    out_ptr := out_ptr + 1.U
  }
  io.deq.bits := ToTmuLSQueueEntry(out_ptr)

  // TLB request
  val tlb_req_entry = ToTmuLSQueueEntry(tlb_ptr)
  io.tlb.req.valid              := tlb_req_entry.valid // block tlb
  io.tlb.req.bits.cmd           := Mux(tlb_req_entry.isLoad, TlbCmd.read, TlbCmd.write)
  io.tlb.req.bits.vaddr         := tlb_req_entry.vaddr
  io.tlb.req.bits.fullva        := DontCare
  io.tlb.req.bits.checkfullva   := false.B
  io.tlb.req.bits.hyperinst     := false.B
  io.tlb.req.bits.hlvx          := false.B
  io.tlb.req.bits.size          := log2Ceil(numTcolsb).U // 2^size = 64B
  io.tlb.req.bits.kill          := false.B
  io.tlb.req.bits.memidx.is_ld  := tlb_req_entry.isLoad
  io.tlb.req.bits.memidx.is_st  := tlb_req_entry.isStore
  io.tlb.req.bits.memidx.idx    := 0.U
  io.tlb.req.bits.isPrefetch    := false.B
  io.tlb.req.bits.no_translate  := false.B
  io.tlb.req.bits.pmp_addr      := RegEnable(io.tlb.resp.bits.paddr(0), io.tlb.resp.fire) // pmp check not activated in tmu

  io.tlb.req.bits.debug         := DontCare

  io.tlb.req_kill := false.B
  io.tlb.resp.ready := tlb_req_entry.valid && !io.tlb.resp.bits.miss
  // tlb_ptr 指针的更新
  when(io.tlb.resp.fire) {
    tlb_ptr := tlb_ptr + 1.U
  }

  // TileLink request
  val tlReq_entry = ToTmuLSQueueEntry(tlReq_ptr)
  io.tileData.rtile := tlReq_entry.tile
  io.tileData.rrow  := tlReq_entry.row
  
  // load to store check!
  val l2sCheck = tlState_queue.zip(ctrl_queue).map { case (state, ctrl) =>
    state === TLState.s_wait_d && ctrl.mem_op(1) &&
    ctrl.tile === tlReq_entry.tile && ctrl.row === tlReq_entry.row // 未写回的 load 请求
  }.reduce(_ || _)
  io.tileData.ren   := tlReq_entry.valid && tlReq_entry.isStore && !l2sCheck && tlReq_entry.paddr_v // 已经完成 vaddr -> paddr 转换
  val tileRdata_valid = ValidHold(io.tileData.ren, io.memBus.req.fire)

  val tileReq_cnt = TLTransCnt()
  when(io.memBus.req.fire) {
    tileReq_cnt.update()
  }
  val tileRdata_vec = VecInit((0 until numBurst).map(i => io.tileData.rdata(l1BusDataWidth*(i+1)-1, l1BusDataWidth*i)))
  
  io.memBus.req.valid := tileRdata_valid || tlReq_entry.valid && tlReq_entry.isLoad && tlReq_entry.paddr_v
  io.memBus.req.bits.source  := tlReq_ptr.value
  io.memBus.req.bits.paddr   := tlReq_entry.paddr
  io.memBus.req.bits.wdata   := tileRdata_vec(tileReq_cnt.value)
  io.memBus.req.bits.isWrite := tlReq_entry.isStore

  val tlReqDone = io.memBus.req.fire && tlReq_entry.isLoad ||
                  io.memBus.req.fire && tlReq_entry.isStore && tileReq_cnt.last

  // tlReq_ptr 指针的更新
  when(tlReqDone) {
    tlReq_ptr := tlReq_ptr + 1.U
  }

  // TileLink response
  io.memBus.resp.ready := true.B // LSQueue always ready for response
  val tlResp_entry = (io.memBus.resp.bits.source)
  val tlResp_cnt = TLTransCnt()
  val tileWdata_buf = Reg(Vec(numBurst-1, UInt(l1BusDataWidth.W)))
  when(io.memBus.resp.fire && !io.memBus.resp.bits.isWrite) { // response for Get
    tlResp_cnt.update()
    when(!tlResp_cnt.last) {
      tileWdata_buf(tlResp_cnt.value) := io.memBus.resp.bits.rdata
    }
  }
  val tileWdata = Cat(io.memBus.resp.bits.rdata, tileWdata_buf.asUInt)
  val tlRespDone = io.memBus.resp.fire && !io.memBus.resp.bits.isWrite && tlResp_cnt.last || 
                   io.memBus.resp.fire && io.memBus.resp.bits.isWrite

  io.tileData.wtile := ctrl_queue(io.memBus.resp.bits.source).tile
  io.tileData.wrow  := ctrl_queue(io.memBus.resp.bits.source).row
  io.tileData.wdata := tileWdata
  io.tileData.wen   := io.memBus.resp.fire && !io.memBus.resp.bits.isWrite && tlResp_cnt.last // write back to tmm when the last beat come

  // tlResp_ptr 指针的更新
  when(tlState_queue(tlResp_ptr.value) === TLState.s_done || tlRespDone && tlResp_ptr.value === io.memBus.resp.bits.source) {
    tlResp_ptr := tlResp_ptr + 1.U
  }

  // LSQueue 表项的更新
  for(i <- 0 until tileLSQueue_sz) {
    when(io.enq.fire && in_ptr.value === i.U) {
      ctrl_queue(i).tile   := io.enq.bits.tile
      ctrl_queue(i).row    := io.enq.bits.row
      ctrl_queue(i).mem_op := io.enq.bits.mem_op
      ctrl_queue(i).vaddr  := io.enq.bits.vaddr
      ctrl_queue(i).robIdx := io.enq.bits.robIdx
    }
  }
  for(i <- 0 until tileLSQueue_sz) {
    when(io.tlb.resp.fire && tlb_ptr.value === i.U) {
      paddr_queue(i).paddr   := io.tlb.resp.bits.paddr(0)
      paddr_queue(i).paddr_v := true.B
    }.elsewhen(io.deq.fire && out_ptr.value === i.U) {
      paddr_queue(i).paddr_v := false.B
    }
  }
  for(i <- 0 until tileLSQueue_sz) {
    when(io.enq.fire && in_ptr.value === i.U) {
      tlState_queue(i) := TLState.s_wait_tlb
    }.elsewhen(io.tlb.resp.fire && tlb_ptr.value === i.U) {
      tlState_queue(i) := TLState.s_wait_a
    }.elsewhen(tlReqDone && tlReq_ptr.value === i.U) {
      tlState_queue(i) := TLState.s_wait_d
    }.elsewhen(tlRespDone && io.memBus.resp.bits.source === i.U) {
      tlState_queue(i) := TLState.s_done
    }.elsewhen(io.deq.fire && out_ptr.value === i.U) {
      tlState_queue(i) := TLState.s_idle
    }
  }
}
