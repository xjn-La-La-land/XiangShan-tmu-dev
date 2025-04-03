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
import xiangshan.cache._


trait TmuParams extends HasXSParameter {
  val numTmm    : Int = 8
  val numTrows  : Int = 16
  val numTcolsb : Int = 64 // bytes per row
  val numTcolsw : Int = numTcolsb / 4 // words per row

  val tile_idx_w : Int = log2Ceil(numTmm)
  val row_idx_w  : Int = log2Ceil(numTrows)
  val row_data_w : Int = numTcolsb * 8
  val row_addr_offset_w = log2Ceil(row_data_w)

  // row walk pointer
  case class RowWalkPtr(max: Int = numTrows) {
    private val ptr = RegInit(0.U(log2Ceil(max + 1).W))
    def value: UInt = ptr(row_idx_w-1, 0)
    def update(): Unit = {
      ptr := Mux(overflow, ptr, ptr + 1.U)
    }
    def overflow: Bool = ptr === (max + 1).U
    def ready_go: Bool = ptr === max.U || overflow // ptr = 16 时，第15行的数据已经读出来，可以拉高 out_valid，在下一个上升沿握手
    def valid:    Bool = !ready_go // ptr = 0~15
    def reset(): Unit = {
      ptr := 0.U
    }
    def walk_past(i: Int): Bool = ptr >= i.U
  }

  // TileLink clinet node params
  val tileLSQueue_sz = 16
  val tmuClientParameters = TLMasterPortParameters.v1(
    clients = Seq(TLMasterParameters.v1(
      "tmu",
      sourceId = IdRange(0, tileLSQueue_sz) // [0, tileLSQueue_sz)
    )),
  )

  val instInfoBuf_sz = 3 // tmu 能容纳的最大指令数量
}


class TmuDataInput(implicit p: Parameters) extends XSBundle with TmuParams {
  val src    = Vec(2, UInt(XLEN.W))
  val imm    = UInt(32.W)
  val func   = FuOpType()
  val robIdx = new RobPtr

  def isTileLS: Bool = TMUOpType.isTileLS(func)
  def isTdp:    Bool = TMUOpType.isTdp(func)
  def isWrite:  Bool = func(1)

  def TdpOp: UInt = func(1, 0)
  def MemOp: UInt = func(1, 0)

  def tmmC: UInt = imm(2, 0)
  def tmmA: UInt = imm(5, 3)
  def tmmB: UInt = imm(8, 6)
  def tmmA_sign:   Bool = func(1) === "b1".U
  def tmmB_sign:   Bool = func(0) === "b1".U

  def vaddr_base: UInt = src(0) + ZeroExt(Cat(imm(31, 3), 0.U(3.W)), VAddrBits) // 目标块在内存中的起始虚地址
  def stride    : UInt = src(1)      // 主轴长度
  def row_vaddr_vec: Vec[UInt] = VecInit( // 每行的虚拟地址(尽量不要这样使用)
    (0 until numTrows).scanLeft(vaddr_base) { (vaddr, _) => vaddr + stride }
  )
}


class TilesReadPort (implicit p: Parameters) extends XSBundle with TmuParams {
  val ren   = Output(Bool())
  val rtile = Output(UInt(tile_idx_w.W))
  val rrow  = Output(UInt(row_idx_w.W))
  val rdata = Input(UInt(row_data_w.W)) // 1 cycle latency

  def toInBundle = {
    val inBundle = Wire(new Bundle {
      val ren   = Bool()
      val rtile = UInt(tile_idx_w.W)
      val rrow  = UInt(row_idx_w.W)
    })
    inBundle.ren   := ren
    inBundle.rtile := rtile
    inBundle.rrow  := rrow
    inBundle
  }
}

class TilesWritePort (implicit p: Parameters) extends XSBundle with TmuParams {
  val wen   = Output(Bool())
  val wtile = Output(UInt(tile_idx_w.W))
  val wrow  = Output(UInt(row_idx_w.W))
  val wdata = Output(UInt(row_data_w.W))
}


// Tmm register (sram implementation)
// sync read, sync write(1 cycle latency)
// no write to read bypass
class TileReg (implicit val p: Parameters) extends Module with TmuParams {
  val io = IO(new Bundle {
    val ren   = Input(Bool())
    val rrow  = Input(UInt(row_idx_w.W))
    val rdata = Output(UInt(row_data_w.W))
    val wrow  = Input(UInt(row_idx_w.W))
    val wen   = Input(Bool())
    val wdata = Input(UInt(row_data_w.W))
  })

  private val tile = Module(new SRAMTemplate(UInt(row_data_w.W), set = numTrows, withClockGate = true))
  tile.io.r.req.valid       := io.ren
  tile.io.r.req.bits.setIdx := io.rrow
  io.rdata                  := tile.io.r.resp.data(0)
  tile.io.w.req.valid       := io.wen
  tile.io.w.req.bits.setIdx := io.wrow
  tile.io.w.req.bits.data   := VecInit(io.wdata)
}


// Tmu memory access bundle
class TmuMemBus (implicit p: Parameters) extends XSBundle with TileLSUnitParams {
  val req = DecoupledIO(new Bundle {
    val source = UInt(sourceIDWidth.W)
    val paddr  = UInt(PAddrBits.W)
    // val wdata  = UInt(l1BusDataWidth.W)
    // val isWrite = Bool()
  })
  val resp = Flipped(DecoupledIO(new Bundle {
    val source = UInt(sourceIDWidth.W)
    val rdata  = UInt(l1BusDataWidth.W)
    // val isWrite = Bool()
  }))


  def ConnectClientNode(node: TLClientNode): Unit = {
    val (bus, edge) = node.out.head
    bus.a.valid := req.valid
    req.ready   := bus.a.ready
    bus.a.bits  := edge.Get(fromSource = req.bits.source, toAddress = req.bits.paddr, lgSize = log2Ceil(l1BusDataWidth/8).U)._2

    resp.valid  := bus.d.valid
    bus.d.ready := resp.ready
    resp.bits.source  := bus.d.bits.source
    resp.bits.rdata   := bus.d.bits.data
  }
}


// use a tiny queue to store robIdx of Xtm instructions
class XtmInstInfo (implicit p: Parameters)extends XSBundle {
  val robIdx   = new RobPtr
  val isTdp    = Bool() // TDP or TILELS
  val ready_go = Bool()
}



// HINT: Tile Matrix Unit
// Tile Matrix Unit is a special functional unit that is used to accelerate matrix operations.
class TmuModule (implicit p: Parameters) extends XSModule with TmuParams with HasCircularQueuePtrHelper {
  val io = IO(new Bundle() {
    // Exu interface
    val in  = Flipped(Decoupled(new TmuDataInput))
    val out = Decoupled(new Bundle{
      val robIdx = new RobPtr
    })
    // Mem interface
    val tlb  = new TlbRequestIO()
    val memBus = new TmuMemBus
    val sbuffer = Vec(EnsbufferWidth, Decoupled(new DCacheWordReqWithVaddrAndPfFlag))
  })

  when(io.in.fire) {
    when(io.in.bits.isTileLS && !io.in.bits.isWrite) {
      printf(p"[TMU] tileloadd tmm${io.in.bits.tmmC}, vaddr = 0x${Hexadecimal(io.in.bits.vaddr_base)}, stride = 0x${Hexadecimal(io.in.bits.stride)}\n")
    }.elsewhen(io.in.bits.isTileLS && io.in.bits.isWrite) {
      printf(p"[TMU] tilestored tmm${io.in.bits.tmmC}, vaddr = 0x${Hexadecimal(io.in.bits.vaddr_base)}, stride = 0x${Hexadecimal(io.in.bits.stride)}\n")
    }.elsewhen(io.in.bits.isTdp) {
      printf(p"[TMU] tdpb??d tmm${io.in.bits.tmmC}, tmm${io.in.bits.tmmA}, tmm${io.in.bits.tmmB}, sign = ${io.in.bits.tmmA_sign}, ${io.in.bits.tmmB_sign}\n")
    }.otherwise {
      printf(p"[TMU] unknown tmu op!\n")
    }
  }


  // tile register file
  val tiles = Seq.fill(numTmm)(Module(new TileReg))
  val tiles_rdatas = VecInit(tiles.map(_.io.rdata))

  // submodules
  val tdpUnit = Module(new TDPUnit)
  val tlsUnit = Module(new TileLSUnit)

  // use a tiny queue to store robIdx of Xtm instructions
  val instInfoBuf = RegInit(VecInit(Seq.fill(instInfoBuf_sz)({
    val info = Wire(new XtmInstInfo)
    info := DontCare
    info.ready_go := false.B
    info
  })))
  class instInfoBufPtr(implicit p: Parameters) extends CircularQueuePtr[instInfoBufPtr](p => instInfoBuf_sz)
  val enq_ptr = RegInit(0.U.asTypeOf(new instInfoBufPtr))
  val deq_ptr = RegInit(0.U.asTypeOf(new instInfoBufPtr))
  val instInfoBufFull = isFull(enq_ptr, deq_ptr)
  val instInfoBufEmpty = isEmpty(enq_ptr, deq_ptr)

  // tdp指令和tilels指令相互阻塞，避免数据冲突
  io.in.ready := !instInfoBufFull &&
                 (io.in.bits.isTdp && tdpUnit.io.tdpin.ready && tlsUnit.io.empty ||
                  io.in.bits.isTileLS && tlsUnit.io.tls_in.ready && tdpUnit.io.empty)
  when(io.in.fire) {
    instInfoBuf(enq_ptr.value).robIdx := io.in.bits.robIdx
    instInfoBuf(enq_ptr.value).isTdp  := io.in.bits.isTdp
    instInfoBuf(enq_ptr.value).ready_go := false.B
    enq_ptr := enq_ptr + 1.U
  }

  when(tdpUnit.io.tdpout.fire || tlsUnit.io.tls_out.fire) {
    instInfoBuf(deq_ptr.value).ready_go := true.B
  }
  io.out.valid       := !instInfoBufEmpty && instInfoBuf(deq_ptr.value).ready_go
  io.out.bits.robIdx := instInfoBuf(deq_ptr.value).robIdx
  when(io.out.fire) {
    deq_ptr := deq_ptr + 1.U
  }

  // connect to tdpUnit
  tdpUnit.io.tdpin.valid := io.in.valid && io.in.bits.isTdp
  tdpUnit.io.tdpin.bits.tmmA  := io.in.bits.tmmA
  tdpUnit.io.tdpin.bits.tmmB  := io.in.bits.tmmB
  tdpUnit.io.tdpin.bits.tmmC  := io.in.bits.tmmC
  tdpUnit.io.tdpin.bits.tdpOp := io.in.bits.TdpOp

  tdpUnit.io.tdpout.ready := instInfoBuf(deq_ptr.value).isTdp && !instInfoBuf(deq_ptr.value).ready_go
  
  // connect to tlsUnit
  tlsUnit.io.tls_in.valid := io.in.valid && io.in.bits.isTileLS
  tlsUnit.io.tls_in.bits.tmm        := io.in.bits.tmmC
  tlsUnit.io.tls_in.bits.vaddr_base := io.in.bits.vaddr_base
  tlsUnit.io.tls_in.bits.stride     := io.in.bits.stride
  tlsUnit.io.tls_in.bits.memOp      := io.in.bits.MemOp

  tlsUnit.io.tls_out.ready := !instInfoBuf(deq_ptr.value).isTdp && !instInfoBuf(deq_ptr.value).ready_go

  val readPorts = Seq(
    tlsUnit.io.tileData.tmmRead,
    tdpUnit.io.tileData.tmmARead,
    tdpUnit.io.tileData.tmmBRead,
    tdpUnit.io.tileData.tmmCRead,
  )
  val writePorts = Seq(
    tlsUnit.io.tileData.tmmWrite,
    tdpUnit.io.tileData.tmmCWrite,
  )

  // connect to tiles
  for (i <- 0 until numTmm) {
    val firstReadPort = PriorityMux(readPorts.map(readPort => {
      (readPort.rtile === i.U && readPort.ren) -> readPort.toInBundle
    }))
    tiles(i).io.ren   := firstReadPort.ren
    tiles(i).io.rrow  := firstReadPort.rrow

    val firstWritePort = PriorityMux(writePorts.map(writePort => {
      (writePort.wtile === i.U && writePort.wen) -> writePort
    }))
    tiles(i).io.wen   := firstWritePort.wen
    tiles(i).io.wrow  := firstWritePort.wrow
    tiles(i).io.wdata := firstWritePort.wdata
  }
  
  readPorts.foreach(readPort => {
    readPort.rdata := tiles_rdatas(readPort.rtile)
  })

  // connect to tlb, memBus, sbuffer
  tlsUnit.io.tlb <> io.tlb
  tlsUnit.io.memBus <> io.memBus
  tlsUnit.io.sbuffer <> io.sbuffer
}


///////////////////////////////////
// TDP computing unit
///////////////////////////////////

trait TDPUnitParams extends TmuParams {
  val numTileBbuf: Int = 3 // number of tileB buffer
  val tileBbuf_ptr_w : Int = log2Ceil(numTileBbuf)

  // TDPUnit stage info stored in registers
  case class TmuStageInfo(in_fire: Bool, out_fire: Bool, in_bits: TDPUnitInput) {
    val valid = RegInit(false.B)
    when(in_fire) {
      valid := true.B
    }.elsewhen(out_fire) {
      valid := false.B
    }

    val regs = RegEnable(in_bits, in_fire)
  }

  object TdpOp {
    val bssd = "b11".U
    val bsud = "b10".U
    val busd = "b01".U
    val buud = "b00".U
    def apply() = UInt(2.W)
  }
}


class TDPUnitInput(implicit p: Parameters) extends XSBundle with TDPUnitParams {
  val tmmA = UInt(tile_idx_w.W)
  val tmmB = UInt(tile_idx_w.W)
  val tmmC = UInt(tile_idx_w.W)
  val tdpOp = TdpOp()
}

class TDPUnitToTiles(implicit val p: Parameters) extends Bundle with TmuParams {
  val tmmARead = new TilesReadPort
  val tmmBRead = new TilesReadPort
  val tmmCRead = new TilesReadPort
  val tmmCWrite = new TilesWritePort
}

class TDPUnit(implicit p: Parameters) extends XSModule with TDPUnitParams {
  val io = IO(new Bundle {
    val tdpin  = Flipped(Decoupled(new TDPUnitInput))
    val tdpout = Decoupled(new Bundle{})
    val tileData = new TDPUnitToTiles
    val empty  = Output(Bool())
  })

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

  // Stage 0: push tmmB into tileB_buf
  val s0_out_valid = Wire(Bool())
  val s1_in_ready  = Wire(Bool())
  val s0_s1_fire   = s0_out_valid && s1_in_ready
  val s0_stall     = Wire(Bool())
  val s0_info      = TmuStageInfo(io.tdpin.fire, s0_s1_fire, io.tdpin.bits)
  val s0_tileB_buf_ptr = RegInit(0.U(tileBbuf_ptr_w.W)) // use tileB_buf(0) or tileB_buf(1) or ...

  when(s0_s1_fire) {
    s0_tileB_buf_ptr := Mux(s0_tileB_buf_ptr === (numTileBbuf-1).U, 0.U, s0_tileB_buf_ptr + 1.U)
  }

  val s0_row_walk_ptr = RowWalkPtr()
  when(s0_s1_fire) {
    s0_row_walk_ptr.reset()
  }.elsewhen(s0_info.valid && !s0_stall) {
    s0_row_walk_ptr.update()
  }
  
  when(s0_info.valid && !s0_row_walk_ptr.overflow) {
    for (r <- 0 until numTrows) {
      if(r == numTrows - 1) { // pump tileB into the last row
        tileB_buf(s0_tileB_buf_ptr)(r) := io.tileData.tmmBRead.rdata
      } else {
        tileB_buf(s0_tileB_buf_ptr)(r) := tileB_buf(s0_tileB_buf_ptr)(r+1)
      }
    }
  }

  s0_out_valid   := s0_info.valid && s0_row_walk_ptr.ready_go
  io.tdpin.ready := s0_s1_fire || !s0_info.valid

  
  // Stage 1: push tmmC into tileC_buf, push tmmA into tileA_buf
  val s1_out_valid = Wire(Bool())
  val s2_in_ready  = Wire(Bool())
  val s1_s2_fire   = s1_out_valid && s2_in_ready
  val s1_stall     = Wire(Bool())
  val s1_info      = TmuStageInfo(s0_s1_fire, s1_s2_fire, s0_info.regs)
  val s1_tileB_buf_ptr = RegEnable(s0_tileB_buf_ptr, s0_s1_fire) // use tileB_buf(0) or tileB_buf(1) or ...

  val s1_row_walk_ptr = RowWalkPtr()
  when(s1_s2_fire) {
    s1_row_walk_ptr.reset()
  }.elsewhen(s1_info.valid && !s1_stall) {
    s1_row_walk_ptr.update()
  }

  s1_out_valid := s1_info.valid && s1_row_walk_ptr.ready_go
  s1_in_ready  := s1_s2_fire || !s1_info.valid


  // Stage 2: pop data from tileC_buf
  val s2_info = TmuStageInfo(s1_s2_fire, io.tdpout.fire, s1_info.regs)
  val s2_tileB_buf_ptr = RegEnable(s1_tileB_buf_ptr, s1_s2_fire) // use tileB_buf(0) or tileB_buf(1) or ...

  val s2_row_walk_ptr = RowWalkPtr()
  when(io.tdpout.fire) {
    s2_row_walk_ptr.reset()
  }.elsewhen(s2_info.valid) {
    s2_row_walk_ptr.update()
  }

  s2_in_ready := io.tdpout.fire || !s2_info.valid
  io.tdpout.valid := s2_row_walk_ptr.ready_go


  // 16 * 16 DPAUnits
  val DPAMatrix = Seq.fill(numTrows)(Seq.fill(numTcolsw)(DPAUnit("int8")))
  for (i <- 0 until numTrows) {
    for (j <- 0 until numTcolsw) {
      val tileB_buf_ptr = Mux(s2_info.valid && s2_row_walk_ptr.walk_past(i), s2_tileB_buf_ptr, s1_tileB_buf_ptr)
      val tdpOp         = Mux(s2_info.valid && s2_row_walk_ptr.walk_past(i), s2_info.regs.tdpOp, s1_info.regs.tdpOp)
      DPAMatrix(i)(j).connect_in(tileA_words(i), tileB_words(tileB_buf_ptr)(i)(j), tileC_words(i)(j), tdpOp)
    }
  }
  val DPAMatrixPop = VecInit((0 until numTcolsw).map(c => DPAMatrix(numTrows-1)(c).out)).asUInt  // data pop from the last line

  val s1_move = s1_info.valid && !s1_row_walk_ptr.overflow
  val s2_move = s2_info.valid && !s2_row_walk_ptr.overflow
  val row_move = s1_move || s2_move

  // tileA_buf and tileC_buf data move
  when(row_move) {
    for (r <- 0 until numTrows) {
      if(r == 0) {
        tileA_buf(r) := io.tileData.tmmARead.rdata
        tileC_buf(r) := io.tileData.tmmCRead.rdata
      } else {
        var w = tileA_buf(r-1).getWidth
        tileA_buf(r) := tileA_buf(r-1)(w-1, 32) // move the remaining data in (r-1)_th row to r_th row
        tileC_buf(r) := VecInit((0 until numTcolsw).map(c => DPAMatrix(r-1)(c).out)).asUInt // move the temp results in (r-1)_th row to r_th row
      }
    }
  }

  // Manage tiles read/write ports
  val s0_ren = VecInit((0 until numTmm).map(i => s0_info.valid && s0_row_walk_ptr.valid && (s0_info.regs.tmmB === i.U)))
  val s1_ren = VecInit((0 until numTmm).map(i => s1_info.valid && s1_row_walk_ptr.valid && (s1_info.regs.tmmA === i.U || s1_info.regs.tmmC === i.U)))
  val s2_wen = VecInit((0 until numTmm).map(i => s2_info.valid && s2_row_walk_ptr.valid && (s2_info.regs.tmmC === i.U)))

  io.tileData.tmmARead.ren    := s1_ren.asUInt.orR
  io.tileData.tmmARead.rtile  := s1_info.regs.tmmA
  io.tileData.tmmARead.rrow   := s1_row_walk_ptr.value
  io.tileData.tmmBRead.ren    := s0_ren.asUInt.orR
  io.tileData.tmmBRead.rtile  := s0_info.regs.tmmB
  io.tileData.tmmBRead.rrow   := s0_row_walk_ptr.value
  io.tileData.tmmCRead.ren    := s1_ren.asUInt.orR
  io.tileData.tmmCRead.rtile  := s1_info.regs.tmmC
  io.tileData.tmmCRead.rrow   := s1_row_walk_ptr.value
  io.tileData.tmmCWrite.wen   := s2_wen.asUInt.orR
  io.tileData.tmmCWrite.wtile := s2_info.regs.tmmC
  io.tileData.tmmCWrite.wrow  := s2_row_walk_ptr.value
  io.tileData.tmmCWrite.wdata := DPAMatrixPop

  s0_stall := s0_ren.zip(s1_ren).map(r => r._1 && r._2).reduce(_ || _) || // s0 and s1 read the same tile
              s0_ren.zip(s2_wen).map(r => r._1 && r._2).reduce(_ || _)    // s0 read and s2 write the same tile
  s1_stall := s1_ren.zip(s2_wen).map(r => r._1 && r._2).reduce(_ || _)    // s1 read and s2 write the same tile

  io.empty := !s0_info.valid && !s1_info.valid && !s2_info.valid
}


///////////////////////////////////
// TMU load/store Unit
///////////////////////////////////
trait TileLSUnitParams extends TmuParams with HasDCacheParameters with HasCircularQueuePtrHelper {
  object MemOp {
    val tldl1 = "b00".U
    val tldl2 = "b01".U
    val tstl1 = "b10".U
    def apply() = UInt(2.W)
    def isLoad(op: UInt): Bool = op === tldl1 || op === tldl2
    def isStore(op: UInt): Bool = op === tstl1
  }

  val sourceIDWidth = log2Ceil(tileLSQueue_sz)

  val l1TldDataWidth = VLEN
  val l1TstDataWidth = EnsbufferWidth * VLEN
  val l2TldDataWidth = l1BusDataWidth
  val l2TldNumBurst = row_data_w / l2TldDataWidth
  val sbufNumEnq    = row_data_w / l1TstDataWidth

  case class DataTransCnt(numBurst: Int) {
    private val cnt = RegInit(0.U(log2Ceil(numBurst).W))
    def value: UInt = cnt
    def update(): Unit = {
      cnt := cnt + 1.U
    }
    def last: Bool = cnt === (numBurst-1).U
  }
  object LSQState {
    val s_idle :: s_tlb :: s_wait_a :: s_wait_d :: s_sbuf :: s_done :: Nil = Enum(6)
    def apply() = UInt(s_idle.getWidth.W)
  }

  class TileLSQPtr(implicit p: Parameters) extends CircularQueuePtr[TileLSQPtr](p => tileLSQueue_sz)
}

class TileLSUnitInput(implicit p: Parameters) extends XSBundle with TileLSUnitParams {
  val tmm        = UInt(tile_idx_w.W)
  val vaddr_base = UInt(VAddrBits.W)
  val stride     = UInt(XLEN.W)
  val memOp      = MemOp()
}

class TileLSUnitToTiles(implicit val p: Parameters) extends Bundle with TileLSUnitParams {
  val tmmRead  = new TilesReadPort
  val tmmWrite = new TilesWritePort
}

class TileLSQEntry(implicit p: Parameters) extends XSBundle with TileLSUnitParams {
  val tmm = UInt(tile_idx_w.W)
  val row = UInt(row_idx_w.W)
  val vaddr = UInt(VAddrBits.W)
  val memOp = MemOp()
  val paddr = UInt(PAddrBits.W)
  val state = LSQState()

  def tlbReqValid:  Bool = state === LSQState.s_tlb
  def l2ReqValid:   Bool = state === LSQState.s_wait_a
  def sbufReqValid: Bool = state === LSQState.s_sbuf
}


// TMU laod/store Unit
class TileLSUnit (implicit p: Parameters) extends XSModule with TileLSUnitParams {
  val io = IO(new Bundle {
    val tls_in  = Flipped(Decoupled(new TileLSUnitInput))
    val tls_out = Decoupled(new Bundle{})
    val tileData = new TileLSUnitToTiles
    val tlb  = new TlbRequestIO()
    // val pmp  = Flipped(new PMPRespBundle()) // pmp check not yet implemented
    val memBus = new TmuMemBus
    val sbuffer = Vec(EnsbufferWidth, Decoupled(new DCacheWordReqWithVaddrAndPfFlag))

    val empty = Output(Bool())
  })
  
  // queues
  val ctrl_queue = Reg(Vec(tileLSQueue_sz, new Bundle{
    val tmm     = UInt(tile_idx_w.W)
    val row     = UInt(row_idx_w.W)
    val memOp   = MemOp()
    val vaddr   = UInt(VAddrBits.W)
  }))
  val paddr_queue = Reg(Vec(tileLSQueue_sz, UInt(PAddrBits.W)))
  val state_queue = RegInit(VecInit.fill(tileLSQueue_sz)(LSQState.s_idle))

  val in_ptr  = RegInit(0.U.asTypeOf(new TileLSQPtr))
  val tlb_ptr = RegInit(0.U.asTypeOf(new TileLSQPtr))
  val mem_ptr = RegInit(0.U.asTypeOf(new TileLSQPtr))
  val out_ptr = RegInit(0.U.asTypeOf(new TileLSQPtr))

  def ToTmuLSQEntry(ptr: TileLSQPtr): TileLSQEntry = {
    val entry = Wire(new TileLSQEntry)
    entry.tmm   := ctrl_queue(ptr.value).tmm
    entry.row   := ctrl_queue(ptr.value).row
    entry.memOp := ctrl_queue(ptr.value).memOp
    entry.vaddr := ctrl_queue(ptr.value).vaddr
    entry.paddr := paddr_queue(ptr.value)
    entry.state := state_queue(ptr.value)
    entry
  }

  

  // tls_in buffer
  val tls_buf = RegEnable(io.tls_in.bits, io.tls_in.fire)
  val tls_buf_valid = RegInit(false.B)
  val lsq_enq_cnt = RegInit(0.U(row_idx_w.W))

  val enq_enable = !isFull(in_ptr, out_ptr) && tls_buf_valid
  when(io.tls_in.fire) {
    tls_buf_valid := true.B
  }.elsewhen(enq_enable && lsq_enq_cnt === (numTrows - 1).U){
    tls_buf_valid := false.B
  }
  
  when(enq_enable) {
    lsq_enq_cnt := lsq_enq_cnt + 1.U
  }
  when(enq_enable) {
    tls_buf.vaddr_base := tls_buf.vaddr_base + tls_buf.stride
  }

  // tls_in buffer -> lsq
  when(enq_enable) {
    in_ptr := in_ptr + 1.U
  }
  for(i <- 0 until tileLSQueue_sz) {
    when(enq_enable && in_ptr.value === i.U) {
      ctrl_queue(i).tmm   := tls_buf.tmm
      ctrl_queue(i).row   := lsq_enq_cnt
      ctrl_queue(i).memOp := tls_buf.memOp
      ctrl_queue(i).vaddr := tls_buf.vaddr_base
    }
  }

  io.tls_in.ready := !tls_buf_valid

  // tlb req and resp
  val tlb_entry = ToTmuLSQEntry(tlb_ptr)
  val tlbReqFlag = RegInit(true.B)
  when(io.tlb.resp.fire && !io.tlb.resp.bits.miss) {
    tlbReqFlag := true.B
  }.elsewhen(io.tlb.req.fire) {
    tlbReqFlag := false.B
  }
  
  io.tlb.req.valid              := tlbReqFlag && tlb_entry.tlbReqValid// blocked tlb
  io.tlb.req.bits.cmd           := Mux(MemOp.isLoad(tlb_entry.memOp), TlbCmd.read, TlbCmd.write)
  io.tlb.req.bits.vaddr         := tlb_entry.vaddr
  io.tlb.req.bits.fullva        := DontCare
  io.tlb.req.bits.checkfullva   := false.B
  io.tlb.req.bits.hyperinst     := false.B
  io.tlb.req.bits.hlvx          := false.B
  io.tlb.req.bits.size          := log2Ceil(numTcolsb).U // 2^size = DataWidth
  io.tlb.req.bits.kill          := false.B
  io.tlb.req.bits.memidx.is_ld  := MemOp.isLoad(tlb_entry.memOp)
  io.tlb.req.bits.memidx.is_st  := MemOp.isStore(tlb_entry.memOp)
  io.tlb.req.bits.memidx.idx    := 0.U
  io.tlb.req.bits.isPrefetch    := false.B
  io.tlb.req.bits.no_translate  := false.B
  io.tlb.req.bits.pmp_addr      := RegEnable(io.tlb.resp.bits.paddr(0), io.tlb.resp.fire) // pmp check not activated in tmu
  io.tlb.req.bits.debug         := DontCare

  io.tlb.req_kill := false.B
  io.tlb.resp.ready := true.B // always ready to receive tlb response
  // tlb_ptr 指针的更新
  val tlbRespDone = io.tlb.resp.fire && !io.tlb.resp.bits.miss
  when(tlbRespDone) {
    tlb_ptr := tlb_ptr + 1.U
  }
  for(i <- 0 until tileLSQueue_sz) {
    when(tlbRespDone && tlb_ptr.value === i.U) {
      paddr_queue(i) := io.tlb.resp.bits.paddr(0)
    }
  }

  
  val mem_entry = ToTmuLSQEntry(mem_ptr)
  
  // TileLink request
  io.memBus.req.valid := mem_entry.l2ReqValid
  io.memBus.req.bits.source  := mem_ptr.value
  io.memBus.req.bits.paddr   := mem_entry.paddr

  // TileLink response
  io.memBus.resp.ready := true.B // LSQueue always ready for response
  val tlResp_entry = (io.memBus.resp.bits.source)
  val tlResp_cnt = DataTransCnt(l2TldNumBurst)
  val l2Rdata_buf = Reg(Vec(l2TldNumBurst-1, UInt(l2TldDataWidth.W)))
  when(io.memBus.resp.fire) { // response for Get
    tlResp_cnt.update()
    when(!tlResp_cnt.last) {
      l2Rdata_buf(tlResp_cnt.value) := io.memBus.resp.bits.rdata
    }
  }
  val l2Rdata = Cat(io.memBus.resp.bits.rdata, l2Rdata_buf.asUInt)
  val tlRespDone = io.memBus.resp.fire && tlResp_cnt.last

  io.tileData.tmmWrite.wtile := ctrl_queue(io.memBus.resp.bits.source).tmm
  io.tileData.tmmWrite.wrow  := ctrl_queue(io.memBus.resp.bits.source).row
  io.tileData.tmmWrite.wdata := l2Rdata
  io.tileData.tmmWrite.wen   := io.memBus.resp.fire && tlResp_cnt.last // write back to tmm when the last beat come

  // SBuffer enq 请求
  io.tileData.tmmRead.rtile := mem_entry.tmm
  io.tileData.tmmRead.rrow  := mem_entry.row
  val l2sCheck = Wire(Bool())
  io.tileData.tmmRead.ren   := mem_entry.sbufReqValid && !l2sCheck // 已经完成 vaddr -> paddr 转换
  val tilesRdataValid = RegInit(false.B)
  
  l2sCheck := state_queue.zip(ctrl_queue).map { case (state, ctrl) =>
    state === LSQState.s_wait_d && MemOp.isLoad(ctrl.memOp) && MemOp.isStore(mem_entry.memOp) &&
    ctrl.tmm === mem_entry.tmm && ctrl.row === mem_entry.row // 未写回的 load 请求
  }.reduce(_ || _) // load to store check!

  val sbufEnq_cnt = Seq.fill(EnsbufferWidth)(DataTransCnt(sbufNumEnq))
  val sbufEnqDoneFlag = Seq.fill(EnsbufferWidth)(RegInit(false.B))
  val addr_offset_vec = Wire(Vec(sbufNumEnq, Vec(EnsbufferWidth, UInt(PAddrBits.W))))
  val tilesRdata_vec  = Wire(Vec(sbufNumEnq, Vec(EnsbufferWidth, UInt(VLEN.W))))
  for (i <- 0 until sbufNumEnq) {
    for (j <- 0 until EnsbufferWidth) {
      var k = i * EnsbufferWidth + j
      addr_offset_vec(i)(j) := Cat(k.U, 0.U(log2Ceil(VLEN/8).W))
      tilesRdata_vec(i)(j)  := io.tileData.tmmRead.rdata((k+1)*VLEN-1, k*VLEN)
    }
  }

  for (i <- 0 until EnsbufferWidth) {
    io.sbuffer(i).valid := mem_entry.sbufReqValid && tilesRdataValid && !sbufEnqDoneFlag(i)
    io.sbuffer(i).bits       := DontCare
    io.sbuffer(i).bits.cmd   := MemoryOpConstants.M_XWR
    io.sbuffer(i).bits.addr  := mem_entry.paddr + addr_offset_vec(sbufEnq_cnt(i).value)(i)
    io.sbuffer(i).bits.vaddr := mem_entry.vaddr + addr_offset_vec(sbufEnq_cnt(i).value)(i)
    io.sbuffer(i).bits.data  := tilesRdata_vec(sbufEnq_cnt(i).value)(i)
    io.sbuffer(i).bits.mask  := Fill(VLEN/8, 1.U(1.W))
    io.sbuffer(i).bits.wline := true.B
    io.sbuffer(i).bits.prefetch  := false.B
    io.sbuffer(i).bits.vecValid  := false.B
    io.sbuffer(i).bits.sqNeedDeq := false.B
  }

  for (i <- 0 until EnsbufferWidth) {
    when(io.sbuffer(i).fire) {
      sbufEnq_cnt(i).update()
    }
  }

  val sbufEnqDone = (0 until EnsbufferWidth).map { i =>
    sbufEnqDoneFlag(i) || io.sbuffer(i).fire && sbufEnq_cnt(i).last
  }.reduce(_ && _)
  for (i <- 0 until EnsbufferWidth) {
    when(sbufEnqDone) {
      sbufEnqDoneFlag(i) := false.B
    }.elsewhen(io.sbuffer(i).fire && sbufEnq_cnt(i).last) {
      sbufEnqDoneFlag(i) := true.B
    }
  }
  when(sbufEnqDone) {
    tilesRdataValid := false.B
  }.elsewhen(io.tileData.tmmRead.ren) {
    tilesRdataValid := true.B // 1 cycle delay
  }

  
  when(io.memBus.req.fire || sbufEnqDone) {
    mem_ptr := mem_ptr + 1.U // mem_ptr 指针更新
  }


  // LSQueue 出队
  val deq_entry = ToTmuLSQEntry(out_ptr)
  val deq_enable = Mux(deq_entry.row === (numTrows-1).U, io.tls_out.fire, deq_entry.state === LSQState.s_done)
  when(deq_enable) {
    out_ptr := out_ptr + 1.U
  }
  io.tls_out.valid := deq_entry.state === LSQState.s_done && deq_entry.row === (numTrows-1).U // last row of tileload/tilestore


  // LSQueue 表项state的更新
  for(i <- 0 until tileLSQueue_sz) {
    switch(state_queue(i)) {
      is(LSQState.s_idle) {
        state_queue(i) := Mux(enq_enable && in_ptr.value === i.U, LSQState.s_tlb, LSQState.s_idle)
      }
      is(LSQState.s_tlb) {
        state_queue(i) := Mux(tlbRespDone && tlb_ptr.value === i.U, 
                          Mux(MemOp.isLoad(ctrl_queue(i).memOp), LSQState.s_wait_a, LSQState.s_sbuf), LSQState.s_tlb)
      }
      is(LSQState.s_wait_a) {
        state_queue(i) := Mux(io.memBus.req.fire && mem_ptr.value === i.U, LSQState.s_wait_d, LSQState.s_wait_a)
      }
      is(LSQState.s_wait_d) {
        state_queue(i) := Mux(tlRespDone && io.memBus.resp.bits.source === i.U, LSQState.s_done, LSQState.s_wait_d)
      }
      is(LSQState.s_sbuf) {
        state_queue(i) := Mux(sbufEnqDone && mem_ptr.value === i.U, LSQState.s_done, LSQState.s_sbuf)
      }
      is(LSQState.s_done) {
        state_queue(i) := Mux(deq_enable && out_ptr.value === i.U, LSQState.s_idle, LSQState.s_done)
      }
    }
  }

  io.empty := isEmpty(in_ptr, out_ptr) && !tls_buf_valid
}

//////////////////////////////////////
// Computing unit implementation
//////////////////////////////////////

// 使用 wallace tree 实现 8bit 乘法
class OneBitAdder extends Module {
  val io = IO(new Bundle {
    val a    = Input(UInt(1.W))
    val b    = Input(UInt(1.W))
    val cin  = Input(UInt(1.W))
    val s    = Output(UInt(1.W))
    val cout = Output(UInt(1.W))
  })
  io.s    := io.a ^ io.b ^ io.cin
  io.cout := (io.a & io.b) | (io.a & io.cin) | (io.b & io.cin)
}

// wallace tree layer
class WallaceTreeLayer(val width: Int) extends Module {
  def soutBitsWidth: Int = (width + 2) / 3 // ceil(w/3)
  def coutBitsWidth: Int = (width + 1) / 3

  val io = IO(new Bundle {
    val inBits   = Input(Vec(width, UInt(1.W)))
    val soutBits = Output(Vec(soutBitsWidth, UInt(1.W)))
    val coutBits = Output(Vec(coutBitsWidth, UInt(1.W)))
  })

  // Get a full adder for every 3 bits
  for (i <- 0 until width/3) {
    val fullAdder = Module(new OneBitAdder)
    fullAdder.io.a   := io.inBits(i*3)
    fullAdder.io.b   := io.inBits(i*3+1)
    fullAdder.io.cin := io.inBits(i*3+2)
    io.soutBits(i)   := fullAdder.io.s
    io.coutBits(i)   := fullAdder.io.cout
  }
  // Handle remaining 1 or 2 bits if width is not a multiple of 3
  if (width%3 == 1) {
    io.soutBits.last := io.inBits.last
  } else if (width%3 == 2) {
    val halfAdder = Module(new OneBitAdder)
    halfAdder.io.a   := io.inBits(width-2)
    halfAdder.io.b   := io.inBits(width-1)
    halfAdder.io.cin := 0.U
    io.soutBits.last := halfAdder.io.s
    io.coutBits.last := halfAdder.io.cout
  }
}

// Wallace Tree (Recursive Compression)
class WallaceTree(val width: Int) extends Module {
  def sin_width: Int = width
  def cin_width: Int = {
    var w = width
    var nAddr = 0
    while(w != 2) {
      nAddr += (w + 1) / 3          // coutBitsWidth
      w = (w + 2) / 3 + (w + 1) / 3 // soutBitsWidth + coutBitsWidth
    }
    nAddr - 1
  }
  def cout_width: Int = cin_width

  val io = IO(new Bundle {
    val sin  = Input(Vec(sin_width, UInt(1.W)))
    val cin  = Input(Vec(cin_width, UInt(1.W)))
    val S    = Output(UInt(1.W))
    val C    = Output(UInt(1.W))
    val cout = Output(Vec(cout_width, UInt(1.W)))
  })

  var currentWidth: Int          = width
  var currentInBits: Vec[UInt]   = io.sin
  var currentSoutBits: Vec[UInt] = null
  var currentCoutBits: Vec[UInt] = null
  var i = 0
  // add layer to compress width, until currentWidth = 2
  while(currentWidth > 2) {
    val layer = Module(new WallaceTreeLayer(currentWidth))
    layer.io.inBits := currentInBits
    currentSoutBits = layer.io.soutBits
    currentCoutBits = layer.io.coutBits
    currentWidth = layer.soutBitsWidth + layer.coutBitsWidth // soutBits + coutBits(from the previous tree cout)
    currentInBits = VecInit(currentSoutBits ++ io.cin.slice(i, i + layer.coutBitsWidth))
    currentCoutBits.zipWithIndex.foreach { case (c, j) =>
      if (i+j < cout_width)
        io.cout(i+j) := c
    }
    i += layer.coutBitsWidth
  }

  // now currentWidth = 2
  io.S := currentSoutBits.head
  io.C := currentCoutBits.head

}


// Wallace Tree Multiplier
class WTMulUnit(val width: Int) extends Module {
  val io = IO(new Bundle {
    val a = Input(UInt(width.W))
    val b = Input(UInt(width.W))
    val c = Output(UInt((2*width).W))
  })

  val (a, b) = (io.a, io.b)

  // TODO: generate partial product
  def num_pp: Int = (width+1)/2
  val pp_s = Wire(Vec(num_pp, UInt((2*width).W))) // 部分积
  val pp_c = Wire(Vec(num_pp, UInt(1.W)))         // 每个部分积对应一个进位
  for (i <- Range(0, width, 2)) {
    val a_flags = if(i == 0) Cat(a(1, 0), 0.U(1.W)) else if(i == width-1) SignExt(a(i, i-1), 3) else a(i+1, i-1)
    val b_shift = SignExt(b, 2*width) << i
    
    pp_s(i/2) := Mux1H(Seq(
      (a_flags === "b001".U || a_flags === "b010".U, b_shift),  // +X
      (a_flags === "b101".U || a_flags === "b110".U, ~b_shift), // -X
      (a_flags === "b011".U,                         (b_shift << 1.U)), // +2X
      (a_flags === "b100".U,                         ~(b_shift << 1.U)) // -2X
    ))
    pp_c(i/2) := a_flags === "b101".U || a_flags === "b110".U || a_flags === "b100".U // -X or -2X(补码取复数需要取反加一)
  }

  // TODO: connect to wallace trees
  var wtree_cout_last: Vec[UInt] = null
  val wtree_S = Wire(Vec(2*width, UInt(1.W)))
  val wtree_C = Wire(Vec(2*width, UInt(1.W)))
  for (i <- 0 until (2*width)) {
    val wtree = Module(new WallaceTree(num_pp))
    wtree.io.sin := VecInit(pp_s.map{ case e => e(i)})
    if(i == 0) {
      wtree.io.cin := VecInit(pp_c.take(wtree.cin_width))
      require(wtree.cin_width + 2 == num_pp)
    }else {
      wtree.io.cin := wtree_cout_last
    }
    wtree_S(i) := wtree.io.S
    wtree_C(i) := wtree.io.C
    wtree_cout_last = wtree.io.cout
  }

  val S = wtree_S.asUInt
  val C = Cat(wtree_C.asUInt(2*width-2, 0), pp_c(num_pp-2)) // 最高位的进位舍弃

  io.c := S + C + pp_c(num_pp-1)

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

  // val c_vec = io.a_vec.zip(io.b_vec).map { case(a, b) =>
  //   val muli8i8i32 = Module(new WTMulUnit(8+1))
  //   muli8i8i32.io.a := Mux(a_sign, SignExt(a, 9), ZeroExt(a, 9))
  //   muli8i8i32.io.b := Mux(b_sign, SignExt(b, 9), ZeroExt(b, 9))
  //   muli8i8i32.io.c
  // }

  // val dp = ParallelSingedExpandingAdd(c_vec.map(_.asSInt))
  // io.c_out := io.c_in + SignExt(dp.asUInt, 32)

  // the following dummy implementation is for faster compilation
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