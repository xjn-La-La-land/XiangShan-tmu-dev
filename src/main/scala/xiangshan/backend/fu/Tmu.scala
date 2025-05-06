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

  val num_tilerf_readPort  = 4
  val num_tilerf_writePort = 3

  // TileLink clinet node params
  val tileLSQ_sz = 32
  val tmuClientParameters = TLMasterPortParameters.v1(
    clients = Seq(TLMasterParameters.v1(
      "tmu",
      sourceId = IdRange(0, tileLSQ_sz) // [0, tileLSQ_sz)
    )),
  )

  val tdpInstBuf_sz = 4 // tdpUnit 能容纳的最大指令数量+1
  val tlsInstBuf_sz = 3 // tlsQueue 能容纳的最大指令数量+1
  class InstBufPtr (size: Int)(implicit p: Parameters) extends CircularQueuePtr[InstBufPtr](p => size)

  val numL2CReadPort = 2 // l2Cache read做成双端口,将数据吞吐提高到64B

  case class InstBufTemplate (size: Int) extends HasCircularQueuePtrHelper {
    val info  = Reg(Vec(size, new TmuDataInput))
    val valid = RegInit(VecInit(Seq.fill(size)(false.B)))

    val enq_ptr = RegInit(0.U.asTypeOf(new InstBufPtr(size)))
    val rdy_ptr = RegInit(0.U.asTypeOf(new InstBufPtr(size)))
    val deq_ptr = RegInit(0.U.asTypeOf(new InstBufPtr(size)))

    def isFull:  Bool = isFull(enq_ptr, deq_ptr)
    def isEmpty: Bool = isEmpty(enq_ptr, deq_ptr)

    def enq(inst: TmuDataInput): Unit = {
      info(enq_ptr.value) := inst
      valid(enq_ptr.value) := true.B
      enq_ptr := enq_ptr + 1.U
    }
    def deqData  = info(deq_ptr.value)
    def deqReady = !isEmpty && !valid(deq_ptr.value)
    def deq(): Unit = {
      deq_ptr := deq_ptr + 1.U
    }
    def setReady(): Unit = {
      valid(rdy_ptr.value) := false.B
      rdy_ptr := rdy_ptr + 1.U
    }
  }
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

  def base_vaddr: UInt = src(0) // 目标块在内存中的起始虚地址
  def stride    : UInt = src(1) // 主轴长度
  def row_vaddr_vec: Vec[UInt] = VecInit( // 每行的虚拟地址(do not use for better timing)
    (0 until numTrows).scanLeft(base_vaddr) { (vaddr, _) => vaddr + stride }
  )
}


class TilesReadPort (implicit p: Parameters) extends XSBundle with TmuParams {
  val ren   = Output(Bool())
  val rtile = Output(UInt(tile_idx_w.W))
  val rrow  = Output(UInt(row_idx_w.W))
  val rdata = Input(UInt(row_data_w.W)) // 1 cycle latency
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

class TileRegFile (implicit val p: Parameters) extends Module with TmuParams {
  val io = IO(new Bundle {
    val readPorts  = Flipped(Vec(num_tilerf_readPort, new TilesReadPort))
    val writePorts = Flipped(Vec(num_tilerf_writePort, new TilesWritePort))
  })

  val tiles = Seq.fill(numTmm)(Module(new TileReg))
  val tiles_rdatas = VecInit(tiles.map(_.io.rdata))

  // connect to tiles
  for (i <- 0 until numTmm) {
    val ren_vec = io.readPorts.map(readPort => {
      readPort.rtile === i.U && readPort.ren
    })
    tiles(i).io.ren   := ParallelOR(ren_vec)
    tiles(i).io.rrow  := ParallelPriorityMux(ren_vec, io.readPorts.map(_.rrow))

    val wen_vec = io.writePorts.map(writePort => {
      writePort.wtile === i.U && writePort.wen
    })
    tiles(i).io.wen   := ParallelOR(wen_vec)
    tiles(i).io.wrow  := ParallelPriorityMux(wen_vec, io.writePorts.map(_.wrow))
    tiles(i).io.wdata := ParallelPriorityMux(wen_vec, io.writePorts.map(_.wdata))
  }

  io.readPorts.foreach(readPort => {
    val rtile_buf = RegEnable(readPort.rtile, readPort.ren)
    readPort.rdata := tiles_rdatas(rtile_buf) // hold read data until next ren!
  })
}


// use a tiny queue to store robIdx of Xtm instructions
class TmuInstBuf (implicit p: Parameters) extends XSModule with TmuParams with HasCircularQueuePtrHelper {
  val io = IO(new Bundle {
    val inst_in  = Flipped(Decoupled(new TmuDataInput))
    val inst_out = Decoupled(new TmuDataInput)
    val tdpInIBufPtr = new InstBufPtr(tdpInstBuf_sz) // new instBuf ptr for tdp
    val tdpStageCtrl = Flipped(new TDPUnitStageCtrl)
    val tdp_done = Input(Bool()) // tdp 指令执行完成信号
    val tls_done = Input(Bool()) // tls 指令执行完成信号
  })

  // tdp指令和tls指令分开存放，便于数据冲突判断
  val tdpInstBuf = InstBufTemplate(tdpInstBuf_sz)
  val tlsInstBuf = InstBufTemplate(tlsInstBuf_sz)
  io.tdpInIBufPtr := tdpInstBuf.enq_ptr

  val tdpS0Inst = tdpInstBuf.info(io.tdpStageCtrl.stageIBufPtr(0).value)
  val tdpS1Inst = tdpInstBuf.info(io.tdpStageCtrl.stageIBufPtr(1).value)
  val tdpS2Inst = tdpInstBuf.info(io.tdpStageCtrl.stageIBufPtr(2).value)

  // tdp 指令的阻塞信号
  val tdpS0_stall_in = ParallelOR(tlsInstBuf.info zip tlsInstBuf.valid map { case (info, valid) =>
    val raw = io.inst_in.bits.tmmB === info.tmmC && !info.isWrite
    raw && valid
  })
  val tdpS1_stall_in = ParallelOR(tlsInstBuf.info zip tlsInstBuf.valid map { case (info, valid) =>
    val s0_tmmA = tdpS0Inst.tmmA
    val s0_tmmC = tdpS0Inst.tmmC
    val s0_robIdx = tdpS0Inst.robIdx
    val raw = (s0_tmmA === info.tmmC || s0_tmmC === info.tmmC) && !info.isWrite && isAfter(s0_robIdx, info.robIdx)
    raw && valid
  })
  val tdpS2_stall_in = ParallelOR(tlsInstBuf.info zip tlsInstBuf.valid map { case (info, valid) =>
    val s1_tmmC = tdpS1Inst.tmmC
    val s1_robIdx = tdpS1Inst.robIdx
    val waw = (s1_tmmC === info.tmmC) && !info.isWrite && isAfter(s1_robIdx, info.robIdx)
    val war = (s1_tmmC === info.tmmC) && info.isWrite && isAfter(s1_robIdx, info.robIdx)
    (waw || war) && valid
  })
  // tls 指令的阻塞信号
  val s0_collision = Seq(tdpS0Inst.tmmA, tdpS0Inst.tmmB, tdpS0Inst.tmmC).map(_ === io.inst_in.bits.tmmC).reduce(_ || _) &&
                     io.tdpStageCtrl.stageValid(0)
  val s1_collision = Seq(tdpS1Inst.tmmA, tdpS1Inst.tmmC).map(_ === io.inst_in.bits.tmmC).reduce(_ || _) &&
                     io.tdpStageCtrl.stageValid(1)
  val s2_collision = Seq(tdpS2Inst.tmmC).map(_ === io.inst_in.bits.tmmC).reduce(_ || _) &&
                     io.tdpStageCtrl.stageValid(2)

  val tls_stall_in = s0_collision || s1_collision || s2_collision

  val stall_in = Wire(Bool())
  val canAccept = Wire(Bool())
  stall_in := Mux(io.inst_in.bits.isTileLS, tls_stall_in, false.B)
  canAccept := Mux(io.inst_in.bits.isTdp, !tdpInstBuf.isFull, !tlsInstBuf.isFull)
  dontTouch(stall_in)
  dontTouch(canAccept)

  io.inst_in.ready := canAccept && !stall_in
  io.tdpStageCtrl.stageAllowIn(0) := !tdpS0_stall_in
  io.tdpStageCtrl.stageAllowIn(1) := !tdpS1_stall_in
  io.tdpStageCtrl.stageAllowIn(2) := !tdpS2_stall_in
  when(io.inst_in.fire) {
    when(io.inst_in.bits.isTdp) {
      tdpInstBuf.enq(io.inst_in.bits)
    }.otherwise {
      tlsInstBuf.enq(io.inst_in.bits)
    }
  }

  when(io.tdp_done) { tdpInstBuf.setReady() }
  when(io.tls_done) { tlsInstBuf.setReady() }

  // 选择两个队列出队列的表项中更老的指令
  val sel_tdp = tdpInstBuf.deqReady && (!tlsInstBuf.deqReady || isBefore(tdpInstBuf.deqData.robIdx, tlsInstBuf.deqData.robIdx))
  io.inst_out.valid := Mux(sel_tdp, tdpInstBuf.deqReady, tlsInstBuf.deqReady)
  io.inst_out.bits  := Mux(sel_tdp, tdpInstBuf.deqData, tlsInstBuf.deqData)
  when(io.inst_out.fire) {
    when(sel_tdp) {
      tdpInstBuf.deq()
    }.otherwise {
      tlsInstBuf.deq()
    }
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
    val tlb     = new TlbRequestIO()
    val memBus  = Vec(numL2CReadPort, new TmuMemBus) // l2-cache load mem bus
    val sbuffer = Decoupled(new DCacheLineReq) // l1-dcache store(write to sbuffer)
  })

  // for debug
  when(io.in.fire) {
    when(io.in.bits.isTileLS && !io.in.bits.isWrite) {
      printf(p"[TMU] tileloadd tmm${io.in.bits.tmmC}, vaddr = 0x${Hexadecimal(io.in.bits.base_vaddr)}, stride = 0x${Hexadecimal(io.in.bits.stride)}\n")
    }.elsewhen(io.in.bits.isTileLS && io.in.bits.isWrite) {
      printf(p"[TMU] tilestored tmm${io.in.bits.tmmC}, vaddr = 0x${Hexadecimal(io.in.bits.base_vaddr)}, stride = 0x${Hexadecimal(io.in.bits.stride)}\n")
    }.elsewhen(io.in.bits.isTdp) {
      printf(p"[TMU] tdpb??d tmm${io.in.bits.tmmC}, tmm${io.in.bits.tmmA}, tmm${io.in.bits.tmmB}, sign = ${io.in.bits.tmmA_sign}, ${io.in.bits.tmmB_sign}\n")
    }.otherwise {
      printf(p"[TMU] unknown tmu op!\n")
    }
  }


  // tile register file
  val tiles = Module(new TileRegFile)

  // submodules
  val tdpUnit = Module(new TDPUnit)
  val tlsQueue = Module(new TileLSQueue)

  // use a tiny queue to store info of Xtm instructions
  val instBuf = Module(new TmuInstBuf)
  instBuf.io.inst_in.bits   := io.in.bits
  instBuf.io.inst_in.valid  := io.in.fire
  instBuf.io.tdpStageCtrl   <> tdpUnit.io.stageCtrl
  instBuf.io.tdp_done       := tdpUnit.io.done
  instBuf.io.tls_done       := tlsQueue.io.done
  instBuf.io.inst_out.ready := io.out.ready
  
  io.in.ready := instBuf.io.inst_in.ready &&
                (io.in.bits.isTdp && tdpUnit.io.tdp_in.ready || io.in.bits.isTileLS && tlsQueue.io.tls_in.ready)
  io.out.valid := instBuf.io.inst_out.valid
  io.out.bits.robIdx := instBuf.io.inst_out.bits.robIdx

  // connect to tdpUnit
  tdpUnit.io.tdp_in.valid := io.in.fire && io.in.bits.isTdp
  tdpUnit.io.tdp_in.bits.tmmA  := io.in.bits.tmmA
  tdpUnit.io.tdp_in.bits.tmmB  := io.in.bits.tmmB
  tdpUnit.io.tdp_in.bits.tmmC  := io.in.bits.tmmC
  tdpUnit.io.tdp_in.bits.tdpOp := io.in.bits.TdpOp
  tdpUnit.io.tdp_in.bits.instBufPtr := instBuf.io.tdpInIBufPtr
  
  // connect to tlsQueue
  tlsQueue.io.tls_in.valid := io.in.fire && io.in.bits.isTileLS
  tlsQueue.io.tls_in.bits.tmm        := io.in.bits.tmmC
  tlsQueue.io.tls_in.bits.base_vaddr := io.in.bits.base_vaddr
  tlsQueue.io.tls_in.bits.stride     := io.in.bits.stride
  tlsQueue.io.tls_in.bits.memOp      := io.in.bits.MemOp

  // connect to tiles
  val readPorts = Seq(
    tlsQueue.io.tileData.tmmRead,
    tdpUnit.io.tileData.tmmCRead,
    tdpUnit.io.tileData.tmmARead,
    tdpUnit.io.tileData.tmmBRead,
  )
  val writePorts = Seq(
    tlsQueue.io.tileData.tmmWrite(0),
    tlsQueue.io.tileData.tmmWrite(1),
    tdpUnit.io.tileData.tmmCWrite,
  )

  tiles.io.readPorts  <> readPorts
  tiles.io.writePorts <> writePorts

  // connect to tlb, memBus, sbuffer
  tlsQueue.io.tlb <> io.tlb
  tlsQueue.io.memBus <> io.memBus
  tlsQueue.io.sbuffer <> io.sbuffer
}


///////////////////////////////////
// TDP computing unit
///////////////////////////////////

trait TDPUnitParams extends TmuParams {
  val numTileBbuf: Int = 3 // number of tileB buffer
  val tileBbuf_ptr_w : Int = log2Ceil(numTileBbuf)

  // TDPUnit stage info stored in registers
  case class TmuStageInfo(in_fire: Bool, out_fire: Bool, in_bits: TDPUnitInput) {
    val valid = ValidHold(in_fire, out_fire)
    val regs  = RegEnable(in_bits, in_fire)
  }

  object TdpOp {
    val bssd = "b11".U
    val bsud = "b10".U
    val busd = "b01".U
    val buud = "b00".U
    def apply() = UInt(2.W)
  }

  // tile row walk pointer
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
    def walk_past(i: Int): Bool = value >= i.U
  }
}

class TDPUnitInput(implicit p: Parameters) extends XSBundle with TDPUnitParams {
  val tmmA = UInt(tile_idx_w.W)
  val tmmB = UInt(tile_idx_w.W)
  val tmmC = UInt(tile_idx_w.W)
  val tdpOp = TdpOp()
  val instBufPtr = new InstBufPtr(tdpInstBuf_sz) // tdpUnit instBuf pointer
}

class TDPUnitToTiles(implicit val p: Parameters) extends Bundle with TmuParams {
  val tmmARead = new TilesReadPort
  val tmmBRead = new TilesReadPort
  val tmmCRead = new TilesReadPort
  val tmmCWrite = new TilesWritePort
}

class TDPUnitStageCtrl(implicit val p: Parameters) extends Bundle with TDPUnitParams {
  val stageIBufPtr = Vec(3, new InstBufPtr(tdpInstBuf_sz)) // stage instBuf pointer
  val stageValid   = Vec(3, Bool()) // stage valid signal
  val stageAllowIn = Flipped(Vec(3, Bool())) // stage allowIn signal
}

class TDPUnit(implicit p: Parameters) extends XSModule with TDPUnitParams {
  val io = IO(new Bundle {
    val tdp_in    = Flipped(Decoupled(new TDPUnitInput))
    val done      = Output(Bool())
    val stageCtrl = new TDPUnitStageCtrl
    val tileData  = new TDPUnitToTiles
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
  val s0_info      = TmuStageInfo(io.tdp_in.fire, s0_s1_fire, io.tdp_in.bits)
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
  io.tdp_in.ready := (s0_s1_fire || !s0_info.valid) && io.stageCtrl.stageAllowIn(0)

  
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
  s1_in_ready  := (s1_s2_fire || !s1_info.valid) && io.stageCtrl.stageAllowIn(1)


  // Stage 2: pop data from tileC_buf
  val s2_info = TmuStageInfo(s1_s2_fire, io.done, s1_info.regs)
  val s2_tileB_buf_ptr = RegEnable(s1_tileB_buf_ptr, s1_s2_fire) // use tileB_buf(0) or tileB_buf(1) or ...

  val s2_row_walk_ptr = RowWalkPtr()
  when(io.done) {
    s2_row_walk_ptr.reset()
  }.elsewhen(s2_info.valid) {
    s2_row_walk_ptr.update()
  }

  s2_in_ready := (io.done || !s2_info.valid) && io.stageCtrl.stageAllowIn(2)
  io.done     := s2_row_walk_ptr.ready_go


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

  io.stageCtrl.stageIBufPtr(0) := s0_info.regs.instBufPtr
  io.stageCtrl.stageIBufPtr(1) := s1_info.regs.instBufPtr
  io.stageCtrl.stageIBufPtr(2) := s2_info.regs.instBufPtr
  io.stageCtrl.stageValid(0)   := s0_info.valid
  io.stageCtrl.stageValid(1)   := s1_info.valid
  io.stageCtrl.stageValid(2)   := s2_info.valid
}


///////////////////////////////////
// TMU load/store Unit
///////////////////////////////////
trait TileLSQParams extends TmuParams with HasDCacheParameters {
  object MemOp {
    val tldl1 = "b00".U
    val tldl2 = "b01".U
    val tstl1 = "b10".U
    def apply() = UInt(2.W)
    def isLoad(op: UInt): Bool = op === tldl1 || op === tldl2
    def isStore(op: UInt): Bool = op === tstl1
  }

  val sourceIDWidth = log2Ceil(tileLSQ_sz)

  val l1TldDataWidth = VLEN
  val l1TstDataWidth = CacheLineSize   // 64B
  val l2TldDataWidth = l1BusDataWidth  // 32B
  val l2TldNumBurst  = row_data_w / l2TldDataWidth

  case class DataTransCnt(numBurst: Int) {
    private val cnt = RegInit(0.U(log2Ceil(numBurst).W))
    def value: UInt = cnt
    def update(): Unit = {
      cnt := cnt + 1.U
    }
    def last: Bool = cnt === (numBurst-1).U
  }
  class LSQState(implicit p: Parameters) extends XSBundle {
    val valid = Bool()
    val done  = Bool()
    val tlb_done = Bool()
    val req_done = Bool()
    def reset(): Unit = {
      valid    := false.B
      done     := false.B
      tlb_done := false.B
      req_done := false.B
    }
    def wait_tlb:  Bool = valid && !tlb_done
    def wait_req:  Bool = valid && tlb_done && !req_done
    def wait_resp: Bool = valid && req_done && !done
  }
  object LSQState {
    def apply() = {
      val state = Wire(new LSQState)
      state.reset()
      state
    }
  }

  class TileLSQPtr(implicit p: Parameters) extends CircularQueuePtr[TileLSQPtr](p => tileLSQ_sz)
  object TileLSQPtr {
    def apply(f: Bool, v: UInt): TileLSQPtr = {
      val ptr = Wire(new TileLSQPtr)
      ptr.flag  := f
      ptr.value := v
      ptr
    }
  }
}

class TileLSQInput(implicit p: Parameters) extends XSBundle with TileLSQParams {
  val tmm        = UInt(tile_idx_w.W)
  val base_vaddr = UInt(VAddrBits.W)
  val stride     = UInt(XLEN.W)
  val memOp      = MemOp()
}

class TileLSQEntry(implicit p: Parameters) extends XSBundle with TileLSQParams {
  val id  = UInt(sourceIDWidth.W)
  val tmm = UInt(tile_idx_w.W)
  val row = UInt(row_idx_w.W)
  val vaddr = UInt(VAddrBits.W)
  val memOp = MemOp()
  val paddr = UInt(PAddrBits.W)
  val state = new LSQState

  def tlbReqValid: Bool    = state.wait_tlb
  def l2CReqValid: Bool    = state.wait_req && MemOp.isLoad(memOp)
  def sbufWriteValid: Bool = state.wait_req && MemOp.isStore(memOp)
}

class TileLSUnitToTiles(implicit val p: Parameters) extends Bundle with TileLSQParams {
  val tmmRead  = new TilesReadPort
  val tmmWrite = Vec(numL2CReadPort, new TilesWritePort)
}

// Tmu l2-cache load mem bus
class TmuMemBus (implicit p: Parameters) extends XSBundle with TileLSQParams {
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


// TMU load/store Queue
class TileLSQueue (implicit p: Parameters) extends XSModule with TileLSQParams with HasCircularQueuePtrHelper {
  val io = IO(new Bundle {
    val tls_in  = Flipped(Decoupled(new TileLSQInput))
    val done    = Output(Bool())
    val tileData = new TileLSUnitToTiles
    val tlb  = new TlbRequestIO()
    // val pmp  = Flipped(new PMPRespBundle()) // pmp check not yet implemented
    val memBus  = Vec(numL2CReadPort, new TmuMemBus) // l2-cache load
    val sbuffer = Decoupled(new DCacheLineReq)       // l1-dcache store(write to sbuffer)
    // val dcache = Flipped(new DCacheToSbufferIO)
  })
  
  // queues
  val ctrl_queue = Reg(Vec(tileLSQ_sz, new Bundle{
    val tmm   = UInt(tile_idx_w.W)
    val row   = UInt(row_idx_w.W)
    val memOp = MemOp()
  }))
  val state_queue = RegInit(VecInit.fill(tileLSQ_sz)(LSQState()))
  val vaddr_queue = Reg(Vec(tileLSQ_sz, UInt(VAddrBits.W)))
  val paddr_queue = Reg(Vec(tileLSQ_sz, UInt(PAddrBits.W)))

  val in_ptr     = RegInit(0.U.asTypeOf(new TileLSQPtr)) // 入队指针
  val tlb_ptr    = RegInit(0.U.asTypeOf(new TileLSQPtr)) // 地址转换指针
  val l2CReq_ptr = RegInit(VecInit.tabulate(numL2CReadPort)(i => TileLSQPtr(false.B, i.U))) // l2-cache load req ptr
  val sbufW_ptr  = RegInit(0.U.asTypeOf(new TileLSQPtr)) // sbuffer write ptr
  val out_ptr    = RegInit(0.U.asTypeOf(new TileLSQPtr)) // 出队指针

  def ToTmuLSQEntry(ptr: TileLSQPtr): TileLSQEntry = {
    val entry = Wire(new TileLSQEntry)
    entry.id    := ptr.value
    entry.tmm   := ctrl_queue(ptr.value).tmm
    entry.row   := ctrl_queue(ptr.value).row
    entry.memOp := ctrl_queue(ptr.value).memOp
    entry.vaddr := vaddr_queue(ptr.value)
    entry.paddr := paddr_queue(ptr.value)
    entry.state := state_queue(ptr.value)
    entry
  }
  def ToTmuLSQEntry(id: UInt): TileLSQEntry = {
    val entry = Wire(new TileLSQEntry)
    entry.id    := id
    entry.tmm   := ctrl_queue(id).tmm
    entry.row   := ctrl_queue(id).row
    entry.memOp := ctrl_queue(id).memOp
    entry.vaddr := vaddr_queue(id)
    entry.paddr := paddr_queue(id)
    entry.state := state_queue(id)
    entry
  }

  // tls_in buffer
  val lsq_enq_cnt = RegInit(0.U(row_idx_w.W))
  val enq_enable  = Wire(Bool())
  val tls_buf       = RegEnable(io.tls_in.bits, io.tls_in.fire)
  val tls_buf_valid = ValidHold(io.tls_in.fire, enq_enable && (lsq_enq_cnt === (numTrows - 1).U))

  enq_enable := !isFull(in_ptr, out_ptr) && tls_buf_valid
  when(enq_enable) {
    lsq_enq_cnt := lsq_enq_cnt + 1.U
    tls_buf.base_vaddr := tls_buf.base_vaddr + tls_buf.stride
  }

  // tls_in buffer -> lsq
  when(enq_enable) {
    in_ptr := in_ptr + 1.U
  }
  for(i <- 0 until tileLSQ_sz) {
    when(enq_enable && in_ptr.value === i.U) {
      ctrl_queue(i).tmm   := tls_buf.tmm
      ctrl_queue(i).row   := lsq_enq_cnt
      ctrl_queue(i).memOp := tls_buf.memOp
      vaddr_queue(i)      := tls_buf.base_vaddr
    }
  }

  io.tls_in.ready := !tls_buf_valid

  // tlb request
  val tlbHit = io.tlb.resp.valid && !io.tlb.resp.bits.miss
  val tlbMiss = io.tlb.resp.valid && io.tlb.resp.bits.miss
  val tlb_req_ptr  = Mux(tlbMiss, tlb_ptr - 1.U, tlb_ptr)
  val tlb_resp_ptr = tlb_ptr - 1.U
  val tlb_req_entry = ToTmuLSQEntry(tlb_req_ptr) // blocked tlb access for tmu

  io.tlb.req.valid := tlb_req_entry.tlbReqValid
  io.tlb.req.bits  := DontCare
  io.tlb.req.bits.cmd           := Mux(MemOp.isLoad(tlb_req_entry.memOp), TlbCmd.read, TlbCmd.write)
  io.tlb.req.bits.vaddr         := tlb_req_entry.vaddr
  io.tlb.req.bits.size          := log2Ceil(numTcolsb).U // 2^size = DataWidth
  io.tlb.req.bits.checkfullva   := false.B
  io.tlb.req.bits.hyperinst     := false.B
  io.tlb.req.bits.hlvx          := false.B
  io.tlb.req.bits.kill          := false.B
  io.tlb.req.bits.isPrefetch    := false.B
  io.tlb.req.bits.no_translate  := false.B
  io.tlb.req_kill               := false.B

  io.tlb.resp.ready := true.B // always ready to receive tlb response
  when(io.tlb.req.fire && !tlbMiss) {
    tlb_ptr := tlb_ptr + 1.U // tlb_ptr 指针的更新
  }

  for(i <- 0 until tileLSQ_sz) {
    when(tlbHit && tlb_resp_ptr.value === i.U) {
      paddr_queue(i) := io.tlb.resp.bits.paddr(0)
    }
  }

  // l2-cache load req
  val l2CReq_entry = (0 until numL2CReadPort).map(i => ToTmuLSQEntry(l2CReq_ptr(i)))
  val ldu = Module(new TmuL2CLoadUnit)
  ldu.io.memBus   <> io.memBus
  ldu.io.tmmWrite <> io.tileData.tmmWrite

  val step = numL2CReadPort
  for(i <- 0 until numL2CReadPort) {
    ldu.io.tlReqEntry(i).valid := l2CReq_entry(i).l2CReqValid
    ldu.io.tlReqEntry(i).bits  := l2CReq_entry(i)
  }

  for(i <- 0 until numL2CReadPort) {
    when((ldu.io.tlReqEntry(i).fire || !l2CReq_entry(i).l2CReqValid) &&
         (l2CReq_ptr(i) + step.U <= tlb_resp_ptr)) {
      l2CReq_ptr(i) := l2CReq_ptr(i) + step.U
    }
  }

  // l2-cache load resp
  for(i <- 0 until numL2CReadPort) {
    ldu.io.tlRespEntry(i) := ToTmuLSQEntry(ldu.io.tlRespId(i))
  }

  // l1-dcache write(to sbuffer)
  val sbufW_entry = ToTmuLSQEntry(sbufW_ptr)
  val l2sCheck = Wire(Bool())
  l2sCheck := ParallelOR(state_queue.zip(ctrl_queue).map { case (state, ctrl) =>
    state.wait_resp && MemOp.isLoad(ctrl.memOp) && 
    ctrl.tmm === sbufW_entry.tmm && ctrl.row === sbufW_entry.row // 未完成的 load 请求
  }) // load to store check!

  val stu = Module(new TmuStoreUnit)
  stu.io.sbufWEntry.valid := sbufW_entry.sbufWriteValid && !l2sCheck
  stu.io.sbufWEntry.bits  := sbufW_entry
  stu.io.tmmRead <> io.tileData.tmmRead
  stu.io.sbuffer <> io.sbuffer
  
  when((stu.io.sbufWEntry.fire || !sbufW_entry.sbufWriteValid) &&
       (sbufW_ptr < tlb_resp_ptr)) {
    sbufW_ptr := sbufW_ptr + 1.U
  }
  
  // LSQueue 出队
  val deq_entry = ToTmuLSQEntry(out_ptr)
  val deq_enable = deq_entry.state.done
  when(deq_enable) {
    out_ptr := out_ptr + 1.U
  }
  io.done := deq_enable && deq_entry.row === (numTrows-1).U // last row of tileload/tilestore

  def tlbRespDone(id: Int): Bool = {
    tlbHit && tlb_resp_ptr.value === id.U
  }
  def l2CReqDone(id: Int): Bool = {
    val i = id % numL2CReadPort
    ldu.io.tlReqEntry(i).fire && ldu.io.tlReqEntry(i).bits.id === id.U
  }
  def l2CRespDone(id: Int): Bool = {
    val i = id % numL2CReadPort
    ldu.io.tlRespDone(i) && ldu.io.tlRespId(i) === id.U
  }
  def sbufWDone(id: Int): Bool = {
    stu.io.sbufWEntry.fire && stu.io.sbufWEntry.bits.id === id.U
  }

  // LSQueue 表项state的更新
  for(i <- 0 until tileLSQ_sz) {
    when(enq_enable && in_ptr.value === i.U) {
      state_queue(i).valid := true.B
    }
    when(tlbRespDone(i)) {
      state_queue(i).tlb_done := true.B
    }
    when(l2CReqDone(i) || sbufWDone(i)) {
      state_queue(i).req_done := true.B
    }
    when(sbufWDone(i) || l2CRespDone(i)) {
      state_queue(i).done := true.B
    }
    when(deq_enable && out_ptr.value === i.U) {
      state_queue(i).reset()
    }
  }
}

class TmuStoreUnit(implicit p: Parameters) extends XSModule with TileLSQParams {
  val io = IO(new Bundle {
    val sbufWEntry = Flipped(Decoupled(new TileLSQEntry))
    val tmmRead    = new TilesReadPort
    val sbuffer    = Decoupled(new DCacheLineReq)
  })

  // s0: read tile register
  val s0_entry = io.sbufWEntry.bits
  val s0_s1_fire  = Wire(Bool())
  val s1_in_ready = Wire(Bool())
  io.tmmRead.ren   := s0_s1_fire
  io.tmmRead.rtile := s0_entry.tmm
  io.tmmRead.rrow  := s0_entry.row

  s0_s1_fire           := io.sbufWEntry.valid && s1_in_ready
  io.sbufWEntry.ready := s0_s1_fire

  // s1: write to sbuffer
  val s1_entry = RegEnable(s0_entry, s0_s1_fire)
  val s1_valid = ValidHold(s0_s1_fire, io.sbuffer.fire)
  io.sbuffer.valid := s1_valid
  io.sbuffer.bits  := DontCare
  io.sbuffer.bits.cmd   := MemoryOpConstants.M_XWR
  io.sbuffer.bits.vaddr := s1_entry.vaddr
  io.sbuffer.bits.addr  := s1_entry.paddr
  io.sbuffer.bits.data  := io.tmmRead.rdata
  io.sbuffer.bits.mask  := Fill(l1TstDataWidth/8, true.B)

  s1_in_ready := io.sbuffer.fire || !s1_valid
}

class TmuL2CLoadUnit(implicit p: Parameters) extends XSModule with TileLSQParams {
  val io = IO(new Bundle {
    val tlReqEntry  = Flipped(Vec(numL2CReadPort, Decoupled(new TileLSQEntry)))
    val memBus      = Vec(numL2CReadPort, new TmuMemBus)
    val tmmWrite    = Vec(numL2CReadPort, new TilesWritePort)
    val tlRespId    = Output(Vec(numL2CReadPort, UInt(sourceIDWidth.W)))
    val tlRespEntry = Flipped(Vec(numL2CReadPort, new TileLSQEntry))
    val tlRespDone  = Output(Vec(numL2CReadPort, Bool()))
  })

  // l2-cache req
  for(i <- 0 until numL2CReadPort) {
    io.memBus(i).req.valid       := io.tlReqEntry(i).valid
    io.memBus(i).req.bits.source := io.tlReqEntry(i).bits.id
    io.memBus(i).req.bits.paddr  := io.tlReqEntry(i).bits.paddr
    io.tlReqEntry(i).ready       := io.memBus(i).req.ready
    // l2-cache resp
    io.memBus(i).resp.ready := true.B // always ready to receive response
    io.tlRespId(i) := io.memBus(i).resp.bits.source
    val tlResp_cnt = DataTransCnt(l2TldNumBurst)
    val l2Rdata_buf = Reg(Vec(l2TldNumBurst-1, UInt(l2TldDataWidth.W)))
    when(io.memBus(i).resp.fire) {
      tlResp_cnt.update()
      when(!tlResp_cnt.last) {
        l2Rdata_buf(tlResp_cnt.value) := io.memBus(i).resp.bits.rdata
      }
    }
    io.tlRespDone(i) := io.memBus(i).resp.fire && tlResp_cnt.last // tilelink D channel transfer done
    // write back to tile register
    io.tmmWrite(i).wen   := io.tlRespDone(i)
    io.tmmWrite(i).wtile := io.tlRespEntry(i).tmm
    io.tmmWrite(i).wrow  := io.tlRespEntry(i).row
    io.tmmWrite(i).wdata := Cat(io.memBus(i).resp.bits.rdata, l2Rdata_buf.asUInt)
  }
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