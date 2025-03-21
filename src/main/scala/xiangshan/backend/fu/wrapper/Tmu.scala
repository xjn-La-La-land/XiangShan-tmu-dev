package xiangshan.backend.fu.wrapper

import org.chipsalliance.cde.config.Parameters
import chisel3._
import xiangshan.backend.fu.{FuConfig, FuncUnit, TmuModule}
import xiangshan.backend.fu.TmuParams
import xiangshan.backend.fu.FuncUnitIO
import xiangshan.cache.mmu.TlbRequestIO
import freechips.rocketchip.tilelink._
import freechips.rocketchip.diplomacy.LazyModule

// add memory io
class TmuFuncUnitIO(cfg: FuConfig)(implicit p: Parameters) extends FuncUnitIO(cfg) with TmuParams {
  val tlb  = new TlbRequestIO()
  val node = TLClientNode(Seq(clientParameters))
}

class Tmu (cfg: FuConfig)(implicit p: Parameters) extends FuncUnit(cfg) {
  override val io: TmuFuncUnitIO = new TmuFuncUnitIO(cfg)

  private val tmu = LazyModule(new TmuModule())
  val tmuIO = tmu.module.io
  io.tlb <> tmuIO.tlb
  io.node := tmu.clientNode

  private val src    = io.in.bits.data.src.take(2) // 2 src
  private val imm    = io.in.bits.data.imm(31, 0)
  private val func   = io.in.bits.ctrl.fuOpType // tmu op
  private val robIdx = io.in.bits.ctrl.robIdx

  tmuIO.in.bits.src    := src
  tmuIO.in.bits.imm    := imm
  tmuIO.in.bits.func   := func
  tmuIO.in.bits.robIdx := robIdx

  // uncertain latency
  tmuIO.in.valid := io.in.valid
  io.in.ready  := tmuIO.in.ready
  io.out.valid := tmuIO.out.valid
  tmuIO.out.ready := io.out.ready

  io.out.bits := DontCare
  io.out.bits.res.data    := 0.U  // no write back data for Xtm instruction
  io.out.bits.ctrl.robIdx := tmuIO.out.bits.robIdx

}

