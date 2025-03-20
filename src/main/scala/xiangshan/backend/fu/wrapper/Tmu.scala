package xiangshan.backend.fu.wrapper

import org.chipsalliance.cde.config.Parameters
import chisel3._
import xiangshan.backend.fu.{FuConfig, FuncUnit, TmuDataModule}
import xiangshan.backend.fu.TmuParams
import xiangshan.backend.fu.FuncUnitIO
import xiangshan.cache.mmu.TlbRequestIO
import freechips.rocketchip.tilelink._

// add memory io
class TmuFuncUnitIO(cfg: FuConfig)(implicit p: Parameters) extends FuncUnitIO(cfg) with TmuParams {
  val tlb  = new TlbRequestIO()
  val node = TLClientNode(Seq(clientParameters))
}

class Tmu (cfg: FuConfig)(implicit p: Parameters) extends FuncUnit(cfg) {
  override val io: TmuFuncUnitIO = new TmuFuncUnitIO(cfg)

  private val tmuModule = Module(new TmuDataModule)
  io.tlb <> tmuModule.io.tlb
  io.node := tmuModule.io.node

  private val src    = io.in.bits.data.src.take(2) // 2 src
  private val imm    = io.in.bits.data.imm(31, 0)
  private val func   = io.in.bits.ctrl.fuOpType // tmu op
  private val robIdx = io.in.bits.ctrl.robIdx

  tmuModule.io.in.bits.src    := src
  tmuModule.io.in.bits.imm    := imm
  tmuModule.io.in.bits.func   := func
  tmuModule.io.in.bits.robIdx := robIdx

  // uncertain latency
  tmuModule.io.in.valid := io.in.valid
  io.in.ready  := tmuModule.io.in.ready
  io.out.valid := tmuModule.io.out.valid
  tmuModule.io.out.ready := io.out.ready

  io.out.bits := DontCare
  io.out.bits.res.data   := 0.U  // no write back data for Xtm instruction
  io.out.bits.ctrl.robIdx := tmuModule.io.out.bits.robIdx

}

