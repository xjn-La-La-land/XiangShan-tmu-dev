package xiangshan.backend.fu.wrapper

import org.chipsalliance.cde.config.Parameters
import chisel3._
import xiangshan.backend.fu.{FuConfig, FuncUnit, TmuDataModule}

class Tmu (cfg: FuConfig)(implicit p: Parameters) extends FuncUnit(cfg) {
  private val tmuModule = Module(new TmuDataModule)

  private val src = io.in.bits.data.src.take(2) // 2 src
  private val imm = io.in.bits.data.imm(31, 0)
  private val func = io.in.bits.ctrl.fuOpType // tmu op

  tmuModule.io.in.bits.src  := src
  tmuModule.io.in.bits.imm  := imm
  tmuModule.io.in.bits.func := func

  // uncertain latency
  tmuModule.io.in.valid := io.in.valid
  io.in.ready  := tmuModule.io.in.ready
  io.out.valid := tmuModule.io.out.valid
  tmuModule.io.out.ready := io.out.ready
  io.out.bits.res.data := 0.U  // no write back data for Xtm instruction

  connectNonPipedCtrlSingal
}