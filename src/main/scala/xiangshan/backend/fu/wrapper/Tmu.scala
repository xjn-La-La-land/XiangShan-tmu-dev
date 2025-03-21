package xiangshan.backend.fu.wrapper

import org.chipsalliance.cde.config.Parameters
import chisel3._
import xiangshan.backend.fu.{FuConfig, FuncUnit, TmuModule, TmuMemBus}
import xiangshan.backend.fu.TmuParams
import xiangshan.backend.fu.FuncUnitIO
import xiangshan.cache.mmu.TlbRequestIO
import freechips.rocketchip.tilelink._
import freechips.rocketchip.diplomacy.LazyModule

class Tmu (cfg: FuConfig)(implicit p: Parameters) extends FuncUnit(cfg) {

  private val tmu = Module(new TmuModule)
  io.tmuTlb.get    <> tmu.io.tlb
  io.tmuMemBus.get <> tmu.io.memBus

  private val src    = io.in.bits.data.src.take(2) // 2 src
  private val imm    = io.in.bits.data.imm(31, 0)
  private val func   = io.in.bits.ctrl.fuOpType // tmu op
  private val robIdx = io.in.bits.ctrl.robIdx

  tmu.io.in.bits.src    := src
  tmu.io.in.bits.imm    := imm
  tmu.io.in.bits.func   := func
  tmu.io.in.bits.robIdx := robIdx

  // uncertain latency
  tmu.io.in.valid := io.in.valid
  io.in.ready  := tmu.io.in.ready
  io.out.valid := tmu.io.out.valid
  tmu.io.out.ready := io.out.ready

  io.out.bits := DontCare
  io.out.bits.res.data    := 0.U  // no write back data for Xtm instruction
  io.out.bits.ctrl.robIdx := tmu.io.out.bits.robIdx

}

