package xiangshan.backend.fu

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import utils._
import utility.{ZeroExt, SignExt}
import xiangshan._
import xiangshan.TMUOpType.isTdp

class TmuDataInput(implicit p: Parameters) extends XSBundle {
  val src  = Vec(2, UInt(XLEN.W))
  val imm  = UInt(32.W)
  val func = FuOpType()
}

// HINT: Tile Matrix Unit
// Tile Matrix Unit is a special functional unit that is used to accelerate matrix operations.
class TmuDataModule(implicit p: Parameters) extends XSModule {
  val io = IO(new Bundle() {
    val in  = Flipped(Decoupled(new TmuDataInput))
    val out = Decoupled(new Bundle{
      // empty
    })
    // val mem = ...
  })

  // regs for input data
  val regs  = RegEnable(io.in.bits, io.in.fire)
  val valid = RegInit(false.B)

  when(io.in.fire) {
    valid := true.B
  }.elsewhen(io.out.fire) {
    valid := false.B
  }
  
  io.in.ready := io.out.fire || !valid

  // mini decode for Xtm instrs
  val isTileLoad  = regs.func === TMUOpType.tileload
  val isTileStore = regs.func === TMUOpType.tilestore
  val isTDP       = isTdp(regs.func)

  val tmm2_sign = regs.func(1) === "0b1".U
  val tmm3_sign = regs.func(0) === "0b1".U

  val tmm1 = regs.imm(2, 0)
  val tmm2 = regs.imm(5, 3)
  val tmm3 = regs.imm(8, 6)

  val tgt_vaddr = regs.src(0) + ZeroExt(Cat(regs.imm(31, 3), 0.U(3.W)), XLEN)
  val stride    = regs.src(1) // 主轴长度

  // implement an empty module for debugging!
  val numDelayCycle = 100
  val delayCnt = RegInit(0.U(numDelayCycle.U.getWidth.W))
  when(valid && delayCnt =/= numDelayCycle.U) {
    delayCnt := delayCnt + 1.U
  }.elsewhen(io.out.fire) {
    delayCnt := 0.U
  }

  io.out.valid := valid && delayCnt === numDelayCycle.U
  
  when(io.out.fire) {
    when(isTileLoad) {
      printf(p"[TMU] tileloadd tmm${tmm1}, vaddr = 0x${tgt_vaddr}%x, stride = 0x${stride}%x\n")
    }.elsewhen(isTileStore) {
      printf(p"[TMU] tilestored tmm${tmm1}, vaddr = 0x${tgt_vaddr}%x, stride = 0x${stride}%x\n")
    }.elsewhen(isTDP) {
      printf(p"[TMU] tdpb??d tmm${tmm1}, tmm${tmm2}, tmm${tmm3}, sign = ${tmm2_sign}, ${tmm3_sign}\n")
    }.otherwise {
      printf(p"[TMU] unknown tmu op!\n")
    }
  }

  
}