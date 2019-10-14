// See LIENSE.Berkeley for license details.
// See LICENSE.SiFive for license details.


package freechips.rocketchip.rocket

import Chisel._
import Chisel.ImplicitConversions._
import chisel3.core.withReset
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.tile._
import freechips.rocketchip.util._
import freechips.rocketchip.util.property._
import scala.collection.immutable.ListMap
import scala.collection.mutable.ArrayBuffer

case class RocketCoreParams(
  bootFreqHz: BigInt = 0,
  useVM: Boolean = true,
  useUser: Boolean = false,
  useDebug: Boolean = true,
  useAtomics: Boolean = true,
  useAtomicsOnlyForIO: Boolean = false,
  useCompressed: Boolean = true,
  nLocalInterrupts: Int = 0,
  nBreakpoints: Int = 1,
  nPMPs: Int = 8,
  nPerfCounters: Int = 0,
  haveBasicCounters: Boolean = true,
  misaWritable: Boolean = true,
  nL2TLBEntries: Int = 0,
  mtvecInit: Option[BigInt] = Some(BigInt(0)),
  mtvecWritable: Boolean = true,
  fastLoadWord: Boolean = true,
  fastLoadByte: Boolean = false,
  tileControlAddr: Option[BigInt] = None,
  mulDiv: Option[MulDivParams] = Some(MulDivParams()),
  //vectorunit: Option[VectorParams] = Some(VectorParams()),
  fpu: Option[FPUParams] = Some(FPUParams())
) extends CoreParams {
  val fetchWidth: Int = if (useCompressed) 2 else 1
  //  fetchWidth doubled, but coreInstBytes halved, for RVC:
  val decodeWidth: Int = fetchWidth / (if (useCompressed) 2 else 1)
  val retireWidth: Int = 1
  val instBits: Int = if (useCompressed) 16 else 32
}

trait HasRocketCoreParameters extends HasCoreParameters {
  val rocketParams: RocketCoreParams = tileParams.core.asInstanceOf[RocketCoreParams]

  val fastLoadWord = rocketParams.fastLoadWord
  val fastLoadByte = rocketParams.fastLoadByte

  val mulDivParams = rocketParams.mulDiv.getOrElse(MulDivParams()) // TODO ask andrew about this
  //val vectorParams = rocketParams.vectorunit.getOrElse(VectorParams())
  require(!fastLoadByte || fastLoadWord)
}

class Rocket(implicit p: Parameters) extends CoreModule()(p)
    with HasRocketCoreParameters
    with HasCoreIO {

  // performance counters
  def pipelineIDToWB[T <: Data](x: T): T =
    RegEnable(RegEnable(RegEnable(x, !ctrl_killd), ex_pc_valid), mem_pc_valid)
  val perfEvents = new EventSets(Seq(
    new EventSet((mask, hits) => Mux(mask(0), wb_xcpt, wb_valid && pipelineIDToWB((mask & hits).orR)), Seq(
      ("exception", () => false.B),
      ("load", () => id_ctrl.mem && id_ctrl.mem_cmd === M_XRD && !id_ctrl.fp),
      ("store", () => id_ctrl.mem && id_ctrl.mem_cmd === M_XWR && !id_ctrl.fp),
      ("amo", () => Bool(usingAtomics) && id_ctrl.mem && (isAMO(id_ctrl.mem_cmd) || id_ctrl.mem_cmd.isOneOf(M_XLR, M_XSC))),
      ("system", () => id_ctrl.csr =/= CSR.N),
      ("arith", () => id_ctrl.wxd && !(id_ctrl.jal || id_ctrl.jalr || id_ctrl.mem || id_ctrl.fp || id_ctrl.mul || id_ctrl.div || id_ctrl.csr =/= CSR.N)),
      ("branch", () => id_ctrl.branch),
      ("jal", () => id_ctrl.jal),
      ("jalr", () => id_ctrl.jalr))
      ++ (if (!usingMulDiv) Seq() else Seq(
        ("mul", () => if (pipelinedMul) id_ctrl.mul else id_ctrl.div && (id_ctrl.alu_fn & ALU.FN_DIV) =/= ALU.FN_DIV),
        ("div", () => if (pipelinedMul) id_ctrl.div else id_ctrl.div && (id_ctrl.alu_fn & ALU.FN_DIV) === ALU.FN_DIV)))
      ++ (if (!usingFPU) Seq() else Seq(
        ("fp load", () => id_ctrl.fp && io.fpu.dec.ldst && io.fpu.dec.wen),
        ("fp store", () => id_ctrl.fp && io.fpu.dec.ldst && !io.fpu.dec.wen),
        ("fp add", () => id_ctrl.fp && io.fpu.dec.fma && io.fpu.dec.swap23),
        ("fp mul", () => id_ctrl.fp && io.fpu.dec.fma && !io.fpu.dec.swap23 && !io.fpu.dec.ren3),
        ("fp mul-add", () => id_ctrl.fp && io.fpu.dec.fma && io.fpu.dec.ren3),
        ("fp div/sqrt", () => id_ctrl.fp && (io.fpu.dec.div || io.fpu.dec.sqrt)),
        ("fp other", () => id_ctrl.fp && !(io.fpu.dec.ldst || io.fpu.dec.fma || io.fpu.dec.div || io.fpu.dec.sqrt))))),
    new EventSet((mask, hits) => (mask & hits).orR, Seq(
      ("load-use interlock", () => id_ex_hazard && ex_ctrl.mem || id_mem_hazard && mem_ctrl.mem || id_wb_hazard && wb_ctrl.mem),
      ("long-latency interlock", () => id_sboard_hazard),
      ("csr interlock", () => id_ex_hazard && ex_ctrl.csr =/= CSR.N || id_mem_hazard && mem_ctrl.csr =/= CSR.N || id_wb_hazard && wb_ctrl.csr =/= CSR.N),
      ("I$ blocked", () => icache_blocked),
      ("D$ blocked", () => id_ctrl.mem && dcache_blocked),
      ("branch misprediction", () => take_pc_mem && mem_direction_misprediction),
      ("control-flow target misprediction", () => take_pc_mem && mem_misprediction && mem_cfi && !mem_direction_misprediction && !icache_blocked),
      ("flush", () => wb_reg_flush_pipe),
      ("replay", () => replay_wb))
      ++ (if (!usingMulDiv) Seq() else Seq(
        ("mul/div interlock", () => id_ex_hazard && (ex_ctrl.mul || ex_ctrl.div) || id_mem_hazard && (mem_ctrl.mul || mem_ctrl.div) || id_wb_hazard && wb_ctrl.div)))
      ++ (if (!usingFPU) Seq() else Seq(
        ("fp interlock", () => id_ex_hazard && ex_ctrl.fp || id_mem_hazard && mem_ctrl.fp || id_wb_hazard && wb_ctrl.fp || id_ctrl.fp && id_stall_fpu)))),
    new EventSet((mask, hits) => (mask & hits).orR, Seq(
      ("I$ miss", () => io.imem.perf.acquire),
      ("D$ miss", () => io.dmem.perf.acquire),
      ("D$ release", () => io.dmem.perf.release),
      ("ITLB miss", () => io.imem.perf.tlbMiss),
      ("DTLB miss", () => io.dmem.perf.tlbMiss),
      ("L2 TLB miss", () => io.ptw.perf.l2miss)))))


/*  when(io.dmem.perf.acquire){
    printf("[checkcachemissrate] %x \n", io.dmem.perf.acquire)
  }

  when((id_ctrl.fp && io.fpu.dec.ldst && io.fpu.dec.wen)||(id_ctrl.fp && io.fpu.dec.ldst && !io.fpu.dec.wen)||(id_ctrl.mem && id_ctrl.mem_cmd === M_XRD && !id_ctrl.fp)||(id_ctrl.mem && id_ctrl.mem_cmd === M_XWR && !id_ctrl.fp)){
    printf("[checkcacheaccess] %x \n", (id_ctrl.fp && io.fpu.dec.ldst && io.fpu.dec.wen)||(id_ctrl.fp && io.fpu.dec.ldst && !io.fpu.dec.wen)||(id_ctrl.mem && id_ctrl.mem_cmd === M_XRD && !id_ctrl.fp)||(id_ctrl.mem && id_ctrl.mem_cmd === M_XWR && !id_ctrl.fp))
  }*/
  val pipelinedMul = usingMulDiv && mulDivParams.mulUnroll == xLen
  val decode_table = {
    (if (usingMulDiv) new MDecode(pipelinedMul) +: (xLen > 32).option(new M64Decode(pipelinedMul)).toSeq else Nil) ++:
    (if (usingAtomics) new ADecode +: (xLen > 32).option(new A64Decode).toSeq else Nil) ++:
    (if (fLen >= 32) new FDecode +: (xLen > 32).option(new F64Decode).toSeq else Nil) ++:
    (if (fLen >= 64) new DDecode +: (xLen > 32).option(new D64Decode).toSeq else Nil) ++:
    (usingRoCC.option(new RoCCDecode)) ++:
    ((xLen > 32).option(new I64Decode)) ++:
    (usingVM.option(new SDecode)) ++:
    (usingDebug.option(new DebugDecode)) ++:
    Seq(new IDecode)
  } flatMap(_.table)

  val ex_ctrl = Reg(new IntCtrlSigs)
  val mem_ctrl = Reg(new IntCtrlSigs)
  val wb_ctrl = Reg(new IntCtrlSigs)

  val ex_reg_xcpt_interrupt  = Reg(Bool())
  val ex_reg_valid           = Reg(Bool())
  val ex_reg_valid_vlsd_vssd = Reg(Bool())
  val ex_reg_rvc             = Reg(Bool())
  val ex_reg_btb_resp        = Reg(new BTBResp)
  val ex_reg_xcpt            = Reg(Bool())
  val ex_reg_flush_pipe      = Reg(Bool())
  val ex_reg_load_use        = Reg(Bool())
  val ex_reg_cause           = Reg(UInt())
  val ex_reg_replay = Reg(Bool())
  val ex_reg_pc = Reg(UInt())
  val ex_reg_inst = Reg(Bits())
  val ex_reg_raw_inst = Reg(UInt())

  val mem_reg_xcpt_interrupt  = Reg(Bool())
  val mem_reg_valid           = Reg(Bool())
  val mem_reg_rvc             = Reg(Bool())
  val mem_reg_btb_resp        = Reg(new BTBResp)
  val mem_reg_xcpt            = Reg(Bool())
  val mem_reg_replay          = Reg(Bool())
  val mem_reg_flush_pipe      = Reg(Bool())
  val mem_reg_cause           = Reg(UInt())
  val mem_reg_slow_bypass     = Reg(Bool())
  val mem_reg_load            = Reg(Bool())
  val mem_reg_store           = Reg(Bool())
  val mem_reg_sfence = Reg(Bool())
  val mem_reg_pc = Reg(UInt())
  val mem_reg_inst = Reg(Bits())
  val mem_reg_raw_inst = Reg(UInt())
  val mem_reg_wdata = Reg(Bits())
  //zazad begins
  val v_mem_reg_wdata = Reg(Bits())
  val v_mem_reg_rs2 = Reg(Bits())
  val wb_xcpt_temp  =  Wire(Bool())
  //zazad ends
  val mem_reg_rs2 = Reg(Bits())
  val mem_br_taken = Reg(Bool())
  val take_pc_mem = Wire(Bool())
  val ctrl_stalld = Wire(Bool())
  val wb_reg_valid           = Reg(Bool())
  val wb_reg_xcpt            = Reg(Bool())
  val wb_reg_replay          = Reg(Bool())
  val wb_reg_flush_pipe      = Reg(Bool())
  val wb_reg_cause           = Reg(UInt())
  val wb_reg_sfence = Reg(Bool())
  val wb_reg_pc = Reg(UInt())
  val wb_reg_inst = Reg(Bits())
  val wb_reg_raw_inst = Reg(UInt())
  val wb_reg_wdata = Reg(Bits())
  //zazad begins
  val v_wb_reg_wdata = Reg(Bits())
  val v_alu_out = Wire(UInt(width=256.W))
  val temp_for_dmem_write_data = Wire(UInt(width=256.W))
  val vector_unit = Module (new Vector_Unit)
  //zazad ends
  val wb_reg_rs2 = Reg(Bits())
  val take_pc_wb = Wire(Bool())
  val prev_ctrl_stalld = Reg(Bool())

  val prev_dmem_resp_valid = Reg(Bool())
  val take_pc_mem_wb = take_pc_wb || take_pc_mem
  val take_pc = take_pc_mem_wb

  // decode stage
  val ibuf = Module(new IBuf)
  val id_expanded_inst = ibuf.io.inst.map(_.bits.inst)
  val id_raw_inst = ibuf.io.inst.map(_.bits.raw)
  val id_inst = id_expanded_inst.map(_.bits)
  ibuf.io.imem <> io.imem.resp
  ibuf.io.kill := take_pc

  require(decodeWidth == 1 /* TODO */ && retireWidth == decodeWidth)
  val id_ctrl = Wire(new IntCtrlSigs()).decode(id_inst(0), decode_table)
  //zazad begins
  //when (id_ctrl.mem && id_ctrl.vec){
    //printf ("[trackloadsnew] %x\n", id_inst(0))
  //}
  //zazad ends
  val id_raddr3 = id_expanded_inst(0).rs3
  val id_raddr2 = id_expanded_inst(0).rs2
  val id_raddr1 = id_expanded_inst(0).rs1
  val id_waddr  = id_expanded_inst(0).rd
  val id_load_use = Wire(Bool())
  val id_reg_fence = Reg(init=Bool(false))
  val id_ren = IndexedSeq(id_ctrl.rxs1, id_ctrl.rxs2)
  val id_vren = IndexedSeq(id_ctrl.rxs1, id_ctrl.rxs2, id_ctrl.rxs2)//added true for the last vector register in VFMADD instruction
  //val id_raddr = IndexedSeq(id_raddr1, id_raddr2)
  val id_raddr = Wire(Vec(2, UInt()))
  //zazad
  val rf = new RegFile(31, xLen)
  val id_rs = id_raddr.map(rf.read _)
  //zazad begins
  //val id_vraddr = Vec(2, UInt(256.W))

  //val vrf = new  RegFile(31, 256)
  val vrf = Mem(31, Bits(width = 256.W))
  val id_vrs = Wire(Vec(3, UInt(256.W)))
  val id_vrs_0 = Wire(UInt(256.W))
  val id_vrs_1 = Wire(UInt(256.W))
  val id_vraddr = Wire(Vec(3, UInt()))

  val csr = Module(new CSRFile(perfEvents))
  val number_of_elements = csr.io.vl //UInt(8)
  val number_of_lanes: Int = 8
  val elements_width = UInt(2)
  val cnt_cache = Reg(init = UInt(0, log2Up(15+1)))
  val cnt_cache_vsd = Reg(init = UInt(0, log2Up(15+1)))

  
  when(io.dmem.perf.acquire){
    printf("[checkcachemissrate] %x \n", io.dmem.perf.acquire)
  }

  when((id_ctrl.fp && io.fpu.dec.ldst && io.fpu.dec.wen)||(id_ctrl.fp && io.fpu.dec.ldst && !io.fpu.dec.wen)||(id_ctrl.mem && id_ctrl.mem_cmd === M_XRD && !id_ctrl.fp)||(id_ctrl.mem && id_ctrl.mem_cmd === M_XWR && !id_ctrl.fp)){
    printf("[checkcacheaccess] %x \n", (id_ctrl.fp && io.fpu.dec.ldst && io.fpu.dec.wen)||(id_ctrl.fp && io.fpu.dec.ldst && !io.fpu.dec.wen)||(id_ctrl.mem && id_ctrl.mem_cmd === M_XRD && !id_ctrl.fp)||(id_ctrl.mem && id_ctrl.mem_cmd === M_XWR && !id_ctrl.fp))
  }


  when (io.dmem.perf.acquire){
    printf ("[cachemissrate] %x \n", io.dmem.perf.acquire)}
  when (isWrite(id_ctrl.mem_cmd) && id_ctrl.mem && id_ctrl.vec && !id_ctrl.wxd && !id_ctrl.scatter_gather)
  {
    id_vraddr := IndexedSeq(id_raddr1,id_raddr3,id_raddr3)
    id_raddr  := IndexedSeq(id_raddr1,id_raddr2)
    val id_vrs_R0 = id_vraddr.map(a => vrf(a))
    id_vrs(2) := vrf(id_raddr3) 
    //id_vraddr.map(vrf.read _)
    val id_rs_R0 = id_raddr.map(rf.read _)
    id_vrs(0) := id_vrs_R0(0)
    id_vrs_0 := id_vrs_R0(0)
    printf ("[checkcachecounter]%d num %d idecode is in 1 [mem_cmd %d mem %b vec %b !wxd %b !scatter_gather %b] @@@@@@@@@@@@@@@@@@@@@@@@@@@@@ %x id_rs [%d %d] [%d %d] id_vrs [%d %x] [%d %x] \n",csr.io.vl, number_of_elements, id_ctrl.mem_cmd, id_ctrl.mem, id_ctrl.vec, !id_ctrl.wxd,!id_ctrl.scatter_gather,id_inst(0), id_raddr(0), id_rs_R0(0), id_raddr(1),id_rs_R0(1), id_vraddr(0), id_vrs_R0(0), id_vraddr(1), id_vrs_R0(1))

  }.elsewhen (isWrite(id_ctrl.mem_cmd) && id_ctrl.mem && id_ctrl.vec && !id_ctrl.wxd && id_ctrl.scatter_gather )
  {
    id_vraddr := IndexedSeq(id_raddr2,id_raddr3,id_raddr3)
    id_raddr  := IndexedSeq(id_raddr1,id_raddr2)
    val id_vrs_R0 = id_vraddr.map(a => vrf(a))
    id_vrs(2) := vrf(id_raddr3) 
      //id_vraddr.map(vrf.read _)
    val id_rs_R0 = id_raddr.map(rf.read _)
    id_vrs(0) := id_vrs_R0(0)
    id_vrs_0 := id_vrs_R0(0)
     if (!enableCommitLog)
    printf ("[checkcachecounter]%d num %d idecode is in 2 [mem_cmd %d mem %b vec %b !wxd %b scatter_gather %b]  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@ %x id_rs [%d %d] [%d %d] id_vrs [%d %x] [%d %x]\n",csr.io.vl,number_of_elements,id_ctrl.mem_cmd, id_ctrl.mem, id_ctrl.vec, !id_ctrl.wxd,id_ctrl.scatter_gather, id_inst(0),id_raddr(0), id_rs_R0(0), id_raddr(1),id_rs_R0(1), id_vraddr(0), id_vrs_R0(0), id_vraddr(1), id_vrs_R0(1))

  }.elsewhen (isRead(id_ctrl.mem_cmd) && id_ctrl.mem && id_ctrl.vec && id_ctrl.wxd && !id_ctrl.scatter_gather )
  {
    id_vraddr := IndexedSeq(id_raddr1,id_raddr2,id_raddr2)
    id_raddr  := IndexedSeq(id_raddr1,id_raddr2)
    val id_vrs_R1 = id_vraddr.map(a => vrf(a))
    id_vrs(2) := vrf(id_raddr3) 
      //id_vraddr.map(vrf.read _)
    val id_rs_R0 = id_raddr.map(rf.read _)
    id_vrs(1) := id_vrs_R1(1)
    id_vrs_1 := id_vrs_R1(1)
      
    printf ("[checkcachecounter]%d  num %d idecode is in 3 [mem_cmd %d mem %b vec_vec %b vec_scalar %b wxd %b !scatter_gather %b] @@@@@@@@@@@@@@@@@@@@@@@@@@@@@ %x id_rs [%d %d] [%d %d] id_vrs [%d %x] [%d %x]\n",csr.io.vl,number_of_elements, id_ctrl.mem_cmd, id_ctrl.mem, id_ctrl.vec,id_ctrl.vec_scalar, id_ctrl.wxd,!id_ctrl.scatter_gather,id_inst(0),id_raddr(0), id_rs_R0(0), id_raddr(1),id_rs_R0(1), id_vraddr(0), id_vrs_R1(0), id_vraddr(1), id_vrs_R1(1))

  }.elsewhen (isRead(id_ctrl.mem_cmd) && id_ctrl.mem && id_ctrl.vec && id_ctrl.wxd && id_ctrl.scatter_gather )
  {
    id_vraddr := IndexedSeq(id_raddr1,id_raddr2,id_raddr2)
    id_raddr  := IndexedSeq(id_raddr1,id_raddr2)
    id_vrs(2) := vrf(id_raddr3) 
    val id_vrs_R1 = id_vraddr.map(a => vrf(a))
      //id_vraddr.map(vrf.read _)
    val id_rs_R0 = id_raddr.map(rf.read _)
    id_vrs(1) := id_vrs_R1(1)
    id_vrs_1 := id_vrs_R1(1)
      
    printf ("[checkcachecounter]%d num %d idecode is in 4 [mem_cmd %d mem %b vec %b vec_scalar %b wxd %b scatter_gather %b] @@@@@@@@@@@@@@@@@@@@@@@@@@@@@ %x id_rs [%d %d] [%d %d] id_vrs [%d %x] [%d %x]\n",csr.io.vl, number_of_elements, id_ctrl.mem_cmd, id_ctrl.mem, id_ctrl.vec, id_ctrl.vec_scalar,id_ctrl.wxd,id_ctrl.scatter_gather,id_inst(0),id_raddr(0), id_rs_R0(0), id_raddr(1),id_rs_R0(1), id_vraddr(0), id_vrs_R1(0), id_vraddr(1), id_vrs_R1(1))
  }.otherwise {

    id_vraddr := IndexedSeq(id_raddr1,id_raddr2,id_raddr3)
    id_raddr  := IndexedSeq(id_raddr1,id_raddr2)
    id_vrs(2) := vrf(id_raddr3) 
    id_vrs(0) := vrf(id_raddr1) 
    id_vrs(1) := vrf(id_raddr2)
    when (id_ctrl.vec){
    printf ("[checkcachecounter]++++++++++++++++%d id_raddr %x %x  num %d idecode is in 4 [mem_cmd %d mem %b vec %b wxd %b scatter_gather %b] @@@@@@@@@@@@@@@@@@@@@@@@@@@@@ %x id_vrs [%d %d %d] [%x %x %x] \n",csr.io.vl, id_raddr1, id_raddr2, number_of_elements, id_ctrl.mem_cmd, id_ctrl.mem, id_ctrl.vec, id_ctrl.wxd,id_ctrl.scatter_gather,id_inst(0), id_vraddr(0), id_vraddr(1), id_vraddr(2), id_vrs(0), id_vrs(1), id_vrs(2))}

  }
 

  val ctrl_killd = Wire(Bool())
  val ctrl_killd_vlsd_vssd = Wire(Bool())
  val id_npc = (ibuf.io.pc.asSInt + ImmGen(IMM_UJ, id_inst(0))).asUInt

//  val csr = Module(new CSRFile(perfEvents))
  val id_csr_en = id_ctrl.csr.isOneOf(CSR.S, CSR.C, CSR.W)
  val id_system_insn = id_ctrl.csr >= CSR.I
  val id_csr_ren = id_ctrl.csr.isOneOf(CSR.S, CSR.C) && id_raddr1 === UInt(0)
  val id_csr = Mux(id_csr_ren, CSR.R, id_ctrl.csr)
  val id_sfence = id_ctrl.mem && id_ctrl.mem_cmd === M_SFENCE
  val id_csr_flush = id_sfence || id_system_insn || (id_csr_en && !id_csr_ren && csr.io.decode(0).write_flush)

  val id_illegal_insn = !id_ctrl.legal ||
    (id_ctrl.mul || id_ctrl.div) && !csr.io.status.isa('m'-'a') ||
    id_ctrl.amo && !csr.io.status.isa('a'-'a') ||
    id_ctrl.fp && (csr.io.decode(0).fp_illegal || io.fpu.illegal_rm) ||
    id_ctrl.dp && !csr.io.status.isa('d'-'a') ||
    ibuf.io.inst(0).bits.rvc && !csr.io.status.isa('c'-'a') ||
    id_ctrl.rocc && csr.io.decode(0).rocc_illegal ||
    id_csr_en && (csr.io.decode(0).read_illegal || !id_csr_ren && csr.io.decode(0).write_illegal) ||
    !ibuf.io.inst(0).bits.rvc && ((id_sfence || id_system_insn) && csr.io.decode(0).system_illegal)
  // stall decode for fences (now, for AMO.rl; later, for AMO.aq and FENCE)
  val id_amo_aq = id_inst(0)(26)
  val id_amo_rl = id_inst(0)(25)
  val id_fence_next = id_ctrl.fence || id_ctrl.amo && id_amo_aq
  val id_mem_busy = !io.dmem.ordered || io.dmem.req.valid
  when (!id_mem_busy) { id_reg_fence := false }
  val id_rocc_busy = Bool(usingRoCC) &&
    (io.rocc.busy || ex_reg_valid && ex_ctrl.rocc ||
     mem_reg_valid && mem_ctrl.rocc || wb_reg_valid && wb_ctrl.rocc)
  val id_do_fence = Wire(init = id_rocc_busy && id_ctrl.fence ||
    id_mem_busy && (id_ctrl.amo && id_amo_rl || id_ctrl.fence_i || id_reg_fence && (id_ctrl.mem || id_ctrl.rocc)))

  val bpu = Module(new BreakpointUnit(nBreakpoints))
  bpu.io.status := csr.io.status
  bpu.io.bp := csr.io.bp
  bpu.io.pc := ibuf.io.pc
  bpu.io.ea := mem_reg_wdata

  val id_xcpt0 = ibuf.io.inst(0).bits.xcpt0
  val id_xcpt1 = ibuf.io.inst(0).bits.xcpt1
  val (id_xcpt, id_cause) = checkExceptions(List(
    (csr.io.interrupt, csr.io.interrupt_cause),
    (bpu.io.debug_if,  UInt(CSR.debugTriggerCause)),
    (bpu.io.xcpt_if,   UInt(Causes.breakpoint)),
    (id_xcpt0.pf.inst, UInt(Causes.fetch_page_fault)),
    (id_xcpt0.ae.inst, UInt(Causes.fetch_access)),
    (id_xcpt1.pf.inst, UInt(Causes.fetch_page_fault)),
    (id_xcpt1.ae.inst, UInt(Causes.fetch_access)),
    (id_illegal_insn,  UInt(Causes.illegal_instruction))))

  val idCoverCauses = List(
    (CSR.debugTriggerCause, "DEBUG_TRIGGER"),
    (Causes.breakpoint, "BREAKPOINT"),
    (Causes.fetch_page_fault, "FETCH_PAGE_FAULT"),
    (Causes.fetch_access, "FETCH_ACCESS"),
    (Causes.illegal_instruction, "ILLEGAL_INSTRUCTION")
  )
  coverExceptions(id_xcpt, id_cause, "DECODE", idCoverCauses)

  //vector processor bypass/forward path begins
  val vector_dcache_bypass_data =
    if (fastLoadByte) io.dmem.resp.bits.DcacheCpu_data(255, 0)
    else if (fastLoadWord) io.dmem.resp.bits.DcacheCpu_data_word_bypass(255, 0)
    else v_wb_reg_wdata
  //zazad ends
    val dcache_bypass_data =
    if (fastLoadByte) io.dmem.resp.bits.DcacheCpu_data(xLen-1, 0)
    else if (fastLoadWord) io.dmem.resp.bits.DcacheCpu_data_word_bypass(xLen-1, 0)
    else wb_reg_wdata

  // detect bypass opportunities
  val ex_waddr = ex_reg_inst(11,7)
  val mem_waddr = mem_reg_inst(11,7)
  val wb_waddr = wb_reg_inst(11,7)
  val bypass_sources = IndexedSeq(
    (Bool(true), UInt(0), UInt(0)), // treat reading x0 as a bypass
    (ex_reg_valid && ex_ctrl.wxd  && !ex_ctrl.vec, ex_waddr, mem_reg_wdata),
    (mem_reg_valid && mem_ctrl.wxd && !mem_ctrl.mem  && !mem_ctrl.vec, mem_waddr, wb_reg_wdata),
    (mem_reg_valid && mem_ctrl.wxd  && !mem_ctrl.vec, mem_waddr, dcache_bypass_data))
  val id_bypass_src = id_raddr.map(raddr => bypass_sources.map(s => s._1 && s._2 === raddr))

/*  val vector_bypass_sources = IndexedSeq(
    (Bool(true), UInt(0), UInt(0)), // treat reading x0 as a bypass
    (/*(*/ex_reg_valid /* || (/*number_of_elements === UInt(16) &&*/ !ex_ctrl.mem && vector_unit.io.resp.valid)) */ && ex_ctrl.wxd  && ex_ctrl.vec, ex_waddr, v_mem_reg_wdata),
    (/*(*/(mem_reg_valid && !ctrl_stalld) /* || (wb_reg_valid && !ctrl_stalld)) */  && mem_ctrl.wxd && !mem_ctrl.mem  && mem_ctrl.vec, mem_waddr, v_wb_reg_wdata),
    (/*(*/(mem_reg_valid && !ctrl_stalld) /* || (wb_reg_valid && !ctrl_stalld)) */  && mem_ctrl.wxd  && mem_ctrl.vec, mem_waddr,vector_dcache_bypass_data))
 */
  val vector_bypass_sources = IndexedSeq(
    (Bool(true), UInt(0), UInt(0)), // treat reading x0 as a bypass
    ((ex_reg_valid || (!ex_ctrl.mem && vector_unit.io.resp.valid))  && ex_ctrl.wxd  && ex_ctrl.vec, ex_waddr, v_mem_reg_wdata),
    (((mem_reg_valid && !ctrl_stalld) || (wb_reg_valid && !ctrl_stalld))  && mem_ctrl.wxd && !mem_ctrl.mem  && mem_ctrl.vec, mem_waddr, v_wb_reg_wdata),
    (((mem_reg_valid && !ctrl_stalld) || (wb_reg_valid && !ctrl_stalld))  && mem_ctrl.wxd  && mem_ctrl.vec, mem_waddr,vector_dcache_bypass_data))


  val vector_id_bypass_src = id_vraddr.map(vraddr => vector_bypass_sources.map(s => s._1 && s._2 === vraddr))


  // execute stage
  val bypass_mux = bypass_sources.map(_._3)
  val ex_reg_rs_bypass = Reg(Vec(id_raddr.size, Bool()))
  val ex_reg_rs_lsb = Reg(Vec(id_raddr.size, UInt(width = log2Ceil(bypass_sources.size))))
  val ex_reg_rs_msb = Reg(Vec(id_raddr.size, UInt()))
  val ex_rs = for (i <- 0 until id_raddr.size)
    yield Mux(ex_reg_rs_bypass(i), bypass_mux(ex_reg_rs_lsb(i)), Cat(ex_reg_rs_msb(i), ex_reg_rs_lsb(i)))

  //bypass vector cache zazad begins
  val vector_bypass_mux = vector_bypass_sources.map(_._3)
  val v_ex_reg_rs_bypass = Reg(Vec(id_vraddr.size, Bool()))
  val v_ex_reg_rs_lsb = Reg(Vec(id_vraddr.size, UInt(width = log2Ceil(vector_bypass_sources.size))))
  val v_ex_reg_rs_msb = Reg(Vec(id_vraddr.size, UInt()))
  val ex_vrs = for (i <- 0 until id_vraddr.size)
    yield Mux(v_ex_reg_rs_bypass(i), vector_bypass_mux(v_ex_reg_rs_lsb(i)), Cat(v_ex_reg_rs_msb(i), v_ex_reg_rs_lsb(i)))

/*  printf("[checkcachecounter]~~~~~~~~~~~~~~~~~ [ ex_reg_valid %b && ex_ctrl.wxd %b && ex_ctrl.vec %b , ex_waddr %x, v_mem_reg_wdata %x] [(mem_reg_valid %b && !ctrl_stalld %b) && mem_ctrl.wxd %b && !mem_ctrl.mem %b && mem_ctrl.vec %b, mem_waddr %x, v_wb_reg_wdata %x] [(mem_reg_valid %b && !ctrl_stalld %b) && mem_ctrl.wxd %b && mem_ctrl.vec %b , mem_waddr %x ,vector_dcache_bypass_data %x)] \n", ex_reg_valid, ex_ctrl.wxd, ex_ctrl.vec, ex_waddr, v_mem_reg_wdata, mem_reg_valid, !ctrl_stalld, mem_ctrl.wxd, !mem_ctrl.mem , mem_ctrl.vec, mem_waddr, v_wb_reg_wdata, mem_reg_valid, !ctrl_stalld, mem_ctrl.wxd , mem_ctrl.vec, mem_waddr, vector_dcache_bypass_data)
  printf("[checkcachecounter]~~~~~~~~~~~~~~~~~ v_ex_reg_rs_bypass(0) %x (1) %x  vector_bypass_mux(v_ex_reg_rs_lsb(0)) %x (1) %x  Cat(v_ex_reg_rs_msb(0) %x, v_ex_reg_rs_lsb(0) %x) %x  (1) %x , (1) %x => %x\n", v_ex_reg_rs_bypass(0), v_ex_reg_rs_bypass(1), vector_bypass_mux(v_ex_reg_rs_lsb(0)),v_ex_reg_rs_msb(0) , v_ex_reg_rs_lsb(0) , Cat(v_ex_reg_rs_msb(0), v_ex_reg_rs_lsb(0)), v_ex_reg_rs_msb(1), v_ex_reg_rs_lsb(1), Cat(v_ex_reg_rs_msb(1), v_ex_reg_rs_lsb(1)))*/
  val ex_reg_rss = Reg(Vec(id_raddr.size, UInt()))
  val v_ex_reg_rss = Reg(Vec(id_vraddr.size, UInt()))
  val temp_ex_vrs = for (i <- 0 until id_raddr.size) yield ex_reg_rss(i)
 val temp_ex_imm = ImmGen(ex_ctrl.sel_imm, ex_reg_inst)

  val ex_imm = Mux( ex_ctrl.mem && ex_ctrl.vec && isRead(ex_ctrl.mem_cmd) , Cat(UInt(1<<27)(26,0),ex_reg_inst(31,27)).asSInt , Mux( ex_ctrl.mem && ex_ctrl.vec && isWrite(ex_ctrl.mem_cmd), Cat(UInt(1<<27)(26,0),ex_reg_inst(11,7)).asSInt, temp_ex_imm))


  val ex_op1 = MuxLookup(ex_ctrl.sel_alu1, SInt(0), Seq(
    A1_RS1 -> ex_rs(0).asSInt,
    A1_PC -> ex_reg_pc.asSInt))
  val ex_op2 = MuxLookup(ex_ctrl.sel_alu2, SInt(0), Seq(
    A2_RS2 -> ex_rs(1).asSInt,
    A2_IMM -> ex_imm,
    A2_SIZE -> Mux(ex_reg_rvc, SInt(2), SInt(4))))

  val alu = Module(new ALU)
  alu.io.dw := ex_ctrl.alu_dw
  alu.io.fn := ex_ctrl.alu_fn
  alu.io.in2 := ex_op2.asUInt
  alu.io.in1 := ex_op1.asUInt
  printf("[checkcachecounter] alu.io.dw %x  alu.io.fn %x alu.io.in2 %x alu.io.in1 %x  [%x %x %x %x]  ex_op1 %x ,ex_ctrl.sel_alu1 %x, ex_rs(0).asSInt %x, ex_reg_pc.asSInt %x, ex_op2 %x, ex_ctrl.sel_alu2 %x, ex_rs(1).asSInt %x , ex_imm %x, ex_reg_rvc %x\n", alu.io.dw, alu.io.fn, alu.io.in2, alu.io.in1, ex_ctrl.alu_dw, ex_ctrl.alu_fn, ex_op2.asUInt, ex_op1.asUInt, ex_op1 ,ex_ctrl.sel_alu1, ex_rs(0).asSInt, ex_reg_pc.asSInt, ex_op2, ex_ctrl.sel_alu2, ex_rs(1).asSInt, ex_imm, ex_reg_rvc)
  //Adding VFPU here
  // val vector_unit = Module (new Vector_Unit)
  vector_unit.io.req.valid    := ex_reg_valid && ex_ctrl.vec && !ex_ctrl.mem
  vector_unit.io.req.bits.in1      := ex_vrs(0)
  vector_unit.io.req.bits.in2      := ex_vrs(1)
  vector_unit.io.req.bits.in3      := ex_vrs(2)
  vector_unit.io.req.bits.fn       := ex_ctrl.alu_fn
  vector_unit.io.req.bits.tag := ex_waddr
  vector_unit.io.req.bits.number_of_elements := number_of_elements
  vector_unit.io.req.bits.vector := (ex_ctrl.vec || mem_ctrl.vec || wb_ctrl.vec)
  v_alu_out := vector_unit.io.resp.bits.data

  // multiplier and divider
  val div = Module(new MulDiv(if (pipelinedMul) mulDivParams.copy(mulUnroll = 0) else mulDivParams, width = xLen))
  div.io.req.valid := ex_reg_valid && ex_ctrl.div
  div.io.req.bits.dw := ex_ctrl.alu_dw
  div.io.req.bits.fn := ex_ctrl.alu_fn
  div.io.req.bits.in1 := ex_rs(0)
  div.io.req.bits.in2 := ex_rs(1)
  div.io.req.bits.tag := ex_waddr
  val mul = pipelinedMul.option {
    val m = Module(new PipelinedMultiplier(xLen, 2))
    m.io.req.valid := ex_reg_valid && ex_ctrl.mul
    m.io.req.bits := div.io.req.bits
    m
  }

  ex_reg_valid := !ctrl_killd
  ex_reg_valid_vlsd_vssd := !ctrl_killd_vlsd_vssd
  ex_reg_replay := !take_pc && ibuf.io.inst(0).valid && ibuf.io.inst(0).bits.replay
  ex_reg_xcpt := !ctrl_killd && id_xcpt
  ex_reg_xcpt_interrupt := !take_pc && ibuf.io.inst(0).valid && csr.io.interrupt

  val ex_raddr2 = Reg(UInt(5.W))
  prev_ctrl_stalld := ctrl_stalld
  prev_dmem_resp_valid := io.dmem.resp.valid
  when (!ctrl_killd) {
    ex_ctrl := id_ctrl
    ex_raddr2 := id_raddr2
    ex_reg_rvc := ibuf.io.inst(0).bits.rvc
    ex_ctrl.csr := id_csr
    when (id_fence_next) { id_reg_fence := true }
    when (id_xcpt) { // pass PC down ALU writeback pipeline for badaddr
      ex_ctrl.alu_fn := ALU.FN_ADD
      ex_ctrl.alu_dw := DW_XPR
      ex_ctrl.sel_alu1 := A1_RS1 // badaddr := instruction
      ex_ctrl.sel_alu2 := A2_ZERO
      when (id_xcpt1.asUInt.orR) { // badaddr := PC+2
        ex_ctrl.sel_alu1 := A1_PC
        ex_ctrl.sel_alu2 := A2_SIZE
        ex_reg_rvc := true
      }
      when (bpu.io.xcpt_if || id_xcpt0.asUInt.orR) { // badaddr := PC
        ex_ctrl.sel_alu1 := A1_PC
        ex_ctrl.sel_alu2 := A2_ZERO
      }
    }
    ex_reg_flush_pipe := id_ctrl.fence_i || id_csr_flush
    ex_reg_load_use := id_load_use
    when (id_sfence) {
      ex_ctrl.mem_type := Cat(id_raddr2 =/= UInt(0), id_raddr1 =/= UInt(0))
    }
    for (i <- 0 until id_raddr.size) {
      val do_bypass = id_bypass_src(i).reduce(_||_)
      val bypass_src = PriorityEncoder(id_bypass_src(i))
      val test_next = id_bypass_src(i)
      ex_reg_rs_bypass(i) := do_bypass
      ex_reg_rs_lsb(i) := bypass_src
//      printf("[checkcachecounter]BYYYYYPASSSSS do_bypass %b ex_reg_rs_lsb(0) %x , ex_reg_rs_lsb(1) %x ,ex_reg_rs_msb(0) %x, ex_reg_rs_msb(1) %x, id_rs(0) %x, id_rs(1) %x \n", do_bypass, ex_reg_rs_lsb(0), ex_reg_rs_lsb(1),ex_reg_rs_msb(0), ex_reg_rs_msb(1), id_rs(0), id_rs(1))
      when (id_ren(i) && !do_bypass) {
        ex_reg_rs_lsb(i) := id_rs(i)(log2Ceil(bypass_sources.size)-1, 0)
        ex_reg_rs_msb(i) := id_rs(i) >> log2Ceil(bypass_sources.size)
        //zazad begins come here to print
        ex_reg_rss(i) := id_vrs(i)
        //zazad ends
      }
    }
    //bypass vector cache zazad begins
    for (i <- 0 until id_vraddr.size ) {                                                    
      val v_do_bypass = vector_id_bypass_src(i).reduce(_||_)                                     
      val v_bypass_src = PriorityEncoder(vector_id_bypass_src(i))                                 
      val v_test_next = vector_id_bypass_src(i)                                                   
      v_ex_reg_rs_bypass(i) := v_do_bypass                                                         
      v_ex_reg_rs_lsb(i) := v_bypass_src                                                           
      when (id_vren(i) && !v_do_bypass) {
        v_ex_reg_rs_lsb(i) := id_vrs(i)(log2Ceil(vector_bypass_sources.size)-1, 0)              
        v_ex_reg_rs_msb(i) := id_vrs(i) >> log2Ceil(vector_bypass_sources.size)
        v_ex_reg_rss(i) := id_vrs(i)
      }

    }

    //zazad ends

    when (id_illegal_insn) {
      val inst = Mux(ibuf.io.inst(0).bits.rvc, id_raw_inst(0)(15, 0), id_raw_inst(0))
      ex_reg_rs_bypass(0) := false
      ex_reg_rs_lsb(0) := inst(log2Ceil(bypass_sources.size)-1, 0)
      ex_reg_rs_msb(0) := inst >> log2Ceil(bypass_sources.size)
    }
  }.elsewhen (ctrl_stalld && ex_ctrl.vec && ex_ctrl.mem) ///////////////to solve conflict between databypassing and sequencer (stalling pipeline)
  {
    v_ex_reg_rs_bypass(0) := false
    v_ex_reg_rs_bypass(1) := false
    /*when (!prev_ctrl_stalld){
        for (i <- 0 until id_vraddr.size) {//in the previous cycle where this instruction was decoded, we did data bypasing since we were writting into the reg it read. Now, the data is ready in the reg we can pass it to ex-vrs instead of the bypass resource (wrong data) ==> Thsi problem does not exist for scalar whcih passes ex in one cycle, we stay there longer so we need to manage it ourselves
          v_ex_reg_rs_lsb(i) := id_vrs(i)(log2Ceil(vector_bypass_sources.size)-1, 0)                                                                                                                      
          v_ex_reg_rs_msb(i) := id_vrs(i) >> log2Ceil(vector_bypass_sources.size)                                                                                                                         
          ex_reg_rss(i) := id_vrs(i)                                                                                                                                                                      
        }
        printf("[checkcachecounter]  inside new block v_ex_reg_rs_bypass %b %b id_vrs %x %x\n", v_ex_reg_rs_bypass(0), v_ex_reg_rs_bypass(1),id_vrs(0) >> log2Ceil(vector_bypass_sources.size), id_vrs(1) >> log2Ceil(vector_bypass_sources.size))
    }*/
  }///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  when (!ctrl_killd || csr.io.interrupt || ibuf.io.inst(0).bits.replay) {
    ex_reg_cause := id_cause
    ex_reg_inst := id_inst(0)
    ex_reg_raw_inst := id_raw_inst(0)
    ex_reg_pc := ibuf.io.pc
    ex_reg_btb_resp := ibuf.io.btb_resp
  }


  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  val stored_sride_vld = Reg(UInt(32.W))
  val stored_sride_vsd = Reg(UInt(32.W))
  val locks_vld = Reg(init=Bool(false))
  val locks_vsd = Reg(init=Bool(false))
  val stride_vld_array = Wire(UInt(256.W))
  val reg_stride_vld_array = Reg(UInt(256.W))
  reg_stride_vld_array := stride_vld_array
  when (!prev_ctrl_stalld){
    stride_vld_array := ex_vrs(1)
  }.otherwise{
    stride_vld_array := reg_stride_vld_array
  }

  val stride_vsd_array = Wire(UInt(256.W))
  val reg_stride_vsd_array = Reg(UInt(256.W))
  reg_stride_vsd_array := stride_vsd_array
  when (!prev_ctrl_stalld){
    stride_vsd_array := ex_vrs(0)
  }.otherwise{
    stride_vsd_array := reg_stride_vsd_array
  }
  val stride_vld = Wire(UInt(32.W))
  stride_vld := Mux(ex_raddr2 === UInt(0) , UInt(1), Mux(locks_vld,stored_sride_vld,ex_rs(1)))
  val stride_vsd = Wire(UInt(32.W))
  stride_vsd := Mux(ex_raddr2 === UInt(0) , UInt(1), Mux(locks_vsd,stored_sride_vsd,ex_rs(1)))

  when (ex_reg_valid && ex_ctrl.vec && ex_ctrl.mem && ex_ctrl.wxd && cnt_cache === UInt(0)){
    stored_sride_vld := ex_rs(1)
  }
  when (ex_reg_valid && ex_ctrl.vec && ex_ctrl.mem && !ex_ctrl.wxd && cnt_cache_vsd === UInt(0)){
    stored_sride_vsd := ex_rs(1)
  
}

  val unit_stride_offset_bit_extraction = Wire(UInt(8.W))
  val unit_stride_extracted_bits = Wire(UInt(256.W))
  val unit_stride_concatenated = Wire(UInt(256.W))
  val prev_address_offset = Reg(init = UInt(0,32.W))
  val prev_address_offset_vsd = Reg(init = UInt(0,32.W))
  val vrf_mem_value = Wire(UInt(256.W))
  val temp_vrf_mem_value = Reg(init = UInt(0,256.W))
  val ttemp_vrf_mem_value = Reg(init = UInt(0,256.W))
  val cache_s1_valid_value = Reg(Bool())
  val offset_bit_extraction = Wire(UInt(8.W))
  val offset_vrf_reg = Wire(UInt(8.W))
  val extracted_bits = Wire(UInt(32.W))
  val tprevious_vector_val = Wire(UInt(256.W))
  //val extracted_bits = Vec(number_of_elements, UInt(16.W))
  //stride one should be separated

  ///////////////////////////////////////////////////VLD///////////////////////////////////////////////////////////////////////////////////////
  when(wb_xcpt_temp){
    cnt_cache := UInt(0)
  }.elsewhen (wb_ctrl.mem && wb_ctrl.vec && wb_ctrl.wxd && (!prev_dmem_resp_valid && io.dmem.resp.valid) && (io.dmem.resp.bits.return_addr === io.dmem.req.bits.addr) && (stride_vld =/= UInt(1)) && !wb_ctrl.vec_scalar ) {
    
         offset_bit_extraction := io.dmem.resp.bits.return_addr(4,0) * UInt(8)
         offset_vrf_reg := cnt_cache * UInt(32)
         //extracted_bits(cnt_cache) := ((UInt(1) << UInt(16)) - UInt(1)) & (io.dmem.resp.bits.DcacheCpu_data >> offset_bit_extraction)
         //temp_vrf_mem_value := Cat(extracted_bits(15),extracted_bits(14),....., extracted_bits(0))
         extracted_bits := ((UInt(1) << UInt(32)) - UInt(1)) & (io.dmem.resp.bits.DcacheCpu_data >> offset_bit_extraction)
         when       (cnt_cache === UInt(0)) {temp_vrf_mem_value  := extracted_bits
         }.elsewhen (cnt_cache === UInt(1)) {temp_vrf_mem_value  := Cat(extracted_bits,temp_vrf_mem_value(31,0))//make them shift and also the left habd side bunch of registers being concateadted at end
         }.elsewhen (cnt_cache === UInt(2)) {temp_vrf_mem_value  := Cat(extracted_bits,temp_vrf_mem_value(63,0))
         }.elsewhen (cnt_cache === UInt(3)) {temp_vrf_mem_value  := Cat(extracted_bits,temp_vrf_mem_value(95,0))
         }.elsewhen (cnt_cache === UInt(4)) {temp_vrf_mem_value  := Cat(extracted_bits,temp_vrf_mem_value(127,0))
         }.elsewhen (cnt_cache === UInt(5)) {temp_vrf_mem_value  := Cat(extracted_bits,temp_vrf_mem_value(159,0))
         }.elsewhen (cnt_cache === UInt(6)) {temp_vrf_mem_value  := Cat(extracted_bits,temp_vrf_mem_value(191,0))
         }.elsewhen (cnt_cache === UInt(7)) {temp_vrf_mem_value  := Cat(extracted_bits,temp_vrf_mem_value(223,0))}

         //not sure where it is used, may need to be removed
         tprevious_vector_val := ((UInt(1) << offset_vrf_reg) - UInt(1)) & ttemp_vrf_mem_value
         when (cnt_cache == UInt(0)){
           ttemp_vrf_mem_value := extracted_bits
         }.otherwise{
           ttemp_vrf_mem_value := Cat(extracted_bits, tprevious_vector_val )
         }

         when (cnt_cache === number_of_elements - UInt(1)){
           when       (cnt_cache === UInt(0)) {vrf_mem_value  := extracted_bits
           }.elsewhen (cnt_cache === UInt(1)) {vrf_mem_value  := Cat(extracted_bits,temp_vrf_mem_value(31,0))//make them shift and also the left habd side bunch of registers being concateadted at end
           }.elsewhen (cnt_cache === UInt(2)) {vrf_mem_value  := Cat(extracted_bits,temp_vrf_mem_value(63,0))
           }.elsewhen (cnt_cache === UInt(3)) {vrf_mem_value  := Cat(extracted_bits,temp_vrf_mem_value(95,0))
           }.elsewhen (cnt_cache === UInt(4)) {vrf_mem_value  := Cat(extracted_bits,temp_vrf_mem_value(127,0))
           }.elsewhen (cnt_cache === UInt(5)) {vrf_mem_value  := Cat(extracted_bits,temp_vrf_mem_value(159,0))
           }.elsewhen (cnt_cache === UInt(6)) {vrf_mem_value  := Cat(extracted_bits,temp_vrf_mem_value(191,0))
           }.elsewhen (cnt_cache === UInt(7)) {vrf_mem_value  := Cat(extracted_bits,temp_vrf_mem_value(223,0))}
           printf("[checkcachecounter] in the fiiiiiiiiiiiiiiiiirst cnt_cache %d  vrf_val %x \n", cnt_cache, vrf_mem_value)
         }.otherwise{
           vrf_mem_value := temp_vrf_mem_value
           printf("[checkcachecounter] in the secoooooooooooooooond cnt_cache %d  vrf_val %x \n", cnt_cache, vrf_mem_value)
         }

    
         when (cnt_cache === number_of_elements - UInt(1)){
           cnt_cache := UInt(0)
         }.otherwise{
           cnt_cache := cnt_cache + UInt(1)
         }

         cache_s1_valid_value := Bool(true)
         //printf("[checkcachecounter] i am inside first part\n")

  }.otherwise {

         printf("[checkcachecounter] ***************** either vec_scalar or unit stride cache return data is %x  addresss %x\n", io.dmem.resp.bits.DcacheCpu_data, io.dmem.resp.bits.return_addr)
         //take care of this part, if we have unit stride we shouldn't return the entire half line fetched from the caceh
         //we should take care of the index (which 16 bit we should start from/32/...) +4 in address mean we don't need the firt two 16-bit elements
         //so extract the correct bits even for unit stride
         //also there is a case where even for unit stride we have part of the data in another half line based on the index we start from
         //take care of this ==> sending two address to cache and fetch partial data from each one

         /*    val unit_stride_offset_bit_extraction = Wire(UInt(8.W))
          val unit_stride_extracted_bits = Wire(UInt(256.W))
          val unit_stride_concatenated = Wire(UInt(256.W))
          val prev_address_offset = Reg(init = UInt(0,32.W)) */
          unit_stride_offset_bit_extraction := io.dmem.resp.bits.return_addr(4,0) * UInt(8)
            unit_stride_extracted_bits := io.dmem.resp.bits.DcacheCpu_data //>> unit_stride_offset_bit_extraction

         when(wb_ctrl.mem && wb_ctrl.vec && wb_ctrl.wxd  && (!prev_dmem_resp_valid && io.dmem.resp.valid) && (io.dmem.resp.bits.return_addr === io.dmem.req.bits.addr) && (stride_vld === UInt(1)) && !wb_ctrl.vec_scalar)
          {//it is unit stride store
            when (cnt_cache === UInt(1))
            {
              cnt_cache := UInt(0)
              cache_s1_valid_value := Bool (false)
              printf("[checkcachecounter]@@@@@@@@@@@@@@@@inprocessor vlsd-unit 1-checkpoint cnt_cache %x cache_s1_valid_value %b\n", cnt_cache, cache_s1_valid_value)
            }.otherwise {
              cache_s1_valid_value := Bool (true)
              cnt_cache := cnt_cache + UInt(1)
              printf("[checkcachecounter]@@@@@@@@@@@@@@@@inprocessor vlsd-unit 2-checkpoint cnt_cache %x  cache_s1_valid_value %b\n", cnt_cache, cache_s1_valid_value)
            }

            when (cnt_cache === UInt(0)){//handles first cycle of unit stride access
              ttemp_vrf_mem_value := unit_stride_extracted_bits
              prev_address_offset := io.dmem.resp.bits.return_addr(4,0)
              printf("[checkcachecounter]@@@@@@@@@@@@@@@@inprocessor vlsd-unit 3-checkpoint cnt_cache %x ttemp_vrf_mem_value %x prev_address_offset %x io.dmem.resp.bits.return_addr %x unit_stride_extracted_bits %x\n", cnt_cache, ttemp_vrf_mem_value, prev_address_offset, io.dmem.resp.bits.return_addr,unit_stride_extracted_bits)
            }
            //handling second cycle of unit stride memory access
            when       (prev_address_offset === UInt(0)) {unit_stride_concatenated := ttemp_vrf_mem_value
            }.elsewhen (prev_address_offset === UInt(1)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(7,0),ttemp_vrf_mem_value(255,8))
            }.elsewhen (prev_address_offset === UInt(2)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(15,0),ttemp_vrf_mem_value(255,16))
            }.elsewhen (prev_address_offset === UInt(3)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(23,0),ttemp_vrf_mem_value(255,24))
            }.elsewhen (prev_address_offset === UInt(4)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(31,0),ttemp_vrf_mem_value(255,32))
            }.elsewhen (prev_address_offset === UInt(5)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(39,0),ttemp_vrf_mem_value(255,40))
            }.elsewhen (prev_address_offset === UInt(6)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(47,0),ttemp_vrf_mem_value(255,48))
            }.elsewhen (prev_address_offset === UInt(7)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(55,0),ttemp_vrf_mem_value(255,56))
            }.elsewhen (prev_address_offset === UInt(8)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(63,0),ttemp_vrf_mem_value(255,64))
            }.elsewhen (prev_address_offset === UInt(9)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(71,0),ttemp_vrf_mem_value(255,72))
            }.elsewhen (prev_address_offset === UInt(10)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(79,0),ttemp_vrf_mem_value(255,80))
            }.elsewhen (prev_address_offset === UInt(11)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(87,0),ttemp_vrf_mem_value(255,88))
            }.elsewhen (prev_address_offset === UInt(12)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(95,0),ttemp_vrf_mem_value(255,96))
            }.elsewhen (prev_address_offset === UInt(13)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(103,0),ttemp_vrf_mem_value(255,104))
            }.elsewhen (prev_address_offset === UInt(14)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(111,0),ttemp_vrf_mem_value(255,112))
            }.elsewhen (prev_address_offset === UInt(15)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(119,0),ttemp_vrf_mem_value(255,120))
            }.elsewhen (prev_address_offset === UInt(16)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(127,0),ttemp_vrf_mem_value(255,128))
            }.elsewhen (prev_address_offset === UInt(17)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(135,0),ttemp_vrf_mem_value(255,136))
            }.elsewhen (prev_address_offset === UInt(18)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(143,0),ttemp_vrf_mem_value(255,144))
            }.elsewhen (prev_address_offset === UInt(19)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(151,0),ttemp_vrf_mem_value(255,152))
            }.elsewhen (prev_address_offset === UInt(20)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(159,0),ttemp_vrf_mem_value(255,160))
            }.elsewhen (prev_address_offset === UInt(21)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(167,0),ttemp_vrf_mem_value(255,168))
            }.elsewhen (prev_address_offset === UInt(22)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(175,0),ttemp_vrf_mem_value(255,176))
            }.elsewhen (prev_address_offset === UInt(23)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(183,0),ttemp_vrf_mem_value(255,184))
            }.elsewhen (prev_address_offset === UInt(24)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(191,0),ttemp_vrf_mem_value(255,192))
            }.elsewhen (prev_address_offset === UInt(25)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(199,0),ttemp_vrf_mem_value(255,200))
            }.elsewhen (prev_address_offset === UInt(26)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(207,0),ttemp_vrf_mem_value(255,208))
            }.elsewhen (prev_address_offset === UInt(27)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(215,0),ttemp_vrf_mem_value(255,216))
            }.elsewhen (prev_address_offset === UInt(28)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(223,0),ttemp_vrf_mem_value(255,224))
            }.elsewhen (prev_address_offset === UInt(29)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(231,0),ttemp_vrf_mem_value(255,232))
            }.elsewhen (prev_address_offset === UInt(30)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(239,0),ttemp_vrf_mem_value(255,240))
            }.elsewhen (prev_address_offset === UInt(31)) {unit_stride_concatenated := Cat(unit_stride_extracted_bits(247,0),ttemp_vrf_mem_value(255,248))}

          }
          printf("[checkcachecounter]@@@@@@@@@@@@@@@@inprocessor vlsd-unit 4-checkpoint cnt_cache %x prev_address_offset %x ttemp_vrf_mem_value %x unit_stride_extracted_bits %x unit_stride_concatenated %x \n", cnt_cache,prev_address_offset, ttemp_vrf_mem_value, unit_stride_extracted_bits, unit_stride_concatenated)

          when (wb_ctrl.vec_scalar){
            cache_s1_valid_value := Bool (false)
          }
          vrf_mem_value := Mux(wb_ctrl.vec_scalar,Cat(io.dmem.resp.bits.DcacheCpu_data(31,0),io.dmem.resp.bits.DcacheCpu_data(31,0),io.dmem.resp.bits.DcacheCpu_data(31,0),io.dmem.resp.bits.DcacheCpu_data(31,0),io.dmem.resp.bits.DcacheCpu_data(31,0),io.dmem.resp.bits.DcacheCpu_data(31,0),io.dmem.resp.bits.DcacheCpu_data(31,0),io.dmem.resp.bits.DcacheCpu_data(31,0)), Mux(stride_vld === UInt(1),/*io.dmem.resp.bits.DcacheCpu_data*//*unit_stride_extracted_bits*/unit_stride_concatenated, temp_vrf_mem_value))



          when(id_inst(0) === "h0007d20b".U || ex_reg_inst ===  "h0007d20b".U || mem_reg_inst === "h0007d20b".U || wb_reg_inst === "h0007d20b".U ||id_inst(0) === "h00971073".U || ex_reg_inst ===  "h00971073".U || mem_reg_inst === "h00971073".U || wb_reg_inst === "h00971073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2403500b".U || ex_reg_inst === "h2403500b".U || mem_reg_inst === "h2403500b".U || wb_reg_inst === "h2403500b".U ||id_inst(0) === "h00991073".U || ex_reg_inst ===  "h00991073".U || mem_reg_inst === "h00991073".U || wb_reg_inst === "h00991073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U || id_inst(0) === "h240e500b".U || ex_reg_inst ===  "h240e500b".U || mem_reg_inst === "h240e500b".U || wb_reg_inst === "h240e500b".U || id_inst(0) === "h240b500b".U || ex_reg_inst ===  "h240b500b".U || mem_reg_inst === "h240b500b".U || wb_reg_inst === "h240b500b".U ||id_inst(0) === "h0007d20b".U || ex_reg_inst ===  "h0007d20b".U || mem_reg_inst === "h0007d20b".U || wb_reg_inst === "h0007d20b".U ||id_inst(0) === "h00971073".U || ex_reg_inst ===  "h00971073".U || mem_reg_inst === "h00971073".U || wb_reg_inst === "h00971073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2403500b".U || ex_reg_inst === "h2403500b".U || mem_reg_inst === "h2403500b".U || wb_reg_inst === "h2403500b".U ||id_inst(0) === "h00991073".U || ex_reg_inst ===  "h00991073".U || mem_reg_inst === "h00991073".U || wb_reg_inst === "h00991073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U || id_inst(0) === "h000fc58b".U || ex_reg_inst === "h000fc58b".U || mem_reg_inst === "h000fc58b".U || wb_reg_inst === "h000fc58b".U || id_inst(0) === "h0005560b".U || ex_reg_inst === "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U  || id_inst(0) === "h000ed68b".U || ex_reg_inst === "h000ed68b".U || mem_reg_inst === "h000ed68b".U || wb_reg_inst === "h000ed68b".U || id_inst(0) === "h02d9570b".U || ex_reg_inst === "h02d9570b".U || mem_reg_inst === "h02d9570b".U || wb_reg_inst === "h02d9570b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h76d9500b".U || ex_reg_inst === "h76d9500b".U || mem_reg_inst === "h76d9500b".U || wb_reg_inst === "h76d9500b".U){


      printf("[checkcachecounterfuncs]:))))))))))))))  vrf_mem_value  %x  io.dmem.resp %x  vec_scalar %b   stride1 %b temp_vrf_mem_value %x   unit_stride_concatenated %x\n", vrf_mem_value, io.dmem.resp.bits.DcacheCpu_data, wb_ctrl.vec_scalar, stride_vld === UInt(1), temp_vrf_mem_value, unit_stride_concatenated)
    }
   // printf("[checkcachecounter] i am inside first part\n")
  }


  val cache_s1_valid_value_vsd = Reg(Bool())

  /*when(wb_xcpt && (wb_cause ===  "hf".U))
  {
    cnt_cache_vsd := UInt(0)
  }.elsewhen (wb_ctrl.mem && wb_ctrl.vec && !wb_ctrl.wxd  && io.dmem.resp.valid && (io.dmem.resp.bits.return_addr === io.dmem.req.bits.addr) && (stride_vsd =/= UInt(1)) && !wb_ctrl.vec_scalar) {
    
    when (cnt_cache_vsd === number_of_elements - UInt(1)){
      cnt_cache_vsd := UInt(0)
    }.otherwise{
      cnt_cache_vsd := cnt_cache_vsd + UInt(1)
    }

    cache_s1_valid_value_vsd := Bool(true)

  }.otherwise {

    when(wb_ctrl.mem && wb_ctrl.vec && !wb_ctrl.wxd  && io.dmem.resp.valid && (io.dmem.resp.bits.return_addr === io.dmem.req.bits.addr) && (stride_vsd === UInt(1)) && !wb_ctrl.vec_scalar){
      when (cnt_cache_vsd === UInt(1))
      {
        cnt_cache_vsd := UInt(0)
        cache_s1_valid_value_vsd := Bool (false)
        printf("[checkcachecounter]@@@@@@@@@@@@@@@@inprocessor vsd-unit 1-checkpoint cnt_cache %x cache_s1_valid_value %b\n", cnt_cache_vsd, cache_s1_valid_value_vsd)
      }.otherwise{
        cache_s1_valid_value_vsd := Bool (true)
        cnt_cache_vsd := cnt_cache_vsd + UInt(1)
        printf("[checkcachecounter]@@@@@@@@@@@@@@@@inprocessor vsd-unit 2-checkpoint cnt_cache %x  cache_s1_valid_value %b\n", cnt_cache_vsd, cache_s1_valid_value_vsd)
      }
    }.otherwise{
      cache_s1_valid_value_vsd := Bool (false)
    }
  }*/
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // replay inst in ex stage?
  val ex_pc_valid = ex_reg_valid || ex_reg_replay || ex_reg_xcpt_interrupt
  val wb_dcache_miss = wb_ctrl.mem && !io.dmem.resp.valid
  val replay_ex_structural = ex_ctrl.mem && !io.dmem.req.ready ||
  ex_ctrl.div && !div.io.req.ready
  //changed for dcache counter
  val replay_ex_load_use = (wb_dcache_miss && !wb_ctrl.vec) && ex_reg_load_use
  val replay_ex = ex_reg_replay || (ex_reg_valid && (replay_ex_structural || replay_ex_load_use))
  val ctrl_killx = take_pc_mem_wb || replay_ex || !ex_reg_valid
  // detect 2-cycle load-use delay for LB/LH/SC
  val ex_slow_bypass = ex_ctrl.mem_cmd === M_XSC || Vec(MT_B, MT_BU, MT_H, MT_HU).contains(ex_ctrl.mem_type)
  val ex_sfence = Bool(usingVM) && ex_ctrl.mem && ex_ctrl.mem_cmd === M_SFENCE

  val (ex_xcpt, ex_cause) = checkExceptions(List(
    (ex_reg_xcpt_interrupt || ex_reg_xcpt, ex_reg_cause)))

  val exCoverCauses = idCoverCauses
  coverExceptions(ex_xcpt, ex_cause, "EXECUTE", exCoverCauses)

  // memory stage
  val mem_pc_valid = mem_reg_valid || mem_reg_replay || mem_reg_xcpt_interrupt
  val mem_br_target = mem_reg_pc.asSInt +
    Mux(mem_ctrl.branch && mem_br_taken, ImmGen(IMM_SB, mem_reg_inst),
    Mux(mem_ctrl.jal, ImmGen(IMM_UJ, mem_reg_inst),
    Mux(mem_reg_rvc, SInt(2), SInt(4))))
  val mem_npc = (Mux(mem_ctrl.jalr || mem_reg_sfence, encodeVirtualAddress(mem_reg_wdata, mem_reg_wdata).asSInt, mem_br_target) & SInt(-2)).asUInt
  val mem_wrong_npc =
    Mux(ex_pc_valid, mem_npc =/= ex_reg_pc,
    Mux(ibuf.io.inst(0).valid || ibuf.io.imem.valid, mem_npc =/= ibuf.io.pc, Bool(true)))
  val mem_npc_misaligned = !csr.io.status.isa('c'-'a') && mem_npc(1) && !mem_reg_sfence
  val mem_int_wdata = Mux(!mem_reg_xcpt && (mem_ctrl.jalr ^ mem_npc_misaligned), mem_br_target, mem_reg_wdata.asSInt).asUInt
  val mem_cfi = mem_ctrl.branch || mem_ctrl.jalr || mem_ctrl.jal
  val v_mem_int_wdata = v_mem_reg_wdata
  val mem_cfi_taken = (mem_ctrl.branch && mem_br_taken) || mem_ctrl.jalr || mem_ctrl.jal
  val mem_direction_misprediction = mem_ctrl.branch && mem_br_taken =/= (usingBTB && mem_reg_btb_resp.taken)
  val mem_misprediction = if (usingBTB) mem_wrong_npc else mem_cfi_taken
  take_pc_mem := mem_reg_valid && (mem_misprediction || mem_reg_sfence)

//  printf("[checkcachecounter] sscratch %x Branches mem_br_taken %b := alu.io.cmp_out %b mem_br_target %x = mem_reg_pc.asSInt %x + [1 %b %x 2 %b %x 3 %b]   mem_int_wdata %x [%b %x %x <= alu.io.out %x]  mem_cfi %b mem_cfi_taken %b mem_direction_misprediction %b mem_misprediction %b   \n", csr.io.sscratch, mem_br_taken, alu.io.cmp_out,mem_br_target,mem_reg_pc.asSInt, mem_ctrl.branch && mem_br_taken,  ImmGen(IMM_SB, mem_reg_inst), mem_ctrl.jal, ImmGen(IMM_UJ, mem_reg_inst), mem_reg_rvc, mem_int_wdata, !mem_reg_xcpt && (mem_ctrl.jalr ^ mem_npc_misaligned), mem_br_target, mem_reg_wdata.asSInt,alu.io.out, mem_cfi, mem_cfi_taken, mem_direction_misprediction, mem_misprediction)
  mem_reg_valid := !ctrl_killx
  mem_reg_replay := !take_pc_mem_wb && replay_ex
  mem_reg_xcpt := !ctrl_killx && ex_xcpt
  mem_reg_xcpt_interrupt := !take_pc_mem_wb && ex_reg_xcpt_interrupt

  // on pipeline flushes, cause mem_npc to hold the sequential npc, which
  // will drive the W-stage npc mux
  when (mem_reg_valid && mem_reg_flush_pipe) {
    mem_reg_sfence := false
  }.elsewhen (ex_pc_valid) {
    mem_ctrl := ex_ctrl
    mem_reg_rvc := ex_reg_rvc
    mem_reg_load := ex_ctrl.mem && isRead(ex_ctrl.mem_cmd)
    mem_reg_store := ex_ctrl.mem && isWrite(ex_ctrl.mem_cmd)
    mem_reg_sfence := ex_sfence
    mem_reg_btb_resp := ex_reg_btb_resp
    mem_reg_flush_pipe := ex_reg_flush_pipe
    mem_reg_slow_bypass := ex_slow_bypass

    mem_reg_cause := ex_cause
    mem_reg_inst := ex_reg_inst
    //printf("[checkcachecounter]^^^^^^^^^^^^ transfer ex %x to mem %x  \n", ex_reg_inst, mem_reg_inst)
    mem_reg_raw_inst := ex_reg_raw_inst
    mem_reg_pc := ex_reg_pc
    mem_reg_wdata := alu.io.out

    mem_br_taken := alu.io.cmp_out

    when (ex_ctrl.rxs2 && (ex_ctrl.mem || ex_ctrl.rocc || ex_sfence)) {
      val typ = Mux(ex_ctrl.rocc, log2Ceil(xLen/8).U, ex_ctrl.mem_type)
      mem_reg_rs2 := new StoreGen(typ, 0.U, ex_rs(1), coreDataBytes).data
    }

    when (ex_ctrl.rxs2 && ex_ctrl.mem && ex_ctrl.vec){
        v_mem_reg_rs2 := ex_vrs(2)
    }
   
    when (ex_ctrl.jalr && csr.io.status.debug) {
      // flush I$ on D-mode JALR to effect uncached fetch without D$ flush
      mem_ctrl.fence_i := true
      mem_reg_flush_pipe := true
    }
  }

  //for vector result from alu should be passed to the vector register file even if exe instruction is not valid
  when (vector_unit.io.resp.valid){
    v_mem_reg_wdata := v_alu_out
  }

  val mem_breakpoint = (mem_reg_load && bpu.io.xcpt_ld) || (mem_reg_store && bpu.io.xcpt_st)
  val mem_debug_breakpoint = (mem_reg_load && bpu.io.debug_ld) || (mem_reg_store && bpu.io.debug_st)
  val (mem_ldst_xcpt, mem_ldst_cause) = checkExceptions(List(
    (mem_debug_breakpoint, UInt(CSR.debugTriggerCause)),
    (mem_breakpoint,       UInt(Causes.breakpoint))))

  val test_mem_xcpt = (mem_reg_xcpt_interrupt || mem_reg_xcpt) || (mem_reg_valid && mem_npc_misaligned) || (mem_reg_valid && mem_ldst_xcpt)
  val (mem_xcpt, mem_cause) = checkExceptions(List(
    (mem_reg_xcpt_interrupt || mem_reg_xcpt, mem_reg_cause),
    (mem_reg_valid && mem_npc_misaligned,    UInt(Causes.misaligned_fetch)),
    (mem_reg_valid && mem_ldst_xcpt,         mem_ldst_cause)))

  val memCoverCauses = (exCoverCauses ++ List(
    (CSR.debugTriggerCause, "DEBUG_TRIGGER"),
    (Causes.breakpoint, "BREAKPOINT"),
    (Causes.misaligned_fetch, "MISALIGNED_FETCH")
  )).distinct
  coverExceptions(mem_xcpt, mem_cause, "MEMORY", memCoverCauses)

  val dcache_kill_mem = mem_reg_valid && mem_ctrl.wxd && io.dmem.replay_next // structural hazard on writeback port
  val fpu_kill_mem = mem_reg_valid && mem_ctrl.fp && io.fpu.nack_mem
  val replay_mem  = dcache_kill_mem || mem_reg_replay || fpu_kill_mem
  val killm_common = dcache_kill_mem || take_pc_wb || mem_reg_xcpt || !mem_reg_valid
  val killm_common_vlsd_vssd = dcache_kill_mem || take_pc_wb || mem_reg_xcpt
  div.io.kill := killm_common && Reg(next = div.io.req.fire())
  val ctrl_killm = killm_common || mem_xcpt || fpu_kill_mem

 //// printf ("[vectorcheckhazards] dcache_kill_mem %b replay_mem %b killm_common %b ctrl_killm %b take_pc_wb %b take_pc_mem %b  \n", dcache_kill_mem ,replay_mem,killm_common,ctrl_killm,take_pc_wb,take_pc_mem)
  // writeback stage
  wb_reg_valid := !ctrl_killm
  wb_reg_replay := replay_mem && !take_pc_wb
  wb_reg_xcpt := mem_xcpt && !take_pc_wb
  wb_reg_flush_pipe := !ctrl_killm && mem_reg_flush_pipe
  when (mem_pc_valid) {
    wb_ctrl := mem_ctrl
    wb_reg_sfence := mem_reg_sfence
    wb_reg_wdata := Mux(!mem_reg_xcpt && mem_ctrl.fp && mem_ctrl.wxd, io.fpu.toint_data, mem_int_wdata)
    //zazad begins
    v_wb_reg_wdata := v_mem_int_wdata
    //printf("[checkvectorlength]Rocket6  %x %x \n", v_wb_reg_wdata, v_mem_int_wdata) 
    //zazad ends
    when (mem_ctrl.rocc || mem_reg_sfence) {
      wb_reg_rs2 := mem_reg_rs2
    }
    wb_reg_cause := mem_cause
    wb_reg_inst := mem_reg_inst
    wb_reg_raw_inst := mem_reg_raw_inst
    wb_reg_pc := mem_reg_pc
  }

  val (wb_xcpt, wb_cause) = checkExceptions(List(
    (wb_reg_xcpt,  wb_reg_cause),
    ((wb_reg_valid || (cnt_cache_vsd =/= UInt(0) && wb_ctrl.vec)) && wb_ctrl.mem && io.dmem.s2_xcpt.ma.st, UInt(Causes.misaligned_store)),
    ((wb_reg_valid || (cnt_cache =/= UInt(0)     && wb_ctrl.vec)) && wb_ctrl.mem && io.dmem.s2_xcpt.ma.ld, UInt(Causes.misaligned_load)),
    ((wb_reg_valid || (cnt_cache_vsd =/= UInt(0) && wb_ctrl.vec)) && wb_ctrl.mem && io.dmem.s2_xcpt.pf.st, UInt(Causes.store_page_fault)),
    ((wb_reg_valid || (cnt_cache =/= UInt(0)     && wb_ctrl.vec)) && wb_ctrl.mem && io.dmem.s2_xcpt.pf.ld, UInt(Causes.load_page_fault)),
    ((wb_reg_valid || (cnt_cache_vsd =/= UInt(0) && wb_ctrl.vec)) && wb_ctrl.mem && io.dmem.s2_xcpt.ae.st, UInt(Causes.store_access)),
    ((wb_reg_valid || (cnt_cache =/= UInt(0)     && wb_ctrl.vec)) && wb_ctrl.mem && io.dmem.s2_xcpt.ae.ld, UInt(Causes.load_access))
  ))

  wb_xcpt_temp := wb_xcpt
  printf("[checkcachecounter]????? wb_xcpt conditions  %b %b %b %b %b %b  \n", (wb_reg_valid || (cnt_cache_vsd =/= UInt(0) && wb_ctrl.vec)) && wb_ctrl.mem && io.dmem.s2_xcpt.ma.st, (wb_reg_valid || (cnt_cache =/= UInt(0)     && wb_ctrl.vec)) && wb_ctrl.mem && io.dmem.s2_xcpt.ma.ld, (wb_reg_valid || (cnt_cache_vsd =/= UInt(0) && wb_ctrl.vec)) && wb_ctrl.mem && io.dmem.s2_xcpt.pf.st, (wb_reg_valid || (cnt_cache =/= UInt(0)     && wb_ctrl.vec)) && wb_ctrl.mem && io.dmem.s2_xcpt.pf.ld, (wb_reg_valid || (cnt_cache_vsd =/= UInt(0) && wb_ctrl.vec)) && wb_ctrl.mem && io.dmem.s2_xcpt.ae.st, (wb_reg_valid || (cnt_cache =/= UInt(0)     && wb_ctrl.vec)) && wb_ctrl.mem && io.dmem.s2_xcpt.ae.ld)

when(wb_xcpt /* && (wb_cause ===  "hf".U) */)
  {
    cnt_cache_vsd := UInt(0)
  }.elsewhen (wb_ctrl.mem && wb_ctrl.vec && !wb_ctrl.wxd  && io.dmem.resp.valid && (io.dmem.resp.bits.return_addr === io.dmem.req.bits.addr) && (stride_vsd =/= UInt(1)) && !wb_ctrl.vec_scalar) {
    
    when (cnt_cache_vsd === number_of_elements - UInt(1)){
      cnt_cache_vsd := UInt(0)
    }.otherwise{
      cnt_cache_vsd := cnt_cache_vsd + UInt(1)
    }

    cache_s1_valid_value_vsd := Bool(true)
   printf("[checkcachecounter]?????? scatter store   cnt_cache_vsd %x cache_s1_valid_value %b\n", cnt_cache_vsd, cache_s1_valid_value_vsd)
  }.otherwise {

    when(wb_ctrl.mem && wb_ctrl.vec && !wb_ctrl.wxd  && io.dmem.resp.valid && (io.dmem.resp.bits.return_addr === io.dmem.req.bits.addr) && (stride_vsd === UInt(1)) && !wb_ctrl.vec_scalar){
      when (cnt_cache_vsd === UInt(1))
      {
        cnt_cache_vsd := UInt(0)
        cache_s1_valid_value_vsd := Bool (false)
       printf("[checkcachecounter]?????? unit_stride cnt_cache_vsd %x cache_s1_valid_value %b\n", cnt_cache_vsd, cache_s1_valid_value_vsd)
      }.otherwise{
        cache_s1_valid_value_vsd := Bool (true)
        cnt_cache_vsd := cnt_cache_vsd + UInt(1)
        printf("[checkcachecounter]?????? unit_stride cnt_cache_vsd %x  cache_s1_valid_value %b\n", cnt_cache_vsd, cache_s1_valid_value_vsd)
      }
    }.otherwise{//vector scalar
      cache_s1_valid_value_vsd := Bool (false)
    }
  }


// instruction exception  printf("[checkcachecounter]FINAL check dcache exceptions  io.dmem.s2_xcpt.ma.st %b io.dmem.s2_xcpt.ma.ld %b io.dmem.s2_xcpt.pf.st %b io.dmem.s2_xcpt.pf.ld %b io.dmem.s2_xcpt.ae.st %b io.dmem.s2_xcpt.ae.ld %b wb_xcpt %b wb_reg_valid %b cnt_cache_vsd %x xnt_cache %x wb_ctrl.vec %b wb_ctrl.mem %b\n", io.dmem.s2_xcpt.ma.st, io.dmem.s2_xcpt.ma.ld, io.dmem.s2_xcpt.pf.st, io.dmem.s2_xcpt.pf.ld, io.dmem.s2_xcpt.ae.st, io.dmem.s2_xcpt.ae.ld, wb_xcpt, wb_reg_valid, cnt_cache_vsd, cnt_cache, wb_ctrl.vec, wb_ctrl.mem)

// instruction exception  printf("checkcachecounter]TAG wb_cause  %x %x %x %x %x %x  wb_reg_cause %x mem_cause %x mem_reg_cause %x %x %x ex_reg_cause %x ex_cause %x id_cause %x csr.io.interupt %x %x %x %x %x %x %x %x\n",  UInt(Causes.misaligned_store), UInt(Causes.misaligned_load), UInt(Causes.store_page_fault), UInt(Causes.load_page_fault), UInt(Causes.store_access), UInt(Causes.load_access), wb_reg_cause, mem_cause, mem_reg_cause, UInt(Causes.misaligned_fetch), mem_ldst_cause, ex_reg_cause, ex_cause, id_cause, csr.io.interrupt_cause,  UInt(CSR.debugTriggerCause), UInt(Causes.breakpoint),  UInt(Causes.fetch_page_fault), UInt(Causes.fetch_access), UInt(Causes.fetch_page_fault), UInt(Causes.fetch_access), UInt(Causes.illegal_instruction))


  val wbCoverCauses = List(
    (Causes.misaligned_store, "MISALIGNED_STORE"),
    (Causes.misaligned_load, "MISALIGNED_LOAD"),
    (Causes.store_page_fault, "STORE_PAGE_FAULT"),
    (Causes.load_page_fault, "LOAD_PAGE_FAULT"),
    (Causes.store_access, "STORE_ACCESS"),
    (Causes.load_access, "LOAD_ACCESS")
  )
  coverExceptions(wb_xcpt, wb_cause, "WRITEBACK", wbCoverCauses)

  val wb_wxd = wb_reg_valid && wb_ctrl.wxd
  //added for dcache counter
  val wb_set_sboard = wb_ctrl.div || (wb_dcache_miss && !wb_ctrl.vec) || wb_ctrl.rocc
  val replay_wb_common = (io.dmem.s2_nack && !wb_ctrl.vec) || wb_reg_replay
  val replay_wb_rocc = wb_reg_valid && wb_ctrl.rocc && !io.rocc.cmd.ready
  val replay_wb = replay_wb_common || replay_wb_rocc
  take_pc_wb := replay_wb || wb_xcpt || csr.io.eret || wb_reg_flush_pipe

  // writeback arbitration
  val dmem_resp_xpu = !io.dmem.resp.bits.tag(0).toBool
  val dmem_resp_fpu =  io.dmem.resp.bits.tag(0).toBool
  val dmem_resp_waddr = io.dmem.resp.bits.tag(5, 1)
  val dmem_resp_valid = io.dmem.resp.valid && io.dmem.resp.bits.has_data
  val dmem_resp_replay = dmem_resp_valid && io.dmem.resp.bits.replay

  div.io.resp.ready := !wb_wxd
  val ll_wdata = Wire(init = div.io.resp.bits.data)
  val ll_waddr = Wire(init = div.io.resp.bits.tag)
  val ll_wen = Wire(init = div.io.resp.fire())
  if (usingRoCC) {
    io.rocc.resp.ready := !wb_wxd
    when (io.rocc.resp.fire()) {
      div.io.resp.ready := Bool(false)
      ll_wdata := io.rocc.resp.bits.data
      ll_waddr := io.rocc.resp.bits.rd
      ll_wen := Bool(true)
    }
  }
  when (dmem_resp_replay && dmem_resp_xpu) {
    div.io.resp.ready := Bool(false)
    if (usingRoCC)
      io.rocc.resp.ready := Bool(false)
    ll_waddr := dmem_resp_waddr
    ll_wen := Bool(true)
  }


  val wb_valid = wb_reg_valid && !replay_wb && !wb_xcpt
  val wb_wen = wb_valid && wb_ctrl.wxd && !wb_ctrl.vec //added !wb_ctrl.vec
  val rf_wen = wb_wen || ll_wen
  val rf_waddr = Mux(ll_wen, ll_waddr, wb_waddr)
  //commentedtocheckcache
  val rf_wdata = Mux(dmem_resp_valid && dmem_resp_xpu, /* io.dmem.resp.bits.data(xLen-1, 0)*/ io.dmem.resp.bits.DcacheCpu_data(xLen-1,0) ,
                 Mux(ll_wen, ll_wdata,
                 Mux(wb_ctrl.csr =/= CSR.N, csr.io.rw.rdata,
                 Mux(wb_ctrl.mul, mul.map(_.io.resp.bits.data).getOrElse(wb_reg_wdata),
                 wb_reg_wdata))))
  when (rf_wen) { rf.write(rf_waddr, rf_wdata) }
printf("[checkcachecounter]SSSSSRRRRRFFFFFFF wb_valid %b wb_wen %b rf_wen %b rf_waddr %x rf_wdata %x [1 %b %x 2 %b %x 3 CSSSsssssssrrrrrr %b %x 4 %b %x 5 %x]  \n", wb_valid, wb_wen, rf_wen, rf_waddr, rf_wdata, dmem_resp_valid && dmem_resp_xpu, io.dmem.resp.bits.DcacheCpu_data(xLen-1,0) , ll_wen, ll_wdata, wb_ctrl.csr =/= CSR.N, csr.io.rw.rdata, wb_ctrl.mul, mul.map(_.io.resp.bits.data).getOrElse(wb_reg_wdata), wb_reg_wdata)

  val reg_cnt_cache =  Reg(init = UInt(0, log2Up(15+1)))
  val vwb_wen_gather = ((wb_valid && !wb_ctrl.mem) || ((wb_ctrl.mem) && (/*reg_cnt_cache*/ cnt_cache === number_of_elements - UInt(1)) && io.dmem.resp.valid && (ex_reg_inst === wb_reg_inst))) && wb_ctrl.wxd && wb_ctrl.vec
  val vwb_wen_unit_stride = (wb_valid || (wb_ctrl.mem && io.dmem.resp.valid)) && wb_ctrl.wxd && wb_ctrl.vec && (ex_reg_inst === wb_reg_inst)//consider the same patter nas scatter/gather here//If we have arithmetic instruction activate vrf write enable
  val vwb_wen_vec_scalar  = (wb_valid || (wb_ctrl.mem && io.dmem.resp.valid)) && wb_ctrl.wxd && wb_ctrl.vec 
  val vwb_wen = Mux((stride_vld === UInt(1) && cnt_cache === UInt(1)) && !wb_ctrl.scatter_gather && !wb_ctrl.vec_scalar, vwb_wen_unit_stride,Mux(wb_ctrl.vec_scalar && !wb_ctrl.scatter_gather, vwb_wen_vec_scalar,vwb_wen_gather))
  val vrf_wen = vwb_wen 
  val vrf_waddr = wb_waddr
  val reg_dmem_resp_valid = Reg(Bool())
  reg_dmem_resp_valid := io.dmem.resp.valid
  reg_cnt_cache := cnt_cache
  val vrf_wdata_gather = Mux(/*reg_dmem_resp_valid ||*/ io.dmem.resp.valid  && (reg_cnt_cache === number_of_elements - UInt(1)),vrf_mem_value (255, 0),/*v_wb_reg_wdata*/ v_alu_out)
  val vrf_wdata_unit_stride = Mux(io.dmem.resp.valid ,vrf_mem_value(255, 0), Mux(number_of_elements <= number_of_lanes ,v_wb_reg_wdata,v_alu_out))
//  val vrf_wdata = Mux((stride_vld === UInt(1) || wb_ctrl.vec_scalar) && !wb_ctrl.scatter_gather, vrf_wdata_unit_stride, vrf_wdata_gather)
 val vrf_wdata = Mux(((stride_vld === UInt(1) || wb_ctrl.vec_scalar) && !wb_ctrl.scatter_gather) || !wb_ctrl.mem, vrf_wdata_unit_stride, vrf_wdata_gather)

 //  printf("[checkcachecounter] @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ vrf_waddr %x vrf_wen %b [vwb_wen %b [stride_vld %x cnt_cache %x !wb_ctrl.scatter_gather %b] ==>vwb_wen_unit_stride %b [wb_ctrl.vec_scalar %b !wb_ctrl.scatter_gather %b] ==>vwb_wen_vec_scalar %b vwb_wen_gather %b ]vrf_wdata %x [vrf_wdata_unit_stride %x [number_of_elements <= number_of_lanes %b v_wb_reg_wdata %x v_alu_out %x]vrf_wdata_gather %x]\n", vrf_waddr,vrf_wen, vwb_wen, stride_vld, cnt_cache, !wb_ctrl.scatter_gather, vwb_wen_unit_stride, wb_ctrl.vec_scalar,  !wb_ctrl.scatter_gather, vwb_wen_vec_scalar, vwb_wen_gather,vrf_wdata, vrf_wdata_unit_stride,number_of_elements <= number_of_lanes,v_wb_reg_wdata,v_alu_out,  vrf_wdata_gather)
  when (vrf_wen) { /*vrf.write(vrf_waddr, vrf_wdata)*/ vrf(vrf_waddr) := vrf_wdata }


  // hook up control/status regfile
  csr.io.decode(0).csr := id_raw_inst(0)(31,20)
  csr.io.exception := wb_xcpt
  csr.io.cause := wb_cause
  csr.io.retire := wb_valid
  csr.io.inst(0) := (if (usingCompressed) Cat(Mux(wb_reg_raw_inst(1, 0).andR, wb_reg_inst >> 16, 0.U), wb_reg_raw_inst(15, 0)) else wb_reg_inst)
  csr.io.interrupts := io.interrupts
  csr.io.hartid := io.hartid
  io.fpu.fcsr_rm := csr.io.fcsr_rm
  csr.io.fcsr_flags := io.fpu.fcsr_flags
  csr.io.rocc_interrupt := io.rocc.interrupt
  csr.io.pc := wb_reg_pc
  val tval_valid = wb_xcpt && wb_cause.isOneOf(Causes.illegal_instruction, Causes.breakpoint,
    Causes.misaligned_load, Causes.misaligned_store,
    Causes.load_access, Causes.store_access, Causes.fetch_access,
    Causes.load_page_fault, Causes.store_page_fault, Causes.fetch_page_fault)
  csr.io.tval := Mux(tval_valid, Mux(wb_ctrl.vec, io.dmem.req.bits.addr,encodeVirtualAddress(wb_reg_wdata, wb_reg_wdata)), 0.U)
  io.ptw.ptbr := csr.io.ptbr
  io.ptw.status := csr.io.status
  io.ptw.pmp := csr.io.pmp
  csr.io.rw.addr := wb_reg_inst(31,20)
  csr.io.rw.cmd := Mux(wb_reg_valid, wb_ctrl.csr, CSR.N)
  csr.io.rw.wdata := Mux(wb_ctrl.vec,io.dmem.req.bits.addr,wb_reg_wdata)
  io.trace := csr.io.trace


//instruction exception  printf("[checkcachecounter] CSSSSSSSSSSRRRRRR inside proce %x %b %x %b %b [%b %b %b %b seip couldn't] %x %b %x %x %x %x  ptbr.ppn %x  status.spie %b  [maxPAddrBits %x - pgIdxBits %x]   tval_valid %b\n", csr.io.decode(0).csr, csr.io.exception, csr.io.cause, csr.io.retire, csr.io.inst(0), csr.io.interrupts.debug, csr.io.interrupts.mtip, csr.io.interrupts.msip, csr.io.interrupts.meip, csr.io.hartid, csr.io.rocc_interrupt, csr.io.pc, csr.io.rw.addr, csr.io.rw.cmd, csr.io.rw.wdata, csr.io.ptbr.ppn, csr.io.status.spie, maxPAddrBits, pgIdxBits,  tval_valid)
  val hazard_targets = Seq((id_ctrl.rxs1 && id_raddr1 =/= UInt(0), id_raddr1),
                           ( id_ctrl.rxs2 && id_raddr2 =/= UInt(0), id_raddr2),
                           (id_ctrl.rxs3 && id_raddr3 =/= UInt(0), id_raddr3), //zazad now added for vector store 
                           ( id_ctrl.wxd  && id_waddr  =/= UInt(0), id_waddr))

  val fp_hazard_targets = Seq((io.fpu.dec.ren1, id_raddr1),
                              (io.fpu.dec.ren2, id_raddr2),
                              (io.fpu.dec.ren3, id_raddr3),
                              (io.fpu.dec.wen, id_waddr))

  
  val sboard = new Scoreboard(32, true)
  
  sboard.clear(ll_wen, ll_waddr)

  def id_sboard_clear_bypass(r: UInt) = { //it's not from pipeline stages bypassing data to not have hazard. it's data comming from insts in scoreboard (replay, div, rocc) they are removing regs from sboardbecause they are ready so .clear will work to remove them but on the fly we want to prevent them from causing hazard again
    // ll_waddr arrives late when D$ has ECC, so reshuffle the hazard check
    if (!tileParams.dcache.get.dataECC.isDefined) ll_wen && ll_waddr === r
    else div.io.resp.fire() && div.io.resp.bits.tag === r || dmem_resp_replay && dmem_resp_xpu && dmem_resp_waddr === r
  }
  
  val id_sboard_hazard = checkHazards(hazard_targets, rd => sboard.read(rd) && !id_sboard_clear_bypass(rd))
  //val wb_wen = wb_valid && wb_ctrl.wxd && !wb_ctrl.vec //added !wb_ctrl.vec
  //val wb_set_sboard = wb_ctrl.div || wb_dcache_miss || wb_ctrl.rocc
  sboard.set(wb_set_sboard && (wb_wen || vwb_wen), wb_waddr)
  
  // stall for RAW/WAW hazards on CSRs, loads, AMOs, and mul/div in execute stage.
  val ex_cannot_bypass = ex_ctrl.csr =/= CSR.N || ex_ctrl.jalr || ex_ctrl.mem || ex_ctrl.mul || ex_ctrl.div || ex_ctrl.fp || ex_ctrl.rocc //data canot be bypassed to id from ex
  val data_hazard_ex = ex_ctrl.wxd && checkHazards(hazard_targets, _ === ex_waddr) //id needs regs being written by instruction in exe 
  val fp_data_hazard_ex = ex_ctrl.wfd && checkHazards(fp_hazard_targets, _ === ex_waddr)
  val id_ex_hazard = ex_reg_valid && (data_hazard_ex && ex_cannot_bypass || fp_data_hazard_ex)

  // stall for RAW/WAW hazards on CSRs, LB/LH, and mul/div in memory stage.
  //mem_reg_slow_bypass := ex_slow_bypass
  //  val ex_slow_bypass = ex_ctrl.mem_cmd === M_XSC || Vec(MT_B, MT_BU, MT_H, MT_HU).contains(ex_ctrl.mem_type
  val mem_mem_cmd_bh =
    if (fastLoadWord) Bool(!fastLoadByte) && mem_reg_slow_bypass
    else Bool(true)
  val mem_cannot_bypass = mem_ctrl.csr =/= CSR.N || mem_ctrl.mem && mem_mem_cmd_bh || mem_ctrl.mul || mem_ctrl.div || mem_ctrl.fp || mem_ctrl.rocc
  val data_hazard_mem = mem_ctrl.wxd && checkHazards(hazard_targets, _ === mem_waddr)
  val fp_data_hazard_mem = mem_ctrl.wfd && checkHazards(fp_hazard_targets, _ === mem_waddr)
  val id_mem_hazard = mem_reg_valid && (data_hazard_mem && mem_cannot_bypass || fp_data_hazard_mem)
  id_load_use := mem_reg_valid && data_hazard_mem && mem_ctrl.mem //instruction in id stage need data loaded from memory
  // stall for RAW/WAW hazards on load/AMO misses and mul/div in writeback.
  val data_hazard_wb = wb_ctrl.wxd && checkHazards(hazard_targets, _ === wb_waddr)
  val fp_data_hazard_wb = wb_ctrl.wfd && checkHazards(fp_hazard_targets, _ === wb_waddr)
  val id_wb_hazard = wb_reg_valid && (data_hazard_wb && wb_set_sboard || fp_data_hazard_wb)

  val id_stall_fpu = if (usingFPU) {
    val fp_sboard = new Scoreboard(32)
    fp_sboard.set((wb_dcache_miss && wb_ctrl.wfd || io.fpu.sboard_set) && wb_valid, wb_waddr)
    fp_sboard.clear(dmem_resp_replay && dmem_resp_fpu, dmem_resp_waddr)
    fp_sboard.clear(io.fpu.sboard_clr, io.fpu.sboard_clra)
    id_csr_en && !io.fpu.fcsr_rdy || checkHazards(fp_hazard_targets, fp_sboard.read _)
  } else Bool(false)

  val dcache_blocked = Reg(Bool())
  dcache_blocked := !io.dmem.req.ready && (io.dmem.req.valid || dcache_blocked)
  val rocc_blocked = Reg(Bool())
  rocc_blocked := !wb_xcpt && !io.rocc.cmd.ready && (io.rocc.cmd.valid || rocc_blocked)

  ////////////////////////////////////////////////////////////////////////////VLD/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  val ctrl_stalld_cond_vld = Wire(Bool())
  when ((mem_reg_valid && mem_reg_flush_pipe && !ex_reg_flush_pipe)  || take_pc  || id_xcpt || ex_xcpt || mem_ldst_xcpt || mem_xcpt || wb_xcpt){
    ctrl_stalld_cond_vld := Bool(false)
    locks_vld := Bool(false)
   printf("[checkcachecounter]????? not stalling vector load/store mem_reg_flush_pipe %b && !ex_reg_flush_pipe %b)  || take_pc  %b || id_xcpt %b || ex_xcpt %b || mem_ldst_xcpt %b || mem_xcpt %b || wb_xcpt %b  \n", mem_reg_flush_pipe, !ex_reg_flush_pipe, take_pc, id_xcpt, ex_xcpt, mem_ldst_xcpt, mem_xcpt, wb_xcpt)
  }.otherwise{

    when (stride_vld === UInt(1) && !ex_ctrl.vec_scalar){
      ctrl_stalld_cond_vld := Mux(cnt_cache === UInt(0), ex_ctrl.vec && ex_ctrl.mem && ex_ctrl.wxd && (cnt_cache =/= UInt(1)) && ex_reg_valid, ex_ctrl.vec && ex_ctrl.mem && (cnt_cache =/= UInt(1)))
      printf("[checkcachecounter]????? unit stride ctrl_stalld_cond_vld %b cnt_cache %x \n",ctrl_stalld_cond_vld, cnt_cache)
    }.elsewhen (ex_ctrl.vec_scalar){
      ctrl_stalld_cond_vld := ex_ctrl.vec && ex_ctrl.mem && ex_ctrl.wxd && ex_reg_valid
      printf("[checkcachecounter]???? vector-scalar ctrl_stalld_cond_vld %b cnt_cache %x \n",ctrl_stalld_cond_vld, cnt_cache)
    }.otherwise{
      ctrl_stalld_cond_vld := Mux(cnt_cache === UInt(0), ex_ctrl.vec && ex_ctrl.mem && ex_ctrl.wxd && (cnt_cache =/= number_of_elements - UInt(1)) && ex_reg_valid, ex_ctrl.vec && ex_ctrl.mem && (cnt_cache =/= number_of_elements - UInt(1)))
     printf("[checkcachecounter]???? otherwise-gather ctrl_stalld_cond_vld %b cnt_cache %x \n",ctrl_stalld_cond_vld, cnt_cache)
    }

    when (stride_vld === UInt(1) && !ex_ctrl.vec_scalar){
      when (ex_reg_valid && ex_ctrl.vec && ex_ctrl.mem && ex_ctrl.wxd && cnt_cache === UInt(0) && !take_pc){
        locks_vld := Bool(true)
        printf("[checkcachecounter]????? unit stride snt_cache 0 locks_vld %b ctrl_stalld_cond_vld %b cnt_cache %x \n",locks_vld, ctrl_stalld_cond_vld, cnt_cache)
      }.elsewhen((cnt_cache === UInt(1)) && io.dmem.resp.valid && (io.dmem.resp.bits.return_addr === io.dmem.req.bits.addr)){
        locks_vld := Bool(false)
        printf("[checkcachecounter]????? unit stride snt_cache 1 locks_vld %b ctrl_stalld_cond_vld %b cnt_cache %x \n",locks_vld, ctrl_stalld_cond_vld, cnt_cache)
      }
    }.elsewhen(ex_ctrl.vec_scalar){
      when (ex_reg_valid && ex_ctrl.vec && ex_ctrl.mem && ex_ctrl.wxd && !take_pc){
        locks_vld := Bool(true)
      }.elsewhen(/*wb_reg_valid && */  wb_ctrl.vec && wb_ctrl.mem && wb_ctrl.wxd && io.dmem.resp.valid){//this condition wroks for stride 1
        locks_vld := Bool(false)
      }
      printf("[checkcachecounter]???? vec_scalar locks_vld %b cnt_cache %x \n",locks_vld, cnt_cache)
    }.otherwise{

      when (ex_reg_valid && ex_ctrl.vec && ex_ctrl.mem && ex_ctrl.wxd && cnt_cache === UInt(0) && !take_pc){
        locks_vld := Bool(true)
      }.elsewhen((cnt_cache === number_of_elements - UInt(1)) && io.dmem.resp.valid && (io.dmem.resp.bits.return_addr === io.dmem.req.bits.addr)){
        locks_vld := Bool(false)
      }
        printf("[checkcachecounter]???? gather load locks_vld %b cnt_cache %x \n",locks_vld, cnt_cache)
    }
  }
   ////////////////////////////////////////////////////////////////////////////VSD/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  val ctrl_stalld_cond_vsd = Wire(Bool())

  when (mem_reg_flush_pipe || take_pc  || id_xcpt || ex_xcpt || mem_ldst_xcpt || mem_xcpt || wb_xcpt){
    ctrl_stalld_cond_vsd := Bool(false)
    locks_vsd := Bool(false)
    // cnt_cache_vsd := UInt(0)      
    printf("[checkcachecounterfuncs]mem_reg_flush_pipe %b exceptions [%b %b %b %b %b] number_of_elements %x\n", mem_reg_flush_pipe, id_xcpt, ex_xcpt, mem_ldst_xcpt, mem_xcpt, wb_xcpt, number_of_elements)
  }.otherwise{
    when (stride_vsd === UInt(1)){
      //ctrl_stalld_cond_vsd := ex_ctrl.vec && ex_ctrl.mem && !ex_ctrl.wxd && ex_reg_valid
      //printf("[checkcachecounterfuncs]stride1 [%b %b %b %b] %b exceptions [%b %b %b %b %b] number_of_elements %x\n", ex_ctrl.vec, ex_ctrl.mem, !ex_ctrl.wxd, ex_reg_valid, mem_reg_flush_pipe, id_xcpt, ex_xcpt, mem_ldst_xcpt, mem_xcpt, wb_xcpt, number_of_elements)
      ctrl_stalld_cond_vsd := Mux(cnt_cache_vsd === UInt(0), ex_ctrl.vec && ex_ctrl.mem && !ex_ctrl.wxd && (cnt_cache_vsd =/= UInt(1)) && ex_reg_valid, ex_ctrl.vec && ex_ctrl.mem && (cnt_cache_vsd =/= UInt(1)))
      printf("[checkcachecounter]????? unit stride vsd ctrl_stalld_cond_vsd %b cnt_cache_vsd %x \n",ctrl_stalld_cond_vsd, cnt_cache_vsd)
    }.otherwise{
      ctrl_stalld_cond_vsd := Mux(cnt_cache_vsd === UInt(0), ex_ctrl.vec && ex_ctrl.mem && !ex_ctrl.wxd /* && (cnt_cache_vsd =/= number_of_elements - UInt(1) */ && ex_reg_valid, ex_ctrl.vec && ex_ctrl.mem && (cnt_cache_vsd =/= number_of_elements - UInt(1)))
        
      printf("[checkcachecounterfuncs]????? not_unit_stride otherwise ctrl_stalld_cond_vsd [%b %b %x %x] cnt_cache %x mem_reg_flush_pipe %b exceptions [%b %b %b %b %b]\n", ctrl_stalld_cond_vsd, ex_ctrl.vec && ex_ctrl.mem && !ex_ctrl.wxd && (cnt_cache_vsd =/= number_of_elements - UInt(1)) && ex_reg_valid,number_of_elements, cnt_cache_vsd,  mem_reg_flush_pipe, cnt_cache_vsd, id_xcpt, ex_xcpt, mem_ldst_xcpt, mem_xcpt, wb_xcpt)
    }

    when (stride_vsd === UInt(1)){

      when (ex_reg_valid && ex_ctrl.vec && ex_ctrl.mem && !ex_ctrl.wxd && cnt_cache_vsd === UInt(0) && !take_pc){
        locks_vsd := Bool(true)    

      }.elsewhen(/*wb_reg_valid && */(cnt_cache_vsd === UInt(1)) &&  wb_ctrl.vec && wb_ctrl.mem && !wb_ctrl.wxd && io.dmem.resp.valid){//this condition wroks for stride 1
        locks_vsd := Bool(false)
      }
      printf("[checkcachecounter]????? unit_stride locks_vsd %b ctrl_stalld_cond_vsd %b cnt_cache_vsd %x \n",locks_vsd, ctrl_stalld_cond_vsd, cnt_cache_vsd)

    }.otherwise{

      when (ex_reg_valid && ex_ctrl.vec && ex_ctrl.mem && !ex_ctrl.wxd && cnt_cache_vsd === UInt(0) && !take_pc){
        locks_vsd := Bool(true)
          
       printf("[checkcachecounter]????? not unit_stride locks_vsd %b ctrl_stalld_cond_vsd %b cnt_cache_vsd %x\n", locks_vsd, ctrl_stalld_cond_vsd, cnt_cache_vsd)
      }.elsewhen((cnt_cache_vsd === number_of_elements - UInt(1)) && io.dmem.resp.valid && (io.dmem.resp.bits.return_addr === io.dmem.req.bits.addr)){
        locks_vsd := Bool(false)
          
        printf("[checkcachecounter] ????? not unit_stride locks_vsd %b ctrl_stalld_cond_vsd %b cnt_cache_vsd %x\n",locks_vsd, ctrl_stalld_cond_vsd, cnt_cache_vsd)
      }
    }
  }
 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  val stall_vec_exe = Wire (Bool())
  when (mem_reg_flush_pipe  && mem_reg_valid )
  { stall_vec_exe := Bool(false)
    printf("[checkcachecounter] ((((((((((((((((((((((((((((((((((((((((((((((((((((mem_reg_flush_pipe %b mem_reg_valid %b stall_vec_exe %b)))))))))))))))))))))))))))))))\n", mem_reg_flush_pipe, mem_reg_valid, stall_vec_exe)
  }.otherwise {
    stall_vec_exe := Mux(number_of_elements > UInt(8), (id_ctrl.vec && !vector_unit.io.req.ready) || vector_unit.io.req.valid, ex_ctrl.vec && !vector_unit.io.req.ready)//it kind of support hazards 
  }

    ctrl_stalld :=
    id_ex_hazard || id_mem_hazard || id_wb_hazard || id_sboard_hazard ||
    csr.io.singleStep && (ex_reg_valid || mem_reg_valid || wb_reg_valid) ||
    id_ctrl.fp && id_stall_fpu ||
    id_ctrl.mem && dcache_blocked || // reduce activity during D$ misses
    id_ctrl.rocc && rocc_blocked || // reduce activity while RoCC is busy
    id_ctrl.div && (!(div.io.req.ready || (div.io.resp.valid && !wb_wxd)) || div.io.req.valid) || // reduce odds of replay
  /*id_ctrl.vec && !vector_unit.io.req.ready || vector_unit.io.req.valid) ||*/
    stall_vec_exe ||
    id_do_fence ||
    csr.io.csr_stall ||
    ((ctrl_stalld_cond_vld || locks_vld) && !take_pc  && !id_xcpt && !ex_xcpt && !mem_ldst_xcpt && !mem_xcpt && !wb_xcpt) || //(mem_direction_misprediction && take_pc_mem)) ||
  /*(stride =/= UInt(1)) &&*/ ((ctrl_stalld_cond_vsd || locks_vsd) && !take_pc  && !id_xcpt && !ex_xcpt && !mem_ldst_xcpt && !mem_xcpt && !wb_xcpt)//!(mem_direction_misprediction && take_pc_mem))

  ctrl_killd := !ibuf.io.inst(0).valid || ibuf.io.inst(0).bits.replay || take_pc_mem_wb || ctrl_stalld || csr.io.interrupt
  ctrl_killd_vlsd_vssd := !ibuf.io.inst(0).valid || ibuf.io.inst(0).bits.replay || take_pc_mem_wb || csr.io.interrupt
  io.imem.req.valid := take_pc
  io.imem.req.bits.speculative := !take_pc_wb

  io.imem.req.bits.pc :=
    Mux(wb_xcpt || csr.io.eret, csr.io.evec, // exception or [m|s]ret
    Mux(replay_wb,              wb_reg_pc,   // replay
                                mem_npc))    // flush or branch misprediction
  io.imem.flush_icache := wb_reg_valid && wb_ctrl.fence_i && !io.dmem.s2_nack
  io.imem.sfence.valid := wb_reg_valid && wb_reg_sfence
  io.imem.sfence.bits.rs1 := wb_ctrl.mem_type(0)
  io.imem.sfence.bits.rs2 := wb_ctrl.mem_type(1)
  io.imem.sfence.bits.addr := wb_reg_wdata
  io.imem.sfence.bits.asid := wb_reg_rs2
  io.ptw.sfence := io.imem.sfence

  ibuf.io.inst(0).ready := !ctrl_stalld
   
  io.imem.btb_update.valid := mem_reg_valid && !take_pc_wb && mem_wrong_npc && (!mem_cfi || mem_cfi_taken)
  io.imem.btb_update.bits.isValid := mem_cfi
  io.imem.btb_update.bits.cfiType :=
    Mux((mem_ctrl.jal || mem_ctrl.jalr) && mem_waddr(0), CFIType.call,
    Mux(mem_ctrl.jalr && mem_reg_inst(19,15) === BitPat("b00?01"), CFIType.ret,
    Mux(mem_ctrl.jal || mem_ctrl.jalr, CFIType.jump,
    CFIType.branch)))
  io.imem.btb_update.bits.target := io.imem.req.bits.pc
  io.imem.btb_update.bits.br_pc := (if (usingCompressed) mem_reg_pc + Mux(mem_reg_rvc, UInt(0), UInt(2)) else mem_reg_pc)
  io.imem.btb_update.bits.pc := ~(~io.imem.btb_update.bits.br_pc | (coreInstBytes*fetchWidth-1))
  io.imem.btb_update.bits.prediction := mem_reg_btb_resp

  io.imem.bht_update.valid := mem_reg_valid && !take_pc_wb
  io.imem.bht_update.bits.pc := io.imem.btb_update.bits.pc
  io.imem.bht_update.bits.taken := mem_br_taken
  io.imem.bht_update.bits.mispredict := mem_wrong_npc
  io.imem.bht_update.bits.branch := mem_ctrl.branch
  io.imem.bht_update.bits.prediction := mem_reg_btb_resp.bht

  io.fpu.valid := !ctrl_killd && id_ctrl.fp
  io.fpu.killx := ctrl_killx
  io.fpu.killm := killm_common
  io.fpu.inst := id_inst(0)
  io.fpu.fromint_data := ex_rs(0)
  io.fpu.dmem_resp_val := dmem_resp_valid && dmem_resp_fpu
  //commentedtocheckcache
  //io.fpu.dmem_resp_data := io.dmem.resp.bits.data_word_bypass
  io.fpu.dmem_resp_data := io.dmem.resp.bits.DcacheCpu_data_word_bypass(xLen-1,0)
  when(dmem_resp_valid && dmem_resp_fpu){
    ///printf ("[checkfpudata] %x %x \n", io.dmem.resp.bits.data_word_bypass, io.dmem.resp.bits.DcacheCpu_data_word_bypass(xLen-1,0))
  }
  io.fpu.dmem_resp_type := io.dmem.resp.bits.typ
  io.fpu.dmem_resp_tag := dmem_resp_waddr

  //io.dmem.req.valid     := ex_reg_valid && ex_ctrl.mem
  val ex_dcache_tag = Cat(ex_waddr, ex_ctrl.fp)
  require(coreDCacheReqTagBits >= ex_dcache_tag.getWidth)
  io.dmem.req.bits.tag  := ex_dcache_tag
  io.dmem.req.bits.cmd  := ex_ctrl.mem_cmd
  io.dmem.req.bits.typ  := ex_ctrl.mem_type
  io.dmem.req.bits.phys := Bool(false)
  io.dmem.req.bits.vector_cache_access_type := stride_vsd === UInt(1)
  io.dmem.req.bits.cnt_cache_vsd := cnt_cache_vsd
  io.dmem.req.bits.element_number := number_of_elements
  io.dmem.req.bits.is_cache_access_vec := ex_ctrl.vec && ex_ctrl.mem
    //add cache count, I assume it depends on the elements size here is 16 bits or 2 byte so the address increases by two in each access
    // val offset = UInt(32) * cnt_cache
  val offset =Wire(UInt(10.W))
  when (ex_ctrl.vec_scalar && !ex_ctrl.scatter_gather){
    //    offset := UInt(32) * cnt_cache * stride_vld
    offset := UInt(0)
   printf("[checkcachecounter]?????? offset calculation vec_scalar vlsd-unit  cnt_cache %x offset %x stride_vld %x\n", cnt_cache, offset,stride_vld)
  }.elsewhen((stride_vld === UInt(1)) && !ex_ctrl.scatter_gather){ // unit stride vlsd
    offset := UInt(32) * cnt_cache * stride_vld
    printf("[checkcachecounter]?????? offset calculatio unitstride cnt_cache %x offset %x stride_vld %x\n", cnt_cache, offset,stride_vld)
  }.elsewhen(!ex_ctrl.scatter_gather){ //constant stride vlsd
    offset := elements_width * cnt_cache * stride_vld


      when(id_inst(0) === "h0007d20b".U || ex_reg_inst ===  "h0007d20b".U || mem_reg_inst === "h0007d20b".U || wb_reg_inst === "h0007d20b".U ||id_inst(0) === "h00971073".U || ex_reg_inst ===  "h00971073".U || mem_reg_inst === "h00971073".U || wb_reg_inst === "h00971073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2403500b".U || ex_reg_inst === "h2403500b".U || mem_reg_inst === "h2403500b".U || wb_reg_inst === "h2403500b".U ||id_inst(0) === "h00991073".U || ex_reg_inst ===  "h00991073".U || mem_reg_inst === "h00991073".U || wb_reg_inst === "h00991073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U ||id_inst(0) === "h0007d20b".U || ex_reg_inst ===  "h0007d20b".U || mem_reg_inst === "h0007d20b".U || wb_reg_inst === "h0007d20b".U ||id_inst(0) === "h00971073".U || ex_reg_inst ===  "h00971073".U || mem_reg_inst === "h00971073".U || wb_reg_inst === "h00971073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2403500b".U || ex_reg_inst === "h2403500b".U || mem_reg_inst === "h2403500b".U || wb_reg_inst === "h2403500b".U ||id_inst(0) === "h00991073".U || ex_reg_inst ===  "h00991073".U || mem_reg_inst === "h00991073".U || wb_reg_inst === "h00991073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U || id_inst(0) === "h000fc58b".U || ex_reg_inst === "h000fc58b".U || mem_reg_inst === "h000fc58b".U || wb_reg_inst === "h000fc58b".U || id_inst(0) === "h0005560b".U || ex_reg_inst === "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U  || id_inst(0) === "h000ed68b".U || ex_reg_inst === "h000ed68b".U || mem_reg_inst === "h000ed68b".U || wb_reg_inst === "h000ed68b".U || id_inst(0) === "h02d9570b".U || ex_reg_inst === "h02d9570b".U || mem_reg_inst === "h02d9570b".U || wb_reg_inst === "h02d9570b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h76d9500b".U || ex_reg_inst === "h76d9500b".U || mem_reg_inst === "h76d9500b".U || wb_reg_inst === "h76d9500b".U){

    printf("[checkcachecounter]????? offset calculation constatnstride vld cnt_cache %d offset %d elementwidth %d \n", cnt_cache,offset, elements_width)
  }

    }.otherwise{ //gather load-vlxd
    val stride_cnt = (stride_vld_array >> cnt_cache * UInt(32))(31,0)
    offset := elements_width * stride_cnt


      when(id_inst(0) === "h0007d20b".U || ex_reg_inst ===  "h0007d20b".U || mem_reg_inst === "h0007d20b".U || wb_reg_inst === "h0007d20b".U ||id_inst(0) === "h00971073".U || ex_reg_inst ===  "h00971073".U || mem_reg_inst === "h00971073".U || wb_reg_inst === "h00971073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2403500b".U || ex_reg_inst === "h2403500b".U || mem_reg_inst === "h2403500b".U || wb_reg_inst === "h2403500b".U ||id_inst(0) === "h00991073".U || ex_reg_inst ===  "h00991073".U || mem_reg_inst === "h00991073".U || wb_reg_inst === "h00991073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U ||id_inst(0) === "h0007d20b".U || ex_reg_inst ===  "h0007d20b".U || mem_reg_inst === "h0007d20b".U || wb_reg_inst === "h0007d20b".U ||id_inst(0) === "h00971073".U || ex_reg_inst ===  "h00971073".U || mem_reg_inst === "h00971073".U || wb_reg_inst === "h00971073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2403500b".U || ex_reg_inst === "h2403500b".U || mem_reg_inst === "h2403500b".U || wb_reg_inst === "h2403500b".U ||id_inst(0) === "h00991073".U || ex_reg_inst ===  "h00991073".U || mem_reg_inst === "h00991073".U || wb_reg_inst === "h00991073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U || id_inst(0) === "h000fc58b".U || ex_reg_inst === "h000fc58b".U || mem_reg_inst === "h000fc58b".U || wb_reg_inst === "h000fc58b".U || id_inst(0) === "h0005560b".U || ex_reg_inst === "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U  || id_inst(0) === "h000ed68b".U || ex_reg_inst === "h000ed68b".U || mem_reg_inst === "h000ed68b".U || wb_reg_inst === "h000ed68b".U || id_inst(0) === "h02d9570b".U || ex_reg_inst === "h02d9570b".U || mem_reg_inst === "h02d9570b".U || wb_reg_inst === "h02d9570b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h76d9500b".U || ex_reg_inst === "h76d9500b".U || mem_reg_inst === "h76d9500b".U || wb_reg_inst === "h76d9500b".U){

   printf("[checkcachecounter]?????? offset calculatio scatter vld stride_cnt %d offset %d cnt_cache %d stride_vld_array %x\n", stride_cnt, offset, cnt_cache, stride_vld_array)
    }
  }

  val offset_vsd =Wire(UInt(10.W))
  when (stride_vsd === UInt(1) && !ex_ctrl.scatter_gather){
    offset_vsd := UInt(32) * cnt_cache_vsd * stride_vsd
    printf("[checkcachecounter]?????? offset calculation unit-stride vsd stride_vsd %d offset %d cnt_cache_vsd %d \n", stride_vsd, offset_vsd, cnt_cache_vsd)

  }.elsewhen(!ex_ctrl.scatter_gather){
    //here we increase offset by 32 to go to the next half-cache-line
    offset_vsd := elements_width * cnt_cache_vsd * stride_vsd


    when(id_inst(0) === "h0007d20b".U || ex_reg_inst ===  "h0007d20b".U || mem_reg_inst === "h0007d20b".U || wb_reg_inst === "h0007d20b".U ||id_inst(0) === "h00971073".U || ex_reg_inst ===  "h00971073".U || mem_reg_inst === "h00971073".U || wb_reg_inst === "h00971073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2403500b".U || ex_reg_inst === "h2403500b".U || mem_reg_inst === "h2403500b".U || wb_reg_inst === "h2403500b".U ||id_inst(0) === "h00991073".U || ex_reg_inst ===  "h00991073".U || mem_reg_inst === "h00991073".U || wb_reg_inst === "h00991073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U || id_inst(0) === "h240e500b".U || ex_reg_inst ===  "h240e500b".U || mem_reg_inst === "h240e500b".U || wb_reg_inst === "h240e500b".U || id_inst(0) === "h240b500b".U || ex_reg_inst ===  "h240b500b".U || mem_reg_inst === "h240b500b".U || wb_reg_inst === "h240b500b".U ||id_inst(0) === "h0007d20b".U || ex_reg_inst ===  "h0007d20b".U || mem_reg_inst === "h0007d20b".U || wb_reg_inst === "h0007d20b".U ||id_inst(0) === "h00971073".U || ex_reg_inst ===  "h00971073".U || mem_reg_inst === "h00971073".U || wb_reg_inst === "h00971073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2403500b".U || ex_reg_inst === "h2403500b".U || mem_reg_inst === "h2403500b".U || wb_reg_inst === "h2403500b".U ||id_inst(0) === "h00991073".U || ex_reg_inst ===  "h00991073".U || mem_reg_inst === "h00991073".U || wb_reg_inst === "h00991073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U || id_inst(0) === "h000fc58b".U || ex_reg_inst === "h000fc58b".U || mem_reg_inst === "h000fc58b".U || wb_reg_inst === "h000fc58b".U || id_inst(0) === "h0005560b".U || ex_reg_inst === "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U  || id_inst(0) === "h000ed68b".U || ex_reg_inst === "h000ed68b".U || mem_reg_inst === "h000ed68b".U || wb_reg_inst === "h000ed68b".U || id_inst(0) === "h02d9570b".U || ex_reg_inst === "h02d9570b".U || mem_reg_inst === "h02d9570b".U || wb_reg_inst === "h02d9570b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h76d9500b".U || ex_reg_inst === "h76d9500b".U || mem_reg_inst === "h76d9500b".U || wb_reg_inst === "h76d9500b".U){

    printf("[checkcachecounter]?????? offset calculation constatnstride vsd cnt_cache %d offset %d elementwidth %d stride_vsd %d \n", cnt_cache_vsd,offset_vsd, elements_width, stride_vsd)
     }
  }.otherwise{
    val stride_cnt = (stride_vsd_array >> cnt_cache_vsd * UInt(32))(31,0)
    offset_vsd := elements_width * stride_cnt


    when(id_inst(0) === "h0007d20b".U || ex_reg_inst ===  "h0007d20b".U || mem_reg_inst === "h0007d20b".U || wb_reg_inst === "h0007d20b".U ||id_inst(0) === "h00971073".U || ex_reg_inst ===  "h00971073".U || mem_reg_inst === "h00971073".U || wb_reg_inst === "h00971073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2403500b".U || ex_reg_inst === "h2403500b".U || mem_reg_inst === "h2403500b".U || wb_reg_inst === "h2403500b".U ||id_inst(0) === "h00991073".U || ex_reg_inst ===  "h00991073".U || mem_reg_inst === "h00991073".U || wb_reg_inst === "h00991073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U || id_inst(0) === "h240e500b".U || ex_reg_inst ===  "h240e500b".U || mem_reg_inst === "h240e500b".U || wb_reg_inst === "h240e500b".U || id_inst(0) === "h240b500b".U || ex_reg_inst ===  "h240b500b".U || mem_reg_inst === "h240b500b".U || wb_reg_inst === "h240b500b".U ||id_inst(0) === "h0007d20b".U || ex_reg_inst ===  "h0007d20b".U || mem_reg_inst === "h0007d20b".U || wb_reg_inst === "h0007d20b".U ||id_inst(0) === "h00971073".U || ex_reg_inst ===  "h00971073".U || mem_reg_inst === "h00971073".U || wb_reg_inst === "h00971073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2403500b".U || ex_reg_inst === "h2403500b".U || mem_reg_inst === "h2403500b".U || wb_reg_inst === "h2403500b".U ||id_inst(0) === "h00991073".U || ex_reg_inst ===  "h00991073".U || mem_reg_inst === "h00991073".U || wb_reg_inst === "h00991073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U || id_inst(0) === "h000fc58b".U || ex_reg_inst === "h000fc58b".U || mem_reg_inst === "h000fc58b".U || wb_reg_inst === "h000fc58b".U || id_inst(0) === "h0005560b".U || ex_reg_inst === "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U  || id_inst(0) === "h000ed68b".U || ex_reg_inst === "h000ed68b".U || mem_reg_inst === "h000ed68b".U || wb_reg_inst === "h000ed68b".U || id_inst(0) === "h02d9570b".U || ex_reg_inst === "h02d9570b".U || mem_reg_inst === "h02d9570b".U || wb_reg_inst === "h02d9570b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h76d9500b".U || ex_reg_inst === "h76d9500b".U || mem_reg_inst === "h76d9500b".U || wb_reg_inst === "h76d9500b".U){

    printf("[checkcachecounter]????? offset calculation  gather vsd stride_cnt %d offset %d cnt_cache %d stride_vsd %x\n", stride_cnt, offset_vsd, cnt_cache_vsd, stride_vsd)

 }
   }
  

  val old_ready = Reg(Bool())
  old_ready := io.dmem.req.ready

  val edgeready = Reg(Bool())
  edgeready := !old_ready && io.dmem.req.ready

  val replay_after_miss = Reg(Bool())
  //change it to a mux
  when(wb_ctrl.vec && wb_ctrl.mem && /*PosEdge(io.dmem.req.ready)*/ edgeready && wb_ctrl.wxd)
  {
    replay_after_miss := Bool(true)
  }.otherwise {
    replay_after_miss := Bool(false)
  }

  val replay_after_miss_vsd = Reg(Bool())
  //change it to a mux
  when(wb_ctrl.vec && wb_ctrl.mem && /*PosEdge(io.dmem.req.ready)*/ edgeready  && !wb_ctrl.wxd)
  {
    replay_after_miss_vsd := Bool(true)
  }.otherwise {
    replay_after_miss_vsd := Bool(false)
  }

 // val old_ready = Reg(Bool())
 // old_ready := io.dmem.req.ready

  //if miss happens for 0 then what? how make valid when ready is valid?
  when (ex_ctrl.vec && ex_ctrl.mem && ex_ctrl.wxd){

     when (cnt_cache === UInt(0)){
       io.dmem.req.bits.addr := encodeVirtualAddress(ex_rs(0), alu.io.adder_out) + offset
       prev_address_offset := io.dmem.resp.bits.return_addr(4,0)  
        printf("[checkcachecounter]????? address vld cnt_cache 0 io.dmem.req.bits.addr %x io.dmem.req.valid %b offset %x ex_rs(0) %x alu.io.adder_out %x\n", io.dmem.req.bits.addr, io.dmem.req.valid, offset, ex_rs(0), alu.io.adder_out)
       val checkboundary = number_of_elements * 2 + io.dmem.req.bits.addr(4,0)
       when (checkboundary > UInt(32))
       {
         printf("[checkcachecounter]?????? in cnt_cache =0 next cycle is =>{{{{{{{{{{{{ NEED TO ACCESS NEXT HL }}}}}}}}}}}}}}}}}}}}  \n")
       }
       io.dmem.req.valid := Mux(replay_after_miss, Bool(true), ex_reg_valid && ex_ctrl.mem)
       io.dmem.s1_kill := killm_common_vlsd_vssd //Bool(false)
     }.elsewhen(wb_ctrl.vec && wb_ctrl.mem /* && edgeready */ /*(PosEdge(io.dmem.resp.valid))*/ /* || PosEdge(io.dmem.req.ready) */){
       when (stride_vld === UInt(1) && !ex_ctrl.vec_scalar){//wb_ctrl.vec_scalar
         //io.dmem.req.bits.addr := encodeVirtualAddress(ex_rs(0), alu.io.adder_out) + UInt(32)
         val checkboundary = number_of_elements * 2 + prev_address_offset(4,0)
         when (checkboundary > UInt(32)){
           io.dmem.req.bits.addr := (((encodeVirtualAddress(ex_rs(0), alu.io.adder_out)) >> UInt(5)) << UInt(5)) + UInt(32)
         }.otherwise{
            io.dmem.req.bits.addr := encodeVirtualAddress(ex_rs(0), alu.io.adder_out)
         }
         io.dmem.req.valid := Mux(replay_after_miss, Bool(true), cache_s1_valid_value && !io.dmem.resp.valid && ex_reg_valid_vlsd_vssd)
         io.dmem.s1_kill := killm_common_vlsd_vssd //Bool(false)
         printf("[checkcachecounter]????? address unit-stride vld  > 32 %b prev_addr_offset %x ex_rs(0) %x alu.io.adder_out %x io.dmem.req.bits.addr %x [%x %x] io.dmem.req.valid %b !io.dmem.resp.valid %b cache_s1_valid_value %b replay_after_miss %b io.dmem.s1_kill %b\n",checkboundary > UInt(32),prev_address_offset, ex_rs(0), alu.io.adder_out, io.dmem.req.bits.addr, ((encodeVirtualAddress(ex_rs(0), alu.io.adder_out)) >> UInt(5)), (((encodeVirtualAddress(ex_rs(0), alu.io.adder_out)) >> UInt(5)) << UInt(5)),io.dmem.req.valid, !io.dmem.resp.valid, cache_s1_valid_value, replay_after_miss, io.dmem.s1_kill)
       }.elsewhen (ex_ctrl.vec_scalar){//wb_ctrl.vec_scalar
         printf("[checkcachecounter]????? address vec_scalar vld io.dmem.req.bits.addr %x io.dmem.req.valid %b\n", io.dmem.req.bits.addr, io.dmem.req.valid)
       }.otherwise{ //gather load
         io.dmem.req.bits.addr := encodeVirtualAddress(ex_rs(0), alu.io.adder_out) + offset
         io.dmem.req.valid := Mux(replay_after_miss, Bool(true), cache_s1_valid_value && !io.dmem.resp.valid && ex_reg_valid_vlsd_vssd)
         io.dmem.s1_kill := killm_common_vlsd_vssd //Bool(false)
         printf("[checkcachecounter]????? address vld gather  offset %x io.dmem.req.bits.addr %x io.dmem.req.valid %b [RAM %b %b %b %b] io.dmem.s1_kill %b\n", offset, io.dmem.req.bits.addr, io.dmem.req.valid, replay_after_miss, cache_s1_valid_value, ex_reg_valid_vlsd_vssd, killm_common_vlsd_vssd,io.dmem.s1_kill)
       }
     }

  }.elsewhen(ex_ctrl.vec && ex_ctrl.mem && !ex_ctrl.wxd){


      when (cnt_cache_vsd === UInt(0)){ //stride_1 and the first element of scatter store
           io.dmem.req.bits.addr := encodeVirtualAddress(ex_rs(0), alu.io.adder_out) + offset_vsd
           io.dmem.req.valid     := Mux(replay_after_miss_vsd, Bool(true), ex_reg_valid && ex_ctrl.mem)
           io.dmem.s1_kill := killm_common_vlsd_vssd //Bool(false)
           prev_address_offset_vsd := io.dmem.resp.bits.return_addr(4,0)
           val checkboundary = number_of_elements * 2 + io.dmem.req.bits.addr(4,0)
           when (checkboundary > UInt(32))
           {
             printf("[checkcachecounter]{{{{{{{{{{{{{{{{{{{{ vsd_NEED TO ACCESS NEXT HL }}}}}}}}}}}}}}}}}}}}  \n")
           }
           io.dmem.DcacheCpu_s1_data.data := temp_for_dmem_write_data
           val offset_reduction = UInt(32) - io.dmem.req.bits.addr(4,0)
           val vsd_element      = offset_reduction / UInt(2)
           io.dmem.req.bits.element_number := Mux(vsd_element > number_of_elements, number_of_elements, vsd_element)
           printf("[checkcachecounter]???? cnt_cache_vsd=0  ++++++++__________************** io.dmem.req.bits.addr %x  vsd cnt_cache_vsd===0 ex_rs(0) %x alu.io.adder_out %x offset_vsd %x  offset_reduction %x vsd_element %x io.dmem.req.bits.element_number %x element_number %x dmem_resp_valid %b io.dmem.s1_kill %b  \n", io.dmem.req.bits.addr,ex_rs(0), alu.io.adder_out,offset_vsd, offset_reduction, vsd_element, io.dmem.req.bits.element_number, number_of_elements, io.dmem.resp.valid, io.dmem.s1_kill)


      }.elsewhen(/*wb_ctrl.vec && wb_ctrl.mem && !wb_ctrl.wxd*/cnt_cache_vsd =/= UInt(0)){
       //io.dmem.req.bits.addr := encodeVirtualAddress(ex_rs(0), alu.io.adder_out) + offset_vsd
           when (stride_vsd === UInt(1) && !ex_ctrl.vec_scalar){//wb_ctrl.vec_scalar
                val checkboundary = number_of_elements * 2 + prev_address_offset_vsd(4,0)
                when (checkboundary > UInt(32)){
                  io.dmem.req.bits.addr := (((encodeVirtualAddress(ex_rs(0), alu.io.adder_out)) >> UInt(5)) << UInt(5)) + UInt(32)
                  val offset_reduction = UInt(32) - prev_address_offset_vsd(4,0)
                  val vsd_element      = offset_reduction / UInt(2)
                  io.dmem.req.bits.element_number := number_of_elements - vsd_element
                  io.dmem.DcacheCpu_s1_data.data := temp_for_dmem_write_data >> (vsd_element * UInt(2) * UInt(8))
                  printf("[checkcachecounter]????? cnt_cache_vsd > 0 unit_stride++++++++__________************** vsd checkboundary>32 io.dmem.req.bits.addr %x ex_rs(0) %x alu.io.adder_out %x  prev_addr_offset_vsd %x  offset_reduction %x vsd_element %x io.dmem.req.bits.element_number %x io.dmem.s1_kill %b\n", io.dmem.req.bits.addr, ex_rs(0), alu.io.adder_out,prev_address_offset_vsd, offset_reduction, vsd_element, io.dmem.req.bits.element_number,io.dmem.s1_kill)
                }.otherwise{
                  io.dmem.req.bits.addr := encodeVirtualAddress(ex_rs(0), alu.io.adder_out)
                  val offset_reduction = UInt(32) - io.dmem.req.bits.addr(4,0)
                  val vsd_element      = offset_reduction / UInt(2)
                  io.dmem.req.bits.element_number := Mux(vsd_element > number_of_elements, number_of_elements, vsd_element)
                  io.dmem.DcacheCpu_s1_data.data := temp_for_dmem_write_data
                  printf("[checkcachecounter]????? cnt_cache_vsd > 0 unitstride ++++++++__________************** vsd checkboundary<32 io.dmem.req.bits.addr %x ex_rs(0) %x alu.io.adder_out %x  offset_reduction %x vsd_element %x io.dmem.req.bits.element_number %x io.dmem.s1_kill %b \n", io.dmem.req.bits.addr, ex_rs(0), alu.io.adder_out,offset_reduction, vsd_element, io.dmem.req.bits.element_number, io.dmem.s1_kill)
                }


                io.dmem.req.valid := Mux(replay_after_miss_vsd, Bool(true), cache_s1_valid_value_vsd && !io.dmem.resp.valid && ex_reg_valid_vlsd_vssd)
                io.dmem.s1_kill := killm_common_vlsd_vssd //Bool(false)
                printf("[checkcachecounter]????? cnt_cache_vsd > 0 unitstride ex_rs(0) %x alu.io.adder_out %x io.dmem.req.bits.addr %x [%x %x] io.dmem.req.valid %b !io.dmem.resp.valid %b cache_s1_valid_value %b replay_after_miss %b io.dmem.s1_kill %b\n", ex_rs(0), alu.io.adder_out, io.dmem.req.bits.addr, ((encodeVirtualAddress(ex_rs(0), alu.io.adder_out)) >> UInt(5)), (((encodeVirtualAddress(ex_rs(0), alu.io.adder_out)) >> UInt(5)) << UInt(5)),io.dmem.req.valid, !io.dmem.resp.valid, cache_s1_valid_value, replay_after_miss, io.dmem.s1_kill)
           }.otherwise{ //scatter store
                io.dmem.req.bits.addr := encodeVirtualAddress(ex_rs(0), alu.io.adder_out) + offset_vsd
                io.dmem.DcacheCpu_s1_data.data := temp_for_dmem_write_data
                io.dmem.req.valid := Mux(replay_after_miss_vsd, Bool(true), cache_s1_valid_value_vsd && !io.dmem.resp.valid && ex_reg_valid_vlsd_vssd)
                io.dmem.s1_kill := killm_common_vlsd_vssd //Bool(false)
                printf("[checkcachecounter]????? scatter store ex_rs(0) %x alu.io.adder_out %x io.dmem.req.bits.addr %x offset_vsd %x io.dmem.req.valid %b !io.dmem.resp.valid %b cache_s1_valid_value %b replay_after_miss %b ex_reg_valid_vlsd_vssd %b killm_common_vlsd_vssd %b io.dmem.s1_kill %b\n", ex_rs(0), alu.io.adder_out, io.dmem.req.bits.addr,offset_vsd,io.dmem.req.valid, !io.dmem.resp.valid, cache_s1_valid_value, replay_after_miss, ex_reg_valid_vlsd_vssd,killm_common_vlsd_vssd, io.dmem.s1_kill)
           }
      }
  }.otherwise{//not vector store/load
     io.dmem.req.valid     := ex_reg_valid && ex_ctrl.mem
     io.dmem.req.bits.addr := encodeVirtualAddress(ex_rs(0), alu.io.adder_out)
     io.dmem.s1_kill := killm_common || mem_ldst_xcpt
     io.dmem.DcacheCpu_s1_data.data := temp_for_dmem_write_data
      printf("[checkcachecounter] ===================================> inside otherwise for memory address %x  ex_rs(0) %x alu.io.adder_out %x data %x temp %x io.dmem.s1_kill %b \n", io.dmem.req.bits.addr, ex_rs(0), alu.io.adder_out, io.dmem.DcacheCpu_s1_data.data, temp_for_dmem_write_data, io.dmem.s1_kill)
  }


 /// io.dmem.s1_kill := killm_common || mem_ldst_xcpt
  io.dmem.invalidate_lr := wb_xcpt
  io.dmem.req.bits.isvector := ex_ctrl.vec && ex_ctrl.mem && !ex_ctrl.vec_scalar
  //io.dmem.s1_data.data := (if (fLen == 0) mem_reg_rs2 else Mux(mem_ctrl.fp, Fill((xLen max fLen) / fLen, io.fpu.store_data), mem_reg_rs2)


  io.dmem.s1_data.data := Mux(mem_ctrl.vec,v_mem_reg_rs2(63,0),(if (fLen == 0) mem_reg_rs2 else Mux(mem_ctrl.fp, Fill((xLen max fLen) / fLen, io.fpu.store_data), mem_reg_rs2)))

  // io.dmem.DcacheCpu_s1_data.data
  temp_for_dmem_write_data := Mux(mem_ctrl.vec,v_mem_reg_rs2(255,0),Cat(UInt(1<<64)(63,1),UInt (0),UInt(1<<64)(63,1),UInt (0),UInt(1<<64)(63,1),UInt (0),(if (fLen == 0) mem_reg_rs2 else Mux(mem_ctrl.fp, Fill((xLen max fLen) / fLen, io.fpu.store_data), mem_reg_rs2))))


  io.rocc.cmd.valid := wb_reg_valid && wb_ctrl.rocc && !replay_wb_common
  io.rocc.exception := wb_xcpt && csr.io.status.xs.orR
  io.rocc.cmd.bits.status := csr.io.status
  io.rocc.cmd.bits.inst := new RoCCInstruction().fromBits(wb_reg_inst)
  io.rocc.cmd.bits.rs1 := wb_reg_wdata
  io.rocc.cmd.bits.rs2 := wb_reg_rs2

  // evaluate performance counters
  val icache_blocked = !(io.imem.resp.valid || RegNext(io.imem.resp.valid))
  csr.io.counters foreach { c => c.inc := RegNext(perfEvents.evaluate(c.eventSel)) }










  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*   printf("[checkcachecounter]IMMMMMMMMEM ibuf.io.pc.asSInt %x ibuf.io.inst(0).bits.xcpt0 [pf.inst %b ae.inst %b]  ibuf.io.inst(0).bits.xcpt1 [pf.inst %b ae.inst %b] id_xcpt %b id_cause %x csr.io.interrupt %b bpu.io.debug_if %b bpu.io.xcpt_if %b id_xcpt0.pf.inst %b id_xcpt0.ae.inst %b id_xcpt1.pf.inst %b id_xcpt1.ae.inst %b id_illegal_insn %b  ||||| ibuf.io.inst(0).valid %b ibuf.io.inst(0).bits.replay %b ||||||| ctrl_killd %b\n", ibuf.io.pc.asSInt, ibuf.io.inst(0).bits.xcpt0.pf.inst, ibuf.io.inst(0).bits.xcpt0.ae.inst, ibuf.io.inst(0).bits.xcpt1.pf.inst, ibuf.io.inst(0).bits.xcpt1.ae.inst, id_xcpt, id_cause, csr.io.interrupt,  bpu.io.debug_if, bpu.io.xcpt_if, id_xcpt0.pf.inst, id_xcpt0.ae.inst, id_xcpt1.pf.inst, id_xcpt1.ae.inst, id_illegal_insn, ibuf.io.inst(0).valid, ibuf.io.inst(0).bits.replay, ctrl_killd)

     printf("[checkcachecounter]IMMMMMMMMEM  ctrl_killd %b ctrl_killd_vlsd_vssd %b io.imem.req.valid %b take_pc %b io.imem.req.bits.speculative %b take_pc_wb %b io.imem.req.bits.pc %x wb_xcpt %b csr.io.eret %b csr.io.evec %x replay_wb %b wb_reg_pc %x mem_npc %x io.imem.flush_icache %b  wb_reg_valid %b && wb_ctrl.fence_i %b && !io.dmem.s2_nack %b]  io.imem.sfence.valid %b :=[ wb_reg_valid %b && wb_reg_sfence %b] io.imem.sfence.bits.rs1 %b := wb_ctrl.mem_type(0) %x io.imem.sfence.bits.rs2 %x := wb_ctrl.mem_type(1) %x io.imem.sfence.bits.addr %x := wb_reg_wdata %x io.imem.sfence.bits.asid %x := wb_reg_rs2 %x  ibuf.io.inst(0).ready %b !ctrl_stalld %b  icache_blocked %b  io.imem.resp.valid %b \n", ctrl_killd, ctrl_killd_vlsd_vssd, io.imem.req.valid, take_pc, io.imem.req.bits.speculative, take_pc_wb, io.imem.req.bits.pc, wb_xcpt, csr.io.eret, csr.io.evec, replay_wb, wb_reg_pc, mem_npc, io.imem.flush_icache, wb_reg_valid, wb_ctrl.fence, !io.dmem.s2_nack,io.imem.sfence.valid, wb_reg_valid, wb_reg_sfence, io.imem.sfence.bits.rs1, wb_ctrl.mem_type(0), io.imem.sfence.bits.rs2, wb_ctrl.mem_type(1), io.imem.sfence.bits.addr, wb_reg_wdata,io.imem.sfence.bits.asid, wb_reg_rs2, ibuf.io.inst(0).ready, !ctrl_stalld, icache_blocked,io.imem.resp.valid)

  // instruction exception  printf("[checkcachecounter]FINAL io.dmem.req.valid %b RAM %b caches1valid %b ex_reg_valid_vlsd %b io.dmem.s1_kill %b kill_common_vlsd %b killm_common %b mem_reg_valid %b killm_common [dcache_kill_mem %b take_pc_wb %b mem_reg_xcpt %b] mem_ldst_xcpt %b ex_reg_valid %b io.dmem [replay_next %b s2_nack %b] \n" , io.dmem.req.valid, replay_after_miss_vsd, cache_s1_valid_value_vsd, ex_reg_valid_vlsd_vssd, io.dmem.s1_kill, killm_common_vlsd_vssd, killm_common, mem_reg_valid, dcache_kill_mem,take_pc_wb,mem_reg_xcpt,  mem_ldst_xcpt, ex_reg_valid, io.dmem.replay_next, io.dmem.s2_nack)
 // csr printf("[checkcachecounter]FINAL replay_wb %b = replay_wb_common %b || replay_wb_rocc %b  replay_wb_common [%b && ! %b || %b]  take_pc_wb %b [%b || %b || %b || %b] wb_reg_flush_pipe %b[! %b && %b] ctrl_killm %b [%b || %b || %b] killm_common %b [%b || %b || %b || ! %b]  mem_reg_xcpt %b [! %b & %b] ctrl_killx [%b || %b || ! %b] ex_xcpt%b  [%b || %b]\n", replay_wb, replay_wb_common, replay_wb_rocc, io.dmem.s2_nack, wb_ctrl.vec, wb_reg_replay, take_pc_wb, replay_wb, wb_xcpt, csr.io.eret, wb_reg_flush_pipe, wb_reg_flush_pipe, ctrl_killm, mem_reg_flush_pipe, ctrl_killm, killm_common, mem_xcpt, fpu_kill_mem, killm_common, dcache_kill_mem, take_pc_wb, mem_reg_xcpt, mem_reg_valid, mem_reg_xcpt, ctrl_killx, ex_xcpt, take_pc_mem_wb, replay_ex, ex_reg_valid, ex_xcpt, ex_reg_xcpt_interrupt, ex_reg_xcpt)

  printf("[checkcachecounter] ******************* send memory request address %x [ex_rs(0) %x alu_io.adder.out %x -> encode %x ] valid %b  s1_data %x \n", io.dmem.req.bits.addr,ex_rs(0), alu.io.adder_out, encodeVirtualAddress(ex_rs(0), alu.io.adder_out), io.dmem.req.valid, io.dmem.DcacheCpu_s1_data.data)
    printf("[checkcachecounter]S1_DAAATA temp_for_dmem_write_data %x mem_ctrl.vec %b v_mem_reg_rs2 %x fLen %x mem_reg_rs2  %x mem_ctrl.fp %b io.fpu.store_data %x ||||| ex_pc_valid %b => when %b ex_ctrl.rxs2 %b && (ex_ctrl.mem %b || ex_ctrl.rocc %b || ex_sfence %b)  ex_ctrl.mem_type %x storegen %x  ||| ex_rs(1) %x ex_vrs(2) %x\n", temp_for_dmem_write_data, mem_ctrl.vec, v_mem_reg_rs2, fLen, mem_reg_rs2, mem_ctrl.fp, io.fpu.store_data, ex_pc_valid, ex_ctrl.rxs2 && (ex_ctrl.mem || ex_ctrl.rocc || ex_sfence) ,ex_ctrl.rxs2, ex_ctrl.mem, ex_ctrl.rocc, ex_sfence, ex_ctrl.mem_type, new StoreGen(Mux(ex_ctrl.rocc, log2Ceil(xLen/8).U, ex_ctrl.mem_type), 0.U, ex_rs(1), coreDataBytes).data, ex_rs(1), ex_vrs(2))
 */
 //when (id_inst(0) === "h02bb560b".U || id_inst(0)  === "h00985073".U ||  id_inst(0)   === "h000c558b".U || id_inst(0)   === "h02ba570b".U || id_inst(0)   === "h02b9d78b".U || id_inst(0)   === "h80f75257".U || id_inst(0) === "h26bb500b".U ||  ex_reg_inst  === "h00985073".U ||  ex_reg_inst  === "h000c558b".U || ex_reg_inst  === "h02ba570b".U || ex_reg_inst  === "h02b9d78b".U || ex_reg_inst === "h02bb560b".U || ex_reg_inst  === "h80f75257".U || ex_reg_inst === "h26bb500b".U || mem_reg_inst === "h00985073".U ||  mem_reg_inst === "h000c558b".U || mem_reg_inst === "h02ba570b".U || mem_reg_inst === "h02bb560b".U ||  mem_reg_inst === "h02b9d78b".U || mem_reg_inst === "h80f75257".U || mem_reg_inst === "h26bb500b".U ||  wb_reg_inst  === "h00985073".U ||  wb_reg_inst  === "h000c558b".U || wb_reg_inst  === "h02ba570b".U  || wb_reg_inst  === "h02b9d78b".U || wb_reg_inst  === "h80f75257".U || wb_reg_inst === "h26bb500b".U || wb_reg_inst === "h02bb560b".U ){
//matrix_multiply when (id_inst(0) === "h00879073".U || id_inst(0) === "h00979073".U  || id_inst(0) === "h009f1073".U  || id_inst(0) === "h00941073".U  || id_inst(0) === "h0005c58b".U  || id_inst(0) === "h0007d60b".U  || id_inst(0) === "h0006d70b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h7406d00b".U  || id_inst(0) === "h0007d60b".U  || id_inst(0) === "h0003d70b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h7403d00b".U  || id_inst(0) === "h0007560b".U  || id_inst(0) === "h0006d68b".U  || id_inst(0) === "h02d9570b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h76d9500b".U  || id_inst(0) === "h0005560b".U  || id_inst(0) === "h000ed68b".U  || id_inst(0) === "h02d9570b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h76d9500b".U  || ex_reg_inst === "h00879073".U   || ex_reg_inst === "h00979073".U   || ex_reg_inst === "h009f1073".U   || ex_reg_inst === "h00941073".U   || ex_reg_inst === "h0005c58b".U   || ex_reg_inst === "h0007d60b".U   || ex_reg_inst === "h0006d70b".U   || ex_reg_inst === "h72c5976b".U   || ex_reg_inst === "h7406d00b".U   || ex_reg_inst === "h0007d60b".U   || ex_reg_inst === "h0003d70b".U  || ex_reg_inst === "h72c5976b".U  || ex_reg_inst === "h7403d00b".U  || ex_reg_inst === "h0007560b".U  || ex_reg_inst === "h0006d68b".U  || ex_reg_inst === "h02d9570b".U  || ex_reg_inst === "h72c5976b".U  || ex_reg_inst === "h76d9500b".U  || ex_reg_inst === "h0005560b".U  || ex_reg_inst === "h000ed68b".U  || ex_reg_inst === "h02d9570b".U  || ex_reg_inst === "h72c5976b".U  || ex_reg_inst === "h76d9500b".U || mem_reg_inst === "h00879073".U  || mem_reg_inst === "h00979073".U  || mem_reg_inst === "h009f1073".U  || mem_reg_inst === "h00941073".U  || mem_reg_inst === "h0005c58b".U  || mem_reg_inst === "h0007d60b".U  || mem_reg_inst === "h0006d70b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h7406d00b".U  || mem_reg_inst === "h0007d60b".U  || mem_reg_inst === "h0003d70b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h7403d00b".U  || mem_reg_inst === "h0007560b".U  || mem_reg_inst === "h0006d68b".U  || mem_reg_inst === "h02d9570b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h76d9500b".U  || mem_reg_inst === "h0005560b".U  || mem_reg_inst === "h000ed68b".U  || mem_reg_inst === "h02d9570b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h76d9500b".U || wb_reg_inst === "h00879073".U  || wb_reg_inst === "h00979073".U  || wb_reg_inst === "h009f1073".U  || wb_reg_inst === "h00941073".U  || wb_reg_inst === "h0005c58b".U  || wb_reg_inst === "h0007d60b".U  || wb_reg_inst === "h0006d70b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h7406d00b".U  || wb_reg_inst === "h0007d60b".U  || wb_reg_inst === "h0003d70b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h7403d00b".U  || wb_reg_inst === "h0007560b".U  || wb_reg_inst === "h0006d68b".U  || wb_reg_inst === "h02d9570b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h76d9500b".U  || wb_reg_inst === "h0005560b".U  || wb_reg_inst === "h000ed68b".U  || wb_reg_inst === "h02d9570b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h76d9500b".U){
//sparse_matrix bench
  //when (id_inst(0) === "h00881073".U || id_inst(0) === "h00929073".U || id_inst(0) === "h00999073".U || id_inst(0) === "h0007c58b".U || id_inst(0) === "h0008560b".U || id_inst(0) === "h22c5926b".U || id_inst(0) === "h240e500b".U || id_inst(0) === "h2407d00b".U || id_inst(0) === "h0040425b".U || ex_reg_inst === "h00881073".U || ex_reg_inst === "h00929073".U || ex_reg_inst === "h00999073".U || ex_reg_inst  === "h0007c58b".U || ex_reg_inst === "h0008560b".U || ex_reg_inst === "h22c5926b".U || ex_reg_inst === "h240e500b".U || ex_reg_inst === "h2407d00b".U || ex_reg_inst === "h0040425b".U || mem_reg_inst === "h00881073".U || mem_reg_inst === "h00929073".U || mem_reg_inst === "h00999073".U || mem_reg_inst === "h0007c58b".U || mem_reg_inst === "h0008560b".U || mem_reg_inst === "h22c5926b".U || mem_reg_inst === "h240e500b".U || mem_reg_inst === "h2407d00b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h00881073".U || wb_reg_inst === "h00929073".U || wb_reg_inst === "h00999073".U || wb_reg_inst === "h0007c58b".U || wb_reg_inst === "h0008560b".U || wb_reg_inst === "h22c5926b".U || wb_reg_inst === "h240e500b".U ||  wb_reg_inst  === "h2407d00b".U || wb_reg_inst === "h0040425b".U){

//when(id_inst(0) === "h0005c58b".U || ex_reg_inst === "h0005c58b".U || mem_reg_inst === "h0005c58b".U || wb_reg_inst === "h0005c58b".U){
//  when(id_inst(0) === "h0007c58b".U || ex_reg_inst === "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U || id_inst(0) === "h0008560b".U || ex_reg_inst === "h0008560b".U || mem_reg_inst === "h0008560b".U || wb_reg_inst === "h0008560b".U || id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U || id_inst(0) === "h".U || ex_reg_inst === "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U){

  //when(id_inst(0) === "h0007c58b".U || ex_reg_inst === "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U || id_inst(0) === "h0008560b".U || ex_reg_inst === "h0008560b".U || mem_reg_inst === "h0008560b".U || wb_reg_inst === "h0008560b".U || id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U || id_inst(0) === "h0007c58b".U || ex_reg_inst === "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U || ibuf.io.pc === "h6b6da".U ){
//  when(id_inst(0) === "h0005c58b".U || ex_reg_inst === "h0005c58b".U || mem_reg_inst === "h0005c58b".U || wb_reg_inst === "h0005c58b".U || id_inst(0) === "h00959073".U || ex_reg_inst === "h00959073".U || mem_reg_inst === "h00959073".U || wb_reg_inst === "h00959073".U || id_inst(0) === "h000c558b".U || ex_reg_inst === "h000c558b".U || mem_reg_inst === "h000c558b".U || wb_reg_inst === "h000c558b".U || id_inst(0) === "h000a470b".U || ex_reg_inst === "h000a470b".U || mem_reg_inst === "h000a470b".U || wb_reg_inst === "h000a470b".U || id_inst(0) === "h0009c78b".U || ex_reg_inst === "h0009c78b".U || mem_reg_inst === "h0009c78b".U || wb_reg_inst === "h0009c78b".U || id_inst(0) === "h72f7126b".U || ex_reg_inst === "h72f7126b".U || mem_reg_inst === "h72f7126b".U || wb_reg_inst === "h72f7126b".U || id_inst(0) === "h26bb500b".U || ex_reg_inst === "h26bb500b".U || mem_reg_inst === "h26bb500b".U || wb_reg_inst === "h26bb500b".U || id_inst(0) === "h000a470b".U || ex_reg_inst === "h000a470b".U || mem_reg_inst === "h000a470b".U || wb_reg_inst === "h000a470b".U || id_inst(0) === "h0009c78b".U || ex_reg_inst === "h0009c78b".U || mem_reg_inst === "h0009c78b".U || wb_reg_inst === "h0009c78b".U || id_inst(0) === "h80f75257".U || ex_reg_inst === "h80f75257".U || mem_reg_inst === "h80f75257".U || wb_reg_inst === "h80f75257".U || id_inst(0) === "h240bd00b".U || ex_reg_inst === "h240bd00b".U || mem_reg_inst === "h240bd00b".U || wb_reg_inst === "h240bd00b".U ){
  // when(id_inst(0) === "h0007d20b".U || ex_reg_inst ===  "h0007d20b".U || mem_reg_inst === "h0007d20b".U || wb_reg_inst === "h0007d20b".U || id_inst(0) === "h00879073".U || ex_reg_inst ===  "h00879073".U || mem_reg_inst === "h00879073".U || wb_reg_inst === "h00879073".U || id_inst(0) === "h000fc58b".U || ex_reg_inst === "h000fc58b".U || mem_reg_inst === "h000fc58b".U || wb_reg_inst === "h000fc58b".U || id_inst(0) === "h0007560b".U || ex_reg_inst === "h0007560b".U || mem_reg_inst === "h0007560b".U || wb_reg_inst === "h0007560b".U || id_inst(0) === "h0006d68b".U || ex_reg_inst === "h0006d68b".U || mem_reg_inst === "h0006d68b".U || wb_reg_inst === "h0006d68b".U || id_inst(0) === "h02d9570b".U || ex_reg_inst === "h02d9570b".U || mem_reg_inst === "h02d9570b".U || wb_reg_inst === "h02d9570b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h76d9500b".U || ex_reg_inst === "h76d9500b".U || mem_reg_inst === "h76d9500b".U || wb_reg_inst === "h76d9500b".U || id_inst(0) === "h00979073".U || ex_reg_inst === "h00979073".U || mem_reg_inst === "h00979073".U || wb_reg_inst === "h00979073".U || id_inst(0) === "h0005560b".U || ex_reg_inst === "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U || id_inst(0) === "h000ed68b".U || ex_reg_inst === "h000ed68b".U || mem_reg_inst === "h000ed68b".U || wb_reg_inst === "h000ed68b".U || id_inst(0) === "h02d9570b".U || ex_reg_inst === "h02d9570b".U || mem_reg_inst === "h02d9570b".U || wb_reg_inst === "h02d9570b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h76d9500b".U || ex_reg_inst === "h76d9500b".U || mem_reg_inst === "h76d9500b".U || wb_reg_inst === "h76d9500b".U || id_inst(0) === "h0005c58b".U || ex_reg_inst === "h0005c58b".U || mem_reg_inst === "h0005c58b".U || wb_reg_inst === "h0005c58b".U || id_inst(0) === "h00879073".U || ex_reg_inst === "h00879073".U || mem_reg_inst === "h00879073".U || wb_reg_inst === "h00879073".U || id_inst(0) === "h00979073".U || ex_reg_inst === "h00979073".U || mem_reg_inst === "h00979073".U || wb_reg_inst === "h00979073".U || id_inst(0) === "h0005c58b".U || ex_reg_inst === "h0005c58b".U || mem_reg_inst === "h0005c58b".U || wb_reg_inst === "h0005c58b".U || id_inst(0) === "h0006d70b".U || ex_reg_inst === "h0006d70b".U || mem_reg_inst === "h0006d70b".U || wb_reg_inst === "h0006d70b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h7406d00b".U || ex_reg_inst === "h7406d00b".U || mem_reg_inst === "h7406d00b".U || wb_reg_inst === "h7406d00b".U || id_inst(0) === "h0007d60b".U || ex_reg_inst === "h0007d60b".U || mem_reg_inst === "h0007d60b".U || wb_reg_inst === "h0007d60b".U || id_inst(0) === "h000ed70b".U || ex_reg_inst === "h000ed70b".U || mem_reg_inst === "h000ed70b".U || wb_reg_inst === "h000ed70b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h740ed00b".U || ex_reg_inst === "h740ed00b".U || mem_reg_inst === "h740ed00b".U || wb_reg_inst === "h740ed00b".U ){}


  
/*  printf ("[checkcachecounter] regs### %d %d %d rd %d  id_raddr(0,1) %d %d id_vraddr(0,1) %d %d id_rs %x %x id_vrs %x %x %x   \n", id_raddr1, id_raddr2, id_raddr3, id_waddr, id_raddr(0), id_raddr(1), id_vraddr(0), id_vraddr(1), id_rs(0), id_rs(1), id_vrs(0), id_vrs(1), id_vrs(2))

 printf("[checkcachecounter]TAG wb_reg_xcpt %b ⇒ mem_xcpt %b [[mem_reg_xcpt_interrupt %b || mem_reg_xcpt %b]  [mem_reg_valid %b && mem_npc_misaligned %b] [mem_reg_valid %b && mem_ldst_xcpt %b] ]  && !take_pc_wb \n", wb_reg_xcpt, mem_xcpt, mem_xcpt, mem_reg_xcpt_interrupt, mem_reg_xcpt, mem_reg_valid, mem_npc_misaligned, mem_reg_valid, mem_ldst_xcpt, !take_pc_wb)

  printf("[checkcachecounter]TAG mem_reg_xcpt %b = [!ctrl_killx  %b && ex_xcpt %b]   mem_reg_xcpt_interrupt %b := [!take_pc_mem_wb %b && ex_reg_xcpt_interrupt %b] \n", mem_reg_xcpt, !ctrl_killx, ex_xcpt, mem_reg_xcpt_interrupt, !take_pc_mem_wb, ex_reg_xcpt_interrupt)

  printf("[checkcachecounter]TAG ex_xcpt %b [ex_reg_xcpt_interrupt %b || ex_reg_xcpt %b [!ctrl_killd %b && id_xcpt %b]] \n", ex_xcpt, ex_reg_xcpt_interrupt, ex_reg_xcpt, !ctrl_killd, id_xcpt)
  printf("[checkcachecounter]TAG csr.io.interrupt %b bpu.io.debug_if %b bpu.io.xcpt_if %b pf.inst %b  ae.inst %b pf.inst %b ae.inst %b id_illegal_insn %b \n", csr.io.interrupt, bpu.io.debug_if, bpu.io.xcpt_if, id_xcpt0.pf.inst, id_xcpt0.ae.inst, id_xcpt1.pf.inst, id_xcpt1.ae.inst, id_illegal_insn)
 */
//  printf("[checkcachecounter]S1_DAAATA bypass part ex_reg_valid %b && ex_ctrl.wxd %b && !ex_ctrl.vec %b ex_waddr %x mem_reg_wdata %x em_reg_valid %b && mem_ctrl.wxd %b && !mem_ctrl.mem %b && !mem_ctrl.vec %b mem_waddr %x wb_reg_wdata %x dcache_bypass_data %x\n", ex_reg_valid, ex_ctrl.wxd, !ex_ctrl.vec, ex_waddr, mem_reg_wdata, mem_reg_valid,mem_ctrl.wxd,!mem_ctrl.mem,!mem_ctrl.vec, mem_waddr, wb_reg_wdata, dcache_bypass_data)
  //bypass vector cache zazad begins
 // val bypass_source_num_elements = Mux(number_of_elements === UInt(8), v_mem_reg_wdata, v_alu_out)

//  when (id_inst(0) === "h02bb560b".U || id_inst(0)  === "h00985073".U ||  id_inst(0)   === "h000c558b".U || id_inst(0)   === "h02ba570b".U || id_inst(0)   === "h02b9d78b".U || id_inst(0)   === "h80f75257".U || id_inst(0) === "h26bb500b".U ||  ex_reg_inst  === "h00985073".U ||  ex_reg_inst  === "h000c558b".U || ex_reg_inst  === "h02ba570b".U || ex_reg_inst  === "h02b9d78b".U || ex_reg_inst === "h02bb560b".U || ex_reg_inst  === "h80f75257".U || ex_reg_inst === "h26bb500b".U || mem_reg_inst === "h00985073".U ||  mem_reg_inst === "h000c558b".U || mem_reg_inst === "h02ba570b".U || mem_reg_inst === "h02bb560b".U ||  mem_reg_inst === "h02b9d78b".U || mem_reg_inst === "h80f75257".U || mem_reg_inst === "h26bb500b".U ||  wb_reg_inst  === "h00985073".U ||  wb_reg_inst  === "h000c558b".U || wb_reg_inst  === "h02ba570b".U  || wb_reg_inst  === "h02b9d78b".U || wb_reg_inst  === "h80f75257".U || wb_reg_inst === "h26bb500b".U || wb_reg_inst === "h02bb560b".U ){
//when (id_inst(0) === "h00879073".U || id_inst(0) === "h00979073".U  || id_inst(0) === "h009f1073".U  || id_inst(0) === "h00941073".U  || id_inst(0) === "h0005c58b".U  || id_inst(0) === "h0007d60b".U  || id_inst(0) === "h0006d70b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h7406d00b".U  || id_inst(0) === "h0007d60b".U  || id_inst(0) === "h0003d70b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h7403d00b".U  || id_inst(0) === "h0007560b".U  || id_inst(0) === "h0006d68b".U  || id_inst(0) === "h02d9570b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h76d9500b".U  || id_inst(0) === "h0005560b".U  || id_inst(0) === "h000ed68b".U  || id_inst(0) === "h02d9570b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h76d9500b".U  || ex_reg_inst === "h00879073".U   || ex_reg_inst === "h00979073".U   || ex_reg_inst === "h009f1073".U   || ex_reg_inst === "h00941073".U   || ex_reg_inst === "h0005c58b".U   || ex_reg_inst === "h0007d60b".U   || ex_reg_inst === "h0006d70b".U   || ex_reg_inst === "h72c5976b".U   || ex_reg_inst === "h7406d00b".U   || ex_reg_inst === "h0007d60b".U   || ex_reg_inst === "h0003d70b".U  || ex_reg_inst === "h72c5976b".U  || ex_reg_inst === "h7403d00b".U  || ex_reg_inst === "h0007560b".U  || ex_reg_inst === "h0006d68b".U  || ex_reg_inst === "h02d9570b".U  || ex_reg_inst === "h72c5976b".U  || ex_reg_inst === "h76d9500b".U  || ex_reg_inst === "h0005560b".U  || ex_reg_inst === "h000ed68b".U  || ex_reg_inst === "h02d9570b".U  || ex_reg_inst === "h72c5976b".U  || ex_reg_inst === "h76d9500b".U || mem_reg_inst === "h00879073".U  || mem_reg_inst === "h00979073".U  || mem_reg_inst === "h009f1073".U  || mem_reg_inst === "h00941073".U  || mem_reg_inst === "h0005c58b".U  || mem_reg_inst === "h0007d60b".U  || mem_reg_inst === "h0006d70b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h7406d00b".U  || mem_reg_inst === "h0007d60b".U  || mem_reg_inst === "h0003d70b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h7403d00b".U  || mem_reg_inst === "h0007560b".U  || mem_reg_inst === "h0006d68b".U  || mem_reg_inst === "h02d9570b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h76d9500b".U  || mem_reg_inst === "h0005560b".U  || mem_reg_inst === "h000ed68b".U  || mem_reg_inst === "h02d9570b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h76d9500b".U || wb_reg_inst === "h00879073".U  || wb_reg_inst === "h00979073".U  || wb_reg_inst === "h009f1073".U  || wb_reg_inst === "h00941073".U  || wb_reg_inst === "h0005c58b".U  || wb_reg_inst === "h0007d60b".U  || wb_reg_inst === "h0006d70b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h7406d00b".U  || wb_reg_inst === "h0007d60b".U  || wb_reg_inst === "h0003d70b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h7403d00b".U  || wb_reg_inst === "h0007560b".U  || wb_reg_inst === "h0006d68b".U  || wb_reg_inst === "h02d9570b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h76d9500b".U  || wb_reg_inst === "h0005560b".U  || wb_reg_inst === "h000ed68b".U  || wb_reg_inst === "h02d9570b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h76d9500b".U){

/*   when (id_inst(0) === "h00881073".U || id_inst(0) === "h00929073".U || id_inst(0) === "h00999073".U || id_inst(0) === "h0007c58b".U || id_inst(0) === "h0008560b".U || id_inst(0) === "h22c5926b".U || id_inst(0) === "h240e500b".U || id_inst(0) === "h2407d00b".U || id_inst(0) === "h0040425b".U || ex_reg_inst === "h00881073".U || ex_reg_inst === "h00929073".U || ex_reg_inst === "h00999073".U || ex_reg_inst  === "h0007c58b".U || ex_reg_inst === "h0008560b".U || ex_reg_inst === "h22c5926b".U || ex_reg_inst === "h240e500b".U || ex_reg_inst === "h2407d00b".U || ex_reg_inst === "h0040425b".U || mem_reg_inst === "h00881073".U || mem_reg_inst === "h00929073".U || mem_reg_inst === "h00999073".U || mem_reg_inst === "h0007c58b".U || mem_reg_inst === "h0008560b".U || mem_reg_inst === "h22c5926b".U || mem_reg_inst === "h240e500b".U || mem_reg_inst === "h2407d00b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h00881073".U || wb_reg_inst === "h00929073".U || wb_reg_inst === "h00999073".U || wb_reg_inst === "h0007c58b".U || wb_reg_inst === "h0008560b".U || wb_reg_inst === "h22c5926b".U || wb_reg_inst === "h240e500b".U ||  wb_reg_inst  === "h2407d00b".U || wb_reg_inst === "h0040425b".U){
 */

/* when(id_inst(0) === "h0007d20b".U || ex_reg_inst ===  "h0007d20b".U || mem_reg_inst === "h0007d20b".U || wb_reg_inst === "h0007d20b".U ||id_inst(0) === "h00971073".U || ex_reg_inst ===  "h00971073".U || mem_reg_inst === "h00971073".U || wb_reg_inst === "h00971073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2403500b".U || ex_reg_inst === "h2403500b".U || mem_reg_inst === "h2403500b".U || wb_reg_inst === "h2403500b".U ||id_inst(0) === "h00991073".U || ex_reg_inst ===  "h00991073".U || mem_reg_inst === "h00991073".U || wb_reg_inst === "h00991073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U){

    // val vtest = vector_id_bypass_src(0)
    //val vnext_test = vector_id_bypass_src(1)
    //val vnext_next_test = vector_id_bypass_src(2)
//    printf("[checkcachecounter]@@@@@@@@@@@@@@@@@@@@@ vec_bypass_source_conds %b %b %b \n", (ex_reg_valid || (number_of_elements === UInt(16) && !ex_ctrl.mem && vector_unit.io.resp.valid)) && ex_ctrl.wxd  && ex_ctrl.vec, ((mem_reg_valid && !ctrl_stalld) || (wb_reg_valid && !ctrl_stalld))  && mem_ctrl.wxd && !mem_ctrl.mem  && mem_ctrl.vec,((mem_reg_valid && !ctrl_stalld) || (wb_reg_valid && !ctrl_stalld))  && mem_ctrl.wxd  && mem_ctrl.vec)
  // printf("[checkcachecounter]@@@@@@@@@@@@@@@@@@@@@@vectorbypass (ex_reg_valid %b || numelements=16 %b !exctrl.mem %b && vecunitrespvalid %b ) && ex_ctrl.wxd %b vec %b  ex_waddr %d mem_reg_wdata %x \n", ex_reg_valid, number_of_elements === UInt(16),!ex_ctrl.mem,vector_unit.io.resp.valid,ex_ctrl.wxd,ex_ctrl.vec, ex_waddr,v_mem_reg_wdata)
  // printf("[checkcachecounter]@@@@@@@@@@@@@@@@@@@@@@vectorbypass (mem_reg_valid %b || wb_reg_valid %b && !ctrl_stalld %b)&& mem_ctrl.wxd %b && !mem_ctrl.mem %b vec %b  mem_waddr %d wb_reg_wdata %x \n", mem_reg_valid, wb_reg_valid,!ctrl_stalld,  mem_ctrl.wxd , !mem_ctrl.mem, mem_ctrl.vec, mem_waddr, v_wb_reg_wdata)
  // printf("[checkcachecounter]@@@@@@@@@@@@@@@@@@@@@@vectorbypass (mem_reg_valid %b || wb_reg_valid %b && !ctrl_stalld %b) && mem_ctrl.wxd %b vec %b mem_waddr %d dcache_bypass_data %x [%b %b %x %x] \n", mem_reg_valid, wb_reg_valid,!ctrl_stalld, mem_ctrl.wxd,mem_ctrl.vec, mem_waddr, vector_dcache_bypass_data, fastLoadByte, fastLoadWord, io.dmem.resp.bits.DcacheCpu_data_word_bypass, v_wb_reg_wdata)
      //printf("[checkcachecounter]vectorbypass id_bypass_src [0] %b %b %b %b [1] %b %b %b %b [2] %b %b %b %b id_raddr1 %d id_raddr2 %d \n",vtest(0),vtest(1), vtest(2), vtest(3), vnext_test(0),vnext_test(1),vnext_test(2),vnext_test(3),vnext_next_test(0), vnext_next_test(1),vnext_next_test(2),vnext_next_test(3),id_vraddr(0), id_vraddr(1))
     
    }
 */

//    printf("[checkcachecounter]@@@@@@@@@@@@@@@@@@@@@@@ bypass id_raddr %x %x ex_rs %x %x id_addr.size %d bypass ex_reg_rs_bypass(i 0) %b bypass_mux(ex_reg_rs_lsb(i) %x) %x Cat(ex_reg_rs_msb(i) %x, ex_reg_rs_lsb(i) %x) %x\n",id_raddr(0), id_raddr(1),ex_rs(0), ex_rs(1), id_vraddr.size, ex_reg_rs_bypass(0),ex_reg_rs_lsb(0), bypass_mux(v_ex_reg_rs_lsb(0)), ex_reg_rs_msb(0),ex_reg_rs_lsb(0), Cat(ex_reg_rs_msb(0), ex_reg_rs_lsb(0)))
  // printf("[checkcachecounter]@@@@@@@@@@@@@@@@@@@@@@@@ bypass ex_reg_rs_bypass(i 1) %b bypass_mux(ex_reg_rs_lsb(i) %x) %x Cat(ex_reg_rs_msb(i) %x, ex_reg_rs_lsb(i) %x) %x\n", ex_reg_rs_bypass(1), ex_reg_rs_lsb(1), bypass_mux(ex_reg_rs_lsb(1)), ex_reg_rs_msb(1),ex_reg_rs_lsb(1), Cat(ex_reg_rs_msb(1), ex_reg_rs_lsb(1)))
//     printf("[checkcachecounter]@@@@@@@@@@@@@@@@@@@@@@@vectorbypass ex_reg_rs_bypass(i 1) %b bypass_mux(ex_reg_rs_lsb(i) %x) %x Cat(ex_reg_rs_msb(i) %x, ex_reg_rs_lsb(i) %x) %x\n", ex_reg_rs_bypass(2), ex_reg_rs_lsb(2), bypass_mux(ex_reg_rs_lsb(2)), ex_reg_rs_msb(2),ex_reg_rs_lsb(2), Cat(ex_reg_rs_msb(2), ex_reg_rs_lsb(2)))
  
/* when(id_inst(0) === "h0007d20b".U || ex_reg_inst ===  "h0007d20b".U || mem_reg_inst === "h0007d20b".U || wb_reg_inst === "h0007d20b".U ||id_inst(0) === "h00971073".U || ex_reg_inst ===  "h00971073".U || mem_reg_inst === "h00971073".U || wb_reg_inst === "h00971073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2403500b".U || ex_reg_inst === "h2403500b".U || mem_reg_inst === "h2403500b".U || wb_reg_inst === "h2403500b".U ||id_inst(0) === "h00991073".U || ex_reg_inst ===  "h00991073".U || mem_reg_inst === "h00991073".U || wb_reg_inst === "h00991073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U){

      //  printf("[checkcachecounter]@@@@@@@@@@@@@@@@@@@@@@ id_vrs(0) %x id_vrs(1) %x id_vrs(3) %x \n", id_vrs(0), id_vrs(1),id_vrs(2))
       // printf("[checkcachecounter]@@@@@@@@@@@@@@@@@@@@@@@ id_vaddr.size %d vectorbypass ex_reg_rs_bypass(i 0) %b bypass_mux(ex_reg_rs_lsb(i) %x) %x Cat(ex_reg_rs_msb(i) %x, ex_reg_rs_lsb(i) %x) %x\n",id_vraddr.size, v_ex_reg_rs_bypass(0),v_ex_reg_rs_lsb(0), vector_bypass_mux(v_ex_reg_rs_lsb(0)), v_ex_reg_rs_msb(0),v_ex_reg_rs_lsb(0), Cat(v_ex_reg_rs_msb(0), v_ex_reg_rs_lsb(0)))
      // printf("[checkcachecounter]@@@@@@@@@@@@@@@@@@@@@@@@ vectorbypass ex_reg_rs_bypass(i 1) %b bypass_mux(ex_reg_rs_lsb(i) %x) %x Cat(ex_reg_rs_msb(i) %x, ex_reg_rs_lsb(i) %x) %x\n", v_ex_reg_rs_bypass(1), v_ex_reg_rs_lsb(1), vector_bypass_mux(v_ex_reg_rs_lsb(1)), v_ex_reg_rs_msb(1),v_ex_reg_rs_lsb(1), Cat(v_ex_reg_rs_msb(1), v_ex_reg_rs_lsb(1)))
       // printf("[checkcachecounter]@@@@@@@@@@@@@@@@@@@@@@@vectorbypass ex_reg_rs_bypass(i 1) %b bypass_mux(ex_reg_rs_lsb(i) %x) %x Cat(ex_reg_rs_msb(i) %x, ex_reg_rs_lsb(i) %x) %x\n", v_ex_reg_rs_bypass(2), v_ex_reg_rs_lsb(2), vector_bypass_mux(v_ex_reg_rs_lsb(2)), v_ex_reg_rs_msb(2),v_ex_reg_rs_lsb(2), Cat(v_ex_reg_rs_msb(2), v_ex_reg_rs_lsb(2)))
  
 }*/

//      when (id_inst(0) === "h02bb560b".U || id_inst(0)  === "h00985073".U ||  id_inst(0)   === "h000c558b".U || id_inst(0)   === "h02ba570b".U || id_inst(0)   === "h02b9d78b".U || id_inst(0)   === "h80f75257".U || id_inst(0) === "h26bb500b".U ||  ex_reg_inst  === "h00985073".U ||  ex_reg_inst  === "h000c558b".U || ex_reg_inst  === "h02ba570b".U || ex_reg_inst  === "h02b9d78b".U || ex_reg_inst === "h02bb560b".U || ex_reg_inst  === "h80f75257".U || ex_reg_inst === "h26bb500b".U || mem_reg_inst === "h00985073".U ||  mem_reg_inst === "h000c558b".U || mem_reg_inst === "h02ba570b".U || mem_reg_inst === "h02bb560b".U ||  mem_reg_inst === "h02b9d78b".U || mem_reg_inst === "h80f75257".U || mem_reg_inst === "h26bb500b".U ||  wb_reg_inst  === "h00985073".U ||  wb_reg_inst  === "h000c558b".U || wb_reg_inst  === "h02ba570b".U  || wb_reg_inst  === "h02b9d78b".U || wb_reg_inst  === "h80f75257".U || wb_reg_inst === "h26bb500b".U || wb_reg_inst === "h02bb560b".U ){
//       when (id_inst(0) === "h00879073".U || id_inst(0) === "h00979073".U  || id_inst(0) === "h009f1073".U  || id_inst(0) === "h00941073".U  || id_inst(0) === "h0005c58b".U  || id_inst(0) === "h0007d60b".U  || id_inst(0) === "h0006d70b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h7406d00b".U  || id_inst(0) === "h0007d60b".U  || id_inst(0) === "h0003d70b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h7403d00b".U  || id_inst(0) === "h0007560b".U  || id_inst(0) === "h0006d68b".U  || id_inst(0) === "h02d9570b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h76d9500b".U  || id_inst(0) === "h0005560b".U  || id_inst(0) === "h000ed68b".U  || id_inst(0) === "h02d9570b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h76d9500b".U  || ex_reg_inst === "h00879073".U   || ex_reg_inst === "h00979073".U   || ex_reg_inst === "h009f1073".U   || ex_reg_inst === "h00941073".U   || ex_reg_inst === "h0005c58b".U   || ex_reg_inst === "h0007d60b".U   || ex_reg_inst === "h0006d70b".U   || ex_reg_inst === "h72c5976b".U   || ex_reg_inst === "h7406d00b".U   || ex_reg_inst === "h0007d60b".U   || ex_reg_inst === "h0003d70b".U  || ex_reg_inst === "h72c5976b".U  || ex_reg_inst === "h7403d00b".U  || ex_reg_inst === "h0007560b".U  || ex_reg_inst === "h0006d68b".U  || ex_reg_inst === "h02d9570b".U  || ex_reg_inst === "h72c5976b".U  || ex_reg_inst === "h76d9500b".U  || ex_reg_inst === "h0005560b".U  || ex_reg_inst === "h000ed68b".U  || ex_reg_inst === "h02d9570b".U  || ex_reg_inst === "h72c5976b".U  || ex_reg_inst === "h76d9500b".U || mem_reg_inst === "h00879073".U  || mem_reg_inst === "h00979073".U  || mem_reg_inst === "h009f1073".U  || mem_reg_inst === "h00941073".U  || mem_reg_inst === "h0005c58b".U  || mem_reg_inst === "h0007d60b".U  || mem_reg_inst === "h0006d70b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h7406d00b".U  || mem_reg_inst === "h0007d60b".U  || mem_reg_inst === "h0003d70b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h7403d00b".U  || mem_reg_inst === "h0007560b".U  || mem_reg_inst === "h0006d68b".U  || mem_reg_inst === "h02d9570b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h76d9500b".U  || mem_reg_inst === "h0005560b".U  || mem_reg_inst === "h000ed68b".U  || mem_reg_inst === "h02d9570b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h76d9500b".U || wb_reg_inst === "h00879073".U  || wb_reg_inst === "h00979073".U  || wb_reg_inst === "h009f1073".U  || wb_reg_inst === "h00941073".U  || wb_reg_inst === "h0005c58b".U  || wb_reg_inst === "h0007d60b".U  || wb_reg_inst === "h0006d70b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h7406d00b".U  || wb_reg_inst === "h0007d60b".U  || wb_reg_inst === "h0003d70b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h7403d00b".U  || wb_reg_inst === "h0007560b".U  || wb_reg_inst === "h0006d68b".U  || wb_reg_inst === "h02d9570b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h76d9500b".U  || wb_reg_inst === "h0005560b".U  || wb_reg_inst === "h000ed68b".U  || wb_reg_inst === "h02d9570b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h76d9500b".U){
/* when (id_inst(0) === "h00881073".U || id_inst(0) === "h00929073".U || id_inst(0) === "h00999073".U || id_inst(0) === "h0007c58b".U || id_inst(0) === "h0008560b".U || id_inst(0) === "h22c5926b".U || id_inst(0) === "h240e500b".U || id_inst(0) === "h2407d00b".U || id_inst(0) === "h0040425b".U || ex_reg_inst === "h00881073".U || ex_reg_inst === "h00929073".U || ex_reg_inst === "h00999073".U || ex_reg_inst  === "h0007c58b".U || ex_reg_inst === "h0008560b".U || ex_reg_inst === "h22c5926b".U || ex_reg_inst === "h240e500b".U || ex_reg_inst === "h2407d00b".U || ex_reg_inst === "h0040425b".U || mem_reg_inst === "h00881073".U || mem_reg_inst === "h00929073".U || mem_reg_inst === "h00999073".U || mem_reg_inst === "h0007c58b".U || mem_reg_inst === "h0008560b".U || mem_reg_inst === "h22c5926b".U || mem_reg_inst === "h240e500b".U || mem_reg_inst === "h2407d00b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h00881073".U || wb_reg_inst === "h00929073".U || wb_reg_inst === "h00999073".U || wb_reg_inst === "h0007c58b".U || wb_reg_inst === "h0008560b".U || wb_reg_inst === "h22c5926b".U || wb_reg_inst === "h240e500b".U ||  wb_reg_inst  === "h2407d00b".U || wb_reg_inst === "h0040425b".U){
        
//        printf("[checkcachecounter] id_vaddr.size %d vectorbypass ex_reg_rs_bypass(i 0) %b bypass_mux(ex_reg_rs_lsb(i) %x) %x Cat(ex_reg_rs_msb(i) %x, ex_reg_rs_lsb(i) %x) %x\n",id_vraddr.size, v_ex_reg_rs_bypass(0),v_ex_reg_rs_lsb(0), vector_bypass_mux(v_ex_reg_rs_lsb(0)), v_ex_reg_rs_msb(0),v_ex_reg_rs_lsb(0), Cat(v_ex_reg_rs_msb(0), v_ex_reg_rs_lsb(0)))
  //     printf("[checkcachecounter]vectorbypass ex_reg_rs_bypass(i 1) %b bypass_mux(ex_reg_rs_lsb(i) %x) %x Cat(ex_reg_rs_msb(i) %x, ex_reg_rs_lsb(i) %x) %x\n", v_ex_reg_rs_bypass(1), v_ex_reg_rs_lsb(1), vector_bypass_mux(v_ex_reg_rs_lsb(1)), v_ex_reg_rs_msb(1),v_ex_reg_rs_lsb(1), Cat(v_ex_reg_rs_msb(1), v_ex_reg_rs_lsb(1)))
    //    printf("[checkcachecounter]vectorbypass ex_reg_rs_bypass(i 1) %b bypass_mux(ex_reg_rs_lsb(i) %x) %x Cat(ex_reg_rs_msb(i) %x, ex_reg_rs_lsb(i) %x) %x\n", v_ex_reg_rs_bypass(2), v_ex_reg_rs_lsb(2), vector_bypass_mux(v_ex_reg_rs_lsb(2)), v_ex_reg_rs_msb(2),v_ex_reg_rs_lsb(2), Cat(v_ex_reg_rs_msb(2), v_ex_reg_rs_lsb(2)))
          
   }
 */

  //csr printf("[checkcachecounter]PPPPPPRRRRRRRRROOOOOOOOOCCCCCCEEEESSSSOOOORR io.cpu.req.bits.number_of_elements %x number_of_elements %x \n", vector_unit.io.req.bits.number_of_elements, number_of_elements)

//         when (id_inst(0) === "h02bb560b".U || id_inst(0)  === "h00985073".U ||  id_inst(0)   === "h000c558b".U || id_inst(0)   === "h02ba570b".U || id_inst(0)   === "h02b9d78b".U || id_inst(0)   === "h80f75257".U || id_inst(0) === "h26bb500b".U ||  ex_reg_inst  === "h00985073".U ||  ex_reg_inst  === "h000c558b".U || ex_reg_inst  === "h02ba570b".U || ex_reg_inst  === "h02b9d78b".U || ex_reg_inst === "h02bb560b".U || ex_reg_inst  === "h80f75257".U || ex_reg_inst === "h26bb500b".U || mem_reg_inst === "h00985073".U ||  mem_reg_inst === "h000c558b".U || mem_reg_inst === "h02ba570b".U || mem_reg_inst === "h02bb560b".U ||  mem_reg_inst === "h02b9d78b".U || mem_reg_inst === "h80f75257".U || mem_reg_inst === "h26bb500b".U ||  wb_reg_inst  === "h00985073".U ||  wb_reg_inst  === "h000c558b".U || wb_reg_inst  === "h02ba570b".U  || wb_reg_inst  === "h02b9d78b".U || wb_reg_inst  === "h80f75257".U || wb_reg_inst === "h26bb500b".U || wb_reg_inst === "h02bb560b".U ){


        
  printf("[checkcachecounter] id_raddr %d %d %d ex reg1 %x reg2 %x reg3 %x valid %b\n", id_raddr1, id_raddr2, id_raddr3, ex_vrs(0), ex_vrs(1), ex_vrs(2), vector_unit.io.req.valid)

//}
//  printf("[checkcachecounter] id_raddr %d %d %d ex reg1 %x reg2 %x reg3 %x valid %b\n", id_raddr1, id_raddr2, id_raddr3, ex_vrs(0), ex_vrs(1), ex_vrs(2), vector_unit.io.req.valid)

  /*  when (id_inst(0) === "h02bb560b".U || id_inst(0)  === "h00985073".U ||  id_inst(0)   === "h000c558b".U || id_inst(0)   === "h02ba570b".U || id_inst(0)   === "h02b9d78b".U || id_inst(0)   === "h80f75257".U || id_inst(0) === "h26bb500b".U ||  ex_reg_inst  === "h00985073".U ||  ex_reg_inst  === "h000c558b".U || ex_reg_inst  === "h02ba570b".U || ex_reg_inst  === "h02b9d78b".U || ex_reg_inst === "h02bb560b".U || ex_reg_inst  === "h80f75257".U || ex_reg_inst === "h26bb500b".U || mem_reg_inst === "h00985073".U ||  mem_reg_inst === "h000c558b".U || mem_reg_inst === "h02ba570b".U || mem_reg_inst === "h02bb560b".U ||  mem_reg_inst === "h02b9d78b".U || mem_reg_inst === "h80f75257".U || mem_reg_inst === "h26bb500b".U ||  wb_reg_inst  === "h00985073".U ||  wb_reg_inst  === "h000c558b".U || wb_reg_inst  === "h02ba570b".U  || wb_reg_inst  === "h02b9d78b".U || wb_reg_inst  === "h80f75257".U || wb_reg_inst === "h26bb500b".U || wb_reg_inst === "h02bb560b".U ){


         //printf ("[checkcachecounter]~~~~~~~~~~vectorbypass bypass id_vraddr i %d vex_reg_rss(i) %x := id_vrs(i) %x id_vec %b inst %x ex_vec %b inst %x  !ctrl_killd %b || (ctrl_stalld %b && ex_ctrl.vec %b) \n ",i, ex_reg_rss(i),id_vrs(i), id_ctrl.vec,id_inst(0), ex_ctrl.vec, ex_reg_inst, !ctrl_killd,ctrl_stalld,ex_ctrl.vec)                                                                               

         //printf ("[checkcachecounter]~~~~~~~~~~~vectorbypass i %d vdo_bypass %b v_ex_reg_rs_bypass(i) %b  vid_bypass_src %b %b %b  vbypass_src %d %x [0] vex_reg_rs_lsb %x vex_reg_rs_msb %x [1] vex_reg_rs_lsb %x vex_reg_rs_msb %x  \n", i,v_do_bypass,v_ex_reg_rs_bypass(i),v_test_next(0), v_test_next(1),v_test_next(2), v_bypass_src,v_bypass_src,v_ex_reg_rs_lsb(0),v_ex_reg_rs_msb(0),v_ex_reg_rs_lsb(1),v_ex_reg_rs_msb(1))                                                                                                                                                                                          
 
       } 
   */


  //   when (id_inst(0) === "h02bb560b".U || id_inst(0)  === "h00985073".U ||  id_inst(0)   === "h000c558b".U || id_inst(0)   === "h02ba570b".U || id_inst(0)   === "h02b9d78b".U || id_inst(0)   === "h80f75257".U || id_inst(0) === "h26bb500b".U ||  ex_reg_inst  === "h00985073".U ||  ex_reg_inst  === "h000c558b".U || ex_reg_inst  === "h02ba570b".U || ex_reg_inst  === "h02b9d78b".U || ex_reg_inst === "h02bb560b".U || ex_reg_inst  === "h80f75257".U || ex_reg_inst === "h26bb500b".U || mem_reg_inst === "h00985073".U ||  mem_reg_inst === "h000c558b".U || mem_reg_inst === "h02ba570b".U || mem_reg_inst === "h02bb560b".U ||  mem_reg_inst === "h02b9d78b".U || mem_reg_inst === "h80f75257".U || mem_reg_inst === "h26bb500b".U ||  wb_reg_inst  === "h00985073".U ||  wb_reg_inst  === "h000c558b".U || wb_reg_inst  === "h02ba570b".U  || wb_reg_inst  === "h02b9d78b".U || wb_reg_inst  === "h80f75257".U || wb_reg_inst === "h26bb500b".U || wb_reg_inst === "h02bb560b".U ){
//when (id_inst(0) === "h00879073".U || id_inst(0) === "h00979073".U  || id_inst(0) === "h009f1073".U  || id_inst(0) === "h00941073".U  || id_inst(0) === "h0005c58b".U  || id_inst(0) === "h0007d60b".U  || id_inst(0) === "h0006d70b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h7406d00b".U  || id_inst(0) === "h0007d60b".U  || id_inst(0) === "h0003d70b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h7403d00b".U  || id_inst(0) === "h0007560b".U  || id_inst(0) === "h0006d68b".U  || id_inst(0) === "h02d9570b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h76d9500b".U  || id_inst(0) === "h0005560b".U  || id_inst(0) === "h000ed68b".U  || id_inst(0) === "h02d9570b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h76d9500b".U  || ex_reg_inst === "h00879073".U   || ex_reg_inst === "h00979073".U   || ex_reg_inst === "h009f1073".U   || ex_reg_inst === "h00941073".U   || ex_reg_inst === "h0005c58b".U   || ex_reg_inst === "h0007d60b".U   || ex_reg_inst === "h0006d70b".U   || ex_reg_inst === "h72c5976b".U   || ex_reg_inst === "h7406d00b".U   || ex_reg_inst === "h0007d60b".U   || ex_reg_inst === "h0003d70b".U  || ex_reg_inst === "h72c5976b".U  || ex_reg_inst === "h7403d00b".U  || ex_reg_inst === "h0007560b".U  || ex_reg_inst === "h0006d68b".U  || ex_reg_inst === "h02d9570b".U  || ex_reg_inst === "h72c5976b".U  || ex_reg_inst === "h76d9500b".U  || ex_reg_inst === "h0005560b".U  || ex_reg_inst === "h000ed68b".U  || ex_reg_inst === "h02d9570b".U  || ex_reg_inst === "h72c5976b".U  || ex_reg_inst === "h76d9500b".U || mem_reg_inst === "h00879073".U  || mem_reg_inst === "h00979073".U  || mem_reg_inst === "h009f1073".U  || mem_reg_inst === "h00941073".U  || mem_reg_inst === "h0005c58b".U  || mem_reg_inst === "h0007d60b".U  || mem_reg_inst === "h0006d70b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h7406d00b".U  || mem_reg_inst === "h0007d60b".U  || mem_reg_inst === "h0003d70b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h7403d00b".U  || mem_reg_inst === "h0007560b".U  || mem_reg_inst === "h0006d68b".U  || mem_reg_inst === "h02d9570b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h76d9500b".U  || mem_reg_inst === "h0005560b".U  || mem_reg_inst === "h000ed68b".U  || mem_reg_inst === "h02d9570b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h76d9500b".U || wb_reg_inst === "h00879073".U  || wb_reg_inst === "h00979073".U  || wb_reg_inst === "h009f1073".U  || wb_reg_inst === "h00941073".U  || wb_reg_inst === "h0005c58b".U  || wb_reg_inst === "h0007d60b".U  || wb_reg_inst === "h0006d70b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h7406d00b".U  || wb_reg_inst === "h0007d60b".U  || wb_reg_inst === "h0003d70b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h7403d00b".U  || wb_reg_inst === "h0007560b".U  || wb_reg_inst === "h0006d68b".U  || wb_reg_inst === "h02d9570b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h76d9500b".U  || wb_reg_inst === "h0005560b".U  || wb_reg_inst === "h000ed68b".U  || wb_reg_inst === "h02d9570b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h76d9500b".U){

//  when (id_inst(0) === "h00881073".U || id_inst(0) === "h00929073".U || id_inst(0) === "h00999073".U || id_inst(0) === "h0007c58b".U || id_inst(0) === "h0008560b".U || id_inst(0) === "h22c5926b".U || id_inst(0) === "h240e500b".U || id_inst(0) === "h2407d00b".U || id_inst(0) === "h0040425b".U || ex_reg_inst === "h00881073".U || ex_reg_inst === "h00929073".U || ex_reg_inst === "h00999073".U || ex_reg_inst  === "h0007c58b".U || ex_reg_inst === "h0008560b".U || ex_reg_inst === "h22c5926b".U || ex_reg_inst === "h240e500b".U || ex_reg_inst === "h2407d00b".U || ex_reg_inst === "h0040425b".U || mem_reg_inst === "h00881073".U || mem_reg_inst === "h00929073".U || mem_reg_inst === "h00999073".U || mem_reg_inst === "h0007c58b".U || mem_reg_inst === "h0008560b".U || mem_reg_inst === "h22c5926b".U || mem_reg_inst === "h240e500b".U || mem_reg_inst === "h2407d00b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h00881073".U || wb_reg_inst === "h00929073".U || wb_reg_inst === "h00999073".U || wb_reg_inst === "h0007c58b".U || wb_reg_inst === "h0008560b".U || wb_reg_inst === "h22c5926b".U || wb_reg_inst === "h240e500b".U ||  wb_reg_inst  ==="h2407d00b".U || wb_reg_inst === "h0040425b".U){
    //when(id_inst(0) === "h0005c58b".U || ex_reg_inst === "h0005c58b".U || mem_reg_inst === "h0005c58b".U || wb_reg_inst === "h0005c58b".U){
//when(id_inst(0) === "h0007c58b".U || ex_reg_inst === "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U || id_inst(0) === "h0008560b".U || ex_reg_inst === "h0008560b".U || mem_reg_inst === "h0008560b".U || wb_reg_inst === "h0008560b".U || id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U || id_inst(0) === "h".U || ex_reg_inst === "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U){
//when(id_inst(0) === "h0007c58b".U || ex_reg_inst === "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U || id_inst(0) === "h0008560b".U || ex_reg_inst === "h0008560b".U || mem_reg_inst === "h0008560b".U || wb_reg_inst === "h0008560b".U || id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U || id_inst(0) === "h0007c58b".U || ex_reg_inst === "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U || ibuf.io.pc === "h6b6da".U ){
// when( id_inst(0) === "h0005c58b".U || ex_reg_inst === "h0005c58b".U || mem_reg_inst === "h0005c58b".U || wb_reg_inst === "h0005c58b".U || id_inst(0) === "h00959073".U || ex_reg_inst === "h00959073".U || mem_reg_inst === "h00959073".U || wb_reg_inst === "h00959073".U || id_inst(0) === "h000c558b".U || ex_reg_inst === "h000c558b".U || mem_reg_inst === "h000c558b".U || wb_reg_inst === "h000c558b".U || id_inst(0) === "h000a470b".U || ex_reg_inst === "h000a470b".U || mem_reg_inst === "h000a470b".U || wb_reg_inst === "h000a470b".U || id_inst(0) === "h0009c78b".U || ex_reg_inst === "h0009c78b".U || mem_reg_inst === "h0009c78b".U || wb_reg_inst === "h0009c78b".U || id_inst(0) === "h72f7126b".U || ex_reg_inst === "h72f7126b".U || mem_reg_inst === "h72f7126b".U || wb_reg_inst === "h72f7126b".U || id_inst(0) === "h26bb500b".U || ex_reg_inst === "h26bb500b".U || mem_reg_inst === "h26bb500b".U || wb_reg_inst === "h26bb500b".U || id_inst(0) === "h000a470b".U || ex_reg_inst === "h000a470b".U || mem_reg_inst === "h000a470b".U || wb_reg_inst === "h000a470b".U || id_inst(0) === "h0009c78b".U || ex_reg_inst === "h0009c78b".U || mem_reg_inst === "h0009c78b".U || wb_reg_inst === "h0009c78b".U || id_inst(0) === "h80f75257".U || ex_reg_inst === "h80f75257".U || mem_reg_inst === "h80f75257".U || wb_reg_inst === "h80f75257".U || id_inst(0) === "h240bd00b".U || ex_reg_inst === "h240bd00b".U || mem_reg_inst === "h240bd00b".U || wb_reg_inst === "h240bd00b".U ){

   /*when(id_inst(0) === "h00879073".U || ex_reg_inst ===  "h00879073".U || mem_reg_inst === "h00879073".U || wb_reg_inst === "h00879073".U || id_inst(0) === "h000fc58b".U || ex_reg_inst === "h000fc58b".U || mem_reg_inst === "h000fc58b".U || wb_reg_inst === "h000fc58b".U || id_inst(0) === "h0007560b".U || ex_reg_inst === "h0007560b".U || mem_reg_inst === "h0007560b".U || wb_reg_inst === "h0007560b".U || id_inst(0) === "h0006d68b".U || ex_reg_inst === "h0006d68b".U || mem_reg_inst === "h0006d68b".U || wb_reg_inst === "h0006d68b".U || id_inst(0) === "h02d9570b".U || ex_reg_inst === "h02d9570b".U || mem_reg_inst === "h02d9570b".U || wb_reg_inst === "h02d9570b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h76d9500b".U || ex_reg_inst === "h76d9500b".U || mem_reg_inst === "h76d9500b".U || wb_reg_inst === "h76d9500b".U || id_inst(0) === "h00979073".U || ex_reg_inst === "h00979073".U || mem_reg_inst === "h00979073".U || wb_reg_inst === "h00979073".U || id_inst(0) === "h0005560b".U || ex_reg_inst === "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U || id_inst(0) === "h000ed68b".U || ex_reg_inst === "h000ed68b".U || mem_reg_inst === "h000ed68b".U || wb_reg_inst === "h000ed68b".U || id_inst(0) === "h02d9570b".U || ex_reg_inst === "h02d9570b".U || mem_reg_inst === "h02d9570b".U || wb_reg_inst === "h02d9570b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h76d9500b".U || ex_reg_inst === "h76d9500b".U || mem_reg_inst === "h76d9500b".U || wb_reg_inst === "h76d9500b".U || id_inst(0) === "h0005c58b".U || ex_reg_inst === "h0005c58b".U || mem_reg_inst === "h0005c58b".U || wb_reg_inst === "h0005c58b".U || id_inst(0) === "h00879073".U || ex_reg_inst === "h00879073".U || mem_reg_inst === "h00879073".U || wb_reg_inst === "h00879073".U || id_inst(0) === "h00979073".U || ex_reg_inst === "h00979073".U || mem_reg_inst === "h00979073".U || wb_reg_inst === "h00979073".U || id_inst(0) === "h0005c58b".U || ex_reg_inst === "h0005c58b".U || mem_reg_inst === "h0005c58b".U || wb_reg_inst === "h0005c58b".U || id_inst(0) === "h0006d70b".U || ex_reg_inst === "h0006d70b".U || mem_reg_inst === "h0006d70b".U || wb_reg_inst === "h0006d70b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h7406d00b".U || ex_reg_inst === "h7406d00b".U || mem_reg_inst === "h7406d00b".U || wb_reg_inst === "h7406d00b".U || id_inst(0) === "h0007d60b".U || ex_reg_inst === "h0007d60b".U || mem_reg_inst === "h0007d60b".U || wb_reg_inst === "h0007d60b".U || id_inst(0) === "h000ed70b".U || ex_reg_inst === "h000ed70b".U || mem_reg_inst === "h000ed70b".U || wb_reg_inst === "h000ed70b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h740ed00b".U || ex_reg_inst === "h740ed00b".U || mem_reg_inst === "h740ed00b".U || wb_reg_inst === "h740ed00b".U ){
    */
//    when(id_inst(0) === "h0007d20b".U || ex_reg_inst ===  "h0007d20b".U || mem_reg_inst === "h0007d20b".U || wb_reg_inst === "h0007d20b".U ||id_inst(0) === "h00971073".U || ex_reg_inst ===  "h00971073".U || mem_reg_inst === "h00971073".U || wb_reg_inst === "h00971073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2403500b".U || ex_reg_inst === "h2403500b".U || mem_reg_inst === "h2403500b".U || wb_reg_inst === "h2403500b".U ||id_inst(0) === "h00991073".U || ex_reg_inst ===  "h00991073".U || mem_reg_inst === "h00991073".U || wb_reg_inst === "h00991073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U || id_inst(0) === "h000fc58b".U || ex_reg_inst === "h000fc58b".U || mem_reg_inst === "h000fc58b".U || wb_reg_inst === "h000fc58b".U || id_inst(0) === "h0005560b".U || ex_reg_inst === "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U  || id_inst(0) === "h000ed68b".U || ex_reg_inst === "h000ed68b".U || mem_reg_inst === "h000ed68b".U || wb_reg_inst === "h000ed68b".U || id_inst(0) === "h02d9570b".U || ex_reg_inst === "h02d9570b".U || mem_reg_inst === "h02d9570b".U || wb_reg_inst === "h02d9570b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h76d9500b".U || ex_reg_inst === "h76d9500b".U || mem_reg_inst === "h76d9500b".U || wb_reg_inst === "h76d9500b".U){





 when(id_inst(0) === "h0007560b".U || ex_reg_inst ===  "h0007560b".U || mem_reg_inst === "h0007560b".U || wb_reg_inst === "h0007560b".U || id_inst(0) === "h00279793".U || ex_reg_inst ===  "h00279793".U || mem_reg_inst === "h00279793".U || wb_reg_inst === "h00279793".U ||id_inst(0) === "h00179713".U || ex_reg_inst ===  "h00179713".U || mem_reg_inst === "h00179713".U || wb_reg_inst === "h00179713".U || id_inst(0) === "h0007d20b".U || ex_reg_inst ===  "h0007d20b".U || mem_reg_inst === "h0007d20b".U || wb_reg_inst === "h0007d20b".U ||id_inst(0) === "h00971073".U || ex_reg_inst ===  "h00971073".U || mem_reg_inst === "h00971073".U || wb_reg_inst === "h00971073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2403500b".U || ex_reg_inst === "h2403500b".U || mem_reg_inst === "h2403500b".U || wb_reg_inst === "h2403500b".U ||id_inst(0) === "h00991073".U || ex_reg_inst ===  "h00991073".U || mem_reg_inst === "h00991073".U || wb_reg_inst === "h00991073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U || id_inst(0) === "h240e500b".U || ex_reg_inst ===  "h240e500b".U || mem_reg_inst === "h240e500b".U || wb_reg_inst === "h240e500b".U || id_inst(0) === "h240b500b".U || ex_reg_inst ===  "h240b500b".U || mem_reg_inst === "h240b500b".U || wb_reg_inst === "h240b500b".U ||id_inst(0) === "h0007d20b".U || ex_reg_inst ===  "h0007d20b".U || mem_reg_inst === "h0007d20b".U || wb_reg_inst === "h0007d20b".U ||id_inst(0) === "h00971073".U || ex_reg_inst ===  "h00971073".U || mem_reg_inst === "h00971073".U || wb_reg_inst === "h00971073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2403500b".U || ex_reg_inst === "h2403500b".U || mem_reg_inst === "h2403500b".U || wb_reg_inst === "h2403500b".U ||id_inst(0) === "h00991073".U || ex_reg_inst ===  "h00991073".U || mem_reg_inst === "h00991073".U || wb_reg_inst === "h00991073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U || id_inst(0) === "h000fc58b".U || ex_reg_inst === "h000fc58b".U || mem_reg_inst === "h000fc58b".U || wb_reg_inst === "h000fc58b".U || id_inst(0) === "h0005560b".U || ex_reg_inst === "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U  || id_inst(0) === "h000ed68b".U || ex_reg_inst === "h000ed68b".U || mem_reg_inst === "h000ed68b".U || wb_reg_inst === "h000ed68b".U || id_inst(0) === "h02d9570b".U || ex_reg_inst === "h02d9570b".U || mem_reg_inst === "h02d9570b".U || wb_reg_inst === "h02d9570b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h76d9500b".U || ex_reg_inst === "h76d9500b".U || mem_reg_inst === "h76d9500b".U || wb_reg_inst === "h76d9500b".U){

  printf("[checkcachecounter]%%%%%% offset_vld %d offset_vsd %d cacheaccesstype %b cnt_cache_vsd %d caches1validvalue %b stride [vsd %d vld %d] ctrlstalldcond %b locks %b ctrl_stalld %b s1_data %x [v %x s %x] ex_vrs(1) %x \n",offset, offset_vsd, io.dmem.req.bits.vector_cache_access_type ,cnt_cache_vsd, cache_s1_valid_value_vsd, stride_vsd,stride_vld, ctrl_stalld_cond_vsd, locks_vsd, ctrl_stalld,io.dmem.DcacheCpu_s1_data.data, v_mem_reg_rs2, mem_reg_rs2, ex_vrs(1))
    printf("[checkcachecounter]checkreplay valireq %b  vsd %b vld %b [%b %b posedge %b %b]edgeready_assign %b  reg_ready %b old_ready %b newready %b \n",cache_s1_valid_value,  replay_after_miss_vsd, replay_after_miss, wb_ctrl.vec, wb_ctrl.mem , edgeready, wb_ctrl.wxd, !old_ready && io.dmem.req.ready, Reg(io.dmem.req.ready), old_ready, io.dmem.req.ready)
 
  printf("[checkcachecounterfuncs] csr.io.vl %d element# %d Riiiiiiiiight just strides ****** prev_ctr-stalld %b larray [vld %x vsd %x] <== ex_vrs %x %x ex_rs %d %d constantstrided [vld %d vsd %d]   unit_stride %b [ex_raddr2(%d) ===0]  reg_stride_vld %x reg_vsd_stride %x \n", csr.io.vl,number_of_elements, prev_ctrl_stalld,stride_vld_array, stride_vsd_array,ex_vrs(0), ex_vrs(1),ex_rs(0), ex_rs(1),stride_vld, stride_vsd, ex_raddr2 === UInt(0),ex_raddr2, reg_stride_vld_array, reg_stride_vsd_array)
  printf("[checkcachecounter]PPPPPPRRRRRRRRROOOOOOOOOCCCCCCEEEESSSSOOOORR io.cpu.req.bits.number_of_elements %x number_of_elements %x \n", vector_unit.io.req.bits.number_of_elements, number_of_elements) 
  printf("[checkcachecounterfuncs] wb_ctrl.mem %b wb_ctrl.vec %b io.dmem.resp.valid %b io.dmem.resp.bits.addr %x return_addr %x extracted_bits %x cnt_cache %d offset %d (%d) vrf_offset %d tempvrf %x vrf %x\n",wb_ctrl.mem,wb_ctrl.vec,io.dmem.resp.valid,io.dmem.resp.bits.addr, io.dmem.resp.bits.return_addr, extracted_bits, cnt_cache, offset_bit_extraction, io.dmem.resp.bits.return_addr(4,0) * UInt(8),offset_vrf_reg, temp_vrf_mem_value, vrf_mem_value)
   printf("[checkcachecounter] +++++++++++++++++++ vwb_wen %b addr %x (cond %b) [vwb_wen_gather %b  vwb_wen_unit_stride %b] \n", vwb_wen,vrf_waddr,stride_vld === UInt(1) && !wb_ctrl.scatter_gather, vwb_wen_gather, vwb_wen_unit_stride)
   printf("[checkcachecounter] +++++++++++++++++++ vrf_wdata_gather %x cond(%b) [vrf_mem_value %x  v_alu_out %x] \n", vrf_wdata_gather, io.dmem.resp.valid  && (reg_cnt_cache === number_of_elements - UInt(1)), vrf_mem_value (255, 0), v_alu_out)
   printf("[checkcachecounter] ++++++++++++++++++ vrf_wdata_unit_stride %x cond(%b) [vrf_mem_value %x  cond(%b)[v_wb_reg_wdata %x v_alu_out %x]]\n", vrf_wdata_unit_stride, io.dmem.resp.valid, vrf_mem_value(255, 0), number_of_elements <= number_of_lanes, v_wb_reg_wdata, v_alu_out)
   printf("[checkcachecounter] ++++++++++++++++++ vrf_wdata %x cond(%b [%b || %b && %b]) [vrf_wdata_unit_stride %x  vrf_wdata_gather %x]\n", vrf_wdata, (stride_vld === UInt(1) || wb_ctrl.vec_scalar) && !wb_ctrl.scatter_gather,stride_vld === UInt(1), wb_ctrl.vec_scalar, !wb_ctrl.scatter_gather, vrf_wdata_unit_stride, vrf_wdata_gather)
  printf("[checkcachecounter]@@@@@checkpointwriteintovectoregfile stride %d vrf_wdata %x [g %x u %x] vwb_wen %b [g %b [u %b cnt=1 %b || vec_scalar %b]] wb_waddr %d reg_dmem_resp_valid %b io.dmem.req.ready %b [iodmemrespv %b data %x] [dmemrespvalid %b iodmem %b] cnt_cache %d reg_cnt_cache %d vrf_mem_value %x v_wb_reg_wdata %x \n", stride_vld, vrf_wdata, vrf_wdata_gather, vrf_wdata_unit_stride, vwb_wen, vwb_wen_gather, vwb_wen_unit_stride, cnt_cache === UInt(1), wb_ctrl.vec_scalar, wb_waddr, reg_dmem_resp_valid, io.dmem.req.ready, io.dmem.resp.valid, io.dmem.resp.bits.DcacheCpu_data(255, 0),dmem_resp_valid, io.dmem.resp.valid,  cnt_cache, reg_cnt_cache, vrf_mem_value, v_wb_reg_wdata)
  
     printf("[checkcachecounter]@@@@@@checkpoint vector_resp_valid %b v_alu_out %x %x %x %x ||||||||||||(cond g %b us %b stride %b)==>to write into vregfile\n", vector_unit.io.resp.valid, v_alu_out, v_mem_reg_wdata, v_mem_int_wdata, v_wb_reg_wdata, reg_dmem_resp_valid && (reg_cnt_cache === number_of_elements - UInt(1)), io.dmem.resp.valid, (stride_vld === UInt(1) && !wb_ctrl.scatter_gather))

        printf("[checkcachecounterfuncs]^^^^^^^^^^^ctrl_stalld %b  id_exhaz %b  id_memhaz %b id_wbhaz %b id_sboard %b  \n", ctrl_stalld, id_ex_hazard, id_mem_hazard, id_wb_hazard, id_sboard_hazard)
  
     printf("[checkcachecounterfuncs]^^^^^^^^^^^mem_reg_valid %b && mem_reg_flush_pipe %b ==> %b @ex_pc_valid %b (%b %b %b) [ex_inst %x => mem_inst %x]@ mem_pc_valid %b (%b %b %b) [mem_inst %x => wb_inst %x]@\n", mem_reg_valid, mem_reg_flush_pipe, mem_reg_valid && mem_reg_flush_pipe, ex_pc_valid, ex_reg_valid, ex_reg_replay, ex_reg_xcpt_interrupt, ex_reg_inst, mem_reg_inst, mem_pc_valid, mem_reg_valid, mem_reg_replay, mem_reg_xcpt_interrupt, mem_reg_inst, wb_reg_inst)
  
     printf("[checkcachecounterfuncs]^^^^^^^^^^^exregflush %b [%b %b] memregflushpip %b ctrl_killd %b ctrl_killx %b ctrl_killm %b exregecptinterrupt %b [%b %b %b] memregxcptinterruprt %b [%b %b]\n", ex_reg_flush_pipe,id_ctrl.fence_i, id_csr_flush, mem_reg_flush_pipe, ctrl_killd, ctrl_killx, ctrl_killm, ex_reg_xcpt_interrupt, !take_pc, ibuf.io.inst(0).valid, csr.io.interrupt,mem_reg_xcpt_interrupt, !take_pc_mem_wb, ex_reg_xcpt_interrupt)
  
     printf("[checkcachecounter] stall_vec_exe %b = Mux(number_of_elements > UInt(8) %b, id_ctrl.vec %b && !vector_unit.io.req.ready %b || vector_unit.io.req.valid %b ==>result %b, ex_ctrl.vec %b && !vector_unit.io.req.ready %b \n", stall_vec_exe, number_of_elements > UInt(8), id_ctrl.vec, !vector_unit.io.req.ready, vector_unit.io.req.valid, (id_ctrl.vec && !vector_unit.io.req.ready ) || vector_unit.io.req.valid ,ex_ctrl.vec, !vector_unit.io.req.ready)

         printf("[checkcachecounter]@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@[ex_inst %x] offset %x io.dmem.req.bits  element_numebr %x addr %x tag %x cmd %x typ %x vector_cacheaccesstype %x cnt_cache_vsd %x el# %x is_cache_access_vec %x \n", ex_reg_inst ,offset_vsd, io.dmem.req.bits.addr,io.dmem.req.bits.element_number, io.dmem.req.bits.tag, io.dmem.req.bits.cmd, io.dmem.req.bits.typ, io.dmem.req.bits.vector_cache_access_type, io.dmem.req.bits.cnt_cache_vsd, io.dmem.req.bits.element_number, io.dmem.req.bits.is_cache_access_vec)
  printf("[checkcachecounterfuncs] ctrl_stalld %b [%b %b %b %b %b %b %b %b %b %b %b] vld[ctrl_stalled_cond %b locks %b] vsd[ctrl_stalled_cond %b locks %b] ex_ctrl.vec %b ex_ctrl.mem %b ex_reg_valid %b cnt_cache != 7 %b cnt_cache %d s1_data %x mem_reg_flush_pipe %b exceptions[%b %b %b %b %b %b]\n",ctrl_stalld,id_ex_hazard || id_mem_hazard || id_wb_hazard || id_sboard_hazard, csr.io.singleStep && (ex_reg_valid || mem_reg_valid || wb_reg_valid),id_ctrl.fp && id_stall_fpu,   id_ctrl.mem && dcache_blocked,id_ctrl.rocc && rocc_blocked, id_ctrl.div && (!(div.io.req.ready || (div.io.resp.valid && !wb_wxd)) || div.io.req.valid), stall_vec_exe, id_do_fence, csr.io.csr_stall, ctrl_stalld_cond_vld || locks_vld, ctrl_stalld_cond_vsd || locks_vsd, ctrl_stalld_cond_vld, locks_vld, ctrl_stalld_cond_vsd, locks_vsd, ex_ctrl.vec, ex_ctrl.mem,ex_reg_valid, cnt_cache != 7, cnt_cache, io.dmem.DcacheCpu_s1_data.data, mem_reg_flush_pipe, take_pc, id_xcpt, ex_xcpt, mem_ldst_xcpt, mem_xcpt, wb_xcpt)
   
  printf("[checkcachecounter] mem_br_taken %b alu.io.cmp_out %b branch [%b %b %b %b  %x %x %b] take_pc[%b %b %b %b] fence %b replay [ex %b mem %b mem_reg_replay %b] xcpt [%b memint %b ex_int %b wb_exception %b wb_cause %x] ctrl_kill[%b %b %b %b] kill_v[%b %b] killm_common_vlsd_vssd [dcache_kill_mem %b|| take_pc_wb %b|| mem_reg_xcpt %b] dmem_replay_next %b dcache_blocked %b dmem_s1_kill %b io.dmem.resp.valid %b dmem_resp %x\n",mem_br_taken, alu.io.cmp_out, mem_direction_misprediction, mem_misprediction, mem_wrong_npc, mem_cfi_taken, mem_npc, ex_reg_pc, ex_pc_valid, take_pc, take_pc_mem, take_pc_wb, take_pc_wb, mem_reg_sfence, replay_ex, replay_mem, mem_reg_replay, mem_reg_xcpt, mem_reg_xcpt_interrupt, ex_reg_xcpt_interrupt,wb_xcpt, wb_cause, ctrl_killd, ctrl_killx, ctrl_killm, ctrl_stalld, ctrl_killd_vlsd_vssd, killm_common_vlsd_vssd, dcache_kill_mem, take_pc_wb, mem_reg_xcpt,io.dmem.replay_next, dcache_blocked, io.dmem.s1_kill, io.dmem.resp.valid, io.dmem.resp.bits.DcacheCpu_data(255,0))
printf("[checkcachecounter]TAG mem_reg_wdat %x wb_reg_wdata %x csr.io.tval %x  tval_valid %b encodeVirtualAddress(wb_reg_wdata, wb_reg_wdata %x) %x dmem.io.ready %x mem_npc %x [mem_ctrl.jalr %b || mem_reg_sfence %b , encodeVirtualAddress(mem_reg_wdata, mem_reg_wdata %x).asSInt %x, mem_br_target %x]  mem_wrong_npc %b [ ex_pc_vali %b mem_npc %x =/= ex_reg_pc %x, ibuf.io.inst(0).valid %b || ibuf.io.imem.valid %b, mem_npc %x =/= ibuf.io.pc %x]   mem_reg_wdata %x := alu.io.out %x  mem_br_taken %b := alu.io.cmp_out %b\n", mem_reg_wdata, wb_reg_wdata, csr.io.tval, tval_valid,wb_reg_wdata, encodeVirtualAddress(wb_reg_wdata, wb_reg_wdata), io.dmem.req.ready, mem_npc, mem_ctrl.jalr, mem_reg_sfence, mem_reg_wdata, encodeVirtualAddress(mem_reg_wdata, mem_reg_wdata).asSInt, mem_br_target, mem_wrong_npc, ex_pc_valid, mem_npc, ex_reg_pc, ibuf.io.inst(0).valid, ibuf.io.imem.valid, mem_npc, ibuf.io.pc,  mem_reg_wdata, alu.io.out, mem_br_taken, alu.io.cmp_out)

   printf("[checkcachecounterfuncs] ctrl_staldl %b [line0 %b line1 %b] take_pc %b [[%b pc %x] id_inst %x ==> DASM(%x)] [[%b pc %x] ex %x ==> DASM(%x)] [[ %b pc %x] mem %x ==> DASM(%x)] [[pc %x] wb %x ==> DASM(%x)] vwb_wen %b csr.io.pc %x\n",ctrl_stalld, ((ctrl_stalld_cond_vld || locks_vld) && !take_pc),  ((ctrl_stalld_cond_vsd || locks_vsd) && !take_pc), take_pc, !ctrl_killd || csr.io.interrupt || ibuf.io.inst(0).bits.replay, ibuf.io.pc, id_inst(0),id_inst(0), ex_pc_valid, ex_reg_pc, ex_reg_inst,ex_reg_inst, mem_pc_valid, mem_reg_pc, mem_reg_inst,mem_reg_inst, wb_reg_pc, wb_reg_inst,wb_reg_inst,vwb_wen, csr.io.pc /*,vrf_mem_value*/)

printf("[checkcachecounter]TAG wb_reg_xcpt %b > mem_xcpt %b [[mem_reg_xcpt_interrupt %b || mem_reg_xcpt %b]  [mem_reg_valid %b  && mem_npc_misaligned %b] [mem_reg_valid %b && mem_ldst_xcpt %b] ]  && !take_pc_wb %b \n", wb_reg_xcpt, mem_xcpt, mem_reg_xcpt_interrupt, mem_reg_xcpt, mem_reg_valid, mem_npc_misaligned, mem_reg_valid, mem_ldst_xcpt, !take_pc_wb)

  printf("[checkcachecounter]TAG mem_reg_xcpt %b = [!ctrl_killx  %b && ex_xcpt %b]   mem_reg_xcpt_interrupt %b := [!take_pc_mem_wb %b && ex_reg_xcpt_interrupt %b] \n", mem_reg_xcpt, !ctrl_killx, ex_xcpt, mem_reg_xcpt_interrupt, !take_pc_mem_wb, ex_reg_xcpt_interrupt)

  printf("[checkcachecounter]TAG ex_xcpt %b [ex_reg_xcpt_interrupt %b || ex_reg_xcpt %b [!ctrl_killd %b && id_xcpt %b]]  csr.io.retire %b := wb_valid %b  \n", ex_xcpt, ex_reg_xcpt_interrupt, ex_reg_xcpt, !ctrl_killd, id_xcpt, csr.io.retire , wb_valid)
  printf("[checkcachecounter]TAG csr.io.interrupt %b bpu.io.debug_if %b bpu.io.xcpt_if %b pf.inst %b  ae.inst %b pf.inst %b ae.inst %b id_illegal_insn %b \n", csr.io.interrupt, bpu.io.debug_if, bpu.io.xcpt_if, id_xcpt0.pf.inst, id_xcpt0.ae.inst, id_xcpt1.pf.inst, id_xcpt1.ae.inst, id_illegal_insn)
  printf ("[checkcachecounter] regs### %d %d %d rd %d  id_raddr(0,1) %d %d id_vraddr(0,1) %d %d id_rs %x %x id_vrs %x %x %x   \n", id_raddr1, id_raddr2, id_raddr3, id_waddr, id_raddr(0), id_raddr(1), id_vraddr(0), id_vraddr(1), id_rs(0), id_rs(1), id_vrs(0), id_vrs(1), id_vrs(2))

  printf("[checkcachecounter]TAG mem_reg_xcpt %b = [!ctrl_killx  %b && ex_xcpt %b]   mem_reg_xcpt_interrupt %b := [!take_pc_mem_wb %b && ex_reg_xcpt_interrupt %b] \n", mem_reg_xcpt, !ctrl_killx, ex_xcpt, mem_reg_xcpt_interrupt, !take_pc_mem_wb, ex_reg_xcpt_interrupt)

  printf("[checkcachecounter]TAG ex_xcpt %b [ex_reg_xcpt_interrupt %b || ex_reg_xcpt %b [!ctrl_killd %b && id_xcpt %b]] \n", ex_xcpt, ex_reg_xcpt_interrupt, ex_reg_xcpt, !ctrl_killd, id_xcpt)
  printf("[checkcachecounter]TAG csr.io.interrupt %b bpu.io.debug_if %b bpu.io.xcpt_if %b pf.inst %b  ae.inst %b pf.inst %b ae.inst %b id_illegal_insn %b \n", csr.io.interrupt, bpu.io.debug_if, bpu.io.xcpt_if, id_xcpt0.pf.inst, id_xcpt0.ae.inst, id_xcpt1.pf.inst, id_xcpt1.ae.inst, id_illegal_insn)

 printf("[checkcachecounter]IMMMMMMMMEM ibuf.io.pc.asSInt %x ibuf.io.inst(0).bits.xcpt0 [pf.inst %b ae.inst %b]  ibuf.io.inst(0).bits.xcpt1 [pf.inst %b ae.inst %b] id_xcpt %b id_cause %x csr.io.interrupt %b bpu.io.debug_if %b bpu.io.xcpt_if %b id_xcpt0.pf.inst %b id_xcpt0.ae.inst %b id_xcpt1.pf.inst %b id_xcpt1.ae.inst %b id_illegal_insn %b  ||||| ibuf.io.inst(0).valid %b ibuf.io.inst(0).bits.replay %b ||||||| ctrl_killd %b\n", ibuf.io.pc.asSInt, ibuf.io.inst(0).bits.xcpt0.pf.inst, ibuf.io.inst(0).bits.xcpt0.ae.inst, ibuf.io.inst(0).bits.xcpt1.pf.inst, ibuf.io.inst(0).bits.xcpt1.ae.inst, id_xcpt, id_cause, csr.io.interrupt,  bpu.io.debug_if, bpu.io.xcpt_if, id_xcpt0.pf.inst, id_xcpt0.ae.inst, id_xcpt1.pf.inst, id_xcpt1.ae.inst, id_illegal_insn, ibuf.io.inst(0).valid, ibuf.io.inst(0).bits.replay, ctrl_killd)

   printf("[checkcachecounter]IMMMMMMMMEM  ctrl_killd %b ctrl_killd_vlsd_vssd %b io.imem.req.valid %b take_pc %b io.imem.req.bits.speculative %b take_pc_wb %b io.imem.req.bits.pc %x wb_xcpt %b csr.io.eret %b csr.io.evec %x replay_wb %b wb_reg_pc %x mem_npc %x io.imem.flush_icache %b  wb_reg_valid %b && wb_ctrl.fence_i %b && !io.dmem.s2_nack %b]  io.imem.sfence.valid %b :=[ wb_reg_valid %b && wb_reg_sfence %b] io.imem.sfence.bits.rs1 %b := wb_ctrl.mem_type(0) %x io.imem.sfence.bits.rs2 %x := wb_ctrl.mem_type(1) %x io.imem.sfence.bits.addr %x := wb_reg_wdata %x io.imem.sfence.bits.asid %x := wb_reg_rs2 %x  ibuf.io.inst(0).ready %b !ctrl_stalld %b  icache_blocked %b  io.imem.resp.valid %b \n", ctrl_killd, ctrl_killd_vlsd_vssd, io.imem.req.valid, take_pc, io.imem.req.bits.speculative, take_pc_wb, io.imem.req.bits.pc, wb_xcpt, csr.io.eret, csr.io.evec, replay_wb, wb_reg_pc, mem_npc, io.imem.flush_icache, wb_reg_valid, wb_ctrl.fence, !io.dmem.s2_nack,io.imem.sfence.valid, wb_reg_valid, wb_reg_sfence, io.imem.sfence.bits.rs1, wb_ctrl.mem_type(0), io.imem.sfence.bits.rs2, wb_ctrl.mem_type(1), io.imem.sfence.bits.addr, wb_reg_wdata,io.imem.sfence.bits.asid, wb_reg_rs2, ibuf.io.inst(0).ready, !ctrl_stalld, icache_blocked,io.imem.resp.valid)

  printf("[checkcachecounter]FINAL io.dmem.req.valid %b RAM %b caches1valid %b ex_reg_valid_vlsd %b io.dmem.s1_kill %b kill_common_vlsd %b killm_common %b mem_reg_valid %b killm_common [dcache_kill_mem %b take_pc_wb %b mem_reg_xcpt %b] mem_ldst_xcpt %b ex_reg_valid %b io.dmem [replay_next %b s2_nack %b] \n" , io.dmem.req.valid, replay_after_miss_vsd, cache_s1_valid_value_vsd, ex_reg_valid_vlsd_vssd, io.dmem.s1_kill, killm_common_vlsd_vssd, killm_common, mem_reg_valid, dcache_kill_mem,take_pc_wb,mem_reg_xcpt,  mem_ldst_xcpt, ex_reg_valid, io.dmem.replay_next, io.dmem.s2_nack)
 printf("[checkcachecounter]FINAL replay_wb %b = replay_wb_common %b || replay_wb_rocc %b  replay_wb_common [%b && ! %b || %b]  take_pc_wb %b [%b || %b || %b || %b] wb_reg_flush_pipe %b[! %b && %b] ctrl_killm %b [%b || %b || %b] killm_common %b [%b || %b || %b || ! %b]  mem_reg_xcpt %b [! %b & %b] ctrl_killx [%b || %b || ! %b] ex_xcpt%b  [%b || %b]\n", replay_wb, replay_wb_common, replay_wb_rocc, io.dmem.s2_nack, wb_ctrl.vec, wb_reg_replay, take_pc_wb, replay_wb, wb_xcpt, csr.io.eret, wb_reg_flush_pipe, wb_reg_flush_pipe, ctrl_killm, mem_reg_flush_pipe, ctrl_killm, killm_common, mem_xcpt, fpu_kill_mem, killm_common, dcache_kill_mem, take_pc_wb, mem_reg_xcpt, mem_reg_valid, mem_reg_xcpt, ctrl_killx, ex_xcpt, take_pc_mem_wb, replay_ex, ex_reg_valid, ex_xcpt, ex_reg_xcpt_interrupt, ex_reg_xcpt)

 printf("[checkcachecounter] ******************* send memory request address %x [ex_rs(0) %x alu_io.adder.out %x -> encode %x ] valid %b  s1_data %x \n", io.dmem.req.bits.addr,ex_rs(0), alu.io.adder_out, encodeVirtualAddress(ex_rs(0), alu.io.adder_out), io.dmem.req.valid, io.dmem.DcacheCpu_s1_data.data)
    printf("[checkcachecounter]S1_DAAATA temp_for_dmem_write_data %x mem_ctrl.vec %b v_mem_reg_rs2 %x fLen %x mem_reg_rs2  %x mem_ctrl.fp %b io.fpu.store_data %x ||||| ex_pc_valid %b => when %b ex_ctrl.rxs2 %b && (ex_ctrl.mem %b || ex_ctrl.rocc %b || ex_sfence %b)  ex_ctrl.mem_type %x storegen %x  ||| ex_rs(1) %x ex_vrs(2) %x\n", temp_for_dmem_write_data, mem_ctrl.vec, v_mem_reg_rs2, fLen, mem_reg_rs2, mem_ctrl.fp, io.fpu.store_data, ex_pc_valid, ex_ctrl.rxs2 && (ex_ctrl.mem || ex_ctrl.rocc || ex_sfence) ,ex_ctrl.rxs2, ex_ctrl.mem, ex_ctrl.rocc, ex_sfence, ex_ctrl.mem_type, new StoreGen(Mux(ex_ctrl.rocc, log2Ceil(xLen/8).U, ex_ctrl.mem_type), 0.U, ex_rs(1), coreDataBytes).data, ex_rs(1), ex_vrs(2))

 printf("[checkcachecounter]  CSSSSSSSSSSRRRRRR wb_xcpt %b || csr.io.eret %b, csr.io.evec %x   replay_wb %b wb_reg_pc %x mem_npc %x [mem_ctrl.jalr %b || mem_reg_sfence %b got to  %x else got to %x]io.imem.req.bits.pc %x\n", wb_xcpt, csr.io.eret, csr.io.evec, replay_wb, wb_reg_pc, mem_npc,mem_ctrl.jalr, mem_reg_sfence, encodeVirtualAddress(mem_reg_wdata, mem_reg_wdata).asSInt, mem_br_target,  io.imem.req.bits.pc)

        printf("[checkcachecounter] @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ vrf_waddr %x vrf_wen %b [vwb_wen %b [stride_vld %x cnt_cache %x !wb_ctrl.scatter_gather %b] ==>vwb_wen_unit_stride %b [wb_ctrl.vec_scalar %b !wb_ctrl.scatter_gather %b] ==>vwb_wen_vec_scalar %b vwb_wen_gather %b ]vrf_wdata %x [vrf_wdata_unit_stride %x [number_of_elements <= number_of_lanes %b v_wb_reg_wdata %x v_alu_out %x]vrf_wdata_gather %x]\n", vrf_waddr,vrf_wen, vwb_wen, stride_vld, cnt_cache, !wb_ctrl.scatter_gather, vwb_wen_unit_stride, wb_ctrl.vec_scalar,  !wb_ctrl.scatter_gather, vwb_wen_vec_scalar, vwb_wen_gather,vrf_wdata, vrf_wdata_unit_stride,number_of_elements <= number_of_lanes,v_wb_reg_wdata,v_alu_out,  vrf_wdata_gather)
 
 printf("[checkcachecounter] sscratch %x Branches mem_br_taken %b := alu.io.cmp_out %b mem_br_target %x = mem_reg_pc.asSInt %x + [1 %b %x 2 %b %x 3 %b]   mem_int_wdata %x [%b %x %x <= alu.io.out %x]  mem_cfi %b mem_cfi_taken %b mem_direction_misprediction %b mem_misprediction %b   \n", csr.io.sscratch, mem_br_taken, alu.io.cmp_out,mem_br_target,mem_reg_pc.asSInt, mem_ctrl.branch && mem_br_taken,  ImmGen(IMM_SB, mem_reg_inst), mem_ctrl.jal, ImmGen(IMM_UJ, mem_reg_inst), mem_reg_rvc, mem_int_wdata, !mem_reg_xcpt && (mem_ctrl.jalr ^ mem_npc_misaligned), mem_br_target, mem_reg_wdata.asSInt,alu.io.out, mem_cfi, mem_cfi_taken, mem_direction_misprediction, mem_misprediction)
  
  printf("[checkcachecounter]~~~~~~~~~~~~~~~~~ [ ex_reg_valid %b && ex_ctrl.wxd %b && ex_ctrl.vec %b , ex_waddr %x, v_mem_reg_wdata %x] [(mem_reg_valid %b && !ctrl_stalld %b) && mem_ctrl.wxd %b && !mem_ctrl.mem %b && mem_ctrl.vec %b, mem_waddr %x, v_wb_reg_wdata %x] [(mem_reg_valid %b && !ctrl_stalld %b) && mem_ctrl.wxd %b && mem_ctrl.vec %b , mem_waddr %x ,vector_dcache_bypass_data %x)] \n", ex_reg_valid, ex_ctrl.wxd, ex_ctrl.vec, ex_waddr, v_mem_reg_wdata, mem_reg_valid, !ctrl_stalld, mem_ctrl.wxd, !mem_ctrl.mem , mem_ctrl.vec, mem_waddr, v_wb_reg_wdata, mem_reg_valid, !ctrl_stalld, mem_ctrl.wxd , mem_ctrl.vec, mem_waddr, vector_dcache_bypass_data)
  printf("[checkcachecounter]~~~~~~~~~~~~~~~~~ v_ex_reg_rs_bypass(0) %x (1) %x  vector_bypass_mux(v_ex_reg_rs_lsb(0)) %x (1) %x  Cat(v_ex_reg_rs_msb(0) %x, v_ex_reg_rs_lsb(0) %x) %x  (1) %x , (1) %x => %x\n", v_ex_reg_rs_bypass(0), v_ex_reg_rs_bypass(1), vector_bypass_mux(v_ex_reg_rs_lsb(0)),  vector_bypass_mux(v_ex_reg_rs_lsb(1)), v_ex_reg_rs_msb(0) , v_ex_reg_rs_lsb(0) , Cat(v_ex_reg_rs_msb(0), v_ex_reg_rs_lsb(0)), v_ex_reg_rs_msb(1), v_ex_reg_rs_lsb(1), Cat(v_ex_reg_rs_msb(1), v_ex_reg_rs_lsb(1)))
   
    }
  






//   when (id_inst(0) === "h02bb560b".U || id_inst(0)  === "h00985073".U ||  id_inst(0)   === "h000c558b".U || id_inst(0)   === "h02ba570b".U || id_inst(0)   === "h02b9d78b".U || id_inst(0)   === "h80f75257".U || id_inst(0) === "h26bb500b".U ||  ex_reg_inst  === "h00985073".U ||  ex_reg_inst  === "h000c558b".U || ex_reg_inst  === "h02ba570b".U || ex_reg_inst  === "h02b9d78b".U || ex_reg_inst === "h02bb560b".U || ex_reg_inst  === "h80f75257".U || ex_reg_inst === "h26bb500b".U || mem_reg_inst === "h00985073".U ||  mem_reg_inst === "h000c558b".U || mem_reg_inst === "h02ba570b".U || mem_reg_inst === "h02bb560b".U ||  mem_reg_inst === "h02b9d78b".U || mem_reg_inst === "h80f75257".U || mem_reg_inst === "h26bb500b".U ||  wb_reg_inst  === "h00985073".U ||  wb_reg_inst  === "h000c558b".U || wb_reg_inst  === "h02ba570b".U  || wb_reg_inst  === "h02b9d78b".U || wb_reg_inst  === "h80f75257".U || wb_reg_inst === "h26bb500b".U || wb_reg_inst === "h02bb560b".U ){
//when (id_inst(0) === "h00879073".U || id_inst(0) === "h00979073".U  || id_inst(0) === "h009f1073".U  || id_inst(0) === "h00941073".U  || id_inst(0) === "h0005c58b".U  || id_inst(0) === "h0007d60b".U  || id_inst(0) === "h0006d70b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h7406d00b".U  || id_inst(0) === "h0007d60b".U  || id_inst(0) === "h0003d70b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h7403d00b".U  || id_inst(0) === "h0007560b".U  || id_inst(0) === "h0006d68b".U  || id_inst(0) === "h02d9570b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h76d9500b".U  || id_inst(0) === "h0005560b".U  || id_inst(0) === "h000ed68b".U  || id_inst(0) === "h02d9570b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h76d9500b".U  || ex_reg_inst === "h00879073".U   || ex_reg_inst === "h00979073".U   || ex_reg_inst === "h009f1073".U   || ex_reg_inst === "h00941073".U   || ex_reg_inst === "h0005c58b".U   || ex_reg_inst === "h0007d60b".U   || ex_reg_inst === "h0006d70b".U   || ex_reg_inst === "h72c5976b".U   || ex_reg_inst === "h7406d00b".U   || ex_reg_inst === "h0007d60b".U   || ex_reg_inst === "h0003d70b".U  || ex_reg_inst === "h72c5976b".U  || ex_reg_inst === "h7403d00b".U  || ex_reg_inst === "h0007560b".U  || ex_reg_inst === "h0006d68b".U  || ex_reg_inst === "h02d9570b".U  || ex_reg_inst === "h72c5976b".U  || ex_reg_inst === "h76d9500b".U  || ex_reg_inst === "h0005560b".U  || ex_reg_inst === "h000ed68b".U  || ex_reg_inst === "h02d9570b".U  || ex_reg_inst === "h72c5976b".U  || ex_reg_inst === "h76d9500b".U || mem_reg_inst === "h00879073".U  || mem_reg_inst === "h00979073".U  || mem_reg_inst === "h009f1073".U  || mem_reg_inst === "h00941073".U  || mem_reg_inst === "h0005c58b".U  || mem_reg_inst === "h0007d60b".U  || mem_reg_inst === "h0006d70b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h7406d00b".U  || mem_reg_inst === "h0007d60b".U  || mem_reg_inst === "h0003d70b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h7403d00b".U  || mem_reg_inst === "h0007560b".U  || mem_reg_inst === "h0006d68b".U  || mem_reg_inst === "h02d9570b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h76d9500b".U  || mem_reg_inst === "h0005560b".U  || mem_reg_inst === "h000ed68b".U  || mem_reg_inst === "h02d9570b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h76d9500b".U || wb_reg_inst === "h00879073".U  || wb_reg_inst === "h00979073".U  || wb_reg_inst === "h009f1073".U  || wb_reg_inst === "h00941073".U  || wb_reg_inst === "h0005c58b".U  || wb_reg_inst === "h0007d60b".U  || wb_reg_inst === "h0006d70b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h7406d00b".U  || wb_reg_inst === "h0007d60b".U  || wb_reg_inst === "h0003d70b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h7403d00b".U  || wb_reg_inst === "h0007560b".U  || wb_reg_inst === "h0006d68b".U  || wb_reg_inst === "h02d9570b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h76d9500b".U  || wb_reg_inst === "h0005560b".U  || wb_reg_inst === "h000ed68b".U  || wb_reg_inst === "h02d9570b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h76d9500b".U){

//   when (id_inst(0) === "h00881073".U || id_inst(0) === "h00929073".U || id_inst(0) === "h00999073".U || id_inst(0) === "h0007c58b".U || id_inst(0) === "h0008560b".U || id_inst(0) === "h22c5926b".U || id_inst(0) === "h240e500b".U || id_inst(0) === "h2407d00b".U || id_inst(0) === "h0040425b".U || ex_reg_inst === "h00881073".U || ex_reg_inst === "h00929073".U || ex_reg_inst === "h00999073".U || ex_reg_inst  === "h0007c58b".U || ex_reg_inst === "h0008560b".U || ex_reg_inst === "h22c5926b".U || ex_reg_inst === "h240e500b".U || ex_reg_inst === "h2407d00b".U || ex_reg_inst === "h0040425b".U || mem_reg_inst === "h00881073".U || mem_reg_inst === "h00929073".U || mem_reg_inst === "h00999073".U || mem_reg_inst === "h0007c58b".U || mem_reg_inst === "h0008560b".U || mem_reg_inst === "h22c5926b".U || mem_reg_inst === "h240e500b".U || mem_reg_inst === "h2407d00b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h00881073".U || wb_reg_inst === "h00929073".U || wb_reg_inst === "h00999073".U || wb_reg_inst === "h0007c58b".U || wb_reg_inst === "h0008560b".U || wb_reg_inst === "h22c5926b".U || wb_reg_inst === "h240e500b".U ||  wb_reg_inst  === "h2407d00b".U || wb_reg_inst === "h0040425b".U){
// when(id_inst(0) === "h0005c58b".U || ex_reg_inst === "h0005c58b".U || mem_reg_inst === "h0005c58b".U || wb_reg_inst === "h0005c58b".U){
//when(id_inst(0) === "h0007c58b".U || ex_reg_inst === "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U || id_inst(0) === "h0008560b".U || ex_reg_inst === "h0008560b".U || mem_reg_inst === "h0008560b".U || wb_reg_inst === "h0008560b".U || id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U || id_inst(0) === "h".U || ex_reg_inst === "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ){
//when(id_inst(0) === "h0007c58b".U || ex_reg_inst === "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U || id_inst(0) === "h0008560b".U || ex_reg_inst === "h0008560b".U || mem_reg_inst === "h0008560b".U || wb_reg_inst === "h0008560b".U || id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U || id_inst(0) === "h0007c58b".U || ex_reg_inst === "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U || ibuf.io.pc === "h6b6da".U ){

// when(id_inst(0) === "h0005c58b".U || ex_reg_inst ===  "h0005c58b".U || mem_reg_inst === "h0005c58b".U || wb_reg_inst === "h0005c58b".U || id_inst(0) === "h00959073".U || ex_reg_inst === "h00959073".U || mem_reg_inst === "h00959073".U || wb_reg_inst === "h00959073".U || id_inst(0) === "h000c558b".U || ex_reg_inst === "h000c558b".U || mem_reg_inst === "h000c558b".U || wb_reg_inst === "h000c558b".U || id_inst(0) === "h000a470b".U || ex_reg_inst === "h000a470b".U || mem_reg_inst === "h000a470b".U || wb_reg_inst === "h000a470b".U || id_inst(0) === "h0009c78b".U || ex_reg_inst === "h0009c78b".U || mem_reg_inst === "h0009c78b".U || wb_reg_inst === "h0009c78b".U || id_inst(0) === "h72f7126b".U || ex_reg_inst === "h72f7126b".U || mem_reg_inst === "h72f7126b".U || wb_reg_inst === "h72f7126b".U || id_inst(0) === "h26bb500b".U || ex_reg_inst === "h26bb500b".U || mem_reg_inst === "h26bb500b".U || wb_reg_inst === "h26bb500b".U || id_inst(0) === "h000a470b".U || ex_reg_inst === "h000a470b".U || mem_reg_inst === "h000a470b".U || wb_reg_inst === "h000a470b".U || id_inst(0) === "h0009c78b".U || ex_reg_inst === "h0009c78b".U || mem_reg_inst === "h0009c78b".U || wb_reg_inst === "h0009c78b".U || id_inst(0) === "h80f75257".U || ex_reg_inst === "h80f75257".U || mem_reg_inst === "h80f75257".U || wb_reg_inst === "h80f75257".U || id_inst(0) === "h240bd00b".U || ex_reg_inst === "h240bd00b".U || mem_reg_inst === "h240bd00b".U || wb_reg_inst === "h240bd00b".U ){
/*   when(id_inst(0) === "h00879073".U || ex_reg_inst ===  "h00879073".U || mem_reg_inst === "h00879073".U || wb_reg_inst === "h00879073".U || id_inst(0) === "h000fc58b".U || ex_reg_inst === "h000fc58b".U || mem_reg_inst === "h000fc58b".U || wb_reg_inst === "h000fc58b".U || id_inst(0) === "h0007560b".U || ex_reg_inst === "h0007560b".U || mem_reg_inst === "h0007560b".U || wb_reg_inst === "h0007560b".U || id_inst(0) === "h0006d68b".U || ex_reg_inst === "h0006d68b".U || mem_reg_inst === "h0006d68b".U || wb_reg_inst === "h0006d68b".U || id_inst(0) === "h02d9570b".U || ex_reg_inst === "h02d9570b".U || mem_reg_inst === "h02d9570b".U || wb_reg_inst === "h02d9570b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h76d9500b".U || ex_reg_inst === "h76d9500b".U || mem_reg_inst === "h76d9500b".U || wb_reg_inst === "h76d9500b".U || id_inst(0) === "h00979073".U || ex_reg_inst === "h00979073".U || mem_reg_inst === "h00979073".U || wb_reg_inst === "h00979073".U || id_inst(0) === "h0005560b".U || ex_reg_inst === "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U || id_inst(0) === "h000ed68b".U || ex_reg_inst === "h000ed68b".U || mem_reg_inst === "h000ed68b".U || wb_reg_inst === "h000ed68b".U || id_inst(0) === "h02d9570b".U || ex_reg_inst === "h02d9570b".U || mem_reg_inst === "h02d9570b".U || wb_reg_inst === "h02d9570b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h76d9500b".U || ex_reg_inst === "h76d9500b".U || mem_reg_inst === "h76d9500b".U || wb_reg_inst === "h76d9500b".U || id_inst(0) === "h0005c58b".U || ex_reg_inst === "h0005c58b".U || mem_reg_inst === "h0005c58b".U || wb_reg_inst === "h0005c58b".U || id_inst(0) === "h00879073".U || ex_reg_inst === "h00879073".U || mem_reg_inst === "h00879073".U || wb_reg_inst === "h00879073".U || id_inst(0) === "h00979073".U || ex_reg_inst === "h00979073".U || mem_reg_inst === "h00979073".U || wb_reg_inst === "h00979073".U || id_inst(0) === "h0005c58b".U || ex_reg_inst === "h0005c58b".U || mem_reg_inst === "h0005c58b".U || wb_reg_inst === "h0005c58b".U || id_inst(0) === "h0006d70b".U || ex_reg_inst === "h0006d70b".U || mem_reg_inst === "h0006d70b".U || wb_reg_inst === "h0006d70b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h7406d00b".U || ex_reg_inst === "h7406d00b".U || mem_reg_inst === "h7406d00b".U || wb_reg_inst === "h7406d00b".U || id_inst(0) === "h0007d60b".U || ex_reg_inst === "h0007d60b".U || mem_reg_inst === "h0007d60b".U || wb_reg_inst === "h0007d60b".U || id_inst(0) === "h000ed70b".U || ex_reg_inst === "h000ed70b".U || mem_reg_inst === "h000ed70b".U || wb_reg_inst === "h000ed70b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h740ed00b".U || ex_reg_inst === "h740ed00b".U || mem_reg_inst === "h740ed00b".U || wb_reg_inst === "h740ed00b".U ){
 */
/*      when(id_inst(0) === "h0007d20b".U || ex_reg_inst ===  "h0007d20b".U || mem_reg_inst === "h0007d20b".U || wb_reg_inst === "h0007d20b".U ||id_inst(0) === "h00971073".U || ex_reg_inst ===  "h00971073".U || mem_reg_inst === "h00971073".U || wb_reg_inst === "h00971073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2403500b".U || ex_reg_inst === "h2403500b".U || mem_reg_inst === "h2403500b".U || wb_reg_inst === "h2403500b".U ||id_inst(0) === "h00991073".U || ex_reg_inst ===  "h00991073".U || mem_reg_inst === "h00991073".U || wb_reg_inst === "h00991073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U || id_inst(0) === "h240e500b".U || ex_reg_inst ===  "h240e500b".U || mem_reg_inst === "h240e500b".U || wb_reg_inst === "h240e500b".U || id_inst(0) === "h240b500b".U || ex_reg_inst ===  "h240b500b".U || mem_reg_inst === "h240b500b".U || wb_reg_inst === "h240b500b".U ){

  printf("[checkcachecounterfuncs] wb_ctrl.mem %b wb_ctrl.vec %b io.dmem.resp.valid %b io.dmem.resp.bits.addr %x return_addr %x extracted_bits %x cnt_cache %d offset %d (%d) vrf_offset %d tempvrf %x vrf %x\n",wb_ctrl.mem,wb_ctrl.vec,io.dmem.resp.valid,io.dmem.resp.bits.addr, io.dmem.resp.bits.return_addr, extracted_bits, cnt_cache, offset_bit_extraction, io.dmem.resp.bits.return_addr(4,0) * UInt(8),offset_vrf_reg, temp_vrf_mem_value, vrf_mem_value)

}*/


/*  when(id_inst(0) === "h0007d20b".U || ex_reg_inst ===  "h0007d20b".U || mem_reg_inst === "h0007d20b".U || wb_reg_inst === "h0007d20b".U ||id_inst(0) === "h00971073".U || ex_reg_inst ===  "h00971073".U || mem_reg_inst === "h00971073".U || wb_reg_inst === "h00971073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2403500b".U || ex_reg_inst === "h2403500b".U || mem_reg_inst === "h2403500b".U || wb_reg_inst === "h2403500b".U ||id_inst(0) === "h00991073".U || ex_reg_inst ===  "h00991073".U || mem_reg_inst === "h00991073".U || wb_reg_inst === "h00991073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U){
   // printf("[checkcachecounter] +++++++++++++++++++ vwb_wen %b addr %x (cond %b) [vwb_wen_gather %b  vwb_wen_unit_stride %b] \n", vwb_wen,vrf_waddr,stride_vld === UInt(1) && !wb_ctrl.scatter_gather, vwb_wen_gather, vwb_wen_unit_stride)
//    printf("[checkcachecounter] +++++++++++++++++++ vrf_wdata_gather %x cond(%b) [vrf_mem_value %x  v_alu_out %x] \n", vrf_wdata_gather, io.dmem.resp.valid  && (reg_cnt_cache === number_of_elements - UInt(1)), vrf_mem_value (255, 0), v_alu_out)
  //  printf("[checkcachecounter] ++++++++++++++++++ vrf_wdata_unit_stride %x cond(%b) [vrf_mem_value %x  cond(%b)[v_wb_reg_wdata %x v_alu_out %x]]\n", vrf_wdata_unit_stride, io.dmem.resp.valid, vrf_mem_value(255, 0), number_of_elements <= number_of_lanes, v_wb_reg_wdata, v_alu_out)
    //printf("[checkcachecounter] ++++++++++++++++++ vrf_wdata %x cond(%b [%b || %b && %b]) [vrf_wdata_unit_stride %x  vrf_wdata_gather %x]\n", vrf_wdata, (stride_vld === UInt(1) || wb_ctrl.vec_scalar) && !wb_ctrl.scatter_gather,stride_vld === UInt(1), wb_ctrl.vec_scalar, !wb_ctrl.scatter_gather, vrf_wdata_unit_stride, vrf_wdata_gather)
  }
 */
/*when(id_inst(0) === "h0007d20b".U || ex_reg_inst ===  "h0007d20b".U || mem_reg_inst === "h0007d20b".U || wb_reg_inst === "h0007d20b".U ||id_inst(0) === "h00971073".U || ex_reg_inst ===  "h00971073".U || mem_reg_inst === "h00971073".U || wb_reg_inst === "h00971073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2403500b".U || ex_reg_inst === "h2403500b".U || mem_reg_inst === "h2403500b".U || wb_reg_inst === "h2403500b".U ||id_inst(0) === "h00991073".U || ex_reg_inst ===  "h00991073".U || mem_reg_inst === "h00991073".U || wb_reg_inst === "h00991073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U || id_inst(0) === "h240e500b".U || ex_reg_inst ===  "h240e500b".U || mem_reg_inst === "h240e500b".U || wb_reg_inst === "h240e500b".U || id_inst(0) === "h240b500b".U || ex_reg_inst ===  "h240b500b".U || mem_reg_inst === "h240b500b".U || wb_reg_inst === "h240b500b".U){

  printf("[checkcachecounter]@@@@@checkpointwriteintovectoregfile stride %d vrf_wdata %x [g %x u %x] vwb_wen %b [g %b [u %b cnt=1 %b || vec_scalar %b]] wb_waddr %d reg_dmem_resp_valid %b io.dmem.req.ready %b [iodmemrespv %b data %x] [dmemrespvalid %b iodmem %b] cnt_cache %d reg_cnt_cache %d vrf_mem_value %x v_wb_reg_wdata %x \n", stride_vld, vrf_wdata, vrf_wdata_gather, vrf_wdata_unit_stride, vwb_wen, vwb_wen_gather, vwb_wen_unit_stride, cnt_cache === UInt(1), wb_ctrl.vec_scalar, wb_waddr, reg_dmem_resp_valid, io.dmem.req.ready, io.dmem.resp.valid, io.dmem.resp.bits.DcacheCpu_data(255, 0),dmem_resp_valid, io.dmem.resp.valid,  cnt_cache, reg_cnt_cache, vrf_mem_value, v_wb_reg_wdata)
  
     printf("[checkcachecounter]@@@@@@checkpoint vector_resp_valid %b v_alu_out %x %x %x %x ||||||||||||(cond g %b us %b stride %b)==>to write into vregfile\n", vector_unit.io.resp.valid, v_alu_out, v_mem_reg_wdata, v_mem_int_wdata, v_wb_reg_wdata, reg_dmem_resp_valid && (reg_cnt_cache === number_of_elements - UInt(1)), io.dmem.resp.valid, (stride_vld === UInt(1) && !wb_ctrl.scatter_gather))
      
    }*/
//   when (id_inst(0) === "h02bb560b".U || id_inst(0)  === "h00985073".U ||  id_inst(0)   === "h000c558b".U || id_inst(0)   === "h02ba570b".U || id_inst(0)   === "h02b9d78b".U || id_inst(0)   === "h80f75257".U || id_inst(0) === "h26bb500b".U ||  ex_reg_inst  === "h00985073".U ||  ex_reg_inst  === "h000c558b".U || ex_reg_inst  === "h02ba570b".U || ex_reg_inst  === "h02b9d78b".U || ex_reg_inst === "h02bb560b".U || ex_reg_inst  === "h80f75257".U || ex_reg_inst === "h26bb500b".U || mem_reg_inst === "h00985073".U ||  mem_reg_inst === "h000c558b".U || mem_reg_inst === "h02ba570b".U || mem_reg_inst === "h02bb560b".U ||  mem_reg_inst === "h02b9d78b".U || mem_reg_inst === "h80f75257".U || mem_reg_inst === "h26bb500b".U ||  wb_reg_inst  === "h00985073".U ||  wb_reg_inst  === "h000c558b".U || wb_reg_inst  === "h02ba570b".U  || wb_reg_inst  === "h02b9d78b".U || wb_reg_inst  === "h80f75257".U || wb_reg_inst === "h26bb500b".U || wb_reg_inst === "h02bb560b".U ){
//when (id_inst(0) === "h00879073".U || id_inst(0) === "h00979073".U  || id_inst(0) === "h009f1073".U  || id_inst(0) === "h00941073".U  || id_inst(0) === "h0005c58b".U  || id_inst(0) === "h0007d60b".U  || id_inst(0) === "h0006d70b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h7406d00b".U  || id_inst(0) === "h0007d60b".U  || id_inst(0) === "h0003d70b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h7403d00b".U  || id_inst(0) === "h0007560b".U  || id_inst(0) === "h0006d68b".U  || id_inst(0) === "h02d9570b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h76d9500b".U  || id_inst(0) === "h0005560b".U  || id_inst(0) === "h000ed68b".U  || id_inst(0) === "h02d9570b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h76d9500b".U  || ex_reg_inst === "h00879073".U   || ex_reg_inst === "h00979073".U   || ex_reg_inst === "h009f1073".U   || ex_reg_inst === "h00941073".U   || ex_reg_inst === "h0005c58b".U   || ex_reg_inst === "h0007d60b".U   || ex_reg_inst === "h0006d70b".U   || ex_reg_inst === "h72c5976b".U   || ex_reg_inst === "h7406d00b".U   || ex_reg_inst === "h0007d60b".U   || ex_reg_inst === "h0003d70b".U  || ex_reg_inst === "h72c5976b".U  || ex_reg_inst === "h7403d00b".U  || ex_reg_inst === "h0007560b".U  || ex_reg_inst === "h0006d68b".U  || ex_reg_inst === "h02d9570b".U  || ex_reg_inst === "h72c5976b".U  || ex_reg_inst === "h76d9500b".U  || ex_reg_inst === "h0005560b".U  || ex_reg_inst === "h000ed68b".U  || ex_reg_inst === "h02d9570b".U  || ex_reg_inst === "h72c5976b".U  || ex_reg_inst === "h76d9500b".U || mem_reg_inst === "h00879073".U  || mem_reg_inst === "h00979073".U  || mem_reg_inst === "h009f1073".U  || mem_reg_inst === "h00941073".U  || mem_reg_inst === "h0005c58b".U  || mem_reg_inst === "h0007d60b".U  || mem_reg_inst === "h0006d70b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h7406d00b".U  || mem_reg_inst === "h0007d60b".U  || mem_reg_inst === "h0003d70b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h7403d00b".U  || mem_reg_inst === "h0007560b".U  || mem_reg_inst === "h0006d68b".U  || mem_reg_inst === "h02d9570b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h76d9500b".U  || mem_reg_inst === "h0005560b".U  || mem_reg_inst === "h000ed68b".U  || mem_reg_inst === "h02d9570b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h76d9500b".U || wb_reg_inst === "h00879073".U  || wb_reg_inst === "h00979073".U  || wb_reg_inst === "h009f1073".U  || wb_reg_inst === "h00941073".U  || wb_reg_inst === "h0005c58b".U  || wb_reg_inst === "h0007d60b".U  || wb_reg_inst === "h0006d70b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h7406d00b".U  || wb_reg_inst === "h0007d60b".U  || wb_reg_inst === "h0003d70b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h7403d00b".U  || wb_reg_inst === "h0007560b".U  || wb_reg_inst === "h0006d68b".U  || wb_reg_inst === "h02d9570b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h76d9500b".U  || wb_reg_inst === "h0005560b".U  || wb_reg_inst === "h000ed68b".U  || wb_reg_inst === "h02d9570b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h76d9500b".U){
// when (id_inst(0) === "h00881073".U || id_inst(0) === "h00929073".U || id_inst(0) === "h00999073".U || id_inst(0) === "h0007c58b".U || id_inst(0) === "h0008560b".U || id_inst(0) === "h22c5926b".U || id_inst(0) === "h240e500b".U || id_inst(0) === "h2407d00b".U || id_inst(0) === "h0040425b".U || ex_reg_inst === "h00881073".U || ex_reg_inst === "h00929073".U || ex_reg_inst === "h00999073".U || ex_reg_inst  === "h0007c58b".U || ex_reg_inst === "h0008560b".U || ex_reg_inst === "h22c5926b".U || ex_reg_inst === "h240e500b".U || ex_reg_inst === "h2407d00b".U || ex_reg_inst === "h0040425b".U || mem_reg_inst === "h00881073".U || mem_reg_inst === "h00929073".U || mem_reg_inst === "h00999073".U || mem_reg_inst === "h0007c58b".U || mem_reg_inst === "h0008560b".U || mem_reg_inst === "h22c5926b".U || mem_reg_inst === "h240e500b".U || mem_reg_inst === "h2407d00b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h00881073".U || wb_reg_inst === "h00929073".U || wb_reg_inst === "h00999073".U || wb_reg_inst === "h0007c58b".U || wb_reg_inst === "h0008560b".U || wb_reg_inst === "h22c5926b".U || wb_reg_inst === "h240e500b".U ||  wb_reg_inst  === "h2407d00b".U || wb_reg_inst === "h0040425b".U){
// when(id_inst(0) === "h0005c58b".U || ex_reg_inst === "h0005c58b".U || mem_reg_inst === "h0005c58b".U || wb_reg_inst === "h0005c58b".U){
//when(id_inst(0) === "h0007c58b".U || ex_reg_inst === "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U || id_inst(0) === "h0008560b".U || ex_reg_inst === "h0008560b".U || mem_reg_inst === "h0008560b".U || wb_reg_inst === "h0008560b".U || id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U || id_inst(0) === "h".U || ex_reg_inst === "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U){
//when(id_inst(0) === "h0007c58b".U || ex_reg_inst === "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U || id_inst(0) === "h0008560b".U || ex_reg_inst === "h0008560b".U || mem_reg_inst === "h0008560b".U || wb_reg_inst === "h0008560b".U || id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U || id_inst(0) === "h0007c58b".U || ex_reg_inst === "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U || ibuf.io.pc === "h6b6da".U ){

//   when(id_inst(0) === "h0005c58b".U || ex_reg_inst ===  "h0005c58b".U || mem_reg_inst === "h0005c58b".U || wb_reg_inst === "h0005c58b".U || id_inst(0) === "h00959073".U || ex_reg_inst === "h00959073".U || mem_reg_inst === "h00959073".U || wb_reg_inst === "h00959073".U || id_inst(0) === "h000c558b".U || ex_reg_inst === "h000c558b".U || mem_reg_inst === "h000c558b".U || wb_reg_inst === "h000c558b".U || id_inst(0) === "h0009c70b".U || ex_reg_inst === "h0009c70b".U || mem_reg_inst === "h0009c70b".U || wb_reg_inst === "h0009c70b".U || id_inst(0) === "h0009c78b".U || ex_reg_inst === "h0009c78b".U || mem_reg_inst === "h0009c78b".U || wb_reg_inst === "h0009c78b".U || id_inst(0) === "h72f7126b".U || ex_reg_inst === "h72f7126b".U || mem_reg_inst === "h72f7126b".U || wb_reg_inst === "h72f7126b".U || id_inst(0) === "h26bb500b".U || ex_reg_inst === "h26bb500b".U || mem_reg_inst === "h26bb500b".U || wb_reg_inst === "h26bb500b".U || id_inst(0) === "h0009c70b".U || ex_reg_inst === "h0009c70b".U || mem_reg_inst === "h0009c70b".U || wb_reg_inst === "h0009c70b".U || id_inst(0) === "h0009c78b".U || ex_reg_inst === "h0009c78b".U || mem_reg_inst === "h0009c78b".U || wb_reg_inst === "h0009c78b".U || id_inst(0) === "h80f75257".U || ex_reg_inst === "h80f75257".U || mem_reg_inst === "h80f75257".U || wb_reg_inst === "h80f75257".U || id_inst(0) === "h240bd00b".U || ex_reg_inst === "h240bd00b".U || mem_reg_inst === "h240bd00b".U || wb_reg_inst === "h240bd00b".U ){

/*   when(id_inst(0) === "h00879073".U || ex_reg_inst ===  "h00879073".U || mem_reg_inst === "h00879073".U || wb_reg_inst === "h00879073".U || id_inst(0) === "h000fc58b".U || ex_reg_inst === "h000fc58b".U || mem_reg_inst === "h000fc58b".U || wb_reg_inst === "h000fc58b".U || id_inst(0) === "h0007560b".U || ex_reg_inst === "h0007560b".U || mem_reg_inst === "h0007560b".U || wb_reg_inst === "h0007560b".U || id_inst(0) === "h0006d68b".U || ex_reg_inst === "h0006d68b".U || mem_reg_inst === "h0006d68b".U || wb_reg_inst === "h0006d68b".U || id_inst(0) === "h02d9570b".U || ex_reg_inst === "h02d9570b".U || mem_reg_inst === "h02d9570b".U || wb_reg_inst === "h02d9570b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h76d9500b".U || ex_reg_inst === "h76d9500b".U || mem_reg_inst === "h76d9500b".U || wb_reg_inst === "h76d9500b".U || id_inst(0) === "h00979073".U || ex_reg_inst === "h00979073".U || mem_reg_inst === "h00979073".U || wb_reg_inst === "h00979073".U || id_inst(0) === "h0005560b".U || ex_reg_inst === "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U || id_inst(0) === "h000ed68b".U || ex_reg_inst === "h000ed68b".U || mem_reg_inst === "h000ed68b".U || wb_reg_inst === "h000ed68b".U || id_inst(0) === "h02d9570b".U || ex_reg_inst === "h02d9570b".U || mem_reg_inst === "h02d9570b".U || wb_reg_inst === "h02d9570b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h76d9500b".U || ex_reg_inst === "h76d9500b".U || mem_reg_inst === "h76d9500b".U || wb_reg_inst === "h76d9500b".U || id_inst(0) === "h0005c58b".U || ex_reg_inst === "h0005c58b".U || mem_reg_inst === "h0005c58b".U || wb_reg_inst === "h0005c58b".U || id_inst(0) === "h00879073".U || ex_reg_inst === "h00879073".U || mem_reg_inst === "h00879073".U || wb_reg_inst === "h00879073".U || id_inst(0) === "h00979073".U || ex_reg_inst === "h00979073".U || mem_reg_inst === "h00979073".U || wb_reg_inst === "h00979073".U || id_inst(0) === "h0005c58b".U || ex_reg_inst === "h0005c58b".U || mem_reg_inst === "h0005c58b".U || wb_reg_inst === "h0005c58b".U || id_inst(0) === "h0006d70b".U || ex_reg_inst === "h0006d70b".U || mem_reg_inst === "h0006d70b".U || wb_reg_inst === "h0006d70b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h7406d00b".U || ex_reg_inst === "h7406d00b".U || mem_reg_inst === "h7406d00b".U || wb_reg_inst === "h7406d00b".U || id_inst(0) === "h0007d60b".U || ex_reg_inst === "h0007d60b".U || mem_reg_inst === "h0007d60b".U || wb_reg_inst === "h0007d60b".U || id_inst(0) === "h000ed70b".U || ex_reg_inst === "h000ed70b".U || mem_reg_inst === "h000ed70b".U || wb_reg_inst === "h000ed70b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h740ed00b".U || ex_reg_inst === "h740ed00b".U || mem_reg_inst === "h740ed00b".U || wb_reg_inst === "h740ed00b".U ){
 */
/*      when(id_inst(0) === "h0007d20b".U || ex_reg_inst ===  "h0007d20b".U || mem_reg_inst === "h0007d20b".U || wb_reg_inst === "h0007d20b".U ||id_inst(0) === "h00971073".U || ex_reg_inst ===  "h00971073".U || mem_reg_inst === "h00971073".U || wb_reg_inst === "h00971073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2403500b".U || ex_reg_inst === "h2403500b".U || mem_reg_inst === "h2403500b".U || wb_reg_inst === "h2403500b".U ||id_inst(0) === "h00991073".U || ex_reg_inst ===  "h00991073".U || mem_reg_inst === "h00991073".U || wb_reg_inst === "h00991073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U || id_inst(0) === "h240e500b".U || ex_reg_inst ===  "h240e500b".U || mem_reg_inst === "h240e500b".U || wb_reg_inst === "h240e500b".U || id_inst(0) === "h240b500b".U || ex_reg_inst ===  "h240b500b".U || mem_reg_inst === "h240b500b".U || wb_reg_inst === "h240b500b".U){

  printf("[checkcachecounterfuncs]^^^^^^^^^^^ctrl_stalld %b  id_exhaz %b  id_memhaz %b id_wbhaz %b id_sboard %b  \n", ctrl_stalld, id_ex_hazard, id_mem_hazard, id_wb_hazard, id_sboard_hazard)
  
     printf("[checkcachecounterfuncs]^^^^^^^^^^^mem_reg_valid %b && mem_reg_flush_pipe %b ==> %b @ex_pc_valid %b (%b %b %b) [ex_inst %x => mem_inst %x]@ mem_pc_valid %b (%b %b %b) [mem_inst %x => wb_inst %x]@\n", mem_reg_valid, mem_reg_flush_pipe, mem_reg_valid && mem_reg_flush_pipe, ex_pc_valid, ex_reg_valid, ex_reg_replay, ex_reg_xcpt_interrupt, ex_reg_inst, mem_reg_inst, mem_pc_valid, mem_reg_valid, mem_reg_replay, mem_reg_xcpt_interrupt, mem_reg_inst, wb_reg_inst)
  
     printf("[checkcachecounterfuncs]^^^^^^^^^^^exregflush %b [%b %b] memregflushpip %b ctrl_killd %b ctrl_killx %b ctrl_killm %b exregecptinterrupt %b [%b %b %b] memregxcptinterruprt %b [%b %b]\n", ex_reg_flush_pipe,id_ctrl.fence_i, id_csr_flush, mem_reg_flush_pipe, ctrl_killd, ctrl_killx, ctrl_killm, ex_reg_xcpt_interrupt, !take_pc, ibuf.io.inst(0).valid, csr.io.interrupt,mem_reg_xcpt_interrupt, !take_pc_mem_wb, ex_reg_xcpt_interrupt)
  
     printf("[checkcachecounter] stall_vec_exe %b = Mux(number_of_elements > UInt(8) %b, id_ctrl.vec %b && !vector_unit.io.req.ready %b || vector_unit.io.req.valid %b ==>result %b, ex_ctrl.vec %b && !vector_unit.io.req.ready %b \n", stall_vec_exe, number_of_elements > UInt(8), id_ctrl.vec, !vector_unit.io.req.ready, vector_unit.io.req.valid, (id_ctrl.vec && !vector_unit.io.req.ready ) || vector_unit.io.req.valid ,ex_ctrl.vec, !vector_unit.io.req.ready)
  
}*/
//printf("[checkcachecounter]^^^^^^^^^^^ctrl_stalld %b  id_exhaz %b  id_memhaz %b id_wbhaz %b id_sboard %b  \n", ctrl_stalld, id_ex_hazard, id_mem_hazard, id_wb_hazard, id_sboard_hazard)

  //  when (id_inst(0) === "h02bb560b".U || id_inst(0)  === "h00985073".U ||  id_inst(0)   === "h000c558b".U || id_inst(0)   === "h02ba570b".U || id_inst(0)   === "h02b9d78b".U || id_inst(0)   === "h80f75257".U || id_inst(0) === "h26bb500b".U ||  ex_reg_inst  === "h00985073".U ||  ex_reg_inst  === "h000c558b".U || ex_reg_inst  === "h02ba570b".U || ex_reg_inst  === "h02b9d78b".U || ex_reg_inst === "h02bb560b".U || ex_reg_inst  === "h80f75257".U || ex_reg_inst === "h26bb500b".U || mem_reg_inst === "h00985073".U ||  mem_reg_inst === "h000c558b".U || mem_reg_inst === "h02ba570b".U || mem_reg_inst === "h02bb560b".U ||  mem_reg_inst === "h02b9d78b".U || mem_reg_inst === "h80f75257".U || mem_reg_inst === "h26bb500b".U ||  wb_reg_inst  === "h00985073".U ||  wb_reg_inst  === "h000c558b".U || wb_reg_inst  === "h02ba570b".U  || wb_reg_inst  === "h02b9d78b".U || wb_reg_inst  === "h80f75257".U || wb_reg_inst === "h26bb500b".U || wb_reg_inst === "h02bb560b".U ){
//when (id_inst(0) === "h00879073".U || id_inst(0) === "h00979073".U  || id_inst(0) === "h009f1073".U  || id_inst(0) === "h00941073".U  || id_inst(0) === "h0005c58b".U  || id_inst(0) === "h0007d60b".U  || id_inst(0) === "h0006d70b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h7406d00b".U  || id_inst(0) === "h0007d60b".U  || id_inst(0) === "h0003d70b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h7403d00b".U  || id_inst(0) === "h0007560b".U  || id_inst(0) === "h0006d68b".U  || id_inst(0) === "h02d9570b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h76d9500b".U  || id_inst(0) === "h0005560b".U  || id_inst(0) === "h000ed68b".U  || id_inst(0) === "h02d9570b".U  || id_inst(0) === "h72c5976b".U  || id_inst(0) === "h76d9500b".U  || ex_reg_inst === "h00879073".U   || ex_reg_inst === "h00979073".U   || ex_reg_inst === "h009f1073".U   || ex_reg_inst === "h00941073".U   || ex_reg_inst === "h0005c58b".U   || ex_reg_inst === "h0007d60b".U   || ex_reg_inst === "h0006d70b".U   || ex_reg_inst === "h72c5976b".U   || ex_reg_inst === "h7406d00b".U   || ex_reg_inst === "h0007d60b".U   || ex_reg_inst === "h0003d70b".U  || ex_reg_inst === "h72c5976b".U  || ex_reg_inst === "h7403d00b".U  || ex_reg_inst === "h0007560b".U  || ex_reg_inst === "h0006d68b".U  || ex_reg_inst === "h02d9570b".U  || ex_reg_inst === "h72c5976b".U  || ex_reg_inst === "h76d9500b".U  || ex_reg_inst === "h0005560b".U  || ex_reg_inst === "h000ed68b".U  || ex_reg_inst === "h02d9570b".U  || ex_reg_inst === "h72c5976b".U  || ex_reg_inst === "h76d9500b".U || mem_reg_inst === "h00879073".U  || mem_reg_inst === "h00979073".U  || mem_reg_inst === "h009f1073".U  || mem_reg_inst === "h00941073".U  || mem_reg_inst === "h0005c58b".U  || mem_reg_inst === "h0007d60b".U  || mem_reg_inst === "h0006d70b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h7406d00b".U  || mem_reg_inst === "h0007d60b".U  || mem_reg_inst === "h0003d70b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h7403d00b".U  || mem_reg_inst === "h0007560b".U  || mem_reg_inst === "h0006d68b".U  || mem_reg_inst === "h02d9570b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h76d9500b".U  || mem_reg_inst === "h0005560b".U  || mem_reg_inst === "h000ed68b".U  || mem_reg_inst === "h02d9570b".U  || mem_reg_inst === "h72c5976b".U  || mem_reg_inst === "h76d9500b".U || wb_reg_inst === "h00879073".U  || wb_reg_inst === "h00979073".U  || wb_reg_inst === "h009f1073".U  || wb_reg_inst === "h00941073".U  || wb_reg_inst === "h0005c58b".U  || wb_reg_inst === "h0007d60b".U  || wb_reg_inst === "h0006d70b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h7406d00b".U  || wb_reg_inst === "h0007d60b".U  || wb_reg_inst === "h0003d70b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h7403d00b".U  || wb_reg_inst === "h0007560b".U  || wb_reg_inst === "h0006d68b".U  || wb_reg_inst === "h02d9570b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h76d9500b".U  || wb_reg_inst === "h0005560b".U  || wb_reg_inst === "h000ed68b".U  || wb_reg_inst === "h02d9570b".U  || wb_reg_inst === "h72c5976b".U  || wb_reg_inst === "h76d9500b".U){
// when (id_inst(0) === "h00881073".U || id_inst(0) === "h00929073".U || id_inst(0) === "h00999073".U || id_inst(0) === "h0007c58b".U || id_inst(0) === "h0008560b".U || id_inst(0) === "h22c5926b".U || id_inst(0) === "h240e500b".U || id_inst(0) === "h2407d00b".U || id_inst(0) === "h0040425b".U || ex_reg_inst === "h00881073".U || ex_reg_inst === "h00929073".U || ex_reg_inst === "h00999073".U || ex_reg_inst  === "h0007c58b".U || ex_reg_inst === "h0008560b".U || ex_reg_inst === "h22c5926b".U || ex_reg_inst === "h240e500b".U || ex_reg_inst === "h2407d00b".U || ex_reg_inst === "h0040425b".U || mem_reg_inst === "h00881073".U || mem_reg_inst === "h00929073".U || mem_reg_inst === "h00999073".U || mem_reg_inst === "h0007c58b".U || mem_reg_inst === "h0008560b".U || mem_reg_inst === "h22c5926b".U || mem_reg_inst === "h240e500b".U || mem_reg_inst === "h2407d00b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h00881073".U || wb_reg_inst === "h00929073".U || wb_reg_inst === "h00999073".U || wb_reg_inst === "h0007c58b".U || wb_reg_inst === "h0008560b".U || wb_reg_inst === "h22c5926b".U || wb_reg_inst === "h240e500b".U ||  wb_reg_inst  === "h2407d00b".U || wb_reg_inst === "h0040425b".U){
// when(id_inst(0) === "h0005c58b".U || ex_reg_inst === "h0005c58b".U || mem_reg_inst === "h0005c58b".U || wb_reg_inst === "h0005c58b".U){
//when(id_inst(0) === "h0007c58b".U || ex_reg_inst === "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U || id_inst(0) === "h0008560b".U || ex_reg_inst === "h0008560b".U || mem_reg_inst === "h0008560b".U || wb_reg_inst === "h0008560b".U || id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U || id_inst(0) === "h0007c58b".U || ex_reg_inst === "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U || ibuf.io.pc === "h6b6da".U ){
// when(id_inst(0) === "h0005c58b".U || ex_reg_inst ===  "h0005c58b".U || mem_reg_inst === "h0005c58b".U || wb_reg_inst === "h0005c58b".U || id_inst(0) === "h00959073".U || ex_reg_inst === "h00959073".U || mem_reg_inst === "h00959073".U || wb_reg_inst === "h00959073".U || id_inst(0) === "h000c558b".U || ex_reg_inst === "h000c558b".U || mem_reg_inst === "h000c558b".U || wb_reg_inst === "h000c558b".U || id_inst(0) === "h0009c70b".U || ex_reg_inst === "h0009c70b".U || mem_reg_inst === "h0009c70b".U || wb_reg_inst === "h0009c70b".U || id_inst(0) === "h0009c78b".U || ex_reg_inst === "h0009c78b".U || mem_reg_inst === "h0009c78b".U || wb_reg_inst === "h0009c78b".U || id_inst(0) === "h72f7126b".U || ex_reg_inst === "h72f7126b".U || mem_reg_inst === "h72f7126b".U || wb_reg_inst === "h72f7126b".U || id_inst(0) === "h26bb500b".U || ex_reg_inst === "h26bb500b".U || mem_reg_inst === "h26bb500b".U || wb_reg_inst === "h26bb500b".U || id_inst(0) === "h0009c70b".U || ex_reg_inst === "h0009c70b".U || mem_reg_inst === "h0009c70b".U || wb_reg_inst === "h0009c70b".U || id_inst(0) === "h0009c78b".U || ex_reg_inst === "h0009c78b".U || mem_reg_inst === "h0009c78b".U || wb_reg_inst === "h0009c78b".U || id_inst(0) === "h80f75257".U || ex_reg_inst === "h80f75257".U || mem_reg_inst === "h80f75257".U || wb_reg_inst === "h80f75257".U || id_inst(0) === "h240bd00b".U || ex_reg_inst === "h240bd00b".U || mem_reg_inst === "h240bd00b".U || wb_reg_inst === "h240bd00b".U ){
/*   when(id_inst(0) === "h00879073".U || ex_reg_inst ===  "h00879073".U || mem_reg_inst === "h00879073".U || wb_reg_inst === "h00879073".U || id_inst(0) === "h000fc58b".U || ex_reg_inst === "h000fc58b".U || mem_reg_inst === "h000fc58b".U || wb_reg_inst === "h000fc58b".U || id_inst(0) === "h0007560b".U || ex_reg_inst === "h0007560b".U || mem_reg_inst === "h0007560b".U || wb_reg_inst === "h0007560b".U || id_inst(0) === "h0006d68b".U || ex_reg_inst === "h0006d68b".U || mem_reg_inst === "h0006d68b".U || wb_reg_inst === "h0006d68b".U || id_inst(0) === "h02d9570b".U || ex_reg_inst === "h02d9570b".U || mem_reg_inst === "h02d9570b".U || wb_reg_inst === "h02d9570b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h76d9500b".U || ex_reg_inst === "h76d9500b".U || mem_reg_inst === "h76d9500b".U || wb_reg_inst === "h76d9500b".U || id_inst(0) === "h00979073".U || ex_reg_inst === "h00979073".U || mem_reg_inst === "h00979073".U || wb_reg_inst === "h00979073".U || id_inst(0) === "h0005560b".U || ex_reg_inst === "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U || id_inst(0) === "h000ed68b".U || ex_reg_inst === "h000ed68b".U || mem_reg_inst === "h000ed68b".U || wb_reg_inst === "h000ed68b".U || id_inst(0) === "h02d9570b".U || ex_reg_inst === "h02d9570b".U || mem_reg_inst === "h02d9570b".U || wb_reg_inst === "h02d9570b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h76d9500b".U || ex_reg_inst === "h76d9500b".U || mem_reg_inst === "h76d9500b".U || wb_reg_inst === "h76d9500b".U || id_inst(0) === "h0005c58b".U || ex_reg_inst === "h0005c58b".U || mem_reg_inst === "h0005c58b".U || wb_reg_inst === "h0005c58b".U || id_inst(0) === "h00879073".U || ex_reg_inst === "h00879073".U || mem_reg_inst === "h00879073".U || wb_reg_inst === "h00879073".U || id_inst(0) === "h00979073".U || ex_reg_inst === "h00979073".U || mem_reg_inst === "h00979073".U || wb_reg_inst === "h00979073".U || id_inst(0) === "h0005c58b".U || ex_reg_inst === "h0005c58b".U || mem_reg_inst === "h0005c58b".U || wb_reg_inst === "h0005c58b".U || id_inst(0) === "h0006d70b".U || ex_reg_inst === "h0006d70b".U || mem_reg_inst === "h0006d70b".U || wb_reg_inst === "h0006d70b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h7406d00b".U || ex_reg_inst === "h7406d00b".U || mem_reg_inst === "h7406d00b".U || wb_reg_inst === "h7406d00b".U || id_inst(0) === "h0007d60b".U || ex_reg_inst === "h0007d60b".U || mem_reg_inst === "h0007d60b".U || wb_reg_inst === "h0007d60b".U || id_inst(0) === "h000ed70b".U || ex_reg_inst === "h000ed70b".U || mem_reg_inst === "h000ed70b".U || wb_reg_inst === "h000ed70b".U || id_inst(0) === "h72c5976b".U || ex_reg_inst === "h72c5976b".U || mem_reg_inst === "h72c5976b".U || wb_reg_inst === "h72c5976b".U || id_inst(0) === "h740ed00b".U || ex_reg_inst === "h740ed00b".U || mem_reg_inst === "h740ed00b".U || wb_reg_inst === "h740ed00b".U ){
 */
      /*when(id_inst(0) === "h0007d20b".U || ex_reg_inst ===  "h0007d20b".U || mem_reg_inst === "h0007d20b".U || wb_reg_inst === "h0007d20b".U ||id_inst(0) === "h00971073".U || ex_reg_inst ===  "h00971073".U || mem_reg_inst === "h00971073".U || wb_reg_inst === "h00971073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2403500b".U || ex_reg_inst === "h2403500b".U || mem_reg_inst === "h2403500b".U || wb_reg_inst === "h2403500b".U ||id_inst(0) === "h00991073".U || ex_reg_inst ===  "h00991073".U || mem_reg_inst === "h00991073".U || wb_reg_inst === "h00991073".U ||id_inst(0) === "h0040425b".U || ex_reg_inst ===  "h0040425b".U || mem_reg_inst === "h0040425b".U || wb_reg_inst === "h0040425b".U ||id_inst(0) === "h0007c58b".U || ex_reg_inst ===  "h0007c58b".U || mem_reg_inst === "h0007c58b".U || wb_reg_inst === "h0007c58b".U ||id_inst(0) === "h0005560b".U || ex_reg_inst ===  "h0005560b".U || mem_reg_inst === "h0005560b".U || wb_reg_inst === "h0005560b".U ||id_inst(0) === "h22c5926b".U || ex_reg_inst === "h22c5926b".U || mem_reg_inst === "h22c5926b".U || wb_reg_inst === "h22c5926b".U ||id_inst(0) === "h2407d00b".U || ex_reg_inst === "h2407d00b".U || mem_reg_inst === "h2407d00b".U || wb_reg_inst === "h2407d00b".U || id_inst(0) === "h240e500b".U || ex_reg_inst ===  "h240e500b".U || mem_reg_inst === "h240e500b".U || wb_reg_inst === "h240e500b".U || id_inst(0) === "h240b500b".U || ex_reg_inst ===  "h240b500b".U || mem_reg_inst === "h240b500b".U || wb_reg_inst === "h240b500b".U){
 printf("[checkcachecounter]@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@[ex_inst %x] offset %x io.dmem.req.bits  element_numebr %x addr %x tag %x cmd %x typ %x vector_cacheaccesstype %x cnt_cache_vsd %x el# %x is_cache_access_vec %x \n", ex_reg_inst ,offset_vsd, io.dmem.req.bits.addr,io.dmem.req.bits.element_number, io.dmem.req.bits.tag, io.dmem.req.bits.cmd, io.dmem.req.bits.typ, io.dmem.req.bits.vector_cache_access_type, io.dmem.req.bits.cnt_cache_vsd, io.dmem.req.bits.element_number, io.dmem.req.bits.is_cache_access_vec)
       }*/
/*   printf("[checkcachecounter]@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@[ex_inst %x] offset %x io.dmem.req.bits  element_numebr %x addr %x tag %x cmd %x typ %x vector_cacheaccesstype %x cnt_cache_vsd %x el# %x is_cache_access_vec %x \n", ex_reg_inst ,offset_vsd, io.dmem.req.bits.addr,io.dmem.req.bits.element_number, io.dmem.req.bits.tag, io.dmem.req.bits.cmd, io.dmem.req.bits.typ, io.dmem.req.bits.vector_cache_access_type, io.dmem.req.bits.cnt_cache_vsd, io.dmem.req.bits.element_number, io.dmem.req.bits.is_cache_access_vec)
  printf("[checkcachecounterfuncs] ctrl_stalld %b [%b %b %b %b %b %b %b %b %b %b %b] vld[ctrl_stalled_cond %b locks %b] vsd[ctrl_stalled_cond %b locks %b] ex_ctrl.vec %b ex_ctrl.mem %b ex_reg_valid %b cnt_cache != 7 %b cnt_cache %d s1_data %x mem_reg_flush_pipe %b exceptions[%b %b %b %b %b %b]\n",ctrl_stalld,id_ex_hazard || id_mem_hazard || id_wb_hazard || id_sboard_hazard, csr.io.singleStep && (ex_reg_valid || mem_reg_valid || wb_reg_valid),id_ctrl.fp && id_stall_fpu,   id_ctrl.mem && dcache_blocked,id_ctrl.rocc && rocc_blocked, id_ctrl.div && (!(div.io.req.ready || (div.io.resp.valid && !wb_wxd)) || div.io.req.valid), stall_vec_exe, id_do_fence, csr.io.csr_stall, ctrl_stalld_cond_vld || locks_vld, ctrl_stalld_cond_vsd || locks_vsd, ctrl_stalld_cond_vld, locks_vld, ctrl_stalld_cond_vsd, locks_vsd, ex_ctrl.vec, ex_ctrl.mem,ex_reg_valid, cnt_cache != 7, cnt_cache, io.dmem.DcacheCpu_s1_data.data, mem_reg_flush_pipe, take_pc, id_xcpt, ex_xcpt, mem_ldst_xcpt, mem_xcpt, wb_xcpt)
 */    
    printf("[checkcachecounterfuncs]id_inst DASM(%x) [pc %x inst %x id_ctrl.legal %b valid %b] (pc %x inst %x scatter %b v %b vec %b mem %b)ex  DASM(%x) (pc %x inst %x scatter %b v %b vec %b mem %b)mem DASM(%x) (pc %x inst %x scatter %b v %b vec %b mem %b)wb DASM(%x) vwb_wen %b vrf_mem_value %x \n", id_inst(0), ibuf.io.pc,id_inst(0), id_ctrl.legal,ibuf.io.inst(0).valid, ex_reg_pc,ex_reg_inst, ex_ctrl.scatter_gather, ex_reg_valid, ex_ctrl.vec, ex_ctrl.mem,  ex_reg_inst,mem_reg_pc, mem_reg_inst, mem_ctrl.scatter_gather, mem_reg_valid, mem_ctrl.vec,mem_ctrl.mem, mem_reg_inst,wb_reg_pc, wb_reg_inst, wb_ctrl.scatter_gather, wb_reg_valid, wb_ctrl.vec, wb_ctrl.mem, wb_reg_inst,vwb_wen,vrf_mem_value)
  
 

 // printf("[checkcachecounter] mem_br_taken %b alu.io.cmp_out %b branch [%b %b %b %b  %x %x %b] take_pc[%b %b %b %b] fence %b replay [ex %b mem %b mem_reg_replay %b] xcpt [%b memint %b ex_int %b wb_exception %b wb_cause %x] ctrl_kill[%b %b %b %b] kill_v[%b %b] killm_common_vlsd_vssd [dcache_kill_mem %b|| take_pc_wb %b|| mem_reg_xcpt %b] dmem_replay_next %b dcache_blocked %b dmem_s1_kill %b io.dmem.resp.valid %b dmem_resp %x\n",mem_br_taken, alu.io.cmp_out, mem_direction_misprediction, mem_misprediction, mem_wrong_npc, mem_cfi_taken, mem_npc, ex_reg_pc, ex_pc_valid, take_pc, take_pc_mem, take_pc_wb, take_pc_wb, mem_reg_sfence, replay_ex, replay_mem, mem_reg_replay, mem_reg_xcpt, mem_reg_xcpt_interrupt, ex_reg_xcpt_interrupt,wb_xcpt, wb_cause, ctrl_killd, ctrl_killx, ctrl_killm, ctrl_stalld, ctrl_killd_vlsd_vssd, killm_common_vlsd_vssd, dcache_kill_mem, take_pc_wb, mem_reg_xcpt,io.dmem.replay_next, dcache_blocked, io.dmem.s1_kill, io.dmem.resp.valid, io.dmem.resp.bits.DcacheCpu_data(255,0))


// csr  printf("[checkcachecounter]TAG mem_reg_wdat %x wb_reg_wdata %x csr.io.tval %x  tval_valid %b encodeVirtualAddress(wb_reg_wdata, wb_reg_wdata %x) %x dmem.io.ready %x mem_npc %x [mem_ctrl.jalr %b || mem_reg_sfence %b , encodeVirtualAddress(mem_reg_wdata, mem_reg_wdata %x).asSInt %x, mem_br_target %x]  mem_wrong_npc %b [ ex_pc_vali %b mem_npc %x =/= ex_reg_pc %x, ibuf.io.inst(0).valid %b || ibuf.io.imem.valid %b, mem_npc %x =/= ibuf.io.pc %x]   mem_reg_wdata %x := alu.io.out %x  mem_br_taken %b := alu.io.cmp_out %b\n", mem_reg_wdata, wb_reg_wdata, csr.io.tval, tval_valid,wb_reg_wdata, encodeVirtualAddress(wb_reg_wdata, wb_reg_wdata), io.dmem.req.ready, mem_npc, mem_ctrl.jalr, mem_reg_sfence, mem_reg_wdata, encodeVirtualAddress(mem_reg_wdata, mem_reg_wdata).asSInt, mem_br_target, mem_wrong_npc, ex_pc_valid, mem_npc, ex_reg_pc, ibuf.io.inst(0).valid, ibuf.io.imem.valid, mem_npc, ibuf.io.pc,  mem_reg_wdata, alu.io.out, mem_br_taken, alu.io.cmp_out)

//  val mem_npc = (Mux(mem_ctrl.jalr || mem_reg_sfence, encodeVirtualAddress(mem_reg_wdata, mem_reg_wdata).asSInt, mem_br_target) & SInt(-2)).asUInt
  //val mem_wrong_npc =
    //Mux(ex_pc_valid, mem_npc =/= ex_reg_pc,
      //     Mux(ibuf.io.inst(0).valid || ibuf.io.imem.valid, mem_npc =/= ibuf.io.pc, Bool(true)))
  //csr printf("[checkcachecounterfuncs] ctrl_staldl %b [line0 %b line1 %b] take_pc %b [[%b pc %x] id_inst %x ==> DASM(%x)] [[%b pc %x] ex %x ==> DASM(%x)] [[ %b pc %x] mem %x ==> DASM(%x)] [[pc %x] wb %x ==> DASM(%x)] vwb_wen %b csr.io.pc %x\n",ctrl_stalld, ((ctrl_stalld_cond_vld || locks_vld) && !take_pc),  ((ctrl_stalld_cond_vsd || locks_vsd) && !take_pc), take_pc, !ctrl_killd || csr.io.interrupt || ibuf.io.inst(0).bits.replay, ibuf.io.pc, id_inst(0),id_inst(0), ex_pc_valid, ex_reg_pc, ex_reg_inst,ex_reg_inst, mem_pc_valid, mem_reg_pc, mem_reg_inst,mem_reg_inst, wb_reg_pc, wb_reg_inst,wb_reg_inst,vwb_wen, csr.io.pc /*,vrf_mem_value*/)

/* printf("[checkcachecounter]TAG wb_reg_xcpt %b > mem_xcpt %b [[mem_reg_xcpt_interrupt %b || mem_reg_xcpt %b]  [mem_reg_valid %b  && mem_npc_misaligned %b] [mem_reg_valid %b && mem_ldst_xcpt %b] ]  && !take_pc_wb %b \n", wb_reg_xcpt, mem_xcpt, mem_reg_xcpt_interrupt, mem_reg_xcpt, mem_reg_valid, mem_npc_misaligned, mem_reg_valid, mem_ldst_xcpt, !take_pc_wb)

  printf("[checkcachecounter]TAG mem_reg_xcpt %b = [!ctrl_killx  %b && ex_xcpt %b]   mem_reg_xcpt_interrupt %b := [!take_pc_mem_wb %b && ex_reg_xcpt_interrupt %b] \n", mem_reg_xcpt, !ctrl_killx, ex_xcpt, mem_reg_xcpt_interrupt, !take_pc_mem_wb, ex_reg_xcpt_interrupt)

  printf("[checkcachecounter]TAG ex_xcpt %b [ex_reg_xcpt_interrupt %b || ex_reg_xcpt %b [!ctrl_killd %b && id_xcpt %b]]  csr.io.retire %b := wb_valid %b  \n", ex_xcpt, ex_reg_xcpt_interrupt, ex_reg_xcpt, !ctrl_killd, id_xcpt, csr.io.retire , wb_valid)
  printf("[checkcachecounter]TAG csr.io.interrupt %b bpu.io.debug_if %b bpu.io.xcpt_if %b pf.inst %b  ae.inst %b pf.inst %b ae.inst %b id_illegal_insn %b \n", csr.io.interrupt, bpu.io.debug_if, bpu.io.xcpt_if, id_xcpt0.pf.inst, id_xcpt0.ae.inst, id_xcpt1.pf.inst, id_xcpt1.ae.inst, id_illegal_insn)
 excpetion problem s1_data*/

  if (enableCommitLog) {
    val t = csr.io.trace(0)
    val rd = wb_waddr
    val wfd = wb_ctrl.wfd
    val wxd = wb_ctrl.wxd
    val has_data = wb_wen && !wb_set_sboard

    when (t.valid && !t.exception) {
      when (wfd) {
        ///printf ("%d 0x%x (0x%x) f%d p%d 0xXXXXXXXXXXXXXXXX\n", t.priv, t.iaddr, t.insn, rd, rd+UInt(32))
      }
      .elsewhen (wxd && rd =/= UInt(0) && has_data) {
        ///printf ("%d 0x%x (0x%x) x%d 0x%x\n", t.priv, t.iaddr, t.insn, rd, rf_wdata)
      }
      .elsewhen (wxd && rd =/= UInt(0) && !has_data) {
        ///printf ("%d 0x%x (0x%x) x%d p%d 0xXXXXXXXXXXXXXXXX\n", t.priv, t.iaddr, t.insn, rd, rd)
      }
      .otherwise {
       /// printf ("%d 0x%x (0x%x)\n", t.priv, t.iaddr, t.insn)
      }
    }

    when (ll_wen && rf_waddr =/= UInt(0)) {
      ///printf ("x%d p%d 0x%x\n", rf_waddr, rf_waddr, rf_wdata)
    }
  }
  else {
   /* printf("C%d: %d [%d] pc=[%x] W[r%d=%x][%d] R[r%d=%x] R[r%d=%x] inst=[%x] DASM(%x)\n",
         io.hartid, csr.io.time(31,0), csr.io.trace(0).valid && !csr.io.trace(0).exception,
         csr.io.trace(0).iaddr(vaddrBitsExtended-1, 0),
         Mux(rf_wen && !(wb_set_sboard && wb_wen), rf_waddr, UInt(0)), rf_wdata, rf_wen,
         wb_reg_inst(19,15), Reg(next=Reg(next=ex_rs(0))),
         wb_reg_inst(24,20), Reg(next=Reg(next=ex_rs(1))),
         csr.io.trace(0).insn, csr.io.trace(0).insn)*/
  }

  PlusArg.timeout(
    name = "max_core_cycles",
    docstring = "Kill the emulation after INT rdtime cycles. Off if 0."
  )(csr.io.time)

  def checkExceptions(x: Seq[(Bool, UInt)]) =
    (x.map(_._1).reduce(_||_), PriorityMux(x))

  def coverExceptions(exceptionValid: Bool, cause: UInt, labelPrefix: String, coverCausesLabels: Seq[(Int, String)]): Unit = {
    for ((coverCause, label) <- coverCausesLabels) {
      cover(exceptionValid && (cause === UInt(coverCause)), s"${labelPrefix}_${label}")
    }
  }

  def checkHazards(targets: Seq[(Bool, UInt)], cond: UInt => Bool) =
    targets.map(h => h._1 && cond(h._2)).reduce(_||_)

  def encodeVirtualAddress(a0: UInt, ea: UInt) = if (vaddrBitsExtended == vaddrBits) ea else {
    // efficient means to compress 64-bit VA into vaddrBits+1 bits
    // (VA is bad if VA(vaddrBits) != VA(vaddrBits-1))
    val a = a0.asSInt >> vaddrBits
    val msb = Mux(a === 0.S || a === -1.S, ea(vaddrBits), !ea(vaddrBits-1))
    Cat(msb, ea(vaddrBits-1,0))
  }

  class Scoreboard(n: Int, zero: Boolean = false)
  {
    def set(en: Bool, addr: UInt): Unit = update(en, _next | mask(en, addr))
    def clear(en: Bool, addr: UInt): Unit = update(en, _next & ~mask(en, addr))
    def read(addr: UInt): Bool = r(addr)
    def readBypassed(addr: UInt): Bool = _next(addr)

    private val _r = Reg(init=Bits(0, n))
    private val r = if (zero) (_r >> 1 << 1) else _r
    private var _next = r
    private var ens = Bool(false)
    private def mask(en: Bool, addr: UInt) = Mux(en, UInt(1) << addr, UInt(0))
    private def update(en: Bool, update: UInt) = {
      _next = update
      ens = ens || en
      when (ens) { _r := _next }
    }
  }
}

class RegFile(n: Int, w: Int, zero: Boolean = false) {
  private val rf = Mem(n, UInt(width = w))
  private def access(addr: UInt) = rf(~addr(log2Up(n)-1,0))
  private val reads = ArrayBuffer[(UInt,UInt)]()
  private var canRead = true
  def read(addr: UInt) = {
    require(canRead)
    reads += addr -> Wire(UInt())
    reads.last._2 := Mux(Bool(zero) && addr === UInt(0), UInt(0), access(addr))
    reads.last._2
  }
  def write(addr: UInt, data: UInt) = {
    canRead = false
    when (addr =/= UInt(0)) {
      access(addr) := data
      for ((raddr, rdata) <- reads)
        when (addr === raddr) { rdata := data }
    }
  }
}

object ImmGen {
  def apply(sel: UInt, inst: UInt) = {
    val sign = Mux(sel === IMM_Z, SInt(0), inst(31).asSInt)
    val b30_20 = Mux(sel === IMM_U, inst(30,20).asSInt, sign)
    val b19_12 = Mux(sel =/= IMM_U && sel =/= IMM_UJ, sign, inst(19,12).asSInt)
    val b11 = Mux(sel === IMM_U || sel === IMM_Z, SInt(0),
              Mux(sel === IMM_UJ, inst(20).asSInt,
              Mux(sel === IMM_SB, inst(7).asSInt, sign)))
    val b10_5 = Mux(sel === IMM_U || sel === IMM_Z, Bits(0), inst(30,25))
    val b4_1 = Mux(sel === IMM_U, Bits(0),
               Mux(sel === IMM_S || sel === IMM_SB, inst(11,8),
               Mux(sel === IMM_Z, inst(19,16), inst(24,21))))
    val b0 = Mux(sel === IMM_S, inst(7),
             Mux(sel === IMM_I, inst(20),
             Mux(sel === IMM_Z, inst(15), Bits(0))))

    Cat(sign, b30_20, b19_12, b11, b10_5, b4_1, b0).asSInt
  }
}
