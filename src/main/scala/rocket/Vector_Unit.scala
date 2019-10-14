// See LICENSE.Berkeley for license details.
// See LICENSE.SiFive for license details.

package freechips.rocketchip.rocket

import Chisel._
import Chisel.ImplicitConversions._
import freechips.rocketchip.util._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.tile.CoreModule
import ALU._

class Vector_UnitReq extends Bundle {
  val fn  = Bits(width = SZ_ALU_VFN.getWidth) // Later on change to SZ_ALU_VFN
  val in1 = Bits(width = 256.W)
  val in2 = Bits(width = 256.W)
  val in3 = Bits(width = 256.W)
  val tag = UInt(width = 5.W)
  val number_of_elements = UInt(width = 10.W)
  val vector = Bool()
  override def cloneType = new Vector_UnitReq().asInstanceOf[this.type]
}

class Vector_UnitResp extends Bundle {
  val data = Bits(width = 256.W)
  val tag = UInt(width = 5.W)
  override def cloneType = new Vector_UnitResp().asInstanceOf[this.type]
}

class Vector_UnitIO extends Bundle {
  val req = Decoupled(new Vector_UnitReq).flip
  val kill = Bool(INPUT)
  val resp = Decoupled(new Vector_UnitResp)
}
 

class Vector_Unit(implicit p: Parameters) extends CoreModule()(p){

   val io = new Vector_UnitIO
   val n: Int = 4
//  val number_of_elements = UInt(8)//UInt(16) //io.req.bits.number_of_elements//UInt(8)
   //val required_cycles: Int = number_of_elements / n
  // val s_ready :: s_neg_inputs :: s_mul :: s_div :: s_dummy :: s_neg_output :: s_done_mul :: s_done_div :: Nil = Enum(UInt(), 8)
  //val state = Reg(init=s_ready)

//  printf("[checkcachecounter]VVVVVVVVEEEEEEEEECCCCCCCCCCTTTTTTTTOOOOOOOORRRRRRRR io.cpu.req.bits.number_of_elements %x \n", io.req.bits.number_of_elements)
  val reduction_sum = Bool(false)
  val counter_num = Reg(init = UInt(0,1))
  val req = Reg(io.req.bits)
  val start_count = Wire(Bool())
  val req_ready = Reg(init = Bool(true))
  io.req.ready := req_ready

  when (/*io.req.bits.*/ io.req.bits.number_of_elements > n){

    when (io.req.fire() && counter_num =/= UInt(1)) {
      req := io.req.bits
      start_count := Bool(true)
      io.resp.valid := Bool(false)
      req_ready := Bool(false)
//      printf("[checkcachecounter]@@@VPU in1 \n")
    } .otherwise {
      start_count := Bool(false)
      io.resp.valid := Bool(true)
      req_ready := Bool(true)
  //    printf("[checkcachecounter]@@@VPU in2 \n")
    }

  }.otherwise{

    start_count := Bool(false)
    io.resp.valid := Bool(true)
    req_ready := Bool(true)
//    printf("[checkcachecounter]@@@VPU in3 \n")
  }

  when (start_count){
    counter_num := counter_num + UInt(1)
  } .otherwise {
    counter_num := UInt(0)
  }

  //////////////////////////////////////////////////////////////////////Adder////////////////////////////////////////////////////////////////////////////////////
  val Adders = Vec(Seq.fill(4){Module( new ValExec_MulAddRecF32_add()).io })//fill(8)
  val adder_out = Wire(Vec(4, UInt(32.W)))
  val adder_out_cat = Wire(UInt(128.W))

  // wire up the ports of the adders
 when (!reduction_sum){
  for(i <- 0 until n) {
     Adders(i).a := (((io.req.bits.in1 >> counter_num * UInt(128))(127,0)) >> 32*i)(31,0)
     Adders(i).b := (((io.req.bits.in2 >> counter_num * UInt(128))(127,0)) >> 32*i)(31,0) 
     adder_out(i) := Adders(i).result
  }
} /*.otherwise{

   for(i <- 0 until 4) {
     Adders(i).a := (((io.req.bits.in1 >> counter_num * UInt(128))(127,0)) >> 16*(2*i))(15,0)
     Adders(i).b := (((io.req.bits.in2 >> counter_num * UInt(128))(127,0)) >> 16*(2*i+1))(15,0) 
     adder_out(i) := Adders(i).result
   }
 
     Adders(4).a := adder_out(0)
     Adders(4).b := adder_out(1)
     adder_out(4) := Adders(4).result

     Adders(5).a := adder_out(2)
     Adders(5).b := adder_out(3)
     adder_out(5) := Adders(5).result

     Adders(6).a := adder_out(4)
     Adders(6).b := adder_out(5)
     adder_out(6) := Adders(6).result

 }
 */
   adder_out_cat := Cat(adder_out(3),adder_out(2),adder_out(1),adder_out(0))
   val Reg_adder_out = Reg(UInt(256.W))
  Reg_adder_out := Mux(counter_num === UInt(0), adder_out_cat, Cat(adder_out_cat, Reg_adder_out(127,0)))


/*  val reg_sixth = Reg(UInt(32.W))
  reg_sixth := adder_out(6)
  val redsum_out_cat = Cat(adder_out(6),adder_out(6),adder_out(6),adder_out(6),adder_out(6),adder_out(6),adder_out(6),adder_out(6),adder_out(6),adder_out(6),adder_out(6),adder_out(6),adder_out(6),adder_out(6),adder_out(6),adder_out(6)) //or reg_sixth
  val redsum_out_cat_sec = Cat(adder_out(6) + reg_sixth,adder_out(6) + reg_sixth,adder_out(6) + reg_sixth,adder_out(6) + reg_sixth,adder_out(6) + reg_sixth,adder_out(6) + reg_sixth,adder_out(6) + reg_sixth,adder_out(6) + reg_sixth,adder_out(6) + reg_sixth,adder_out(6) + reg_sixth,adder_out(6) + reg_sixth,adder_out(6) + reg_sixth,adder_out(6) + reg_sixth,adder_out(6) + reg_sixth,adder_out(6) + reg_sixth,adder_out(6) + reg_sixth)
  val Reg_redsum_out = Reg(UInt(256.W))
   Reg_redsum_out := Mux(counter_num === UInt(0), redsum_out_cat, redsum_out_cat_sec)
 */
    printf("[checkcachecounter]**********add a [ %x %x %x %x] b [ %x %x %x %x] out [ %x %x %x %x] \n ",  Adders(3).a, Adders(2).a, Adders(1).a, Adders(0).a,  Adders(3).b, Adders(2).b, Adders(1).b, Adders(0).b,adder_out(3),adder_out(2),adder_out(1),adder_out(0))



  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  val vbcastx_out_cat = Wire(UInt(128.W))
  vbcastx_out_cat := UInt(0)
  //Cat(io.req.bits.in1(15,0),io.req.bits.in1(15,0),io.req.bits.in1(15,0),io.req.bits.in1(15,0),io.req.bits.in1(15,0),io.req.bits.in1(15,0),io.req.bits.in1(15,0),io.req.bits.in1(15,0))
  val Reg_vbcastx_out = Reg(UInt(256.W))
  Reg_vbcastx_out := UInt(0)
  //Mux(counter_num === UInt(0), vbcastx_out_cat, Cat(vbcastx_out_cat, Reg_vbcastx_out(127,0)))

   when (io.req.bits.fn === ALU.FN_VBCASTX){
     printf("[checkcachecounter]invpu1_FN_VBCASTX a %x b %x out %x out %x\n ",io.req.bits.in1, io.req.bits.in2, Reg_vbcastx_out, vbcastx_out_cat)
  }
  //////////////////////////////////////////////////////////////////////multipler////////////////////////////////////////////////////////////////////////////////////
   val Multipliers = Vec(Seq.fill(4){Module( new ValExec_MulAddRecF32_mul()).io })
   val multiplier_out = Wire(Vec(4, UInt(32.W)))
   val multiplier_out_cat = Wire(UInt(128.W))

   // wire up the ports of the adders
   for(i <- 0 until n) {
     Multipliers(i).a := (((io.req.bits.in1 >> counter_num * UInt(128))(127,0)) >> 32*i)(31,0)
     Multipliers(i).b := (((io.req.bits.in2 >> counter_num * UInt(128))(127,0)) >> 32*i)(31,0) 
     multiplier_out(i) := Multipliers(i).result
   }

   multiplier_out_cat := Cat(multiplier_out(3),multiplier_out(2),multiplier_out(1),multiplier_out(0))
   val Reg_multiplier_out = Reg(UInt(256.W))
  Reg_multiplier_out := Mux(counter_num === UInt(0), multiplier_out_cat, Cat(multiplier_out_cat, Reg_multiplier_out(127,0)))

   //when (io.req.bits.fn === ALU.FN_VMUL){
//     printf("[checkcachecounter]invpu1_multiply a [%x %x %x %x %x %x %x %x] b [%x %x %x %x %x %x %x %x] out [%x %x %x %x %x %x %x %x] \n ", Multipliers(7).a, Multipliers(6).a, Multipliers(5).a, Multipliers(4).a, Multipliers(3).a, Multipliers(2).a, Multipliers(1).a, Multipliers(0).a, Multipliers(7).b, Multipliers(6).b, Multipliers(5).b, Multipliers(4).b, Multipliers(3).b, Multipliers(2).b, Multipliers(1).b, Multipliers(0).b,multiplier_out(7),multiplier_out(6),multiplier_out(5),multiplier_out(4),multiplier_out(3),multiplier_out(2),multiplier_out(1),multiplier_out(0))
  //}

  ////////////////////////////////////////////////VFMADD//////////////////////////////////////////////////////////////////////////////////////////////////////

   val MulAdds = Vec(Seq.fill(4){Module( new ValExec_MulAddRecF32()).io })
   val MulAdd_out = Wire(Vec(4, UInt(32.W)))
   val MulAdd_out_cat = Wire(UInt(128.W))

   // wire up the ports of the adders
   for(i <- 0 until n) {
     MulAdds(i).a  := (((io.req.bits.in1 >> counter_num * UInt(128))(127,0)) >> 32*i)(31,0)
     MulAdds(i).b  := (((io.req.bits.in2 >> counter_num * UInt(128))(127,0)) >> 32*i)(31,0)
     MulAdds(i).c  := (((io.req.bits.in3 >> counter_num * UInt(128))(127,0)) >> 32*i)(31,0)
     MulAdds(i).roundingMode := UInt(0)
     MulAdds(i).detectTininess := UInt(1)  
     MulAdd_out(i) := MulAdds(i).result
    }

   MulAdd_out_cat := Cat(MulAdd_out(3),MulAdd_out(2),MulAdd_out(1),MulAdd_out(0))
   val Reg_MulAdd_out = Reg(UInt(256.W))
   Reg_MulAdd_out := Mux(counter_num === UInt(0), MulAdd_out_cat, Cat(MulAdd_out_cat, Reg_MulAdd_out(127,0)))

  when (io.req.bits.fn === ALU.FN_VFMADD){
     printf("[checkcachecounter]********************invpu1 mult_add a [%x %x %x %x] b [%x %x %x %x]  c [%x %x %x %x] out [%x %x %x %x] \n ", MulAdds(3).a, MulAdds(2).a, MulAdds(1).a, MulAdds(0).a,MulAdds(3).b, MulAdds(2).b, MulAdds(1).b, MulAdds(0).b,MulAdds(3).c, MulAdds(2).c, MulAdds(1).c, MulAdds(0).c,MulAdd_out(3),MulAdd_out(2),MulAdd_out(1),MulAdd_out(0))
  }

  //////////////////////////////////////////////////////////////////////comparator////////////////////////////////////////////////////////////////////////////////////
   val comparators_lt = Vec(Seq.fill(4){Module( new ValExec_CompareRecF32_lt()).io })
   val comparator_lt_out = Wire(Vec(4, UInt(32.W)))
   val comparator_lt_out_cat = Wire(UInt(128.W))

   // wire up the ports of the adders
   for(i <- 0 until n) {
     comparators_lt(i).a := (((io.req.bits.in1 >> counter_num * UInt(128))(127,0)) >> 32*i)(31,0)
     comparators_lt(i).b := (((io.req.bits.in2 >> counter_num * UInt(128))(127,0)) >> 32*i)(31,0) 
     comparator_lt_out(i) := comparators_lt(i).actual.out
    }

    comparator_lt_out_cat := Cat(comparator_lt_out(3),comparator_lt_out(2),comparator_lt_out(1),comparator_lt_out(0))
   val Reg_comparator_lt_out = Reg(UInt(256.W))
   Reg_comparator_lt_out := Mux(counter_num === UInt(0), comparator_lt_out_cat, Cat(comparator_lt_out_cat, Reg_comparator_lt_out(127,0)))


   val comparators_le = Vec(Seq.fill(4){Module( new ValExec_CompareRecF32_le()).io })
   val comparator_le_out = Wire(Vec(4, UInt(32.W)))
   val comparator_le_out_cat = Wire(UInt(128.W))

   // wire up the ports of the adders
   for(i <- 0 until n) {
     comparators_le(i).a := (((io.req.bits.in1 >> counter_num * UInt(128))(127,0)) >> 32*i)(31,0)
     comparators_le(i).b := (((io.req.bits.in2 >> counter_num * UInt(128))(127,0)) >> 32*i)(31,0) 
     comparator_le_out(i) := comparators_le(i).actual.out
    }

   comparator_le_out_cat := Cat(comparator_le_out(3),comparator_le_out(2),comparator_le_out(1),comparator_le_out(0))
   val Reg_comparator_le_out = Reg(UInt(256.W))
   Reg_comparator_le_out := Mux(counter_num === UInt(0), comparator_le_out_cat, Cat(comparator_le_out_cat, Reg_comparator_le_out(127,0)))


   val comparators_eq = Vec(Seq.fill(4){Module( new ValExec_CompareRecF32_eq()).io })
   val comparator_eq_out = Wire(Vec(4, UInt(32.W)))
   val comparator_eq_out_cat = Wire(UInt(128.W))

   // wire up the ports of the adders
   for(i <- 0 until n) {
     comparators_eq(i).a := (((io.req.bits.in1 >> counter_num * UInt(128))(127,0)) >> 32*i)(31,0)
     comparators_eq(i).b := (((io.req.bits.in2 >> counter_num * UInt(128))(127,0)) >> 32*i)(31,0) 
     comparator_eq_out(i) := comparators_eq(i).actual.out
    }

   comparator_eq_out_cat := Cat(comparator_eq_out(3),comparator_eq_out(2),comparator_eq_out(1),comparator_eq_out(0))
   val Reg_comparator_eq_out = Reg(UInt(256.W))
   Reg_comparator_eq_out := Mux(counter_num === UInt(0), comparator_eq_out_cat, Cat(comparator_eq_out_cat, Reg_comparator_eq_out(127,0)))

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  val min_out = Wire(Vec(4, UInt(32.W)))
  val min_out_cat = Wire(UInt(128.W))

  val max_out = Wire(Vec(4, UInt(32.W)))
  val max_out_cat = Wire(UInt(128.W))

  val lt_bool_val = Wire(Vec(4, Bool()))
  val eq_bool_val = Wire(Vec(4, Bool()))
  val le_bool_val = Wire(Vec(4, Bool()))
  for(i <- 0 until n) {
   lt_bool_val(i) := comparator_lt_out(i) === UInt(1)
   eq_bool_val(i) := comparator_eq_out(i) === UInt(1)
   min_out(i) := Mux(lt_bool_val(i), (((io.req.bits.in1 >> counter_num * UInt(128))(127,0)) >> 32*i)(31,0), Mux(eq_bool_val(i), (((io.req.bits.in1 >> counter_num * UInt(128))(127,0)) >> 32*i)(31,0), (((io.req.bits.in2 >> counter_num * UInt(128))(127,0)) >> 32*i)(31,0)))
  }

  for(i <- 0 until n) {
   le_bool_val(i) := comparator_le_out(i) === UInt(1)
   max_out(i) := Mux(le_bool_val(i), (((io.req.bits.in2 >> counter_num * UInt(128))(127,0)) >> 32*i)(31,0), (((io.req.bits.in1 >> counter_num * UInt(128))(127,0)) >> 32*i)(31,0))
 }

  min_out_cat := Cat(min_out(3), min_out(2), min_out(1), min_out(0))

  val Reg_min_out = Reg(UInt(256.W))
  Reg_min_out := Mux(counter_num === UInt(0), min_out_cat, Cat(min_out_cat, Reg_min_out(127,0)))


  max_out_cat := Cat(max_out(3), max_out(2), max_out(1), max_out(0))

  val Reg_max_out = Reg(UInt(256.W))
  Reg_max_out := Mux(counter_num === UInt(0), max_out_cat, Cat(max_out_cat, Reg_min_out(127,0)))

  /////////////////////////////////////////////////////////////////////
  val prev_alu_func = Reg(UInt(width = SZ_ALU_VFN.getWidth))
  prev_alu_func := io.req.bits.fn
  val v_alu_out = Mux(/*io.req.bits.*/ io.req.bits.number_of_elements <= n, Mux(io.req.bits.fn === ALU.FN_VADD, adder_out_cat , Mux(io.req.bits.fn ===  ALU.FN_VMUL, multiplier_out_cat, Mux(io.req.bits.fn ===  ALU.FN_VFMADD, MulAdd_out_cat, Mux(io.req.bits.fn ===  ALU.FN_VFMIN, min_out_cat, Mux(io.req.bits.fn === ALU.FN_VBCASTX, vbcastx_out_cat, max_out_cat))))), Mux(prev_alu_func ===  ALU.FN_VADD, Reg_adder_out, Mux(prev_alu_func ===  ALU.FN_VMUL, Reg_multiplier_out, Mux(prev_alu_func ===  ALU.FN_VFMADD, Reg_MulAdd_out, Mux(prev_alu_func ===  ALU.FN_VFMIN, Reg_min_out,Mux(prev_alu_func === ALU.FN_VBCASTX, Reg_vbcastx_out, Reg_max_out))))))

  io.resp.bits <> req
  io.resp.bits.data := v_alu_out

//  printf("[checkcachecounter] AAAAAAAAAAAAAAAAAAAAA inside vpu number_of_elements %x n %x io.req.bits.fn %x === add %b mul %b fmadd %b fmin %b vbcast %b  prev_alu_func %x === add %b mul %b fmadd %b fmin %b vbcast %b  v_alu_out %x\n", io.req.bits.number_of_elements, n, io.req.bits.fn, io.req.bits.fn === ALU.FN_VADD, io.req.bits.fn ===  ALU.FN_VMUL, io.req.bits.fn ===  ALU.FN_VFMADD, io.req.bits.fn ===  ALU.FN_VFMIN, io.req.bits.fn === ALU.FN_VBCASTX, prev_alu_func, prev_alu_func ===  ALU.FN_VADD, prev_alu_func ===  ALU.FN_VMUL, prev_alu_func ===  ALU.FN_VFMADD, prev_alu_func ===  ALU.FN_VFMIN, prev_alu_func === ALU.FN_VBCASTX, v_alu_out)
//   printf("[checkcachecounter] AAAAAAAAAAAAAAAAAAAAA inside vpu adder_out_cat %x Reg_adder_out %x multiplier_out_cat %x Reg_multiplier_out %x MulAdd_out_cat %x Reg_MulAdd_out %x min_out_cat %x Reg_min_out %x vbcastx_out_cat %x Reg_vbcastx_out %x max_out_cat %x Reg_max_out %x\n", adder_out_cat, Reg_adder_out, multiplier_out_cat, Reg_multiplier_out, MulAdd_out_cat, Reg_MulAdd_out, min_out_cat, Reg_min_out, vbcastx_out_cat, Reg_vbcastx_out, max_out_cat, Reg_max_out)
 //printf("[checkcachecounter]invpu2 SZ_ALU_FN.getWidth %d  el#<=8  %b  in1 %x in2 %x in3 %x [v_alu_out %x muladd %x add %x] [req.fn %x fn_vadd %x fn_vmul %x fn_vfmadd %x] \n", SZ_ALU_FN.getWidth, number_of_elements <= n, io.req.bits.in1, io.req.bits.in2, io.req.bits.in3, v_alu_out,Reg_MulAdd_out, Reg_adder_out,io.req.bits.fn, ALU.FN_VADD, ALU.FN_VMUL, ALU.FN_VFMADD)

  //////////////////////////////////////////////////////debug statements////////////////////////////////////////////////////////////////////////////////////////////////////
/*when (io.req.bits.vector){
  printf("[checkcachecounter]%b@@@VPU in1 %x in2 %x \n", number_of_elements <= n, io.req.bits.in1, io.req.bits.in2)
  printf("[checkcachecounter]@@@VPU io.a %x %x %x %x %x io.b %x %x %x %x %x  \n", (io.req.bits.in1 >> (counter_num*UInt(128)))(63,48),(io.req.bits.in1 >> (counter_num*UInt(128)))(79,64), (io.req.bits.in1 >> (counter_num*UInt(128)))(95,80), (io.req.bits.in1 >> (counter_num*UInt(128)))(111,96), (io.req.bits.in1 >> (counter_num*UInt(128)))(127,112), (io.req.bits.in2 >> (counter_num*UInt(128)))(63,48), (io.req.bits.in2 >> (counter_num*UInt(128)))(79,64), (io.req.bits.in2 >> (counter_num*UInt(128)))(95,80), (io.req.bits.in2 >> (counter_num*UInt(128)))(111,96), (io.req.bits.in2 >> (counter_num*UInt(128)))(127,112))
  printf("[checkcachecounter]@@@VPU adder counter_num %d  alu_out(cat) %x [%x %x %x %x %x %x %x %x]\n", counter_num, adder_out_cat , adder_out(7), adder_out(6), adder_out(5), adder_out(4), adder_out(3), adder_out(2), adder_out(1), adder_out(0))
  printf("[checkcachecounter]@@@VPU multiplier counter_num %d  alu_out(cat) %x [%x %x %x %x %x %x %x %x]\n", counter_num, multiplier_out_cat , multiplier_out(7), multiplier_out(6), multiplier_out(5), multiplier_out(4), multiplier_out(3), multiplier_out(2), multiplier_out(1), multiplier_out(0))
  printf("[checkcachecounter]@@@VPU adder counter_num %d Reg_adder_out %x \n", counter_num, Reg_adder_out)  
   printf("[checkcachecounter]@@@VPU start_coutn %b req_fire %b respvalid %b req_ready %b io.req.valid %b io.req.ready %b counter %d adder-reg [%x] adder_out_cat %x mult [%x] alu_out %x \n", start_count,io.req.fire(), io.resp.valid, req_ready, io.req.valid, io.req.ready, counter_num,Reg_adder_out,adder_out_cat, multiplier_out_cat,v_alu_out)
 }
 */ 
}

