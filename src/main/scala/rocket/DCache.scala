// See LICENSE.SiFive for license details.

package freechips.rocketchip.rocket

import Chisel._
import Chisel.ImplicitConversions._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.subsystem.{RocketTilesKey}
import freechips.rocketchip.diplomacy.{AddressSet, RegionType}
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import freechips.rocketchip.util.property._
import chisel3.internal.sourceinfo.SourceInfo
import chisel3.experimental.dontTouch
import TLMessages._

class DCacheErrors(implicit p: Parameters) extends L1HellaCacheBundle()(p)
  with CanHaveErrors {
  val correctable = (cacheParams.tagCode.canCorrect || cacheParams.dataCode.canCorrect).option(Valid(UInt(width = paddrBits)))
  val uncorrectable = (cacheParams.tagCode.canDetect || cacheParams.dataCode.canDetect).option(Valid(UInt(width = paddrBits)))
  val bus = Valid(UInt(width = paddrBits))
}

class DCacheDataReq(implicit p: Parameters) extends L1HellaCacheBundle()(p) {
  val addr = Bits(width = untagBits)
  val write = Bool()
  val isvec = Bool()
  val wdata = UInt(width = encBits * rowBytes / eccBytes)
  val vwdata= UInt(256.W)
  val wordMask = UInt(width = rowBytes / wordBytes)
  val eccMask = UInt(width = wordBytes / eccBytes)
  val way_en = Bits(width = nWays)
  val vector_cache_access_type = Bool()
  val cnt_cache_vsd = Bits (width = 5)
  val element_number = UInt(width = 5)
  val is_cache_access_vec = Bool()
}

class DCacheDataArray_modified2(implicit p: Parameters) extends L1HellaCacheModule()(p) {
  val io = new Bundle {
    val req = Valid(new DCacheDataReq).flip
    val resp = Vec(nWays, UInt(width = req.bits.wdata.getWidth )).asOutput
    val resp_vector = Vec(nWays, UInt(width = 256 )).asOutput
  }

  require(rowBytes % wordBytes == 0)

  val Rtagt = Cat (UInt(0), UInt(0), UInt(0), io.req.bits.addr)
  val Rtag = Rtagt (5, 0)
  val Rbyteoff = Rtag >> 3
  val word = Rbyteoff
  val wordinHL =Rbyteoff(1, 0)
  val temp_input_mask = io.req.bits.eccMask
  val sel0 = (wordinHL === UInt(0x0))
  val sel1 = (wordinHL === UInt(0x1))
  val sel2 = (wordinHL === UInt(0x2))
  val sel3 = (wordinHL === UInt(0x3))

  val shiftedmask = Mux(sel3, temp_input_mask << 24, Mux(sel2, Cat (UInt(1<<8)(7,0), temp_input_mask << 16), Mux(sel1, Cat (UInt(1<<8)(7,0),UInt(1<<8)(7,0),  temp_input_mask <<  8) , Mux(sel0, Cat (UInt(1<<8)(7,0),UInt(1<<8)(7,0),UInt(1<<8)(7,0), temp_input_mask), UInt(0)))))
  val eccMask = if (eccBits == wordBits) Seq(true.B) else shiftedmask.toBools
  val wMask = if (nWays == 1) eccMask else (0 until nWays).flatMap(i => eccMask.map(_ && io.req.bits.way_en(i)))

  //zazad begins
  //generate mask based on the element position in the cache line
  //2 bytes masks for 16-bit width data elements
  val mask_vector_offset = Rtagt (4,0)
  //val s1_vmaskval = UInt("hffffffff")
  val s1_vmaskval = Cat(UInt(1) << (UInt(32)-(io.req.bits.element_number*UInt(2))), ((UInt(1) << (io.req.bits.element_number*UInt(2))) - UInt(1)))(31,0)
  val s1_vmaskval_shift_offset = (s1_vmaskval << io.req.bits.addr(4,0))(31,0) 
  val element_size_in_byte = UInt(2)
  val scatter_gather_vmaskval = Wire(UInt(32.W))
  val scatter_gather_vwdata   = Wire(UInt(256.W))
  val extracted_bits = Wire(UInt(16.W))
  extracted_bits := ((io.req.bits.vwdata) >> io.req.bits.cnt_cache_vsd * UInt(16))(15,0)
  scatter_gather_vmaskval := ((UInt(1) << element_size_in_byte) - UInt(1)) << mask_vector_offset
  scatter_gather_vwdata   := extracted_bits << mask_vector_offset * UInt(8)
  val vwdataval= Mux(io.req.bits.vector_cache_access_type, (io.req.bits.vwdata << (io.req.bits.addr(4,0) * UInt(8)))(255,0), scatter_gather_vwdata)
  val vmaskval = Mux(io.req.bits.vector_cache_access_type, s1_vmaskval_shift_offset, scatter_gather_vmaskval)
 printf("[checkcachecounter]###insidecache cnt_cache_vsd %d  mask_vector_offset %d scatter_gather_mask %x maskval %x cacheaccesstype %b  data %x extracted_data %x scatter %x final %x\n",io.req.bits.cnt_cache_vsd, mask_vector_offset, scatter_gather_vmaskval, vmaskval, io.req.bits.vector_cache_access_type, io.req.bits.vwdata,extracted_bits, scatter_gather_vwdata, vwdataval)
  printf("[checkcachecounter] +++++++++++insidedcache #element %x  s1_vmaskval %x scatter_gather_vmaskval %x  vector_cache_access_type %x final vmaskval %x \n ", io.req.bits.element_number, s1_vmaskval, scatter_gather_vmaskval, io.req.bits.vector_cache_access_type, vmaskval)
  val veccMask = vmaskval.toBools
  val vwMask = if (nWays == 1) veccMask else (0 until nWays).flatMap(i => veccMask.map(_ && io.req.bits.way_en(i)))
  val vwWords = vwdataval.grouped(256)

  val testmul = io.req.bits.element_number*UInt(2);

  printf("[checkcachecounter]@@@@@@@@@@@@@@@@@incache  el# %x test_mul %x s1_vmaskval %x s1_vmaskval_shift_offset %x [first_half %x second_half %x fixed %x [el*2 %x 1<<el*2 %x]] access_type %b scatter_gather_mask %x vmaskval %x\n",io.req.bits.element_number,testmul,s1_vmaskval, s1_vmaskval_shift_offset, UInt(1) << (UInt(32)-(io.req.bits.element_number*UInt(2))) ,UInt(1) << (io.req.bits.element_number*UInt(2)) - UInt(1), ((UInt(1) << (io.req.bits.element_number*UInt(2))) - UInt(1)), io.req.bits.element_number*UInt(2), UInt(1) << (io.req.bits.element_number*UInt(2)) ,io.req.bits.vector_cache_access_type,  scatter_gather_vmaskval, vmaskval)

  //zazad ends
  val vectordatatemp = Mux(sel3,io.req.bits.wdata << 192 , Mux(sel2, Cat (UInt(1<<64)(63,1), UInt (0), io.req.bits.wdata <<  128), Mux(sel1,Cat (UInt(1<<64)(63,1), UInt (0),UInt(1<<64)(63,1),UInt (0), io.req.bits.wdata <<  64) , Mux(sel0,Cat (UInt(1<<64)(63,1),UInt (0),UInt(1<<64)(63,1),UInt (0),UInt(1<<64)(63,1),UInt (0), io.req.bits.wdata), UInt(0)))))
  val wWords = vectordatatemp.grouped(256)
  val addr = io.req.bits.addr >> 5
  val data_arrays = Seq.fill(rowBytes / wordBytes) { SeqMem(nSets * 2/*HL*/, Vec(nWays * (32), UInt(width = encBits))) }
  val rdata = for ((array, i) <- data_arrays zipWithIndex) yield {

    val valid = io.req.valid && (Bool(data_arrays.size == 1) || io.req.bits.wordMask(i))
    when (valid && io.req.bits.write){
      ///printf ("[CacheChecksmodified]%x w ============================================================================================================= \n",io.req.bits.addr)
      ///printf ("[CacheChecksmodified]%x w io.addr %x  io.eccmask [%x] io.wayen[%b][%b][%b][%b] io.wdata[%x] wordinhl[%d] sel[%b][%b][%b][%b] addr %x  shiftedmask[%x] wwrods[%x]\n", io.req.bits.addr, io.req.bits.addr,io.req.bits.eccMask,io.req.bits.way_en(3),io.req.bits.way_en(2),io.req.bits.way_en(1),io.req.bits.way_en(0),io.req.bits.wdata, wordinHL, sel3, sel2, sel1, sel0,addr,shiftedmask, wWords(0))
  
       /// printf ("[CacheChecksmodified]%x w wMask %b  %b %b %b %b %b %b %b %b %b %b %b %b %b %b %b %b %b %b %b %b %b %b \n",io.req.bits.addr, wMask(0), wMask(8), wMask(16), wMask(24), wMask(32), wMask(40),wMask(41),wMask(42),wMask(43),wMask(44),wMask(45),wMask(46),wMask(47),wMask(48), wMask(56), wMask(64), wMask(72), wMask(80), wMask(88), wMask(96), wMask(104), wMask(112),wMask(120))

      ///printf("[CacheChecksmodified]%x w \n\n", io.req.bits.addr)
    }

    when (valid && !io.req.bits.write){
      ///printf ("[CacheChecksmodified]%x r ============================================================================================================= \n", io.req.bits.addr)
      ///printf ("[CacheChecksmodified]%x r io.addr %x io.wayen[%b][%b][%b][%b] addr %x\n", io.req.bits.addr,io.req.bits.addr,io.req.bits.way_en(3),io.req.bits.way_en(2),io.req.bits.way_en(1),io.req.bits.way_en(0),addr)
    }

    when (valid && io.req.bits.write && !io.req.bits.isvec) {
      val wData = wWords(i).grouped(encBits)
      array.write(addr, Vec((0 until nWays).flatMap(i => wData)), wMask)//32 byte is written into the cache for four ways simultaneously
      printf ("[checkcachecounter]@@@@@@@@@@@@@@@@@@@@incache scalar write %x io.req.bits.addr %x io.req.bits.wdata %x io.req.bits.vwadat %x io.req.bits.eccmask %x eccmask %b%b%b%b%b%b%b%b %b%b%b%b%b%b%b%b %b%b%b%b%b%b%b%b %b%b%b%b%b%b%b%b\n", io.req.bits.write, io.req.bits.addr,io.req.bits.wdata,io.req.bits.vwdata, io.req.bits.eccMask, wMask(31), wMask(30), wMask(29), wMask(28), wMask(27), wMask(26), wMask(25), wMask(24), wMask(23), wMask(22), wMask(21), wMask(20), wMask(19), wMask(18), wMask(17), wMask(16), wMask(15), wMask(14), wMask(13), wMask(12), wMask(11), wMask(10), wMask(9), wMask(8), wMask(7), wMask(6), wMask(5), wMask(4), wMask(3), wMask(2), wMask(1), wMask(0))
     printf ("[trackloadsnew] scalar sel %b %b %b %b \n", sel3, sel2, sel1, sel0)
    }.elsewhen (valid && io.req.bits.write && io.req.bits.isvec){
      val vwData = vwWords(i).grouped(encBits)
      array.write(addr, Vec((0 until nWays).flatMap(i => vwData)), vwMask)//32 byte is written into the cache for four ways simultaneously
      printf ("[checkcachecounter]@@@@@@@@@@@@@@@incache vector el# %x write %xx  nways %x vwwords %x  vector addr %x shifteddata %x wdata %x wdata %x veccmask %b%b%b%b%b%b%b%b %b%b%b%b%b%b%b%b %b%b%b%b%b%b%b%b %b%b%b%b%b%b%b%b vwMask %b%b%b%b%b%b%b%b %b%b%b%b%b%b%b%b %b%b%b%b%b%b%b%b %b%b%b%b%b%b%b%b\n",io.req.bits.element_number,io.req.bits.write, nWays,vwWords(0),io.req.bits.addr, (io.req.bits.vwdata << (io.req.bits.addr(4,0) * UInt(8)))(255,0), io.req.bits.vwdata,io.req.bits.wdata,veccMask(31),veccMask(30),veccMask(29),veccMask(28),veccMask(27),veccMask(26),veccMask(25),veccMask(24),veccMask(23),veccMask(22),veccMask(21),veccMask(20),veccMask(19),veccMask(18),veccMask(17),veccMask(16),veccMask(15),veccMask(14),veccMask(13),veccMask(12),veccMask(11),veccMask(10),veccMask(9),veccMask(8),veccMask(7),veccMask(6),veccMask(5),veccMask(4),veccMask(3),veccMask(2),veccMask(1),veccMask(0), vwMask(127), vwMask(126),vwMask(125), vwMask(124), vwMask(123), vwMask(122),vwMask(121),vwMask(120), vwMask(119), vwMask(118), vwMask(117), vwMask(116), vwMask(115), vwMask(114), vwMask(113), vwMask(112), vwMask(111), vwMask(110), vwMask(109), vwMask(108), vwMask(107), vwMask(106), vwMask(105), vwMask(104), vwMask(103), vwMask(102), vwMask(101), vwMask(100), vwMask(99), vwMask(98), vwMask(97), vwMask(96))
      printf ("[checkcachecounter] vector sel %b %b %b %b \n", sel3, sel2, sel1, sel0)
    }
    val data = array.read(addr, valid && !io.req.bits.write)//4*32B is read
    data.grouped(32).map(_.asUInt).toSeq //grouped into 4 32B
  }

  val temprdata0= rdata(0).asUInt
  //val temprdata1= rdata(1).asUInt
  //val temprdata2= rdata(2).asUInt
  //val temprdata3= rdata(3).asUInt
 /// printf ("[CacheChecksmodified]%x rdata(0) %x %x \n\n",io.req.bits.addr,temprdata0(255,0), temprdata0)
  //printf ("[CacheChecksmodified]%x rdata(1) %x %x \n\n",io.req.bits.addr,temprdata1(255,0), temprdata1)
  //printf ("[CacheChecksmodified]%x rdata(2) %x %x \n\n",io.req.bits.addr,temprdata2(255,0), temprdata2)
  //printf ("[CacheChecksmodified]%x rdata(3) %x %x \n\n",io.req.bits.addr,temprdata3(255,0), temprdata3)

  (io.resp_vector zip rdata.transpose).foreach { case (resp_vector, data) => resp_vector := data.asUInt }

  ////printf ("[CacheChecksmodified]%x io.resp.vector %x %x %x %x   \n\n",io.req.bits.addr, io.resp_vector(0), io.resp_vector(1), io.resp_vector(2), io.resp_vector(3))

  when (io.req.valid && !io.req.bits.write){
     val extractedword_0 = Mux(sel3,io.resp_vector(0) (255,192), Mux(sel2, io.resp_vector(0) (191,128) , Mux(sel1,  io.resp_vector(0) (127,64) ,io.resp_vector(0) (63,0))))
     ////printf ("[checkdata]%x  %x  0********  %x %x %x %x  sels [%b %b %b %b] \n\n",io.req.bits.addr,extractedword_0, io.resp_vector(0) (255,192), io.resp_vector(0) (191,128) , io.resp_vector(0) (127,64) ,io.resp_vector(0) (63,0), sel3, sel2, sel1,sel0)
     val extractedword_1 = Mux(sel3,io.resp_vector(1) (255,192), Mux(sel2, io.resp_vector(1) (191,128) , Mux(sel1,  io.resp_vector(1) (127,64) ,io.resp_vector(1) (63,0))))
     ///printf ("[checkdata]%x  %x  1********  %x %x %x %x  sels [%b %b %b %b] \n\n",io.req.bits.addr,extractedword_1, io.resp_vector(1) (255,192), io.resp_vector(1) (191,128) , io.resp_vector(1) (127,64) ,io.resp_vector(1) (63,0), sel3, sel2, sel1,sel0)
     val extractedword_2 = Mux(sel3,io.resp_vector(2) (255,192), Mux(sel2, io.resp_vector(2) (191,128) , Mux(sel1,  io.resp_vector(2) (127,64) ,io.resp_vector(2) (63,0))))
     ///printf ("[checkdata]%x  %x  2********  %x %x %x %x  sels [%b %b %b %b] \n\n",io.req.bits.addr,extractedword_2, io.resp_vector(2) (255,192), io.resp_vector(2) (191,128) , io.resp_vector(2) (127,64) ,io.resp_vector(2) (63,0), sel3, sel2, sel1,sel0)
     val extractedword_3 = Mux(sel3,io.resp_vector(3) (255,192), Mux(sel2, io.resp_vector(3) (191,128) , Mux(sel1,  io.resp_vector(3) (127,64) ,io.resp_vector(3) (63,0))))
     ///printf ("[checkdata]%x %x  3********  %x %x %x %x  sels [%b %b %b %b] \n\n",io.req.bits.addr,extractedword_3, io.resp_vector(3) (255,192), io.resp_vector(3) (191,128) , io.resp_vector(3) (127,64) ,io.resp_vector(3) (63,0), sel3, sel2, sel1,sel0)

  io.resp(0) := extractedword_0 
  io.resp(1) := extractedword_1
  io.resp(2) := extractedword_2
  io.resp(3) := extractedword_3
   }

  when (io.req.valid && !io.req.bits.write){
    ///printf ("\n[checkdata] addr %x io.vectorresp %x %x %x %x resp %x %x %x %x  \n\n", io.req.bits.addr, io.resp_vector(3), io.resp_vector(2), io.resp_vector(1), io.resp_vector(0), io.resp(3), io.resp(2), io.resp(1), io.resp(0) )

  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


class DCacheDataArray(implicit p: Parameters) extends L1HellaCacheModule()(p) {
  val io = new Bundle {
    val req = Valid(new DCacheDataReq).flip
    val resp = Vec(nWays, UInt(width = req.bits.wdata.getWidth )).asOutput
  }

  require(rowBytes % wordBytes == 0)
  val eccMask = if (eccBits == wordBits) Seq(true.B) else io.req.bits.eccMask.toBools
  val wMask = if (nWays == 1) eccMask else (0 until nWays).flatMap(i => eccMask.map(_ && io.req.bits.way_en(i)))
  val wWords = io.req.bits.wdata.grouped(encBits * (wordBits / eccBits))
  val addr = io.req.bits.addr >> rowOffBits
  val data_arrays = Seq.fill(rowBytes / wordBytes) { SeqMem(nSets * refillCycles, Vec(nWays * (wordBits / eccBits), UInt(width = encBits))) }
  val rdata = for ((array, i) <- data_arrays zipWithIndex) yield {
    val valid = io.req.valid && (Bool(data_arrays.size == 1) || io.req.bits.wordMask(i))
    when (valid && io.req.bits.write) {
      val wData = wWords(i).grouped(encBits)
      array.write(addr, Vec((0 until nWays).flatMap(i => wData)), wMask)//32 byte is written into the cache for four ways simultaneously
     /// printf ("[checkdatamain] addr %x  data %x eccmask %x\n", io.req.bits.addr, io.req.bits.wdata, io.req.bits.eccMask)

                              val temp_print_wData= Vec((0 until nWays).flatMap(i => wData)).grouped(32).map(_.asUInt).toSeq
        ///                      printf ("[CacheChecksmain]%x wmain ============================================================================================================= \n",io.req.bits.addr)
           ///                   printf ("[CacheChecksmain]%x wmain array.write.data %x \n",io.req.bits.addr, temp_print_wData(0))
              ///                printf("[CacheChecksmain]%x wmain \n\n",io.req.bits.addr)
                              val readfrommem = array.read(addr, Bool(true))
                              val readfrommem_grouped = readfrommem.grouped(8).map(_.asUInt).toSeq
                 ///             printf("[CacheChecksmain]%x wmain grouped_read_after_write %x %x %x %x \n",io.req.bits.addr, readfrommem_grouped(3),readfrommem_grouped(2),readfrommem_grouped(1),readfrommem_grouped(0))
                    ///          printf ("[CacheChecksmain]%x wmain io.addr %x  io.eccmask [%x] io.wayen[%b][%b][%b][%b] io.wdata[%x] addr %x wwrods[%x]\n", io.req.bits.addr,io.req.bits.addr,io.req.bits.eccMask,io.req.bits.way_en(3),io.req.bits.way_en(2),io.req.bits.way_en(1),io.req.bits.way_en(0),io.req.bits.wdata, addr, wWords(0))

///                              printf ("[CacheChecksmain]%x wmain wMask %b %b %b %b %b %b %b %b %b %b %b %b %b %b %b %b %b %b %b %b %b %b %b %b %b %b %b %b %b %b %b %b \n",io.req.bits.addr, wMask(0),wMask(1),wMask(2),wMask(3),wMask(4),wMask(5),wMask(6),wMask(7), wMask(8),wMask(9),wMask(10),wMask(11),wMask(12),wMask(13),wMask(14),wMask(15),wMask(16) ,wMask(17), wMask(18),wMask(19), wMask(20),wMask(21),wMask(22),wMask(23),wMask(24),wMask(25),wMask(26),wMask(27), wMask(28),wMask(29),wMask(30), wMask(31))
   ///                           printf ("[CacheChecksmain]%x wmain ============================================================================================================= \n",io.req.bits.addr)

    }
    val data = array.read(addr, valid && !io.req.bits.write)//32B is read

                            when (valid && !io.req.bits.write){
                              val print_data = data.grouped(32).map(_.asUInt).toSeq
      ///                        printf("[CacheChecksmain]%x r readdata %x  \n", io.req.bits.addr, print_data(0))
         ///                    printf ("[CacheChecksmain]%x rmain ============================================================================================================= \n",io.req.bits.addr)
            ///                 printf ("[CacheChecksmain]%x rmain io.addr %x io.wayen[%b][%b][%b][%b] addr %x\n",io.req.bits.addr, io.req.bits.addr,io.req.bits.way_en(3),io.req.bits.way_en(2),io.req.bits.way_en(1),io.req.bits.way_en(0),addr)
                             val temp_read_data = data.grouped(8).map(_.asUInt).toSeq
               ///              printf ("[CacheChecksmain]%x rmain readdata %x %x %x %x\n",io.req.bits.addr, temp_read_data(3),temp_read_data(2),temp_read_data(1),temp_read_data(0))
                  ///           printf ("[CacheChecksmain]%x rmain ============================================================================================================= \n",io.req.bits.addr)
                           }




    data.grouped(wordBits / eccBits).map(_.asUInt).toSeq //grouped into 4 words (8b bytes)
  }
    (io.resp zip rdata.transpose).foreach { case (resp, data) => resp := data.asUInt }

 //// printf ("[CacheChecksmain]%x io.resp  0******** %x \n", io.req.bits.addr, io.resp(0))
  ///printf ("[CacheChecksmain]%x io.resp  1******** %x \n", io.req.bits.addr, io.resp(1))
  ///printf ("[CacheChecksmain]%x io.resp  2******** %x \n", io.req.bits.addr, io.resp(2))
  ///printf ("[CacheChecksmain]%x io.resp  3******** %x \n", io.req.bits.addr, io.resp(3))

   when (io.req.valid && !io.req.bits.write){
    ///printf ("\n[checkdata] addr %x resp %x %x %x %x  \n\n", io.req.bits.addr, io.resp(3), io.resp(2), io.resp(1), io.resp(0) )

  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


class DCacheMetadataReq(implicit p: Parameters) extends L1HellaCacheBundle()(p) {
  val write = Bool()
  val addr = UInt(width = vaddrBitsExtended)
  val way_en = UInt(width = nWays)
  val data = new L1Metadata
}

class DCache(hartid: Int, val scratch: () => Option[AddressSet] = () => None, val bufferUncachedRequests: Option[Int] = None)(implicit p: Parameters) extends HellaCache(hartid)(p) {
  override lazy val module = new DCacheModule(this) 
}

class DCacheModule(outer: DCache) extends HellaCacheModule(outer) {
  val tECC = cacheParams.tagCode
  val dECC = cacheParams.dataCode
  require(isPow2(eccBytes) && eccBytes <= wordBytes)
  require(eccBytes == 1 || !dECC.isInstanceOf[IdentityCode])
  val usingRMW = eccBytes > 1 || usingAtomicsInCache

  // tags
  val replacer = cacheParams.replacement
  val metaArb = Module(new Arbiter(new DCacheMetadataReq, 8))
  val tag_array = SeqMem(nSets, Vec(nWays, UInt(width = tECC.width(metaArb.io.out.bits.data.getWidth))))

  // data
  //comment_final_dcache val data = Module(new DCacheDataArray)
  val dataArb = Module(new Arbiter(new DCacheDataReq, 4))
  
  //comment_final_dcache data.io.req <> dataArb.io.out
  //comment_final_dcache data.io.req.bits.wdata := encodeData(dataArb.io.out.bits.wdata(rowBits-1, 0))

  ///printf ("[checkdata]if encoding works wdata %x  data_Arbout %x \n", data.io.req.bits.wdata,dataArb.io.out.bits.wdata(rowBits-1, 0))

  //zazad begins
  val data_modified2 = Module(new DCacheDataArray_modified2)
  data_modified2.io.req <> dataArb.io.out
  data_modified2.io.req.bits.wdata := encodeData(dataArb.io.out.bits.wdata(rowBits-1, 0))
  printf("[checkcachecounter]@@@@@@@@@@@@@@@@@@@@@@@in cache  data_modified2.io.req element_numebr %x addr %x \n", data_modified2.io.req.bits.addr, data_modified2.io.req.bits.element_number)
  //zazad ends

  dataArb.io.out.ready := true
  metaArb.io.out.ready := true

  val tl_out_a = Wire(tl_out.a)
  tl_out.a <> outer.bufferUncachedRequests
                .map(_ min maxUncachedInFlight-1)
                .map(Queue(tl_out_a, _, flow = true))
                .getOrElse(tl_out_a)

  val (tl_out_c, release_queue_empty) =
    if (cacheParams.acquireBeforeRelease) {
      val q = Module(new Queue(tl_out.c.bits.cloneType, cacheDataBeats, flow = true))
      tl_out.c <> q.io.deq
      (q.io.enq, q.io.count === 0)
    } else {
      (tl_out.c, true.B)
    }

  val s1_valid = Reg(next=io.cpu.req.fire(), init=Bool(false))
  val s1_probe = Reg(next=tl_out.b.fire(), init=Bool(false))
  val probe_bits = RegEnable(tl_out.b.bits, tl_out.b.fire()) // TODO has data now :(
  val s1_nack = Wire(init=Bool(false))
  val s1_valid_masked = s1_valid && !io.cpu.s1_kill
  val s1_valid_not_nacked = s1_valid && !s1_nack
  val s1_req = Reg(io.cpu.req.bits)
  val s0_clk_en = metaArb.io.out.valid && !metaArb.io.out.bits.write
  when (s0_clk_en) {
    s1_req := io.cpu.req.bits
    s1_req.addr := Cat(metaArb.io.out.bits.addr >> blockOffBits, io.cpu.req.bits.addr(blockOffBits-1,0))
    when (!metaArb.io.in(7).ready) { s1_req.phys := true }
  }
  val s1_read = isRead(s1_req.cmd)
  val s1_write = isWrite(s1_req.cmd)
  val s1_vector = s1_req.isvector
  //select stride type
  val s1_vector_cache_access_type = s1_req.vector_cache_access_type
  val s1_cnt_cache_vsd = s1_req.cnt_cache_vsd
  val s1_readwrite = s1_read || s1_write
  val s1_sfence = s1_req.cmd === M_SFENCE
  val s1_flush_valid = Reg(Bool())
  val s1_waw_hazard = Wire(Bool())
  

  val s_ready :: s_voluntary_writeback :: s_probe_rep_dirty :: s_probe_rep_clean :: s_probe_retry :: s_probe_rep_miss :: s_voluntary_write_meta :: s_probe_write_meta :: Nil = Enum(UInt(), 8)
  val cached_grant_wait = Reg(init=Bool(false))
  val release_ack_wait = Reg(init=Bool(false))
  val can_acquire_before_release = !release_ack_wait && release_queue_empty
  val release_state = Reg(init=s_ready)
  val any_pstore_valid = Wire(Bool())
  val inWriteback = release_state.isOneOf(s_voluntary_writeback, s_probe_rep_dirty)
  val releaseWay = Wire(UInt())
  io.cpu.req.ready := (release_state === s_ready) && !cached_grant_wait && !s1_nack
   //printf("[justprintinsts] \n")

  // I/O MSHRs
  val mmioOffset = if (outer.scratch().isDefined) 0 else 1
  val uncachedInFlight = Seq.fill(maxUncachedInFlight) { RegInit(Bool(false)) }
  val uncachedReqs = Seq.fill(maxUncachedInFlight) { Reg(new HellaCacheReq) }

  // hit initiation path
  val s0_needsRead = needsRead(io.cpu.req.bits)
  val s0_read = isRead(io.cpu.req.bits.cmd)

  dataArb.io.in(3).valid := io.cpu.req.valid && s0_needsRead
  dataArb.io.in(3).bits.write := false
  dataArb.io.in(3).bits.addr := io.cpu.req.bits.addr
  dataArb.io.in(3).bits.wordMask := UIntToOH(io.cpu.req.bits.addr.extract(rowOffBits-1,offsetlsb))
  dataArb.io.in(3).bits.way_en := ~UInt(0, nWays)
  dataArb.io.in(3).bits.element_number := io.cpu.req.bits.element_number
  dataArb.io.in(3).bits.vector_cache_access_type := io.cpu.req.bits.vector_cache_access_type
  printf("[checkcachecounter]@@@@@@@@@@@@@@@@@@@@incache dataarb.io.in3  valid %x write %x entire addr %x element# %x access_type %x \n", dataArb.io.in(3).valid, dataArb.io.in(3).bits.write, dataArb.io.in(3).bits.addr,dataArb.io.in(3).bits.element_number, dataArb.io.in(3).bits.vector_cache_access_type)
  printf("[checkcachecounter]@@@@@@@@@@@@@@@@@@@@incache io.cpu.req valid %x entire addr %x el# %x access_type %x \n", io.cpu.req.valid, io.cpu.req.bits.addr, io.cpu.req.bits.element_number, io.cpu.req.bits.vector_cache_access_type)
  when (!dataArb.io.in(3).ready && s0_read) { io.cpu.req.ready := false }
  val s1_did_read = RegEnable(dataArb.io.in(3).fire(), s0_clk_en)
  metaArb.io.in(7).valid := io.cpu.req.valid
  metaArb.io.in(7).bits.write := false
  metaArb.io.in(7).bits.addr := io.cpu.req.bits.addr
  metaArb.io.in(7).bits.way_en := ~UInt(0, nWays)
  metaArb.io.in(7).bits.data := metaArb.io.in(4).bits.data
  when (!metaArb.io.in(7).ready) { io.cpu.req.ready := false }

  // address translation
  val tlb = Module(new TLB(false, log2Ceil(coreDataBytes), nTLBEntries))
  io.ptw <> tlb.io.ptw
  tlb.io.req.valid := s1_valid && !io.cpu.s1_kill && (s1_readwrite || s1_sfence)
  tlb.io.req.bits.sfence.valid := s1_sfence
  tlb.io.req.bits.sfence.bits.rs1 := s1_req.typ(0)
  tlb.io.req.bits.sfence.bits.rs2 := s1_req.typ(1)
  //zazad to be replaced
  tlb.io.req.bits.sfence.bits.asid := io.cpu.s1_data.data
  tlb.io.req.bits.sfence.bits.addr := s1_req.addr
  tlb.io.req.bits.passthrough := s1_req.phys
  tlb.io.req.bits.vaddr := s1_req.addr
  tlb.io.req.bits.size := s1_req.typ
  tlb.io.req.bits.cmd := s1_req.cmd
  when (!tlb.io.req.ready && !tlb.io.ptw.resp.valid && !io.cpu.req.bits.phys) { io.cpu.req.ready := false }
  when (s1_valid && s1_readwrite && tlb.io.resp.miss) { s1_nack := true
   // printf ("[justprintinsts] s1_nack s1_valid && s1_readwrite && tlb.io.resp.miss \n")
  }

  printf("[checkcachecounter]TLLLLLLLLllllb in cache vaddr %x passthrough %b tlb.io.req valid %b %b %x %x %x %x %b %x %x %x tlb.io.resp.miss %b tlb.io.ptw.resp.valid %b \n", s1_req.addr, s1_req.phys, tlb.io.req.valid, tlb.io.req.bits.sfence.valid,  tlb.io.req.bits.sfence.bits.rs1, tlb.io.req.bits.sfence.bits.rs2, tlb.io.req.bits.sfence.bits.asid,  tlb.io.req.bits.sfence.bits.addr, tlb.io.req.bits.passthrough, tlb.io.req.bits.vaddr, tlb.io.req.bits.size , tlb.io.req.bits.cmd, tlb.io.resp.miss, tlb.io.ptw.resp.valid )

  val s1_paddr = tlb.io.resp.paddr
  val s1_victim_way = Wire(init = replacer.way)
  val (s1_hit_way, s1_hit_state, s1_meta, s1_victim_meta) =
    if (usingDataScratchpad) {
      val baseAddr = GetPropertyByHartId(p(RocketTilesKey), _.dcache.flatMap(_.scratch.map(_.U)), io.hartid)
      val inScratchpad = s1_paddr >= baseAddr && s1_paddr < baseAddr + nSets * cacheBlockBytes
      val hitState = Mux(inScratchpad, ClientMetadata.maximum, ClientMetadata.onReset)
      val dummyMeta = L1Metadata(UInt(0), ClientMetadata.onReset)
      (inScratchpad, hitState, Seq(tECC.encode(dummyMeta.asUInt)), dummyMeta)
    } else {
      val metaReq = metaArb.io.out
      val metaIdx = metaReq.bits.addr(idxMSB, idxLSB)
      when (metaReq.valid && metaReq.bits.write) {
        val wdata = tECC.encode(metaReq.bits.data.asUInt)
        val wmask = if (nWays == 1) Seq(true.B) else metaReq.bits.way_en.toBools
        tag_array.write(metaIdx, Vec.fill(nWays)(wdata), wmask)
      }
      val s1_meta = tag_array.read(metaIdx, metaReq.valid && !metaReq.bits.write)
      val s1_meta_uncorrected = s1_meta.map(tECC.decode(_).uncorrected.asTypeOf(new L1Metadata))
      val s1_tag = s1_paddr >> untagBits
      val s1_meta_hit_way = s1_meta_uncorrected.map(r => r.coh.isValid() && r.tag === s1_tag).asUInt
      val s1_meta_hit_state = ClientMetadata.onReset.fromBits(
        s1_meta_uncorrected.map(r => Mux(r.tag === s1_tag && !s1_flush_valid, r.coh.asUInt, UInt(0)))
          .reduce (_|_))
      //printf("[checkcachecounter] metareqaddr %x metaIdx %x s1_meta [%x %x %x %x] s1_tag %x s1hitway %x coh %b %b %b %b tags %x %x %x %x \n", metaReq.bits.addr,  metaReq.bits.addr(idxMSB, idxLSB), s1_meta(0),s1_meta(1),s1_meta(2),s1_meta(3),s1_tag, s1_meta_hit_way,s1_meta_uncorrected(0).coh.isValid(),s1_meta_uncorrected(1).coh.isValid(),s1_meta_uncorrected(2).coh.isValid(),s1_meta_uncorrected(3).coh.isValid(), s1_meta_uncorrected(0).tag,s1_meta_uncorrected(1).tag,s1_meta_uncorrected(2).tag,s1_meta_uncorrected(3).tag)

      (s1_meta_hit_way, s1_meta_hit_state, s1_meta, s1_meta_uncorrected(s1_victim_way))
    }
  val s1_data_way = Wire(init = Mux(inWriteback, releaseWay, s1_hit_way))
  //comment_final_dcache val s1_all_data_ways = Vec(data.io.resp :+ dummyEncodeData(tl_out.d.bits.data))
  
  
  ///printf ("[checkdata] %x s1_data_way %x releaseway %x s1_hit_way %x inwriteback %b \n", s1_req.addr(11,0), s1_data_way, releaseWay, s1_hit_way, inWriteback)

  //val modified2_s1_all_data_ways = Vec(data_modified2.io.resp :+ dummyEncodeData(tl_out.d.bits.data))

 /* when (s1_read && s1_valid){
    printf("[CacheChecksmodified]%x s1_all_data_ways %x %x %x %x\n",io.cpu.req.bits.addr(11,0), data_modified2.io.resp(3),data_modified2.io.resp(2),data_modified2.io.resp(1),data_modified2.io.resp(0))
    printf("[CacheChecksmain]%x s1_all_data_ways  %x %x %x %x \n",io.cpu.req.bits.addr(11,0), data.io.resp(3),data.io.resp(2),data.io.resp(1),data.io.resp(0))
  }
  */

    val Rtagt = Cat (UInt(0), UInt(0), UInt(0), s1_req.addr(11,0))
    val Rtag = Rtagt (5, 0)
    val Rbyteoff = Rtag >> 3
    val word = Rbyteoff
    val wordinHL =Rbyteoff(1, 0)
    val sel0 = (wordinHL === UInt(0x0))
    val sel1 = (wordinHL === UInt(0x1))
    val sel2 = (wordinHL === UInt(0x2))
    val sel3 = (wordinHL === UInt(0x3))

     val extractedword_0 = Mux(sel3,data_modified2.io.resp_vector(0) (255,192), Mux(sel2, data_modified2.io.resp_vector(0) (191,128) , Mux(sel1, data_modified2.io.resp_vector(0) (127,64) ,data_modified2.io.resp_vector(0) (63,0))))
   //  printf ("[checkdataoutsidemod]%x  %x  0********  %x %x %x %x  sels [%b %b %b %b] \n\n",s1_req.addr(11,0),extractedword_0, data_modified2.io.resp_vector(0) (255,192), data_modified2.io.resp_vector(0) (191,128) , data_modified2.io.resp_vector(0) (127,64) ,data_modified2.io.resp_vector(0) (63,0), sel3, sel2, sel1,sel0)
     val extractedword_1 = Mux(sel3,data_modified2.io.resp_vector(1) (255,192), Mux(sel2, data_modified2.io.resp_vector(1) (191,128) , Mux(sel1,  data_modified2.io.resp_vector(1) (127,64) ,data_modified2.io.resp_vector(1) (63,0))))
     //printf ("[checkdataoutsidemod]%x  %x  1********  %x %x %x %x  sels [%b %b %b %b] \n\n",s1_req.addr(11,0),extractedword_1, data_modified2.io.resp_vector(1) (255,192), data_modified2.io.resp_vector(1) (191,128) , data_modified2.io.resp_vector(1) (127,64) ,data_modified2.io.resp_vector(1) (63,0), sel3, sel2, sel1,sel0)
     val extractedword_2 = Mux(sel3,data_modified2.io.resp_vector(2) (255,192), Mux(sel2, data_modified2.io.resp_vector(2) (191,128) , Mux(sel1,  data_modified2.io.resp_vector(2) (127,64) ,data_modified2.io.resp_vector(2) (63,0))))
     //printf ("[checkdataoutsidemod]%x  %x  2********  %x %x %x %x  sels [%b %b %b %b] \n\n",s1_req.addr(11,0),extractedword_2, data_modified2.io.resp_vector(2) (255,192), data_modified2.io.resp_vector(2) (191,128) , data_modified2.io.resp_vector(2) (127,64) ,data_modified2.io.resp_vector(2) (63,0), sel3, sel2, sel1,sel0)
     val extractedword_3 = Mux(sel3,data_modified2.io.resp_vector(3) (255,192), Mux(sel2, data_modified2.io.resp_vector(3) (191,128) , Mux(sel1,  data_modified2.io.resp_vector(3) (127,64) ,data_modified2.io.resp_vector(3) (63,0))))
     //printf ("[checkdataoutsidemod]%x %x  3********  %x %x %x %x  sels [%b %b %b %b] \n\n",s1_req.addr(11,0),extractedword_3, data_modified2.io.resp_vector(3) (255,192), data_modified2.io.resp_vector(3) (191,128) , data_modified2.io.resp_vector(3) (127,64) ,data_modified2.io.resp_vector(3) (63,0), sel3, sel2, sel1,sel0)

   val io_resp_0 = extractedword_0 
   val io_resp_1 = extractedword_1
   val io_resp_2 = extractedword_2
   val io_resp_3 = extractedword_3
   val modified2_s1_all_data_ways = Vec(io_resp_0, io_resp_1, io_resp_2, io_resp_3, dummyEncodeData(tl_out.d.bits.data))
   val modified2_s1_all_data_ways_vectors = Vec(data_modified2.io.resp_vector(0), data_modified2.io.resp_vector(1), data_modified2.io.resp_vector(2), data_modified2.io.resp_vector(3))
   //modified2_s1_all_data_ways(0) := io_resp_0
   //modified2_s1_all_data_ways(1) := io_resp_1
   //modified2_s1_all_data_ways(2) := io_resp_2
   //modified2_s1_all_data_ways(3) := io_resp_3
   //modified2_s1_all_data_ways(4) := dummyEncodeData(tl_out.d.bits.data)


  when (s1_valid){

    ///printf ("\n\n[checkdatamain] [tlout %x %x %x %x %x] <== [%x %x %x %x] %b %b \n\n",s1_all_data_ways(4),s1_all_data_ways(3), s1_all_data_ways(2), s1_all_data_ways(1), s1_all_data_ways(0),data.io.resp(3), data.io.resp(2), data.io.resp(1), data.io.resp(0), s1_read, s1_valid )

    ///printf ("\n\n[checkdatamod] [tlout %x %x %x %x %x] <== [%x %x %x %x] %b %b \n\n",modified2_s1_all_data_ways(4),modified2_s1_all_data_ways(3), modified2_s1_all_data_ways(2), modified2_s1_all_data_ways(1), modified2_s1_all_data_ways(0),data_modified2.io.resp(3), data_modified2.io.resp(2), data_modified2.io.resp(1), data_modified2.io.resp(0), s1_read, s1_valid )

    ///printf ("[checkdatamod]%x resp.vector %x %x %x %x   \n\n",s1_req.addr(11,0), data_modified2.io.resp_vector(3), data_modified2.io.resp_vector(2), data_modified2.io.resp_vector(1), data_modified2.io.resp_vector(0))
  }
  
  val s1_mask = Mux(s1_req.cmd === M_PWR, io.cpu.s1_data.mask, new StoreGen(s1_req.typ, s1_req.addr, UInt(0), wordBytes).mask)
  printf("[checkcachecounter]@@@@@@@@@@@@@@@@@@@@@@incache s1_mask %x io.cpu.s1_data.mask %x s1_req.typ %x s1_req.addr %x wordByte %x\n", s1_mask, io.cpu.s1_data.mask, s1_req.typ, s1_req.addr, wordBytes)
  val s2_valid_pre_xcpt = Reg(next=s1_valid_masked && !s1_sfence, init=Bool(false))
  val s2_valid = s2_valid_pre_xcpt && !io.cpu.s2_xcpt.asUInt.orR
  val s2_probe = Reg(next=s1_probe, init=Bool(false))
  val releaseInFlight = s1_probe || s2_probe || release_state =/= s_ready
  val s2_valid_masked = s2_valid && Reg(next = !s1_nack)
  val s2_req = Reg(io.cpu.req.bits)
  val s2_req_block_addr = (s2_req.addr >> idxLSB) << idxLSB
  val s2_uncached = Reg(Bool())
  val s2_uncached_resp_addr = Reg(UInt()) // should be DCE'd in synthesis
  when (s1_valid_not_nacked || s1_flush_valid) {
    s2_req := s1_req
    s2_req.addr := s1_paddr
    //zazad begins
    s2_req.return_addr := s1_req.addr
    //zazad ends
    s2_uncached := !tlb.io.resp.cacheable
  }
  val s2_read = isRead(s2_req.cmd)
  val s2_write = isWrite(s2_req.cmd)
  //zazad begins
  val s2_isvec = s2_req.isvector
  val s2_vector_cache_access_type = s2_req.vector_cache_access_type
  val s2_cnt_cache_vsd = s2_req.cnt_cache_vsd 
  //zazad ends
  val s2_readwrite = s2_read || s2_write
  val s2_flush_valid_pre_tag_ecc = RegNext(s1_flush_valid)
  val s1_meta_decoded = s1_meta.map(tECC.decode(_))
  val s1_meta_clk_en = s1_valid_not_nacked || s1_flush_valid || s1_probe
  val s2_meta_correctable_errors = s1_meta_decoded.map(m => RegEnable(m.correctable, s1_meta_clk_en)).asUInt
  val s2_meta_uncorrectable_errors = s1_meta_decoded.map(m => RegEnable(m.uncorrectable, s1_meta_clk_en)).asUInt
  val s2_meta_error_uncorrectable = s2_meta_uncorrectable_errors.orR
  val s2_meta_corrected = s1_meta_decoded.map(m => RegEnable(m.corrected, s1_meta_clk_en).asTypeOf(new L1Metadata))
  val s2_meta_error = (s2_meta_uncorrectable_errors | s2_meta_correctable_errors).orR
  val s2_flush_valid = s2_flush_valid_pre_tag_ecc && !s2_meta_error
  val s2_data = {
    val en = s1_valid || inWriteback || tl_out.d.fire()
    when (en){
       // printf("[checkdata]s1_all_data_way  way %d addr%x  %x  %x  %x  %x %x\n",s1_data_way, s1_req.addr(11,0), s1_all_data_ways(3), s1_all_data_ways(2), s1_all_data_ways(1), s1_all_data_ways(0), s1_all_data_ways(4))

     //  printf("[hateit]s1_all_data_way  way %d addr%x  %x  %x  %x  %x %x\n",s1_data_way, s1_req.addr(11,0), s1_all_data_ways(3), s1_all_data_ways(2), s1_all_data_ways(1), s1_all_data_ways(0), s1_all_data_ways(4))

        ///printf("[CacheChecksmain]%x rmain s1_hit/data_way %x\n",s1_req.addr(11,0), s1_data_way)
        ///printf("[CacheChecksmain]%x rmain s1_hit/s1_all_data_way %x  %x  %x  %x\n",s1_req.addr(11,0), s1_all_data_ways(3), s1_all_data_ways(2), s1_all_data_ways(1), s1_all_data_ways(0))
    }
    if (cacheParams.pipelineWayMux && nWays > 1) {
      val s2_data_way = RegEnable(s1_data_way, en) 
      //comment_final_dcache val s2_all_data_ways = (0 to nWays).map(i => RegEnable(s1_all_data_ways(i), en && s1_data_way(i)))
       //comment_final_dcache Mux1H(s2_data_way, s2_all_data_ways)
    } else {
      //comment_final_dcache RegEnable(Mux1H(s1_data_way, s1_all_data_ways), en)
    }
  }

  val help_to_print_s2_data_way = RegEnable(s1_data_way, s1_valid || inWriteback || tl_out.d.fire())
 //comment_final_dcache  val help_to_print_s2_all_data_ways = (0 to nWays).map(i => RegEnable(s1_all_data_ways(i), ( s1_valid || inWriteback || tl_out.d.fire()) && s1_data_way(i)))
  ///printf("[checkdata]s2 en %b way [s1 s2][%d  %d] addr%x  %x  %x  %x  %x %x\n",s1_valid || inWriteback || tl_out.d.fire(),s1_data_way, help_to_print_s2_data_way, s2_req.addr(11,0), help_to_print_s2_all_data_ways(3), help_to_print_s2_all_data_ways(2), help_to_print_s2_all_data_ways(1), help_to_print_s2_all_data_ways(0), help_to_print_s2_all_data_ways(4))

  //zazad begins
  val modified2_s2_data = {
    val en = s1_valid || inWriteback || tl_out.d.fire()
    when (en){
     ///   printf("[checkdata]mods1addr%x way %d  %x  %x  %x  %x %x \n",s1_req.addr(11,0),s1_data_way, modified2_s1_all_data_ways(3), modified2_s1_all_data_ways(2), modified2_s1_all_data_ways(1), modified2_s1_all_data_ways(0), modified2_s1_all_data_ways(4))
      ///  printf("[CacheChecksmodified]%x modified s1_hit/s1_data_way %x\n",s1_req.addr(11,0), s1_data_way)
       /// printf("[CacheChecksmodified]%x modified s1_hit/s1_all_data_way %x  %x  %x  %x\n",s1_req.addr(11,0), modified2_s1_all_data_ways(3), modified2_s1_all_data_ways(2), modified2_s1_all_data_ways(1), modified2_s1_all_data_ways(0))
      }
    if (cacheParams.pipelineWayMux && nWays > 1) {
      val s2_data_way = RegEnable(s1_data_way, en)
      val modified2_s2_all_data_ways = (0 to nWays).map(i => RegEnable(modified2_s1_all_data_ways(i), en && s1_data_way(i)))
      Mux1H(s2_data_way, modified2_s2_all_data_ways)
    } else {
      RegEnable(Mux1H(s1_data_way, modified2_s1_all_data_ways), en)
    }
  }


  val modified2_s2_data_vector = {
    val en = s1_valid || inWriteback || tl_out.d.fire()
    if (cacheParams.pipelineWayMux && nWays > 1) {
      val s2_data_way = RegEnable(s1_data_way, en)
      val modified2_s2_all_data_ways_vectors = (0 to nWays).map(i => RegEnable(modified2_s1_all_data_ways_vectors(i), en && s1_data_way(i)))
      Mux1H(s2_data_way, modified2_s2_all_data_ways_vectors)
    } else {
      RegEnable(Mux1H(s1_data_way, modified2_s1_all_data_ways_vectors), en)
    }
  }

  val help_to_print_modified2_s2_all_data_ways = (0 to nWays).map(i => RegEnable(modified2_s1_all_data_ways(i), ( s1_valid || inWriteback || tl_out.d.fire()) && s1_data_way(i)))

  ///printf("[checkdatamod]s2 en %b way [s1 s2][%d  %d] addr%x  %x  %x  %x  %x %x\n",s1_valid || inWriteback || tl_out.d.fire(),s1_data_way, help_to_print_s2_data_way, s2_req.addr(11,0), help_to_print_modified2_s2_all_data_ways(3), help_to_print_modified2_s2_all_data_ways(2), help_to_print_modified2_s2_all_data_ways(1), help_to_print_modified2_s2_all_data_ways(0), help_to_print_modified2_s2_all_data_ways(4))

 // printf ("[checkdata] s1/s2 data_way  %d %d \n", s1_data_way, help_to_print_s2_data_way)
  //printf ("[hateit] s1/s2 data_way  %d %d \n", s1_data_way, help_to_print_s2_data_way)

  //zazad ends

  val s2_probe_way = RegEnable(s1_hit_way, s1_probe)
  val s2_probe_state = RegEnable(s1_hit_state, s1_probe)
  val s2_hit_way = RegEnable(s1_hit_way, s1_valid_not_nacked)
  val s2_hit_state = RegEnable(s1_hit_state, s1_valid_not_nacked || s1_flush_valid)
  val s2_waw_hazard = RegEnable(s1_waw_hazard, s1_valid_not_nacked)
  val s2_store_merge = Wire(Bool())
  val s2_hit_valid = s2_hit_state.isValid()
  val (s2_hit, s2_grow_param, s2_new_hit_state) = s2_hit_state.onAccess(s2_req.cmd)
  //comment_final_dcache val s2_data_decoded = decodeData(s2_data)

  //zazad begins
  val modified2_s2_data_decoded = decodeData(modified2_s2_data)
  //zazad ends

  when(s2_read){
  ///  printf("[CacheChecksmain]%x s2_read-true s2_data %x \n",s2_req.addr (11,0), s2_data)
       printf("[checkcachecounter]**************%x s2_read modified2_s2_data %x \n",s2_req.addr (11,0), modified2_s2_data)
  }

  val s2_word_idx = s2_req.addr.extract(log2Up(rowBits/8)-1, log2Up(wordBytes))
  val s2_did_read = RegEnable(s1_did_read, s1_valid_not_nacked)
  //val s2_data_error = s2_did_read && (s2_data_decoded.map(_.error).grouped(wordBits/eccBits).map(_.reduce(_||_)).toSeq)(s2_word_idx)
  //val s2_data_error_uncorrectable = (s2_data_decoded.map(_.uncorrectable).grouped(wordBits/eccBits).map(_.reduce(_||_)).toSeq)(s2_word_idx)
  //comment_final_dcache val s2_data_corrected = (s2_data_decoded.map(_.corrected): Seq[UInt]).asUInt
  //comment_final_dcache val s2_data_uncorrected = (s2_data_decoded.map(_.uncorrected): Seq[UInt]).asUInt
  val s2_valid_hit_pre_data_ecc = s2_valid_masked && s2_readwrite && !s2_meta_error && s2_hit
  //val s2_valid_data_error = s2_valid_hit_pre_data_ecc && s2_data_error && can_acquire_before_release

  //zazad begins
  val modified2_s2_data_error = s2_did_read && (modified2_s2_data_decoded.map(_.error).grouped(wordBits/eccBits).map(_.reduce(_||_)).toSeq)(s2_word_idx)
  val modified2_s2_data_error_uncorrectable = (modified2_s2_data_decoded.map(_.uncorrectable).grouped(wordBits/eccBits).map(_.reduce(_||_)).toSeq)(s2_word_idx)
  val modified2_s2_data_corrected = (modified2_s2_data_decoded.map(_.corrected): Seq[UInt]).asUInt
  val modified2_s2_data_uncorrected = (modified2_s2_data_decoded.map(_.uncorrected): Seq[UInt]).asUInt
  val modified2_s2_valid_data_error = s2_valid_hit_pre_data_ecc && modified2_s2_data_error && can_acquire_before_release
  //zazad ends
 /// printf ("[checkcoding] %x %x %x \n", s2_data_uncorrected, s2_data,s2_data_corrected)
  val s2_valid_hit = s2_valid_hit_pre_data_ecc  && !modified2_s2_data_error && (!s2_waw_hazard || s2_store_merge)
  val s2_valid_miss = s2_valid_masked && s2_readwrite  && !s2_meta_error  && !s2_hit && can_acquire_before_release
  val s2_valid_cached_miss = s2_valid_miss && !s2_uncached && !uncachedInFlight.asUInt.orR
  dontTouch(s2_valid_cached_miss)
  val s2_victimize = Bool(!usingDataScratchpad) && (s2_valid_cached_miss || modified2_s2_valid_data_error || s2_flush_valid)
  val s2_valid_uncached_pending = s2_valid_miss && s2_uncached && !uncachedInFlight.asUInt.andR
  val s2_victim_way = Mux(s2_hit_valid, s2_hit_way, UIntToOH(RegEnable(s1_victim_way, s1_valid_not_nacked || s1_flush_valid)))
  val s2_victim_tag = Mux(modified2_s2_valid_data_error, s2_req.addr >> untagBits, RegEnable(s1_victim_meta.tag, s1_valid_not_nacked || s1_flush_valid))
  val s2_victim_state = Mux(s2_hit_valid, s2_hit_state, RegEnable(s1_victim_meta.coh, s1_valid_not_nacked || s1_flush_valid))

  val (s2_prb_ack_data, s2_report_param, probeNewCoh)= s2_probe_state.onProbe(probe_bits.param)
  val (s2_victim_dirty, s2_shrink_param, voluntaryNewCoh) = s2_victim_state.onCacheControl(M_FLUSH)
  dontTouch(s2_victim_dirty)
  val s2_update_meta = s2_hit_state =/= s2_new_hit_state
  io.cpu.s2_nack := s2_valid && !s2_valid_hit && !(s2_valid_uncached_pending && tl_out_a.ready)
  when (io.cpu.s2_nack || (s2_valid_hit && s2_update_meta)) { s1_nack := true
    //printf("[justprintinsts] s1_nack io.cpu.s2_nack || (s2_valid_hit && s2_update_meta \n")
  }

  // tag updates on ECC errors
  metaArb.io.in(1).valid := s2_meta_error && (s2_valid_masked || s2_flush_valid_pre_tag_ecc || s2_probe)
  metaArb.io.in(1).bits.write := true
  metaArb.io.in(1).bits.way_en := s2_meta_uncorrectable_errors | Mux(s2_meta_error_uncorrectable, 0.U, PriorityEncoderOH(s2_meta_correctable_errors))
  metaArb.io.in(1).bits.addr := Cat(io.cpu.req.bits.addr >> untagBits, Mux(s2_probe, probe_bits.address, s2_req.addr)(idxMSB, 0))
  metaArb.io.in(1).bits.data := PriorityMux(s2_meta_correctable_errors, s2_meta_corrected)
  when (s2_meta_error_uncorrectable) { metaArb.io.in(1).bits.data.coh := ClientMetadata.onReset }

  // tag updates on hit/miss
  metaArb.io.in(2).valid := (s2_valid_hit && s2_update_meta) || (s2_victimize && !s2_victim_dirty)
  metaArb.io.in(2).bits.write := true
  metaArb.io.in(2).bits.way_en := s2_victim_way
  metaArb.io.in(2).bits.addr := Cat(io.cpu.req.bits.addr >> untagBits, s2_req.addr(idxMSB, 0))
  metaArb.io.in(2).bits.data.coh := Mux(s2_valid_hit, s2_new_hit_state, ClientMetadata.onReset)
  metaArb.io.in(2).bits.data.tag := s2_req.addr >> untagBits

  //zaza begins
 // io.cpu.s2_nack := s2_valid && !s2_valid_hit && !(s2_valid_uncached_pending && tl_out_a.ready)
  //////////////////printf("[checkcachecounter]addr io.cpu.req %x s1_req %x s1_type [%d]  s2_req %x hit_way s1 %d s2 %d s2validhit %b s2_hit %b s2_nack %b  s1_valid_not_nacked %b s1_valid %b !s1_nack %b [s2_valid_hit %b %b %b] [s2_valid_hit_pre_data_ecc %b %b %b %b %b] s2_valid %b [%b [%b [%b %b]  %b] %b] \n", io.cpu.req.bits.addr, s1_req.addr,s1_req.typ, s2_req.addr, s1_hit_way,s2_hit_way, s2_valid_hit,s2_hit, io.cpu.s2_nack, s1_valid_not_nacked,s1_valid ,!s1_nack, s2_valid_hit_pre_data_ecc, !s2_data_error , (!s2_waw_hazard || s2_store_merge), s2_valid_hit_pre_data_ecc, s2_valid_masked, s2_readwrite, !s2_meta_error, s2_hit, s2_valid,s2_valid_pre_xcpt,s1_valid_masked, s1_valid , !io.cpu.s1_kill, !s1_sfence,!io.cpu.s2_xcpt.asUInt.orR)
  //zazad ends

  // load reservations and TL error reporting
  val s2_lr = Bool(usingAtomics && !usingDataScratchpad) && s2_req.cmd === M_XLR
  val s2_sc = Bool(usingAtomics && !usingDataScratchpad) && s2_req.cmd === M_XSC
  val lrscCount = Reg(init=UInt(0))
  val tl_error_valid = RegInit(false.B)
  val lrscValid = lrscCount > lrscBackoff
  val lrscAddr = Reg(UInt())
  val lrscAddrMatch = lrscAddr === (s2_req.addr >> blockOffBits)
  val s2_sc_fail = s2_sc && !(lrscValid && lrscAddrMatch)
  val s2_tl_error = tl_error_valid && lrscAddrMatch
  when (s2_valid_hit && s2_lr && !cached_grant_wait || s2_valid_cached_miss) {
    tl_error_valid := false
    lrscCount := Mux(s2_hit, lrscCycles - 1, 0.U)
    lrscAddr := s2_req.addr >> blockOffBits
  }
  when (lrscCount > 0) { lrscCount := lrscCount - 1 }
  when ((s2_valid_masked && lrscCount > 0) || io.cpu.invalidate_lr) { lrscCount := 0 }
  when (s2_valid_masked || io.cpu.invalidate_lr) { tl_error_valid := false }

  // don't perform data correction if it might clobber a recent store
  val s2_correct = modified2_s2_data_error && !any_pstore_valid && !RegNext(any_pstore_valid) && Bool(usingDataScratchpad)

  //zazad beginds
   val modified2_s2_correct = modified2_s2_data_error && !any_pstore_valid && !RegNext(any_pstore_valid) && Bool(usingDataScratchpad)
  //zazad ends
  // pending store buffer
  val s2_valid_correct = s2_valid_hit_pre_data_ecc  && s2_correct
  val s2_store_valid = s2_valid_hit && s2_write && !s2_sc_fail
  val pstore1_cmd = RegEnable(s1_req.cmd, s1_valid_not_nacked && s1_write)
  val pstore1_addr = RegEnable(s1_paddr, s1_valid_not_nacked && s1_write)
  val pstore1_data = RegEnable(/*io.cpu.s1_data.data*/io.cpu.DcacheCpu_s1_data.data(63,0), s1_valid_not_nacked && s1_write)

  //zazad begins
  val pstore1_isvec = RegEnable(s1_req.isvector, s1_valid_not_nacked && s1_write)
  val pstore1_vector_cache_access_type = RegEnable(s1_req.vector_cache_access_type, s1_valid_not_nacked && s1_write)
  val pstore1_cnt_cache_vsd = RegEnable(s1_req.cnt_cache_vsd, s1_valid_not_nacked && s1_write)
  val vector_pstore1_data = RegEnable(io.cpu.DcacheCpu_s1_data.data, s1_valid_not_nacked && s1_write)
  when (pstore1_isvec){
    ///printf("[trackloadsnew] vector_pstore1_data %x  isvec %b addr %x\n", vector_pstore1_data, pstore1_isvec, pstore1_addr(11, 0))
  }
    //zazad ends
  val pstore1_way = RegEnable(s1_hit_way, s1_valid_not_nacked && s1_write)
  val pstore1_mask = RegEnable(s1_mask, s1_valid_not_nacked && s1_write)
  val pstore1_storegen_data = Wire(init = pstore1_data)

  //zazad begins
   val vector_pstore1_storegen_data = Wire(init = vector_pstore1_data)
  //zazad ends
  val pstore1_rmw = Bool(usingRMW) && RegEnable(needsRead(s1_req), s1_valid_not_nacked && s1_write)
  val pstore1_valid = Wire(Bool())
  val pstore1_merge = s2_store_valid && s2_store_merge
  val pstore2_valid = Reg(Bool())
  any_pstore_valid := pstore1_valid || pstore2_valid
  val pstore_drain_structural = pstore1_valid && pstore2_valid && ((s1_valid && s1_write) || pstore1_rmw)
  val pstore_drain_opportunistic = !(io.cpu.req.valid && s0_needsRead)
  val pstore_drain_on_miss = releaseInFlight || (s2_valid && !s2_valid_hit && !s2_valid_uncached_pending)
  ccover(pstore_drain_structural, "STORE_STRUCTURAL_HAZARD", "D$ read-modify-write structural hazard")
  ccover(pstore1_valid && pstore_drain_on_miss, "STORE_DRAIN_ON_MISS", "D$ store buffer drain on miss")
  ccover(s1_valid_not_nacked && s1_waw_hazard, "WAW_HAZARD", "D$ write-after-write hazard")
  val pstore_drain = !pstore1_merge &&
    (Bool(usingRMW) && pstore_drain_structural ||
     (((pstore1_valid && !pstore1_rmw) || pstore2_valid) && (pstore_drain_opportunistic || pstore_drain_on_miss)))
  pstore1_valid := {
    val pstore1_held = Reg(Bool())
    assert(!s2_store_valid || !pstore1_held)
    pstore1_held := (s2_store_valid && !s2_store_merge || pstore1_held) && pstore2_valid && !pstore_drain
    s2_store_valid || pstore1_held
  }
  val advance_pstore1 = (pstore1_valid || s2_valid_correct) && (pstore2_valid === pstore_drain)
  pstore2_valid := pstore2_valid && !pstore_drain || advance_pstore1
  val pstore2_addr = RegEnable(Mux(s2_correct, s2_req.addr, pstore1_addr), advance_pstore1)
  val pstore2_way = RegEnable(Mux(s2_correct, s2_hit_way, pstore1_way), advance_pstore1)
  val pstore2_storegen_data = {
    for (i <- 0 until wordBytes)
      yield RegEnable(pstore1_storegen_data(8*(i+1)-1, 8*i), advance_pstore1 || pstore1_merge && pstore1_mask(i))
  }.asUInt

  //zazad begins
  val vector_pstore2_storegen_data = {
    for (i <- 0 until 32)
      yield RegEnable(vector_pstore1_storegen_data(8*(i+1)-1, 8*i), advance_pstore1 || pstore1_merge)
  }.asUInt

  val pstore2_isvec =RegEnable(pstore1_isvec, advance_pstore1)
  val pstore2_vector_cache_access_type = RegEnable (pstore1_vector_cache_access_type, advance_pstore1)
  val pstore2_cnt_cache_vsd = RegEnable (pstore1_cnt_cache_vsd, advance_pstore1)
  //zazad ends
  val pstore2_storegen_mask = {
    val mask = Reg(UInt(width = wordBytes))
    when (advance_pstore1 || pstore1_merge) {
      val mergedMask = pstore1_mask | Mux(pstore1_merge, mask, 0.U)
      mask := ~Mux(s2_correct, 0.U, ~mergedMask)
    }
    mask
  }

  when (pstore2_isvec){

    ///printf("[trackloadsnew] %x %b %x\n", pstore1_addr, pstore1_isvec, pstore2_storegen_data)
  }
    s2_store_merge := (if (eccBytes == 1) false.B else {
    ccover(pstore1_merge, "STORE_MERGED", "D$ store merged")
    // only merge stores to ECC granules that are already stored-to, to avoid
    // WAW hazards
    val wordMatch = (eccMask(pstore2_storegen_mask) | ~eccMask(pstore1_mask)).andR
    val idxMatch = s2_req.addr(untagBits-1, log2Ceil(wordBytes)) === pstore2_addr(untagBits-1, log2Ceil(wordBytes))
    val tagMatch = (s2_hit_way & pstore2_way).orR
    pstore2_valid && wordMatch && idxMatch && tagMatch
  })


  //printf("[checkcachecounter] correct[%b %b] s2_store-valid %b[s2validhit %b s2write %b] pstore1addr %x [s1paddr %x validnotbackedandwrite %b] data[%x] pstore1merge %b[%b %b] any_pstore_valid %b[%b %b] [structural %b opportunastic %b] pstore_drain_on_miss %b [releaseinflight %b s2valid %b s2validhit %b !s2uncachedpending %b] [drain %b held/valid %b] advance_pstore_1 %b [%b %b %b %b] pstore2_addr %x s2data %x\n", s2_correct, s2_valid_correct,s2_store_valid, s2_valid_hit, s2_write, pstore1_addr, s1_paddr, s1_valid_not_nacked && s1_write, vector_pstore1_data, pstore1_merge, s2_store_valid, s2_store_merge, any_pstore_valid, pstore1_valid, pstore2_valid, pstore_drain_structural,pstore_drain_opportunistic,pstore_drain_on_miss,releaseInFlight,s2_valid ,s2_valid_hit,!s2_valid_uncached_pending,pstore_drain, pstore1_valid, advance_pstore1,pstore1_valid ,s2_valid_correct,pstore2_valid,pstore_drain, pstore2_addr, vector_pstore2_storegen_data)

  dataArb.io.in(0).valid := pstore_drain
  dataArb.io.in(0).bits.write := true
  dataArb.io.in(0).bits.addr := Mux(pstore2_valid, pstore2_addr, pstore1_addr)
  dataArb.io.in(0).bits.way_en := Mux(pstore2_valid, pstore2_way, pstore1_way)
  dataArb.io.in(0).bits.wdata := Fill(rowWords, Mux(pstore2_valid, pstore2_storegen_data, pstore1_data))
  //zazad begins
  dataArb.io.in(0).bits.vwdata := Fill(rowWords, Mux(pstore2_valid, vector_pstore2_storegen_data,vector_pstore1_data))
  dataArb.io.in(0).bits.isvec  := Mux(pstore2_valid, pstore2_isvec, pstore1_isvec)
  dataArb.io.in(0).bits.vector_cache_access_type  := Mux(pstore2_valid, pstore2_vector_cache_access_type, pstore1_vector_cache_access_type)
  dataArb.io.in(0).bits.cnt_cache_vsd  := Mux(pstore2_valid, pstore2_cnt_cache_vsd, pstore1_cnt_cache_vsd)
  dataArb.io.in(0).bits.element_number := io.cpu.req.bits.element_number

    printf("[checkcachecounter]@@@@@@@@@@@@@@@@@@@@incache dataarb.io.in0 for write  valid %x write %x addr %x element# %x access_type %x \n", dataArb.io.in(0).valid, dataArb.io.in(0).bits.write, dataArb.io.in(0).bits.addr,dataArb.io.in(0).bits.element_number, dataArb.io.in(0).bits.vector_cache_access_type)

  when (pstore2_isvec){
    ////printf("[trackloadsnew] pstore2==>dataarray  vwdata %x  isvec %b  \n", dataArb.io.in(0).bits.vwdata,dataArb.io.in(0).bits.isvec)
  }
  //zazad ends
  dataArb.io.in(0).bits.wordMask := UIntToOH(Mux(pstore2_valid, pstore2_addr, pstore1_addr).extract(rowOffBits-1,offsetlsb))
  dataArb.io.in(0).bits.eccMask := eccMask(Mux(pstore2_valid, pstore2_storegen_mask, pstore1_mask))

  // store->load RAW hazard detection
  def s1Depends(addr: UInt, mask: UInt) =
    addr(idxMSB, wordOffBits) === s1_req.addr(idxMSB, wordOffBits) &&
    Mux(s1_write, (eccByteMask(mask) & eccByteMask(s1_mask)).orR, (mask & s1_mask).orR)
  val s1_hazard =
    (pstore1_valid && s1Depends(pstore1_addr, pstore1_mask)) ||
     (pstore2_valid && s1Depends(pstore2_addr, pstore2_storegen_mask))
  val s1_raw_hazard = s1_read && s1_hazard
  s1_waw_hazard := (if (eccBytes == 1) false.B else {
    ccover(s1_valid_not_nacked && s1_waw_hazard, "WAW_HAZARD", "D$ write-after-write hazard")
    s1_write && (s1_hazard || needsRead(s1_req) && !s1_did_read)
  })
  when (s1_valid && s1_raw_hazard) { s1_nack := true
   // printf ("[justprintinsts] s1_nack in s1_valid && s1_raw_hazard \n")
  }

  // Prepare a TileLink request message that initiates a transaction
  val a_source = PriorityEncoder(~uncachedInFlight.asUInt << mmioOffset) // skip the MSHR
  val acquire_address = s2_req_block_addr
  val access_address = s2_req.addr
  val a_size = mtSize(s2_req.typ)
  val a_data = Fill(beatWords, pstore1_data)
  val acquire = if (edge.manager.anySupportAcquireT) {
    edge.AcquireBlock(UInt(0), acquire_address, lgCacheBlockBytes, s2_grow_param)._2 // Cacheability checked by tlb
  } else {
    Wire(new TLBundleA(edge.bundle))
  }
  val get     = edge.Get(a_source, access_address, a_size)._2
  val put     = edge.Put(a_source, access_address, a_size, a_data)._2
  val atomics = if (edge.manager.anySupportLogical) {
    MuxLookup(s2_req.cmd, Wire(new TLBundleA(edge.bundle)), Array(
      M_XA_SWAP -> edge.Logical(a_source, access_address, a_size, a_data, TLAtomics.SWAP)._2,
      M_XA_XOR  -> edge.Logical(a_source, access_address, a_size, a_data, TLAtomics.XOR) ._2,
      M_XA_OR   -> edge.Logical(a_source, access_address, a_size, a_data, TLAtomics.OR)  ._2,
      M_XA_AND  -> edge.Logical(a_source, access_address, a_size, a_data, TLAtomics.AND) ._2,
      M_XA_ADD  -> edge.Arithmetic(a_source, access_address, a_size, a_data, TLAtomics.ADD)._2,
      M_XA_MIN  -> edge.Arithmetic(a_source, access_address, a_size, a_data, TLAtomics.MIN)._2,
      M_XA_MAX  -> edge.Arithmetic(a_source, access_address, a_size, a_data, TLAtomics.MAX)._2,
      M_XA_MINU -> edge.Arithmetic(a_source, access_address, a_size, a_data, TLAtomics.MINU)._2,
      M_XA_MAXU -> edge.Arithmetic(a_source, access_address, a_size, a_data, TLAtomics.MAXU)._2))
  } else {
    // If no managers support atomics, assert fail if processor asks for them
    assert (!(tl_out_a.valid && s2_read && s2_write && s2_uncached))
    Wire(new TLBundleA(edge.bundle))
  }

  tl_out_a.valid := (s2_valid_cached_miss && (Bool(cacheParams.acquireBeforeRelease) || !s2_victim_dirty)) || s2_valid_uncached_pending
  tl_out_a.bits := Mux(!s2_uncached, acquire, Mux(!s2_write, get, Mux(!s2_read, put, atomics)))

  // Set pending bits for outstanding TileLink transaction
  val a_sel = UIntToOH(a_source, maxUncachedInFlight+mmioOffset) >> mmioOffset
  when (tl_out_a.fire()) {
    when (s2_uncached) {
      (a_sel.toBools zip (uncachedInFlight zip uncachedReqs)) foreach { case (s, (f, r)) =>
        when (s) {
          f := Bool(true)
          r := s2_req
        }
      }
    }.otherwise {
      cached_grant_wait := true
    }
  }

  // grant
  val (d_first, d_last, d_done, d_address_inc) = edge.addr_inc(tl_out.d)
  val grantIsCached = {
    val res = tl_out.d.bits.opcode.isOneOf(Grant, GrantData)
    if (usingDataScratchpad) {
      assert(!(tl_out.d.valid && res))
      false.B
    } else {
      res
    }
  }
  val grantIsUncached = tl_out.d.bits.opcode.isOneOf(AccessAck, AccessAckData, HintAck)
  val grantIsUncachedData = tl_out.d.bits.opcode === AccessAckData
  val grantIsVoluntary = tl_out.d.bits.opcode === ReleaseAck // Clears a different pending bit
  val grantIsRefill = tl_out.d.bits.opcode === GrantData     // Writes the data array
  val grantInProgress = Reg(init=Bool(false))
  val blockProbeAfterGrantCount = Reg(init=UInt(0))
  when (blockProbeAfterGrantCount > 0) { blockProbeAfterGrantCount := blockProbeAfterGrantCount - 1 }
  val canAcceptCachedGrant = if (cacheParams.acquireBeforeRelease) !release_state.isOneOf(s_voluntary_writeback, s_voluntary_write_meta) else true.B
  tl_out.d.ready := Mux(grantIsCached, (!d_first || tl_out.e.ready) && canAcceptCachedGrant, true.B)
  when (tl_out.d.fire()) {
    when (grantIsCached) {
      grantInProgress := true
      assert(cached_grant_wait, "A GrantData was unexpected by the dcache.")
      when(d_last) {
        tl_error_valid := tl_out.d.bits.error
        cached_grant_wait := false
        grantInProgress := false
        blockProbeAfterGrantCount := blockProbeAfterGrantCycles - 1
        replacer.miss
      }
    } .elsewhen (grantIsUncached) {
      val d_sel = UIntToOH(tl_out.d.bits.source, maxUncachedInFlight+mmioOffset) >> mmioOffset
      val req = Mux1H(d_sel, uncachedReqs)
      (d_sel.toBools zip uncachedInFlight) foreach { case (s, f) =>
        when (s && d_last) {
          assert(f, "An AccessAck was unexpected by the dcache.") // TODO must handle Ack coming back on same cycle!
          f := false
        }
      }
      when (grantIsUncachedData) {
        s1_data_way := 1.U << nWays
        s2_req.cmd := M_XRD
        s2_req.typ := req.typ
        s2_req.tag := req.tag
        s2_req.addr := Cat(s1_paddr >> beatOffBits /* don't-care */, req.addr(beatOffBits-1, 0))
        s2_uncached_resp_addr := req.addr
      }
    } .elsewhen (grantIsVoluntary) {
      assert(release_ack_wait, "A ReleaseAck was unexpected by the dcache.") // TODO should handle Ack coming back on same cycle!
      release_ack_wait := false
    }
  }

  // Finish TileLink transaction by issuing a GrantAck
  tl_out.e.valid := tl_out.d.valid && d_first && grantIsCached && canAcceptCachedGrant
  tl_out.e.bits := edge.GrantAck(tl_out.d.bits)
  assert(tl_out.e.fire() === (tl_out.d.fire() && d_first && grantIsCached))

  // data refill
  // note this ready-valid signaling ignores E-channel backpressure, which
  // benignly means the data RAM might occasionally be redundantly written
  dataArb.io.in(1).valid := tl_out.d.valid && grantIsRefill && canAcceptCachedGrant
  when (grantIsRefill && !dataArb.io.in(1).ready) {
    tl_out.e.valid := false
    tl_out.d.ready := false
  }
  dataArb.io.in(1).bits.write := true
  dataArb.io.in(1).bits.addr :=  s2_req_block_addr | d_address_inc
  dataArb.io.in(1).bits.way_en := s2_victim_way
  //needstobereplaced
  dataArb.io.in(1).bits.wdata := tl_out.d.bits.data
  dataArb.io.in(1).bits.wordMask := ~UInt(0, rowBytes / wordBytes)
  dataArb.io.in(1).bits.eccMask := ~UInt(0, wordBytes / eccBytes)

  when (tl_out.d.valid && grantIsRefill && canAcceptCachedGrant){
    ///printf("[TCSAddr]%x  tl wdata %x \n", s2_req_block_addr | d_address_inc(11,0),tl_out.d.bits.data)
  }

  // tag updates on refill
  // ignore backpressure from metaArb, which can only be caused by tag ECC
  // errors on hit-under-miss.  failing to write the new tag will leave the
  // line invalid, so we'll simply request the line again later.
  metaArb.io.in(3).valid := grantIsCached && d_done && !tl_out.d.bits.error
  metaArb.io.in(3).bits.write := true
  metaArb.io.in(3).bits.way_en := s2_victim_way
  metaArb.io.in(3).bits.addr := Cat(io.cpu.req.bits.addr >> untagBits, s2_req.addr(idxMSB, 0))
  metaArb.io.in(3).bits.data.coh := s2_hit_state.onGrant(s2_req.cmd, tl_out.d.bits.param)
  metaArb.io.in(3).bits.data.tag := s2_req.addr >> untagBits
  // don't accept uncached grants if there's a structural hazard on s2_data...
  val blockUncachedGrant = Reg(Bool())
  blockUncachedGrant := dataArb.io.out.valid
  when (grantIsUncachedData && (blockUncachedGrant || s1_valid)) {
    tl_out.d.ready := false
    // ...but insert bubble to guarantee grant's eventual forward progress
    when (tl_out.d.valid) {
      io.cpu.req.ready := false
      dataArb.io.in(1).valid := true
      dataArb.io.in(1).bits.write := false
      blockUncachedGrant := !dataArb.io.in(1).ready
    }
  }
  ccover(tl_out.d.valid && !tl_out.d.ready, "BLOCK_D", "D$ D-channel blocked")

  // Handle an incoming TileLink Probe message
  val block_probe = releaseInFlight || grantInProgress || blockProbeAfterGrantCount > 0 || lrscValid || (s2_valid_hit && s2_lr)
  metaArb.io.in(6).valid := tl_out.b.valid && !block_probe
  tl_out.b.ready := metaArb.io.in(6).ready && !block_probe && !s1_valid && (!s2_valid || s2_valid_hit)
  metaArb.io.in(6).bits.write := false
  metaArb.io.in(6).bits.addr := Cat(io.cpu.req.bits.addr >> paddrBits, tl_out.b.bits.address)
  metaArb.io.in(6).bits.way_en := ~UInt(0, nWays)
  metaArb.io.in(6).bits.data := metaArb.io.in(4).bits.data

  // release
  val (c_first, c_last, releaseDone, c_count) = edge.count(tl_out_c)
  val releaseRejected = tl_out_c.valid && !tl_out_c.ready
  val s1_release_data_valid = Reg(next = dataArb.io.in(2).fire())
  val s2_release_data_valid = Reg(next = s1_release_data_valid && !releaseRejected)
  val releaseDataBeat = Cat(UInt(0), c_count) + Mux(releaseRejected, UInt(0), s1_release_data_valid + Cat(UInt(0), s2_release_data_valid))
  //val writeback_data_error = s2_data_decoded.map(_.error).reduce(_||_)
  //val writeback_data_uncorrectable = s2_data_decoded.map(_.uncorrectable).reduce(_||_)

  //zazad begins not required can be deleted just for error reports
  val modified2_writeback_data_error = modified2_s2_data_decoded.map(_.error).reduce(_||_)
  val modified2_writeback_data_uncorrectable = modified2_s2_data_decoded.map(_.uncorrectable).reduce(_||_)
//  printf("%x %x [%x %x] %x %x[%x %x] %x[%x] \n", modified2_writeback_data_error, modified2_writeback_data_uncorrectable, writeback_data_error, writeback_data_uncorrectable,modified2_s2_data_corrected, modified2_s2_data_uncorrected,s2_data_corrected, s2_data_uncorrected, modified2_s2_correct, s2_correct)
  //zazad ends
  val nackResponseMessage = edge.ProbeAck(b = probe_bits, reportPermissions = TLPermissions.NtoN)
  val cleanReleaseMessage = edge.ProbeAck(b = probe_bits, reportPermissions = s2_report_param)
  val dirtyReleaseMessage = edge.ProbeAck(b = probe_bits, reportPermissions = s2_report_param, data = 0.U)

  tl_out_c.valid := s2_release_data_valid
  tl_out_c.bits := nackResponseMessage
  val newCoh = Wire(init = probeNewCoh)
  releaseWay := s2_probe_way

  if (!usingDataScratchpad) {
    when (s2_victimize && s2_victim_dirty) {
      assert(!(s2_valid && s2_hit_valid && !modified2_s2_data_error))
      release_state := s_voluntary_writeback
      probe_bits.address := Cat(s2_victim_tag, s2_req.addr(idxMSB, idxLSB)) << idxLSB
    }
    when (s2_probe) {
      val probeNack = Wire(init = true.B)
      when (s2_meta_error) {
        release_state := s_probe_retry
      }.elsewhen (s2_prb_ack_data) {
        release_state := s_probe_rep_dirty
      }.elsewhen (s2_probe_state.isValid()) {
        tl_out_c.valid := true
        tl_out_c.bits := cleanReleaseMessage
        release_state := Mux(releaseDone, s_probe_write_meta, s_probe_rep_clean)
      }.otherwise {
        tl_out_c.valid := true
        probeNack := !releaseDone
        release_state := Mux(releaseDone, s_ready, s_probe_rep_miss)
      }
      when (probeNack) { s1_nack := true
        // printf("[justprintinsts] s1_nack  probeNack \n")
      }
    }
    when (release_state === s_probe_retry) {
      metaArb.io.in(6).valid := true
      metaArb.io.in(6).bits.addr := Cat(io.cpu.req.bits.addr >> paddrBits, probe_bits.address)
      when (metaArb.io.in(6).ready) {
        release_state := s_ready
        s1_probe := true
      }
    }
    when (release_state === s_probe_rep_miss) {
      tl_out_c.valid := true
      when (releaseDone) { release_state := s_ready }
    }
    when (release_state === s_probe_rep_clean) {
      tl_out_c.valid := true
      tl_out_c.bits := cleanReleaseMessage
      when (releaseDone) { release_state := s_probe_write_meta }
    }
    when (release_state === s_probe_rep_dirty) {
      tl_out_c.bits := dirtyReleaseMessage
      when (releaseDone) { release_state := s_probe_write_meta }
    }
    when (release_state.isOneOf(s_voluntary_writeback, s_voluntary_write_meta)) {
      tl_out_c.bits := edge.Release(fromSource = 0.U,
                                    toAddress = 0.U,
                                    lgSize = lgCacheBlockBytes,
                                    shrinkPermissions = s2_shrink_param,
                                    data = 0.U)._2
      newCoh := voluntaryNewCoh
      releaseWay := s2_victim_way
      when (releaseDone) { release_state := s_voluntary_write_meta }
      when (tl_out_c.fire() && c_first) { release_ack_wait := true }
    }
    tl_out_c.bits.address := probe_bits.address
    //needstobereplaced

    val select0 = (c_count === UInt(0)) || (c_count === UInt(4))
    val select1 = (c_count === UInt(1)) || (c_count === UInt(5))
    val select2 = (c_count === UInt(2)) || (c_count === UInt(6))
    //val select3 = ((c_count === UInt(3)) || (c_count === UInt(7)))
    val extracted_part_modified_data =Mux( select0, modified2_s2_data_vector(63,0), Mux(select1, modified2_s2_data_vector(127,64), Mux(select2 , modified2_s2_data_vector(191,128), modified2_s2_data_vector(255,192))))

          //printf("[testcacheagain] inwriteback %b s1_probe %b s2_probe %b tl_data %x data %x part_vec_data %x vector_data %x c[first %d last %d cout %d releasedatabeat %d ] done %b tl.fire %b addr %x [%x  %x]\n",inWriteback, s1_probe, s2_probe, tl_out_c.bits.data, s2_data_corrected,extracted_part_modified_data ,modified2_s2_data_corrected, c_first, c_last, c_count,releaseDataBeat,  releaseDone, tl_out_c.fire(), tl_out_c.bits.address | (releaseDataBeat(log2Up(refillCycles)-1,0) << rowOffBits), tl_out_c.bits.address , (releaseDataBeat(log2Up(refillCycles)-1,0) << rowOffBits))
    
    //tl_out_c.bits.data := s2_data_corrected
    tl_out_c.bits.data := extracted_part_modified_data
    tl_out_c.bits.error := inWriteback && {
      val accrued = Reg(Bool())
      val next = modified2_writeback_data_uncorrectable || (accrued && !c_first)
      when (tl_out_c.fire()) { accrued := next }
      next
    }
  }

  dataArb.io.in(2).valid := inWriteback && releaseDataBeat < refillCycles
  dataArb.io.in(2).bits.write := false
  dataArb.io.in(2).bits.addr := tl_out_c.bits.address | (releaseDataBeat(log2Up(refillCycles)-1,0) << rowOffBits)
  dataArb.io.in(2).bits.wordMask := ~UInt(0, rowBytes / wordBytes)
  dataArb.io.in(2).bits.way_en := ~UInt(0, nWays)


  metaArb.io.in(4).valid := release_state.isOneOf(s_voluntary_write_meta, s_probe_write_meta)
  metaArb.io.in(4).bits.write := true
  metaArb.io.in(4).bits.way_en := releaseWay
  metaArb.io.in(4).bits.addr := Cat(io.cpu.req.bits.addr >> untagBits, tl_out_c.bits.address(idxMSB, 0))
  metaArb.io.in(4).bits.data.coh := newCoh
  metaArb.io.in(4).bits.data.tag := tl_out_c.bits.address >> untagBits
  when (metaArb.io.in(4).fire()) { release_state := s_ready }

  // cached response
  io.cpu.resp.valid := s2_valid_hit
  io.cpu.resp.bits <> s2_req
  io.cpu.resp.bits.has_data := s2_read
  io.cpu.resp.bits.replay := false
  io.cpu.ordered := !(s1_valid || s2_valid || cached_grant_wait || uncachedInFlight.asUInt.orR)


  //printf("[checkcachecounter] %b %b %b %b %d %d %d %d\n", s1_vector_cache_access_type, s2_vector_cache_access_type, pstore1_vector_cache_access_type, pstore2_vector_cache_access_type, s1_cnt_cache_vsd,s2_cnt_cache_vsd, pstore1_cnt_cache_vsd, pstore2_cnt_cache_vsd)
  val s1_xcpt_valid = tlb.io.req.valid && !s1_nack
  val s1_xcpt = tlb.io.resp
  io.cpu.s2_xcpt := Mux(RegNext(s1_xcpt_valid), RegEnable(s1_xcpt, s1_valid_not_nacked), 0.U.asTypeOf(s1_xcpt))
  ccoverNotScratchpad(s2_valid_pre_xcpt && s2_tl_error, "D_ERROR_REPORTED", "D$ reported TL error to processor")
  when (s2_valid_pre_xcpt && s2_tl_error) {
    assert(!s2_valid_hit && !s2_uncached)
    when (s2_write) { io.cpu.s2_xcpt.ae.st := true }
    when (s2_read) { io.cpu.s2_xcpt.ae.ld := true }
  }

  printf("[checkcachecounter]TLLLLLLLLLLLLB tlb.io.req.valid %b s1_nack %b s1_xcpt_valid %b tlb.io.resp [ %b %x pf[ %b %b %b] ae[%b %b %b] ma[%b %b %b] %b %b] s1_valid_not_nacked %b \n", tlb.io.req.valid, s1_nack, s1_xcpt_valid, tlb.io.resp.miss,tlb.io.resp.paddr, tlb.io.resp.pf.ld, tlb.io.resp.pf.st, tlb.io.resp.pf.inst, tlb.io.resp.ae.ld, tlb.io.resp.ae.st, tlb.io.resp.ae.inst, tlb.io.resp.ma.ld, tlb.io.resp.ma.st, tlb.io.resp.ma.inst,  tlb.io.resp.cacheable, tlb.io.resp.prefetchable, s1_valid_not_nacked)
  val s2_isSlavePortAccess = s2_req.phys
  if (usingDataScratchpad) {
    require(!usingVM) // therefore, req.phys means this is a slave-port access
    when (s2_isSlavePortAccess) {
      assert(!s2_valid || s2_hit_valid)
      io.cpu.s2_xcpt := 0.U.asTypeOf(io.cpu.s2_xcpt)
    }
    assert(!(s2_valid_masked && s2_req.cmd.isOneOf(M_XLR, M_XSC)))
  } else {
    ccover(tl_out.b.valid && !tl_out.b.ready, "BLOCK_B", "D$ B-channel blocked")
  }

  // uncached response
  io.cpu.replay_next := tl_out.d.fire() && grantIsUncachedData
  val doUncachedResp = Reg(next = io.cpu.replay_next)
  when (doUncachedResp) {
    assert(!s2_valid_hit)
    io.cpu.resp.valid := true
    io.cpu.resp.bits.replay := true
    io.cpu.resp.bits.addr := s2_uncached_resp_addr
  }

  //printf("[justprintinsts] uncached %b tl.fire %b replay_next %b io.cpu.resp.valid %b s2_valid_hit %b miss_cached %b uncached_pending %b release_state %d =s_ready %b cached_grant_wait %b s1_nack %b \n", grantIsUncachedData, tl_out.d.fire(), io.cpu.replay_next,io.cpu.resp.valid, s2_valid_hit, s2_valid_cached_miss, s2_valid_uncached_pending,release_state, (release_state === s_ready),cached_grant_wait,s1_nack);
  // load data subword mux/sign extension
 //comment_final_dcache  val s2_data_word = ((0 until rowBits by wordBits).map(i => s2_data_uncorrected(wordBits+i-1,i)): Seq[UInt])(s2_word_idx)
 //comment_final_dcache  val s2_data_word_corrected = ((0 until rowBits by wordBits).map(i => s2_data_corrected(wordBits+i-1,i)): Seq[UInt])(s2_word_idx)
//comment_final_dcache  val loadgen = new LoadGen(s2_req.typ, mtSigned(s2_req.typ), s2_req.addr, s2_data_word,UInt(0), s2_sc, wordBytes)//change UInt(0) to the modified 32B data
//  io.cpu.resp.bits.data := loadgen.data | s2_sc_fail
  //printf("[removecacheincache] %x %x\n", io.cpu.resp.bits.data, io.cpu.resp.bits.DcacheCpu_data(xLen-1, 0))
  //io.cpu.resp.bits.data_word_bypass := loadgen.wordData
  //io.cpu.resp.bits.data_raw := s2_data_word
  //io.cpu.resp.bits.store_data := pstore1_data

  //zazad begins
  val modified2_s2_data_word = ((0 until rowBits by wordBits).map(i => modified2_s2_data_uncorrected(wordBits+i-1,i)): Seq[UInt])(s2_word_idx)
  val modified2_s2_data_word_corrected = ((0 until rowBits by wordBits).map(i => modified2_s2_data_corrected(wordBits+i-1,i)): Seq[UInt])(s2_word_idx)
  val modified2_loadgen = new LoadGen(s2_req.typ, mtSigned(s2_req.typ), s2_req.addr, modified2_s2_data_word,UInt(0), s2_sc, wordBytes)//change UInt(0) to the modified 32B data
  io.cpu.resp.bits.DcacheCpu_data := Mux(s2_req.isvector, modified2_s2_data_vector, Cat(UInt(1<<64)(63,1),UInt (0),UInt(1<<64)(63,1),UInt (0),UInt(1<<64)(63,1),UInt (0),modified2_loadgen.data | s2_sc_fail))
  io.cpu.resp.bits.DcacheCpu_data_word_bypass := Mux(s2_req.isvector, modified2_s2_data_vector, Cat(UInt(1<<64)(63,1),UInt (0),UInt(1<<64)(63,1),UInt (0),UInt(1<<64)(63,1),UInt (0),modified2_loadgen.wordData))
  io.cpu.resp.bits.DcacheCpu_data_raw := modified2_s2_data_word
  io.cpu.resp.bits.DcacheCpu_vector_data := modified2_s2_data_vector

   ///printf("[checkvectorlength]incachetocheckbypass %x v %b  %x %x  [%x %x %x]\n", io.cpu.resp.bits.DcacheCpu_data, s2_req.isvector,modified2_s2_data_vector, Cat(UInt(1<<64)(63,1),UInt (0),UInt(1<<64)(63,1),UInt (0),UInt(1<<64)(63,1),UInt (0),modified2_loadgen.data | s2_sc_fail), io.cpu.resp.bits.DcacheCpu_data_word_bypass,io.cpu.resp.bits.DcacheCpu_data_raw,io.cpu.resp.bits.DcacheCpu_vector_data)
  ///printf("[CacheChecksmainandmodified2] in dcache io.cpu.resp.data main %x modified %x\n", io.cpu.resp.bits.data(xLen-1, 0),io.cpu.resp.bits.DcacheCpu_data(xLen-1, 0) )
  ///printf("[checkbypassdat] in dcache io.cpu.resp.data main %x modified %x\n", io.cpu.resp.bits.data(xLen-1, 0),io.cpu.resp.bits.DcacheCpu_data(xLen-1, 0))
 /// printf("[checkbypassdat] in dcache io.cpu.resp.data_wrod main %x modified %x\n", io.cpu.resp.bits.data_word_bypass(xLen-1, 0),io.cpu.resp.bits.DcacheCpu_data_word_bypass(xLen-1, 0) 

  // AMOs
//  val modified_amoalu_loadgen = new LoadGen(s2_req.typ, mtSigned(s2_req.typ), s2_req.addr, modified2_s2_data_word,UInt(0), s2_sc, wordBytes)
 // val modified_amoalu_corrected_loadgen = new LoadGen(s2_req.typ, mtSigned(s2_req.typ), s2_req.addr,modified2_s2_data_word_corrected,UInt(0), s2_sc, wordBytes)

  if (usingRMW) {
    // when xLen < coreDataBits (e.g. RV32D), this AMOALU is wider than necessary
    val amoalu = Module(new AMOALU(coreDataBits))
    amoalu.io.mask := pstore1_mask
    amoalu.io.cmd := (if (usingAtomicsInCache) pstore1_cmd else M_XWR)
    //needstobereplaced
    amoalu.io.lhs := modified2_s2_data_word//modified2_loadgen.data //right:s2_data_word
    //comment_final_dcache printf("[removecacheinamoalu] %x %x %x\n", s2_data_word, modified2_loadgen.data, modified2_s2_data_word)
    amoalu.io.rhs := pstore1_data
    vector_pstore1_storegen_data := Cat(UInt(1<<64)(63,1),UInt(0),UInt(1<<64)(63,1),UInt(0),UInt(1<<64)(63,1),UInt(0),amoalu.io.out)
    pstore1_storegen_data := amoalu.io.out
  } else if (!usingAtomics) {
    assert(!(s1_valid_masked && s1_read && s1_write), "unsupported D$ operation")
  }

  when (s2_correct) { pstore1_storegen_data := modified2_s2_data_word_corrected //s2_data_word_corrected
                      vector_pstore1_storegen_data := Cat(UInt(1<<64)(63,1),UInt(0),UInt(1<<64)(63,1),UInt(0),UInt(1<<64)(63,1),UInt(0),modified2_s2_data_word_corrected)
                     //comment_final_dcache  printf("[removecacheinamoalucorrec] %x %x %x \n", s2_data_word_corrected, modified2_loadgen.data, modified2_s2_data_word_corrected)
                    }

  // flushes
  val resetting = RegInit(false.B)
  if (!usingDataScratchpad)
    when (RegNext(reset)) { resetting := true }
  val flushed = Reg(init=Bool(true))
  val flushing = Reg(init=Bool(false))
  val flushCounter = Reg(init=UInt(nSets * (nWays-1), log2Ceil(nSets * nWays)))
  val flushCounterNext = flushCounter +& 1
  val flushDone = (flushCounterNext >> log2Ceil(nSets)) === nWays
  val flushCounterWrap = flushCounterNext(log2Ceil(nSets)-1, 0)
  when (s2_valid_masked && s2_req.cmd === M_FLUSH_ALL) {
    io.cpu.s2_nack := !flushed
    when (!flushed) {
      flushing := !release_ack_wait && !uncachedInFlight.asUInt.orR
    }
  }
  ccover(s2_valid_masked && s2_req.cmd === M_FLUSH_ALL && s2_meta_error, "TAG_ECC_ERROR_DURING_FENCE_I", "D$ ECC error in tag array during cache flush")
  ccover(s2_valid_masked && s2_req.cmd === M_FLUSH_ALL && modified2_s2_data_error, "DATA_ECC_ERROR_DURING_FENCE_I", "D$ ECC error in data array during cache flush")
  s1_flush_valid := metaArb.io.in(5).fire() && !s1_flush_valid && !s2_flush_valid_pre_tag_ecc && release_state === s_ready && !release_ack_wait
  metaArb.io.in(5).valid := flushing
  metaArb.io.in(5).bits.write := false
  metaArb.io.in(5).bits.addr := Cat(io.cpu.req.bits.addr >> untagBits, flushCounter(idxBits-1, 0) << blockOffBits)
  metaArb.io.in(5).bits.way_en := ~UInt(0, nWays)
  metaArb.io.in(5).bits.data := metaArb.io.in(4).bits.data

  // Only flush D$ on FENCE.I if some cached executable regions are untracked.
  val supports_flush = !edge.manager.managers.forall(m => !m.supportsAcquireT || !m.executable || m.regionType >= RegionType.TRACKED || m.regionType <= RegionType.UNCACHEABLE)
  if (supports_flush) {
    when (tl_out_a.fire() && !s2_uncached) { flushed := false }
    when (flushing) {
      s1_victim_way := flushCounter >> log2Up(nSets)
      when (s2_flush_valid) {
        flushCounter := flushCounterNext
        when (flushDone) {
          flushed := true
          if (!isPow2(nWays)) flushCounter := flushCounterWrap
        }
      }
      when (flushed && release_state === s_ready && !release_ack_wait) {
        flushing := false
      }
    }
  }
  metaArb.io.in(0).valid := resetting
  metaArb.io.in(0).bits.addr := metaArb.io.in(5).bits.addr
  metaArb.io.in(0).bits.write := true
  metaArb.io.in(0).bits.way_en := ~UInt(0, nWays)
  metaArb.io.in(0).bits.data.coh := ClientMetadata.onReset
  metaArb.io.in(0).bits.data.tag := s2_req.addr >> untagBits
  when (resetting) {
    flushCounter := flushCounterNext
    when (flushDone) {
      resetting := false
      if (!isPow2(nWays)) flushCounter := flushCounterWrap
    }
  }

  // performance events
  io.cpu.perf.acquire := edge.done(tl_out_a)
  io.cpu.perf.release := edge.done(tl_out_c)
  io.cpu.perf.tlbMiss := io.ptw.req.fire()

  // report errors
  val (data_error, data_error_uncorrectable, data_error_addr) =
    if (usingDataScratchpad) (modified2_s2_valid_data_error, modified2_s2_data_error_uncorrectable, s2_req.addr) else {
      (tl_out_c.fire() && inWriteback && modified2_writeback_data_error,
        modified2_writeback_data_uncorrectable,
        tl_out_c.bits.address)
    }
  {
    val error_addr =
      Mux(metaArb.io.in(1).valid, Cat(metaArb.io.in(1).bits.data.tag, metaArb.io.in(1).bits.addr(untagBits-1, idxLSB)),
          data_error_addr >> idxLSB) << idxLSB
    io.errors.uncorrectable.foreach { u =>
      u.valid := metaArb.io.in(1).valid && s2_meta_error_uncorrectable || data_error && data_error_uncorrectable
      u.bits := error_addr
    }
    io.errors.correctable.foreach { c =>
      c.valid := metaArb.io.in(1).valid || data_error
      c.bits := error_addr
      io.errors.uncorrectable.foreach { u => when (u.valid) { c.valid := false } }
    }
    io.errors.bus.valid := tl_out.d.fire() && tl_out.d.bits.error
    io.errors.bus.bits := Mux(grantIsCached, s2_req.addr >> idxLSB << idxLSB, 0.U)

    ccoverNotScratchpad(io.errors.bus.valid && grantIsCached, "D_ERROR_CACHED", "D$ D-channel error, cached")
    ccover(io.errors.bus.valid && !grantIsCached, "D_ERROR_UNCACHED", "D$ D-channel error, uncached")
  }

  def encodeData(x: UInt) = x.grouped(eccBits).map(dECC.encode(_)).asUInt
  def dummyEncodeData(x: UInt) = x.grouped(eccBits).map(dECC.swizzle(_)).asUInt
  def decodeData(x: UInt) = x.grouped(dECC.width(eccBits)).map(dECC.decode(_))
  def eccMask(byteMask: UInt) = byteMask.grouped(eccBytes).map(_.orR).asUInt
  def eccByteMask(byteMask: UInt) = FillInterleaved(eccBytes, eccMask(byteMask))

  def needsRead(req: HellaCacheReq) =
    isRead(req.cmd) ||
    (isWrite(req.cmd) && (req.cmd === M_PWR || mtSize(req.typ) < log2Ceil(eccBytes)))

  def ccover(cond: Bool, label: String, desc: String)(implicit sourceInfo: SourceInfo) =
    cover(cond, s"DCACHE_$label", "MemorySystem;;" + desc)
  def ccoverNotScratchpad(cond: Bool, label: String, desc: String)(implicit sourceInfo: SourceInfo) =
    if (!usingDataScratchpad) ccover(cond, label, desc)

  if (usingDataScratchpad) {
    val data_error_cover = Seq(
      CoverBoolean(!data_error, Seq("no_data_error")),
      CoverBoolean(data_error && !data_error_uncorrectable, Seq("data_correctable_error")),
      CoverBoolean(data_error && data_error_uncorrectable, Seq("data_uncorrectable_error")))
    val request_source = Seq(
      CoverBoolean(s2_isSlavePortAccess, Seq("from_TL")),
      CoverBoolean(!s2_isSlavePortAccess, Seq("from_CPU")))

    cover(new CrossProperty(
      Seq(data_error_cover, request_source),
      Seq(),
      "MemorySystem;;Scratchpad Memory Bit Flip Cross Covers"))
  } else {

    val data_error_type = Seq(
      CoverBoolean(!modified2_s2_valid_data_error, Seq("no_data_error")),
      CoverBoolean(modified2_s2_valid_data_error && !modified2_s2_data_error_uncorrectable, Seq("data_correctable_error")),
      CoverBoolean(modified2_s2_valid_data_error && modified2_s2_data_error_uncorrectable, Seq("data_uncorrectable_error")))
    val data_error_dirty = Seq(
      CoverBoolean(!s2_victim_dirty, Seq("data_clean")),
      CoverBoolean(s2_victim_dirty, Seq("data_dirty")))
    val request_source = if (supports_flush) {
        Seq(
          CoverBoolean(!flushing, Seq("access")),
          CoverBoolean(flushing, Seq("during_flush")))
      } else {
        Seq(CoverBoolean(true.B, Seq("never_flush")))
      }
    val tag_error_cover = Seq(
      CoverBoolean( !metaArb.io.in(1).valid, Seq("no_tag_error")),
      CoverBoolean( metaArb.io.in(1).valid && !s2_meta_error_uncorrectable, Seq("tag_correctable_error")),
      CoverBoolean( metaArb.io.in(1).valid && s2_meta_error_uncorrectable, Seq("tag_uncorrectable_error")))
    cover(new CrossProperty(
      Seq(data_error_type, data_error_dirty, request_source, tag_error_cover),
      Seq(),
      "MemorySystem;;Cache Memory Bit Flip Cross Covers"))
  }
}
