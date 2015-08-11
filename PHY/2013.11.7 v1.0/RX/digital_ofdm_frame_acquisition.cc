/* -*- c++ -*- */
/*
 * Copyright 2006-2008,2010,2011 Free Software Foundation, Inc.
 * 
 * This file is part of GNU Radio
 * 
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <digital_ofdm_frame_acquisition.h>
#include <gr_io_signature.h>
#include <gr_expj.h>
#include <gr_math.h>
#include <cstdio>

#include <stdexcept>//cyjadd
#include <cmath>//cyjadd
#include <iostream>//cyjadd
#include <fstream>//cyjadd
#include <gr_fxpt.h>//cyjadd
#include <digital_crc32.h>//cyjadd
#include <math.h>//cyjadd
#include <constellation_cyj.h>//cyjadd

extern unsigned int digital_update_crc32(unsigned int crc, const unsigned char *data, size_t len);//cyjadd
extern std::vector<gr_complex> constellation(Modulation m);//cyjadd
extern unsigned char Demapper(const gr_complex x, const std::vector<gr_complex> ConstMap);//cyjadd

  /*Signal message*/
/////////////////////////////////////////////////////////////////////////////////////////
static Modulation Scheme = bpsk;//cyjadd Modulation Scheme {bpsk =1,qpsk=2,qam16=4,qam64=6}
static std::vector<gr_complex> ConstMap;//cyjadd
#define RXDataByte 4000//cyjadd if change, also change SymbolLen and UpThreshold
#define CRCByte 4//cyjadd
#define Data_tones 46//cyjadd
#define BitPerSymbol (46*Scheme)//cyjadd
static unsigned int  SymbolLen = ceil((double)(RXDataByte + CRCByte)*8/(double)BitPerSymbol);//cyjadd SymbolLen for each reveiver 's packet, data + crc
  /*For RX*/
/////////////////////////////////////////////////////////////////////////////////////////
#define RX1SymbolStart 3//cyjadd For RX1
#define RX1SymbolEnd (RX1SymbolStart + SymbolLen -1)//cyjadd 
#define RX2SymbolStart (RX1SymbolEnd + 1)//cyjadd For RX2
#define RX2SymbolEnd (RX2SymbolStart + SymbolLen -1)//cyjadd //cyjadd
/////////////////////////////////////////////////////////////////////////////////////////
  /*For Phase offset*/
/////////////////////////////////////////////////////////////////////////////////////////
static float NextAngle = 0;//cyjadd
static const bool EncodeArray[] ={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};//cyjadd 140bit -s 800(141)  -s 350(63)
const int ArrayLen = sizeof(EncodeArray)/sizeof(EncodeArray[0]);//cyjadd
static bool DecodeArray[ArrayLen] = {0};//cyjadd
static int BitErrorCount = 0;//Bit error count for phase offset for each packet
/////////////////////////////////////////////////////////////////////////////////////////

#define VERBOSE 0
#define M_TWOPI (2*M_PI)
//#define MAX_NUM_SYMBOLS 1000//original
#define MAX_NUM_SYMBOLS 20000//cyjadd
#define UpThreshold (SymbolLen*2 + 1)//cyjadd depend on the packet size

static int SymbolCount = 0;//cyjadd
static int SymbolCount1 = 0;//cyjadd only for headercheck
static long int PacketCount = 0;//cyjadd 
static bool PreambleFlag = 0;//cyjadd
static bool HeaderOK = 0;//cyjadd
static const char HeaderBitLen = 32;//cyjadd 
  /*For RX*/
/////////////////////////////////////////////////////////////////////////////////////////
static const unsigned int LEN_RX_RAW_Bit = (RXDataByte + CRCByte)*8;//cyjadd

static unsigned char RXData_Byte[RXDataByte] = {0x00};//cyjadd
static std::vector<bool> RX_RAW_Bit_TEMP;//cyjadd
static bool RX_RAW_Bit[LEN_RX_RAW_Bit] ={0};//cyjadd
static std::vector<gr_complex> RX_MAP_Bit;//cyjadd

static unsigned int  RXcrc32 = 0;//cyjadd
static bool CRCResult =0;//cyjadd
static bool crcFlag = 1;// crcFlag is aimed to filter out the extral crc symbol, like 66 symbol in 65 case
/////////////////////////////////////////////////////////////////////////////////////////
  /*For Channel Estimation*/
/////////////////////////////////////////////////////////////////////////////////////////
//static std::vector<gr_complex> Known_data(52);
//static std::vector<gr_complex> New_hestimate(52);
#define KnownSeqStart 2//cyjadd
static const bool Known_seq[46] = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,
				   0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0};//cyjadd
/////////////////////////////////////////////////////////////////////////////////////////
digital_ofdm_frame_acquisition_sptr
digital_make_ofdm_frame_acquisition (unsigned int occupied_carriers,
				     unsigned int fft_length, 
				     unsigned int cplen,
				     const std::vector<gr_complex> &known_symbol,
				     unsigned int max_fft_shift_len)
{
  return gnuradio::get_initial_sptr(new digital_ofdm_frame_acquisition (occupied_carriers, fft_length, cplen,
									known_symbol, max_fft_shift_len));
}

digital_ofdm_frame_acquisition::digital_ofdm_frame_acquisition (unsigned occupied_carriers,
								unsigned int fft_length, 
								unsigned int cplen,
								const std::vector<gr_complex> &known_symbol,
								unsigned int max_fft_shift_len)
  : gr_block ("ofdm_frame_acquisition",
	      gr_make_io_signature2 (2, 2, sizeof(gr_complex)*fft_length, sizeof(char)*fft_length),
	      gr_make_io_signature2 (2, 2, sizeof(gr_complex)*occupied_carriers, sizeof(char))),
    d_occupied_carriers(occupied_carriers),
    d_fft_length(fft_length),
    d_cplen(cplen),
    d_freq_shift_len(max_fft_shift_len),
    d_known_symbol(known_symbol),
    d_coarse_freq(0),
    d_phase_count(0)
{ 
  //Initialize Constellation Map
  ConstMap = constellation(Scheme);//cyjadd Constellation Map
  PacketCount = 0;//cyjadd reset when object is set up
  std::cout<<"SymbolLen: "<<SymbolLen<<" RX_MAP_Bit: "<<RX_MAP_Bit.size()<<": ConstMap.size():"<<ConstMap.size()<<std::endl;//cyjadd for test

  d_symbol_phase_diff.resize(d_fft_length);
  d_known_phase_diff.resize(d_occupied_carriers);
  d_hestimate.resize(d_occupied_carriers);

  unsigned int i = 0, j = 0;

  std::fill(d_known_phase_diff.begin(), d_known_phase_diff.end(), 0);
  for(i = 0; i < d_known_symbol.size()-2; i+=2) {
    d_known_phase_diff[i] = norm(d_known_symbol[i] - d_known_symbol[i+2]);
  }
  
  d_phase_lut = new gr_complex[(2*d_freq_shift_len+1) * MAX_NUM_SYMBOLS];
  for(i = 0; i <= 2*d_freq_shift_len; i++) {
    for(j = 0; j < MAX_NUM_SYMBOLS; j++) {
      d_phase_lut[j + i*MAX_NUM_SYMBOLS] =  gr_expj(-M_TWOPI*d_cplen/d_fft_length*(i-d_freq_shift_len)*j);
    }
  }
}

digital_ofdm_frame_acquisition::~digital_ofdm_frame_acquisition(void)
{
  delete [] d_phase_lut;
}

void
digital_ofdm_frame_acquisition::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
  unsigned ninputs = ninput_items_required.size ();
  for (unsigned i = 0; i < ninputs; i++)
    ninput_items_required[i] = 1;
}

gr_complex
digital_ofdm_frame_acquisition::coarse_freq_comp(int freq_delta, int symbol_count)
{
  //  return gr_complex(cos(-M_TWOPI*freq_delta*d_cplen/d_fft_length*symbol_count),
  //	    sin(-M_TWOPI*freq_delta*d_cplen/d_fft_length*symbol_count));

  return gr_expj(-M_TWOPI*freq_delta*d_cplen/d_fft_length*symbol_count);

  //return d_phase_lut[MAX_NUM_SYMBOLS * (d_freq_shift_len + freq_delta) + symbol_count];
}

void
digital_ofdm_frame_acquisition::correlate(const gr_complex *symbol, int zeros_on_left)
{
  unsigned int i,j;
  
  std::fill(d_symbol_phase_diff.begin(), d_symbol_phase_diff.end(), 0);
  for(i = 0; i < d_fft_length-2; i++) {
    d_symbol_phase_diff[i] = norm(symbol[i] - symbol[i+2]);
  }

  // sweep through all possible/allowed frequency offsets and select the best
  int index = 0;
  float max = 0, sum=0;
  for(i =  zeros_on_left - d_freq_shift_len; i < zeros_on_left + d_freq_shift_len; i++) {
    sum = 0;
    for(j = 0; j < d_occupied_carriers; j++) {
      sum += (d_known_phase_diff[j] * d_symbol_phase_diff[i+j]);
    }
    if(sum > max) {
      max = sum;
      index = i;
    }
  }
  
  // set the coarse frequency offset relative to the edge of the occupied tones
  d_coarse_freq = index - zeros_on_left;
}

void
digital_ofdm_frame_acquisition::calculate_equalizer(const gr_complex *symbol, int zeros_on_left)
{
  unsigned int i=0;

  // Set first tap of equalizer
  d_hestimate[0] = d_known_symbol[0] / 
    (coarse_freq_comp(d_coarse_freq,1)*symbol[zeros_on_left+d_coarse_freq]);

  // set every even tap based on known symbol
  // linearly interpolate between set carriers to set zero-filled carriers
  // FIXME: is this the best way to set this?
  for(i = 2; i < d_occupied_carriers; i+=2) {
    d_hestimate[i] = d_known_symbol[i] / 
      (coarse_freq_comp(d_coarse_freq,1)*(symbol[i+zeros_on_left+d_coarse_freq]));
    d_hestimate[i-1] = (d_hestimate[i] + d_hestimate[i-2]) / gr_complex(2.0, 0.0);    
  }

  // with even number of carriers; last equalizer tap is wrong
  if(!(d_occupied_carriers & 1)) {
    d_hestimate[d_occupied_carriers-1] = d_hestimate[d_occupied_carriers-2];
  }

  if(VERBOSE) {
    fprintf(stderr, "Equalizer setting:\n");
    for(i = 0; i < d_occupied_carriers; i++) {
      gr_complex sym = coarse_freq_comp(d_coarse_freq,1)*symbol[i+zeros_on_left+d_coarse_freq];
      gr_complex output = sym * d_hestimate[i];
      fprintf(stderr, "sym: %+.4f + j%+.4f  ks: %+.4f + j%+.4f  eq: %+.4f + j%+.4f  ==>  %+.4f + j%+.4f\n", 
	      sym .real(), sym.imag(),
	      d_known_symbol[i].real(), d_known_symbol[i].imag(),
	      d_hestimate[i].real(), d_hestimate[i].imag(),
	      output.real(), output.imag());
    }
    fprintf(stderr, "\n");
  }
}

int
digital_ofdm_frame_acquisition::general_work(int noutput_items,
					     gr_vector_int &ninput_items,
					     gr_vector_const_void_star &input_items,
					     gr_vector_void_star &output_items)
{
  const gr_complex *symbol = (const gr_complex *)input_items[0];
  const char *signal_in = (const char *)input_items[1];

  gr_complex *out = (gr_complex *) output_items[0];
  char *signal_out = (char *) output_items[1];
  
  int unoccupied_carriers = d_fft_length - d_occupied_carriers;
  int zeros_on_left = (int)ceil(unoccupied_carriers/2.0);

  gr_complex phase[52] = {};//cyjadd
  gr_complex sum_phase = 0;//cyjadd
  float angle[52] = {};//cyjadd
  int PilotCount = 0;//cyjadd
  gr_complex Pilot = 0;//cyjadd
  gr_complex TempOut[52] = {};//cyjadd
  float DiffAngle = 0;//cyjadd
  float oi, oq;//cyjadd for 
  unsigned int Header;//cyjadd

  std::fstream file("/home/qiangroup/src/RxSymbol.txt",std::ios::out|std::ios::app);//cyjadd app表示多次写入文件
  if(signal_in[0]) {
    d_phase_count = 1;
    correlate(symbol, zeros_on_left);
    calculate_equalizer(symbol, zeros_on_left);
    signal_out[0] = 1;

    if(SymbolCount >= 350)//filter the peak(outlier) among two preambles, the threshold depend on the usrp tx byte
	PreambleFlag = 1;//cyjadd set the flag to 1, signal the preamble is coming
	
    else
	PreambleFlag = 0;
    if(PreambleFlag == 1)
     {
       SymbolCount1 =0;//cyjadd only for header check
     }
     
   
  }
  else {
    signal_out[0] = 0;
    PreambleFlag = 0;//cyjadd
    SymbolCount++;//cyjadd
    SymbolCount1++;//cyjadd
    //file<<" SymbolCount: "<<SymbolCount<<" SymbolCount1: "<<SymbolCount1<<std::endl; //For test
  } 

  /*Update d_hestimate*/ 
///////////////////////////////////////////////////////////////////////////////////////////////////// 
  if(SymbolCount == KnownSeqStart)
  {
     //New_hestimate.assign(New_hestimate.size() ,0);//set all elements zero
     for(unsigned int i = 0,RXIndex = 0; i < d_occupied_carriers; i++)
	{
          if(i==5 || i==19 || i==25 || i==26|| i==32 || i ==46)//cyjadd
	    {
	      d_hestimate[i] = gr_complex(1,0) / 
	      (coarse_freq_comp(d_coarse_freq,d_phase_count)*symbol[i+zeros_on_left+d_coarse_freq]);//Pilot subcarriers
              continue;
            }
	  d_hestimate[i] = gr_complex(Known_seq[RXIndex]*2 -1,0) / 
	  (coarse_freq_comp(d_coarse_freq,d_phase_count)*symbol[i+zeros_on_left+d_coarse_freq]);
	  RXIndex ++;
	  //file<<d_hestimate[i].real()<<" "<<d_hestimate[i].imag()<<std::endl;//For test
	} 
  }
///////////////////////////////////////////////////////////////////////////////////////////////////// 

  for(unsigned int i = 0; i < d_occupied_carriers; i++) {
    
    if (PreambleFlag == 0)
      {
	if(i==5 || i==19 || i==32 || i ==46)//cyjadd
	  {
		   PilotCount ++;//cyjadd

		   switch (i)
		    {
		      case 5: {Pilot = gr_complex(1,0);break;}
		      case 19: {Pilot = gr_complex(1,0);break;}
		      case 32: {Pilot = gr_complex(1,0);break;}
		      case 46: {Pilot = gr_complex(1,0);break;}
		    }
		   phase[i] = Pilot*coarse_freq_comp(d_coarse_freq,d_phase_count)*symbol[i+zeros_on_left+d_coarse_freq]*std::conj((gr_complex(1,0)/d_hestimate[i]));//cyjadd
		   angle[i] = gr_fast_atan2f(phase[i]);//cyjadd
		   sum_phase += phase[i];//cyjadd
		   //file<<angle[i]*360/(2*M_PI)<<std::endl;//cyjadd output the 4 pilot phase offset angle
				 
		  if (PilotCount==4)
		    {
		  	PilotCount =0;
			gr_int32 sumangle = gr_fxpt::float_to_fixed (gr_fast_atan2f(sum_phase));//cyjadd
			gr_fxpt::sincos (sumangle, &oq, &oi);//cyjadd
		    }	 
	  }
      }
	
    TempOut[i] = d_hestimate[i]*coarse_freq_comp(d_coarse_freq,d_phase_count)*symbol[i+zeros_on_left+d_coarse_freq];//cyjadd buffer the symbol
    //if(SymbolCount > RX2SymbolStart && SymbolCount <= RX2SymbolEnd)
       //TempOut[i] = New_hestimate[i] * symbol[i+zeros_on_left+d_coarse_freq];//channel
  }

  /*Reset RXcrc32,RX_MAP_Bit,RX_RAW_Bit,RX_RAW_Bit_TEMP,RXData_Byte array,CRCResult,crcFlag,DecodeArray,BitErrorCount*/
///////////////////////////////////////////////////////////////////////////////////////////////////// 
  if(SymbolCount == 2)
    {RXcrc32 = 0X0000;RX_MAP_Bit.clear();RX_RAW_Bit_TEMP.clear();memset(RX_RAW_Bit, 0x00, sizeof(RX_RAW_Bit));memset(RXData_Byte, 0x00, sizeof(RXData_Byte));CRCResult = 0;crcFlag = 1;memset(DecodeArray, 0, sizeof(DecodeArray));BitErrorCount = 0;/*clear the BitErrorCount when finish decoding*/
NextAngle = 0;/*cyjadd clear the angle state for new packet*/}
///////////////////////////////////////////////////////////////////////////////////////////////////// 

  /*Decoding for phase offset*/
///////////////////////////////////////////////////////////////////////////////////////////////////// 
/*
  if (PreambleFlag == 0)//cyjadd
     {
          
	  if (SymbolCount>=2 && SymbolCount<=ArrayLen+1)//cyjadd
	   {
		DiffAngle = gr_fast_atan2f(sum_phase)*360/(2*M_PI) - NextAngle;
		
		if(DiffAngle > 180)
		   DiffAngle -=360;
		if(DiffAngle < -180)
		   DiffAngle +=360;
		if(DiffAngle > 0 && DiffAngle <180)
		   DecodeArray[SymbolCount-2] = 1;
		if(DiffAngle < 0 && DiffAngle > -180)
		   DecodeArray[SymbolCount-2] = 0;
		if(DecodeArray[SymbolCount-2] - EncodeArray[SymbolCount-2] !=0)
		   BitErrorCount++;
		//file<<SymbolCount<<" ";//for test
	        if(SymbolCount==ArrayLen+1)//finish decoding phase offset 
		  { 
		    //file<<BitErrorCount<<std::endl;//for test
		    for(int i = 0;i < ArrayLen;i++)//print out the DecodeArray Bit
		      file<<DecodeArray[i];
 		    file<<std::endl;
                  
		  }
	   }
	   NextAngle = gr_fast_atan2f(sum_phase)*360/(2*M_PI);//set the current Angle to Next Angle
     }
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////

  /*Compensate the symbol with phase offset using pilot*/
  /*Demodulation each symbol*/
///////////////////////////////////////////////////////////////////////////////////////////////////// 
  if (PreambleFlag == 0)
   {
	for(unsigned int i = 0; i < d_occupied_carriers; i++)
   	    {
		TempOut[i] = TempOut[i]/gr_complex (oi, oq);//cyjadd
		
		if(i==5 || i==19 || i==25 || i==26|| i==32 || i ==46)
		  {  
		    //Known_data[i] = gr_complex(1,0);//channel
		    continue;
		  }
		//For RX1, Copy Input buffer(data + CRC) to RX_MAP_Bit
		if(SymbolCount >= RX1SymbolStart && SymbolCount <= RX1SymbolEnd)
		  {
 		     unsigned char bits = Demapper(TempOut[i],ConstMap);
		     //Known_data[i] = ConstMap[bits];//channel
   		     for(int i = 0; i < Scheme; i++)
			{
			  //Demodulation, Map RX_MAP_Bit to RX_RAW_Bit_TEMP
			  RX_RAW_Bit_TEMP.push_back( bool(bits >> ((Scheme-1) -i) & 0x01) );
			}
                     
		  }
	    }
   }
///////////////////////////////////////////////////////////////////////////////////////////////////// 

  /*Output the compensated symbol*/
///////////////////////////////////////////////////////////////////////////////////////////////////// 
  memcpy(out, TempOut, sizeof(gr_complex)*d_occupied_carriers);//cyjadd 
///////////////////////////////////////////////////////////////////////////////////////////////////// 

  /*Reconstruct each packet and CRC check for each packet*/
///////////////////////////////////////////////////////////////////////////////////////////////////// 
  if(PreambleFlag == 0 && SymbolCount == RX1SymbolEnd && crcFlag == 1)//for RX1
   {
    //Copy the RX_RAW_Bit from RX_RAW_Bit_TEMP
    for(unsigned int i = 0; i < LEN_RX_RAW_Bit; i++)
	RX_RAW_Bit[i] = RX_RAW_Bit_TEMP[i];

    //Construct RXData_Byte from RX_RAW_Bit
    unsigned int RXDataByteIndex = 0;//cyjadd, index for RXData_Byte, range is from 0 to RXDataByte
    for(int k = 0; k < int(RXDataByte*8); k++)
     {
       RXData_Byte[RXDataByteIndex] = RX_RAW_Bit[k] << (7-k%8) | RXData_Byte[RXDataByteIndex];
      
       if(k%8 == 7)
         {
	   RXDataByteIndex++;
	 }
     }

    //Get the RXcrc32 from RX_RAW_Bit last 32 bit(4 bytes)
    for(int k = 0; k < int(CRCByte*8); k++)
     {
       RXcrc32 = RX_RAW_Bit[LEN_RX_RAW_Bit - CRCByte*8 + k] << (31 - k) | RXcrc32;//BPSK		  
     }
    
    for(int i = 0;i<RXDataByte;i++)
	file<<i<<" "<<short(RXData_Byte[i]&0xff)<<std::endl;//for test
    //file<<"RXcrc32:"<<RXcrc32<<std::endl;//For test

    //Calculated crc from RXData_Byte and compare it with RXcrc32
    CRCResult = (digital_update_crc32(0xffffffff, RXData_Byte, sizeof(RXData_Byte)/sizeof(RXData_Byte[0])) ^ 0xffffffff) == RXcrc32 ? 1:0;

    //Turn off the crcFlag to avoid double entering
    crcFlag = 0;
    //std::cout<<"RX_MAP_Bit: "<<RX_MAP_Bit.size()<<" ConstMap.size():"<<ConstMap.size()<<std::endl;//cyjadd for test
   }

///////////////////////////////////////////////////////////////////////////////////////////////////// 

  /*check Header if OK or Not*/
//////////////////////////////////////////////////////////////////////////////////////////////////// 
  if(SymbolCount1 == 1 )
   {
	int BitCount = 0;
	for(unsigned int i = 0; i < d_occupied_carriers; i++)
	   { 
		if(i==5 || i==19 || i==25 || i==26|| i==32 || i ==46)// remove the bit from 4 pilots and two DC
		   continue;
	        //only for BPSK
   	    	if (TempOut[i].real() > 0)
	   	   Header = (Header << 1) | (0x01);
	        if (TempOut[i].real() < 0)
		   Header = (Header << 1) | (0x00);
	        BitCount++;
	        if(BitCount == HeaderBitLen)//32bit
		   break;
	    }
        if(((Header >> 16) ^ (Header & 0xffff)) == 0)
	   { 
	     PacketCount++;	
	     //file<<"PacketCount:"<<PacketCount<<" SymbolCount:"<<SymbolCount-1<<" CRCResult:"<<CRCResult<<std::endl; 
	     file<<std::endl;    
	     SymbolCount =1;
	     
	   }
	else
	     {HeaderOK = 0;
	     SymbolCount++;}
   }
///////////////////////////////////////////////////////////////////////////////////////////////////// 

  d_phase_count++;
  if(d_phase_count == MAX_NUM_SYMBOLS) {
    d_phase_count = 1;
  }

  consume_each(1);
  return 1;
}
