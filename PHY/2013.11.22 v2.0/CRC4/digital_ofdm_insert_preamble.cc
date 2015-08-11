/* -*- c++ -*- */
/*
 * Copyright 2007,2010-2012 Free Software Foundation, Inc.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <digital_ofdm_insert_preamble.h>
#include <gr_io_signature.h>
#include <stdexcept>
#include <iostream>
#include <string.h>

#include <iostream>//cyjadd
#include <fstream>//cyjadd
#include <math.h>//cyjadd
#include <digital_crc32.h>//cyjadd
#include <gr_expj.h>//cyjadd
#include <constellation_cyj.h>//cyjadd
#include <crc_cyj.h>//cyjadd
#include <algorithm>//cyjadd

extern unsigned int  digital_update_crc32(unsigned int crc, const unsigned char *buf, size_t len);//cyjadd
extern unsigned char Demapper(const gr_complex x, const std::vector<gr_complex> ConstMap);//cyjadd
extern std::vector<gr_complex> constellation(Modulation m);//cyjadd

  /*For Signal*/
/////////////////////////////////////////////////////////////////////////////////
static Modulation Scheme = qam64;//cyjadd Modulation Scheme {bpsk =1,qpsk=2,qam16=4,qam64=6}
static std::vector<gr_complex> ConstMap;//cyjadd
#define TXDataByte 1000//cyjadd if change, also change SymbolLen
#define CRCByte 4//cyjadd
#define Data_tones 46
#define BitPerSymbol (46*Scheme)//cyjadd
static unsigned int  SymbolLen = ceil((double)(TXDataByte + CRCByte)*8/(double)BitPerSymbol);//cyjadd SymbolLen for each reveiver 's packet, data + crc
/////////////////////////////////////////////////////////////////////////////////
  /*For TX1 TX2*/
/////////////////////////////////////////////////////////////////////////////////
#define TX1SymbolStart 4//cyjadd the first symbol is for header
#define TX1SymbolEnd (TX1SymbolStart + SymbolLen -1)//cyjadd 
#define TX2SymbolStart (TX1SymbolEnd + 1)//cyjadd
#define TX2SymbolEnd (TX2SymbolStart + SymbolLen -1)//cyjadd 
/////////////////////////////////////////////////////////////////////////////////

static int SymbolCount = 0;//cyjadd
static long Packetcount = 0;//cyjadd

/*For TX1*/
static const unsigned int LEN_TX1_RAW_Bit = (TXDataByte + CRCByte)*8;//cyjadd
static const bool TX1Data_Bit[8] = {1,1,1,0,0,0,1,0};//cyjadd, For one byte 226
static unsigned char TX1Data_Byte[TXDataByte] = {0x00};//cyjadd
static bool TX1_RAW_Bit[LEN_TX1_RAW_Bit] = {0};//cyjadd, data + crc (175+4)*8 = 1432 bit
static std::vector<gr_complex> TX1_MAP_Bit;//cyjadd
static unsigned int TX1crc32 = 0;//cyjadd

/*For TX2*/
static const unsigned int LEN_TX2_RAW_Bit = (TXDataByte + CRCByte)*8;//cyjadd
static const bool TX2Data_Bit[8] = {0,0,0,1,1,1,0,1};//cyjadd, For one byte 29
static unsigned char TX2Data_Byte[TXDataByte] = {0x00};//cyjadd
static bool TX2_RAW_Bit[LEN_TX2_RAW_Bit] = {0};//cyjadd, data + crc (175+4)*8 = 1432 bit
static std::vector<gr_complex> TX2_MAP_Bit;//cyjadd
static unsigned int TX2crc32 = 0;//cyjadd

/*For Phase offset*/
static long NextAngle = 0;//cyjadd must be int, not unsigned

/*For Symbol CRC*/
static const unsigned char CRCn = 4;//n indicates how many symbols in each group
static const unsigned polynomial = 0x03;
static unsigned char TXCRCTag = 0x00;

/*For Channel estimation*/
#define KnownSeqStart 2
#define KnownSeqEnd 3
static const bool Known_seq1[46] = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,
                                    0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0};//bpsk
static const bool Known_seq2[46] = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,
                                    0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0};//bpsk 

digital_ofdm_insert_preamble_sptr
digital_make_ofdm_insert_preamble(int fft_length,
				  const std::vector<std::vector<gr_complex> > &preamble)
{
  return gnuradio::get_initial_sptr(new digital_ofdm_insert_preamble(fft_length,
								     preamble));
}

digital_ofdm_insert_preamble::digital_ofdm_insert_preamble
       (int fft_length,
	const std::vector<std::vector<gr_complex> > &preamble)
  : gr_block("ofdm_insert_preamble",
	     gr_make_io_signature2(1, 2,
				   sizeof(gr_complex)*fft_length,
				   sizeof(char)),
	     gr_make_io_signature2(1, 2,
				   sizeof(gr_complex)*fft_length,
				   sizeof(char))),
    d_fft_length(fft_length),
    d_preamble(preamble),
    d_state(ST_IDLE),
    d_nsymbols_output(0),
    d_pending_flag(0)
{
  // sanity check preamble symbols
  for(size_t i = 0; i < d_preamble.size(); i++) {
    if(d_preamble[i].size() != (size_t) d_fft_length)
      throw std::invalid_argument("digital_ofdm_insert_preamble: invalid length for preamble symbol");
  }


  /*Initialize bits for transmission, and crc */
///////////////////////////////////////////////////////////////////////////////////////////////////
  //Initialize Constellation Map
  ConstMap = constellation(Scheme);//cyjadd Constellation Map
  //Transfer TX1Data_Bit to TX1_RAW_Bit
  for(int i = 0; i < int(TXDataByte*8); i++)
  {
    TX1_RAW_Bit[i] = TX1Data_Bit[i%8];
  }
  //Construct TX1Data_Byte
  for(int i = 0; i < int(TXDataByte*8); i++)
  {
    static int j = 0;
    TX1Data_Byte[j] = TX1_RAW_Bit[i] << (7-i%8) | TX1Data_Byte[j]; // Big endian
    if(i%8 == 7)
      {
        //std::cout<<j<<" "<<(short(TX1Data_Byte[j])&0xff)<<" "<<char(TX1Data_Byte[j])<<std::endl;//for test
        j++;
      }
  }
  //Calculated Crc for TX1
  TX1crc32 = digital_update_crc32(0xffffffff, TX1Data_Byte, sizeof(TX1Data_Byte)/sizeof(TX1Data_Byte[0])) ^ 0xffffffff;
  std::cout<<"TX1crc32: "<<TX1crc32<<std::endl;//cyjadd 257295604
  //Copy TX1crc32 to TX1_RAW_Bit(last 32 bit)
  for(int i = 0; i < CRCByte*8; i++)
  {
    TX1_RAW_Bit[TXDataByte*8 + i] = bool(TX1crc32 >> (31-i) & 0x00000001);
  }
  //Modulation, Map TX1_RAW_Bit to TX1_MAP_Bit
  for(unsigned int i = 0; i < LEN_TX1_RAW_Bit; i++)
  {
    static unsigned char bits = 0x00;
    bits =  TX1_RAW_Bit[i] << ((Scheme-1) - i%Scheme) | bits;
    if(i == (LEN_TX1_RAW_Bit - 1))
      {
	TX1_MAP_Bit.push_back(ConstMap[bits]);//padding zero for last modualted bits
	break;
      }
    if(i%Scheme == (Scheme-1))
      {
	TX1_MAP_Bit.push_back(ConstMap[bits]); 
        bits = 0x00;//clear bits
      }
  }
  //std::cout<<"SymbolLen: "<<SymbolLen<<" TX1_MAP_Bit: "<<TX1_MAP_Bit.size()<<": ConstMap.size():"<<ConstMap.size()<<std::endl;//cyjadd for test

///////////////////////////////////////////////////////////////////////////////////////////////////
  //Transfer TX2Data_Bit to TX2_RAW_Bit
  for(int i = 0; i < int(TXDataByte*8); i++)
  {
    TX2_RAW_Bit[i] = TX2Data_Bit[i%8];
  }
  //Construct TX2Data_Byte
  for(int i = 0; i < int(TXDataByte*8); i++)
  {
    static int j = 0;
    TX2Data_Byte[j] = TX2_RAW_Bit[i] << (7-i%8) | TX2Data_Byte[j]; // Big endian
    if(i%8 == 7)
      {
        //std::cout<<j<<" "<<(short(TX2Data_Byte[j])&0xff)<<" "<<char(TX2Data_Byte[j])<<std::endl;//for test
        j++;
      }
  }
  //Calculated Crc for TX2
  TX2crc32 = digital_update_crc32(0xffffffff, TX2Data_Byte, sizeof(TX2Data_Byte)/sizeof(TX2Data_Byte[0])) ^ 0xffffffff;
  std::cout<<"TX2crc32: "<<TX2crc32<<std::endl;//cyjadd 1498786761
  //Copy TX2crc32 to TX2_RAW_Bit(last 32 bit)
  for(int i = 0; i < CRCByte*8; i++)
  {
    TX2_RAW_Bit[TXDataByte*8 + i] = bool(TX2crc32 >> (31-i) & 0x00000001);
  }
  //Modulation, Map TX2_RAW_Bit to TX2_MAP_Bit
  for(unsigned int i = 0; i < LEN_TX2_RAW_Bit; i++)
  {
    static unsigned char bits = 0x00;
    bits =  TX2_RAW_Bit[i] << ((Scheme-1) - i%Scheme) | bits;
    if(i == (LEN_TX2_RAW_Bit - 1))
      {
	TX2_MAP_Bit.push_back(ConstMap[bits]);//padding zero for last modualted bits
	break;
      }
    if(i%Scheme == (Scheme-1))
      {
	TX2_MAP_Bit.push_back(ConstMap[bits]); 
        bits = 0x00;//clear bits
      }
  }
  std::cout<<"SymbolLen: "<<SymbolLen<<" TX2_MAP_Bit.size: "<<TX2_MAP_Bit.size()<<" ConstMap.size:"<<ConstMap.size()<<std::endl;//cyjadd for test
///////////////////////////////////////////////////////////////////////////////////////////////////
/*
  unsigned char const  data[] = {0xCC,0xAA};
  std::size_t const    data_len = sizeof( data ) / sizeof( data[0] );
  boost::crc_optimal<3, 0x05, 0, 0, false, false>  crc_ccitt2;
  crc_ccitt2 = std::for_each( data, data + data_len, crc_ccitt2 );
  unsigned char checksum = crc_ccitt2.checksum();
  std::cout<<"crc_ccitt2: "<<checksum<<std::endl;
*/
  enter_idle();
}


digital_ofdm_insert_preamble::~digital_ofdm_insert_preamble()
{
}

void digital_ofdm_insert_preamble::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
  ninput_items_required[0] = noutput_items;
}

int
digital_ofdm_insert_preamble::general_work(int noutput_items,
					   gr_vector_int &ninput_items_v,
					   gr_vector_const_void_star &input_items,
					   gr_vector_void_star &output_items)
{
  int ninput_items = ninput_items_v.size()==2?std::min(ninput_items_v[0], ninput_items_v[1]):ninput_items_v[0];
  const gr_complex *in_sym = (const gr_complex *) input_items[0];//original
  //gr_complex *in_sym = ( gr_complex *) input_items[0];//cyjadd set in_sym to non-const, for phase offset encode
  const unsigned char *in_flag = 0;
  if (input_items.size() == 2)
    in_flag = (const unsigned char *) input_items[1];

  gr_complex *out_sym = (gr_complex *) output_items[0];
  gr_complex out_sym_buf[64] = {gr_complex(0,0)};//cyjadd buffer the out_sym
  //std::cout<<"input_items[0]:"<<input_items[0]<<in_sym[0]<<" "<<(( gr_complex *) input_items[0])[0]<<" input_items[1]:"<<input_items[1]<<std::endl;//cyjadd for test
  unsigned char *out_flag = 0;
  if (output_items.size() == 2)
    out_flag = (unsigned char *) output_items[1];


  int no = 0;	// number items output
  int ni = 0;	// number items read from input


#define write_out_flag() 			\
  do { if (out_flag) 				\
          out_flag[no] = d_pending_flag; 	\
       d_pending_flag = 0; 			\
  } while(0)

  std::fstream file("/home/qiangroup/src/TxSymbol.txt",std::ios::out|std::ios::app);//cyjadd

  //std::cout<<"noutput_items"<< noutput_items<<" ninput_items"<<ninput_items<<std::endl;//cyjadd
  while (no < noutput_items && ni < ninput_items){
    switch(d_state){
    case ST_IDLE:
      if (in_flag && in_flag[ni] & 0x1)	// this is first symbol of new payload
	enter_preamble();
      else
	ni++;			// eat one input symbol
      break;
      
    case ST_PREAMBLE:
      assert(!in_flag || in_flag[ni] & 0x1);
      if (d_nsymbols_output >= (int) d_preamble.size()){
	// we've output all the preamble
	enter_first_payload();
      }
      else {
	memcpy(&out_sym[no * d_fft_length],
	       &d_preamble[d_nsymbols_output][0],
	       d_fft_length*sizeof(gr_complex));

	write_out_flag();
	no++;
	d_nsymbols_output++;
      }
      break;
      
    case ST_FIRST_PAYLOAD:
      // copy first payload symbol from input to output
      memcpy(&out_sym[no * d_fft_length],
	     &in_sym[ni * d_fft_length],
	     d_fft_length * sizeof(gr_complex));

      write_out_flag();
      no++;
      ni++;
      SymbolCount++;//cyjadd
      enter_payload();
      break;
      
    case ST_PAYLOAD:
      if (in_flag && in_flag[ni] & 0x1){	// this is first symbol of a new payload
	enter_preamble();
	//std::cout<<"SymbolCount: "<<SymbolCount<<std::endl;//cyjadd For test
        Packetcount ++;//cyjadd
	//file<<"Packetcount: "<<Packetcount<<" SymbolCount: "<<SymbolCount<<std::endl;//cyjadd
	SymbolCount = 0;//cyjadd clear SymbolCount for each new packet
	NextAngle = 0;//cyjadd clear angle state for each new packet
	break;
      }

      // copy a symbol from input to output
      SymbolCount++;//cyjadd count self-add before output

  /*start sending to multiple clients from the 4th Symbol, leave the 1st symbol containing the header info., 2nd, 3rd symbol as known sequence*/
///////////////////////////////////////////////////////////////////////////////////////////////////
      //clear the out_sym_buf before entering
      memset(out_sym_buf,0,d_fft_length * sizeof(gr_complex));

      file<<"SymbolCount: "<<(SymbolCount)<<std::endl;//cyjadd For test
      for (int j = 0,TXIndex = 0;j < d_fft_length; j++)
      {
        
	if(j < 6 || j > 57 || j == 31 || j ==32)//2 DC and 12 zero padded
	   {out_sym_buf[j] = gr_complex(0,0);//cyjadd
	    //file<<"j:"<<j<<" out_sym_buf:"<<out_sym_buf[j]<<std::endl;//cyjadd for test 
	   continue;}
	if(j == 11 || j == 25 || j == 38 || j == 52)
	   {out_sym_buf[j] = gr_complex(1,0);//cyjadd
	    //file<<"j:"<<j<<" out_sym_buf:"<<out_sym_buf[j]<<std::endl;//cyjadd for test 
	   continue;}
        //send the known sequence 1 and 2 in the 2nd and 3rd symbol
	if(SymbolCount == KnownSeqStart)
	   {out_sym_buf[j] = gr_complex(Known_seq1[TXIndex]*2-1,0); TXIndex ++;continue;}
        if(SymbolCount == KnownSeqEnd)
	   {out_sym_buf[j] = gr_complex(Known_seq2[TXIndex]*2-1,0); TXIndex ++;continue;}

        //Copy TX_Map_Bit(data+crc) to output buffer
	if(SymbolCount >= TX1SymbolStart && SymbolCount <= TX1SymbolEnd)//For Tx1
	   {out_sym_buf[j] = TX1_MAP_Bit[(SymbolCount - TX1SymbolStart)*Data_tones + TXIndex];TXIndex ++;
            if(((SymbolCount - TX1SymbolStart)*Data_tones + TXIndex) >= TX1_MAP_Bit.size()) {TXIndex = 0;TXIndex ++;}//random padding for last symbol
           }
	if(SymbolCount >= TX2SymbolStart && SymbolCount <= TX2SymbolEnd)//For Tx2
	   {out_sym_buf[j] = TX2_MAP_Bit[(SymbolCount - TX2SymbolStart)*Data_tones + TXIndex];TXIndex ++;
            if(((SymbolCount - TX2SymbolStart)*Data_tones + TXIndex) >= TX2_MAP_Bit.size()) {TXIndex = 0;TXIndex ++;}//random padding for last symbol
           }

	//file<<"j:"<<j<<" out_sym_buf:"<<out_sym_buf[j]<<" TXIndex:"<<TXIndex<<std::endl;//cyjadd for test 
      }
      //file<<std::endl;//cyjadd for test 

  /*Compute the TXCRCTag at the beginning of each symbol group*/
///////////////////////////////////////////////////////////////////////////////////////////////////
         
      if((SymbolCount - TX1SymbolStart) % CRCn == 0 && SymbolCount >= TX1SymbolStart && SymbolCount <= TX1SymbolEnd)//For Tx1
        { 	
          //Clear the TXCRCTag first
          TXCRCTag = 0x00;    
          std::vector<bool> Group_Symbol_Bit;
          //make sure not exceed the size of raw bit array 
          if((unsigned int)((SymbolCount - TX1SymbolStart)*BitPerSymbol + BitPerSymbol*CRCn) <= LEN_TX1_RAW_Bit)
             {
                //Get the group symbol bit 
                for(unsigned int i = 0; i < (unsigned int)(BitPerSymbol*CRCn); i++)
             	   Group_Symbol_Bit.push_back(TX1_RAW_Bit[(SymbolCount - TX1SymbolStart)*BitPerSymbol + i]);
                //Transfer Group_Symbol_Bit to Group_Symbol_Byte
                const unsigned int LEN_Group_Symbol_Byte = ceil((double)(Group_Symbol_Bit.size()) / (double)(8));

                unsigned char* Group_Symbol_Byte = new unsigned char[LEN_Group_Symbol_Byte];
                memset(Group_Symbol_Byte, 0x00, LEN_Group_Symbol_Byte);
                unsigned  int GroupByteIndex = 0;//cyjadd, index for Group_Symbol_Byte, range is from 0 to LEN_Group_Symbol_Byte
  	        for(unsigned int i = 0; i < Group_Symbol_Bit.size(); i++)
                   {
                     Group_Symbol_Byte[GroupByteIndex] = Group_Symbol_Bit[i] << (7-i%8) | Group_Symbol_Byte[GroupByteIndex]; // Big endian
                     if(i%8 == 7)
                       {
                         //std::cout<<GroupByteIndex<<" "<<short(Group_Symbol_Byte[GroupByteIndex]&0xff)<<std::endl;
                         GroupByteIndex ++;
                       }
                   }
	        //Compute the TXCRCTag based on Group_Symbol_Byte
                boost::crc_optimal<CRCn, polynomial, 0, 0, false, false>  GroupCRC;
                GroupCRC = std::for_each( Group_Symbol_Byte, Group_Symbol_Byte + LEN_Group_Symbol_Byte, GroupCRC );
                TXCRCTag = GroupCRC.checksum();
                delete[] Group_Symbol_Byte;//delete Group_Symbol_Byte
                file<<"TXCRCTag: "<<short(TXCRCTag&0xff)<<" "<<LEN_Group_Symbol_Byte<<std::endl;//for test
             }
          
          
        }
///////////////////////////////////////////////////////////////////////////////////////////////////

  /* Encode Phase offset from 4th symbol*/
///////////////////////////////////////////////////////////////////////////////////////////////////

      if(SymbolCount >= TX1SymbolStart && SymbolCount <= TX1SymbolEnd)//cyjadd must set in_sym to non-const for adding phaseoffset to incoming symbol
	{  
           bool TXCRCBitTag = TXCRCTag >> (CRCn-1 - (SymbolCount - TX1SymbolStart) % CRCn) & 0x01;
           //file<<CRCBitTag<<std::endl;//for test
	   switch(TXCRCBitTag)
            {
	      case 0:
		     NextAngle = NextAngle - 90;
		     for(int i = 0; i<d_fft_length; i++)
			out_sym_buf[i] = out_sym_buf[i]*gr_expj(NextAngle*2*M_PI/360);
		     
		     break;
	      case 1:
                     NextAngle = NextAngle + 90;
		     for(int i = 0; i<d_fft_length; i++)
			out_sym_buf[i] = out_sym_buf[i]*gr_expj(NextAngle*2*M_PI/360);
		     break;		   
	    } 	
	}

///////////////////////////////////////////////////////////////////////////////////////////////////

  /* Output the encoded symbol from out_sym_buf*/
///////////////////////////////////////////////////////////////////////////////////////////////////
      memcpy(&out_sym[no * d_fft_length],
	     &out_sym_buf,
	     d_fft_length * sizeof(gr_complex));
     
///////////////////////////////////////////////////////////////////////////////////////////////////
      if (SymbolCount > TX2SymbolEnd)//cyjadd, avoid zero padding for remaining no replacing symbol
      memcpy(&out_sym[no * d_fft_length],
	     &in_sym[ni * d_fft_length],
	     d_fft_length * sizeof(gr_complex));//original
     
      write_out_flag();
      no++;
      ni++;
      
      break;

    default:
      std::cerr << "digital_ofdm_insert_preamble: (can't happen) invalid state, resetting\n";
      enter_idle();
    }
  }

  consume_each(ni);
  //std::cout<<"no"<<no<<" ni"<<ni<<std::endl;//cyjadd
  return no;
}

void
digital_ofdm_insert_preamble::enter_idle()
{
  d_state = ST_IDLE;
  d_nsymbols_output = 0;
  d_pending_flag = 0;
}

void
digital_ofdm_insert_preamble::enter_preamble()
{
  d_state = ST_PREAMBLE;
  d_nsymbols_output = 0;
  d_pending_flag = 1;
}

void
digital_ofdm_insert_preamble::enter_first_payload()
{
  d_state = ST_FIRST_PAYLOAD;
}

void
digital_ofdm_insert_preamble::enter_payload()
{
  d_state = ST_PAYLOAD;
}
