#ifndef INCLUDED_CONSTELLATION_CYJ_H
#define INCLUDED_CONSTELLATION_CYJ_H

#include <gr_message.h>
#include <gr_math.h>
#include <gr_complex.h>
#include <math.h>
#include <iostream>
#include <stdlib.h>

//bpsk
const gr_complex BpskArray[] = 
{gr_complex(1,0),
gr_complex(-1,0)};
//qpsk
const gr_complex QpskArray[] =
{gr_complex(0.707,0.707),
gr_complex(-0.707,0.707),
gr_complex(0.707,-0.707),
gr_complex(-0.707,-0.707)};
//qam16
const gr_complex Qam16Array[] =
{gr_complex(0.333851,0.333851),
gr_complex(0.333851,1.00155),
gr_complex(1.00155,0.333851),
gr_complex(1.00155,1.00155),
gr_complex(-0.333851,0.333851),
gr_complex(-1.00155,0.333851),
gr_complex(-0.333851,1.00155),
gr_complex(-1.00155,1.00155),
gr_complex(-0.333851,-0.333851),
gr_complex(-0.333851,-1.00155),
gr_complex(-1.00155,-0.333851),
gr_complex(-1.00155,-1.00155),
gr_complex(0.333851,-0.333851),
gr_complex(1.00155,-0.333851),
gr_complex(0.333851,-1.00155),
gr_complex(1.00155,-1.00155)};
//qam64
const gr_complex Qam64Array[] =
{gr_complex(0.164288,0.164288),
gr_complex(0.164288,0.492863),
gr_complex(0.164288,0.821438),
gr_complex(0.164288,1.15001),
gr_complex(0.492863,0.164288),
gr_complex(0.492863,0.492863),
gr_complex(0.492863,0.821438),
gr_complex(0.492863,1.15001),
gr_complex(0.821438,0.164288),
gr_complex(0.821438,0.492863),
gr_complex(0.821438,0.821438),
gr_complex(0.821438,1.15001),
gr_complex(1.15001,0.164288),
gr_complex(1.15001,0.492863),
gr_complex(1.15001,0.821438),
gr_complex(1.15001,1.15001),
gr_complex(-0.164288,0.164288),
gr_complex(-0.492863,0.164288),
gr_complex(-0.821438,0.164288),
gr_complex(-1.15001,0.164288),
gr_complex(-0.164288,0.492863),
gr_complex(-0.492863,0.492863),
gr_complex(-0.821438,0.492863),
gr_complex(-1.15001,0.492863),
gr_complex(-0.164288,0.821438),
gr_complex(-0.492863,0.821438),
gr_complex(-0.821438,0.821438),
gr_complex(-1.15001,0.821438),
gr_complex(-0.164288,1.15001),
gr_complex(-0.492863,1.15001),
gr_complex(-0.821438,1.15001),
gr_complex(-1.15001,1.15001),
gr_complex(-0.164288,-0.164288),
gr_complex(-0.164288,-0.492863),
gr_complex(-0.164288,-0.821438),
gr_complex(-0.164288,-1.15001),
gr_complex(-0.492863,-0.164288),
gr_complex(-0.492863,-0.492863),
gr_complex(-0.492863,-0.821438),
gr_complex(-0.492863,-1.15001),
gr_complex(-0.821438,-0.164288),
gr_complex(-0.821438,-0.492863),
gr_complex(-0.821438,-0.821438),
gr_complex(-0.821438,-1.15001),
gr_complex(-1.15001,-0.164288),
gr_complex(-1.15001,-0.492863),
gr_complex(-1.15001,-0.821438),
gr_complex(-1.15001,-1.15001),
gr_complex(0.164288,-0.164288),
gr_complex(0.492863,-0.164288),
gr_complex(0.821438,-0.164288),
gr_complex(1.15001,-0.164288),
gr_complex(0.164288,-0.492863),
gr_complex(0.492863,-0.492863),
gr_complex(0.821438,-0.492863),
gr_complex(1.15001,-0.492863),
gr_complex(0.164288,-0.821438),
gr_complex(0.492863,-0.821438),
gr_complex(0.821438,-0.821438),
gr_complex(1.15001,-0.821438),
gr_complex(0.164288,-1.15001),
gr_complex(0.492863,-1.15001),
gr_complex(0.821438,-1.15001),
gr_complex(1.15001,-1.15001)};
//Map
const std::vector<gr_complex> BpskMap(BpskArray,BpskArray + 2);
const std::vector<gr_complex> QpskMap(QpskArray,QpskArray + 4);
const std::vector<gr_complex> Qam16Map(Qam16Array,Qam16Array + 16);
const std::vector<gr_complex> Qam64Map(Qam64Array,Qam64Array + 64);
//Modulation Scheme
enum Modulation {bpsk =1,qpsk=2,qam16=4,qam64=6};
//External Function
std::vector<gr_complex> constellation(Modulation m);
unsigned char Demapper(const gr_complex x, const std::vector<gr_complex> ConstMap);
#endif
