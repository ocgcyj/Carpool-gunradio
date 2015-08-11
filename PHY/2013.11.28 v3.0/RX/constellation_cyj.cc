#include <constellation_cyj.h>

std::vector<gr_complex> constellation(Modulation m) 
{
   switch(m)
     {   
	case bpsk: return BpskMap;
	case qpsk: return QpskMap;
	case qam16: return Qam16Map;
	case qam64: return Qam64Map;
     }
}

unsigned char Demapper(const gr_complex x, const std::vector<gr_complex> ConstMap)
{
  unsigned int min_index = 0;
  float min_euclid_dist = norm(x - ConstMap[0]);
  float euclid_dist = 0;
  
  for (unsigned int j = 1; j < (unsigned int)ConstMap.size(); j++){
    euclid_dist = norm(x - ConstMap[j]);
    if (euclid_dist < min_euclid_dist){
      min_euclid_dist = euclid_dist;
      min_index = j;
    }
  }
  return (unsigned char)(min_index & 0xff);
}
