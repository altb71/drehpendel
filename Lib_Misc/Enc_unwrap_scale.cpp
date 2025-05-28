/*  
    Enc_unwrap_scale Class, differentiate encoder counts for cuboid, applies LP filter and unwrapping
    
              b*(1 - z^-1)                      s
      G(z) = -------------  <-- tustin --  ----------- = G(s)
              1 - a*z^-1                      T*s + 1
*/

#include "Enc_unwrap_scale.h"
#define   pi 3.141592653589793
using namespace std;

Enc_unwrap_scale::Enc_unwrap_scale(uint16_t incPerRev, uint8_t bits)
{   
    this->incPerRev = incPerRev;    // incr encoder with 4000inc/rev
    this->max_delta = 1<<(bits-1);
    inc2rad = 2.0*pi/static_cast<double>(incPerRev); 
    num_overflows = 0;

}
Enc_unwrap_scale::~Enc_unwrap_scale() {}

float Enc_unwrap_scale::doStep(int32_t inc)
{
        int32_t d = (inc - incPast);
        incPast = inc;
        if(d < -max_delta)  // inc got larger than e.g. 32000 and went to -32000
            ++num_overflows;
        else if(d > max_delta)
            --num_overflows;
        return inc2rad * (float)((int32_t)inc+num_overflows*2*max_delta);
}