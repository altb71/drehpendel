#include "LinearCharacteristics.h"

using namespace std;

LinearCharacteristics::LinearCharacteristics(float gain,float offset){    // standard lin characteristics
    this->gain = gain;
    this->offset = offset;
    this->ulim = 999999.0;          // a large number
    this->llim = -999999.0;         // a large neg. number
}
LinearCharacteristics::LinearCharacteristics(float xmin,float xmax, float ymin, float ymax){    // standard lin characteristics
    this->gain = (ymax-ymin)/(xmax-xmin);
    this->offset = xmin - ymin/this->gain;
    this->ulim = 999999.0;          // a large number
    this->llim = -999999.0;         // a large neg. number
}


LinearCharacteristics::~LinearCharacteristics() {}


float LinearCharacteristics::evaluate(float x)
{   
    // calculate result as y(x) = gain * (x-offset)
    float ret_val = gain * (x - offset);
    if(ret_val > ulim)
        return ulim;
    else if(ret_val < llim)
        return llim;
    else
        return ret_val;
    // oder
    // (ret_val > ulim) ? return ulim : ((ret_val < llim) ? return llim : return ret_val); 
}

void LinearCharacteristics::set_limits(float ll, float ul)
{
    this->llim = ll;
    this->ulim = ul;
}