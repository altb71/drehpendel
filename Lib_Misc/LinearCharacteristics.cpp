#include "LinearCharacteristics.h"

using namespace std;

LinearCharacteristics::LinearCharacteristics(float gain,float offset){    // standard lin characteristics
    this->gain = gain;
    this->offset = offset;
    this->ulim = 999999.0;
    this->llim = -999999.0;
}

LinearCharacteristics::LinearCharacteristics(float xmin,float xmax, float ymin, float ymax){    // standard lin characteristics
    this->gain = (ymax - ymin)/(xmax - xmin);
    this->offset = xmax - ymax/this->gain;
    this->ulim = 999999.0;
    this->llim = -999999.0;
    
}
LinearCharacteristics::LinearCharacteristics(float xmin,float xmax, float ymin, float ymax,float ll, float ul){    // standard lin characteristics
    this->gain = (ymax - ymin)/(xmax - xmin);
    this->offset = xmax - ymax/this->gain;
    this->llim = ll;
    this->ulim = ul;
    
}

LinearCharacteristics::~LinearCharacteristics() {}


float LinearCharacteristics::evaluate(float x)
{   
float dum = this->gain*(x - this->offset);
if(dum > this->ulim)
    dum = this->ulim;
if(dum < this->llim)
    dum = this->llim;
return dum;
    }

void LinearCharacteristics::setup(float xmin,float xmax, float ymin, float ymax){    // standard lin characteristics
    this->gain = (ymax - ymin)/(xmax - xmin);
    this->offset = xmax - ymax/this->gain;
    this->ulim = 999999.0;
    this->llim = -999999.0;   
}
void LinearCharacteristics::setup(float xmin,float xmax, float ymin, float ymax,float ll, float ul){    // standard lin characteristics
    this->gain = (ymax - ymin)/(xmax - xmin);
    this->offset = xmax - ymax/this->gain;
    this->llim = ll;
    this->ulim = ul; 
}

