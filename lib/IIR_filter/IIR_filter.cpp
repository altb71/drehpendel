#include "IIR_filter.h"


// constructors for Integration
IIR_filter::IIR_filter(float Ts)
{
    b0 = Ts/2;
    b1 =  Ts/2;
    a0 = -1;
    yk = 0;
    uk = 0;
}
// constructors for derivative with low-pass filtering
IIR_filter::IIR_filter(float tau,float Ts)
{
    a0 = -(1-Ts/tau);       // 2nd version with LP-filter
    b0 = -1/tau;            // see Zwischenpruefung!!!
    b1 =  1/tau;  
    yk = 0;
    uk = 0;
}
IIR_filter::IIR_filter(float tau,float Ts,float K)
{
    b0 = K*Ts/tau;
    b1 = 0;
    a0 = -(1-Ts/tau);
    yk = 0;
    uk = 0;
}
// Methods:
float IIR_filter::eval(float u)
{
/* */
    float y_new = -a0*yk + b1 * u + b0* uk;
    yk = y_new;
    uk = u;
    return y_new;       //
}


// Deconstructor
IIR_filter::~IIR_filter() {} 