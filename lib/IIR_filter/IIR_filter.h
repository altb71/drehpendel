#pragma once
#include <stdint.h>

/* class IIR-filter
Constructors:
...(float tau, float Ts, float K)   // "        "          with gain K
methods:
reset(void)         // sets internal vars to zero
eval(float u)       // make one step iteration with input u
operator: (float)   // calls "eval" 
*/
class IIR_filter
{
public:
    IIR_filter(){};
    IIR_filter(float);
    IIR_filter(float,float);
    IIR_filter(float,float,float);
    virtual ~IIR_filter();
    float eval(float);
    float operator()(float u){
            return eval((float)u);
         }
private:
    float *B;
    float *A;
    float a0,b0,b1,yk,uk;
    uint8_t nb,na;
};