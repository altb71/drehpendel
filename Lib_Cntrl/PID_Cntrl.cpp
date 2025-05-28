#include "PID_Cntrl.h"

// Matlab
// Tn = .005;
// Gpi= tf([Tn 1],[Tn 0]);
// Kp = 0.0158;
// pid(Kp*Gpi);

PID_Cntrl::PID_Cntrl(float P, float I, float D, float tau_f, float Ts, float uMin, float uMax)
{
    // ------------------
    this->P = P;
    this->I = I;
    this->D = D;
    this->tau_f = tau_f;
    this->Ts = Ts;
    this->uMin = uMin;
    this->uMax = uMax;
    reset(0);
}

PID_Cntrl::~PID_Cntrl() {}

void PID_Cntrl::reset(float initValue)
{
    // -----------------------
}


float PID_Cntrl::update(float e)
{
    // the main update function
    // AUFGABE 6.3, 6.4, 7.1
    return 0;   // saturate and return 
}

float PID_Cntrl::saturate(float x)
{
if(x > uMax)
    return uMax;
else if(x < uMin)
    return uMin;
return x;
}