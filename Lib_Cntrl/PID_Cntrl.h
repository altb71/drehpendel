#pragma once


class PID_Cntrl
{
public:

    PID_Cntrl(float P, float I, float D, float tau_f, float Ts, float uMin, float uMax);
    PID_Cntrl() {};

    float operator()(float e)
    {
        return update(e);
    }

    virtual ~PID_Cntrl();

    void    reset(float initValue);
    void    setCoefficients(float P, float I, float D, float tau_f, float Ts, float uMin, float uMax);
    float   update(float e);
    float   saturate(float);


private:
    float P,I,D,tau_f,Ts,uMax,uMin; // add other variables needed
    float Ipart,Dpart,e_old;
};
