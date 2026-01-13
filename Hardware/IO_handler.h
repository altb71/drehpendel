#pragma once
/* class IO_handler
Tasks for students:
    - scale ios correctly
    - define derivative filter correctly
*/
#include <cstdint>
#include "EncoderCounter.h"
#include "EncoderCounterIndex.h"
#include "IIR_filter.h"
#include "LinearCharacteristics.h"
#include "Enc_unwrap_scale.h"
#include "pendel_kinematics.h"
#include "FastPWM.h"


class IO_handler
{
public:
    IO_handler(pendel_kinematics *, float Ts);        // default constructor
    virtual ~IO_handler();   // deconstructor
    void read_encoders_calc_speed(void);       // read both encoders and calculate speeds
    void force_enable_motors(bool);
    void enable_motors(bool);       // enable/disable motors via DigitalOut, send a "true" and also press button
    void write_voltage(float);  // write current to motors (0,...) for motor 1, (1,...) for motor 2
    float get_v_pendel(void);
    float get_phi_pendel(void);
    float get_v_motor(void);
    float get_phi_motor(void);
private:
    IIR_filter di1;
    IIR_filter di2;
    ///------------- Encoder -----------------------
    EncoderCounter counter1;    // initialize counter on PA_6 and PC_7
    // ------------------------------------
    EncoderCounter counter2;    // initialize counter on PB_6 and PB_7
    
    FastPWM mot_pwm;           // desired voltage values
    DigitalOut mot_dir;
    DigitalOut u_enable;
    //-------------------------------------
    LinearCharacteristics u2pwm;
    LinearCharacteristics u2i;
    Enc_unwrap_scale uw1;
    Enc_unwrap_scale uw2;
    pendel_kinematics *m_kin;
    float v_pendel,v_motor,phi_pendel,phi_motor;
};