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
#include "data_structs.h"
#include "Enc_unwrap_scale.h"
#include "Mirror_Kinematic.h"


class IO_handler
{
public:
    IO_handler(Data_Xchange *,Mirror_Kinematic *, float Ts);        // default constructor
    virtual ~IO_handler();   // deconstructor
    void read_encoders_calc_speed(void);       // read both encoders and calculate speeds
    void set_des_current(uint8_t);  // set desired current on actuator
    void force_enable_motors(bool);
    void enable_motors(bool);       // enable/disable motors via DigitalOut, send a "true" and also press button
    void write_current(uint8_t,float);  // write current to motors (0,...) for motor 1, (1,...) for motor 2
    void set_laser_on_off(bool);    // set laser on or off
    bool motors_are_referenced();
private:
    IIR_filter di1;
    IIR_filter di2;
    DigitalIn big_button;         // Enable button an backside
    ///------------- Encoder -----------------------
    EncoderCounter counter1;    // initialize counter on PA_6 and PC_7
    InterruptIn indexpulse1;
    EncoderCounterIndex index1;   
    // ------------------------------------
    EncoderCounter counter2;    // initialize counter on PB_6 and PB_7
    InterruptIn indexpulse2;
    EncoderCounterIndex index2;    // 

    AnalogOut i_des1;           // desired current values
    AnalogOut i_des2;
    DigitalOut i_enable;
    DigitalOut laser_on;
    //-------------------------------------
    LinearCharacteristics i2u;
    LinearCharacteristics u2i;
    Enc_unwrap_scale uw1;
    Enc_unwrap_scale uw2;
    Data_Xchange *m_data;
    Mirror_Kinematic *m_mk;

};