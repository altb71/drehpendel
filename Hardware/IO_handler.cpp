#include "IO_handler.h"

#define PI 3.1415927
// constructors
#define VOLTAGE 16.8


// Deconstructor
IO_handler::IO_handler(pendel_kinematics *kin, float Ts) : di1(2*Ts,Ts),di2(2*Ts,Ts),big_button(PC_3),counter1(PA_6, PC_7),
                            counter2(PB_6, PB_7),u_enable(PB_9), mot_pwm(PB_15),mot_dir(PB_14)
{
    this->m_kin = kin;
    u2pwm.setup(0,VOLTAGE,0.01f,.99f,.01f,.99f);
    mot_pwm.period_ms((int)(Ts*1000));
    mot_pwm.write(.01);
	
    uw1 = Enc_unwrap_scale(4096,16);
    uw2 = Enc_unwrap_scale(4096,16);
    counter1.reset();   // encoder reset
    counter2.reset();   // encoder reset
}
IO_handler::~IO_handler() {} 

void IO_handler::read_encoders_calc_speed(void)
{
    phi_motor = uw1(counter1);
    phi_pendel = uw2(counter2) - 3.1415927f;
    v_motor = di1(phi_motor);
    v_pendel = di2(phi_pendel);
}

void IO_handler::enable_motors(bool enable)
{
    u_enable = enable;    
}

void IO_handler::write_voltage(float u_des)
{
        mot_pwm.write(u2pwm(fabs(u_des)));  // write uses duty 0...1
        mot_dir = u_des>=0;
}

float IO_handler::get_phi_motor()
{
    return phi_motor;
}
float IO_handler::get_v_motor()
{
    return phi_motor;
}
float IO_handler::get_phi_pendel()
{
    return phi_motor;
}
float IO_handler::get_v_pendel()
{
    return phi_motor;
}