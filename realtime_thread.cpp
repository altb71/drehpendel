#include "realtime_thread.h"
#include <cstdint>

extern GPA myGPA;
extern DataLogger myDataLogger;

// contructor for controller loop
realtime_thread::realtime_thread(IO_handler *io, pendel_kinematics *kin, float Ts) : thread(osPriorityHigh,4096)
{
    this->Ts = Ts;
    this->m_io = io;            // link to hardware
    this->m_kin = kin;            // link to kinematics
    ti.reset();
    ti.start();
    controller_state = CNTRL_IDLE;  // the local state machine
    //v_cntrl_0 = PID_Cntrl(,,,,Ts,-0.8,0.8); // zunaechst nur PI-Regler, 1 
    }
// decontructor for controller loop
realtime_thread::~realtime_thread() {}
// ----------------------------------------------------------------------------
// this is the main loop called every Ts with high priority
void realtime_thread::loop(void){
    float u_des,i_des1,v_des,phi_des,v_des_vorst;
    K4 << -0.3162,5.9553,-0.3132,0.5182;
    while(1)
        {
        ThisThread::flags_wait_any(threadFlag);
        // THE LOOP ------------------------------------------------------------
        m_io->read_encoders_calc_speed();       // first read encoders and calculate speed
        x_state << m_io->get_phi_motor(),m_io->get_phi_pendel(),m_io->get_v_motor(),m_io->get_v_pendel();

        // -------------------------------------------------------------
        // at very beginning: move system slowly to find the zero pulse
        float ti_loc = ti.read();
        switch(controller_state)
            {
            case CNTRL_IDLE:
                u_des =  0;
                break;
            case CNTRL_POS:
                if(fabsf(m_io->get_phi_pendel())< 0.1)
                    {
                    m_io->enable_motors(true);      // enable motors
                    u_des = K4*x_state;
                    }
                else{
                    m_io->enable_motors(false);      // enable motors
                    u_des = 0;
                    }
                break;
            case CNTRL_STOP:
                m_io->enable_motors(false);      // enable motors
                u_des = 0;
                break;
            // ------------------------ do the control first
            default:
                break;
            }
        m_io->write_voltage(u_des);
            
        }// endof the main loop
}

void realtime_thread::sendSignal() {
    thread.flags_set(threadFlag);
}
void realtime_thread::start_loop(void)
{
    thread.start(callback(this, &realtime_thread::loop));
    ticker.attach(callback(this, &realtime_thread::sendSignal), Ts);
}
// several public functions to allow the controller statemachine to switch 
// to other states from external.
void realtime_thread::switch_to_cntrl_stop()
{
    controller_state = CNTRL_STOP;
}
void realtime_thread::switch_to_GPA_ident()
{
    controller_state = GPA_IDENT_PLANT;
}
void realtime_thread::switch_to_cntrl_pos()
{
    controller_state = CNTRL_POS;
}
void realtime_thread::init_controllers(void)
{
    // set values for your velocity and position controller here!
}
   
void realtime_thread::reset_pids(void)
{
    // reset all cntrls.
}