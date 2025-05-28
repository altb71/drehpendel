#include "realtime_thread.h"
#include <cstdint>
using namespace std;

extern GPA myGPA;
extern DataLogger myDataLogger;

// contructor for controller loop
realtime_thread::realtime_thread(Data_Xchange *data,IO_handler *io, Mirror_Kinematic *mk, float Ts) : thread(osPriorityHigh,4096)
{
    this->Ts = Ts;
    this->m_data = data;        // link to data
    this->m_io = io;            // link to hardware
    this->m_mk = mk;            // link to kinematics
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
    float i_des0,i_des1,v_des,phi_des,v_des_vorst;
    uint8_t k = 0;
    float kv = 0;
    float kp = .02;
    while(1)
        {
        ThisThread::flags_wait_any(threadFlag);
        // THE LOOP ------------------------------------------------------------
        m_io->read_encoders_calc_speed();       // first read encoders and calculate speed
        // -------------------------------------------------------------
        // at very beginning: move system slowly to find the zero pulse
        float ti_loc = ti.read();
        switch(controller_state)
            {
            case CNTRL_IDLE:
                i_des0 = i_des1 = 0;
                break;
            case FIND_INDEX:
                // Aufgabe 8.x
                i_des0 = 0;
                i_des1 = 0;
                m_io->enable_motors(true);      // enable motors, still read the bigButton to enable
                break;
            case GPA_IDENT_PLANT:
                m_io->enable_motors(true);      // enable motors, still read the bigButton to enable
                // AUFGABE 5.2, 5.3
                i_des0 = 0; 
                i_des1 = 0;
                break;
            case CNTRL_VEL:
                // AUFGABE 6.3, 6.4
                i_des0 = 0;
                i_des1 = 0;
                m_io->enable_motors(true);      // enable motors
                break;
            case CNTRL_POS:
                // AUFGABE 7.x
                m_io->enable_motors(true);      // enable motors
            // Winkelregler 
                break;
            // ------------------------ do the control first
            default:
                break;
            }
        m_io->write_current(0,i_des0);
        m_io->write_current(1,i_des1);       // set 2nd motor to 0A
        m_io->set_laser_on_off(m_data->laser_on);
        if(++k>=10)     // kinematic transformation from angles to xy values only every 10th time.
            {
            m_mk->P2X(m_data->sens_phi,m_data->est_xy);
            k = 0;
            }
            
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
void realtime_thread::switch_to_find_index()
{
    controller_state = FIND_INDEX;
}
void realtime_thread::switch_to_GPA_ident()
{
    controller_state = GPA_IDENT_PLANT;
}
void realtime_thread::switch_to_cntrl_vel()
{
    controller_state = CNTRL_VEL;
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