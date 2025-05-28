#pragma once
#include "mbed.h"
#include "EncoderCounter.h"
#include "EncoderCounterIndex.h"
#include "LinearCharacteristics.h"
#include "ThreadFlag.h"
#include "PID_Cntrl.h"
#include "pendel_kinematics.h"
#include "GPA.h"
#include "DataLogger.h"
#include "FastPWM.h"
#include "IO_handler.h"
#include "Dense.h"
using namespace std;
using namespace Eigen;


#define CNTRL_IDLE 0
#define GPA_IDENT_PLANT 10
#define CNTRL_POS 20
#define CNTRL_STOP 30


// This is the loop class, it is not a controller at first hand, it guarantees a cyclic call
class realtime_thread
{
public:
    realtime_thread(IO_handler *,pendel_kinematics *,float Ts);
    virtual ~realtime_thread();
    void start_loop(void);
    void init_controllers(void);
    void reset_pids(void);
    void switch_to_cntrl_stop(void);
    void switch_to_GPA_ident(void);
    void switch_to_cntrl_vel(void);
    void switch_to_cntrl_pos(void);

private:
    void loop(void);
    Thread thread;
    Ticker ticker;
    ThreadFlag threadFlag;
    Timer ti;
    float Ts;
    void sendSignal();
    bool is_initialized;
    void find_index(void);
    PID_Cntrl v_cntrl_0, v_cntrl_1;
    IIR_filter ableit_vorst;
    IO_handler *m_io;
    pendel_kinematics *m_kin;
    uint8_t controller_state;
    Matrix<float, 4, 1> x_state;
    Matrix<float, 1, 4> K4;
    
};
