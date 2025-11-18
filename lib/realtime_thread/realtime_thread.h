#pragma once
#include "mbed.h"
#include "IO_handler.h"
#include "ThreadFlag.h"
#include "observer.h"
#include <Eigen/Dense>
#include "IIR_filter.h"
#include "serial_pipe.h"

class realtime_thread
{
public:
    realtime_thread(IO_handler *, float Ts);
    virtual     ~realtime_thread();
    void start_loop(void);
    IO_handler *m_io;

private:
    void loop(void);
    Timer ti;
    Thread thread;
    Ticker ticker;
    ThreadFlag threadFlag;
    void sendSignal();
    float Ts,u_out;
    SerialPipe sp;  // Serial pipe for UART communication
};
