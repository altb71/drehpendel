#pragma once
#include "mbed.h"
#include "IO_handler.h"
#include "realtime_thread.h"

#define INIT 1
#define STOP 2
#define CONTROL 3
#define STATE_GPA 4


// This is the loop class, it is not a controller at first hand, it guarantees a cyclic call
class state_machine
{
public:
    state_machine(IO_handler *,realtime_thread *,float Ts);
    virtual     ~state_machine();
    void start_loop(void);

private:
    void loop(void);
    uint8_t CS;             // the current state
    Thread thread;
    Ticker ticker;
    ThreadFlag threadFlag;
    Timer ti;
    float Ts;
    void sendSignal();
    IO_handler *m_io;
    realtime_thread *m_loop;
};
