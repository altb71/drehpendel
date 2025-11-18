#pragma once
#include "mbed.h"
#include "IO_handler.h"
#include "ThreadFlag.h"
#include "observer.h"
#include <Eigen/Dense>
#include "IIR_filter.h"
#include "serial_pipe.h"

#define POWERSUPPLY_VOLTAGE 24.0f // Voltage of the power supply in Volts
#define OFFSET_VOLTAGE 2.0f       // Offset voltage to overcome motor deadzone in Volts
#define WATCHDOG_TIMEOUT 0.3f     // Watchdog timeout in seconds
#define KP_I 2.5119f              // Proportional gain current controller
#define KI_I 8.6900e+03f          // Integral gain current controller

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
    float saturate(float val, float min, float max);
};
