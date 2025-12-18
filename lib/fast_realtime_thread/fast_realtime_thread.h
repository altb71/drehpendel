#pragma once

#include <chrono>

#include "IIRFilter.h"
#include "IO_handler.h"
#include "ThreadFlag.h"
#include "mbed.h"
#include "rtos.h"

using namespace std::chrono;

// Current loop and motor constants (updated to match your tuned version)
#define POWERSUPPLY_VOLTAGE 24.0f // Voltage of the power supply in Volts
#define OFFSET_VOLTAGE 2.16f      // Offset voltage to overcome motor deadzone in Volts
#define KP_I 2.0000f              // Proportional gain current controller
#define KI_I 2.7646e+03f          // Integral gain current controller
#define F_CUT_HZ 180.0f           // Second order low-pass filter cutoff frequency in Hz
#define D 0.7f                    // Second order low-pass filter damping ratio

class fast_realtime_thread
{
public:
    fast_realtime_thread(IO_handler &io, float Ts);
    virtual ~fast_realtime_thread();
    void start_loop(void);

    // // Individual setters (still available if needed)
    // void set_enabled(bool enable);
    // void set_current_setpoint(float current_setpoint);

    // Coherent update of enable + setpoint under one lock
    void update_state(bool enable, float current_setpoint);

private:
    Thread thread;
    Ticker ticker;
    ThreadFlag threadFlag;
    float Ts;
    IO_handler &io_handler;
    IIRFilter lowPass2;

    rtos::Mutex m_mutex;
    bool m_is_enabled;
    float m_current_setpoint;

    void loop(void);
    void sendSignal();
    float clamp(float val, float min, float max);
};
