#pragma once

#include <chrono>

#include "IIRFilter.h"
#include "IO_handler.h"
#include "ThreadFlag.h"
#include "mbed.h"
#include "rtos.h"

// Current loop and motor constants (updated to match your tuned version)
#define POWERSUPPLY_VOLTAGE 24.0f // Voltage of the power supply in Volts
#define OFFSET_VOLTAGE 2.16f      // Offset voltage to overcome motor deadzone in Volts
#define KP_I 2.0000f              // Proportional gain current controller
// #define KI_I 2.7646e+03f          // Integral gain current controller
// #define F_CUT_HZ 180.0f           // Second order low-pass filter cutoff frequency in Hz
#define KI_I 6.9027e+03f          // Integral gain current controller
#define F_CUT_HZ 1000.0f          // Second order low-pass filter cutoff frequency in Hz
#define D 0.7f                    // Second order low-pass filter damping ratio

using namespace std::chrono;

class fast_realtime_thread
{
public:
    fast_realtime_thread(IO_handler &io, float Ts);
    virtual ~fast_realtime_thread();
    void start_loop(void);

    // Coherent update of enable + setpoint under one lock; returns latest filtered current
    float updateState(bool enable, float current_setpoint);

private:
    Thread thread;
    Ticker ticker;
    ThreadFlag threadFlag;
    float Ts;
    IO_handler &io_handler;
    IIRFilter lowPass2;

    rtos::Mutex m_mutex;
    bool m_is_enabled{false};
    float m_current_setpoint{0.0f};
    float m_current{0.0f};

    void loop(void);
    void sendSignal() { thread.flags_set(threadFlag); }
    float clamp(float val, float min, float max);
};
