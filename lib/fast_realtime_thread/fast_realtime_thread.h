#pragma once

#include <chrono>

#include "IIRFilter.h"
#include "IO_handler.h"
#include "ThreadFlag.h"
#include "mbed.h"
#include "rtos.h"

#define PERFORM_GPA_MEAS false

#if PERFORM_GPA_MEAS
#include "GPA.h"
#endif

#define POWERSUPPLY_VOLTAGE 24.0f // Voltage of the power supply in Volts
#define OFFSET_VOLTAGE 2.16f      // Offset voltage to overcome motor deadzone in Volts
#define KP_I 3.9811f              // Proportional gain current controller
#define KI_I 1.3772e+04f          // Integral gain current controller
#define F_CUT_HZ 250.0f           // Second order low-pass filter cutoff frequency in Hz
#define D 0.9f                    // Second order low-pass filter damping ratio

using namespace std::chrono;

class fast_realtime_thread
{
public:
    fast_realtime_thread(IO_handler &io, float Ts);
    virtual ~fast_realtime_thread();
    void start_loop(void);

    // Coherent update of enable + setpoint under one lock; returns latest values
    void updateState(bool enable, float current_setpoint);
    void updateStateAndReturnMeasurements(bool enable, float current_setpoint, float &current, float &motor_angle, float &pendulum_angle);

private:
    Thread thread;
    Ticker ticker;
    ThreadFlag threadFlag;
    float Ts;
    IO_handler &io_handler;
    IIRFilter lowPass2[2];

    rtos::Mutex m_mutex;
    bool m_is_enabled{false};
    float m_current_setpoint{0.0f};
    float m_current{0.0f};
    float m_motor_angle{0.0f};
    float m_pendulum_angle{0.0f};

#if PERFORM_GPA_MEAS
    GPA m_GPA;
#endif

    void loop(void);
    void sendSignal() { thread.flags_set(threadFlag); }
    float clamp(float val, float min, float max);
};
