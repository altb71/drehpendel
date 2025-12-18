#include "fast_realtime_thread.h"

#include <chrono>
#include <cmath>
#include <cstdint>

using MutexLock = rtos::ScopedMutexLock;

fast_realtime_thread::fast_realtime_thread(IO_handler &io, float Ts)
    : thread(osPriorityHigh2, 1024)
    , Ts(Ts)
    , io_handler(io)
{
    lowPass2.lowPass2Init(F_CUT_HZ, D, Ts);
}

fast_realtime_thread::~fast_realtime_thread() {}

float fast_realtime_thread::updateState(bool enable, float current_setpoint)
{
    MutexLock lock(m_mutex);
    m_is_enabled = enable;
    m_current_setpoint = current_setpoint;
    return m_current; // latest filtered current from fast loop
}

void fast_realtime_thread::loop(void)
{
    float kp_i = KP_I;
    float ki_i = KI_I;
    float u_i = 0.0f;

    while (true) {
        ThisThread::flags_wait_any(threadFlag);

        io_handler.set_enable_frtt_do(true);

        // Read filtered current measurement
        const float current = lowPass2.apply(io_handler.read_current());

        bool is_enabled;
        float current_setpoint;
        {
            MutexLock lock(m_mutex);
            is_enabled = m_is_enabled;
            current_setpoint = m_current_setpoint;
            m_current = current;
        }

        // Current controller
        if (is_enabled) {

            // Error
            const float current_error = current_setpoint - current;
            // I-Term and saturation
            u_i = clamp(u_i + ki_i * current_error * Ts, -POWERSUPPLY_VOLTAGE, POWERSUPPLY_VOLTAGE);
            // Control output and saturation
            float u = clamp(u_i + kp_i * current_error, -POWERSUPPLY_VOLTAGE, POWERSUPPLY_VOLTAGE);

            // Caluclate direction and PWM value
            if (u > 0.0f)
                io_handler.set_dir(0);
            else
                io_handler.set_dir(1);

            io_handler.write_pwm_motor(clamp((fabs(u) + OFFSET_VOLTAGE) / POWERSUPPLY_VOLTAGE, 0.0f, 1.0f));
        } else {
            io_handler.write_pwm_motor(0.0f);
            u_i = 0.0f;
        }

        io_handler.set_enable_frtt_do(false);
    }
}

void fast_realtime_thread::start_loop(void)
{
    thread.start(callback(this, &fast_realtime_thread::loop));
    ticker.attach(callback(this, &fast_realtime_thread::sendSignal), microseconds{static_cast<int64_t>(Ts * 1e6f)});
}

float fast_realtime_thread::clamp(float val, float min, float max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}
