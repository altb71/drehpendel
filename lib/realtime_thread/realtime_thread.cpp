#include "realtime_thread.h"

#include <chrono>
#include <cstdint>
#include <cstring>

realtime_thread::realtime_thread(IO_handler &io, float Ts, float Ts_fast)
    : thread(osPriorityHigh1, 1024)
    , Ts(Ts)
    , io_handler(io)
    , sp(USBTX, USBRX, 460800, 8, 5)
    , m_SerialStream(PB_10, PC_5, 30, 2000000)
    , m_Chirp(F0_HZ, (1.0f / 2.0f) / Ts, T1_SEC, Ts)
    , fast_rt_thread(io, Ts_fast)
{
}

realtime_thread::~realtime_thread() {}

void realtime_thread::loop(void)
{
    float f_val[2];
    char buffer[5];
    bool is_enabled = false;
    bool do_reset_encoders = true;
    uint32_t watchdog_counter = (uint32_t)(WATCHDOG_TIMEOUT / Ts + 0.5f);
    m_Timer.start();
    m_time_previous_us = m_Timer.elapsed_time();

    while (true) {
        ThisThread::flags_wait_any(threadFlag);

        if (sp.readable()) {

            // Read values (set values and enable) from UART
            const int nread = sp.get(buffer, 5, true);
            if (nread <= 0) {
                // Error or no data: skip this tick and keep last state
                continue;
            } else if (nread != 5) {
                // Incomplete frame: skip this tick and keep last setpoint/enable
                continue;
            }

            // We have a valid fresh packet -> reset watchdog
            watchdog_counter = (uint32_t)(WATCHDOG_TIMEOUT / Ts + 0.5f);

            // From Matlab: 1 float value (4 bytes) + enable (1 byte) are sent
            float w = 0.0f;
            std::memcpy(&w, &buffer[0], sizeof(float)); // assumes little-endian IEEE754
            bool enable = (buffer[4] == 1);             // uint8 value -> bool

            // Update is_enabled and h-bridge enable pin if enable has changed
            if (enable != is_enabled) {
                is_enabled = enable;
                io_handler.set_enable_motor(enable);

                if (!enable) {
                    // Disabled: force fast loop off and clear setpoint
                    fast_rt_thread.update_state(false, 0.0f);
                    do_reset_encoders = true; // ensure reset encoders when re-enabled
                } else {
                    // Rising edge: enable with current setpoint
                    fast_rt_thread.update_state(true, w);
                }
            } else if (is_enabled) {
                // Enabled and no change in enable: update setpoint
                fast_rt_thread.update_state(true, w);
            }

            // If enabled, reset encoders once
            if (is_enabled && do_reset_encoders) {
                do_reset_encoders = false;
                io_handler.reset_encoders();
            }

            // Write encoder values back to Matlab
            f_val[0] = io_handler.read_encoder_motor();
            // f_val[1] = current_setpoint;
            // f_val[1] = current;
            f_val[1] = io_handler.read_encoder_pendulum();
            sp.put((char *)&f_val[0], 8, true); // write to UART

        } else {
            if (watchdog_counter > 0) {
                watchdog_counter--;
            } else {
                // Watchdog timeout: no communication for WATCHDOG_TIMEOUT seconds
                if (is_enabled) { // only do this once per timeout
                    is_enabled = false;
                    io_handler.set_enable_motor(false);
                    fast_rt_thread.update_state(false, 0.0f);
                    do_reset_encoders = true; // ensure reset encoders when re-enabled
                }
            }
        }

        // // Measure delta time since last cycle
        // const microseconds time_us = m_Timer.elapsed_time();
        // const float dtime_us = duration_cast<microseconds>(time_us - m_time_previous_us).count();
        // m_time_previous_us = time_us;
    }
}

void realtime_thread::sendSignal() { thread.flags_set(threadFlag); }

void realtime_thread::start_loop(void)
{
    fast_rt_thread.start_loop();

    thread.start(callback(this, &realtime_thread::loop));
    ticker.attach(callback(this, &realtime_thread::sendSignal),
                  microseconds{static_cast<int64_t>(Ts * 1e6f)});
}

float realtime_thread::clamp(float val, float min, float max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}
