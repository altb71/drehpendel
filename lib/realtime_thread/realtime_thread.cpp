#include "realtime_thread.h"

#include <chrono>
#include <cstdint>

realtime_thread::realtime_thread(IO_handler &io_handler, float Ts)
    : thread(osPriorityHigh1, 1024)
    , Ts(Ts)
    , io_handler(io_handler)
    , sp(USBTX, USBRX, 460800, 8, 5)
    , m_SerialStream(PB_10, PC_5, 30, 2000000)
    , m_Chirp(F0_HZ, (1.0f / 2.0f) / Ts, T1_SEC, Ts)
{
    lowPass2.lowPass2Init(F_CUT_HZ, D, Ts);
}

realtime_thread::~realtime_thread() {}

void realtime_thread::loop(void)
{
    float f_val[2];
    char buffer[5];
    bool is_enabled = false;
    bool do_reset_encoders = true;
    float kp_i = KP_I;
    float ki_i = KI_I;
    float current_setpoint = 0.0f;
    float u_i = 0.0f;
    uint32_t watchdog_counter = (uint32_t)(WATCHDOG_TIMEOUT / Ts + 0.5f);
    m_Timer.start();
    m_time_previous_us = m_Timer.elapsed_time();

    while (true) {
        ThisThread::flags_wait_any(threadFlag);

        // Read filtered current measurement
        const float current = lowPass2.apply(io_handler.read_current());

        if (sp.readable()) {

            // Reset watchdog whenever we get a fresh packet
            watchdog_counter = (uint32_t)(WATCHDOG_TIMEOUT / Ts + 0.5f);

            sp.get(buffer, 5, true);        // read values (set values and enable) from UART
            float w = *(float *)&buffer[0]; // from Matlab 1 float value (4 bytes) + enable (1 byte) are sent
            bool enable = (buffer[4] == 1); // uint8 value -> bool

            // Update is_enabled and h-bridge enable pin if enable has changed
            if (enable != is_enabled) {
                is_enabled = enable;
                io_handler.set_enable_motor(enable);

                // if disabled, reset setpoint, integrator and flag to reset encoders
                if (!enable) {
                    current_setpoint = 0.0f;
                    u_i = 0.0f;
                    do_reset_encoders = true; // ensure reset encoders when re-enabled
                }
            }

            // If enabled, reset encoders once and update setpoint
            if (is_enabled) {
                if (do_reset_encoders) {
                    do_reset_encoders = false;
                    io_handler.reset_encoders();
                }
                current_setpoint = w;
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
                    current_setpoint = 0.0f;
                    u_i = 0.0f;
                    do_reset_encoders = true; // ensure reset encoders when re-enabled
                }
            }
        }

        // // Measure delta time since last cycle
        // const microseconds time_us = m_Timer.elapsed_time();
        // const float dtime_us = duration_cast<microseconds>(time_us - m_time_previous_us).count();
        // m_time_previous_us = time_us;

        // Current controller
        if (is_enabled) {

            // Error
            const float current_error = current_setpoint - current;
            // I-Term and saturation
            u_i = saturate(u_i + ki_i * current_error * Ts, -POWERSUPPLY_VOLTAGE, POWERSUPPLY_VOLTAGE);
            // Control output and saturation
            float u = saturate(u_i + kp_i * current_error, -POWERSUPPLY_VOLTAGE, POWERSUPPLY_VOLTAGE);

            // // Directly set the voltage via Matlab
            // float u = current_setpoint;

            // // Chirp generator
            // float u = OFFSET_V;
            // if (m_Chirp.update())
            //     u = AMP_V * m_Chirp.getExc() + OFFSET_V;

            // // Transmit data frame (same layout as original)
            // m_SerialStream.write(dtime_us);
            // m_SerialStream.write(u);
            // m_SerialStream.write(current);
            // m_SerialStream.send();

            // Caluclate direction and PWM value
            if (u > 0.0f)
                io_handler.set_dir(0);
            else
                io_handler.set_dir(1);

            io_handler.write_pwm_motor(saturate((fabs(u) + OFFSET_VOLTAGE) / POWERSUPPLY_VOLTAGE, 0.0f, 1.0f));
        } else {
            io_handler.write_pwm_motor(0.0f);
            // io_handler.write_pwm_motor(saturate((fabs(OFFSET_V) + OFFSET_VOLTAGE) / POWERSUPPLY_VOLTAGE,
            // 0.0f, 1.0f));
            u_i = 0.0f;
        }
    }
}

void realtime_thread::sendSignal() { thread.flags_set(threadFlag); }

void realtime_thread::start_loop(void)
{
    thread.start(callback(this, &realtime_thread::loop));
    ticker.attach(callback(this, &realtime_thread::sendSignal), microseconds{static_cast<int64_t>(Ts * 1e6f)});
}

float realtime_thread::saturate(float val, float min, float max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}
