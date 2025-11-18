#include <chrono>
#include <cstdint>
#include "realtime_thread.h"
#include "DataLogger.h"
#include "GPA.h"

extern DataLogger myDataLogger;
extern GPA myGPA;
using namespace Eigen;
using namespace std::chrono;
// contructor for realtime_thread loop
realtime_thread::realtime_thread(IO_handler *io,float Ts) : thread(osPriorityHigh1, 1024),sp(USBTX,USBRX,460800,8,5)
{
  this->Ts = Ts;        // the sampling time
  this->m_io = io;      // a pointer to the inputs/outputs
  ti.reset();
  ti.start();
}

// decontructor for controller loop
realtime_thread::~realtime_thread() {}

// ----------------------------------------------------------------------------
// this is the main loop called every Ts with high priority
void realtime_thread::loop(void)
{
  float tim,w,V,u,y2;
  float f_val[2];
  Matrix<float,1,2> K2;
  char buffer[5];
  bool is_enabled = false;
  bool do_reset_encoders = true;
  float kp_i = KP_I;
  float ki_i = KI_I;
  float current_setpoint = 0.0f;
  float u_i = 0.0f;
  uint32_t watchdog_counter = (uint32_t)(WATCHDOG_TIMEOUT / Ts + 0.5f);
  while (1)
    {
    ThisThread::flags_wait_any(threadFlag);
    tim = 1e-6*(duration_cast<microseconds>(ti.elapsed_time()).count());
// --------------------- THE LOOP -----------------------------------------
        if (sp.readable()) {

            // Reset watchdog whenever we get a fresh packet
            watchdog_counter = (uint32_t)(WATCHDOG_TIMEOUT / Ts + 0.5f);

            sp.get(buffer, 5, true);        // read values (set values and enable) from UART
            w = *(float *)&buffer[0];       // from Matlab 1 float value (4 bytes) + enable (1 byte) are sent
            bool enable = (buffer[4] == 1); // uint8 value -> bool

            // Update is_enabled and h-bridge enable pin if enable has changed
            if (enable != is_enabled) {
                is_enabled = enable;
                m_io->set_enable_motor(enable);

                // if disabled, reset setpoint, integrator and flag to reset encoders
                if (!enable) {
                    current_setpoint   = 0.0f;
                    u_i                = 0.0f;
                    do_reset_encoders  = true; // ensure reset encoders when re-enabled
                }
            }

            // If enabled, reset encoders once and update setpoint
            if (is_enabled) {
                if (do_reset_encoders) {
                    do_reset_encoders = false;
                    m_io->reset_encoders();
                }
                current_setpoint = w;
            }

            // Write encoder values back to Matlab
            f_val[0] = m_io->read_encoder_motor();
            f_val[1] = m_io->read_encoder_pendulum();
            sp.put((char *)&f_val[0], 8, true);   // write to UART

        } else {
            if (watchdog_counter > 0) {
                watchdog_counter--;
            } else {
                // Watchdog timeout: no communication for WATCHDOG_TIMEOUT seconds
                if (is_enabled) { // only do this once per timeout
                    is_enabled = false;
                    m_io->set_enable_motor(false);
                    current_setpoint  = 0.0f;
                    u_i               = 0.0f;
                    do_reset_encoders = true; // ensure reset encoders when re-enabled
                }
            }
        }

        // Current controller
        if (is_enabled) {

            // Error
            const float current_error = current_setpoint - m_io->read_current();
            // I-Term and saturation
            u_i = saturate(u_i + ki_i * current_error * Ts, -POWERSUPPLY_VOLTAGE, POWERSUPPLY_VOLTAGE);
            // Control output and saturation
            float u = saturate(u_i + kp_i * current_error, -POWERSUPPLY_VOLTAGE, POWERSUPPLY_VOLTAGE);

            // Caluclate direction and PWM value
            if (u > 0.0f) m_io->set_dir(0);
            else m_io->set_dir(1);
            m_io->write_pwm_motor(saturate( (fabs(u) + OFFSET_VOLTAGE) / POWERSUPPLY_VOLTAGE, 0.0f, 1.0f) );
        } else {
            m_io->write_pwm_motor(0.0f);
            u_i = 0.0f;
        }

    } // endof the main loop
}

// ----------------------------------------------------------------------------
void realtime_thread::sendSignal() { thread.flags_set(threadFlag); }

void realtime_thread::start_loop(void)
{
  thread.start(callback(this, &realtime_thread::loop));
  ticker.attach(callback(this, &realtime_thread::sendSignal), Ts);
}

float realtime_thread::saturate(float val, float min, float max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}
