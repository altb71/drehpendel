#pragma once

#include <Eigen/Dense>

#include "Chirp.h"
#include "IIRFilter.h"
#include "IO_handler.h"
#include "SerialStream.h"
#include "ThreadFlag.h"
#include "mbed.h"
#include "serial_pipe.h"

using namespace std::chrono;

#define POWERSUPPLY_VOLTAGE 24.0f // Voltage of the power supply in Volts
#define OFFSET_VOLTAGE 2.0f       // Offset voltage to overcome motor deadzone in Volts
#define WATCHDOG_TIMEOUT 0.3f     // Watchdog timeout in seconds
#define KP_I 1.5000f              // Proportional gain current controller
#define KI_I 2.0735e+03f          // Integral gain current controller
#define F_CUT_HZ 180.0f           // Second order low-pass filter cutoff frequency in Hz
#define D 0.7f                    // Second order low-pass filter damping ratio
#define F0_HZ 0.05f
#define T1_SEC 1 / F0_HZ
#define AMP_V 4.0f
#define OFFSET_V 6.0f

class realtime_thread
{
public:
    realtime_thread(IO_handler &io_handler, float Ts);
    virtual ~realtime_thread();
    void start_loop(void);

private:
    Thread thread;
    Ticker ticker;
    ThreadFlag threadFlag;
    float Ts;
    IO_handler &io_handler;
    SerialPipe sp; // Serial pipe for UART communication
    IIRFilter lowPass2;

    SerialStream m_SerialStream;
    Timer m_Timer;
    microseconds m_time_previous_us{0};
    Chirp m_Chirp;
    float m_sinarg{0.0f};

    void loop(void);
    void sendSignal();
    float saturate(float val, float min, float max);
};
