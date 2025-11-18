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
  char enable;
  // AUFGABE 1.4
  while (1)
    {
    ThisThread::flags_wait_any(threadFlag);
    tim = 1e-6*(duration_cast<microseconds>(ti.elapsed_time()).count());
// --------------------- THE LOOP -----------------------------------------
    if(sp.readable())
        {
        sp.get(buffer, 5, true);            // read values (set values and enable) from UART
        w =	*(float *)&buffer[0];           // from Matlab 1 float value (4 bytes) + enable (1 byte) are sent
        enable = buffer[4];                 // uint8 value
        //m_io->write_aout(w);              // write to analog output
        f_val[0] = w;                       // write value 1
        f_val[1] = (float)enable;           // write value 2
        sp.put((char*)&f_val[0],8,true);      // write to UART
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
