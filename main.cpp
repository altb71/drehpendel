#include "mbed.h"
#include <stdint.h>
#include "math.h" 
#include "GPA.h"
#include "DataLogger.h"
#include "realtime_thread.h"
#include "pendel_kinematics.h"
#include "FastPWM.h"
#include "IO_handler.h"
#include "uart_comm_thread_send.h"
#include "uart_comm_thread_receive.h"
#include "state_machine.h"
 
float Ts = 0.002f;                    // sampling time
// --------- local functions
//----------------------------------------- global variables (uhh!) ---------------------------
//init values:    (f0,   f1, nbPts, A0, A1, Ts)
GPA          myGPA(5 , 1000,    30,1,1, Ts);
DataLogger   myDataLogger(1);

//******************************************************************************
//---------- main loop -------------
//******************************************************************************
int main()
{
    // --------- Mirror kinematik, define values, trafos etc there
    pendel_kinematics kin;     // Mirror_Kinematics class, the geom. parameters, trafos etc. are done
    IO_handler hardware(&kin,Ts);
    static BufferedSerial uart_serial(USBTX, USBRX, 115200);
    uart_serial.set_format(8,BufferedSerial::None,1);
    uart_serial.set_blocking(false); // force to send whenever possible and data is there
    uart_comm_thread_send uart_com_send(&hardware,&uart_serial, .02f); // communication send thread
    uart_comm_thread_receive uart_com_receive(&kin,&uart_serial, .02f); // communication receive thread
    realtime_thread loop(&hardware,&kin,Ts);       // this is for the main controller loop
    state_machine sm(&hardware,&loop,.01);              // handles states
    ThisThread::sleep_for(200);
// ----------------------------------
    
    loop.init_controllers();
    uart_com_receive.start_uart();
    uart_com_send.start_uart();
    loop.start_loop();
    sm.start_loop();
    while(1)
        ThisThread::sleep_for(200);
     
}   // END OF main
