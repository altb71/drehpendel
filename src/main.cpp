/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "IO_handler.h"
#include "realtime_thread.h"
#include "GPA.h"
#include "DataLogger.h"
#include "uart_comm_thread_send.h"
#include "uart_comm_thread_receive.h"

float Ts = 0.0005f;
GPA myGPA (1, 1000, 30, .1,.2, Ts);
DataLogger myDataLogger(1);

int main()
{
    // thi input/output handling
    IO_handler hardware;
// Communication is put in the RT thread here! (also Serial Port definitions)
    realtime_thread rt_thread(&hardware,Ts);
    rt_thread.start_loop();

    while (true) {
        ThisThread::sleep_for(500ms);
    }
}
