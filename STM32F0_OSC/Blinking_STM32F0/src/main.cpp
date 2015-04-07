//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"



extern "C" {
#include "uart.h"
#include "stm32f0xx.h"
#include "Timer.h"
#include "BlinkLed.h"
}

#include "OSC/OSCMessage.h"
#include "OSC/SLIPEncodedSerial.h"

// ----------------------------------------------------------------------------
//
// STM32F0 led blink sample (trace via $(trace)).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// To demonstrate POSIX retargetting, reroute the STDOUT and STDERR to the
// trace device and display messages on both of them.
//
// Then demonstrates how to blink a led with 1Hz, using a
// continuous loop and SysTick delays.
//
// On DEBUG, the uptime in seconds is also displayed on the trace device.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
// The external clock frequency is specified as a preprocessor definition
// passed to the compiler via a command line option (see the 'C/C++ General' ->
// 'Paths and Symbols' -> the 'Symbols' tab, if you want to change it).
// The value selected during project creation was HSE_VALUE=8000000.
//
// Note: The default clock settings take the user defined HSE_VALUE and try
// to reach the maximum possible system clock. For the default 8MHz input
// the result is guaranteed, but for other values it might not be possible,
// so please adjust the PLL settings in system/src/cmsis/system_stm32f0xx.c
//

// ----- Timing definitions -------------------------------------------------

// Keep the LED on for 2/3 of a second.
#define BLINK_ON_TICKS  (TIMER_FREQUENCY_HZ * 2 / 3)
#define BLINK_OFF_TICKS (TIMER_FREQUENCY_HZ - BLINK_ON_TICKS)

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

SLIPEncodedSerial SLIPSerial;

// reset to default turn on state
void reset(OSCMessage &msg){
   // blink_led_off();
}

void renumber(OSCMessage &msg){
  // send out a
  /*OSCMessage  msgOut("asdf");
  SLIPSerial.beginPacket();
  msgOut.send(SLIPSerial);
  SLIPSerial.endPacket();
  msgOut.empty();*/
  //blink_led_on();
}

int
main(int argc, char* argv[])
{
  // By customising __initialize_args() it is possible to pass arguments,
  // for example when running tests with semihosting you can pass various
  // options to the test.
  // trace_dump_args(argc, argv);

  // Send a greeting to the trace device (skipped on Release).
 // trace_puts("Hello ARM World!");

  // The standard output and the standard error should be forwarded to
  // the trace device. For this to work, a redirection in _write.c is
  // required.
 // puts("Standard output message.");
  //fprintf(stderr, "Standard error message.\n");

  // At this stage the system clock should have already been configured
  // at high speed.
  //trace_printf("System clock: %uHz\n", SystemCoreClock);

 // timer_start();

 // blink_led_init();
  
  uint32_t seconds = 0;


  //the message wants an OSC address as first argument
  OSCMessage msg("/thing/test");
  msg.add(99);

  OSCMessage msgIn;

  int size;

  uart2_init();

  blink_led_init();

  timer_start();
  // Infinite loop

  int msgInSize = 0;

  while (1)
    {
     // blink_led_on();
    /*  timer_sleep(BLINK_ON_TICKS);

     // blink_led_off();
      timer_sleep(BLINK_OFF_TICKS);

     // uart_2_send(0xAA);

      SLIPSerial.beginPacket();
      msg.send(SLIPSerial);
      SLIPSerial.endPacket();

      ++seconds;*/

      // Count seconds on the trace device.
     // trace_printf("Second %u\n", seconds);

              // use msgInSize hack cause endofPacket can return true at beginning and end of a packet
            while((!SLIPSerial.endofPacket()) || (msgInSize < 4) ) {

              if( (size =SLIPSerial.available()) > 0) {

                while(size--) {
                  msgIn.fill(SLIPSerial.read());
                  msgInSize++;
                }
              }
            }
            msgInSize = 0;

            if(!msgIn.hasError()) {

                // pass it along
                blink_led_on();
                SLIPSerial.beginPacket();
                msgIn.send(SLIPSerial); // send the bytes to the SLIP stream
                SLIPSerial.endPacket(); // mark the end of the OSC Packet
                blink_led_off();

                // renumber
                msgIn.dispatch("/sys/renumber", renumber, 0);

                // reset
                msgIn.dispatch("/sys/reset", reset, 0);

                // type count
               // address = "/" + type;
                //msgIn.dispatch(address.c_str(), incIndex);

                msgIn.empty(); // free space occupied by message


            }
           else {   // just empty it if there was an error
                msgIn.empty(); // free space occupied by message
            }

    }
  // Infinite loop, never return.
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
