//*************************/
    //SPINNE
/*
*   bluetooth lamp
*   marius unsel 2015
*
***************************/

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>


#include "spinne.h"



volatile uint8_t timestep_5ms;

int main(void){
  enum SpinneState state = IDLE;
  enum MessageState messageState;
  initDigitalDirections();
  initUART();
  initADC();
  initPWMChannels();
  initTimer();
  sei(); //global enable interrupts
  turnMotordriversOn();
PORTB |= (1 << DIM_LED); // turn spotlight on

  while(1) //forever loop
  {

    if (timestep_5ms)
    {
      timestep_5ms = 0;
      if (isNewMessageAvailable())
      {
        messageState = readMessage();
        if (messageState == MESSAGE_OK)
        {
          state = actOnMessage(); //this guy does all the settings
        }
        else if( messageState == MESSAGE_ERROR)
        {
          continue;
        }
      }
      if (state == CONTROLLING)
      {
        state = controlMotors();
      }
      if(state == DONE_CONTROLLING)
      {
        sendDoneStatus('u');
        state = IDLE;
      }
    }

  }

  return 0; //never reached
}

ISR(TIMER1_OVF_vect)
{
  TCNT1 = 0x3FFF; //set timer value, so that it overflows in 5ms
  timestep_5ms = 1;
}
