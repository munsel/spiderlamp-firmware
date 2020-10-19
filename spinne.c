#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#ifndef F_CPU
#define F_CPU 9830600UL
//#define F_CPU 9830400UL
//#define F_CPU 16000000UL
//#define F_CPU 1000000UL
//define F_CPU 3686400UL
#endif

#define BAUD 9600UL
#include <util/setbaud.h>
#include <util/delay.h>

#include "spinne.h"

  volatile struct UartBuffer uartRxBuffer = { {}, 0, 0};
  volatile struct UartBuffer uartTxBuffer = { {}, 0, 0};
  enum BufferStatus rx_buffer_status;
  uint8_t rx_buffer_last_byte;
  volatile uint8_t rx_buffer_bytes_cnt = 0;
  struct UartMessage incommingMessage = {0,{}};

  struct SpinneStatusVariables currentStatus;
  struct SpinneStatusVariables newStatus;

void initUART(void)
{
  UBRR0H = UBRRH_VALUE; // Baudrate high byte
  UBRR0L = UBRRL_VALUE; // Baudrate low byte
  // enable transmitter, receiver and interrupts
  UCSR0B |= (1<<TXEN0) | (1<<UDRIE0) | (1<<RXCIE0) | (1<<RXEN0);
  // Frame Format: Asynchronous 8N1
  UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
}

void initADC(void)
{
  ADMUX |= _BV(REFS0); //AVCC as Voltage reference
  ADCSRA |= _BV(ADPS1) | _BV(ADPS0); //ADC frequency = CLOCK / 8
  ADCSRA |= _BV(ADEN); //enable ADC

  //dummy readout
  ADCSRA |= _BV(ADSC);
  while (ADCSRA & ( _BV(ADSC) ) ){}//wait till bit is cleared
  (void) ADCW;
}

void initDigitalDirections(void)
{
  MCUCR |= _BV(PUD);//disable pullup-resistors
  DDRD |= (1<< DDD6) | (1<<DDD5) | (1<<DDD3); //RGB
  //PORTD = 1<<PD6;

  DDRB |= (1 << DIM_LED); //spotlight

  DDRB &= ~( (_BV(M1_ASNS)) | (_BV(M1_BSNS)) );

  DDRB |= (1 << M1_AIN1) | (1 << KILL);
  DDRC |=  (1 << M1_AIN2) | (1 << M1_BIN1) | (1 << M1_BIN2) | (1 << M2_AIN1) | (1 << M2_AIN2);
  DDRD |= (1 << M2_BIN1) | (1 << M2_BIN2) | (1 << MOTORS_ON);

}


void initPWMChannels(void)
{
  TCCR0A |=  _BV(COM0A1) |_BV(COM0B1)| (1 << WGM00) | (1 << WGM01);
  TCCR0B |= _BV(CS01) | _BV(CS00);
  OCR0A = 10;
  OCR0B = 50;

  TCCR2A |= _BV(COM2A1) | _BV(COM2B1) | _BV(WGM00) | _BV(WGM01);
  TCCR2B |= _BV(CS01) | _BV(CS00);
  OCR2B = 30;
}

void initTimer(void)
{
  TCNT1= 0x3FFF;
  TCCR1B |= _BV(CS10); // no prescaler on clock source
  //enable timer Overflowus interruptus
  TIMSK1 |= _BV(TOIE1);
}


void turnM1AClockwise(void){
  PORTD |= (1 << M2_BIN1);
  PORTD &= ~(1 << M2_BIN2);
}
void turnM1ACounterclockwise(void){
  PORTD &= ~(1 << M2_BIN1);
  PORTD |= (1 << M2_BIN2);
}
void stopM1A(void){
  PORTD &= ~(1 << M2_BIN1);
  PORTD &= ~(1 << M2_BIN2);
}

void turnM1BClockwise(void){
  PORTB &= ~(1 << M1_AIN1);
  PORTC |= (1 << M1_AIN2);
}
void turnM1BCounterclockwise(void){
  PORTB |= (1 << M1_AIN1);
  PORTC &= ~(1 << M1_AIN2);
}
void stopM1B(void){
  PORTB &= ~(1 << M1_AIN1);
  PORTC &= ~(1 << M1_AIN2);
}

void turnM2AClockwise(void){
  PORTC |= (1 << M1_BIN1);
  PORTC &= ~(1 << M1_BIN2);
}
void turnM2ACounterclockwise(void){
  PORTC &= ~(1 << M1_BIN1);
  PORTC |= (1 << M1_BIN2);
}
void stopM2A(void){
  PORTC &= ~(1 << M1_BIN1);
  PORTC &= ~(1 << M1_BIN2);
}

void turnM2BClockwise(void){
  PORTC |= (1 << M2_AIN1);
  PORTC &= ~(1 << M2_AIN2);
}
void turnM2BCounterclockwise(void){
  PORTC &= ~(1 << M2_AIN1);
  PORTC |= (1 << M2_AIN2);
}
void stopM2B(void){
  PORTC &= ~(1 << M2_AIN1);
  PORTC &= ~(1 << M2_AIN2);
}

void stopAllMotors(void)
{
  stopM1A();
  stopM1B();
  stopM2A();
  stopM2B();
}
void turnMotordriversOn(void)
{
  PORTD |= (1<< MOTORS_ON);
}

void turnMotordriversOff(void){
  PORTD &= ~(1<< MOTORS_ON);
}

/**
  functions to read out the HIGH or LOW state
  of the four lightswitches
  two of them via reading the digital values,
  two of them via reading out the ADC_value
**/
uint8_t isM1AOptoHigh(void)
{
  return (PINB & _BV(M1_ASNS));
}
uint8_t isM1BOptoHigh(void)
{
  return (PINB & _BV(M1_BSNS));
}

uint8_t readDigitalWithADC(uint8_t ADCPin){
  uint16_t tempy;
  ADMUX = (ADMUX & ~(0x1F)) | (ADCPin &0x1F);
  ADCSRA |= _BV(ADSC);
  for (uint8_t i = 0; i < 3; i++) {
    while(ADCSRA & (_BV(ADSC) )){}
    tempy = ADCW;
  }
  while(ADCSRA & (_BV(ADSC) )){}
  tempy = ADCW;
  if (tempy > 800){
    return 1;
  }
  //setRGB(0,255,0);
  return 0;
}

uint8_t isM2AOptoHigh(void)
{
  return readDigitalWithADC(M2_ASNS);
}
uint8_t isM2BOptoHigh(void)
{
  return readDigitalWithADC(M2_BSNS);
}

/**
   Containers to hold the length of each status and
   the length, it should be controlled to
**/
struct MotorStatusVariables statusM1A = {0,0,0,0,OFF,OFF};
struct MotorStatusVariables statusM1B = {0,0,0,0,OFF,OFF};
struct MotorStatusVariables statusM2A = {0,0,0,0,OFF,OFF};
struct MotorStatusVariables statusM2B = {0,0,0,0,OFF,OFF};

void decodeOptoBridge( struct MotorStatusVariables *status,
   uint8_t (*currentOptoLevel)(void) )
{
  uint8_t newOpto = (*currentOptoLevel)();
  status->optoPrevious = status->optoNow;
  status->optoNow = newOpto;
  if (  ( !status->optoPrevious ) && ( status->optoNow ) )
  {
    switch (status->motorState)
    {
      case CLOCKWISE:
        status->currentValue--;
        break;
      case COUNTERCLOCKWISE:
        status->currentValue++;
        break;
      case OFF:
        break;
    }
  }
}

uint8_t controlSingleMotor( struct MotorStatusVariables *variables,
  uint8_t (*currentOptoLevel)(void),
   void (*turnClockwise)(void),
   void (*turnCounterclockwise)(void),
   void (*stop)(void))
{
  if(variables->setValue != variables->currentValue)
  {
    if(variables->setValue < variables->currentValue)
    {
      variables->motorState = CLOCKWISE;
      (*turnClockwise)();
    }
    else
    {
      variables->motorState = COUNTERCLOCKWISE;
      (*turnCounterclockwise)();
    }
  }
  decodeOptoBridge(variables, currentOptoLevel);
  if (variables->currentValue == variables->setValue)
  {
    variables->motorStatePrevious = variables->motorState;
    variables->motorState = OFF;
    (*stop)();
    return 0;
  }
  return 1;
}

enum SpinneState controlMotors(void){
  static uint8_t nTicks = 0;
  uint8_t eM1A, eM1B, eM2A, eM2B;
  eM1A = controlSingleMotor(&statusM1A,
    isM1AOptoHigh,
    turnM1AClockwise,
    turnM1ACounterclockwise,
    stopM1A);
  eM1B = controlSingleMotor(&statusM1B,
    isM1BOptoHigh,
    turnM1BClockwise,
    turnM1BCounterclockwise,
    stopM1B);
  eM2A = controlSingleMotor(&statusM2A,
    isM2AOptoHigh,
    turnM2AClockwise,
    turnM2ACounterclockwise,
    stopM2A);
  eM2B = controlSingleMotor(&statusM2B,
    isM2BOptoHigh,
    turnM2BClockwise,
    turnM2BCounterclockwise,
    stopM2B);
  if (eM1A|eM1B|eM2A|eM2B)
    return CONTROLLING;
  if (nTicks++ == 5)
  {
    nTicks = 0;
    return IDLE;
  }
  return CONTROLLING;
}

void setRGB(uint8_t r, uint8_t g, uint8_t b)
{
  DDRD |= _BV(DDD6) | _BV(DDD5) | _BV(DDD3);
  OCR0A = r;
  OCR0B = g;
  OCR2B = b;
  if (r==0)DDRD &= ~_BV(DDD6);
  if (g==0)DDRD &= ~_BV(DDD5);
  if (b==0)DDRD &= ~_BV(DDD3);
}

void setLampIntensity(uint8_t intensity)
{
  OCR2A = intensity;
}

/**returns ADC Value measured at battery **/
uint16_t measureBattery(void)
{
  ADMUX = (ADMUX & ~(0x1F)) | (BAT_STAT &0x1F);
  ADCSRA |= _BV(ADSC);
  while(ADCSRA & (_BV(ADSC) )){}
  return ADCW;
}



enum BufferStatus bufferWrite(volatile struct UartBuffer *buffer, uint8_t byte){
  uint8_t new_write_index = ( ((buffer->write_index)+1) % BUFFER_SIZE);
  if (new_write_index == buffer->read_index) {
    return BUFFER_FULL;
  }
  buffer->data[new_write_index] = byte;
  buffer->write_index = new_write_index;
  return BUFFER_OK;
}

enum BufferStatus bufferRead(volatile struct UartBuffer *buffer, uint8_t *readByte){
  if (buffer->read_index == buffer->write_index) {
    return BUFFER_EMPTY;
  }
  buffer->read_index = ((buffer->read_index+1)%BUFFER_SIZE);
  *readByte = buffer->data[buffer->read_index];
  return BUFFER_OK;
}

uint8_t getBufferSize( volatile struct UartBuffer *buffer)
{
   return (BUFFER_SIZE + buffer->write_index-buffer->read_index)%BUFFER_SIZE;
}

enum BufferStatus bufferPeek(volatile struct UartBuffer *buffer, uint8_t *peekByte){
  uint8_t last_write_index = ((BUFFER_SIZE + (buffer->write_index)-1) % BUFFER_SIZE);
  if (buffer->read_index == buffer->write_index) {
    return BUFFER_EMPTY;
  }
  *peekByte = buffer->data[last_write_index];
  return BUFFER_OK;
}

/** to enable the interrupts for the serial interface**/
static inline void enable_transmission(void){
    UCSR0B |= (1<<UDRIE0);
}

static inline void disable_transmission(void){
    UCSR0B &= ~(1<<UDRIE0);
}

/**interrupt service routines to fill the uartRxBuffer
 * or to clear the uartTxBuffer
 **/
ISR(USART_RX_vect)
{
  bufferWrite(&uartRxBuffer, UDR0);
}

ISR(USART_UDRE_vect)
{
    uint8_t tempy;
    enum BufferStatus status_tx;
    status_tx = bufferRead(&uartTxBuffer, &tempy);
    if (status_tx == BUFFER_EMPTY)
    {
        disable_transmission();
    } else
    {
        UDR0 = tempy; /* write it out to the hardware serial */
    }
}

void sendBatteryStatus(uint16_t status)
{
  bufferWrite(&uartTxBuffer, 6);
  bufferWrite(&uartTxBuffer, status & 0xFF);
  bufferWrite(&uartTxBuffer, (status>>8) & 0xFF);
  bufferWrite(&uartTxBuffer, '\n');
  enable_transmission();
  return;
}


void sendDoneStatus(uint8_t id)
{
  bufferWrite(&uartTxBuffer, id);
  bufferWrite(&uartTxBuffer, 1);
  bufferWrite(&uartTxBuffer, 1);
  bufferWrite(&uartTxBuffer, '\n');
  enable_transmission();
  return;
}

void sendNotReceivedStatus(void){
  return;
}

void sendMessageNotReceivedStatus(void){
   return;
}



uint8_t isNewMessageAvailable(void){
  if (getBufferSize(&uartRxBuffer)>=4) {
    //sendDoneStatus('d');
    return 1;
  }

 //rx_buffer_status =  bufferPeek(&uartRxBuffer, &rx_buffer_last_byte);
 //if (rx_buffer_status == BUFFER_OK && rx_buffer_last_byte == '\n'
/*  && getBufferSize(&uartRxBuffer)>=4) */ //){
    //return 1;
  //}
  /*if (rx_buffer_status == BUFFER_OK && rx_buffer_bytes_cnt >=4) {
    return 1;
  }*/
  return 0;
}

enum MessageState readMessage(void){
  uint8_t newByte;
  uint8_t i;
  //first readout should be the id
  rx_buffer_status = bufferRead(&uartRxBuffer, &newByte );
  if (rx_buffer_status == BUFFER_OK) {
    incommingMessage.id = newByte;
  }
  //then the data bytes
  for ( i = 0; i < MESSAGE_SIZE; i++)
  {
    rx_buffer_status = bufferRead(&uartRxBuffer, &newByte );

    if (rx_buffer_status == BUFFER_OK)
    {
        incommingMessage.data[i] = newByte;
    }
  }
  //now check, if the last byte is "\n"
  //newByte ='a'; //something that is not \n
  rx_buffer_status = bufferRead(&uartRxBuffer, &newByte );

  if ((rx_buffer_status == BUFFER_OK) && (newByte == 0x0A)) {
      return MESSAGE_OK;
  }

//when something fails...
  if (newByte != '\n')
  {
    setRGB(255,0,0);
  }
  if (rx_buffer_status == BUFFER_EMPTY)
  {
    setRGB(0,255,0);
  }
  if(newByte != '\n' && rx_buffer_status == BUFFER_EMPTY)
  {
    setRGB(255,255,0);
  }
  return MESSAGE_ERROR;
}

void clearBuffer(volatile struct UartBuffer *buffer){
  buffer->write_index = 0;
  buffer->read_index = 0;
}


/** interprets the obtained message and
*   changes the status of the lamp
*/
enum SpinneState actOnMessage(void){
  switch ( (incommingMessage.id) ) {
    //check for new RGB values
    case 1:
      newStatus.r = incommingMessage.data[0];
      sendDoneStatus(1);
      break;
    case 2:
      newStatus.g = incommingMessage.data[0];
      sendDoneStatus(2);
      return IDLE;
    case 3:
      newStatus.b = incommingMessage.data[0];
      sendDoneStatus(3);
      return IDLE;
    //check for spotlight
    case 4:
      newStatus.spotIntensity = incommingMessage.data[0];
      sendDoneStatus(4);
      return IDLE;
    //check for update signal
    case 5:
      currentStatus = newStatus;
      updateStatus();
      sendDoneStatus(5);
      //clearBuffer(&uartRxBuffer);
      return CONTROLLING;
      //check for battery status request
    case 6:
        sendBatteryStatus(measureBattery());
        break;
    //check for status lengths
    case 7:
      statusM1A.setValue = (incommingMessage.data[0]<<8)| incommingMessage.data[1];
      sendDoneStatus(7);
      break;
    case 8:
      statusM1B.setValue = (incommingMessage.data[0]<<8)| incommingMessage.data[1];
      sendDoneStatus(8);
      break;
    case 9:
      statusM2A.setValue = (incommingMessage.data[0]<<8)| incommingMessage.data[1];
      sendDoneStatus(9);
      break;
    case 10:
      statusM2B.setValue = (incommingMessage.data[0]<<8)| incommingMessage.data[1];
      sendDoneStatus(10);
      break;
    case 11:
      statusM1A.currentValue = (incommingMessage.data[0]<<8)| incommingMessage.data[1];
      sendDoneStatus(11);
      break;
    case 12:
      statusM1B.currentValue = (incommingMessage.data[0]<<8)| incommingMessage.data[1];
      sendDoneStatus(12);
      break;
    case 13:
      statusM2A.currentValue = (incommingMessage.data[0]<<8)| incommingMessage.data[1];
      sendDoneStatus(13);
      break;
    case 14:
      statusM2B.currentValue = (incommingMessage.data[0]<<8)| incommingMessage.data[1];
      sendDoneStatus(14);
      break;
    //check for off signal
    case 15:
      sendDoneStatus(15);
      //_delay_ms(1000);
      //switchOff();
      stopAllMotors();
      break;
    //if there is anything else, do nothing
    default:
      //setRGB(0, 0, 255);
      return IDLE;
  }
  return IDLE;
}


/**updates the LEDs **/
void updateStatus(void)
{
  setLampIntensity(currentStatus.spotIntensity);
  setRGB(currentStatus.r, currentStatus.g, currentStatus.b);
}


void switchOff(void)
{
  PORTB |= (1 << KILL);
}
