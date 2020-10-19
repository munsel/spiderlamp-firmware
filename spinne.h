/** //SPINNE control program
*  designed for atmega328P
*  Marius Unsel 2015
**/




#ifndef _SPINNE_H
#define _SPINNE_H
#include <inttypes.h>
#include <avr/io.h>

/**
* pin definitions of the micro
**/

#define KILL PB4
#define MOTORS_ON PD7

#define M1_AIN1 PB1
#define M1_AIN2 PC1
#define M1_BIN1 PC3
#define M1_BIN2 PC2
#define M1_ASNS PB0
#define M1_BSNS PB2

#define M2_AIN1 PC4
#define M2_AIN2 PC5
#define M2_BIN1 PD4
#define M2_BIN2 PD2
#define M2_ASNS 6
#define M2_BSNS 7

#define BAT_STAT PC0

#define RGB_R PD6
#define RGB_G PD5
#define RGB_B PD4

#define DIM_LED PB3

/**  init routines
**/


void initUART(void);
void initADC(void);
void initDigitalDirections(void);
void initPWMChannels(void);
void initTimer(void);

/** state of SPINNE
**/
enum SpinneState{IDLE, CONTROLLING, DONE_CONTROLLING};

struct SpinneStatusVariables{
  uint8_t r, g, b;
  uint8_t spotIntensity;
};

/** motor routines
*  first just on or off, because
  not every motor input is connected to a PWM channel
**/
enum MotorStates{CLOCKWISE, COUNTERCLOCKWISE, OFF};

void turnM1A(enum MotorStates state);
void turnM1B(enum MotorStates state);
void turnM2A(enum MotorStates state);
void turnM2B(enum MotorStates state);

void turnMotordriversOn(void);
void turnMotordriversOff(void);


//void turnMotor(unint8_t motorA,unint8_t motorB, enum Motordirections direction);

enum SpinneState controlMotors(void);


/**each spindle gets information about its
   rope length and the length, it should
   go
**/
struct MotorStatusVariables
{
    uint16_t currentValue;
    uint16_t setValue;
    uint8_t optoNow;
    uint8_t optoPrevious;
    enum MotorStates motorState;
    enum MotorStates motorStatePrevious;
};
int32_t getRopelengthError(struct MotorStatusVariables status);


/** LED routines
*  to set color values from 0-255
*  and the intensity of the main lamp
**/

void setRGB(uint8_t r, uint8_t g, uint8_t b);
void setLampIntensity(uint8_t intensity);

/** battery routines
**/
uint16_t measureBattery(void);

/**
* messages and UART buffer
**/
#define BUFFER_SIZE 16
struct UartBuffer
{
  uint8_t data[BUFFER_SIZE];
  uint8_t write_index;
  uint8_t read_index;
};

enum BufferStatus {BUFFER_OK, BUFFER_EMPTY, BUFFER_FULL};
enum BufferStatus bufferWrite(volatile struct UartBuffer *buffer, uint8_t byte);
enum BufferStatus bufferRead(volatile struct UartBuffer *buffer, uint8_t *readByte);
uint8_t getBufferSize( volatile struct UartBuffer *buffer);

#define MESSAGE_SIZE 2
struct UartMessage{
  uint8_t id;
  uint8_t data[MESSAGE_SIZE];
};
enum MessageState{MESSAGE_OK, MESSAGE_ERROR};

void sendBatteryStatus(uint16_t status);
void sendDoneStatus(uint8_t id);
void sendNotReceivedStatus(void);
void sendMessageNotReceivedStatus(void);

uint8_t isNewMessageAvailable(void);
enum MessageState readMessage(void);

enum SpinneState actOnMessage(void);
void updateStatus(void);

void switchOff(void);


#endif
