#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <string>
#include <initializer_list>
#define ARDUINO_ISR_ATTR
#define LOW 0
#define HIGH 1
#define OUTPUT 1
#define INPUT_PULLUP 2
#define RISING 1
#define SERIAL_8N1 0
#define PI 3.14159265358979323846
using portMUX_TYPE=int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) (void)(x)
#define portEXIT_CRITICAL(x) (void)(x)
#define portENTER_CRITICAL_ISR(x) (void)(x)
#define portEXIT_CRITICAL_ISR(x) (void)(x)
extern uint32_t clockMs;extern uint32_t clockUs;
inline uint32_t micros(){return clockUs;}
inline uint32_t millis(){return clockMs;}
inline void delay(int){}
inline void delayMicroseconds(int){}
inline void digitalWrite(int,int){}
extern bool testButton;
inline int digitalRead(int pin){return pin==9&&testButton?LOW:HIGH;}
inline void pinMode(int,int){}
inline int digitalPinToInterrupt(int p){return p;}
inline void attachInterrupt(int,void(*)(),int){}
inline void ledcWrite(int,int){}
inline bool ledcAttach(int,int,int){return true;}
inline double ledcSetup(int,int,int){return 20000;}
inline void ledcAttachPin(int,int){}
struct HardwareSerial{HardwareSerial(int){}void begin(uint32_t,int,int,int){}void end(){}void setRxBufferSize(int){}int available(){return 0;}int read(){return 0;}void write(const uint8_t*,int){}void flush(){}};
