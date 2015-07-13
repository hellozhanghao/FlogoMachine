#include "SPI.h"
#define SH_CP_PORT PORTB
#define ST_CP_PORT PORTB
#define DS_PORT PORTB
#define SH_CP_PIN 13
#define ST_CP_PIN 10
#define DS_PIN 11

#define SH_CP_high() SH_CP_PORT |= _BV(SH_CP_PIN - 8)
#define SH_CP_low() SH_CP_PORT &= ~_BV(SH_CP_PIN - 8)
#define ST_CP_high() ST_CP_PORT |= _BV(ST_CP_PIN - 8)
#define ST_CP_low() ST_CP_PORT &= ~_BV(ST_CP_PIN - 8)
#define DS_high() DS_PORT |= _BV(DS_PIN - 8)
#define DS_low() DS_PORT &= ~_BV(DS_PIN - 8)

//rotate left and rotate right
#define rol(val, bits) ((val << 1) | (val >> (bits - 1))) & ((1 << bits) - 1)
#define ror(val, bits) ((val >> 1) | (val << (bits - 1))) & ((1 << bits) - 1)

#define DELAY_VAL 3
#define REGISTER_COUNT 5
#define FULL_STEP_CYCLE 2047
#define HALF_STEP_CYCLE 4095

void setup() {
  SPI.begin();
  
  pinMode(SH_CP_PIN, OUTPUT);
  pinMode(ST_CP_PIN, OUTPUT);
  pinMode(DS_PIN, OUTPUT);
  
  clearRegister(REGISTER_COUNT); 

  testAllStepper();  

  clearRegister(REGISTER_COUNT); 
}

void loop() {}

void customShift(const unsigned long& val, const int& bits) {
  for (int i = 0; i < bits; ++i) {
    if ((_BV(i) & val) == _BV(i)) {
      DS_high();
    } else {
      DS_low();
    }
    SH_CP_high();
    SH_CP_low();    
  }
}

void clearRegister(const int& register_count) {
  for (int i = 0; i != register_count; ++i)
    SPI.transfer(0);
  ST_CP_low();
  ST_CP_high();
}

void testAllStepper() {
  byte val = B00010001; 
  for (int i = 0; i != FULL_TURN_STEPS; ++i) {
    ST_CP_low();
    
    for (int j = 0; j != REGISTER_COUNT; ++j)
      SPI.transfer(val);
    ST_CP_high();

    //val = rol(val, 4); 
    val = rol(val, 8); 
    delay(DELAY_VAL);
  }
}
