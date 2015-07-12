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
#define test(bits) (1 << bits)

#define delay_val 3

void setup() {
  Serial.begin(9600);
  SPI.begin();
  
  pinMode(SH_CP_PIN, OUTPUT);
  pinMode(ST_CP_PIN, OUTPUT);
  pinMode(DS_PIN, OUTPUT);
  
//  customShift(0, 8);
//  customShift(0, 8);
//  customShift(B10101010, 8);
  
  //byte val = 16;
  byte val = _BV(0) + _BV(4);
  for (int i = 0; i != 2047; ++i) {
    ST_CP_low();
    
    for (int j = 0; j != 5; ++j)
      SPI.transfer(val);
    ST_CP_high();
    //val = (val << 1 | val >> 3) & B11110000;
    val = (val << 1 | val >> 7);
    delay(delay_val);
  }
  
  for (int i = 0; i != 5; ++i)
    SPI.transfer(0);
  ST_CP_low();
  ST_CP_high();
  
  val = _BV(0) + _BV(4);
  for (int i = 0; i != 2047; ++i) {
    ST_CP_low();
    
    for (int j = 0; j != 5; ++j)
      SPI.transfer(val);
    ST_CP_high();
    //val = (val << 1 | val >> 3) & B11110000;
    val = (val >> 1 | val << 7);
    delay(delay_val);
  }
  
  for (int i = 0; i != 5; ++i)
    SPI.transfer(0);
  ST_CP_low();
  ST_CP_high();
}

void loop() {}

void customShift(const unsigned long& val, const int& bits) {
//  Serial.print("val: ");
//  Serial.println(val);
  for (int i = 0; i < bits; ++i) {
    if ((_BV(i) & val) == _BV(i)) {
      //Serial.print(1);
      DS_high();
    } else {
      //Serial.print(0);
      DS_low();
    }
    SH_CP_high();
    SH_CP_low();    
  }
}
