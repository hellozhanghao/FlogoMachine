#include "SPI.h"

#define ST_CP_PORT PORTB
#define SH_CP_PIN 13
#define ST_CP_PIN 10
#define DS_PIN 11

#define ST_CP_high() ST_CP_PORT |= _BV(ST_CP_PIN - 8)
#define ST_CP_low() ST_CP_PORT &= ~_BV(ST_CP_PIN - 8)

#define rol(val, bits) ((val << 1) | (val >> (bits - 1))) & ((1 << bits) - 1)
#define ror(val, bits) ((val >> 1) | (val << (bits - 1))) & ((1 << bits) - 1)

#define DELAY_VAL 1
#define GEAR_VAL 400
#define REGISTER_COUNT 5
#define STEPS_FOR_COMPLETE_SHUT1 800
#define STEPS_FOR_COMPLETE_SHUT2 1200

//#define DELAY_VAL 1
//#define GEAR_VAL 5
//#define REGISTER_COUNT 5
//#define STEPS_FOR_COMPLETE_SHUT1 10
//#define STEPS_FOR_COMPLETE_SHUT2 15

class Motor {
public:
    Motor() {};
//    Motor(const int steps) {target_steps = steps;};
    int val = 0;
    unsigned long steps_taken = 0;
    unsigned long target_steps = 0;
    int dir = 0;
};

class Register {
public:
    Register() {};
//    Register(const int step1, const int step2) {
//        motor1 = Motor(step1);
//        motor2 = Motor(step2);
//    }
    Motor motor1, motor2;
};

class Shutter {
public:
    Shutter(const int);
    void resetSteps();
    void populateRegister();
    void populateRegister(const int, const char*);
    void populateRegisterForClosing();
    unsigned long getRemainingSteps();
    unsigned long getRemainingStepsVerbose();
    void beginShutter();
    void closeShutter();
    void resetShutter();
    int getRegisterValue(const int);
    void updateRegisterValue(const int);

private:

    int register_count = 0;
    Register* reg;
};


Shutter::Shutter(const int reg_count) {
    register_count = reg_count;
    reg = new Register[reg_count];
}

void Shutter::resetSteps() {
    for (int i = 0; i != register_count; ++i) {
        reg[i].motor1.steps_taken = 0;
        reg[i].motor2.steps_taken = 0;
    }
}

void Shutter::populateRegister() {
    char msg_buffer[11]; 
    for (int i = 0; i != register_count; ++i) {
        Serial.readBytes(msg_buffer, 11);
        if (msg_buffer[0] == 'S' && msg_buffer[10] == 'E')
            Shutter::populateRegister(i, msg_buffer);
        else
            Serial.println("error");
    }
}

void Shutter::populateRegister(const int register_index, const char* serial_msg) {
    String reg_info = String(serial_msg);
    reg[register_index].motor1.target_steps = reg_info.substring(5, 7).toInt() * GEAR_VAL;
    reg[register_index].motor2.target_steps = reg_info.substring(8, 10).toInt() * GEAR_VAL;
}

void Shutter::populateRegisterForClosing() {
    for (int i = 0; i != register_count; ++i) {
        reg[i].motor1.target_steps = STEPS_FOR_COMPLETE_SHUT1;
        reg[i].motor2.target_steps = STEPS_FOR_COMPLETE_SHUT2;
    }
}

unsigned long Shutter::getRemainingSteps() {
    unsigned long steps = 0;
    for (int i = 0; i!= register_count; ++i) {
        steps += reg[i].motor1.target_steps - reg[i].motor1.steps_taken;
        steps += reg[i].motor2.target_steps - reg[i].motor2.steps_taken;
    }
    return steps;
}

unsigned long Shutter::getRemainingStepsVerbose() {
    unsigned long steps = 0;
    for (int i = 0; i!= register_count; ++i) {
        steps += reg[i].motor1.target_steps - reg[i].motor1.steps_taken;
        Serial.println("remaining steps:");
        Serial.print("R");
        Serial.println(i);
        Serial.print("motor1: ");
        Serial.println(reg[i].motor1.target_steps - reg[i].motor1.steps_taken);
        steps += reg[i].motor2.target_steps - reg[i].motor2.steps_taken;
        Serial.print("motor2: ");
        Serial.println(reg[i].motor2.target_steps - reg[i].motor2.steps_taken);
    }
    return steps;
}

void Shutter::beginShutter() {
    resetSteps();
    unsigned long remaining_steps = getRemainingSteps();
//    Serial.println("S");
//    Serial.println(remaining_steps);
//    Serial.println("E");
    for (int i = 0; i != register_count; ++i) {
        reg[i].motor1.dir = 1;
        reg[i].motor1.val = 1;

        reg[i].motor2.dir = 1;
        reg[i].motor2.val = 1;
    }

    while (remaining_steps > 0) {
        ST_CP_low();
        for (int i = 0; i != register_count; ++i) {
            SPI.transfer(getRegisterValue(i));
            //Serial.println(getRegisterValue(i), BIN);
            updateRegisterValue(i);
            delay(DELAY_VAL);
        }
        ST_CP_high();
        //Serial.println("stored");
        remaining_steps = getRemainingSteps();
        //Serial.println("remaining steps in beginShutter");
        //Serial.println(remaining_steps);
    }
}

void Shutter::resetShutter() {
    resetSteps();
    unsigned long remaining_steps = getRemainingSteps();
    for (int i = 0; i != register_count; ++i) {
        reg[i].motor1.dir = -1;
        reg[i].motor1.val = 1;

        reg[i].motor2.dir = -1;
        reg[i].motor2.val = 1;
    }
    while (remaining_steps > 0) {
        ST_CP_low();
        for (int i = 0; i != register_count; ++i) {
            SPI.transfer(getRegisterValue(i));
            //Serial.println(getRegisterValue(i), BIN);
            updateRegisterValue(i);
            delay(DELAY_VAL);
        }
        ST_CP_high();
        remaining_steps = getRemainingSteps();
        //Serial.println("remaining steps in resetShutter");
        //Serial.println(remaining_steps);
    }
}

void Shutter::closeShutter() {
    populateRegisterForClosing();
    unsigned long remaining_steps = getRemainingSteps();
    //Serial.println(remaining_steps);
    for (int i = 0; i != register_count; ++i) {
        reg[i].motor1.dir = 1;
        reg[i].motor1.val = reg[i].motor1.steps_taken < reg[i].motor1.target_steps ? 1 : 0;

        reg[i].motor2.dir = 1;
        reg[i].motor2.val = reg[i].motor2.steps_taken < reg[i].motor2.target_steps ? 1 : 0;
    }
    while (remaining_steps > 0) {
        ST_CP_low();
        for (int i = 0; i != register_count; ++i) {
            SPI.transfer(getRegisterValue(i));
            //Serial.println(getRegisterValue(i), BIN);
            updateRegisterValue(i);
            delay(DELAY_VAL);
        }
        ST_CP_high();
        remaining_steps = getRemainingSteps();
        //Serial.println("remaining steps in closeShutter");
        //Serial.println(remaining_steps);
    }
}

int Shutter::getRegisterValue(const int reg_index) {
    return (reg[reg_index].motor1.val << 4) + reg[reg_index].motor2.val;
}

void Shutter::updateRegisterValue(const int reg_index) {
    int* val1 = &reg[reg_index].motor1.val;
    int* val2 = &reg[reg_index].motor2.val;
    if (reg[reg_index].motor1.steps_taken < reg[reg_index].motor1.target_steps) {
        *val1 = reg[reg_index].motor1.dir > 0 ? rol(*val1, 4) : ror(*val1, 4);
        reg[reg_index].motor1.steps_taken++;
    } else {
        *val1 = 0;
    }
    if (reg[reg_index].motor2.steps_taken < reg[reg_index].motor2.target_steps) {
        *val2 = reg[reg_index].motor2.dir > 0 ? rol(*val2, 4) : ror(*val2, 4);
        reg[reg_index].motor2.steps_taken++;
    } else {
        *val2 = 0;
    }
    
}

Shutter shutter(REGISTER_COUNT);

void setup() {
    Serial.begin(9600);
    SPI.begin();
    pinMode(SH_CP_PIN, OUTPUT);
    pinMode(ST_CP_PIN, OUTPUT);
    pinMode(DS_PIN, OUTPUT);
}

void loop() {
    if (Serial.available() > 0) {
        if (Serial.read() == 'B') {
            clearRegister(5);
            
            shutter.populateRegister();
            shutter.beginShutter();
            shutter.closeShutter();
            shutter.resetShutter();
            
            clearRegister(5);
            Serial.println("done");
        }
    }
}

void clearRegister(const int& register_count) {
  for (int i = 0; i != register_count; ++i)
    SPI.transfer(0);
  ST_CP_low();
  ST_CP_high();
}
