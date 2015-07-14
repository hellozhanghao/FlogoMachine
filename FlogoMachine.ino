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

class Motor {
public:
    Motor() {};
    int val = 0;
    unsigned long steps_taken = 0;
    unsigned long target_steps = 0;
    int dir = 0;
};

class Register {
public:
    Register() {};
    Motor motor[2];
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
    void moveMotor();
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
        for (int j = 0; j != 2; ++j)
            reg[i].motor[j].steps_taken = 0;
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
    reg[register_index].motor[0].target_steps = reg_info.substring(5, 7).toInt() * GEAR_VAL;
    reg[register_index].motor[1].target_steps = reg_info.substring(8, 10).toInt() * GEAR_VAL;
}

void Shutter::populateRegisterForClosing() {
    for (int i = 0; i != register_count; ++i) {
        reg[i].motor[0].target_steps = STEPS_FOR_COMPLETE_SHUT1;
        reg[i].motor[1].target_steps = STEPS_FOR_COMPLETE_SHUT2;
    }
}

unsigned long Shutter::getRemainingSteps() {
    unsigned long steps = 0;
    for (int i = 0; i!= register_count; ++i) {
        for (int j = 0; j != 2; ++j)
            steps += reg[i].motor[j].target_steps - reg[i].motor[j].steps_taken;
    }
    return steps;
}


unsigned long Shutter::getRemainingStepsVerbose() {
    unsigned long steps = 0;
    for (int i = 0; i!= register_count; ++i) {

        Serial.println("remaining steps:");
        Serial.print("R");
        Serial.println(i);

        for (int j = 0; j != 2; ++j) {
            steps += reg[i].motor[j].target_steps - reg[i].motor[j].steps_taken;

            Serial.print("Motor");
            Serial.print(j + 1);
            Serial.print(": ");
            Serial.println(reg[i].motor[j].target_steps - reg[i].motor[j].steps_taken);
        }

    }
    return steps;
}

void Shutter::moveMotor() {
    unsigned long remaining_steps = getRemainingSteps();
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
    }
}

void Shutter::beginShutter() {
    resetSteps();
    for (int i = 0; i != register_count; ++i) {
        for (int j = 0; j != 2; ++j) {
            reg[i].motor[j].dir = 1;
            reg[i].motor[j].val = 1;
        }
    }
    moveMotor();
}

void Shutter::resetShutter() {
    resetSteps();
    unsigned long remaining_steps = getRemainingSteps();
    for (int i = 0; i != register_count; ++i) {
        for (int j = 0; j != 2; ++j) {
            reg[i].motor[j].dir = -1;
            reg[i].motor[j].val = 1;
        }
    }
    moveMotor();
}

void Shutter::closeShutter() {
    populateRegisterForClosing();
    unsigned long remaining_steps = getRemainingSteps();
    for (int i = 0; i != register_count; ++i) {
        for (int j = 0; j != 2; ++j) {
            reg[i].motor[j].dir = 1;
            reg[i].motor[j].val = reg[i].motor[j].steps_taken < reg[i].motor[j].target_steps ? 1 : 0;
        }
    }
    moveMotor();
}

int Shutter::getRegisterValue(const int reg_index) {
    return (reg[reg_index].motor[0].val << 4) + reg[reg_index].motor[1].val;
}

void Shutter::updateRegisterValue(const int reg_index) {
    for (int j = 0; j != 2; ++j) {
        if (reg[reg_index].motor[j].steps_taken < reg[reg_index].motor[j].target_steps) {
            if (reg[reg_index].motor[j].dir > 0)
                rol(reg[reg_index].motor[j].val, 4);
            else
                ror(reg[reg_index].motor[j].val, 4);
            reg[reg_index].motor[j].steps_taken++;
        } else {
            reg[reg_index].motor[j].val = 0;
        }
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
