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
#define REGISTER_COUNT 32
#define GEAR_VAL 400
#define MOTOR1_GRID_COUNT 2
#define MOTOR2_GRID_COUNT 3
#define STEPS_FOR_COMPLETE_SHUT1 GEAR_VAL * MOTOR1_GRID_COUNT
#define STEPS_FOR_COMPLETE_SHUT2 GEAR_VAL * MOTOR2_GRID_COUNT
#define CENTRE_POSITION(val) GEAR_VAL * ((REGISTER_COUNT / 2) + val)

class Motor {
public:
    Motor() {};
    byte val = 0;
    unsigned long current_position = 0;
    unsigned long target_position = 0;
    char dir = 0;
};

class Register {
public:
    Register() {};
    Motor motor[2];
};

class Shutter {
public:
    Shutter(const byte);
    void resetSteps();
    void populateRegister();
    void populateRegisterForClosing();
    void populateRegisterForReset();
    unsigned long getRemainingSteps();
    unsigned long getRemainingStepsVerbose();
    void moveMotor();
    void beginShutter();
    void closeShutter();
    void resetShutter();
    byte getRegisterValue(const byte);
    void updateRegisterValue(const byte);
    bool isReady();

private:

    byte register_count = 0;
    Register* reg;
    bool is_ready = true;
};


Shutter::Shutter(const byte reg_count) {
    register_count = reg_count;
    reg = new Register[reg_count];
}

void Shutter::resetSteps() {
    for (byte i = 0; i != register_count; ++i) {
        for (byte j = 0; j != 2; ++j)
            reg[i].motor[j].current_position = 0;
    }
}

void Shutter::populateRegister() {
    char msg_buffer[11]; 
    byte motor_steps[REGISTER_COUNT * 2];

    // populate array directly with motor steps
    // first half of array contain steps of left side motors
    // other half of array contain steps of right side motors
    for (byte i = 0; i != register_count; ++i) {
        Serial.readBytes(msg_buffer, 11);
        String reg_info = String(msg_buffer);
        if (msg_buffer[0] == 'S' && msg_buffer[10] == 'E') {
            motor_steps[i * 2] = reg_info.substring(5, 7).toInt();
            motor_steps[i * 2 + 1] = reg_info.substring(8, 10).toInt();
        } else
            Serial.println("error");
    }

    // rearrange motor orders
    byte first_motor_index_of_PCB[] = {0, 1, register_count, register_count + 1};
    byte corrected_motor_order[REGISTER_COUNT * 2];

    // looping through for 4 PCBs
    byte k = 0;
    for (byte i = 0; i != 4; ++i) {
        for (byte j = 0; j != register_count / 2; ++j) {
            corrected_motor_order[k] = first_motor_index_of_PCB[i] + 2 * j;
            ++k;
        }
    }

    // populate the individual register value base on the motor_steps array
    // taking into account the staggered motors
    for (byte i = 0; i != register_count; ++i) {
        reg[i].motor[0].target_position = motor_steps[corrected_motor_order[i * 2]] * GEAR_VAL;
        reg[i].motor[1].target_position = motor_steps[corrected_motor_order[i * 2 + 1]] * GEAR_VAL;
    }
}

void Shutter::populateRegisterForClosing() {
    // It is possible that one side of the strip may be further in than the other.
    // In that case, we'll probably just want to close in the strip that is further away.
    for (byte i = 0; i != register_count; ++i) {
        byte moving_strip = 0, stationary_strip = -1;
        if (reg[i].motor[0].target_position >= CENTRE_POSITION(0) || reg[i].motor[1].target_position >= CENTRE_POSITION(1)) {
            // center_pos may be different in odd numbered sides
            moving_strip = reg[i].motor[0].target_position >= CENTRE_POSITION(0) ? 1 : 0;
            stationary_strip = !moving_strip;
        }
        // (moving_strip * GEAR_VAL): fix for odd numbered sides
        reg[i].motor[moving_strip].target_position = CENTRE_POSITION(0) + (moving_strip * GEAR_VAL);

        if (stationary_strip == -1)
            reg[i].motor[!moving_strip].target_position = CENTRE_POSITION(0) + (!moving_strip * GEAR_VAL);
    }
}

void Shutter::populateRegisterForReset() {
    for (byte i = 0; i != register_count; ++i) {
        for (byte j = 0; j != 2; ++j) 
            reg[i].motor[j].target_position = 0;
    }
}    

unsigned long Shutter::getRemainingSteps() {
    unsigned long steps = 0;
    for (byte i = 0; i!= register_count; ++i) {
        for (byte j = 0; j != 2; ++j)
            steps += abs(reg[i].motor[j].target_position - reg[i].motor[j].current_position);
    }
    return steps;
}

unsigned long Shutter::getRemainingStepsVerbose() {
    unsigned long steps = 0;
    for (byte i = 0; i!= register_count; ++i) {

        Serial.println("remaining steps:");
        Serial.print("R");
        Serial.println(i);

        for (byte j = 0; j != 2; ++j) {
            steps += abs(reg[i].motor[j].target_position - reg[i].motor[j].current_position);

            Serial.print("Motor");
            Serial.print(j + 1);
            Serial.print(": ");
            Serial.println(abs(reg[i].motor[j].target_position - reg[i].motor[j].current_position));
        }

    }
    return steps;
}

void Shutter::beginShutter() {
    is_ready = false;
    for (byte i = 0; i != register_count; ++i) {
        for (byte j = 0; j != 2; ++j) {
            if (reg[i].motor[j].target_position != reg[i].motor[j].current_position) {
                reg[i].motor[j].dir = 1;
                if (reg[i].motor[j].target_position < reg[i].motor[j].current_position) {
                    reg[i].motor[j].dir = -1;
                }
                reg[i].motor[j].val = 1;
            } else {
                reg[i].motor[j].val = 0;
            }
        }
    }
    Serial.println("beginShutter");
    moveMotor();
    clearRegister(REGISTER_COUNT);
    is_ready = true;
    Serial.println("ready");
}

void Shutter::resetShutter() {
    is_ready = false;
    populateRegisterForReset();
    unsigned long remaining_steps = getRemainingSteps();
    for (byte i = 0; i != register_count; ++i) {
        for (byte j = 0; j != 2; ++j) {
            reg[i].motor[j].dir = -1;
            reg[i].motor[j].val = 1;
        }
    }
    Serial.println("resetShutter");
    moveMotor();
    clearRegister(REGISTER_COUNT);
    is_ready = true;
    Serial.println("ready");
}

void Shutter::closeShutter() {
    is_ready = false;
    populateRegisterForClosing();
    unsigned long remaining_steps = getRemainingSteps();
    for (byte i = 0; i != register_count; ++i) {
        for (byte j = 0; j != 2; ++j) {
            reg[i].motor[j].dir = 1;
            reg[i].motor[j].val = reg[i].motor[j].current_position != reg[i].motor[j].target_position ? 1 : 0;
        }
    }
    Serial.println("closeShutter");
    moveMotor();
    clearRegister(REGISTER_COUNT);
    is_ready = true;
    Serial.println("ready");
}

void Shutter::moveMotor() {
    unsigned long remaining_steps = getRemainingSteps();
    while (remaining_steps > 0) {
        ST_CP_low();
        for (byte i = 0; i != register_count; ++i) {
            SPI.transfer(getRegisterValue(i));
            //Serial.println(getRegisterValue(i), BIN);
            updateRegisterValue(i);
            delay(DELAY_VAL);
        }
        ST_CP_high();
        remaining_steps = getRemainingSteps();
    }
}

byte Shutter::getRegisterValue(const byte reg_index) {
    return (reg[reg_index].motor[0].val << 4) + reg[reg_index].motor[1].val;
}

void Shutter::updateRegisterValue(const byte reg_index) {
    for (byte j = 0; j != 2; ++j) {
        if (reg[reg_index].motor[j].current_position != reg[reg_index].motor[j].target_position) {
            if (reg[reg_index].motor[j].dir > 0)
                reg[reg_index].motor[j].val = ror(reg[reg_index].motor[j].val, 4);
            else
                reg[reg_index].motor[j].val = rol(reg[reg_index].motor[j].val, 4);
            reg[reg_index].motor[j].current_position += reg[reg_index].motor[j].dir;
        } else {
            reg[reg_index].motor[j].val = 0;
        }
    }
}

bool Shutter::isReady() {
    return is_ready;
}

Shutter shutter(REGISTER_COUNT);

void setup() {
    Serial.begin(9600);
    SPI.begin();
    pinMode(SH_CP_PIN, OUTPUT);
    pinMode(ST_CP_PIN, OUTPUT);
    pinMode(DS_PIN, OUTPUT);
    clearRegister(REGISTER_COUNT);
    Serial.println("ready");
}

char serial_read = '0';

void loop() {
    if (Serial.available() > 0) {
        serial_read = Serial.read();
        if (serial_read == 'B') {
            shutter.populateRegister();
            shutter.beginShutter();
        } else if (serial_read == 'O') {
            shutter.resetShutter();
        } else if (serial_read == 'C') {
            shutter.closeShutter();
        } else if (serial_read == 'R') {
            if (shutter.isReady())
                Serial.println("ready");
        }
    }
}

void clearRegister(const byte& register_count) {
    for (byte i = 0; i != register_count; ++i)
        SPI.transfer(0);
    ST_CP_low();
    ST_CP_high();
}

