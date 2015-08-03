#include "SPI.h"

#define ST_CP_PORT PORTB
#define SH_CP_PIN 13
#define ST_CP_PIN 10
#define DS_PIN 11

#define ST_CP_high() ST_CP_PORT |= _BV(ST_CP_PIN - 8)
#define ST_CP_low() ST_CP_PORT &= ~_BV(ST_CP_PIN - 8)

#define rol(val, bits) ((val << 1) | (val >> (bits - 1))) & ((1 << bits) - 1)
#define ror(val, bits) ((val >> 1) | (val << (bits - 1))) & ((1 << bits) - 1)

#define DELAY_VAL 100
#define REGISTER_COUNT 32
#define GEAR_VAL 400
#define CENTRE_POSITION GEAR_VAL * (REGISTER_COUNT / 2)

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
    void forceStop();
    void forceOpen();
    byte getRegisterValue(const byte);
    void updateRegisterValue(const byte);
    bool isReady();

private:

    byte register_count = 0;
    Register* reg;
    bool is_ready = true;
    bool force_stop = false;
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
            motor_steps[i] = reg_info.substring(5, 7).toInt();
            motor_steps[register_count + i] = reg_info.substring(8, 10).toInt();
        } else
            Serial.println("error");
    }

    /*
     *Serial.println("Motor steps:");
     *for (byte i = 0; i != REGISTER_COUNT * 2; ++i) {
     *    Serial.print("Motor ");
     *    Serial.print(i + 1);
     *    Serial.print(": ");
     *    Serial.println(motor_steps[i]);
     *}
     */

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
    
    //Serial.println("Motor target position:");
    for (byte i = 0; i != register_count; ++i) {
        /*
         *Serial.print("Register ");
         *Serial.print(i + 1);
         *Serial.print(": L");
         */
        reg[i].motor[0].target_position = motor_steps[corrected_motor_order[i * 2]] * GEAR_VAL;
        reg[i].motor[1].target_position = motor_steps[corrected_motor_order[i * 2 + 1]] * GEAR_VAL;
        /*
         *Serial.print(reg[i].motor[0].target_position);
         *Serial.print(" R");
         *Serial.println(reg[i].motor[1].target_position);
         */
    }
}


void Shutter::populateRegisterForClosing() {
    // It is possible that one side of the strip may be further in than the other.
    // In that case, we'll probably just want to close in the strip that is further away.
    byte opposite_end = (register_count / 2);
    for (byte i = 0; i != opposite_end; ++i) {
        for (byte j = 0; j != 2; ++j) {
            byte moving_strip = 0, stationary_strip = -1;
            // checks which shutter has passed the centre position
            if (reg[i].motor[j].current_position >= CENTRE_POSITION || reg[i + opposite_end].motor[j].current_position >= CENTRE_POSITION) {
                moving_strip = reg[i].motor[j].current_position >= CENTRE_POSITION ? opposite_end : 0;
                stationary_strip = 1;
            }

            byte other_strip = moving_strip == 0 ? opposite_end : 0;

            if (stationary_strip == 1) {
                // if one of them is going to be station, move the other up to the remaining distance
                reg[i + moving_strip].motor[j].target_position = GEAR_VAL * register_count - reg[i + other_strip].motor[j].target_position;
            }
            else {
                reg[i + moving_strip].motor[j].target_position = CENTRE_POSITION;
                reg[i + other_strip].motor[j].target_position = CENTRE_POSITION;
            }
        }
    }
/*
 *    Serial.println("in populateRegisterForClosing:");
 *    
 *    for (byte i = 0; i != register_count; ++i) {
 *        Serial.print("Target position of register ");
 *        Serial.print(i + 1);
 *        Serial.print(": L");
 *        Serial.print(reg[i].motor[0].target_position);
 *        Serial.print(" R");
 *        Serial.println(reg[i].motor[1].target_position);
 *
 *        Serial.print("Current position of register ");
 *        Serial.print(i + 1);
 *        Serial.print(": L");
 *        Serial.print(reg[i].motor[0].current_position);
 *        Serial.print(" R");
 *        Serial.println(reg[i].motor[1].current_position);
 *
 *    }
 */
}

void Shutter::populateRegisterForReset() {
    for (byte i = 0; i != register_count; ++i) {
        for (byte j = 0; j != 2; ++j) 
            reg[i].motor[j].target_position = 0;
    }
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
            // initialise only those motor that is not already at target position
            reg[i].motor[j].val = reg[i].motor[j].current_position != reg[i].motor[j].target_position ? 1 : 0;
        }
    }
    Serial.println("closeShutter");
    moveMotor();
    clearRegister(REGISTER_COUNT);
    is_ready = true;
    Serial.println("ready");
}

void Shutter::forceStop() {
    force_stop = true;
}

void Shutter::forceOpen() {
    is_ready = false;

    // Set current position as centre position regardless of actual position
    for (byte i = 0; i != register_count; ++i) {
        for (byte j = 0; j != 2; ++j) 
            reg[i].motor[j].current_position = CENTRE_POSITION;
            reg[i].motor[j].target_position = 0;
    }

    unsigned long remaining_steps = getRemainingSteps();
    for (byte i = 0; i != register_count; ++i) {
        for (byte j = 0; j != 2; ++j) {
            reg[i].motor[j].dir = 0;
            reg[i].motor[j].val = 1;
        }
    }
    Serial.println("forceOpen");
    moveMotor();
    clearRegister(REGISTER_COUNT);
    is_ready = true;
    Serial.println("ready");
}

void Shutter::moveMotor() {
    unsigned long remaining_steps = getRemainingSteps();
    while (remaining_steps > 0) {
        if (force_stop) {
            force_stop = false;
            break;
        }
        ST_CP_low();
        for (byte i = 0; i != register_count; ++i) {
            SPI.transfer(getRegisterValue(i));
            //Serial.println(getRegisterValue(i), BIN);
            updateRegisterValue(i);
            delayMicroseconds(DELAY_VAL);
        }
        ST_CP_high();
        remaining_steps = getRemainingSteps();
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
        } else if (serial_read == 'S') {
            shutter.forceStop();
        } else if (serial_read == 'F') {
            shutter.forceOpen();
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

