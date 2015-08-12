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
#define GEAR_VAL 370
#define CENTRE_POSITION GEAR_VAL * (REGISTER_COUNT / 2)
#define ORIGIN CENTRE_POSITION

class Motor {
public:
    byte val = 0;
    unsigned int current_position = ORIGIN;
    unsigned int target_position = ORIGIN;
    char dir = 0;
};

class Register {
public:
    Motor motor[2];
};

class Shutter {
public:
    Shutter(const byte);
    void resetSteps();
    byte physicalToGUIMotorIndex(const byte);
    void populateRegister();
    void populateRegisterForClosing();
    void populateRegisterForOpening();
    bool hasRemainingSteps();
    unsigned long getRemainingStepsVerbose();
    void moveMotor();
    void beginShutter();
    void closeShutter();
    void openShutter();
    void hardReset();
    void forceOpen();
    void forceClose();
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

byte Shutter::physicalToGUIMotorIndex(byte physical_index) {
    // probably better not do a const byte to save memory
    byte physical_GUI_index_table[REGISTER_COUNT * 2];

    for (byte i = 0; i != REGISTER_COUNT / 2; ++i) {
        physical_GUI_index_table[i] = i * 2;
        physical_GUI_index_table[i + 16] = i * 2 + 1;
    }

    for (byte i = 0; i != REGISTER_COUNT / 2; ++i) {
        physical_GUI_index_table[i + REGISTER_COUNT] = (REGISTER_COUNT * 2) - 1 - (i * 2);
        physical_GUI_index_table[i + (REGISTER_COUNT / 2 * 3)] = (REGISTER_COUNT * 2) - 2 - (i * 2);
    }

    return physical_GUI_index_table[physical_index]; 
}

void Shutter::populateRegister() {
    char msg_buffer[11]; 
    byte GUI_motor_steps[REGISTER_COUNT * 2];

    // populate array directly with motor steps
    // first half of array contain steps of left side motors
    // other half of array contain steps of right side motors
    for (byte i = 0; i != register_count; ++i) {
        Serial.readBytes(msg_buffer, 11);
        String reg_info = String(msg_buffer);
        if (msg_buffer[0] == 'S' && msg_buffer[10] == 'E') {
            GUI_motor_steps[i] = reg_info.substring(5, 7).toInt();
            GUI_motor_steps[register_count + i] = reg_info.substring(8, 10).toInt();
        } else
            Serial.println("error");
    }

    /*
     *Serial.println("Motor steps:");
     *for (byte i = 0; i != REGISTER_COUNT * 2; ++i) {
     *    Serial.print("Motor ");
     *    Serial.print(i + 1);
     *    Serial.print(": ");
     *    Serial.println(GUI_motor_steps[i]);
     *}
     */

    /*
     *Serial.println("Physical to GUI motor index table:");
     *for (byte i = 0; i!= REGISTER_COUNT * 2; ++i) {
     *    Serial.print(i);
     *    Serial.print("\t");
     *    Serial.println(physicalToGUIMotorIndex(i));
     *}
     */

    // populate the individual register value base on the GUI_motor_steps array
    // taking into account the staggered motors
    
    //Serial.println("Motor target position:");
    for (byte i = 0; i != register_count; ++i) {
        /*
         *Serial.print("Motor ");
         *Serial.print(i * 2);
         *Serial.print(": ");
         */
        reg[i].motor[0].target_position = GUI_motor_steps[physicalToGUIMotorIndex(i * 2)] * GEAR_VAL;
        reg[i].motor[1].target_position = GUI_motor_steps[physicalToGUIMotorIndex(i * 2 + 1)] * GEAR_VAL;
        /*
         *Serial.println(reg[i].motor[0].target_position);
         *Serial.print("Motor ");
         *Serial.print(i * 2 + 1);
         *Serial.print(": ");
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
     *Serial.println("Closing stats:");
     *Serial.println("\tCurrent\tTarget:");
     *for (byte i = 0; i != register_count; ++i) {
     *    Serial.print("Motor ");
     *    Serial.print(i * 2);
     *    Serial.print(":\t");
     *    Serial.print(reg[i].motor[0].current_position);
     *    Serial.print("\t");
     *    Serial.println(reg[i].motor[0].target_position);
     *    Serial.print("Motor ");
     *    Serial.print(i * 2 + 1);
     *    Serial.print(":\t");
     *    Serial.print(reg[i].motor[1].current_position);
     *    Serial.print("\t");
     *    Serial.println(reg[i].motor[1].target_position);
     *}
     */


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

void Shutter::populateRegisterForOpening() {
    for (byte i = 0; i != register_count; ++i) {
        for (byte j = 0; j != 2; ++j) 
            reg[i].motor[j].target_position = 0;
    }
}    

void Shutter::beginShutter() {
    is_ready = false;
    for (byte i = 0; i != register_count; ++i) {
        for (byte j = 0; j != 2; ++j) {
            reg[i].motor[j].dir = reg[i].motor[j].current_position < reg[i].motor[j].target_position ? 1 : -1;
            // initialise only those motor that is not already at target position
            reg[i].motor[j].val = reg[i].motor[j].current_position != reg[i].motor[j].target_position ? 1 : 0;
        }
    }
    Serial.println("beginShutter");
    moveMotor();
    clearRegister(REGISTER_COUNT);
    is_ready = true;
    Serial.println("ready");
}

void Shutter::openShutter() {
    is_ready = false;
    populateRegisterForOpening();
    for (byte i = 0; i != register_count; ++i) {
        for (byte j = 0; j != 2; ++j) {
            reg[i].motor[j].dir = -1;
            reg[i].motor[j].val = 1;
        }
    }
    Serial.println("openShutter");
    moveMotor();
    clearRegister(REGISTER_COUNT);
    is_ready = true;
    Serial.println("ready");
}

void Shutter::closeShutter() {
    is_ready = false;
    populateRegisterForClosing();
    for (byte i = 0; i != register_count; ++i) {
        for (byte j = 0; j != 2; ++j) {
            reg[i].motor[j].dir = reg[i].motor[j].current_position < reg[i].motor[j].target_position ? 1 : -1;
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

void Shutter::forceOpen() {
    is_ready = false;

    // Set current position as centre position regardless of actual position
    for (byte i = 0; i != register_count; ++i) {
        for (byte j = 0; j != 2; ++j) {
            reg[i].motor[j].current_position = CENTRE_POSITION;
            reg[i].motor[j].target_position = 0;
        }
    }

    for (byte i = 0; i != register_count; ++i) {
        for (byte j = 0; j != 2; ++j) {
            reg[i].motor[j].dir = -1;
            reg[i].motor[j].val = 1;
        }
    }
    Serial.println("forceOpen");
    moveMotor();
    clearRegister(REGISTER_COUNT);
    is_ready = true;
    Serial.println("ready");
}

void Shutter::forceClose() {
    is_ready = false;

    // Set current position as centre position regardless of actual position
    for (byte i = 0; i != register_count; ++i) {
        for (byte j = 0; j != 2; ++j) {
            reg[i].motor[j].current_position = 0;
            reg[i].motor[j].target_position = CENTRE_POSITION;
        }
    }

    for (byte i = 0; i != register_count; ++i) {
        for (byte j = 0; j != 2; ++j) {
            reg[i].motor[j].dir = 1;
            reg[i].motor[j].val = 1;
        }
    }
    Serial.println("forceOpen");
    moveMotor();
    clearRegister(REGISTER_COUNT);
    is_ready = true;
    Serial.println("ready");
}


void Shutter::hardReset() {
    is_ready = false;

    // Reset current position to zero
    for (byte i = 0; i != register_count; ++i) {
        for (byte j = 0; j != 2; ++j) {
            reg[i].motor[j].current_position = ORIGIN;
            reg[i].motor[j].target_position = ORIGIN;
        }
    }

    Serial.println("Hard reset");
    clearRegister(REGISTER_COUNT);
    is_ready = true;
    Serial.println("ready");
}

void Shutter::moveMotor() {
    bool has_remaining_steps = hasRemainingSteps();
    while (has_remaining_steps) {
        if (Serial.available() > 0) {
            if (Serial.read() == 'S')
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
        has_remaining_steps = hasRemainingSteps();
    }
}

bool Shutter::hasRemainingSteps() {
    for (byte i = 0; i!= register_count; ++i) {
        if (reg[i].motor[0].val != 0 || reg[i].motor[1].val != 0)
            return true;
    }
    return false;
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
    /*
     *if (reg_index == 16) { // check to see if this is the correct register
     *    reg[reg_index].motor[0].val = 0;
     *    reg[reg_index].motor[1].val = 0;
     *}
     */
    //static bool stopped0 = false;
    //static bool stopped1 = false;
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
    /*
     *if (reg_index == 16) {
     *    if (!stopped0 && reg[reg_index].motor[0].val == 0) {
     *        stopped0 = true;
     *        Serial.print("motor");
     *        Serial.print(0);
     *        Serial.println(" stopped.");
     *    } else if (reg[reg_index].motor[0].val != 0) {
     *        stopped0 = false;
     *    }
     *    if (!stopped1 && reg[reg_index].motor[1].val == 0) {
     *        stopped1 = true;
     *        Serial.print("motor");
     *        Serial.print(1);
     *        Serial.println(" stopped.");
     *    } else if (reg[reg_index].motor[1].val != 0) {
     *        stopped1 = false;
     *    }
     *}
     */
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
            shutter.openShutter();
        } else if (serial_read == 'C') {
            shutter.closeShutter();
        } else if (serial_read == 'H') {
            shutter.hardReset();
        } else if (serial_read == 'F') {
            shutter.forceOpen();
        } else if (serial_read == 'X') {
            shutter.forceClose();
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

