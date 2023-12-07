#define BAUDRATE 9600  // Serial port baud rate [Baud]
#define PERIOD 80      // Period of sampling [ms]
#define DEADZONE 50    // Deadzone of joystick axes [LSB]

// Codes to receive
#define TX_STOP 0   // Stops the transmission (acquire = 0)
#define TX_BEGIN 1  // Starts the transmission (acquire = 1)

// Codes to send
#define READY 0x0404  // Signals that the Arduino is ready to send data
#define DATA 0x0405   // Signals a packet of sensors' data
#define ERROR 0x0406  // Signals an error

typedef struct {
    int16_t x, y, z;
} joy_t;
typedef struct {
    uint16_t buttons;
    joy_t left, right;
} sensors_t;

void sendCode(uint16_t code);
void sendData();
int16_t deadzoneCenter(uint16_t value);

bool paused = true;
sensors_t data;
uint64_t previousMillis = 0, currentMillis;

/**
 * Function called when a byte is received on the Serial connection.
 * It is used to stop and resume sensors sampling
 */
void serialEvent() {
    int code;
    code = Serial.read();
    switch (code) {
        case TX_STOP:
            paused = true;
            break;
        case TX_BEGIN:
            paused = false;
            sendCode(READY);
            break;
        default:
            sendCode(ERROR);
            break;
    }
}

void setup() {
    // Digital input: buttons and switches
    pinMode(2, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);
    pinMode(4, INPUT_PULLUP);
    pinMode(5, INPUT_PULLUP);
    pinMode(6, INPUT_PULLUP);
    pinMode(8, INPUT_PULLUP);
    pinMode(9, INPUT_PULLUP);
    pinMode(10, INPUT_PULLUP);
    pinMode(11, INPUT_PULLUP);
    pinMode(12, INPUT_PULLUP);

    // Buzzer
    pinMode(13, OUTPUT);

    Serial.begin(BAUDRATE);
}

/**
 * Samples the sensors' values and sends them on the Serial connection
 * with the sampling period defined, while the acquisition is requested by
 * the computer.
 */
void loop() {
    if (paused) return;
    currentMillis = millis();
    if (currentMillis - previousMillis < PERIOD) return;

    previousMillis = currentMillis;

    // Buttons and switches (PIN 2-6, 8-12)
    data.buttons = digitalRead(2) << 9 | digitalRead(3) << 8 |
                   digitalRead(4) << 7 | digitalRead(5) << 6 |
                   digitalRead(6) << 5 | digitalRead(8) << 4 |
                   digitalRead(9) << 3 | digitalRead(10) << 2 |
                   digitalRead(11) << 1 | digitalRead(12);

    // Right joystick (PIN A0-A2)
    data.right.x = deadzoneCenter(analogRead(A0), false);
    data.right.y = deadzoneCenter(analogRead(A1), true);
    data.right.z = deadzoneCenter(analogRead(A2), false);

    // Left joystick (PIN A3-A5)
    data.left.x = deadzoneCenter(analogRead(A3), true);
    data.left.y = deadzoneCenter(analogRead(A4), false);
    data.left.z = deadzoneCenter(analogRead(A5), false);

    sendData();
}

/**
 * Sends the packet identifier to the Serial port
 *
 * @param code: the code to send (READY, DATA, ERROR)
 */
void sendCode(uint16_t code) { Serial.write((uint8_t*)&code, 2); }

/**
 * Rescales the values of joystick axes
 *
 * @param value: A joystick axis value read on 10bits
 * @param invert: inverts the joystick value, to account for 
 * symmetrical mounting of the joysticks
 * @returns The axis value rescaled in the [-512, 511] range,
 * with a deadzone of 50LSB
 */
int16_t deadzoneCenter(uint16_t value, bool invert) {
    if (invert) value = 1023 - value;
    if (value >= 512 - DEADZONE && value < 512 + DEADZONE) return 0;
    return value - 512;
}

/**
 * Sends the sensors' data to the serial port, located in the global variable
 * data.
 */
void sendData() {
    sendCode(DATA);
    Serial.write((uint8_t*)&data, sizeof(sensors_t));
}