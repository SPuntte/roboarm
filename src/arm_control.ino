/*
Robotic Arm Controller Arduino Sketch

Licenced under the MIT License:

Copyright (c) 2015 Pontus Lundström

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

// Disable debug output to serial
#define NDEBUG

#ifdef NDEBUG
#define DEBUG_OUTPUT_SERIAL(a)
#else
#define DEBUG_OUTPUT_SERIAL(a) a
#endif

#include <Wire.h> // Must include Wire library for I2C
#include <Servo.h>

#include <SFE_MMA8452Q.h> // Includes the SFE_MMA8452Q library

#include <stdint.h>
#include <math.h>

MMA8452Q accel;

// === Pin configuration =======================================================
// Flex sensor
const uint8_t flex_pin = A3;

// Locally controlled servos
Servo pitch_servo;
const uint8_t pitch_pin = 2;

Servo roll_servo;
const uint8_t roll_pin = 3;

Servo gripper_servo;
const uint8_t gripper_pin = 4;

// Remotely ("fly by serial") controlled servos
const uint8_t turntable_pin = 13;
const uint8_t shoulder_pin = 12;
const uint8_t elbow_pin = 11;

const uint8_t num_serial_servos = 3;
Servo serial_servos[num_serial_servos];

uint16_t servo_delays[num_serial_servos];
const uint16_t initial_delays[num_serial_servos] = {1500, 850, 2000};

const uint16_t serial_baud_rate = 57600;

// Logic frequency (Hz)
const int loop_frequency = 800;

// Initializes accelerometer
void initAccel(void)
{
    // Range:         +-2 g
    // Data rate: 100 Hz
    accel.init(SCALE_2G, ODR_100);
}

// Clamps value to [v_min, v_max]
float clamp(float value, float v_min, float v_max)
{
    return ((value < v_min) ? v_min : ((value > v_max) ? v_max : value));
}

// Drives wrist servos according to accelerometer data
void update_pitch_roll()
{
    float roll = atan2(accel.cz, accel.cx) * 180.0f / M_PI;
    float pitch = atan2(accel.cz, accel.cy) * 180.0f / M_PI;
    pitch_servo.write(clamp(180-pitch, 0, 180));
    roll_servo.write(clamp(180-roll, 40, 130));
}

// Gripper control with flex sensor
void update_gripper()
{
    uint16_t flex_value = analogRead(flex_pin);
    gripper_servo.writeMicroseconds(clamp(map(flex_value, 50, 500, 1130, 2100), 1130, 2100));
}

void update_serial_servos()
{
    int in_bytes = Serial.available();
    if (in_bytes >= 2)
    {
        uint16_t cmd = 0;
        Serial.readBytes((char*)&cmd, 2);

        byte index = (cmd >> 12); // Select servo with 4 MSBs

        if (index < num_serial_servos)
        {
            servo_delays[index] = (cmd & 0x0FFF);
        }
        else if (index == 0xF) // 0xF wildcard selects all servos
        {
            for (byte i = 0; i < num_serial_servos; ++i)
            {
                servo_delays[i] = (cmd & 0x0FFF);
            }
        }
    }
    for (byte i = 0; i < num_serial_servos; ++i)
    {
        serial_servos[i].writeMicroseconds(servo_delays[i]);
    }
}

void setup()
{
    Serial.begin(serial_baud_rate);

    initAccel();
    
    // Local servo setup
    roll_servo.attach(roll_pin);
    pitch_servo.attach(pitch_pin);
    gripper_servo.attach(gripper_pin);

    // Serial servo setup
    serial_servos[0].attach(turntable_pin);
    serial_servos[1].attach(shoulder_pin);
    serial_servos[2].attach(elbow_pin);
    for (byte i = 0; i < num_serial_servos; ++i)
    {
        servo_delays[i] = initial_delays[i];
    }
}

void loop()
{
    DEBUG_OUTPUT_SERIAL(Serial.println("Hello World."));
    
    if (accel.available())
    {
        DEBUG_OUTPUT_SERIAL(Serial.println("Reading accelerometer."));
        accel.read();
        DEBUG_OUTPUT_SERIAL(Serial.println("Got back safely."));
        
        update_pitch_roll();
    }
    
    update_gripper();
    update_serial_servos();
    
    DEBUG_OUTPUT_SERIAL(Serial.println("Still alive."));
    delay(1000/loop_frequency);
}
