#include <Servo.h>

#include <stdint.h>

int led_pin = 11;
int baud_rate = 115200;
int update_freq = 50;

const byte num_servos = 2;
int initial_delay = 1500;

Servo servo[num_servos];
uint16_t servo_delays[num_servos];

void setup()
{
    pinMode(led_pin, OUTPUT);
    Serial.begin(baud_rate);

    servo[0].attach(14);
    servo[1].attach(7);

    for (byte i = 0; i < num_servos; ++i)
    {
        servo_delays[i] = initial_delay;
    }
}

void loop()
{
    digitalWrite(led_pin, LOW);
    int in_bytes = Serial.available();
    if (in_bytes > 0)
    {
        uint16_t cmd;
        Serial.readBytes((char*)&cmd, 2);

        byte index = (cmd >> 12); // Select servo with 4 MSBs

        if (index < num_servos)
        {
            servo_delays[index] = (cmd & 0x0FFF);
            digitalWrite(led_pin, HIGH);
        }
        else if (index == 0xF) // 0xF wildcard selects all servos
        {
            for (byte i = 0; i < num_servos; ++i)
            {
                servo_delays[i] = (cmd & 0x0FFF);
            }
        }
    }
    for (byte i = 0; i < num_servos; ++i)
    {
        servo[i].writeMicroseconds(servo_delays[i]);
    }
    delay(1000/update_freq);
}