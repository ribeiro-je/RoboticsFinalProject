#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeAuriga.h>
#include <SPI.h>
#include <Pixy2.h>
#include <Pixy2I2C.h>

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeRGBLed rgbled_0(0, 12);
MeUltrasonicSensor ultrasonic_8(8);
Pixy2I2C pixy;

void isr_process_encoder1(void)
{
  if (digitalRead(Encoder_1.getPortB()) == 0) {
    Encoder_1.pulsePosMinus();
  } else {
    Encoder_1.pulsePosPlus();
  }
}
void isr_process_encoder2(void)
{
  if (digitalRead(Encoder_2.getPortB()) == 0) {
    Encoder_2.pulsePosMinus();
  } else {
    Encoder_2.pulsePosPlus();
  }
}

void _delay(float seconds) {
  if (seconds < 0.0) {
    seconds = 0.0;
  }
  long endTime = millis() + seconds * 1000;
  while (millis() < endTime) _loop();
}

void move(int direction, int speed)
{
  int leftSpeed = 0;
  int rightSpeed = 0;
  if (direction == 1) {
    leftSpeed = -speed;
    rightSpeed = speed;
  } else if (direction == 2) {
    leftSpeed = speed;
    rightSpeed = -speed;
  } else if (direction == 3) {
    leftSpeed = -speed;
    rightSpeed = -speed;
  } else if (direction == 4) {
    leftSpeed = speed;
    rightSpeed = speed;
  }
  Encoder_1.setTarPWM(leftSpeed);
  Encoder_2.setTarPWM(rightSpeed);
}

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");
  rgbled_0.setpin(44);
  rgbled_0.fillPixelsBak(0, 2, 1);
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);

  pixy.init();
}

void _loop() {
  Encoder_1.loop();
  Encoder_2.loop();
}

bool is_object_detected(int sig_num) {
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks)
  {
    for (int i = 0; i < pixy.ccc.numBlocks; i++)
    {
      if (pixy.ccc.blocks[i].m_signature == sig_num) {
        Serial.println("Detected");
        return true;
      }
    }
  }
  return false;
}

void loop()
{
  _loop();

  bool is_1_detected = is_object_detected(1);

  if (is_1_detected) {
    rgbled_0.setColor(0, 0, 93, 255);
    rgbled_0.show();
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
    _delay(10);
  }

  Serial.println(ultrasonic_8.distanceCm());
  if (ultrasonic_8.distanceCm() < 15) {
    rgbled_0.setColor(0, 255, 0, 0);
    rgbled_0.show();

    move(3, 50 / 100.0 * 255);
    _delay(1);
    move(3, 0);

  } else {
    rgbled_0.setColor(0, 174, 255, 0);
    rgbled_0.show();
    move(1, 70 / 100.0 * 255);
  }
}
