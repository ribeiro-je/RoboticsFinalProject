#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeAuriga.h>
#include <SPI.h>
#include <Pixy2.h>
#include <Pixy2I2C.h>
#include <SoftwareSerial.h>

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeRGBLed rgbled_0(0, 12);
MeUltrasonicSensor ultrasonic_8(9);

MeBuzzer buzzer;
Pixy2I2C pixy;

double pos_t = 0.0;
double x = 0.0;
double y = 0.0;

long randNumber;
int count = 1;

const double STEP = 0.1;
const double THETA_INCR = 0.1;

bool was_1_detected = false;
bool was_2_detected = false;
bool was_3_detected = false;
bool was_4_detected = false;


void isr_process_encoder1(void)
{
  if (digitalRead(Encoder_1.getPortB()) == 0)
  {
    Encoder_1.pulsePosMinus();
  }
  else
  {
    Encoder_1.pulsePosPlus();
  }
}
void isr_process_encoder2(void)
{
  if (digitalRead(Encoder_2.getPortB()) == 0)
  {
    Encoder_2.pulsePosMinus();
  }
  else
  {
    Encoder_2.pulsePosPlus();
  }
}

void _delay(float seconds)
{
  if (seconds < 0.0)
  {
    seconds = 0.0;
  }
  long endTime = millis() + seconds * 1000;
  while (millis() < endTime)
    _loop();
}

void set_x_y_pos() {

  if (pos_t < 1.8 && pos_t > 1.2 ) {
    y += STEP;
  }
  else if (pos_t > -1.8 && pos_t < -1.2) {
    y -= STEP;
  }
  else if (pos_t < 0.3 && pos_t > -0.3) {
    x += STEP;
  }
  else if (pos_t > 2.8 || pos_t < -2.8) {
    x -= STEP;
  }
  /// --------
  //  else if (pos_t < 1.05 && pos_t > 0.45) {
  //    x += STEP;
  //    y += STEP;
  //  }
  //  else if (pos_t < -0.45 && pos_t > -1.05) {
  //    x += STEP;
  //    y -= STEP;
  //  }
  //  else if (pos_t < -2.62 && pos_t > -2.02) {
  //    x -= STEP;
  //    y -= STEP;
  //  }
  //  else if (pos_t < 2.62 && pos_t > 2.02) {
  //    x -= STEP;
  //    y += STEP;
  //  }
}

void move(int direction, int speed)
{
  int leftSpeed = 0;
  int rightSpeed = 0;
  if (direction == 1)
  {
    leftSpeed = -speed;
    rightSpeed = speed;
  }
  else if (direction == 2)
  {
    leftSpeed = speed;
    rightSpeed = -speed;
  }
  else if (direction == 3)
  {
    pos_t += THETA_INCR;
    if (pos_t > 3.14) {
      pos_t = fmod(pos_t, 3.14) - 3.14;
    }
    leftSpeed = -speed;
    rightSpeed = -speed;
  }
  else if (direction == 4)
  {
    pos_t -= THETA_INCR;
    if (pos_t < -3.14) {
      pos_t = fmod(pos_t, -3.14) + 3.14;
    }
    leftSpeed = speed;
    rightSpeed = speed;
  }
  Encoder_1.setTarPWM(leftSpeed);
  Encoder_2.setTarPWM(rightSpeed);

  set_x_y_pos();
}

void setup()
{
  Serial.begin(9600);
  Serial.print("Starting...\n");
  rgbled_0.setpin(44);
  rgbled_0.fillPixelsBak(0, 2, 1);
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  buzzer.setpin(45);
  randomSeed(analogRead(0));

  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);

  pos_t = 0.0;
  pixy.init();
}

void _loop()
{
  Encoder_1.loop();
  Encoder_2.loop();
}

bool is_object_detected(int sign)
{
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks)
  {
    for (int i = 0; i < pixy.ccc.numBlocks; i++)
    {
      if (pixy.ccc.blocks[i].m_signature == sign)
      {
        return true;
      }
    }
  }
  return false;
}

void process_data() {

  Serial.print("d");
  Serial.println(ultrasonic_8.distanceCm());
  Serial.print("p");
  Serial.println(pos_t);
  Serial.print("x");
  Serial.println(x);
  Serial.print("y");
  Serial.println(y);

  Serial.print("1");
  bool is_1_detected = is_object_detected(1);
  Serial.println(is_1_detected);
  Serial.print("2");
  bool is_2_detected = is_object_detected(2);
  Serial.println(is_2_detected);
  Serial.print("3");
  bool is_3_detected = is_object_detected(3);
  Serial.println(is_3_detected);
  Serial.print("4");
  bool is_4_detected = is_object_detected(4);
  Serial.println(is_4_detected);

  if (is_1_detected && !was_1_detected)
  {
    was_1_detected = true;
    rgbled_0.setColor(0, 0, 93, 255);
    rgbled_0.show();
    buzzer.tone(100, 1 * 1000);
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
    _delay(1);
  }
  else if (is_2_detected && !was_2_detected)
  {
    was_2_detected = true;
    rgbled_0.setColor(0, 255, 0, 0);
    rgbled_0.show();
    buzzer.tone(300, 1 * 1000);
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
    _delay(1);
  }
  else if (is_3_detected && !was_3_detected)
  {
    was_3_detected = true;
    rgbled_0.setColor(0, 146, 73, 0);
    rgbled_0.show();
    buzzer.tone(500, 1 * 1000);
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
    _delay(1);
  }
  else if (is_4_detected && !was_4_detected)
  {
    was_4_detected = true;
    rgbled_0.setColor(0, 72, 243, 0);
    rgbled_0.show();
    buzzer.tone(700, 1 * 1000);
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
    _delay(1);
  }
  else {
    rgbled_0.setColor(0, 0, 0, 0);
    rgbled_0.show();
  }
}

void loop()
{
   randNumber = random(0, 1);
  if (ultrasonic_8.distanceCm() < 25)
  {
    if (randNumber == 0.0) {
      move(3, 55 / 100.0 * 255);
    }
    else {
      move(4, 55 / 100.0 * 255);
    }
  }

  else
  {
    move(1, 60 / 100.0 * 255);
  }

  _loop();

  count += 1;
  process_data();
}
