#include <Servo.h>

// Arduino pin assignment
#define PIN_IR    A0
#define PIN_LED   9
#define PIN_SERVO 10

// Servo duty limits
#define _DUTY_MIN 500
#define _DUTY_NEU 1500
#define _DUTY_MAX 2500

// Distance limits (mm)
#define _DIST_MIN 100.0   // 10 cm
#define _DIST_MAX 250.0   // 25 cm

// Hysteresis margin (mm)
#define _DIST_MARGIN 20.0  // 

// EMA filter coefficient (0~1)
#define EMA_ALPHA 0.10    

// Loop interval (ms)
#define LOOP_INTERVAL 20

Servo myservo;
unsigned long last_loop_time = 0;
float dist_ema = _DIST_MIN;
bool led_on = false;  // 

void setup()
{
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_IR, INPUT);
  
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);
  
  Serial.begin(1000000);
}

void loop()
{
  unsigned long time_curr = millis();
  if (time_curr < (last_loop_time + LOOP_INTERVAL))
    return;
  last_loop_time += LOOP_INTERVAL;

 
  float a_value = analogRead(PIN_IR);

  
  float dist_raw = (6762.0 / (a_value - 9.0) - 4.0) * 10.0 - 60.0;
  if (dist_raw < 50.0) dist_raw = 50.0;
  if (dist_raw > 2000.0) dist_raw = 2000.0;

  
  dist_ema = EMA_ALPHA * dist_raw + (1.0 - EMA_ALPHA) * dist_ema;

  
  if (!led_on && dist_ema >= _DIST_MIN && dist_ema <= _DIST_MAX)
    led_on = true;   
  else if (led_on && (dist_ema < (_DIST_MIN - _DIST_MARGIN) || dist_ema > (_DIST_MAX + _DIST_MARGIN)))
    led_on = false; 

  digitalWrite(PIN_LED, led_on ? HIGH : LOW);

  
  int duty;
  if (dist_ema < _DIST_MIN) duty = _DUTY_MIN;
  else if (dist_ema > _DIST_MAX) duty = _DUTY_MAX;
  else {
    float t = (dist_ema - _DIST_MIN) / (_DIST_MAX - _DIST_MIN);
    duty = (int)(_DUTY_MIN + t * (_DUTY_MAX - _DUTY_MIN));
  }

  myservo.writeMicroseconds(duty);

  
  Serial.print("IR:");        Serial.print(a_value);
  Serial.print(", dist_raw:");Serial.print(dist_raw);
  Serial.print(", ema:");     Serial.print(dist_ema);
  Serial.print(", LED:");     Serial.print(led_on ? 1 : 0);
  Serial.print(", servo:");   Serial.println(duty);
}
