#include <Arduino.h>

int tf;
int max_rad;
int max_pwm;
byte motor_write[100];
float motor_read[100];

int temp;

int PINO_ENCODER = 2;
int PINO_PWM = 6;
unsigned long pulseHigh;
unsigned long pulseLow;
float periodo;
int periodo_int[100];

// Variaveis do PID
int input = 0;
int lastInput = 0;
int erro = 0;
int output = 0;
double Kp = 1,
  Ki = 0.5,
  Kd = 4;

float I = 0;
float D = 0;
int c = 0;

void setup() {
  Serial.begin(9600);
  pinMode(PINO_ENCODER, INPUT);
  pinMode(PINO_PWM, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(4, HIGH);
  digitalWrite(5, LOW);
}

void recebe() {
  int w = 0;
  while (w < 1) {
    if (Serial.available() > 0) {
      tf = Serial.read();
      w++;
    }
  }

  w = 0;
  while (w < 1) {
    if (Serial.available() > 0) {
      max_rad = Serial.read();
      w++;
    }
  }

  w = 0;
  while (w < 1) {
    if (Serial.available() > 0) {
      max_pwm = Serial.read();
      w++;
    }
  }

  w = 0;
  while (w < 100) {
    if (Serial.available() > 0) {
      motor_write[w] = map(Serial.read(), 0, max_rad, 0, max_pwm);
      w++;
    }
  }
}

void loop() {
  recebe();
  for (int i = 0; i < 100; i++) {

    // Le o encoder
    pulseHigh = pulseIn(PINO_ENCODER, HIGH);
    pulseLow = pulseIn(PINO_ENCODER, LOW);
    motor_read[i] = pulseHigh + pulseLow;
    if (motor_read[i] > 0) {
      periodo = 1 / (motor_read[i]) * 1000000 * 3.14 / 180;
      periodo_int[i] = (int) periodo;
    } else if (motor_read == 0) {
      periodo_int[i] = 0;
    }

    if (i > 25) {
      I = 0;
    }

    c = 67 - i;
    if (c < 0) {
      c = 0;
    }

    input = map(periodo_int[i], 0, max_rad, 0, max_pwm);
    lastInput = input;
    if (motor_write[i] > input) {
      I += Ki * (motor_write[i] - input);
      D = (lastInput - input) * Kd;
      output = (motor_write[i] + Kp * (motor_write[i] - input) + I + D) + c;
    } else if (motor_write[i] < input) {
      I += Ki * (input - motor_write[i]);
      D = (input - lastInput) * Kd;
      output = (motor_write[i] - Kp * (input) - motor_write[i] - I - D) + c;
    } else if (motor_write[i] == input) {
      output = motor_write[i];
    }

    if (output >= max_pwm) {
      output = max_pwm;
    }
    if (output < 0) {
      output = motor_write[i];
    }

    // Saída do Controle
    analogWrite(PINO_PWM, output);
    Serial.write(periodo_int[i]);
    delay((tf * 1000) / 100);
  }

  recebe();

}