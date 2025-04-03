#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10);

const byte address[6] = "01001";

struct data {
  int gyro;
  bool start;
  bool solen;
};

volatile long lenc = 0, renc = 0;

void left_enc() {
  if (digitalRead(A3)) {
    lenc--;
  } else {
    lenc++;
  }
}

void right_enc() {
  if (digitalRead(A4)) {
    renc++;
  } else {
    renc--;
  }
}

void setMotors(int L, int R) {
  digitalWrite(4, L < 0);
  analogWrite(5, abs(constrain(L, -255, 255)));
  digitalWrite(7, R > 0);
  analogWrite(6, abs(constrain(R, -255, 255)));
}

void reset() {
  lenc = 0;
  renc = 0;
}

void stop() {
  reset();
  long t = millis() + 500;
  while (millis() < t) {
    setMotors(lenc * -32, renc * -32);
  }
  setMotors(0, 0);
}

void go_tick(int speed, long tick) {
  reset();
  while (lenc + renc < tick * 2) {
    int err = (lenc - renc) * 10;
    setMotors(speed - err, speed + err);
  }
  stop();
}

double mmToTick(int mm) {
  const int mmr = 250.0;
  const int tickr = 1000.0;
  return tickr * mm / mmr;
}

void go_dist(int speed, int mm) {
  reset();
  go_tick(speed, mmToTick(mm));
}

void turn_angle(int angle) {
  int v = mmToTick(angle * M_PI * 183.0 / 360.0);
  lenc = -v;
  renc = v;
  long t = millis() + 1000;
  while (millis() < t) {
    setMotors(lenc * -50, renc * -50);
    Serial.print(lenc);
    Serial.print(" ");
    Serial.println(renc);
  }
  stop();
}

void setup() {
  for (int i = 4; i <= 7; i++) {
    pinMode(i, OUTPUT);
  }
  Serial.begin(9600);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A3, INPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  attachInterrupt(0, left_enc, RISING);
  attachInterrupt(1, right_enc, RISING);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  setMotors(255, 255);
}

int vl;

void loop() {
  if (radio.available()) {
    data receivedData;
    radio.read(&receivedData, sizeof(receivedData));
    int v = receivedData.gyro;
    if (vl != v) {
      reset();
      vl = v;
    }
    if (v == 2) {
      int err = (lenc - renc) * 10;
      setMotors(100 - err, 100 + err);
    }
    if (v == 1) {
      int err = (abs(lenc) - abs(renc)) * 10;
      setMotors(-100 + err, 100 + err);
    }
    if (v == 3) {
      int err = (abs(lenc) - abs(renc)) * 10;
      setMotors(100 - err, -100 - err);
    }
  }
  Serial.print(analogRead(A0)); Serial.print(" "); Serial.println(analogRead(A1));
}
