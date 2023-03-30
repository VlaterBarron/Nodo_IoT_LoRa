#include <Arduino.h>
#include "SHT1x.h"
#include "src\lmic.h"
#include "src\hal\hal.h"

#define DataPin  P3_6
#define ClockPin  P3_5

SHT1x temp(DataPin,ClockPin);


void setup() {
  // put your setup code here, to run once:
  pinMode(DataPin,INPUT);
  pinMode(ClockPin,INPUT);
  Serial.begin(115200);
}

void loop() {
  int T = ((int)(temp.readTemperatureC()*100))/100;
  float H = temp.readHumidity();
  delay(1000);
  Serial.print("La temperatura es: ");
  Serial.print(T);
  Serial.println(" °C");
  Serial.print("La humedad es: ");
  Serial.println(H);
  // put your main code here, to run repeatedly:
}