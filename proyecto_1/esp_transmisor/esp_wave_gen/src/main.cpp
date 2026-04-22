#include <Arduino.h>
#include <math.h>

const int DAC_PIN = 25;
const int ADC_PIN = 34;
const int N = 256;

uint8_t sineTable[N];
float samples[N];
int indexSine = 0;

void setup()
{
  Serial.begin(115200);
  delay(1000);

  for (int i = 0; i < N; i++)
  {
    float x = 2.0 * PI * i / N;
    sineTable[i] = (uint8_t)((sin(x) + 1.0) * 127.5);
  }

  Serial.println("Buffer capture start");
}

void loop()
{
  for (int i = 0; i < N; i++)
  {
    dacWrite(DAC_PIN, sineTable[indexSine]);
    delayMicroseconds(500);

    samples[i] = analogRead(ADC_PIN);

    indexSine++;
    if (indexSine >= N)
      indexSine = 0;
  }

  Serial.println("Captured block:");
  for (int i = 0; i < N; i++)
  {
    Serial.println(samples[i]);
  }

  Serial.println("----");
  delay(1000);
}