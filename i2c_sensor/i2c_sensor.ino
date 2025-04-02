#include <Wire.h>
#include <Adafruit_TCS34725.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Variables for RGB readings
uint16_t r, g, b, c;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (tcs.begin()) {
    Serial.println("TCS34725 Robust White/Yellow Detection");
  } else {
    Serial.println("TCS34725 not found, check wiring!");
    while (1);
  }

  tcs.setIntegrationTime(TCS34725_INTEGRATIONTIME_154MS);  // Longer for better sensitivity
  tcs.setGain(TCS34725_GAIN_1X);  // Start with 1x gain, adjust if needed
}

void readColors() {
  uint32_t rSum = 0, gSum = 0, bSum = 0, cSum = 0;
  const int samples = 5;

  for (int i = 0; i < samples; i++) {
    tcs.getRawData(&r, &g, &b, &c);
    rSum += r;
    gSum += g;
    bSum += b;
    cSum += c;
    delay(50);  // Wait for integration
  }

  r = rSum / samples;
  g = gSum / samples;
  b = bSum / samples;
  c = cSum / samples;
}

void loop() {
  readColors();

  Serial.print("Red: ");
  Serial.print(r);
  Serial.print(" Green: ");
  Serial.print(g);
  Serial.print(" Blue: ");
  Serial.print(b);
  Serial.print(" Clear: ");
  Serial.println(c);

  // Robust detection
  // White: High balanced R, G, B (e.g., all > 1000, clear high)
  if (r > 1000 && g > 1000 && b > 1000 && abs(r - g) < 200 && abs(g - b) < 200 && abs(r - b) < 200) {
    Serial.println("Detected: White");
  }
  // Yellow: High R and G, low B (e.g., R, G > 800, B < 500)
  else if (r > 800 && g > 800 && b < 500 && r > b && g > b) {
    Serial.println("Detected: Yellow");
  } else {
    Serial.println("Unknown Color");
  }

  delay(1000);
}