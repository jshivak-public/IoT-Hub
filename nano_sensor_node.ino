#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>


//OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


//BMP280
Adafruit_BMP280 bmp;


//Pins
#define MQ_PIN   A0
#define TRIG_PIN 9
#define ECHO_PIN 2


//HC-SR04
long readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);


  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
  if (duration == 0) return -1;


  long cm = (long)(duration * 0.0343 / 2.0);
  return cm;
}


void setup() {
  Serial.begin(9600);
  Wire.begin();


  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);


  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
    while (1) {}
  }


  // BMP init (try both common addresses)
  bool bmp_ok = bmp.begin(0x76);
  if (!bmp_ok) bmp_ok = bmp.begin(0x77);


  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);


  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("BOOT");


  display.setTextSize(1);
  display.print("BMP: ");
  display.println(bmp_ok ? "OK" : "FAIL");


  display.display();
  delay(800);
}


void loop() {
  // Read sensors
  int mq_raw = analogRead(MQ_PIN);
  long dist_cm = readDistanceCm();


  float temp_c = bmp.readTemperature();
  float press_pa = bmp.readPressure();


  // Serial log too
  Serial.print("MQ=");
  Serial.print(mq_raw);
  Serial.print(" DIST_CM=");
  Serial.print(dist_cm);
  Serial.print(" TEMP_C=");
  Serial.print(temp_c, 1);
  Serial.print(" PRESS_PA=");
  Serial.println(press_pa, 0);


  // OLED display layout for 1.3 inch
  display.clearDisplay();


  // Top line: distance big
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("D:");
  if (dist_cm < 0) {
    display.print("--");
  } else {
    display.print(dist_cm);
  }
  display.print("cm");


  // Bottom lines: MQ, Temp, Pressure small
  display.setTextSize(1);


  display.setCursor(0, 24);
  display.print("MQ:");
  display.print(mq_raw);


  display.setCursor(0, 36);
  display.print("T:");
  display.print(temp_c, 1);
  display.print("C");


  display.setCursor(0, 48);
  display.print("P:");
  display.print(press_pa / 100.0, 1);
  display.print("hPa");


  display.display();


  delay(500);
}
