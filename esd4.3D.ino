#include <RTCZero.h>
#include <DHT.h>

// Pin Definitions
const int buttonPin   = 2;  // Button → RED LED
const int sensor1Pin  = 3;  // Sensor → No LED  
const int sensor2Trig = 9;  // Ultrasonic TRIG
const int sensor2Echo = 4;  // Ultrasonic ECHO
const int dhtPin      = 5;  // DHT22 data
const int ledButton   = 6;  // RED LED for button
const int ledUltra    = 7;  // GREEN LED for ultrasonic
const int ledDHT      = 8;  // RED LED for DHT22

// DHT Setup
#define DHTTYPE DHT22
DHT dht(dhtPin, DHTTYPE);

// Globals
volatile bool buttonFlag = false;
volatile bool sensor1Flag = false;
volatile bool timerFlag = false;
bool ultraLedActive = false;
bool dhtLedActive = false;
unsigned long ultraLedTime = 0;
unsigned long dhtLedTime = 0;

// RTC Setup
RTCZero rtc;

void alarmMatch() {
  timerFlag = true;
}

void handleButton() {
  buttonFlag = true;
}

void handleSensor1() {
  sensor1Flag = true;
}

float readDistance() {
  digitalWrite(sensor2Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(sensor2Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensor2Trig, LOW);

  long duration = pulseIn(sensor2Echo, HIGH, 30000);
  if (duration == 0) return -1;
  return duration * 0.034 / 2;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(2000);
  
  // Pin modes
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(sensor1Pin, INPUT_PULLUP);
  pinMode(sensor2Trig, OUTPUT);
  pinMode(sensor2Echo, INPUT);
  pinMode(dhtPin, INPUT_PULLUP);
  pinMode(ledButton, OUTPUT);
  pinMode(ledUltra, OUTPUT);
  pinMode(ledDHT, OUTPUT);

  // Initialize
  digitalWrite(dhtPin, HIGH);
  dht.begin();
  digitalWrite(ledButton, LOW);
  digitalWrite(ledUltra, LOW);
  digitalWrite(ledDHT, LOW);
  digitalWrite(sensor2Trig, LOW);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(buttonPin), handleButton, FALLING);
  attachInterrupt(digitalPinToInterrupt(sensor1Pin), handleSensor1, FALLING);

  // Setup RTC timer
  rtc.begin();
  rtc.setTime(0, 0, 0);
  rtc.setAlarmSeconds(2);
  rtc.enableAlarm(rtc.MATCH_SS);
  rtc.attachInterrupt(alarmMatch);

  Serial.println("✅ SYSTEM READY!");
  Serial.println("RED LED (D6): Button toggle");
  Serial.println("GREEN LED (D7): Ultrasonic object detection"); 
  Serial.println("RED LED (D8): DHT22 reading indicator");
  Serial.println("=====================================");
}

void loop() {
  // Button interrupt - RED LED
  if (buttonFlag) {
    buttonFlag = false;
    digitalWrite(ledButton, !digitalRead(ledButton));
    Serial.println("Button pressed - RED LED toggled");
    delay(50);
  }

  // Sensor interrupt - No LED, just Serial message
  if (sensor1Flag) {
    sensor1Flag = false;
    Serial.println("Sensor triggered - No LED, only Serial message");
    delay(50);
  }

  // Ultrasonic LED control (blink for 1000ms only)
  if (ultraLedActive && millis() - ultraLedTime > 1000) {
    ultraLedActive = false;
    digitalWrite(ledUltra, LOW);
  }

  // DHT22 LED control (blink for 1000ms only)
  if (dhtLedActive && millis() - dhtLedTime > 1000) {
    dhtLedActive = false;
    digitalWrite(ledDHT, LOW);
  }

  // Timer interrupt - Sensor readings
  if (timerFlag) {
    timerFlag = false;
    
    Serial.println("--- Timer Interrupt (2s) ---");
    
    // ULTRASONIC SENSOR - Controls GREEN LED (D7)
    float dist = readDistance();
    if (dist == -1) {
      Serial.println("Ultrasonic: No object detected");
    } else {
      Serial.print("Ultrasonic: ");
      Serial.print(dist);
      Serial.println(" cm");
      
      if (dist > 0 && dist < 20) {
        digitalWrite(ledUltra, HIGH); // Turn ON GREEN LED
        ultraLedActive = true;
        ultraLedTime = millis();
        Serial.println("✅ Object detected - GREEN LED blinking");
      } else {
        Serial.println("❌ No object detected");
      }
    }

    // DHT22 SENSOR - Controls RED LED (D8)
    digitalWrite(ledDHT, HIGH); // Turn ON RED LED
    dhtLedActive = true;
    dhtLedTime = millis();
    
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    
    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("DHT22: Read error!");
    } else {
      Serial.print("DHT22: ");
      Serial.print(temperature);
      Serial.print("°C, ");
      Serial.print(humidity);
      Serial.println("% humidity");
      Serial.println("✅ DHT22 reading - RED LED blinking");
    }
    
    Serial.println("---------------------------");
    rtc.setAlarmSeconds((rtc.getSeconds() + 2) % 60);
  }

  delay(10);
}