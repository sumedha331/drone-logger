#include <HX711_ADC.h>
#include <LiquidCrystal.h>
#include <Servo.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

// HX711 pins
const int HX711_dout = 3;
const int HX711_sck = 4;
HX711_ADC LoadCell(HX711_dout, HX711_sck);

// LCD pins (RS, EN, D4, D5, D6, D7)
LiquidCrystal lcd(A2, A1, 9, 10, 12, 11);

// ACS712 pin
const int CURRENT_SENSOR_PIN = A3;
float currentSensorOffset = 2.5;  // default offset in volts

// ESC signal pin
const int ESC_PIN = 5;
Servo esc;

// Keypad pins
const int programPin = 2;
const int incPin = 6;
const int decPin = 7;
const int enterPin = 8;

int mode = 1;  // 1 = current limit, 2 = PWM limit
int programPresses = 0;
float userMaxCurrent = 0.0;
int userMaxPWM = 1000;
int pwmStart = 800;

unsigned long t = 0;
bool configDone = false;
bool hasEntered = false;

// === EEPROM helpers ===
#define EEPROM_FLAG_ADDR 12
#define EEPROM_VALID_FLAG 0xA5

void saveToEEPROM() {
  EEPROM.put(0, mode);
  EEPROM.put(4, userMaxCurrent);
  EEPROM.put(8, userMaxPWM);
  EEPROM.update(EEPROM_FLAG_ADDR, EEPROM_VALID_FLAG);
}

bool loadFromEEPROM() {
  byte flag = EEPROM.read(EEPROM_FLAG_ADDR);
  if (flag == EEPROM_VALID_FLAG) {
    EEPROM.get(0, mode);
    EEPROM.get(4, userMaxCurrent);
    EEPROM.get(8, userMaxPWM);
    return true;
  }
  return false;
}

void waitForEnterToStart() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Baud Rate : 9600");
  lcd.setCursor(0, 1);
  lcd.print("StartBit:1 StopBit:1");
  lcd.setCursor(0, 2);
  lcd.print("Parity: No");
  lcd.setCursor(0, 3);
  lcd.print("Press ENTER to start");

  while (true) {
    if (digitalRead(enterPin) == LOW) {
      delay(200);
      while (digitalRead(enterPin) == LOW);
      break;
    }
  }

  // Set start PWM
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Set Start PWM");
  lcd.setCursor(0, 1);
  lcd.print("PWM: ");
  lcd.print(pwmStart);
  lcd.print(" us");

  bool incPressed = false, decPressed = false, enterPressed = false;

  while (true) {
    if (digitalRead(incPin) == LOW && !incPressed) {
      pwmStart += 5;
      lcd.setCursor(0, 1);
      lcd.print("PWM: ");
      lcd.print(pwmStart);
      lcd.print(" us  ");
      incPressed = true;
      delay(200);
    }
    if (digitalRead(incPin) == HIGH) incPressed = false;

    if (digitalRead(decPin) == LOW && !decPressed) {
      if (pwmStart > 500) pwmStart -= 5;
      lcd.setCursor(0, 1);
      lcd.print("PWM: ");
      lcd.print(pwmStart);
      lcd.print(" us  ");
      decPressed = true;
      delay(200);
    }
    if (digitalRead(decPin) == HIGH) decPressed = false;

    if (digitalRead(enterPin) == LOW && !enterPressed) {
      delay(200);
      while (digitalRead(enterPin) == LOW);
      break;
    }
  }
}

float measureCurrentSensorOffset() {
  const int samples = 200;
  int readings[samples];
  
  delay(1000); // Allow sensor to stabilize

  for (int i = 0; i < samples; i++) {
    readings[i] = analogRead(CURRENT_SENSOR_PIN);
    delay(2);
  }

  // Sort the readings for median
  for (int i = 0; i < samples - 1; i++) {
    for (int j = 0; j < samples - i - 1; j++) {
      if (readings[j] > readings[j + 1]) {
        int temp = readings[j];
        readings[j] = readings[j + 1];
        readings[j + 1] = temp;
      }
    }
  }

  int medianADC = readings[samples / 2];
  float voltage = (medianADC / 1023.0) * 5.0;
  return voltage;
}

void setup() {
  Serial.begin(9600);
  delay(10);

  pinMode(programPin, INPUT_PULLUP);
  pinMode(incPin, INPUT_PULLUP);
  pinMode(decPin, INPUT_PULLUP);
  pinMode(enterPin, INPUT_PULLUP);

  lcd.begin(20, 4);
  waitForEnterToStart();

  lcd.clear();
  lcd.print("Initializing...");

  currentSensorOffset = measureCurrentSensorOffset();
  Serial.print("Current Sensor Offset: ");
  Serial.println(currentSensorOffset, 4);

  if (loadFromEEPROM()) {
    Serial.println("EEPROM contains valid configuration. Skipping manual setup.");
    configDone = true;
  }

  LoadCell.begin();
  float calibrationValue = -1328.05;
  LoadCell.start(2000, true);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("HX711 Timeout");
    while (1);
  }
  LoadCell.setCalFactor(calibrationValue);
  Serial.println("Startup is complete");

  esc.attach(ESC_PIN);
  esc.writeMicroseconds(800);

  if (!configDone) {
    Serial.println("Use keypad to configure mode and limit.");
    lcd.clear();
    lcd.print("Select Mode...");
  }
}

void loop() {
  static boolean newDataReady = false;
  const int sampleSize = 50;
  const float deadbandThreshold = 0.05;
  const int currentSamplesPerCycle = 5;
  static float currentSquareSum = 0;
  static int currentCount = 0;

  static int pwmValue = 800;
  static bool active = true;
  static unsigned long lastStepTime = 0;
  static bool holdPhase = false;
  static bool decelerationPhase = false;
  static unsigned long holdStartTime = 0;
  static float latestCurrent = 0;

  static bool programPressed = false;
  static bool incPressed = false;
  static bool decPressed = false;
  static bool enterPressed = false;

  if (digitalRead(programPin) == LOW && !programPressed && configDone) {
    for (int i = 0; i < EEPROM.length(); i++) EEPROM.write(i, 0);
    programPresses = 0;
    userMaxCurrent = 0.0;
    userMaxPWM = 1000;
    configDone = false;
    programPressed = true;
    pwmValue = 800;
    esc.writeMicroseconds(pwmValue);
    active = true;
    holdPhase = false;
    decelerationPhase = false;
    lcd.clear();
    lcd.print("Select Mode...");
  }

  if (!configDone) {
    if (digitalRead(programPin) == LOW && !programPressed) {
      programPresses++;
      mode = (programPresses % 2 == 1) ? 1 : 2;
      lcd.clear();
      lcd.print(mode == 1 ? "Mode: Current" : "Mode: PWM");
      delay(300);
      programPressed = true;
    }
    if (digitalRead(programPin) == HIGH) programPressed = false;

    if (digitalRead(incPin) == LOW && !incPressed) {
      if (mode == 1 && userMaxCurrent < 5.0) userMaxCurrent += 1.0;
      else if (mode == 2) userMaxPWM += 25;
      delay(200);
      incPressed = true;
    }
    if (digitalRead(incPin) == HIGH) incPressed = false;

    if (digitalRead(decPin) == LOW && !decPressed) {
      if (mode == 1 && userMaxCurrent > 0.0) userMaxCurrent -= 1.0;
      else if (mode == 2 && userMaxPWM > 1000) userMaxPWM -= 5;
      delay(200);
      decPressed = true;
    }
    if (digitalRead(decPin) == HIGH) decPressed = false;

    lcd.setCursor(0, 1);
    if (mode == 1) {
      lcd.print("Set I: ");
      lcd.print(userMaxCurrent, 1);
      lcd.print(" A      ");
    } else {
      lcd.print("Set PWM: ");
      lcd.print(userMaxPWM);
      lcd.print(" us   ");
    }

    if (digitalRead(enterPin) == LOW && !enterPressed) {
      delay(200);
      enterPressed = true;
      configDone = true;
      saveToEEPROM();
    }
    if (digitalRead(enterPin) == HIGH) enterPressed = false;
    return;
  }

  if (LoadCell.update()) newDataReady = true;

  if (newDataReady) {
    float thrust = LoadCell.getData();
    if (thrust < 0) thrust = 0;

    for (int i = 0; i < currentSamplesPerCycle; i++) {
      int adcValue = analogRead(CURRENT_SENSOR_PIN);
      float voltage = (adcValue / 1023.0) * 5.0;
      float current = (voltage - currentSensorOffset) / 0.18116;
      if (abs(current) < deadbandThreshold) current = 0.0;
      currentSquareSum += current * current;
      currentCount++;
    }

    if (currentCount >= sampleSize) {
      float rmsCurrent = sqrt(currentSquareSum / currentCount);
      latestCurrent = rmsCurrent;

      Serial.print(thrust, 1);
      Serial.print(",");
      Serial.print(rmsCurrent, 1);
      Serial.print(",");
      Serial.println(pwmValue);

      lcd.setCursor(0, 0);
      lcd.print("| Mode   | Limit   |");
      lcd.setCursor(0, 1);
      if (mode == 1) {
        lcd.print("|Current | ");
        lcd.print(userMaxCurrent, 1);
        lcd.print(" A  |");
      } else {
        lcd.print("|PWM     | ");
        lcd.print(userMaxPWM);
        lcd.print("us |");
      }
      lcd.setCursor(0, 2);
      lcd.print("|T:");
      lcd.print((int)thrust);
      lcd.print("gm | I:");
      lcd.print(rmsCurrent, 1);
      lcd.print(" A |");
      lcd.setCursor(0, 3);
      lcd.print("|PWM     |");
      lcd.setCursor(10, 3);
      lcd.print(pwmValue);
      lcd.print(" us |");

      currentSquareSum = 0;
      currentCount = 0;
    }

    newDataReady = false;
    t = millis();
  }

  unsigned long currentTime = millis();

  if (active && !holdPhase && !decelerationPhase) {
    if (pwmValue == 800) {
      esc.writeMicroseconds(pwmValue);
      if (currentTime - lastStepTime >= 5000) {
        pwmValue = 900;
        esc.writeMicroseconds(pwmValue);
        lastStepTime = currentTime;
      }
    } else if (pwmValue == 900 && currentTime - lastStepTime >= 2000) {
      pwmValue = 1000;
      esc.writeMicroseconds(pwmValue);
      lastStepTime = currentTime;
    } else if (pwmValue >= 1000 && currentTime - lastStepTime >= 3000) {
      esc.writeMicroseconds(pwmValue);

      if ((mode == 1 && latestCurrent >= userMaxCurrent) ||
          (mode == 2 && pwmValue >= userMaxPWM)) {
        holdPhase = true;
        holdStartTime = currentTime;
      } else {
        pwmValue += 5;
      }
      lastStepTime = currentTime;
    }
  }

  if (holdPhase && currentTime - holdStartTime >= 5000) {
    holdPhase = false;
    decelerationPhase = true;
    lastStepTime = currentTime;
  }

  if (decelerationPhase && currentTime - lastStepTime >= 3000) {
    if (pwmValue > 800) {
      pwmValue -= 5;
      if (pwmValue < 800) pwmValue = 800;
      esc.writeMicroseconds(pwmValue);
    } else {
      decelerationPhase = false;
      active = false;
    }
    lastStepTime = currentTime;
  }

  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();
  }

  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }
}
