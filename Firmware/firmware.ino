#include <Wire.h>
#include <U8g2lib.h>
#include <Servo.h>
#include "Arduino_LED_Matrix.h"

// ===== DEBUG MODE =====
#define DEBUG_ENABLED true

// ===== OLED SETUP using U8g2 =====
// For 128x64 SSD1306 (hardware I2C)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
bool oledWorking = false;
uint8_t foundAddr = 0;  // For I2C address scanning

// ===== LED MATRIX =====
ArduinoLEDMatrix matrix;

// ===== PINS =====
const uint8_t MOTOR_SERVO_PIN = 6;
const uint8_t ARM_SERVO_PIN   = 9;
const uint8_t HUMID_PIN       = 7;
const uint8_t HAND_SERVO_1_PIN = 10;  // First hand servo
const uint8_t HAND_SERVO_2_PIN = 11;  // Second hand servo
const uint8_t TOUCH_SENSOR_PIN = 2;   // Touch sensor for servo control


// ===== COMMAND TRIGGER STATES (one-shot per activation) =====
bool touchTriggered = false;   // Track if touch sensor has been triggered

// ===== TOUCH SENSOR CYCLE COUNTER =====
int touchCount = 0;  // 0 = waiting for first touch, 1 = motor done, 2 = humidifier done, 3 = catapult done (reset to 0)

// ===== TIMING =====
const unsigned long MOTOR_HOLD_TIME = 5000;  // Time to hold at each position
const unsigned long HUMID_TIME = 30000;      // Humidifier on for 30 seconds
const unsigned long PULSE_VERIFY_DELAY = 500;     // Time to verify pulse is valid (500ms)
const int PULSE_VERIFY_COUNT = 5;                 // Number of consecutive reads required

Servo motorServo;
Servo armServo;
Servo handServo1;
Servo handServo2;

// ===== LED MATRIX EYES (8×12) =====
uint8_t eyeCenter[8][12] = {
  {0,0,0,1,1,1,1,1,1,0,0,0},
  {0,0,1,0,0,0,0,0,0,1,0,0},
  {0,1,0,0,0,0,0,0,0,0,1,0},
  {1,0,0,0,0,1,1,0,0,0,0,1},
  {1,0,0,0,0,1,1,0,0,0,0,1},
  {0,1,0,0,0,0,0,0,0,0,1,0},
  {0,0,1,0,0,0,0,0,0,1,0,0},
  {0,0,0,1,1,1,1,1,1,0,0,0}
};

uint8_t eyeBlink[8][12] = {
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,1,1,1,1,1,1,0,0,0},
  {0,0,1,0,0,0,0,0,0,1,0,0},
  {0,1,0,0,0,1,1,0,0,0,1,0},
  {0,1,0,0,0,1,1,0,0,0,1,0},
  {0,0,1,0,0,0,0,0,0,1,0,0},
  {0,0,0,1,1,1,1,1,1,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0}
};

// Eye looking left
uint8_t eyeLeft[8][12] = {
  {0,0,0,1,1,1,1,1,1,0,0,0},
  {0,0,1,0,0,0,0,0,0,1,0,0},
  {0,1,0,0,0,0,0,0,0,0,1,0},
  {1,0,0,1,1,0,0,0,0,0,0,1},
  {1,0,0,1,1,0,0,0,0,0,0,1},
  {0,1,0,0,0,0,0,0,0,0,1,0},
  {0,0,1,0,0,0,0,0,0,1,0,0},
  {0,0,0,1,1,1,1,1,1,0,0,0}
};

// Eye looking right
uint8_t eyeRight[8][12] = {
  {0,0,0,1,1,1,1,1,1,0,0,0},
  {0,0,1,0,0,0,0,0,0,1,0,0},
  {0,1,0,0,0,0,0,0,0,0,1,0},
  {1,0,0,0,0,0,0,1,1,0,0,1},
  {1,0,0,0,0,0,0,1,1,0,0,1},
  {0,1,0,0,0,0,0,0,0,0,1,0},
  {0,0,1,0,0,0,0,0,0,1,0,0},
  {0,0,0,1,1,1,1,1,1,0,0,0}
};

// Eye looking up
uint8_t eyeUp[8][12] = {
  {0,0,0,1,1,1,1,1,1,0,0,0},
  {0,0,1,0,0,0,0,0,0,1,0,0},
  {0,1,0,0,0,1,1,0,0,0,1,0},
  {1,0,0,0,0,1,1,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,1},
  {0,1,0,0,0,0,0,0,0,0,1,0},
  {0,0,1,0,0,0,0,0,0,1,0,0},
  {0,0,0,1,1,1,1,1,1,0,0,0}
};

// Eye looking down
uint8_t eyeDown[8][12] = {
  {0,0,0,1,1,1,1,1,1,0,0,0},
  {0,0,1,0,0,0,0,0,0,1,0,0},
  {0,1,0,0,0,0,0,0,0,0,1,0},
  {1,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,1,1,0,0,0,0,1},
  {0,1,0,0,0,1,1,0,0,0,1,0},
  {0,0,1,0,0,0,0,0,0,1,0,0},
  {0,0,0,1,1,1,1,1,1,0,0,0}
};

// Wide eye (surprised)
uint8_t eyeWide[8][12] = {
  {0,0,1,1,1,1,1,1,1,1,0,0},
  {0,1,0,0,0,0,0,0,0,0,1,0},
  {1,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,1,1,1,1,0,0,0,1},
  {1,0,0,0,1,1,1,1,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,1},
  {0,1,0,0,0,0,0,0,0,0,1,0},
  {0,0,1,1,1,1,1,1,1,1,0,0}
};

// Squint eye
uint8_t eyeSquint[8][12] = {
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,1,1,1,1,1,1,0,0,0},
  {0,0,1,0,0,1,1,0,0,1,0,0},
  {0,0,1,0,0,1,1,0,0,1,0,0},
  {0,0,0,1,1,1,1,1,1,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0}
};

// Happy eye (curved bottom)
uint8_t eyeHappy[8][12] = {
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,1,1,1,1,1,1,0,0,0},
  {0,0,1,0,0,0,0,0,0,1,0,0},
  {0,1,0,0,0,1,1,0,0,0,1,0},
  {0,1,0,0,0,1,1,0,0,0,1,0},
  {0,0,1,1,0,0,0,0,1,1,0,0},
  {0,0,0,0,1,1,1,1,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0}
};

// ===== DEBUG HELPER =====
void debugPrint(const char* msg) {
  if (DEBUG_ENABLED) {
    Serial.print("[DEBUG] ");
    Serial.println(msg);
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(9600);
  
  // Initialize I2C bus FIRST
  Wire.begin();
  
  if (DEBUG_ENABLED) {
    Serial.println("=================================");
    Serial.println("[DEBUG] ODM2 Starting...");
    Serial.println("[DEBUG] Debug mode: ENABLED");
    Serial.println("=================================");
  }

  pinMode(HUMID_PIN, OUTPUT);
  digitalWrite(HUMID_PIN, LOW);
  debugPrint("Humidifier pin initialized");
  
  // Initialize touch sensor pin with PULLDOWN (prevents floating/noise)
  pinMode(TOUCH_SENSOR_PIN, INPUT);
  debugPrint("Touch sensor pin initialized with PULLDOWN");
  
  
  motorServo.attach(MOTOR_SERVO_PIN);
  armServo.attach(ARM_SERVO_PIN);
  handServo1.attach(HAND_SERVO_1_PIN);
  handServo2.attach(HAND_SERVO_2_PIN);
  
  motorServo.write(90);  // 360 servo stopped
  armServo.write(90);     // Arm servo starts at 0 degrees
  debugPrint("Servos attached - arm servo at 0 degrees");

  // Initialize OLED using U8g2 with I2C scanning
  debugPrint("Initializing OLED with U8g2...");
  Wire.setClock(400000);  // Safe speed for SSD1306
  
  // I2C Scan to find OLED address
  debugPrint("Scanning I2C bus...");
  foundAddr = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      if (DEBUG_ENABLED) {
        Serial.print("[DEBUG] I2C device found at 0x");
        Serial.println(addr, HEX);
      }
      foundAddr = addr;
    }
  }
  
  if (foundAddr == 0) {
    debugPrint("No I2C devices found - check wiring!");
    oledWorking = false;
  } else {
    if (DEBUG_ENABLED) {
      Serial.print("[DEBUG] Using OLED at 0x");
      Serial.println(foundAddr, HEX);
    }
    
    // IMPORTANT: u8g2 expects 8-bit address (7-bit << 1)
    u8g2.setI2CAddress(foundAddr << 1);
    
    if (!u8g2.begin()) {
      debugPrint("u8g2.begin() failed!");
      oledWorking = false;
    } else {
      oledWorking = true;
      debugPrint("OLED initialized with U8g2!");
      
      // Test display
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(30, 30, "OLED OK!");
      u8g2.sendBuffer();
      delay(500);
    }
  }

  matrix.begin(); // initialize LED matrix
  debugPrint("LED Matrix initialized");
  
  // Startup hand animation
  debugPrint("Running startup hand animation...");
  startupHandAnimation();
  debugPrint("Startup animation complete");

  showReady();
  debugPrint("System ready - waiting for touch commands");
  if (DEBUG_ENABLED) {
    Serial.println("[DEBUG] Touch Sensor Cycle Mode:");
    Serial.println("[DEBUG]   Touch 1: Trigger MOTOR");
    Serial.println("[DEBUG]   Touch 2: Trigger HUMIDIFIER");
    Serial.println("[DEBUG]   Touch 3: Trigger CATAPULT");
    Serial.println("[DEBUG] Cycle repeats after 3rd touch");
    Serial.println("=================================");
  }
}

// ===== STARTUP HAND ANIMATION =====
void startupHandAnimation() {
  // Show waking up animation on OLED
  oledAnimateWakeUp();
  
  // Move both hand servos to max counterclockwise (0 degrees)
  debugPrint("Moving hands to max counterclockwise...");
  handServo1.write(0);
  handServo2.write(0);
  delay(1000);  // Hold at max position
  
  // Gradually move back to normal position (90 degrees)
  debugPrint("Gradually returning to normal position...");
  for (int pos = 0; pos <= 90; pos += 2) {
    handServo1.write(pos);
    handServo2.write(pos);
    oledAnimateStretch(pos);  // Animate OLED while moving
    delay(30);
  }
  
  // Hold at normal position
  handServo1.write(90);
  handServo2.write(90);
  delay(500);
  
  debugPrint("Hands at normal position");
}

// ===== MAIN LOOP =====
void loop() {
  // Check touch sensor - cycles through: Motor -> Humidifier -> Catapult
  checkTouchSensor();
}

// Verify pulse is real (not noise) by checking multiple times
bool verifyPulse(uint8_t pin) {
  debugPrint("Verifying pulse...");
  
  for (int i = 0; i < PULSE_VERIFY_COUNT; i++) {
    delay(PULSE_VERIFY_DELAY);
    if (digitalRead(pin) != HIGH) {
      debugPrint("Pulse verification FAILED - was noise");
      return false;  // Not a real pulse, just noise
    }
  }
  
  debugPrint("Pulse verification PASSED - real pulse detected");
  return true;  // Pulse is real
}

// ===== TOUCH SENSOR =====
void checkTouchSensor() {
  int touchValue = digitalRead(TOUCH_SENSOR_PIN);
  
  if (touchValue == HIGH) {
    // Only trigger once per touch (one-shot per activation)
    if (!touchTriggered) {
      // Verify touch is real with multiple checks
      if (verifyPulse(TOUCH_SENSOR_PIN)) {
        touchTriggered = true;
        
        // Cycle through actions based on touch count
        if (touchCount == 0) {
          // First touch: Trigger Motor
          Serial.println("TOUCH #1 verified - Triggering MOTOR!");
          motorAction();
          touchCount = 1;
        } else if (touchCount == 1) {
          // Second touch: Trigger Humidifier
          Serial.println("TOUCH #2 verified - Triggering HUMIDIFIER!");
          humidifierAction();
          touchCount = 2;
        } else if (touchCount == 2) {
          // Third touch: Trigger Catapult (arm servo)
          Serial.println("TOUCH #3 verified - Triggering CATAPULT!");
          catapultAction();
          touchCount = 0;  // Reset for next cycle
        }
      }
    }
  } else {
    // Reset when released so it can trigger again on next touch
    if (touchTriggered) {
      Serial.println("Released - Ready for next touch");
      Serial.print("Next action: ");
      if (touchCount == 0) Serial.println("MOTOR");
      else if (touchCount == 1) Serial.println("HUMIDIFIER");
      else if (touchCount == 2) Serial.println("CATAPULT");
      touchTriggered = false;
    }
  }
}

// ===== ACTIONS =====
void motorAction() {
  debugPrint("Motor action starting...");
  eyeAnimationMotor();
  handAnimationExcited();  // Excited hands at start
  oledAnimateLookRight();
  
  // Step 1: Spin right (clockwise) for 5 seconds
  debugPrint("Spinning right...");
  handAnimationWave();  // Wave while spinning right
  motorServo.write(120);  // Clockwise
  delay(MOTOR_HOLD_TIME);
  motorServo.write(90);   // Stop
  delay(500);
  
  oledAnimateLookLeft();
  
  // Step 2: Spin left (counter-clockwise) for 5 seconds  
  debugPrint("Spinning left...");
  handAnimationStretch();  // Stretch while spinning left
  motorServo.write(60);   // Counter-clockwise
  delay(MOTOR_HOLD_TIME);
  motorServo.write(90);   // Stop
  delay(500);
  
  oledAnimateExcited();
  
  // Step 3: Two full 360 rotations (fast spins)
  debugPrint("First 360 rotation...");
  motorServo.write(180);  // Full speed clockwise
  delay(500);            // Time for ~360 degrees
  motorServo.write(90);   // Stop
  delay(300);
  
  debugPrint("Second 360 rotation...");
  motorServo.write(180);  // Full speed clockwise
  delay(150);            // Time for ~360 degrees
  motorServo.write(90);   // Stop
  
  handAnimationCelebrate();  // Celebrate at the end
  debugPrint("Motor action complete");
  showReady();
}

void catapultAction() {
  debugPrint("Catapult action starting...");
  eyeAnimationServo();
  handAnimationPoint();  // Point toward the arm
  
  // Move servo from 0 to 90 degrees and STOP (no going back)
  armServo.write(0);
  Serial.println("Catapult launched! Servo moved - STOPPED");
  
  handAnimationWave();  // Wave after arm movement
  debugPrint("Catapult action complete");
  showReady();
}

void humidifierAction() {
  debugPrint("Humidifier action starting...");
  eyeAnimationHumidifier();
  handAnimationStretch();  // Stretch at start like waking up
  oledAnimateMisty();
  
  digitalWrite(HUMID_PIN, HIGH);
  debugPrint("Humidifier turned ON for 30 seconds");
  
  // Keep humidifier on for 30 seconds with animated eyes and hands
  unsigned long startTime = millis();
  while (millis() - startTime < HUMID_TIME) {
    oledAnimateDreamy();
    handAnimationRelax();  // Gentle relaxing hand motion
    delay(1000);
  }
  
  digitalWrite(HUMID_PIN, LOW);
  handAnimationWave();  // Wave goodbye to the mist
  debugPrint("Humidifier turned OFF");
  showReady();
}

// ===== EYE ANIMATIONS =====
void eyeAnimationMotor() {
  debugPrint("Playing motor eye animation");
  // Excited spinning look animation
  matrix.renderBitmap(eyeWide, 8, 12);  // Surprised look
  delay(200);
  matrix.renderBitmap(eyeLeft, 8, 12);
  delay(100);
  matrix.renderBitmap(eyeUp, 8, 12);
  delay(100);
  matrix.renderBitmap(eyeRight, 8, 12);
  delay(100);
  matrix.renderBitmap(eyeDown, 8, 12);
  delay(100);
  matrix.renderBitmap(eyeCenter, 8, 12);
  delay(150);
  matrix.renderBitmap(eyeSquint, 8, 12);  // Focused squint
  delay(200);
  matrix.renderBitmap(eyeCenter, 8, 12);
}

void eyeAnimationServo() {
  debugPrint("Playing servo eye animation");
  // Looking down at arm movement
  matrix.renderBitmap(eyeCenter, 8, 12);
  delay(150);
  matrix.renderBitmap(eyeBlink, 8, 12);
  delay(100);
  matrix.renderBitmap(eyeDown, 8, 12);  // Look at arm
  delay(300);
  matrix.renderBitmap(eyeCenter, 8, 12);
  delay(100);
  matrix.renderBitmap(eyeDown, 8, 12);
  delay(200);
  matrix.renderBitmap(eyeHappy, 8, 12);  // Happy about arm movement
  delay(300);
  matrix.renderBitmap(eyeCenter, 8, 12);
}

void eyeAnimationHumidifier() {
  debugPrint("Playing humidifier eye animation");
  // Dreamy/misty animation for humidifier
  matrix.renderBitmap(eyeCenter, 8, 12);
  delay(100);
  matrix.renderBitmap(eyeBlink, 8, 12);
  delay(150);
  matrix.renderBitmap(eyeSquint, 8, 12);  // Squint like feeling mist
  delay(200);
  matrix.renderBitmap(eyeHappy, 8, 12);   // Happy
  delay(250);
  matrix.renderBitmap(eyeBlink, 8, 12);
  delay(100);
  matrix.renderBitmap(eyeCenter, 8, 12);
  delay(150);
  matrix.renderBitmap(eyeWide, 8, 12);    // Refreshed wide eye
  delay(200);
  matrix.renderBitmap(eyeCenter, 8, 12);
}

// Random idle animation for when waiting
void eyeIdleAnimation() {
  int randAnim = random(0, 5);
  switch(randAnim) {
    case 0: // Simple blink
      matrix.renderBitmap(eyeBlink, 8, 12);
      delay(150);
      matrix.renderBitmap(eyeCenter, 8, 12);
      break;
    case 1: // Look left then right
      matrix.renderBitmap(eyeLeft, 8, 12);
      delay(300);
      matrix.renderBitmap(eyeRight, 8, 12);
      delay(300);
      matrix.renderBitmap(eyeCenter, 8, 12);
      break;
    case 2: // Look around
      matrix.renderBitmap(eyeUp, 8, 12);
      delay(200);
      matrix.renderBitmap(eyeDown, 8, 12);
      delay(200);
      matrix.renderBitmap(eyeCenter, 8, 12);
      break;
    case 3: // Double blink
      matrix.renderBitmap(eyeBlink, 8, 12);
      delay(100);
      matrix.renderBitmap(eyeCenter, 8, 12);
      delay(150);
      matrix.renderBitmap(eyeBlink, 8, 12);
      delay(100);
      matrix.renderBitmap(eyeCenter, 8, 12);
      break;
    case 4: // Squint
      matrix.renderBitmap(eyeSquint, 8, 12);
      delay(400);
      matrix.renderBitmap(eyeCenter, 8, 12);
      break;
  }
}

// ===== HAND ANIMATIONS =====
void handAnimationWave() {
  debugPrint("Playing hand wave animation");
  // Quick wave motion
  for (int i = 0; i < 3; i++) {
    handServo1.write(60);
    handServo2.write(120);
    delay(200);
    handServo1.write(120);
    handServo2.write(60);
    delay(200);
  }
  // Return to neutral
  handServo1.write(90);
  handServo2.write(90);
  delay(100);
}

void handAnimationExcited() {
  debugPrint("Playing excited hand animation");
  // Rapid excited movements
  for (int i = 0; i < 4; i++) {
    handServo1.write(45);
    handServo2.write(45);
    delay(100);
    handServo1.write(135);
    handServo2.write(135);
    delay(100);
  }
  // Return to neutral
  handServo1.write(90);
  handServo2.write(90);
  delay(100);
}

void handAnimationStretch() {
  debugPrint("Playing stretch hand animation");
  // Slow stretch outward
  for (int pos = 90; pos >= 30; pos -= 5) {
    handServo1.write(pos);
    handServo2.write(180 - pos);
    delay(40);
  }
  delay(300);
  // Return to neutral
  for (int pos = 30; pos <= 90; pos += 5) {
    handServo1.write(pos);
    handServo2.write(180 - pos);
    delay(40);
  }
  handServo1.write(90);
  handServo2.write(90);
}

void handAnimationRelax() {
  debugPrint("Playing relax hand animation");
  // Gentle swaying motion for humidifier
  for (int i = 0; i < 5; i++) {
    handServo1.write(75);
    handServo2.write(75);
    delay(400);
    handServo1.write(105);
    handServo2.write(105);
    delay(400);
  }
  // Return to neutral
  handServo1.write(90);
  handServo2.write(90);
}

void handAnimationPoint() {
  debugPrint("Playing point hand animation");
  // One hand points down (toward arm servo)
  handServo1.write(90);
  handServo2.write(45);  // Point down
  delay(500);
  handServo2.write(90);
  delay(200);
}

void handAnimationCelebrate() {
  debugPrint("Playing celebrate hand animation");
  // Victory/celebration pose
  handServo1.write(0);
  handServo2.write(180);
  delay(300);
  // Wiggle
  for (int i = 0; i < 3; i++) {
    handServo1.write(20);
    handServo2.write(160);
    delay(150);
    handServo1.write(0);
    handServo2.write(180);
    delay(150);
  }
  // Return to neutral
  for (int pos = 0; pos <= 90; pos += 10) {
    handServo1.write(pos);
    handServo2.write(180 - pos);
    delay(30);
  }
  handServo1.write(90);
  handServo2.write(90);
}

// ===== OLED DRAW using U8g2 =====
void drawEyeOLED(int xOffset, int yOffset, int pupilSize) {
  if (!oledWorking) return;
  
  u8g2.clearBuffer();
  // Draw eye outline (circle)
  u8g2.drawCircle(64, 32, 24, U8G2_DRAW_ALL);
  // Draw pupil (filled circle)
  u8g2.drawDisc(64 + xOffset, 32 + yOffset, pupilSize, U8G2_DRAW_ALL);
  u8g2.sendBuffer();
}

void drawEyeOLED(int offset) {
  drawEyeOLED(offset, 0, 7);
}

// OLED wake up animation (for startup)
void oledAnimateWakeUp() {
  if (!oledWorking) return;
  
  // Eye closed (horizontal line)
  u8g2.clearBuffer();
  u8g2.drawLine(40, 32, 88, 32);
  u8g2.sendBuffer();
  delay(500);
  
  // Eye opening animation
  for (int i = 4; i <= 24; i += 4) {
    u8g2.clearBuffer();
    u8g2.drawCircle(64, 32, i, U8G2_DRAW_ALL);
    if (i > 8) {
      int pupilSize = i / 4;
      u8g2.drawDisc(64, 32, pupilSize, U8G2_DRAW_ALL);
    }
    u8g2.sendBuffer();
    delay(80);
  }
  drawEyeOLED(0, 0, 7);
}

// OLED stretch animation (during hand servo movement)
void oledAnimateStretch(int progress) {
  if (!oledWorking) return;
  
  int yOffset = map(progress, 0, 90, -5, 0);
  int size = map(progress, 0, 90, 5, 7);
  drawEyeOLED(0, yOffset, size);
}

// OLED eye looking right animation
void oledAnimateLookRight() {
  if (!oledWorking) return;
  
  for (int i = 0; i <= 10; i += 2) {
    drawEyeOLED(i, 0, 7);
    delay(50);
  }
}

// OLED eye looking left animation
void oledAnimateLookLeft() {
  if (!oledWorking) return;
  
  for (int i = 10; i >= -10; i -= 2) {
    drawEyeOLED(i, 0, 7);
    delay(50);
  }
  for (int i = -10; i <= 0; i += 2) {
    drawEyeOLED(i, 0, 7);
    delay(50);
  }
}

// OLED excited animation (pupil grows/shrinks)
void oledAnimateExcited() {
  if (!oledWorking) return;
  
  for (int j = 0; j < 3; j++) {
    for (int i = 7; i <= 12; i++) {
      drawEyeOLED(0, 0, i);
      delay(40);
    }
    for (int i = 12; i >= 7; i--) {
      drawEyeOLED(0, 0, i);
      delay(40);
    }
  }
}

// OLED misty/dreamy start animation
void oledAnimateMisty() {
  if (!oledWorking) return;
  
  for (int i = 0; i <= 5; i++) {
    drawEyeOLED(i, -2, 6);
    delay(100);
  }
}

// OLED dreamy floating animation
void oledAnimateDreamy() {
  if (!oledWorking) return;
  
  drawEyeOLED(3, -2, 6);
  delay(400);
  drawEyeOLED(0, 0, 7);
  delay(400);
  drawEyeOLED(-3, 2, 6);
  delay(400);
  drawEyeOLED(0, 0, 7);
  delay(400);
}

// OLED blink animation
void oledAnimateBlink() {
  if (!oledWorking) return;
  
  u8g2.clearBuffer();
  u8g2.drawCircle(64, 32, 24, U8G2_DRAW_ALL);
  u8g2.drawLine(40, 32, 88, 32);
  u8g2.sendBuffer();
  delay(100);
  drawEyeOLED(0, 0, 7);
}

void showReady() {
  debugPrint("Showing ready state");
  if (oledWorking) {
    drawEyeOLED(0, 0, 7);
  }
  matrix.renderBitmap(eyeCenter, 8, 12);
}
