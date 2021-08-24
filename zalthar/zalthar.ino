#include <Wire.h>
#include <Servo.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
Servo rampServo;
Servo mouthServo;

int rampAngle;
int mouthAngle;

int coinSensorAnalogPin = 0; // FSR is connected to analog 0
int coinSensorReading;      // the analog reading from the FSR resistor divider

int winSensorAnalogPin = 1; // FSR is connected to analog 0
int winSensorReadig;      // the analog reading from the FSR resistor divider

int SolenoidPin = 11;

void setup() {
  // put your setup code here, to run once:
  rampServo.attach(9);
  mouthServo.attach(10);

  Serial.begin(9600);
  lcd.begin(16, 2);

  pinMode(SolenoidPin, OUTPUT);
  digitalWrite(SolenoidPin, LOW);  

  while (!Serial) {
    delay(10);
  }

  mouthAngle = 0;
  rampAngle = 179;
  rampServo.write(rampAngle);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  fsrReading = analogRead(fsrAnalogPin);
  Serial.print("Analog reading = ");
  Serial.println(fsrReading);

 
  uint8_t buttons = lcd.readButtons();

  if (buttons) {
    lcd.clear();
    lcd.setCursor(0,0);
    if (buttons & BUTTON_UP) {
      lcd.print("UP ");
      lcd.setBacklight(RED);
    }
    if (buttons & BUTTON_DOWN) {
      lcd.print("DOWN ");
      lcd.setBacklight(YELLOW);
    }
    if (buttons & BUTTON_LEFT) {
      lcd.print("LEFT ");
      lcd.setBacklight(GREEN);
    }
    if (buttons & BUTTON_RIGHT) {
      lcd.print("RIGHT ");
      lcd.setBacklight(TEAL);
    }
    if (buttons & BUTTON_SELECT) {
      lcd.print("SELECT ");
      lcd.setBacklight(VIOLET);
    }
  }
  
}
