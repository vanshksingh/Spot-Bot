#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create an instance of the Adafruit_PWMServoDriver class with the I2C address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Servo parameters for MG996R
int servoChannel = 1;    // Specify the PWM channel connected to the servo
int servoMin = 150;      // Minimum pulse length for the MG996R servo (adjust if needed)
int servoMax = 475;      // Maximum pulse length for the MG996R servo (adjust if needed)
int targetAngle = 90;    // Desired servo angle

void setup() {
  // Initialize the PCA9685 module
  pwm.begin();
  pwm.setPWMFreq(50);  // Set the PWM frequency (usually 50Hz for servos)
}

void loop() {
  // Set the MG996R servo to the target angle (90 degrees)
  setServoAngle(servoChannel, targetAngle);

  // You can add additional code or logic here as needed

  // Delay or perform other tasks
  delay(1000);  // Delay for 1 second
}

void setServoAngle(int channel, int angle) {
  // Calculate the PWM value for the desired angle
  int pulseWidth = map(angle, 0, 180, servoMin, servoMax);
  // Set the PWM signal on the specified channel
  pwm.setPWM(channel, 0, pulseWidth);
}
