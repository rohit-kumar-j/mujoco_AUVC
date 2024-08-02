#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <MPU9250.h>

#include "main.h"

// Set I2C bus to use: Wire, Wire1, etc.
#define WIRE Wire // SDA  18; SCL  19;
Servo m1;
Servo m2;
Servo m3;
Servo m4;
Servo m5;
Servo m6;
Servo led;

MPU9250 mpu;
MS5837 depth;
static double currOrn[3];
static double currPressure =0;

static double direction[3] = {1,0,1};
static double target[3] = {0, 0, 10};
static double balance_buffer[6] = {0};
static double kp = 1.55;
static double ki = 0.0005;
static double kd = 0.0000;

static float error = 0;
static float last_error = 0;
static float intg = 0;
static float diff = 0;
static float prop = 0;
static float balance = 0;

static double zOrn[3] = {0}; // Zero orientation

double pid(double target, double curr_Orn);
void setupDepth();
void setupIMU();
void setupMotorsAndLed();
void updateSensors();
void orn_Control();

void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(2000);

  //Serial.println("");
  //Serial.println(" .... Main Control Code .... ");
  // Send IMU Data;
  setupIMU();
  setupDepth();
  setupMotorsAndLed();

  // Serial.print("\n");

  if(Serial.available() == 0){
    int val = Serial.parseInt();
  }else{
    Serial.println("Serial Not available!");
  }
}

void loop() {
	Serial.println("Enter PWM signal value 1100 to 1900, 1500 to stop");
  if(Serial.available()){
    char receivedChar = Serial.read();  // Read the incoming byte
    Serial.print(receivedChar);         // Echo the received byte
  }
  delay(10);  // Small delay to prevent overwhelming the serial buffer
  Serial.println("hi\n");

  updateSensors();

  Serial.print("orn");
  Serial.print(currOrn[0]);
  Serial.print(currOrn[1]);
  Serial.print(currOrn[2]);
  Serial.println("");

  Serial.print("depth");
  Serial.print(currPressure);
  Serial.println("");

  // Update target orn!

  orn_Control()

}

void setupMotorsAndLed(){
  Serial.println("Setting up Motors");
	m1.attach(mPin1);
	m2.attach(mPin2);
	m3.attach(mPin3);
	m4.attach(mPin4);
	m5.attach(mPin5);
	m6.attach(mPin6);

	led.attach(mPin6);

	m1.writeMicroseconds(1500); // send "stop" signal to ESC.
	m2.writeMicroseconds(1500); // send "stop" signal to ESC.
	m3.writeMicroseconds(1500); // send "stop" signal to ESC.
	m4.writeMicroseconds(1500); // send "stop" signal to ESC.
	m5.writeMicroseconds(1500); // send "stop" signal to ESC.
	m6.writeMicroseconds(1500); // send "stop" signal to ESC.
	led.writeMicroseconds(1100); // LED Setup

	delay(7000); // delay to allow the ESC to recognize the stopped signal
}

void fireLed(){
  led.writeMicroseconds(1700);
}

void setupIMU(){
  if (!mpu.setup(0x68)) {  // change to your own address
    Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
    delay(5000);
  }
  mpu.update();
  zOrn[0] = mpu.getRoll();
  zOrn[1] = mpu.getPitch();
  zOrn[2] = mpu.getYaw();
}

void orn_Control(){
  int isYawCCW = direction[2];
  double yawError = target[2] + zOrn[2] - currOrn[2];
  // TODO: Controller Logic
  if( -170 >= target[2]  && target[2] >= -180){target[2] = - target[2];}
  if (yawError < 5) {
        // printf("c1\n");
        balance_buffer[0] = isYawCCW * -pid(target[2],currOrn[2]);
        balance_buffer[3] = isYawCCW * -pid(target[2],currOrn[2]);
        balance_buffer[1] = isYawCCW * pid(target[2],currOrn[2]);
        balance_buffer[2] = isYawCCW * pid(target[2],currOrn[2]);
  } else if (yawError > 5) {
        // printf("c2\n");
        balance_buffer[0] = isYawCCW * pid(target[2],currOrn[2]);
        balance_buffer[3] = isYawCCW * pid(target[2],currOrn[2]);
        balance_buffer[1] = isYawCCW * -pid(target[2],currOrn[2]);
        balance_buffer[2] = isYawCCW * -pid(target[2],currOrn[2]);
  } else{  // ccw
        // printf("ccw\n");
        balance_buffer[0] = isYawCCW*pid(target[2],currOrn[2]);
        balance_buffer[3] = isYawCCW*pid(target[2],currOrn[2]);
        balance_buffer[1] = isYawCCW*-pid(target[2],currOrn[2]);
        balance_buffer[2] = isYawCCW*-pid(target[2],currOrn[2]);
  }

  int isRollCCW = direction[0];
  double rollError = target[0]  + zOrn[0] - currOrn[0];

  double istargetNeg = (target[0]+0.0001)/(abs(target[0])+0.0001);
  if (rollError < istargetNeg * 5) {
        // printf("c1\n");
        balance_buffer[4] = isRollCCW * -pid(target[0],currOrn[0]);
        balance_buffer[5] = isRollCCW * pid(target[0],currOrn[0]);
  } else{  // ccw
        // printf("ccw\n");
        balance_buffer[4] = isRollCCW * pid(target[0],currOrn[0]);
        balance_buffer[5] = isRollCCW * -pid(target[0],currOrn[0]);
  }
}

void setupDepth(){
  !depth.init();
  delay(500);
  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}

void updateSensors(){
  mpu.update();

  currOrn[0] = mpu.getRoll();
  currOrn[1] = mpu.getPitch();
  currOrn[2] = mpu.getYaw();
  depth.read();
  currPressure = depth.pressure();
}


double pid(double target, double curr_Orn) {
    error = target - curr_Orn;
    prop = error;
    intg = error + intg;
    diff = error - last_error;
    balance = (kp * prop) + (ki * intg) + (kd * diff);
    last_error = error;
    return balance;
}
