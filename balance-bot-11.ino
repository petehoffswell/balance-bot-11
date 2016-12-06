/* balance-bot-11
 *  Balance Bot version 1.1
 *  November 2016
 *  pete@hoffswell.com
 *  
 *  No workie!
 * 
 */

#include <BMI160.h>
#include <CurieIMU.h>
#include <Adafruit_MotorShield.h>
#include <PID_v1.h>

// IMU Complementary Filter as per http://www.pieter-jan.com/node/11
//#define ACCELEROMETER_SENSITIVITY 8192.0
#define GYROSCOPE_SENSITIVITY 16.348
#define M_PI 3.14159265359
#define delayTime (10.0/1000.0)   
#define deadZone 55.00 // minimum speed setting to get motors to run 

uint32_t pastTime = millis();   // Delay tracking

// Set up Motors for https://www.adafruit.com/products/1438
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *Motor1 = AFMS.getMotor(1); // M1 Left Front
Adafruit_DCMotor *Motor2 = AFMS.getMotor(2); // M2 Right Front

// PID Library setup from https://github.com/br3ttb/Arduino-PID-Library
double Input, Output, Setpoint;
double Kp=10; // larger = push harder
double Ki=1; // smaller = react quicker (and greater risk of oscillation)
double Kd=6; // larger = more dampening of oscillation
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  // Prep Motors
  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  // Prep PID
  Setpoint = 3.3;
  myPID.SetSampleTime(10);
  myPID.SetOutputLimits(-255+deadZone,255-deadZone);
  myPID.SetMode(AUTOMATIC);
  
  // initialize device
  Serial.println("Initializing IMU device...");

  //CurieIMU.initialize();
  CurieIMU.begin();
  CurieIMU.setFullScaleAccelRange(BMI160_ACCEL_RANGE_2G);
  CurieIMU.setFullScaleGyroRange(BMI160_GYRO_RANGE_2000);
  //  CurieIMU.setGyroRate(BMI160_GYRO_RATE_100HZ);
  //  CurieIMU.setFullScaleGyroRange(BMI160_GYRO_RANGE_2000);
  CurieIMU.setGyroRate(BMI160_GYRO_RATE_3200HZ);
  //  CurieIMU.setGyroDLPFMode(BMI160_DLPF_MODE_NORM);

  Serial.print(" full scale accel \t");
  Serial.print(CurieIMU.getFullScaleAccelRange());
  Serial.print(" full scale gyro \t");
  Serial.println(CurieIMU.getFullScaleGyroRange());

  // verify connection
  Serial.println("Testing device connections... \t");
  if (CurieIMU.testConnection()) {
    Serial.println("CurieIMU connection successful");
  } else {
    Serial.println("CurieIMU connection failed");
  }

  // use the code below to calibrate accel/gyro offset values
  Serial.println("Internal sensor offsets BEFORE calibration...");
  Serial.print(CurieIMU.getXAccelOffset());
  Serial.print("\t"); // -76
  Serial.print(CurieIMU.getYAccelOffset());
  Serial.print("\t"); // -235
  Serial.print(CurieIMU.getZAccelOffset());
  Serial.print("\t"); // 168
  Serial.print(CurieIMU.getXGyroOffset());
  Serial.print("\t"); // 0
  Serial.print(CurieIMU.getYGyroOffset());
  Serial.print("\t"); // 0
  Serial.println(CurieIMU.getZGyroOffset());
 
  Serial.println("About to calibrate. Make sure your board is stable and upright");
  delay(10);

  // The board must be resting in a horizontal position for
  // the following calibration procedure to work correctly!
  Serial.print("Starting Gyroscope calibration...");
  CurieIMU.autoCalibrateGyroOffset();
  Serial.println(" Done");
  Serial.print("Starting Acceleration calibration...");
  CurieIMU.autoCalibrateXAccelOffset(0);
  CurieIMU.autoCalibrateYAccelOffset(0);
  CurieIMU.autoCalibrateZAccelOffset(1);
  Serial.println(" Done");

  Serial.println("Internal sensor offsets AFTER calibration...");
  Serial.print(CurieIMU.getXAccelOffset());
  Serial.print("\t"); // -76
  Serial.print(CurieIMU.getYAccelOffset());
  Serial.print("\t"); // -2359
  Serial.print(CurieIMU.getZAccelOffset());
  Serial.print("\t"); // 1688
  Serial.print(CurieIMU.getXGyroOffset());
  Serial.print("\t"); // 0
  Serial.print(CurieIMU.getYGyroOffset());
  Serial.print("\t"); // 0
  Serial.println(CurieIMU.getZGyroOffset());

  Serial.println("Enabling Gyroscope/Acceleration offset compensation");
  CurieIMU.setGyroOffsetEnabled(true);
  CurieIMU.setAccelOffsetEnabled(true);
} // setup()

static void ComplementaryFilter(short accData[3], short gyrData[3], float *pitch, float *roll){
  // Pieter-Jan's IMU filter
  
  float pitchAcc, rollAcc;

  // Integrate the gyroscope data -> int(angularSpeed) = angle
  *pitch += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * delayTime; // Angle around the X-axis
  *roll -= ((float)gyrData[1] / GYROSCOPE_SENSITIVITY) * delayTime;    // Angle around the Y-axis

  // Compensate for drift with accelerometer data if !bullshit
  // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
  int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
  if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
  {
    // Turning around the X axis results in a vector on the Y-axis
    pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
    *pitch = *pitch * 0.98 + pitchAcc * 0.02;

    // Turning around the Y axis results in a vector on the X-axis
    rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
    *roll = *roll * 0.98 + rollAcc * 0.02;
  }
} // ComplementaryFilter()

void readIMUSensor(float *Angle_Filtered) {
  // Read the accelerometer and gyroscope and then calculate your current bot angle
  static float roll;
  static float pitch;
  // put your main code here, to run repeatedly:
  short accData[3];
  short gyrData[3];

  CurieIMU.getMotion6(&accData[0], &accData[1], &accData[2], &gyrData[0], &gyrData[1], &gyrData[2]);
  ComplementaryFilter(accData, gyrData, &pitch, &roll);
  Serial.print(String("Angle = ") + String(pitch));
  *Angle_Filtered = pitch;
} // readIMUSensor()


void driveBot(float Output) {
  // Drive the bot, based on the current angle
  double MotorSpeed;
  MotorSpeed = abs(Output) + deadZone;
  // if (MotorSpeed < deadZone) { MotorSpeed = deadZone; }
  Serial.print(" Output: ");
  Serial.print(Output);
  if (Output > 0) {
    Motor1->setSpeed(MotorSpeed);
    Motor2->setSpeed(MotorSpeed);
    Motor1->run(FORWARD);
    Motor2->run(FORWARD); 
    Serial.print(" FORWARD");
  }
  
  else if (Output < 0){
    Motor1->setSpeed(MotorSpeed);
    Motor2->setSpeed(MotorSpeed);
    Motor1->run(BACKWARD);
    Motor2->run(BACKWARD); 
    Serial.print(" BACKWARD");
  }
  else {
      Motor1->run(RELEASE);
      Motor2->run(RELEASE);
      Serial.print(" STOP");
  }
  Serial.println();
  
} // driveBot()


void loop() {
  static float Angle_Filtered;
  if ( millis() - pastTime > delayTime * 1000 ) {
    pastTime = millis();
    readIMUSensor(&Angle_Filtered);  // Get the current bot Angle 
    //   If angle > 45 or < -45 then stop the robot
    if (abs(Angle_Filtered) < 45) {
      // Drive the wheels as needed
      Input = Angle_Filtered;
      myPID.Compute();   
      driveBot(Output);      
    }
    else {
      //TODO:// set zero when robot down.
      //      Output = error = errSum = dErr = 0;
      Serial.println(" STOP");
      Motor1->run(RELEASE);
      Motor2->run(RELEASE);
    }
  }
} // loop()
