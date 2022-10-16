#include <PID_v1.h>
#include <Wire.h>

#define ERROR_SAMPLING_COUNT 200
#define THRESHOLD_ANGLE 10
#define GYRO_ANGLE_MAX_CUMULATIVE_ERROR 5
#define MOTOR1_COMPENSATION 0.8
#define MOTOR2_COMPENSATION 1

/* ---------------------- MPU ------------------------ */
const int MPU = 0x68;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

/* ---------------------- PID ------------------------ */
double setpoint = 0;
double Kp = 5; //Set this first
double Kd = 0.1; //Set this secound
double Ki = 1; //Finally set this

double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

/* ---------------------- PINS ------------------------ */
const int IN1 = 10;
const int IN2 = 5;
const int IN3 = 6;
const int IN4 = 9;

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(20);

  calculate_IMU_error() ;
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255);


  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop()
{
  readMPUAsAngle();
  input = cauculateAngle();
  Serial.print("Angle:");
  Serial.println(input);

  pid.Compute();

  if (input > THRESHOLD_ANGLE) {
    moveForward();
  } else if (input < -THRESHOLD_ANGLE) {
    moveBackward();
  } else {
    stopMotors();
  }
}

void calculate_IMU_error()
{
  while (c < ERROR_SAMPLING_COUNT) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  AccErrorX = AccErrorX / ERROR_SAMPLING_COUNT;
  AccErrorY = AccErrorY / ERROR_SAMPLING_COUNT;
  c = 0;
  while (c < ERROR_SAMPLING_COUNT) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }

  GyroErrorX = GyroErrorX / ERROR_SAMPLING_COUNT;
  GyroErrorY = GyroErrorY / ERROR_SAMPLING_COUNT;
  GyroErrorZ = GyroErrorZ / ERROR_SAMPLING_COUNT;
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}

void readMPUAsAngle() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58;
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58;
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000;
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;

}

void moveForward() {
  analogWrite(IN1, abs(output)*MOTOR1_COMPENSATION);
  digitalWrite(IN2, LOW);
  analogWrite(IN3, abs(output)*MOTOR2_COMPENSATION);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  analogWrite(IN2, abs(output)*MOTOR1_COMPENSATION);
  digitalWrite(IN3, LOW);
  analogWrite(IN4, abs(output)*MOTOR2_COMPENSATION);
}

void stopMotors() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
}

float cauculateAngle() {
  if (abs(accAngleY - gyroAngleY) < GYRO_ANGLE_MAX_CUMULATIVE_ERROR ) {
    return (GyroErrorY / (AccErrorY + GyroErrorY)) * accAngleY + (AccErrorY / (AccErrorY + GyroErrorY)) * gyroAngleY;
  }
  else {
    gyroAngleY = accAngleY;
    return accAngleY;
  }
}
