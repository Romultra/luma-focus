#include <DallasTemperature.h>
#include <Wire.h>
#include <PID_v1.h>
#include <AccelStepper.h>

#define motorPin1  2
#define motorPin2  3
#define motorPin3  4
#define motorPin4  5

#define FULLSTEP 4

AccelStepper stepper(FULLSTEP, motorPin1, motorPin3, motorPin2, motorPin4);

const double maxArea = 1;

const byte oneWireAPin = 6;
const byte oneWireBPin = 7;

const int stepsPerRevolution = 2048;

int tempThreshold = 50; // Temperature threshold
int rpm = 15;           // Stepper motor speed in RPM

unsigned long lastPrintTime = 0; // Stores the time of the last print

double Setpoint, Input, Output;

// Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, 1, 0.5, 0.001, DIRECT);

OneWire oneWireA(oneWireAPin);
OneWire oneWireB(oneWireBPin);

DallasTemperature sensorsA(&oneWireA);
DallasTemperature sensorsB(&oneWireB);

struct result {   // Structure declaration
      bool outcome;           // Member (bool variable)
      double angleDeg;       // Member (double variable)
}; // End the structure with a semicolon

void setup() {
    stepper.setMaxSpeed(1000.0); // Maximum speed of the stepper
    stepper.setAcceleration(500.0); // Acceleration for smoother motion
    Serial.begin(9600);
    sensorsA.begin();
    sensorsB.begin();
    Serial.println("tempC1,tempC2,avg,area,angle,currentTime");

    // Initialize PID
    Input = 0;  // Initial temperature input
    Setpoint = 50;  // Desired temperature
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, maxArea); // Match PID output to area limits
}

result moveToArea(double area){
  bool areaIsNotValid = area > maxArea or area < 0 ;  // à terminer

  struct result result;
  if (areaIsNotValid) {
    Serial.println("invalid area");
    result.outcome = false;
    result.angleDeg = 0;
    return result;
  }

  // Inverse Kinematic (IK)
  double angleRad = (acos(abs(area-1)*25.25/30));

  //steppers.run();

  // Converting radian to degree
  double angleDeg = angleRad*RAD_TO_DEG;

  bool test = moveToAngle(angleDeg);

  if(!test){
    Serial.println("invalid angle");
    result.outcome = false;
    result.angleDeg = angleDeg;
    return result;
  }
  else{
    result.outcome = true;
    result.angleDeg = angleDeg;
    return result;
  }
}

bool moveToAngle(double angleDeg){
  // Set steppers' target position using the desired angle
  //steppers.run();

  bool angleNotValid = angleDeg > 90 or angleDeg < 0;
  if (angleNotValid) {
    return false;
  }

  double offset = 15; // Can be ajusted
  double anglePos = angleDeg - offset;
  double angleInStep = anglePos*stepsPerRevolution/360; // TO CHANGE
  //steppers.run();

  double positions; // Array of desired stepper positions
  positions = angleInStep;

  stepper.moveTo(positions);
  while (stepper.run());

  return true;
}

void loop() {
  delay(5000);
    // Request temperatures
    sensorsA.requestTemperatures();
    sensorsB.requestTemperatures();

    // Get temperatures from sensors
    float tempC1 = sensorsA.getTempCByIndex(0);
    float tempC2 = sensorsB.getTempCByIndex(0);

    // Validate sensor readings
    if (tempC1 == DEVICE_DISCONNECTED_C) tempC1 = tempC2;
    if (tempC2 == DEVICE_DISCONNECTED_C) tempC2 = tempC1;

    // Calculate average temperature
    Input = (tempC1 + tempC2) / 2.0;

    // Measure elapsed time
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastPrintTime;
    lastPrintTime = currentTime;

    //Print temperature readings, average, elapsed time
    //Serial.print("tempC1:");
    //Serial.print(tempC1);
    //Serial.print(',');
    //Serial.print("tempC2:");
    //Serial.print(tempC2);
    //Serial.print(',');
    //Serial.print("avg:");
    //Serial.print(Input);
    //Serial.print(',');
    //Serial.print("currentTime:");
    //Serial.println(currentTime);


    // Compute PID and move stepper based on output
    myPID.Compute();
    result result = moveToArea(Output);
    if (result.outcome) {
    }
    Serial.print(tempC1),  Serial.print(','), Serial.print(tempC2), Serial.print(','), Serial.print(Input), Serial.print(','), Serial.print(Output), Serial.print(','), Serial.print(result.angleDeg), Serial.print(','), Serial.println(currentTime);
}

//preportion between 0 and 1 Area = cos(angle)*30/25.25 
//angle=arccos(Area*25.25/30)
//25.25/30