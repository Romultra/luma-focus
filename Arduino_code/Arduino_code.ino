#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <math.h>
#include <AccelStepper.h>

#define HALFSTEP 8

// Motor pin definitions
#define motorPin1  3     // IN1 on the ULN2003 driver 1
#define motorPin2  4     // IN2 on the ULN2003 driver 1
#define motorPin3  5     // IN3 on the ULN2003 driver 1
#define motorPin4  6     // IN4 on the ULN2003 driver 1

// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper with 28BYJ-48
AccelStepper stepper(HALFSTEP, motorPin1, motorPin3, motorPin2, motorPin4);
// Defines the number of steps per rotation
const int stepsPerRevolution = 2048;


const double maxArea = 255
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,2,5,1, DIRECT);


// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 2
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

void setup(void) {
  stepper.setMaxSpeed(1000.0);
  stepper.setAcceleration(100.0);
  stepper.setSpeed(200);

  // start serial port
  Serial.begin(9600);
  Serial.println("Dallas Temperature IC Control Library Demo");

  // Start up the library
  sensors.begin(); // IC Default 9 bit. If you have troubles consider upping it 12. Ups the delay giving the IC more time to process the temperature measurement

  //initialize the variables we're linked to
  sensors.requestTemperatures(); 
  Input = sensors.getTempCByIndex(0);
  Setpoint = 60;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

bool moveToArea(double area){
  bool areaIsNotValid = area > 255 or area < 0 ;  // à terminer

  if (areaIsNotValid) {
    Serial.println("invalid area");
    return false;
  }

  double angleRad;

  //steppers.run();

  // Inverse Kinematic (IK)
  double angleRad = acos(area/maxArea);

  //steppers.run();

  // Converting radian to degree
  double angleDeg = angleRad*RAD_TO_DEG;

  bool test = moveToAngle(angleDeg);

  if(!test){
    Serial.println("invalid angle");
    return false;
  }
  else{
    return true;
  }
}

bool moveToAngle(double angleDeg){
  // Set steppers' target position using the desired angle
  //steppers.run();

  bool angleNotValid = angleDeg > 90 or angleDeg < 0;
  if (angleNotValid) {
    return false;
  }

  double angleInStep = angleDeg*stepsPerRevolution/360; // TO CHANGE
  //steppers.run();
  double offset = 0; // Can be ajusted
  double positions; // Array of desired stepper positions
  positions = angleInStep - offset;

  while (steppers.run());
  Serial.println("Moved");
  stepper.moveTo(positions);
  
  return true;
}

void loop(void){
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
  
  Serial.print("Temperature for Device 1 is: ");
  Serial.print(sensors.getTempCByIndex(0)); // Why "byIndex"? You can have more than one IC on the same bus. 0 refers to the first IC on the wire

  Input = sensors.getTempCByIndex(0);

  myPID.Compute();
  analogWrite(3,Output);

  moveToArea(Output)
  while (steppers.run());


  /*
  // Rotate CW slowly at 5 RPM
  myStepper.setSpeed(5);
  myStepper.step(stepsPerRevolution);
  delay(1000);

  // Rotate CCW quickly at 10 RPM
  myStepper.setSpeed(10);
  myStepper.step(-stepsPerRevolution);
  delay(1000);
  */
}