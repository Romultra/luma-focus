#include <DallasTemperature.h>
#include <Wire.h>
#include <Stepper.h>

const byte oneWireAPin = 3;
const byte oneWireBPin = 7;

const int StepsPerRevolution = 2048; // Full revolution
const int StepsPer90Degrees = StepsPerRevolution / 4; // 90-degree rotation

int tempThreshold = 30; // Temperature threshold
int rpm = 10;            // Stepper motor speed in RPM

int currentPosition = 0; // Current position of the stepper motor in steps
bool aboveThreshold = false; // State to track if the last reading was above threshold

unsigned long lastPrintTime = 0; // Stores the time of the last print

OneWire oneWireA(oneWireAPin);
OneWire oneWireB(oneWireBPin);

Stepper myStepper(StepsPerRevolution, 8, 10, 9, 11);

DallasTemperature sensorsA(&oneWireA);
DallasTemperature sensorsB(&oneWireB);

void setup() {
    myStepper.setSpeed(rpm);
    Serial.begin(9600);
    sensorsA.begin();
    sensorsB.begin();
}

void loop() {
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
    double avg = (tempC1 + tempC2) / 2;

    // Measure elapsed time
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastPrintTime;
    lastPrintTime = currentTime;

    // Print temperature readings, average, elapsed time (milliseconds)
    Serial.print(tempC1),  Serial.print(','), Serial.print(tempC2), Serial.print(','), Serial.print(avg), Serial.print(','), Serial.println(currentTime);
    // Check temperature and control motor
    if (avg > tempThreshold&& !aboveThreshold && currentPosition < StepsPer90Degrees) {
        // Rotate forward 90 degrees
        myStepper.step(StepsPer90Degrees);
        currentPosition += StepsPer90Degrees;
        aboveThreshold = true; // Mark as above threshold
    } else if (avg < tempThreshold && aboveThreshold && currentPosition > -StepsPer90Degrees) {
        // Rotate backward 90 degrees
        myStepper.step(-StepsPer90Degrees);
        currentPosition -= StepsPer90Degrees;
        aboveThreshold = false; // Mark as below threshold
    }
}

