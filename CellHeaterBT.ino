//includes for EEPROM to store data when microcontroller is off
#include <EEPROM.h>

//includes for temperature sensors
#include <OneWire.h>
#include <DallasTemperature.h>

//include for BT module
#include <SoftwareSerial.h>

//include for PID-library
#include <PID_v1.h>

//include fro parsing serial command
#include <SerialCommand.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

//Output pin for heat power
#define outputPin 3

//Temperature resolution
#define TEMPRES 12

//Minimal duration between serial send (ms)
#define SERDELAY 10000

//Setup software serial
SoftwareSerial SWSerial(4,5);

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress Thermometer[5];

//Adress of the controling themometer
byte ControlAdress[8]={40,51,223,57,3,0,0,164};
int ControlDevice;

// Variable to hold number of devices found
int numDev;

//Vars for temp
double targetTemp = 37.0;
double currentTemp = 20.0;
double Temp[5];
double output = 0;
char tempChar[8];

//Vars for PID
double Kp = 2.5;
double Ki = 1.2;
double Kd = 0.1;

//Var for time of last serial send
unsigned long lastSerial=0;

//Specify the links and initial tuning parameters
PID myPID(&currentTemp, &output, &targetTemp, Kp, Ki, Kd, DIRECT);

void setup()
{
  //initialise serial over USB
  Serial.begin(115200);

  //Initialise software serial
  SWSerial.begin(9600);
  
  Serial.println("Locating devices.");
  SWSerial.println("Locating devices.");
  
  //initialise the temp sensors on onewWire bus
  sensors.begin();


  // print number of sensors
  numDev=sensors.getDeviceCount();
  
  Serial.print("Found ");
  Serial.print(numDev);
  Serial.println(" devices");
  SWSerial.print("Found ");
  SWSerial.print(numDev);
  SWSerial.println(" devices");
  
  //assign temp sensor adress and print it
  for (int i=0;i<numDev;i++) {
    if (!sensors.getAddress(Thermometer[i], i)) {
      Serial.print("Error: device ");
      Serial.println(i);
      SWSerial.print("Error: device ");
      SWSerial.println(i);
      delay(1000);
    } else {
      Serial.print("Device ");
      Serial.print(i);
      Serial.print(": ");
      SWSerial.print("Device ");
      SWSerial.print(i);
      SWSerial.print(": ");
      printAddress(Thermometer[i]);
      sensors.setResolution(Thermometer[i], TEMPRES);
      delay(1000);

      //Check if this thermometer is the controlling one
      if (ByteArrayCompare(Thermometer[i],ControlAdress,8)) {ControlDevice=i;}
    }
  }

  Serial.print("Control themometer is: ");
  Serial.println(ControlDevice);
  SWSerial.print("Control themometer is: ");
  SWSerial.println(ControlDevice);
  
  //initialise PID library
  myPID.SetMode(AUTOMATIC);
  
  Serial.print("PID set to:");
  Serial.print(" ");
  Serial.print(Kp);
  Serial.print(" ");
  Serial.print(Ki);
  Serial.print(" ");
  Serial.println(Kd);
  SWSerial.print("PID set to:");
  SWSerial.print(" ");
  SWSerial.print(Kp);
  SWSerial.print(" ");
  SWSerial.print(Ki);
  SWSerial.print(" ");
  SWSerial.println(Kd);
  
  myPID.SetTunings(Kp, Ki, Kd);

  delay(1000);

  Serial.print("Zero time: ");
  Serial.println(millis());
  SWSerial.print("Zero time: ");
  SWSerial.println(millis());
}

void loop() {

  // Request
  sensors.requestTemperatures();

  delay(1200);

  for (int i=0;i<numDev;i++) {
    //Read and debounce
    double oldTemp = Temp[i];
    Temp[i] = sensors.getTempC(Thermometer[i]);
    
    //if (Temp[i] == -127.0) {Temp[i] = oldTemp;}  //removed to check interference
    //if (Temp[i] == 0.0) {Temp[i] = oldTemp;}
    //if (Temp[i] == 85.0) {Temp[i] = oldTemp;}
    
    if (i==ControlDevice) {currentTemp=Temp[i];}
    
  }
  
  //PID recalculate
  myPID.Compute();

  //Set output to new value
  analogWrite(outputPin, int(output));

  //Draw it all
  updateScreen();

}

void updateScreen() {
  if (millis()>=lastSerial+SERDELAY) {
    lastSerial=millis();
    
    Serial.print("Time: ");
    Serial.print(lastSerial);
    SWSerial.print("Time: ");
    SWSerial.print(lastSerial);
  
    for (int i=0;i<numDev;i++) {
      Serial.print("  Temp");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(Temp[i]);
      SWSerial.print("  Temp");
      SWSerial.print(i);
      SWSerial.print(": ");
      SWSerial.print(Temp[i]);
    }
    Serial.print("  Target: ");
    Serial.print(targetTemp);
    Serial.print("  Output: ");
    Serial.println(int(output));
    SWSerial.print("  Target: ");
    SWSerial.print(targetTemp);
    SWSerial.print("  Output: ");
    SWSerial.println(int(output));
  }
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) {
       Serial.print("0");
       SWSerial.print("0");
    }
    Serial.print(deviceAddress[i], HEX);
    SWSerial.print(deviceAddress[i], HEX);
  }
  Serial.println();
  SWSerial.println();
}

boolean ByteArrayCompare(byte a[], byte b[], int array_size)
{
   for (int i = 0; i < array_size; ++i)
     if (a[i] != b[i])
       return(false);
   return(true);
}

