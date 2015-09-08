//include EEPROM
#include <EEPROM.h>
//EEPROM map
//bytes 0 - 7 ControlAddress
//bytes 8 - 11 float P
//bytes 12 - 15 float I
//bytes 16 - 19 float D
//bytes 20 - 23 float targetTemp

//includes for Servo
#include <Servo.h>

//includes for temperature sensors
#include <OneWire.h>
#include <DallasTemperature.h>

//include for BT module
#include <SoftwareSerial.h>

//include for PID-library
#include <PID_v1.h> //http://playground.arduino.cc/Code/PIDLibrary, https://github.com/br3ttb/Arduino-PID-Library

//include for parsing serial command
#include <SerialCommand.h> //https://github.com/scogswell/ArduinoSerialCommand/

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

//Output pin for heat power
#define outputPin 3

//Temperature resolution
#define TEMPRES 12

//Minimal duration between serial send (ms)
long int serialWait=10000;

//Setup software serial
SoftwareSerial SWSerial=SoftwareSerial(4,5); // RX, TX

//Set up Serial command
SerialCommand swCmd(SWSerial);

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress Thermometer[5];

//Address of the controling themometer
byte ControlAddress[8];
int ControlDevice=-1;

// Variable to hold number of devices found
int numDev;

//Set up servo
Servo Shutter;

#define SERVOPIN 10

//Vars for temp
double targetTemp;
double currentTemp;
double Temp[5];
double output = 0;

//Vars for PID
double Kp;
double Ki;
double Kd;

//Var for time of last serial send
unsigned long lastSerial=0;

//Specify the links and initial tuning parameters
PID myPID(&currentTemp, &output, &targetTemp, Kp, Ki, Kd, DIRECT);

void setup()
{
  //initialise serial over USB
  Serial.begin(9600);

  //Initialise software serial
  SWSerial.begin(9600);
  
  //Set up serial commands for software serial
  swCmd.addCommand("T",Tcmd); // set Tagert Temp
  swCmd.addCommand("S",Scmd); // set shutter position
  swCmd.addCommand("P",Pcmd); // set P in PID
  swCmd.addCommand("I",Icmd); // set I in PID
  swCmd.addCommand("D",Dcmd); // set D in PID
  swCmd.addCommand("R",Rcmd); // request data
  swCmd.addCommand("W",Wcmd); // set wait between serial data send
  swCmd.addCommand("C",Ccmd); // set controlling thermometer
  swCmd.addDefaultHandler(unrecognized);

  //Servo attach
  Shutter.attach(SERVOPIN);

  //Read from EEPROM
  EEPROM.get(0, ControlAddress);
  EEPROM.get(8, Kp);
  EEPROM.get(12, Ki);
  EEPROM.get(16, Kd);
  EEPROM.get(20, targetTemp);
  
  SWprint("Control Address from EEPROM: ");
  printAddress(ControlAddress);
  SWprintln("");
  
  SWprintln("Locating devices.");
  //initialise the temp sensors on onewWire bus
  sensors.begin();

  // print number of sensors
  numDev=sensors.getDeviceCount();
  
  SWprint("Found ");
  SWprinti(numDev);
  SWprintln(" devices");

  //assign temp sensor address and print it
  for (int i=0;i<numDev;i++) {
    if (!sensors.getAddress(Thermometer[i], i)) {
      SWprint("Error: device ");
      SWprinti(i);
      SWprintln("");
      delay(1000);
    } else {
      SWprint("Device ");
      SWprinti(i);
      SWprint(": ");
      printAddress(Thermometer[i]);
      SWprintln("");
      sensors.setResolution(Thermometer[i], TEMPRES);
      delay(1000);

      //Check if this thermometer is the controlling one
      if (ByteArrayCompare(Thermometer[i],ControlAddress,8)) {ControlDevice=i;}
    }
  }

  if (ControlDevice>-1) {
    SWprint("Control themometer is: ");
    SWprinti(ControlDevice);
    SWprintln("");
  } else {
    SWprintln("No control thermometer! Set with C.");
  }

  //initialise PID library
  myPID.SetMode(AUTOMATIC);
  
  SWprintPID();
  
  myPID.SetTunings(Kp, Ki, Kd);

  delay(1000);
}

void loop() {

  // Request
  sensors.requestTemperatures();

  //Read serial command
  swCmd.readSerial();

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
  if (millis()>=lastSerial+serialWait && serialWait > 0) {
    lastSerial=millis();
    SWprintTemp();
  }
}

void SWprintTemp() {
  SWprint("Time: ");
  SWprintul(millis());

  for (int i=0;i<numDev;i++) {
    SWprint("  Temp");
    SWprinti(i);
    SWprint(": ");
    SWprintd(Temp[i]);
  }
  
  SWprint("  Target: ");
  SWprintd(targetTemp);
  SWprint("  Output: ");
  SWprinti(int(output));
  SWprintln("");
}

void SWprintFull() {
  SWprint("Have ");
  SWprinti(numDev);
  SWprintln(" devices");
 
  //assign temp sensor address and print it
  for (int i=0;i<numDev;i++) {
    SWprint("Device ");
    SWprinti(i);
    SWprint(": ");
    printAddress(Thermometer[i]);
    SWprintln("");
  }

  SWprint("Control themometer is: ");
  SWprinti(ControlDevice);
  SWprint(": ");
  printAddress(ControlAddress);
  SWprintln("");

  SWprintPID();

  SWprint("Shutter Position: ");
  SWprinti(Shutter.read());
  SWprintln("");

  SWprint("Serial wait: ");
  SWprintl(serialWait);
  SWprintln("");
}

//Print to both Serial and SWSerial
void SWprint(char *arg) { //String
  Serial.print(arg);
  SWSerial.print(arg);
}

void SWprintln(char *arg) { // String + end of line
  Serial.println(arg);
  SWSerial.println(arg);
}

void SWprinti(int i) { //int
  Serial.print(i);
  SWSerial.print(i);
}

void SWprinth(int h) { //hex
  Serial.print(h, HEX);
  SWSerial.print(h, HEX);
}

void SWprintd(double d) { //double
  Serial.print(d);
  SWSerial.print(d);
}

void SWprintul(unsigned long ul) { //unsigned long
  Serial.print(ul);
  SWSerial.print(ul);
}

void SWprintl(long l) { //long
  Serial.print(l);
  SWSerial.print(l);
}

void SWprintPID() {
  SWprint("PID set to: ");
  SWprintd(Kp);
  SWprint(" ");
  SWprintd(Ki);
  SWprint(" ");
  SWprintd(Kd);
  SWprintln("");
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) {
       SWprint("0");
    }
    SWprinth(deviceAddress[i]);
  }
}

boolean ByteArrayCompare(byte a[], byte b[], int array_size)
{
   for (int i = 0; i < array_size; ++i)
     if (a[i] != b[i])
       return(false);
   return(true);
}

//Commands received
void Tcmd() {
  char *arg;
  arg = swCmd.next();
  if (arg != NULL) {
    targetTemp=atof(arg);
    SWprint("Target set to: ");
    SWprintd(targetTemp);
    SWprintln("");
    EEPROM.put(20,targetTemp);
  } else {
    SWprintln("T: No data.");
  }
}

void Scmd() {
  char *arg;
  int i;
  arg = swCmd.next();
  if (arg != NULL) {
    i=atoi(arg);
    Shutter.write(i);
  } else {
    SWprintln("S: No data.");
  }
}

void Pcmd() {
  char *arg;
  arg = swCmd.next();
  if (arg != NULL) {
    Kp=atof(arg);
    SWprintPID();
    myPID.SetTunings(Kp, Ki, Kd);
    EEPROM.put(8,Kp);
  } else {
    SWprintln("P: No data.");
  }
}

void Icmd() {
  char *arg;
  arg = swCmd.next();
  if (arg != NULL) {
    Ki=atof(arg);
    SWprintPID();
    myPID.SetTunings(Kp, Ki, Kd);
    EEPROM.put(12,Ki);
  } else {
    SWprintln("I: No data.");
  }
}

void Dcmd() {
  char *arg;
  arg = swCmd.next();
  if (arg != NULL) {
    Kd=atof(arg);
    SWprintPID();
    myPID.SetTunings(Kp, Ki, Kd);
    EEPROM.put(16,Kd);
  } else {
    SWprintln("D: No data.");
  }
}

void Wcmd() {
  char *arg;
  arg = swCmd.next();
  if (arg != NULL) {
    serialWait=atol(arg);
  } else {
    SWprintln("W: No data.");
  }
}

void Rcmd() {
  char *arg;
  arg=swCmd.next();
  int i;
  if (arg != NULL) {
    i=atoi(arg);
    switch(i) {
      case 1:
        SWprintTemp();
        break;
      case 2:
        SWprintFull();
        break;
      default:
        SWprintln("R: Unrecognised.");
        break;
      }
  } else {
    SWprintln("R: No data.");
  }
}

void Ccmd() {
  char *arg;
  arg = swCmd.next();
  if (arg != NULL) {
    ControlDevice=atoi(arg);
    if (ControlDevice>numDev-1 || ControlDevice < 0) {
      ControlDevice=-1;
      SWprintln("C: Device does not exist.");
    } else {
      for (int i=0;i<8;i++) {
        ControlAddress[i] = Thermometer[ControlDevice][i];
      }
      EEPROM.put(0,Thermometer[ControlDevice]);
    }
  } else {
    SWprintln("C: No data.");
  }
}

void unrecognized() {
  SWprintln("Command Unrecognised.");
}

