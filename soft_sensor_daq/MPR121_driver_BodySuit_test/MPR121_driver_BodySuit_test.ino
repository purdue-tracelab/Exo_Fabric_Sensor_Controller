/*************************************
  Note: used in videos 3-11 with Dylan/Anjali/Lina, with 
  int chargeCurrent = 48; // uA
float chargeTime = 4; // us

  MPR121_driver.ino

  This driver is for reading capacitive sensor(s) using the MPR121.
  Currently, multiple sensors are only supported if they have the same physical dimensions.
  Sensor units are in picofarads.

  The first time you start up your device, do these three things:
  (1) Plug in to the circuit all the sensors you're going to use
      (a) All sensors should be at their rest length (lowest capacitance)
      (b) If you are using N sensors, use the first N input pins
  (2) Define some USER SETUP variables
      (a) Set NUM_SENSORS to the number of sensors you will use
      (b) Set configMode to true
  (3) Record the serial output from the autoconfiguration and update the variables in "USER SETUP"
      (a) Update chargeCurrent
      (b) Update chargeTime
      (c) Set configMode to false
  
  Written by Will Johnson based on the previous version by Dylan Shah. Last edited 2020-3-10.
*************************************/

// ***** 0. USER SETUP *****
const int NUM_SENSORS = 4; // How many sensors will you use?
bool configMode = false;
int chargeCurrent = 48; // uA
float chargeTime = 4; // us


// ***** 1. Import libraries *****
#include <Wire.h>
//#include <math.h> // For log base n
#include "Adafruit_MPR121.h" // For MPR 121

// ***** 2. Initialize Variables *****
int mpr_given = 0;
float capacitance = 0;
bool print_verbose = true;

unsigned long initial_millis;
unsigned long current_millis;

// You can have up to 4 MPR121 boards on one i2c bus but one is enough for testing!
Adafruit_MPR121 cap = Adafruit_MPR121();
// Adafruit_MPR121 cap2 = Adafruit_MPR121();

// ***** 3. Set up Arduino *****
void setup() {
  // 3.1 Initialize communication lines
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(57600);  // start serial for output. 

  // 3.2 Check for MPR121
  // Default address is 0x5A, if tied to 3.3V its 0x5B
  // If tied to SDA its 0x5C and if SCL then 0x5D
  if (!cap.begin(0x5A)) {
    Serial.println("MPR121 not found, check wiring?");
  }

  /*
  if (!cap2.begin(0x5B)) {
    Serial.println("Second MPR121 not found, check wiring?");
  } */

  // 3.3 configure the settins
  if (configMode) {
    // run the autoconfiguration
    autoconfig_init();
    // print the results of autoconfiguration
    print_autoconfig();
    
  } else {
      // disable autoconfig and use the current and time settings above
      config_with_settings(cap);
      // config_with_settings(cap2);
  }

  initial_millis = millis();
}


// ***** 4. Read sensors and send to PC over Serial, forever *****
void loop() {
  if(!configMode){
    
     //4.1 Print current time
    if (print_verbose) {
      current_millis = millis() - initial_millis;
      Serial.print(current_millis); Serial.print('\t'); // Print time elapsed since setup finished
    }

  
    // 4.3 Read from MPR121
    for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
      mpr_given = cap.filteredData(sensor);
      mpr_given = cap.filteredData(sensor);
      capacitance = chargeCurrent*chargeTime*1024.0/mpr_given/3.3;
      Serial.print(capacitance); Serial.print('\t');
    }
    Serial.print("\n");
    delay(10);
  }
}

// ***** 5. Helper functions *****

// set up the autoconfiguration
void autoconfig_init() {
  // 3.2 Auto-configure Charge-time and charge-current for MPR121
  cap.writeRegister(MPR121_AUTOCONFIG0, 0x00001011);

  // Specify the search boundaries and target for the auto-configuration. correct values for Vdd = 3.3V are 200, 180, 130.
  cap.writeRegister(MPR121_UPLIMIT, 200);     // ((Vdd - 0.7)/Vdd) * 256
  cap.writeRegister(MPR121_TARGETLIMIT, 180); // UPLIMIT * 0.9
  cap.writeRegister(MPR121_LOWLIMIT, 130);    // UPLIMIT * 0.65

  // wait for autoconfiguration to finish (is this necessary)?
  delay(500);
}

void print_autoconfig() {
  // read the results of the autoconfiguration
  byte cdc = cap.readRegister8(MPR121_CHARGECURR_0);
  byte cdt = cap.readRegister8(MPR121_CHARGETIME_1);

  // 3.2.1 Print the autoconfiguration results for the user
  Serial.println("Results of the Autoconfiguration:");
    
  // print the current result
  Serial.print("int chargeCurrent = ");
  Serial.print(Reg2Current(cdc));
  Serial.println("; // uA");

  // print the time result
  Serial.print("int chargeTime = ");
  Serial.print(Reg2Time(cdt));
  Serial.println("; // us");

  // reminder instructions
  Serial.println("Fill in this information at the top of the code.");
}

// prints all 8 bits of a 1-byte register, including leading zeros
void printBits(byte myByte) {
  // start with 120 (0b10000000) and shift the bit to the right each iteration
  for (byte mask = 0b10000000; mask; mask >>= 1) {
    // bitwise AND checks if the current bit is a 1
    if (mask & myByte) {
      Serial.print('1');
    } else {
      Serial.print('0');
    }
  }
}

// this converts the CDC register value into a current in uA
int Reg2Current(byte currentRegister) {
  // get the last 6 bits of the current register and convert to int
  return int(currentRegister);
}

// this converts the INDIVIDUAL CDT register of a SINGLE electrode into a time in microseconds
float Reg2Time(byte timeRegister) {
  // get the leftmost 3 bit0s of the register
  int n = timeRegister & 0b00000111;
  // use the formala on the datasheet to calculate time
  return 0.5*pow(2,n-1);
}

void config_with_settings(Adafruit_MPR121 board) {
  // Disable auto-configuration to specify I and T manually.
   board.writeRegister(MPR121_AUTOCONFIG0, 0x00); // Disable Auto-configuration

  // Global settings
   board.writeRegister(MPR121_CONFIG1, byte(chargeCurrent)); // [7-6] Default first filter iterations = 6 (default); [5-0] CDC = X ua (000001 = 1 ua, 111111 = 63 ua)
   board.writeRegister(MPR121_CONFIG2, time2Reg(chargeTime)); // [7-5] CDT 0.5 us (default) to 32 us. EX: 001 = 0.5 us; 020 = 1 us; 111 = 32 us
  // [4-3] Second filter iterations 00 = 4 (Default)
  // [2-0] Electrode sample interval 000 = 1 ms, 001 = 2 ms, ... 100 = 16 ms (default), 111 = 128 ms
}

// this takes in a charging time and outputs the correct GLOBAL configuration register setting
byte time2Reg(float T) {
  // the only valid floating-point value
  if (T == 0.5) {
    return 0b00100000;
  } else {
    // the (inverse) formula from the data sheet
    byte n = log(2*T)/log(2) + 1;
    return n << 5;
  }
}
