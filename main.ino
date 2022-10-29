#include <SPI.h>
#include <MFRC522.h>
#include <ezButton.h>

/* Here we define constants of our program that are all the pin configurations */

// RFID MFRC522 tag reader pin configurations
#define SS_PIN A3
#define RST_PIN A2

// LED pin configurations
#define GREEN_LED A0
#define RED_LED A1

// motor pin configurations
#define MOTOR_EN_PIN 3

// switch demultiplexer/shift register pin configurations
#define SWITCH_A 8
#define SWITCH_B 9
#define SWITCH_C 10
#define SWITCH_INPUT 11

// motors multiplexer/shift register pin configurations
// SER (Serial Input) pin to feed data into shift register a bit at a time
#define MOTORS_SER 5
// RCLK (Register Clock/Latch) when this pin is set to HIGH, values in Shift Register are copied into Storage Register, therefore we can put this data to output
#define MOTOR_RCLK 12
// SRCLK (Shift Register Clock) is pin for shifting bits into the Shift Register. Bits are transferred in on the rising edge of the clock
#define MOTOR_SRCLK 13

// definition of Arduino specific HIGH and LOW values
#define LOW 0x0
#define HIGH 0x1

// When using Analog pins for output, digital pin HIGH is equal to 255 and digital pin LOW is equal to 0
#define ANALOG_LOW 0
#define ANALOG_HIGH 255

// Ultrasonic sensors pin configurations
#define ECHO_PIN_S1 6 // sensor 1 echo
#define TRIG_PIN_S1 7 // sensor 1 trig
#define ECHO_PIN_S2 4 // sensor 2 echo
#define TRIG_PIN_S2 2 // sensor 2 trig


// RFID reader specific instance and types
MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance.
MFRC522::MIFARE_Key key; // Crytpokey for accessing blocks
MFRC522::StatusCode status; // status type 

// --- Othe constants for program ---

/* Set the block to which we want to write data */
/* Be aware of Sector Trailer Blocks */
int blockNum = 2;  

/* Create another array to read data from Block */
/* Legthn of buffer should be 2 Bytes more than the size of Block (16 Bytes) */
byte bufferLen = 18;
byte readBlockData[18];

/* Create an array of 16 Bytes and fill it with data */
/* This is the actual data which is going to be written into the card */
// we have types for green, PMD, paper, residual and dangerous waste
// data represeents the type of the garbage in bytes
byte blockDataTypeDefault [16] = {"0---------------"};
//byte blockDataGreen [16] = {"0---------------"}; // probably don't need these anymore
//byte blockDataPMD [16] = {"1---------------"};
//byte blockDataPaper [16] = {"2---------------"};
//byte blockDataResidual [16] = {"3---------------"};
//byte blockDataDangerous [16] = {"4---------------"};

// bytes representing tyoes of garbage that would be inseted into byte array of that garbage type 
byte GREEN = 0x00;
byte PMD = 0x01;
byte PAPER = 0x02;
byte RESIDUAL = 0x03;
byte DANGEROUS = 0x04;

// specifications for distances to different sections so that ultrasonic sensor can detect the section that the garbage was put into. Range consist of low value and high value while the section is the range between these numbers
// region for green
int greenSectionLow = 15;
int greenSectionHigh = 24;

// region for PMD
int pmdSectionLow = 10;
int pmdSectionHigh = 18;

// region for pape
int paperSectionLow = 4;
int paperSectionHigh = 12;

// region for residual
int residualSectionLow = 25;
int residualSectionHigh = 30;

// region for dangerous
int dangerousSectionLow = 2;
int dangerousSectionHigh = 12;

//  distances to be measured without having any object inside the bin. This is needed as threshold to detect when something is inserted into the bin
int emptyBinMeasurementSensor1 = 34; // empty bin distance for ultrasonic sensor 1
int emptyBinMeasurementSensor2 = 4; // empty bin distance for ultrasonic sensor 1


// motor configuration constants 
int runMotorInOneDirectionDuration = 200 ; // time to run motor only in one direction duration in ms
int testRunMotorDelay = 100;
int testMotorStrength = 60;

// tag constants
bool tagFound = false;

// motor rotation direction constants
// the bit at index i nees to be 1 and others 0 if we want to set only one motor to turn in some direction. As there are 2 pins for each motor, then the bits in decimal presentation grow by power of 2. 
/**
 * for each of the motors, one of the bit codes 10 and 01 turns motor one way and one of other codes 10 and 01 turns it another away
 */
byte motor1Clockwise = 2;
byte motor1CounterClockwise = 1;
byte motor2Clockwise = 4;
byte motor2CounterClockwise = 8;
byte motor3Clockwise = 16;
byte motor3CounterClockwise = 32;

// one byte that corresponds to the configuration, which motor to run in which direction
// with 0 we don't run any motor
byte motors = 0;

ezButton limitSwitch(2);

// duration for ultrasonic sensor
int duration = 0;
// distance for storing result of ultrasonic sensor
int distance = 0;

//test constants
int testMotorMoveDuration = 200;


void setup() {
  
  // SETUP code to configure all types of pins (INPUT/OUTPUT)

  // print information to the Seial
  Serial.begin(9600);   // Initiate a serial communication
  while(!Serial); // uncomment this line of code for dev
  
  limitSwitch.setDebounceTime(50); // set debounce time to 50 milliseconds
  
  SPI.begin();      // Initiate  SPI bus for RFID reader
  mfrc522.PCD_Init();   // Initiate MFRC522
  Serial.println("Approximate tag to the reader...");
  Serial.println();

  // Ultrasonic sensor setup
  pinMode(TRIG_PIN_S1, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(ECHO_PIN_S1, INPUT); // Sets the echoPin as an INPUT
  pinMode(TRIG_PIN_S2, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(ECHO_PIN_S2, INPUT); // Sets the echoPin as an INPUT

  // LEDs setup
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  // switches shifter setup
  pinMode(SWITCH_A, OUTPUT);
  pinMode(SWITCH_B, OUTPUT);
  pinMode(SWITCH_C, OUTPUT);
  pinMode(SWITCH_INPUT, INPUT);

  // motorsh shifer 74HC595 pins setup 
  pinMode(MOTORS_SER, OUTPUT);
  pinMode(MOTOR_RCLK, OUTPUT);  
  pinMode(MOTOR_SRCLK, OUTPUT);
  pinMode(MOTOR_EN_PIN, OUTPUT);

  /* Prepare the ksy for authentication */
  /* All keys are set to FFFFFFFFFFFFh at chip delivery from the factory */
  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }
}


// main loop of program that goes to main function of the program
void loop() {
  readDistanceLoop();
  
  delay(3000); // delay to be waited after chip has been read to start reading new chip 
}

// read data from specified block of data from RFID chip
void ReadDataFromBlock(int blockNum, byte readBlockData[]) 
{
  /* Authenticating the desired data block for Read access using Key A */
  byte status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, blockNum, &key, &(mfrc522.uid));

  if (status != MFRC522::STATUS_OK)
  {
     Serial.print("Authentication failed for Read: ");
     Serial.println(mfrc522.GetStatusCodeName(status));
     return;
  }
  else
  {
    Serial.println("Authentication success");
  }

  /* Reading data from the Block */
  status = mfrc522.MIFARE_Read(blockNum, readBlockData, &bufferLen);
  if (status != MFRC522::STATUS_OK)
  {
    Serial.print("Reading failed: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  }
  else
  {
    Serial.println("Block was read successfully");  
  }
  
}

void readCardLoop() {
  while (true) {
    readCard(); 
      delay(1000);
  }
}

byte readCard() {
  if ( ! mfrc522.PICC_IsNewCardPresent()) // if new card is not present near the reader, we return as we want to scan for new cards again
  {
    Serial.print("Here");
    return;
  }
  // Select one of the cards
  if ( ! mfrc522.PICC_ReadCardSerial())  // if we can't read the car, we return as we want to scan for new cards again
  {
    return;
  }

  /*** [1] Show some basic information of the presented card such as UID and PICC type ***/
  Serial.print(F("Card UID:")); dump_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size); 
  Serial.println();
  Serial.print(F("PICC type: "));
  MFRC522::PICC_Type piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);
  Serial.println(mfrc522.PICC_GetTypeName(piccType));

/* Read data from the same block */
   Serial.print("\n");
   Serial.println("Reading from Data Block...");
   ReadDataFromBlock(blockNum, readBlockData);
   /* Print the data read from block */
   Serial.print("\n");
   Serial.print("Data in Block:");
   Serial.print(blockNum);
   Serial.print(" --> ");
   byte readRes = readBlockData[0]; // as we store data in first byte of the array of bytes on the block on the tag, then we want to also read this byte = 8 bits
   Serial.println("First byte from data in the block:");
   Serial.println(readRes);
   for (int j=0 ; j<16 ; j++)
   {
     Serial.write(readBlockData[j]);
   }
   mfrc522.PICC_HaltA(); // forgot this current tag so that we can scan it again later
   mfrc522.PCD_StopCrypto1(); // to authenticate new tag again after reading this tag
   
   tagFound = true;
   return readRes; // we return the first byte of the specified block where we also wrote data to
}


// main function to handle all features of our product in correct sequence: 1). detect distance of garbage, 2). read card 3). identify if type matches section 4). give user feedback back 
void readDistanceLoop() {
  byte tagRes;
  int dist1 = emptyBinMeasurementSensor1;
  int dist2 = emptyBinMeasurementSensor2;
  int referenceValue1 = emptyBinMeasurementSensor1 - 1; // we leave room for some error in reference values as ultrasonic is not totally precise in every reading
  int referenceValue2 = emptyBinMeasurementSensor2 - 1;
  
  while (dist1 >= referenceValue1 && dist2 >= referenceValue2) { // read distance from both sensors until the object is not detected within the range of the sections
    dist1 = readDistanceInFromSensor(1);
    dist2 = readDistanceInFromSensor(2);
    delay(200);
  }

  // in which sensor did we detect the object
  int nrOfUltrasonicSensorObjectDetected = 1;

  if (dist2 < referenceValue2) {
    nrOfUltrasonicSensorObjectDetected = 2;
  }
  
  int avgDist = getAverageDistance(10, 100, nrOfUltrasonicSensorObjectDetected);

  // after fixing distance, rotate robot arm here to move the objects to the mechanism below

  // first we need to set the top flat to be on turned counter clock wise so that we ensure that it stays exactly horizontal when the game piece falls onto it
  motors = motor1CounterClockwise;
  waitUntilSwitchIsTurnedOn(1);

  // then we move upper rotating arm until it pushes  
  doOneRotationalMovementWithTopArm();

  while (!tagFound) { // start getting input from reader and get it with loop until card is detected
    tagRes = readCard();
    delay(500);
  }

  // check if the section matches with type of the chip 
  bool ifCorrect = detectIfCorrectWaste(avgDist, tagRes, nrOfUltrasonicSensorObjectDetected);

  if (ifCorrect) { // if type matches then move trash to corresponding section, else move it to other section
    handleCorrectWaste();
  } else {
    handleIncorrectWaste();
  }

  
  tagFound = false;
  dist1 = emptyBinMeasurementSensor1;
  dist2 = emptyBinMeasurementSensor2;
}

int readDistanceInFromSensor(int sensorNr) {
  if (sensorNr == 1) {
    // Clears the trigPin condition
    digitalWrite(TRIG_PIN_S1, LOW);
    delayMicroseconds(2);
    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(TRIG_PIN_S1, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN_S1, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(ECHO_PIN_S1, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2; // 340m/s = 340microm/s Speed of sound wave divided by 2 (go and back)
    // Displays the distance on the Serial Monitor
    Serial.print("Distance from sensor 1: ");
    Serial.print(distance);
    Serial.println(" cm");
    return distance;
  } else {
    // Clears the trigPin condition
    digitalWrite(TRIG_PIN_S2, LOW);
    delayMicroseconds(2);
    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(TRIG_PIN_S2, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN_S2, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(ECHO_PIN_S2, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2; // 340m/s = 340microm/s Speed of sound wave divided by 2 (go and back)
    // Displays the distance on the Serial Monitor
    Serial.print("Distance from sensor 2: ");
    Serial.print(distance);
    Serial.println(" cm");
    return distance;
  }
}

void doOneRotationalMovementWithTopArm() {
  motors = motor3Clockwise | motor1CounterClockwise; // at the same time when moving rotating arm, we must as well ensure that our top flap piece is always kept horizontal
  moveUntilSwitchOn(motors, 6); // with this movement we push the object off the platform
  delay(200);
  motors = motor3CounterClockwise | motor1CounterClockwise;
  moveUntilSwitchOn(motors, 7); // with this movement we move that arm back to its original position
}

bool detectIfCorrectWaste(int dist, byte type, int numberOfSensor) {
  bool res = false;
  
  
  switch (type) {
    case 0:
      if ((dist <= greenSectionHigh) && (dist >= greenSectionLow) && numberOfSensor == 1) {
        Serial.println("correct");
        res = true;
      } else {
        Serial.println("dist");
        Serial.println(dist);
        Serial.println("incorrect");
        res = false;
      }
      break;
    case 1:
      if ((dist <= pmdSectionHigh) && (dist >= pmdSectionLow)&& numberOfSensor == 1) {
        Serial.println("correct");
        res = true;
      } else {
        Serial.println("incorrect");
        res = false;
      }
      break;
    case 2:
      if ((dist <= paperSectionHigh) && (dist >= paperSectionLow) && numberOfSensor == 1) {
        Serial.println("correct");
        res = true;
      } else {
        Serial.println("incorrect");
        res = false;
      }
      break;
    case 3:
      if ((dist <= residualSectionHigh) && (dist >= residualSectionLow) && numberOfSensor == 1) {
        Serial.println("correct");
        res = true;
      } else {
        Serial.println("incorrect");
        res = false;
      }
      break;
    case 4:
      if ((dist <= dangerousSectionHigh) && (dist >= dangerousSectionLow) && numberOfSensor == 2) {
        Serial.println("correct");
        res = true;
      } else {
        Serial.println("incorrect");
        res = false;
      }
      break;
  }
  return res;
}

void handleCorrectWaste() {
  turnGreen(true);
  moveMotorsForCorrectChoice();
  turnGreen(false);
}

void handleIncorrectWaste() {
  turnRed(true);
  moveMotorsForIncorrectChoice();
  turnRed(false);
}

void moveMotorsForCorrectChoice() {
  analogWrite(MOTOR_EN_PIN, 30);
  motors = 0; // set all motors tuned off initially
  updateShiftRegister();
  motors = motor1CounterClockwise; // move 
  waitUntilSwitchIsTurnedOn(1);
  delay(1000);
  motors = motors | motor2CounterClockwise;
  waitUntilSwitchIsTurnedOn(3);
  delay(200);
  motors = motor2CounterClockwise | motor1Clockwise;
  waitUntilSwitchIsTurnedOn(5);
  delay(3000);
  moveUntilSwitchOn(motor1CounterClockwise, 1);
  delay(200);
  moveUntilSwitchOn(motor2Clockwise, 2);
  delay(20000);
}

void moveMotorsForIncorrectChoice() {
  analogWrite(MOTOR_EN_PIN, 30);
  motors = 0; // set all motors tuned off initially
  updateShiftRegister();
  motors = motor1CounterClockwise;
  waitUntilSwitchIsTurnedOn(1);
  delay(1000);
  motors = motor2Clockwise;
  waitUntilSwitchIsTurnedOn(2);
  delay(200);
  motors = motor2Clockwise | motor1Clockwise;
  waitUntilSwitchIsTurnedOn(4);
  delay(1000);
  moveUntilSwitchOn(motor1CounterClockwise, 1);
  delay(50000);
}

// turn motors until they push the switches and we know that the component is in its starting positon
void calibrateMotorsToStartingPositions() {
  analogWrite(MOTOR_EN_PIN, 30);
  motors = 0; // set all motors tuned off initially
  updateShiftRegister(); // by calling this function, we actually forward bit values corresponding to pins to output
  moveUntilSwitchOn(motor1CounterClockwise, 1);
  delay(200);
  moveUntilSwitchOn(motor1Clockwise, 4);
  delay(200);
  moveUntilSwitchOn(motor1CounterClockwise, 1);
  delay(200);
  moveUntilSwitchOn(motor2Clockwise, 2);
  delay(3000);
};

/*
 * updateShiftRegister() - This function sets the latch pin to low, then calls the Arduino function 'shiftOut' to shift out contents of variable 'motors' in the shift register before putting the pin of the Latch high again to actually output the configuration of the motor pins that need to run.
 */
void updateShiftRegister()
{
   digitalWrite(MOTOR_RCLK, LOW);
   shiftOut(MOTORS_SER, MOTOR_SRCLK, MSBFIRST, motors); // MSBFIRST stands for most significant bit first
   digitalWrite(MOTOR_RCLK, HIGH);
}

// move motor in direction specified with motorRotation until switch with switchNr is ON and then stop all the motors
void moveUntilSwitchOn(byte motorRotation, int switchNr) {
  motors = motorRotation;
  updateShiftRegister();
  readSwitchLoopUntilChange(switchNr, 0, 1);
  delay(100);
  motors = 0;
  updateShiftRegister();
}

// function to read the current byte value in motors into the Shift Register and then wait for the specified switch to turn ON. If switch is ON, then we just return but don't clean motor pins
void  waitUntilSwitchIsTurnedOn(int switchNr) {
  updateShiftRegister();
  readSwitchLoopUntilChange(switchNr, 0, 1);
}

// turn on green LED id we set state to true, otherwise turn led off
void turnGreen(bool state) {
  if (state) {
    analogWrite(GREEN_LED, 102); 
  } else {
    analogWrite(GREEN_LED, 0);
  }
}

// turn on red LED id we set state to true, otherwise turn led off
void turnRed(bool state) {
  if (state) {
    analogWrite(RED_LED, 102); 
  } else {
    analogWrite(RED_LED, 0);
  }
}

//void readSwitchLoop() {
//  while (true) {
//   limitSwitch.loop(); // MUST call the loop() function first
//
//  if(limitSwitch.isPressed())
//    Serial.println("The limit switch: UNTOUCHED -> TOUCHED");
//
//  if(limitSwitch.isReleased())
//    Serial.println("The limit switch: TOUCHED -> UNTOUCHED");
//
//  int state = limitSwitch.getState();
//  if(state == HIGH)
//    Serial.println("The limit switch: UNTOUCHED");
//  else
//    Serial.println("The limit switch: TOUCHED"); 
//
//
//  delay(100);
//  }
//}


// read the value of switch specified with variable switchNr until it has chabged from initialState (LOW - 0 or HIGH - 1) to finalState (LOW - 0 or HIGH - 1)
void readSwitchLoopUntilChange(int switchNr, int initialState, int finalState) {
  int resVal = initialState;
  while (resVal != finalState) {
    resVal = switchesRecieve(switchNr);
    Serial.println("Read swtich nr ");
    Serial.print(switchNr);
    Serial.print(" value :");
    Serial.println(resVal);
    delay(50);
  }
}

// return 0 if switch of number specified with switchNr is not pressed and return 1 if that switch is pressed down
boolean switchesRecieve(int switchNr) {
  int switchVal = 0;
  
  switch (switchNr) { // depending on switch index, the corresponding pins are set to LOW and HIGH in bit format of the index, to read the data of corresponding input pin
      case 1: // index of switch nr 1 is 0, so it is in bit format 000
        digitalWrite(SWITCH_A, LOW);
        digitalWrite(SWITCH_B, LOW);
        digitalWrite(SWITCH_C, LOW);
        break;
      case 2: // index of switch nr 2 is 1, so it is in bit format 001
        digitalWrite(SWITCH_A, HIGH);
        digitalWrite(SWITCH_B, LOW);
        digitalWrite(SWITCH_C, LOW);
        break;
      case 3: // index of switch nr 3 is 2, so it is in bit format 010
        digitalWrite(SWITCH_A, LOW);
        digitalWrite(SWITCH_B, HIGH);
        digitalWrite(SWITCH_C, LOW);
        break;
      case 4: // index of switch nr 4 is 3, so it is in bit format 011
        digitalWrite(SWITCH_A, HIGH);
        digitalWrite(SWITCH_B, HIGH);
        digitalWrite(SWITCH_C, LOW);
        break;
      case 5: // index of switch nr 5 is 4, so it is in bit format 100
        digitalWrite(SWITCH_A, LOW);
        digitalWrite(SWITCH_B, LOW);
        digitalWrite(SWITCH_C, HIGH);
        break;
      case 6: // index of switch nr 6 is 5, so it is in bit format 101
        digitalWrite(SWITCH_A, HIGH);
        digitalWrite(SWITCH_B, LOW);
        digitalWrite(SWITCH_C, HIGH);
        break;
      case 7: // index of switch nr 7 is 6, so it is in bit format 110
        digitalWrite(SWITCH_A, LOW);
        digitalWrite(SWITCH_B, HIGH);
        digitalWrite(SWITCH_C, HIGH);
        break;
   }
   switchVal = digitalRead(SWITCH_INPUT);
   return switchVal;
}


// ----------------- HELPER FUNCTIONS TO CONFIGURING THE PRODUCT (these functions are not used in the final execution of program) --------------

// function to write byte array data, which in our case is byte corresponding to type of garbage into the selected block on RFID tag
void WriteDataToBlock(int blockNum, byte blockData[]) 
{

  Serial.println("Write data to card");
  Serial.println(blockData[0]);
  /* Authenticating the desired data block for write access using Key A */
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, blockNum, &key, &(mfrc522.uid));
  if (status != MFRC522::STATUS_OK)
  {
    Serial.print("Authentication failed for Write: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  }
  else
  {
    Serial.println("Authentication success");
  }

  
  /* Write data to the block */
  status = mfrc522.MIFARE_Write(blockNum, blockData, 16);
  if (status != MFRC522::STATUS_OK)
  {
    Serial.print("Writing to Block failed: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  }
  else
  {
    Serial.println("Data was written into Block successfully");
  }
  
}

// overwrite the first byte of default byte array data with byte corresponding to specific type of garbage
void writeTypeDataToChip(byte typeByte) {
  blockDataTypeDefault[0] = typeByte; // overwrite the byte
  Serial.print("\n");
  Serial.println("Writing to Data Block...");
  WriteDataToBlock(blockNum, blockDataTypeDefault); // write block of bytes that correspond to this type of grabge to the chip
}

void writeCard() {
  if ( ! mfrc522.PICC_IsNewCardPresent()) 
  {
    Serial.println("Here");
    return;
  }
  // Select one of the cards
  if ( ! mfrc522.PICC_ReadCardSerial()) 
  {
    return;
  }

  /*** [1] Show some basic information of the presented card ***/
  Serial.print(F("Card UID:")); dump_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size); 
  Serial.println();
  Serial.print(F("PICC type: "));
  MFRC522::PICC_Type piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);
  Serial.println(mfrc522.PICC_GetTypeName(piccType));

  // uncomment the line of code from next lines that represents the type of garbage that you wish to write onto the RFID tag
//  writeTypeDataToChip(GREEN);
  writeTypeDataToChip(PMD);
//  writeTypeDataToChip(PAPER);
//  writeTypeDataToChip(RESIDUAL);
//  writeTypeDataToChip(DANGEROUS);
}

// loop to constantly keep encoding RFID tags with specified data
void writeCardLoop() {
  while (true) {
    writeCard();
    delay(1000);
  };
}

//  distances that correspond to actual section locations of all garbage types, return values of this function will be set to constant values for regions of specific types manually 
void calibrateSectionDistances(int sensorNr) {
  Serial.println("Starting calibration of ultrasonic sensor");
  Serial.println("Measuring base distance 1, please remove all objects...");
  int baseDistance = getAverageDistance(20, 200, 1);
  baseDistance = baseDistance - 2; // to discard measurement inaccuracy, we make basedistance interval shorter
  Serial.println("Base distance 1 successfully measured:");
  Serial.println("Measuring base distance 1, please remove all objects...");
  int baseDistance2 = getAverageDistance(20, 200, 2);
  baseDistance2 = baseDistance2 - 2; // to discard measurement inaccuracy, we make basedistance interval shorter
  Serial.println("Base distance 2 successfully measured:");
  
  Serial.println(baseDistance2);
  Serial.print("cm");

  int measurements [10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  const char* types[5] = { "Green", "PMD", "paper", "residual", "dangerous"};
  
  int indexInMeasurements = 0;

  for (int i = 0; i < 5; i ++) {
    Serial.println("Measuring ");
    Serial.print(types[i]);
    Serial.println("Place object to its lowest postion within 5 seconds:");
    countDown(5);
    Serial.println("Measuring lowest point ...");
    int sensorNr = 1;
    if (i > 2) {
      sensorNr = 2;
    }
    int lowestPoint = getLowestOfDistances(20, 200, sensorNr);
    measurements[indexInMeasurements] = lowestPoint - 1; // to discard measurement inaccuracy, we substract from lowest
    Serial.println("Lowest point successfully measured:");
    Serial.print(lowestPoint - 1);
    Serial.print("cm");
    indexInMeasurements = indexInMeasurements + 1;
    Serial.println("Place object to its highest postion within 5 seconds:");
    countDown(5);
    Serial.println("Measuring highest point for ...");
    int highestPoint = getBiggestDistance(20, 200, sensorNr);
    measurements[indexInMeasurements] = highestPoint + 1; // to discard measurement inaccuracy, we add to highest
    Serial.println("Highest point successfully measured:");
    Serial.print(highestPoint + 1);
    Serial.print("cm");
    indexInMeasurements = indexInMeasurements + 1;
  }
  Serial.println("==== CALIBRATING SECTION LOCATIONS FINISHED SUCCESSFULLY ==== ");
}

// measure base distance if there is nothing put inside the garbage can
void measureEmptyGarbageBinDistance() {
  Serial.println("Starting calibration of ultrasonic sensor");
  Serial.println("Measuring base distance, please remove all objects...");
  int baseDistance = getAverageDistance(20, 200,1);
  baseDistance = baseDistance - 2; // to discard measurement inaccuracy, we make basedistance interval shorter
  Serial.println("Base distance successfully measured:");
  Serial.println(baseDistance);
  Serial.print("cm");
}

// ------------- UTIL FUNCTION USED BY OTHER BIGGER FUNCTIONS FOR COMPUTATIONS -----------------------

// return average distance recorded over list of distance measurements by ultrasonic sensor
int getAverageDistance(int count, int timeDelay, int nrOfSensor) {
  int sum = 0;
  int res = 0;
  for (int i = 0; i < count; i++) {
    sum = sum + readDistanceInFromSensor(nrOfSensor);
    delay(timeDelay);
  }
  res = round(sum / count);
  return res;
}

// return smallest distance recorded over list of distance measurements by ultrasonic sensor
int getLowestOfDistances(int count, int timeDelay, int sensorNr) {
  int d[count];

  for (int i = 0; i < count; i ++) {
    d[i] = readDistanceInFromSensor(sensorNr);
    delay(timeDelay);
  }

  int currMin = d[0];

  for (int i = 0; i < count; i ++) {
    if (d[i] < currMin) {
      currMin = d[i];  
    }
  };

  return currMin;
}

// return biggest distance recorded over list of distance measurements by ultrasonic sensor
int getBiggestDistance(int count, int timeDelay, int sensorNr) {
  int d[count];

  for (int i = 0; i < count; i ++) {
    d[i] = readDistanceInFromSensor(sensorNr);
    delay(timeDelay);
  }

  int currMax = d[0];

  for (int i = 0; i < count; i ++) {
    if (d[i] > currMax) {
      currMax = d[i];  
    }
  };

  return currMax;
}

/*** Helper function to dump a byte array as hex values to Serial Monitor. ***/
void dump_byte_array(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}

int countDown(int countSec) {
  for (int i = countSec; i > 0; i--) {
    Serial.println(i);
    Serial.print(" secounds remaining");
    delay(1000);
  }
}


// ----------------- FUNCTIONS FOR TESTING DIFFERNT COMPONENTS AND UNITS --------------

// test reading values from ultrasonic sensor in loop
void testReadDistanceLoop(int sensorNr) {
  while (true) {
    int dist = readDistanceInFromSensor(sensorNr); 
    delay(1000);  
  }
}

// test reading RFID tag values from tag reader in loop
void testReadTagLoop() {
  while (true) {
    readCard();
    delay(1000);
  }
};

// test running motors in different directionss
void testRunMotor() {
  while (true) {
    moveUntilSwitchOn(motor3Clockwise, 7);
    delay(50000);
  }
}
