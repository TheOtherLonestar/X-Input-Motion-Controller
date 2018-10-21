#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>

int16_t StickOne_X_Val;
int16_t StickOne_Y_Val;
int16_t StickTwo_X_Val;
int16_t StickTwo_Y_Val;
int8_t TriggerLeftVal;
uint8_t TriggerRightVal;
int8_t buttonStatus[11];
int8_t dPadStatus[4];


int buttonREADALL[] = {4, 3, 2};
/*
  //button pin assignments


  RB_Button 2
  L3_Button 3
  R3_Button 4

*/


//General Declarations
#define pinOBLED 13  //Onboard LED pin

//trigger array to update both triggers at once
#define TriggerStatus[2]

//trigger pin assignments
#define TriggerLeft
#define TriggerRight 5

//Stick Update
#define StickOne_X A0
#define StickOne_Y A1
//JoyStick Variables
int JMINX;//min deadspot X position for Joydeadspace calc
int JMAXX;//max deadspot X position for Joydeadspace calc
int JAVX;//average x position at home
int JMINY;//min deadspot Y position for Joydeadspace calc
int JMAXY;//max deadspot y position for Joydeadspace calc
int JAVY;//average y position at home
int Joydeadspace = 5;//size of deadspot for joystick

int16_t StickOne_X_Val_Temp;
int16_t StickOne_Y_Val_Temp;

//IMU variables
int StickTwo_X_Val_Temp;
int StickTwo_Y_Val_Temp;
int minroll = -60;
int maxroll = 60;
int minpitch = -40;
int maxpitch = 60;

long previousMillis;
long interval = 50;
int deadmin = -15;
int deadmax = 15;
/* Assign a unique ID to the sensors */
Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

/**************************************************************************/
/*!
    @brief  Initialises all the sensors used by this example
*/
/**************************************************************************/

//during setup this is called to set button and dpad pins to input pullup
void setupPins()
{

  for (int x = 0; x < 11; x++) {
    pinMode(buttonREADALL[x], INPUT_PULLUP);
  }
  pinMode(TriggerRight, INPUT_PULLUP);

  digitalWrite(pinOBLED, LOW);

}
void initSensors()
{
  if (!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while (1);
  }
  if (!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1);
  }
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void setup(void)
{
  setupPins();

  Serial.begin(115200);
  Serial.println(F("Adafruit 9 DOF Pitch/Roll/Heading Example")); Serial.println("");
  /* does a test on IMU and returns prints to serial if it is not working */
  initSensors();

  //set home position and Joydeadspace for joystick
  delay(1000);
  JAVX = analogRead(StickOne_X); //get average value of x stick at home
  JMINX = JAVX - Joydeadspace;
  JMAXX = JAVX + Joydeadspace;
  JAVY = analogRead(StickOne_Y); //get average value of y stick at home
  JMINY = JAVY - Joydeadspace;
  JMAXY = JAVY + Joydeadspace;
}


void loop(void)
{
  motionVal();//calls IMU function
  buttonRead();//reads all the buttons including D Pad
  JoystickRead();//reads the joystick
  SendSerial();//Sends all the joystick and button information through serial

}


void motionVal()
{
  //IMU STUFF
  sensors_event_t accel_event;
  sensors_vec_t   orientation;

  /* Calculate pitch and roll from the raw accelerometer data */
  accel.getEvent(&accel_event);

  /* Use the new fusionGetOrientation function to merge accel/mag data */
  if (dof.accelGetOrientation(&accel_event, &orientation))
  {
    int ROLL = orientation.pitch;
    int PITCH = orientation.roll;

    StickTwo_X_Val_Temp = constrain(ROLL, minroll, maxroll);
    StickTwo_Y_Val_Temp = constrain(PITCH, minpitch, maxpitch);

  }




  if (StickTwo_X_Val_Temp < deadmin || StickTwo_X_Val_Temp > deadmax) {
    StickTwo_X_Val = map(StickTwo_X_Val_Temp, minroll, maxroll, -32767, 32767);
  }
  else if (StickTwo_X_Val_Temp > deadmin && StickTwo_X_Val_Temp < deadmax)
  {
    StickTwo_X_Val = 0;
  }





  if (StickTwo_Y_Val_Temp < deadmin || StickTwo_Y_Val_Temp > deadmax) {
    StickTwo_Y_Val = map(StickTwo_Y_Val_Temp, minpitch, maxpitch, -32767, 32767);
  }
  else if (StickTwo_Y_Val_Temp > deadmin && StickTwo_Y_Val_Temp < deadmax)
  {
    StickTwo_Y_Val = 0;
  }
  StickTwo_Y_Val = StickTwo_Y_Val * -1;
  StickTwo_X_Val = StickTwo_X_Val * -1;
}


void JoystickRead()
{


  int TriggerTemp = digitalRead(TriggerRight);
  if (TriggerTemp == 0) {
    TriggerRightVal = 255;
  }
  else {
    TriggerRightVal = 0;
  }




  StickOne_X_Val_Temp = analogRead(StickOne_X);
  StickOne_Y_Val_Temp = analogRead(StickOne_Y);



  //map values of left stick
  if (StickOne_X_Val_Temp > JMAXX) {
    StickOne_X_Val = map(StickOne_X_Val_Temp, JMAXX, 1023, 0, 32767);
  }
  else if (StickOne_X_Val_Temp < JMINX) {
    StickOne_X_Val = map(StickOne_X_Val_Temp, JMINX, 0, 0, -32767);
  }

  else if (StickOne_X_Val_Temp < JMAXX && StickOne_X_Val_Temp > JMINX)
  {
    StickOne_X_Val = 0;
  }

  if (StickOne_Y_Val_Temp > JMAXY) {
    StickOne_Y_Val = map(StickOne_Y_Val_Temp, JMAXY, 1023, 0, 32767);
  }
  else if (StickOne_Y_Val_Temp < JMINY) {
    StickOne_Y_Val = map(StickOne_Y_Val_Temp, JMINY, 0, 0, -32767);
  }

  else if (StickOne_Y_Val_Temp < JMAXY && StickOne_Y_Val_Temp > JMINY)
  {
    StickOne_Y_Val = 0;
  }
  StickOne_X_Val = StickOne_X_Val * -1;

}
void buttonRead()
{
  //readbuttons
  for (int x = 0; x < 3; x++) {
    int temp = digitalRead(buttonREADALL[x]);
    if (temp == LOW) {
      buttonStatus[x] = 1;
    }
    else if (temp == HIGH) {
      buttonStatus[x] = 0;
    }
  }



}
void SendSerial()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    Serial.print('<');
    Serial.print(StickOne_X_Val);
    Serial.print(',');
    Serial.print(StickOne_Y_Val);
    Serial.print(',');
    Serial.print(StickTwo_X_Val);
    Serial.print(',');
    Serial.print(StickTwo_Y_Val);
    Serial.print(',');


    for (int x = 0; x < 3; x++) {
      Serial.print(buttonStatus[x]);
      Serial.print(',');
    }
    Serial.print(TriggerRightVal);



    Serial.print('>');
    Serial.println();


  }
}
