#include <Arduino.h>

#include <NmraDcc.h>

#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_PWMServoDriver.h>
#include <TimerOne.h>
#include "Adafruit_WS2801.h"
#include "SPI.h" 

#define SERVO 0
#define RELAY 1

//#define measureMode

#ifdef measureMode
  int loopCounter = 0;
  uint32_t loopTimer = millis();
#endif

#define THROWN 0
#define CLOSED 1

#define SERVOMIN 150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600 // this is the 'maximum' pulse length count (out of 4096)

#define colorDark 0x00000000    //all LED's dark
#define relayON  0x0000007F     //50% blue to indicate active coil on relay
#define relayOFF 0x00000000     //same as dark for when relay is off
#define relayThrown 0x001F0000  //relay is off and on thrown position
#define relayClosed 0x00001F00  //relay is off in closed position
#define servoMinPos 0x00001F00  //servo is in minimum position
#define servoMaxPos 0x001F0000  //servo is in maximum position
#define servoMove 0x00050500    //servo is currently moving
#define aspectHalt 0x007F0000   //signal aspect color for halt (red)
#define aspectSlow 0x005F0F00   //signal aspect color yellow (may also be used for blinking)
#define aspectClear 0x00007F00  //signal aspect color green
#define level1Col 0x000F0000    //red used for level crossing blink light

#define blinkLEDInterval 500    //duration of blink cycle (half of it, it is symmetrical)

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_MCP23017 mcp;

NmraDcc Dcc;

const int DccIntrpt = 0;
const int DccDataPin = 2;
const int DccAckPin = 3; //not connected
//const int LEDDataPin = 6; //comment out if using SPI
//const int LEDClockPin = 7; //comment out if using SPI

#define numChannels 24
#define numPixels 50

Adafruit_WS2801 thisStrip = Adafruit_WS2801(numPixels);
uint32_t pixCopy[numPixels];
bool ledChg = false;

uint32_t ledBlinkTimer = millis();
uint32_t timeElapsed = 0;
bool blinkFlag = false;
float faderCtr = 0;

typedef void (*ledProcess) (void *);

typedef struct
{
  uint16_t dccAddr;
  uint8_t driveType;
  ledProcess ledProc;
  uint8_t ledPos[5];
  uint16_t portA;
  uint16_t portB;
  uint16_t minPos;
  uint16_t maxPos;
  uint16_t targetPos;
  uint16_t currPos;
  uint16_t moveDelayCL;
  uint16_t moveDelayTH;
  uint32_t nextMove;
//  uint32_t currColor;
} myServo;

void relayFct(void * thisServo);
void servoFct(void * thisServo);
void aspectFct(void * thisServo);
void levelCrossingRelay(void * thisServo);
void levelCrossingServo(void * thisServo);

myServo servoArray[numChannels] = {
    {24, SERVO, &servoFct, {1,0,0,0,0}, 2, 2, SERVOMIN, SERVOMAX, SERVOMIN, SERVOMIN + 1, 10, 0, 0},
    {25, SERVO, &servoFct, {2,0,0,0,0}, 1, 1, SERVOMIN, SERVOMAX, SERVOMIN, SERVOMIN + 1, 0, 0, 0},
    {50, SERVO, &servoFct, {3,0,0,0,0}, 0, 0, SERVOMIN, SERVOMAX, SERVOMIN, SERVOMIN + 1, 0, 10, 0},
    {51, SERVO, &servoFct, {4,0,0,0,0}, 3, 3, SERVOMIN, SERVOMAX, SERVOMIN, SERVOMIN + 1, 0, 0, 0},

    {10, SERVO, &levelCrossingServo, {5,0,0,45,46}, 4, 4, SERVOMIN, SERVOMAX, SERVOMIN, SERVOMIN + 1, 10, 10, 0},
    {11, SERVO, &servoFct, {6,0,0,0,0}, 5, 5, SERVOMIN, SERVOMAX, SERVOMIN, SERVOMIN + 1, 0, 0, 0},
    {16, SERVO, &servoFct, {7,0,0,0,0}, 6, 6, SERVOMIN, SERVOMAX, SERVOMIN, SERVOMIN + 1, 0, 0, 0},
    {17, SERVO, &servoFct, {8,0,0,0,0}, 7, 7, SERVOMIN, SERVOMAX, SERVOMIN, SERVOMIN + 1, 0, 0, 0},

    {18, SERVO, &servoFct, {9,0,0,0,0}, 8, 8, SERVOMIN, SERVOMAX, SERVOMIN, SERVOMIN + 1, 0, 0, 0},
    {23, SERVO, &servoFct, {10,0,0,0,0}, 9, 9, SERVOMIN, SERVOMAX, SERVOMIN, SERVOMIN + 1, 0, 0, 0},
    {33, SERVO, &servoFct, {11,0,0,0,0}, 10, 10, SERVOMIN, SERVOMAX, SERVOMIN, SERVOMIN + 1, 0, 0, 0},
    {34, SERVO, &servoFct, {12,0,0,0,0}, 11, 11, SERVOMIN, SERVOMAX, SERVOMIN, SERVOMIN + 1, 0, 0, 0},

    {10800, SERVO, &aspectFct, {15,0,0,0,0}, 12, 12, SERVOMIN, SERVOMAX, SERVOMIN, SERVOMIN + 1, 0, 0, 0},
    {10802, SERVO, &aspectFct, {16,0,0,0,0}, 13, 13, SERVOMIN, SERVOMAX, SERVOMIN, SERVOMIN + 1, 0, 0, 0},
    {10804, SERVO, &aspectFct, {17,0,0,0,0}, 14, 14, SERVOMIN, SERVOMAX, SERVOMIN, SERVOMIN + 1, 0, 0, 0},
    {10806, SERVO, &aspectFct, {18,0,0,0,0}, 15, 15, SERVOMIN, SERVOMAX, SERVOMIN, SERVOMIN + 1, 0, 0, 0},

    {61, RELAY, &relayFct, {21,22,23,0,0}, 0, 1, THROWN, CLOSED, CLOSED, THROWN, 200, 200, 0},
    {62, RELAY, &relayFct, {24,25,26,0,0}, 2, 3, THROWN, CLOSED, CLOSED, THROWN, 200, 200, 0},
    {63, RELAY, &relayFct, {27,28,29,0,0}, 4, 5, THROWN, CLOSED, CLOSED, THROWN, 200, 200, 0},
    {64, RELAY, &relayFct, {30,31,32,0,0}, 6, 7, THROWN, CLOSED, CLOSED, THROWN, 200, 200, 0},

    {65, RELAY, &relayFct, {33,34,35,0,0}, 8, 9, THROWN, CLOSED, CLOSED, THROWN, 200, 200, 0},
    {66, RELAY, &relayFct, {36,37,38,0,0}, 10, 11, THROWN, CLOSED, CLOSED, THROWN, 200, 200, 0},
    {67, RELAY, &relayFct, {39,40,41,0,0}, 12, 13, THROWN, CLOSED, CLOSED, THROWN, 4000, 1000, 0},
    {68, RELAY, &levelCrossingRelay, {42,43,44,47,48}, 14, 15, THROWN, CLOSED, CLOSED, THROWN, 200, 200, 0},
};

uint32_t Color(byte r, byte g, byte b)
{
  //Take the lowest 5 bits of each value and append them end to end
//  return( ((uint16_t)g & 0x1F )<<10 | ((uint16_t)b & 0x1F)<<5 | ((uint16_t)r & 0x1F));  for LPD8603 5bit color depth
  uint32_t retVal = (((uint32_t)r<<16) + ((uint32_t)g<<8) + (uint32_t)b);
  return retVal; 
}

uint32_t blinkColor(uint32_t posCol, uint32_t negCol=0, bool inPhase=true)
{
  if (blinkFlag ^ inPhase)
    return negCol;
  else
    return posCol;
}

uint32_t fadeColor(uint32_t posCol, uint32_t negCol=0, bool inPhase=true)
{
  float dimFactHi, dimFactLo;
  uint8_t faderExp = 8;
  if (negCol != 0)
    faderExp = 7;
  if (inPhase)
  {
    dimFactHi = (pow (2, (faderExp*faderCtr)) - 1)/255;
    dimFactLo = (pow (2, (faderExp*(1-faderCtr))) - 1)/255;
  }
  else
  {
    dimFactLo = (pow (2, (faderExp*faderCtr)) - 1)/255;
    dimFactHi = (pow (2, (faderExp*(1-faderCtr))) - 1)/255;
  }
  
  uint8_t rPos, gPos, bPos;
  rPos = round(((posCol & 0x00FF0000) >> 16) * dimFactHi) & 0xFF;
  gPos = round(((posCol & 0x0000FF00) >> 8) * dimFactHi) & 0xFF;
  bPos = round((posCol & 0x000000FF) * dimFactHi) & 0xFF;
  if (negCol != 0)
  {
    rPos = rPos + (round(((negCol & 0x00FF0000) >> 16) * dimFactLo) & 0xFF);
    gPos = gPos + (round(((negCol & 0x0000FF00) >> 8) * dimFactLo) & 0xFF);
    bPos = bPos + (round((negCol & 0x000000FF) * dimFactLo) & 0xFF);
  }
  return Color(rPos, gPos, bPos);
}

void setLED(uint16_t ledNr, uint32_t newCol)
{
  if (pixCopy[ledNr] != newCol)
  {
    pixCopy[ledNr] = newCol;
    thisStrip.setPixelColor(ledNr, newCol);
    ledChg = true;
  }
}

void relayFct(void * thisServo)  //this strip has color sequence b,g,r
{
  myServo * currServo = (myServo*)thisServo;
  uint32_t pos1Col=colorDark, pos2Col=relayOFF, pos3Col=colorDark;
  if (currServo->targetPos != currServo->currPos)
    pos2Col = relayON;
  else
    if (currServo->currPos == THROWN)
      pos1Col = relayThrown;
    else
      pos3Col = relayClosed;
  if (currServo->ledPos[0] > 0)
      setLED(currServo->ledPos[0]-1, pos1Col);
  if (currServo->ledPos[1] > 0)
      setLED(currServo->ledPos[1]-1, pos2Col);
  if (currServo->ledPos[2] > 0)
      setLED(currServo->ledPos[2]-1, pos3Col);
}

void servoFct(void * thisServo)
{
  myServo * currServo = (myServo*)thisServo;
  uint32_t newCol;
  if (currServo->targetPos != currServo->currPos)
    newCol = blinkColor(servoMove);
  else
    if (currServo->currPos == SERVOMIN)
      newCol = servoMinPos;
    else
      newCol = servoMaxPos;
  if (currServo->ledPos[0] > 0)
    setLED(currServo->ledPos[0]-1, newCol);
}

void aspectFct(void * thisServo)
{
  myServo * currServo = (myServo*)thisServo;
  uint32_t newCol;
  int currAspect = round((currServo->currPos - currServo->minPos) / ((currServo->maxPos - currServo->minPos) / 8));
  switch(currAspect)
  {
    case 0: newCol = aspectHalt; break; 
    case 1:;
    case 2: newCol = aspectSlow; break;
    case 3:; 
    case 4:; 
    case 5: newCol = fadeColor(aspectSlow); break; 
    default: newCol = aspectClear; break;
  }
  if (currServo->ledPos[0] > 0)
    setLED(currServo->ledPos[0]-1, newCol);
}

void levelCrossingRelay(void * thisServo)
{
  relayFct(thisServo);
  myServo * currServo = (myServo*)thisServo;
  if ((currServo->currPos == THROWN) || (currServo->currPos != currServo->targetPos))
  {
    if (currServo->ledPos[3] > 0)
      setLED(currServo->ledPos[3]-1, fadeColor(level1Col));
    if (currServo->ledPos[4] > 0)
      setLED(currServo->ledPos[4]-1, fadeColor(level1Col, 0, false));
  }
  else
  {
    if (currServo->ledPos[3] > 0)
      setLED(currServo->ledPos[3]-1, colorDark);
    if (currServo->ledPos[4] > 0)
      setLED(currServo->ledPos[4]-1, colorDark);
  }
}

void levelCrossingServo(void * thisServo)
{
  servoFct(thisServo);
  myServo * currServo = (myServo*)thisServo;
  if ((currServo->currPos == SERVOMAX) || (currServo->currPos != currServo->targetPos))
  {
    if (currServo->ledPos[3] > 0)
      setLED(currServo->ledPos[3]-1, blinkColor(level1Col));
    if (currServo->ledPos[4] > 0)
      setLED(currServo->ledPos[4]-1, blinkColor(level1Col, 0, false));
  }
  else
  {
    if (currServo->ledPos[3] > 0)
      setLED(currServo->ledPos[3]-1, colorDark);
    if (currServo->ledPos[4] > 0)
      setLED(currServo->ledPos[4]-1, colorDark);
  }
}

// This function is called whenever a normal DCC Turnout Packet is received and we're in Output Addressing Mode
void notifyDccAccTurnoutOutput(uint16_t Addr, uint8_t Direction, uint8_t OutputPower)
{
  Serial.print("notifyDccAccTurnoutOutput: ");
  Serial.print(Addr, DEC);
  Serial.print(',');
  Serial.print(Direction, DEC);
  Serial.print(',');
  Serial.println(OutputPower, HEX);
  for (int i = 0; i < numChannels; i++)
  {
    if (Addr == servoArray[i].dccAddr)
    {
      if (Direction == 0)
        servoArray[i].targetPos = (servoArray[i].driveType == SERVO) ? servoArray[i].minPos : THROWN;
      else
        servoArray[i].targetPos = (servoArray[i].driveType == SERVO) ? servoArray[i].maxPos : CLOSED;
      if (servoArray[i].driveType == RELAY)
        servoArray[i].currPos = (servoArray[i].targetPos == CLOSED) ? THROWN : CLOSED;
    }
  }
}

// This function is called whenever a DCC Signal Aspect Packet is received
void notifyDccSigOutputState(uint16_t Addr, uint8_t State)
{
  Addr += 10000;
  Serial.print("notifyDccSigOutputState: ");
  Serial.print(Addr, DEC);
  Serial.print(',');
  Serial.println(State, HEX);
  for (int i = 0; i < numChannels; i++)
    if ((Addr == servoArray[i].dccAddr) && (servoArray[i].driveType == SERVO))
      servoArray[i].targetPos = min(servoArray[i].maxPos, servoArray[i].minPos + (State * ((servoArray[i].maxPos - servoArray[i].minPos) / 8)));
}

void processServo(myServo *thisServo)
{
  if (thisServo->targetPos != thisServo->currPos)
  {
    if ((thisServo->nextMove < millis()))
    {
      if (thisServo->targetPos > thisServo->currPos)
      {
        thisServo->currPos++;
        pwm.setPWM(thisServo->portA, 0, thisServo->currPos);
        thisServo->nextMove = millis() + thisServo->moveDelayCL;
      }
      else
      {
        thisServo->currPos--;
        pwm.setPWM(thisServo->portA, 0, thisServo->currPos);
        thisServo->nextMove = millis() + thisServo->moveDelayTH;
      }
    }
  }
}

void processRelay(myServo *thisServo)
{
  switch (thisServo->currPos)
  {
  case 0:;
  case 1:
  {
    mcp.digitalWrite(thisServo->portA, thisServo->targetPos == CLOSED);
    mcp.digitalWrite(thisServo->portB, thisServo->targetPos == THROWN);
    thisServo->nextMove = millis() + ((thisServo->targetPos == CLOSED) ? thisServo->moveDelayCL : thisServo->moveDelayTH);
    thisServo->currPos = 0xFFFF;
    break;
  }
  case 0xFFFF:
    if ((thisServo->nextMove < millis()) && ((thisServo->moveDelayTH > 0) || (thisServo->moveDelayCL > 0)))
    {
      thisServo->currPos = thisServo->targetPos;
      mcp.digitalWrite(thisServo->portA, 1);
      mcp.digitalWrite(thisServo->portB, 1);
    }
    break;
  }
}

void processLED(myServo *thisServo)
{
  if (thisServo->ledProc != NULL)
    thisServo->ledProc(thisServo);
}

void processLocations()
{
  for (int i = 0; i < numChannels; i++)
  {
    if (servoArray[i].targetPos != servoArray[i].currPos)
    {
      switch (servoArray[i].driveType)
      {
      case SERVO:
        processServo(&servoArray[i]);
        break;
      case RELAY:
        processRelay(&servoArray[i]);
        break;
      }
    }
    processLED(&servoArray[i]);
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Configure the DCC CV Programing ACK pin for an output
  pinMode(DccAckPin, OUTPUT);

  Serial.println("NMRA DCC Example 1");

  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up
  Dcc.pin(DccIntrpt, DccDataPin, 1);

  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init(MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 0);

  Serial.println("Init Done");

  pwm.begin();

  pwm.setPWMFreq(60); // Analog servos run at ~60 Hz updates

  mcp.begin(); // use default address 0
  for (int i = 0; i < 16; i++)
  {
    mcp.pinMode(i, OUTPUT);
    mcp.digitalWrite(i, 1);
  }


//  thisStrip.setCPUmax(10);  // start with 50% CPU usage. up this if the strand flickers or is slow
  
  // Start up the LED counter
  thisStrip.begin();

  // Update the strip, to start they are all 'off'

  for (int i = 0; i < numPixels; i++)
  {
    thisStrip.setPixelColor(i, 0);
    pixCopy[i] = 0;
  }
  ledChg = true;
}

void loop()
{
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();
  processLocations();
  
  timeElapsed = millis() - ledBlinkTimer;
  if (timeElapsed > blinkLEDInterval)
  {
    blinkFlag = !blinkFlag; //flip flop flag
    ledBlinkTimer += blinkLEDInterval; 
  }
  if (blinkFlag)
    faderCtr = (float)timeElapsed/(float)blinkLEDInterval;  //positive slope ramp from 0 to 1
  else
    faderCtr = 1 - ((float)timeElapsed/(float)blinkLEDInterval); //negative slope ramp from 1 to 0

  if (ledChg)
  {
    thisStrip.show();
    ledChg = false;
  }

#ifdef measureMode
  if (loopTimer < millis())
  {
    Serial.println(loopCounter);
    loopCounter = 0;
    loopTimer += 1000;
  }
  loopCounter++;
#endif  
}
