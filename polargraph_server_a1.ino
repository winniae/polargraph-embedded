//#include <AFMotorSPI.h>
#include <AFMotor.h>
#include <AccelStepper.h>
#include <Servo.h>
#include <EEPROM.h>

//  EEPROM addresses
const int EEPROM_MACHINE_WIDTH = 0;
const int EEPROM_MACHINE_HEIGHT = 2;
const int EEPROM_MACHINE_NAME = 4;
const int EEPROM_MACHINE_MM_PER_REV = 14;
const int EEPROM_MACHINE_STEPS_PER_REV = 16;

// Pen raising servo
Servo penHeight;
int const UP_POSITION = 180;
int const DOWN_POSITION = 90;
int const PEN_HEIGHT_SERVO_PIN = 10;
boolean isPenUp = true;

int motorStepsPerRev = 800;
float mmPerRev = 95;

int machineWidth = 650;
int machineHeight = 800;

const int DEFAULT_MACHINE_WIDTH = 650;
const int DEFAULT_MACHINE_HEIGHT = 650;
const int DEFAULT_MM_PER_REV = 95;
const int DEFAULT_STEPS_PER_REV = 800;

String machineName = "";
const String DEFAULT_MACHINE_NAME = "PG01    ";

float currentMaxSpeed = 800.0;
float currentAcceleration = 400.0;

AF_Stepper motora(motorStepsPerRev, 1);
AF_Stepper motorb(motorStepsPerRev, 2);

int startLengthMM = 800;

double mmPerStep = mmPerRev / motorStepsPerRev;
double stepsPerMM = motorStepsPerRev / mmPerRev;

int pageWidth = machineWidth * stepsPerMM;

static String rowAxis = "A";
const int INLENGTH = 50;
const char INTERMINATOR = 10;

const String DIRECTION_STRING_LTR = "LTR";
const int SRAM_SIZE = 2048;
const String FREE_MEMORY_STRING = "MEMORY,";
int availMem = 0;

static float penWidth = 0.8; // line width in mm

const int stepType = INTERLEAVE;

void forwarda() {  
  motora.onestep(FORWARD, stepType);
}
void backwarda() {  
  motora.onestep(BACKWARD, stepType);
}

AccelStepper accelA(forwarda, backwarda);


void forwardb() {  
  motorb.onestep(FORWARD, stepType);
}
void backwardb() {  
  motorb.onestep(BACKWARD, stepType);
}

AccelStepper accelB(forwardb, backwardb);

boolean currentlyRunning = false;

static String inCmd;
static String inParam1;
static String inParam2;
static String inParam3;
static String inParam4;

int inNoOfParams;

static boolean lastWaveWasTop = true;
static boolean lastMotorBiasWasA = true;
//static boolean drawingLeftToRight = true;

//  Drawing direction
const static byte DIR_NE = 1;
const static byte DIR_SE = 2;
const static byte DIR_SW = 3;
const static byte DIR_NW = 4;

const static byte DIR_N = 5;
const static byte DIR_E = 6;
const static byte DIR_S = 7;
const static byte DIR_W = 8;
static int globalDrawDirection = DIR_NW;

const static byte DIR_MODE_AUTO = 1;
const static byte DIR_MODE_PRESET = 2;
const static byte DIR_MODE_RANDOM = 3;
static int globalDrawDirectionMode = DIR_MODE_AUTO;


static int currentRow = 0;

const String READY = "READY";
const String RESEND = "RESEND";
const String DRAWING = "DRAWING";
const static String OUT_CMD_CARTESIAN = "CARTESIAN,";
const static String OUT_CMD_SYNC = "SYNC,";

static String readyString = READY;

String lastCommand = "";
boolean lastCommandConfirmed = false;

int roveMaxA = 0;
int roveMinA = 0;
int roveMaxB = 0;
int roveMinB = 0;

boolean roving = false;


const static String COMMA = ",";
const static String CMD_EXEC = "EXEC";
const static String CMD_ACK = "ACK,";

const static String CMD_CHANGELENGTH = "C01";
const static String CMD_CHANGEPENWIDTH = "C02";
const static String CMD_CHANGEMOTORSPEED = "C03";
const static String CMD_CHANGEMOTORACCEL = "C04";
const static String CMD_DRAWPIXEL = "C05";
const static String CMD_DRAWSCRIBBLEPIXEL = "C06";
const static String CMD_DRAWRECT = "C07";
const static String CMD_CHANGEDRAWINGDIRECTION = "C08";
const static String CMD_SETPOSITION = "C09";
const static String CMD_TESTPATTERN = "C10";
const static String CMD_TESTPENWIDTHSQUARE = "C11";
const static String CMD_TESTPENWIDTHSCRIBBLE = "C12";
const static String CMD_PENDOWN = "C13";
const static String CMD_PENUP = "C14";
const static String CMD_CHANGELENGTHDIRECT = "C17";
const static String CMD_SETMACHINESIZE = "C24";
const static String CMD_SETMACHINENAME = "C25";
const static String CMD_GETMACHINEDETAILS = "C26";
const static String CMD_RESETEEPROM = "C27";
const static String CMD_DRAWDIRECTIONTEST = "C28";
const static String CMD_SETMACHINEMMPERREV = "C29";
const static String CMD_SETMACHINESTEPSPERREV = "C30";
const static String CMD_SETMOTORSPEED = "C31";
const static String CMD_SETMOTORACCEL = "C32";
const static String CMD_END = ",END";

void setup() 
{
  Serial.begin(57600);           // set up Serial library at 9600 bps
  Serial.print(F("POLARGRAPH ON!"));
  Serial.println();

  pinMode(53, OUTPUT); // necessary for SD card reading to work

  loadMachineSpecFromEeprom();

  accelA.setMaxSpeed(currentMaxSpeed);
  accelA.setAcceleration(currentAcceleration);  
  accelB.setMaxSpeed(currentMaxSpeed);
  accelB.setAcceleration(currentAcceleration);

  float startLength = ((float) startLengthMM / (float) mmPerRev) * (float) motorStepsPerRev;
  accelA.setCurrentPosition(startLength);
  accelB.setCurrentPosition(startLength);

  //testServoRange();
  movePenUp();

  readyString = READY;
  establishContact();
  delay(500);
  outputAvailableMemory();
}



void loadMachineSpecFromEeprom()
{
  machineWidth = EEPROMReadInt(EEPROM_MACHINE_WIDTH);
  if (machineWidth < 1)
  {
    machineWidth = DEFAULT_MACHINE_WIDTH;
  }
  Serial.print(F("Loaded machine width:"));
  Serial.println(machineWidth);
  
  machineHeight = EEPROMReadInt(EEPROM_MACHINE_HEIGHT);
  if (machineHeight < 1)
  {
    machineHeight = DEFAULT_MACHINE_HEIGHT;
  }
  Serial.print(F("Loaded machine height:"));
  Serial.println(machineHeight);

  mmPerRev = EEPROMReadInt(EEPROM_MACHINE_MM_PER_REV);
  if (mmPerRev < 1)
  {
    mmPerRev = DEFAULT_MM_PER_REV;
  }
  Serial.print(F("Loaded mm per rev:"));
  Serial.println(mmPerRev);

  motorStepsPerRev = EEPROMReadInt(EEPROM_MACHINE_STEPS_PER_REV);
  if (motorStepsPerRev < 1)
  {
    motorStepsPerRev = DEFAULT_STEPS_PER_REV;
  }
  Serial.print(F("Loaded motor steps per rev:"));
  Serial.println(motorStepsPerRev);

  mmPerStep = mmPerRev / motorStepsPerRev;
  stepsPerMM = motorStepsPerRev / mmPerRev;
  
  Serial.print(F("Recalculated mmPerStep ("));
  Serial.print(mmPerStep);
  Serial.print(F(") and stepsPerMM ("));
  Serial.print(stepsPerMM);
  Serial.print(F(")"));
  Serial.println();
  
  pageWidth = machineWidth * stepsPerMM;
  Serial.print(F("Recalculated pageWidth in steps ("));
  Serial.print(pageWidth);
  Serial.print(F(")"));
  Serial.println();


  String name = "";
  for (int i = 0; i < 8; i++)
  {
    char b = EEPROM.read(EEPROM_MACHINE_NAME+i);
    name = name + b;
  }
  
  if (name[0] == 0)
    name = DEFAULT_MACHINE_NAME;
  
  machineName = name;
  Serial.print(F("Loaded machine name:"));
  Serial.println(machineName);
}

void penUp()
{
  if (isPenUp == false)
  {
    movePenUp();
  }
}

void movePenUp()
{
  penHeight.attach(PEN_HEIGHT_SERVO_PIN);
  for (int i=DOWN_POSITION; i<UP_POSITION; i++) {
//    Serial.println(i);
    penHeight.write(i);
    delay(10);
  }
  penHeight.detach();
  isPenUp = true;
}  


void penDown()
{
  if (isPenUp == true)
  {
    movePenDown();
  }
}
void movePenDown()
{
  penHeight.attach(PEN_HEIGHT_SERVO_PIN);
  for (int i=UP_POSITION; i>DOWN_POSITION; i--) {
//    Serial.println(i);
    penHeight.write(i);
    delay(10);
  }
  penHeight.detach();
  isPenUp = false;
}
void testPenHeight()
{
  delay(3000);
  penUp();
  delay(3000);
  penDown();
  delay(3000);
}
void testServoRange()
{
  penHeight.attach(PEN_HEIGHT_SERVO_PIN);
  for (int i=0; i<200; i++) {
    Serial.println(i);
    penHeight.write(i);
    delay(15);
  }
  penHeight.detach();
  
}
void establishContact() 
{
  ready();
}
void ready()
{
  Serial.println(READY);
}
void drawing()
{
  Serial.println(DRAWING);
}
void acknowledge(String command)
{
  Serial.print(CMD_ACK);
  Serial.println(command);
}

void loop()
{
  // send ready
  // wait for instruction
  int idleTime = millis();
  while (!lastCommandConfirmed)
  {
//    rove();
    // idle
    String inS = "";

    // get incoming command
    while (inS.length() == 0)
    {
//      rove();
      int timeSince = millis() - idleTime;
      if (timeSince > 5000)
      {
        ready();
        idleTime = millis();
      }
      inS = readCommand();
    }

    if (inS.equals(CMD_EXEC)) // this is confirming the previous command
    {
      // this shit is on
      lastCommandConfirmed = true;
      Serial.print(F("Command confirmed: "));
      Serial.println(lastCommand);
      idleTime = millis();
    }
    else if (inS.startsWith("CANCEL")) // this is cancelling the previous command
    {
      lastCommand = "";
      lastCommandConfirmed = false;
      ready();
      idleTime = millis();
    }
    else // new command
    {
      lastCommand = inS;
      lastCommandConfirmed = false;
      acknowledge(lastCommand);
      idleTime = millis();
    }
  }

  boolean commandParsed = parseCommand(lastCommand);
  if (commandParsed)
  {
    Serial.println(F("Executing command."));
    executeCommand(lastCommand);
    lastCommand = "";
    lastCommandConfirmed = false;
    ready();
  }
  else
  {
    Serial.println(F("Command not parsed."));
    lastCommand = "";
    lastCommandConfirmed = false;
    ready();
  }
}


void requestResend()
{
  Serial.println(RESEND);
}
String extractCommandFromExecute(String inS)
{
  String result = inS.substring(8);
  return result;
}


void executeCommand(String inS)
{
  outputAvailableMemory();
  
  if (inS.startsWith(CMD_CHANGELENGTH))
  {
    changeLength();
  }
  else if (inS.startsWith(CMD_CHANGELENGTHDIRECT))
  {
    changeLengthDirect();
  }
  else if (inS.startsWith(CMD_CHANGEPENWIDTH))
  {
    changePenWidth();
  }
  else if (inS.startsWith(CMD_CHANGEMOTORSPEED))
  {
    changeMotorSpeed();
  }
  else if (inS.startsWith(CMD_CHANGEMOTORACCEL))
  {
    changeMotorAcceleration();
  }
  else if (inS.startsWith(CMD_SETMOTORSPEED))
  {
    setMotorSpeed();
  }
  else if (inS.startsWith(CMD_SETMOTORACCEL))
  {
    setMotorAcceleration();
  }
  else if (inS.startsWith(CMD_DRAWPIXEL))
  {
    // go to coordinates.
    drawSquarePixel();
  }  
  else if (inS.startsWith(CMD_DRAWSCRIBBLEPIXEL))
  {
    // go to coordinates.
    drawScribblePixel();
  }  
  else if (inS.startsWith(CMD_DRAWRECT))
  {
    // go to coordinates.
    drawRectangle();
  }
  else if (inS.startsWith(CMD_CHANGEDRAWINGDIRECTION))
  {
    changeDrawingDirection();
  }
  else if (inS.startsWith(CMD_SETPOSITION))
  {
    setPosition();
  }
  else if (inS.startsWith(CMD_TESTPATTERN))
  {
    testPattern();
  }
  else if (inS.startsWith(CMD_TESTPENWIDTHSQUARE))
  {
    testPenWidth();
  }
  else if (inS.startsWith(CMD_TESTPENWIDTHSCRIBBLE))
  {
    testPenWidthScribble();
  }
  else if (inS.startsWith(CMD_PENDOWN))
  {
    penDown();
  }
  else if (inS.startsWith(CMD_PENUP))
  {
    penUp();
  }
  else if (inS.startsWith(CMD_SETMACHINESIZE))
  {
    setMachineSizeFromCommand();
  }
  else if (inS.startsWith(CMD_SETMACHINENAME))
  {
    setMachineNameFromCommand();
  }
  else if (inS.startsWith(CMD_SETMACHINEMMPERREV))
  {
    setMachineMmPerRevFromCommand();
  }
  else if (inS.startsWith(CMD_SETMACHINESTEPSPERREV))
  {
    setMachineStepsPerRevFromCommand();
  }
  else if (inS.startsWith(CMD_GETMACHINEDETAILS))
  {
    reportMachineSpec();
  }
  else if (inS.startsWith(CMD_RESETEEPROM))
  {
    resetEeprom();
  }
  else if (inS.startsWith(CMD_DRAWDIRECTIONTEST))
  {
    drawTestDirectionSquare();
  }
  else
  {
    Serial.print(F("Sorry, "));
    Serial.print(inS);
    Serial.print(F(" isn't a command I recognise."));
    Serial.println();
    ready();
  }

//  Serial.println("After execute:");
//  outputAvailableMemory();
  
}

boolean parseCommand(String inS)
{
  String endChars = inS.substring(inS.length() - 4);
  if (endChars.equals(CMD_END))
  {
    extractParams(inS);
    return true;
  }
  else
    return false;
}  

String readCommand()
{
  // check if data has been sent from the computer:
  char inString[INLENGTH+1];
  int inCount = 0;
  while (Serial.available() > 0)
  {
    char ch = Serial.read();       // get it
    delay(15);
    inString[inCount] = ch;
    if (ch == INTERMINATOR)
    {
      Serial.flush();
      break;
    }
    inCount++;
  }
  inString[inCount] = 0;                     // null terminate the string
  String inS = inString;
  return inS;
}

long asLong(String inParam)
{
  char paramChar[inParam.length() + 1];
  inParam.toCharArray(paramChar, inParam.length() + 1);
  return atol(paramChar);
}
int asInt(String inParam)
{
  char paramChar[inParam.length() + 1];
  inParam.toCharArray(paramChar, inParam.length() + 1);
  return atoi(paramChar);
}
byte asByte(String inParam)
{
  int i = asInt(inParam);
  return (byte) i;
}
float asFloat(String inParam)
{
  char paramChar[inParam.length() + 1];
  inParam.toCharArray(paramChar, inParam.length() + 1);
  return atof(paramChar);
}

/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************            BELOW IS THE CODE THAT DOES THE WORK      ******************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/

void resetEeprom()
{
  for (int i = 0; i <20; i++)
  {
    EEPROM.write(i, 0);
  }
  loadMachineSpecFromEeprom();
}
void dumpEeprom()
{
  for (int i = 0; i <20; i++)
  {
    Serial.print(i);
    Serial.print(". ");
    Serial.println(EEPROM.read(i));
  }
}  

void reportMachineSpec()
{
  dumpEeprom();
  Serial.print(F("PGNAME,"));
  Serial.print(machineName);
  Serial.println(CMD_END);
  
  Serial.print(F("PGSIZE,"));
  Serial.print(machineWidth);
  Serial.print(COMMA);
  Serial.print(machineHeight);
  Serial.println(CMD_END);

  Serial.print(F("PGMMPERREV,"));
  Serial.print(mmPerRev);
  Serial.println(CMD_END);

  Serial.print(F("PGSTEPSPERREV,"));
  Serial.print(motorStepsPerRev);
  Serial.println(CMD_END);
}

void setMachineSizeFromCommand()
{
  int width = asInt(inParam1);
  int height = asInt(inParam2);
  
  if (width > 10)
  {
    EEPROMWriteInt(EEPROM_MACHINE_WIDTH, width);
  }
  if (height > 10)
  {
    EEPROMWriteInt(EEPROM_MACHINE_HEIGHT, height);
  }

  loadMachineSpecFromEeprom();
}
void setMachineNameFromCommand()
{
  String name = inParam1;
  if (name != DEFAULT_MACHINE_NAME)
  {
    for (int i = 0; i < 8; i++)
    {
      EEPROM.write(EEPROM_MACHINE_NAME+i, name[i]);
    }
  }
  loadMachineSpecFromEeprom();
}

void setMachineMmPerRevFromCommand()
{
  int mmPerRev = asInt(inParam1);
  EEPROMWriteInt(EEPROM_MACHINE_MM_PER_REV, mmPerRev);
  loadMachineSpecFromEeprom();
}
void setMachineStepsPerRevFromCommand()
{
  int stepsPerRev = asInt(inParam1);
  EEPROMWriteInt(EEPROM_MACHINE_STEPS_PER_REV, stepsPerRev);
  loadMachineSpecFromEeprom();
}

void setMotorSpeed()
{
  setMotorSpeed(asFloat(inParam1));
}

void setMotorSpeed(float speed)
{
  currentMaxSpeed = speed;
  accelA.setMaxSpeed(currentMaxSpeed);
  accelB.setMaxSpeed(currentMaxSpeed);
  Serial.print(F("New max speed: "));
  Serial.println(currentMaxSpeed);
}

void changeMotorSpeed()
{
  float speedChange = asFloat(inParam1);
  float newSpeed = currentMaxSpeed + speedChange;
  setMotorSpeed(newSpeed);
 }
 
void setMotorAcceleration()
{
  setMotorAcceleration(asFloat(inParam1));
}
void setMotorAcceleration(float accel)
{
  currentAcceleration = accel;
  accelA.setAcceleration(currentAcceleration);
  accelB.setAcceleration(currentAcceleration);
  Serial.print(F("New acceleration: "));
  Serial.println(currentAcceleration);
}
void changeMotorAcceleration()
{
  float speedChange = asFloat(inParam1);
  float newAccel = currentAcceleration + speedChange;
  setMotorAcceleration(newAccel);
}

void changePenWidth()
{
  penWidth = asFloat(inParam1);
  Serial.print(F("Changed Pen width to "));
  Serial.print(penWidth);
  Serial.print(F("mm"));
  Serial.println();
 }   

void changeDrawingDirection() 
{
  globalDrawDirectionMode = asInt(inParam1);
  globalDrawDirection = asInt(inParam2);
  Serial.print(F("Changed draw direction mode to be "));
  Serial.print(globalDrawDirectionMode);
  Serial.print(F(" and direction is "));
  Serial.println(globalDrawDirection);
}

  void extractParams(String inS) {
    
    // get number of parameters
    // by counting commas
    int length = inS.length();
    
    int startPos = 0;
    int paramNumber = 0;
    for (int i = 0; i < length; i++) {
      if (inS.charAt(i) == ',') {
        String param = inS.substring(startPos, i);
        startPos = i+1;
        
        switch(paramNumber) {
          case 0:
            inCmd = param;
            break;
          case 1:
            inParam1 = param;
            break;
          case 2:
            inParam2 = param;
            break;
          case 3:
            inParam3 = param;
            break;
          case 4:
            inParam4 = param;
            break;
          default:
            break;
        }
        paramNumber++;
      }
    }
    inNoOfParams = paramNumber;
    
//    Serial.print(F("Command:"));
//    Serial.print(inCmd);
//    Serial.print(F(", p1:"));
//    Serial.print(inParam1);
//    Serial.print(F(", p2:"));
//    Serial.print(inParam2);
//    Serial.print(F(", p3:"));
//    Serial.print(inParam3);
//    Serial.print(F(", p4:"));
//    Serial.println(inParam4);
  }
  
  void testPattern()
  {
    int rowWidth = asInt(inParam1);
    int noOfIncrements = asInt(inParam2);

    boolean ltr = true;
    
    for (int w = rowWidth; w < (w+(noOfIncrements*5)); w+=5)
    {
      for (int i = 0;  i <= maxDensity(penWidth, w); i++)
      {
        drawSquarePixel(w, w, i, ltr);
      }
      if (ltr)
        ltr = false;
      else
        ltr = true;
        
      moveB(w);
    }
  }
  
  void testPenWidth()
  {
    int rowWidth = asInt(inParam1);
    float startWidth = asFloat(inParam2);
    float endWidth = asFloat(inParam3); 
    float incSize = asFloat(inParam4);

    int tempDirectionMode = globalDrawDirectionMode;
    globalDrawDirectionMode = DIR_MODE_PRESET;
    
    float oldPenWidth = penWidth;
    int iterations = 0;
    
    for (float pw = startWidth; pw <= endWidth; pw+=incSize)
    {
      iterations++;
      penWidth = pw;
      int maxDens = maxDensity(penWidth, rowWidth);
      Serial.print(F("Penwidth test "));
      Serial.print(iterations);
      Serial.print(F(", pen width: "));
      Serial.print(penWidth);
      Serial.print(F(", max density: "));
      Serial.println(maxDens);
      drawSquarePixel(rowWidth, rowWidth, maxDens, DIR_SE);
    }

    penWidth = oldPenWidth;
    
    moveB(0-rowWidth);
    for (int i = 1; i <= iterations; i++)
    {
      moveB(0-(rowWidth/2));
      moveA(0-rowWidth);
      moveB(rowWidth/2);
    }
    
    penWidth = oldPenWidth;
    globalDrawDirectionMode = tempDirectionMode;
  }    

  void testPenWidthScribble()
  {
    int rowWidth = asInt(inParam1);
    float startWidth = asFloat(inParam2);
    float endWidth = asFloat(inParam3); 
    float incSize = asFloat(inParam4);
    
    boolean ltr = true;
    
    float oldPenWidth = penWidth;
    int iterations = 0;
    
    int posA = accelA.currentPosition();
    int posB = accelB.currentPosition();

    int startColumn = posA;
    int startRow = posB;
    
    for (float pw = startWidth; pw <= endWidth; pw+=incSize)
    {
      iterations++;
      int column = posA;
      
      penWidth = pw;
      int maxDens = maxDensity(penWidth, rowWidth);
      Serial.print(F("Penwidth test "));
      Serial.print(iterations);
      Serial.print(F(", pen width: "));
      Serial.print(penWidth);
      Serial.print(F(", max density: "));
      Serial.println(maxDens);
      
      for (int density = maxDens; density >= 0; density--)
      {
        drawScribblePixel(posA, posB, rowWidth, density);
        posB+=rowWidth;
      }
      
      posA+=rowWidth;
      posB = startRow;
    }
    
    changeLength(posA-(rowWidth/2), startRow-(rowWidth/2));

    penWidth = oldPenWidth;
    
    moveB(0-rowWidth);
    for (int i = 1; i <= iterations; i++)
    {
      moveB(0-(rowWidth/2));
      moveA(0-rowWidth);
      moveB(rowWidth/2);
    }
    
    penWidth = oldPenWidth;
  }    

  void drawRectangle()
  {
    int v1A = asInt(inParam1);
    int v1B = asInt(inParam2);
    int v2A = asInt(inParam3);
    int v2B = asInt(inParam4);
    
    changeLength(v1A, v1B);
    accelA.moveTo(v2A);
    accelA.runToPosition();
    
    accelB.moveTo(v2B);
    accelB.runToPosition();
    
    accelA.moveTo(v1A);
    accelA.runToPosition();
    
    accelB.moveTo(v1B);
    accelB.runToPosition();
    
  }




void changeLength()
{
  int lenA = asInt(inParam1);
  int lenB = asInt(inParam2);
  
  changeLength(lenA, lenB);
}  

void changeLength(int tA, int tB)
{
  accelA.moveTo(tA);
  accelB.moveTo(tB);
  
  while (accelA.distanceToGo() != 0 || accelB.distanceToGo() != 0)
  {
    accelA.run();
    accelB.run();
  }
  
  reportPosition();
}

void changeLength(double tA, double tB)
{
//  int intPos = (int)(tA+0.5);
//  accelA.moveTo(intPos);
//  intPos = (int)(tB+0.5);
//  accelB.moveTo(intPos);

  accelA.moveTo(tA);
  accelB.moveTo(tB);
  
  while (accelA.distanceToGo() != 0 || accelB.distanceToGo() != 0)
  {
    accelA.run();
    accelB.run();
  }
  
  reportPosition();
}

void changeLengthRelative(int tA, int tB)
{
  accelA.move(tA);
  accelB.move(tB);
  
  while (accelA.distanceToGo() != 0 || accelB.distanceToGo() != 0)
  {
    accelA.run();
    accelB.run();
  }
  
  reportPosition();
}

void changeLengthDirect()
{
  float endA = asFloat(inParam1);
  float endB = asFloat(inParam2);
  int maxLength = asInt(inParam3);

  float startA = accelA.currentPosition();
  float startB = accelB.currentPosition();
//  Serial.println(F("Drawing direct line"));
  
  drawBetweenPoints(startA, startB, endA, endB, maxLength);
}  

void drawBetweenPoints(float p1a, float p1b, float p2a, float p2b, int maxLength)
{
  // ok, we're going to plot some dots between p1 and p2.  Using maths. I know! Brave new world etc.
  
//  Serial.print(F("Current native pos: "));
//  Serial.print(p1a);
//  Serial.print(COMMA);
//  Serial.println(p1b);
//  
//  Serial.print(F("Target native pos: "));
//  Serial.print(p2a);
//  Serial.print(COMMA);
//  Serial.println(p2b);

  // First, convert these values to cartesian coordinates
  // We're going to figure out how many segments the line
  // needs chopping into.
  double c1x = getCartesianXFP(p1a, p1b);
  double c1y = getCartesianYFP(c1x, p1a);
  
  double c2x = getCartesianXFP(p2a, p2b);
  double c2y = getCartesianYFP(c2x, p2a);

//  Serial.print(F("Origin cartesian coords: "));
//  Serial.print(c1x*mmPerStep);
//  Serial.print(COMMA);
//  Serial.println(c1y*mmPerStep);
//
//  Serial.print(F("Target cartesian coords: "));
//  Serial.print(c2x*mmPerStep);
//  Serial.print(COMMA);
//  Serial.println(c2y*mmPerStep);
  
  double deltaX = c2x-c1x;    // distance each must move (signed)
  double deltaY = c2y-c1y;

//  Serial.print(F("DeltaX and Y: "));
//  Serial.print(deltaX);
//  Serial.print(COMMA);
//  Serial.print(deltaY);
//  Serial.println();
//
  int xy_incmax = maxLength;  // smallest chop in step counts? configurable?
//   
  int linesegs = 1;            // assume at least 1 line segment will get us there.
  if (abs(deltaX) > abs(deltaY))
  {
    // slope <=1 case    
    while ((abs(deltaX)/linesegs) > xy_incmax)
    {
      linesegs++;
    }
  }
  else
  {
    // slope >1 case
    while ((abs(deltaY)/linesegs) > xy_incmax)
    {
      linesegs++;
    }
  }
  
//  linesegs -= 1;

//  Serial.print(F("Line segments "));
//  Serial.println(linesegs);
   
  double deltaXIncrement = deltaX/linesegs;
  double deltaYIncrement = deltaY/linesegs;
   
//  Serial.print(F("DeltaX and Y: "));
//  Serial.print(deltaX);
//  Serial.print(COMMA);
//  Serial.print(deltaY);
//  Serial.println();
  
  // OK, move to the first point.
//  Serial.println("moving to the first point.");
  changeLength(p1a, p1b);
  
  
  // render the line in N shorter segments
  while (linesegs > 0)
  {
//    Serial.print(F("Line segment "));
//    Serial.println(linesegs);

    // get current cartesian position again
//    c1x = getCartesianXFP(accelA.currentPosition(), accelB.currentPosition());
//    c1y = getCartesianYFP(c1x, accelA.currentPosition());
    
    // recalculate remaining delta
//    deltaX = c2x-c1x;    // distance each must move (signed)
//    deltaY = c2y-c1y;
//
////    Serial.print(F("DeltaX:"));
////    Serial.println(deltaX);
////    Serial.print(F("DeltaY:"));
////    Serial.println(deltaY);
//    
//    deltaXIncrement = deltaX/linesegs;
//    deltaYIncrement = deltaY/linesegs;
    

//    Serial.print(F("DeltaXInc:"));
//    Serial.println(deltaXIncrement);
//    Serial.print(F("DeltaYInc:"));
//    Serial.println(deltaYIncrement);

    c1x = c1x + deltaXIncrement;        // compute next new location
    c1y = c1y + deltaYIncrement;
    
//    Serial.print(F("Next cartesian coords: "));
//    Serial.print(c1x*mmPerStep);
//    Serial.print(COMMA);
//    Serial.println(c1y*mmPerStep);
    
    // convert back to machine space
    double pA = getMachineA(c1x, c1y);
    double pB = getMachineB(c1x, c1y);

//    Serial.print(F("Next native pos: "));
//    Serial.print(pA);
//    Serial.print(COMMA);
//    Serial.println(pB);
  
    changeLength(pA, pB);

    linesegs--;
  }
  
//  delay(1000);
  // do the end point in case theres been some rounding errors etc
  changeLength(p2a, p2b);
}

double getMachineA(double cX, double cY)
{
  double a = sqrt(sq(cX)+sq(cY));
  return a;
}
double getMachineB(double cX, double cY)
{
  double b = sqrt(sq((pageWidth)-cX)+sq(cY));
  return b;
}



void drawTestDirectionSquare()
{
  int rowWidth = asInt(inParam1);
  int segments = asInt(inParam2);
  drawSquarePixel(rowWidth, rowWidth, segments, DIR_SE);
  moveA(rowWidth*2);
  
  drawSquarePixel(rowWidth, rowWidth, segments, DIR_SW);
  moveB(rowWidth*2);
  
  drawSquarePixel(rowWidth, rowWidth, segments, DIR_NW);
  moveA(0-(rowWidth*2));
  
  drawSquarePixel(rowWidth, rowWidth, segments, DIR_NE);
  moveB(0-(rowWidth*2));
  
}

void drawSquarePixel() 
{
    int originA = asInt(inParam1);
    int originB = asInt(inParam2);
    int size = asInt(inParam3);
    int density = asInt(inParam4);

    int halfSize = size / 2;
    
    int startPointA;
    int startPointB;
    int endPointA;
    int endPointB;

    int calcFullSize = halfSize * 2; // see if there's any rounding errors
    int offsetStart = size - calcFullSize;
    
    if (globalDrawDirectionMode == DIR_MODE_AUTO)
      globalDrawDirection = getAutoDrawDirection(originA, originB, accelA.currentPosition(), accelB.currentPosition());
      

    if (globalDrawDirection == DIR_SE) 
    {
//      Serial.println(F("d: SE"));
      startPointA = originA - halfSize;
      startPointA += offsetStart;
      startPointB = originB;
      endPointA = originA + halfSize;
      endPointB = originB;
    }
    else if (globalDrawDirection == DIR_SW)
    {
//      Serial.println(F("d: SW"));
      startPointA = originA;
      startPointB = originB - halfSize;
      startPointB += offsetStart;
      endPointA = originA;
      endPointB = originB + halfSize;
    }
    else if (globalDrawDirection == DIR_NW)
    {
//      Serial.println(F("d: NW"));
      startPointA = originA + halfSize;
      startPointA -= offsetStart;
      startPointB = originB;
      endPointA = originA - halfSize;
      endPointB = originB;
    }
    else //(drawDirection == DIR_NE)
    {
//      Serial.println(F("d: NE"));
      startPointA = originA;
      startPointB = originB + halfSize;
      startPointB -= offsetStart;
      endPointA = originA;
      endPointB = originB - halfSize;
    }

    density = scaleDensity(density, 255, maxDensity(penWidth, size));
//    Serial.print(F("Start point: "));
//    Serial.print(startPointA);
//    Serial.print(COMMA);
//    Serial.print(startPointB);
//    Serial.print(F(". end point: "));
//    Serial.print(endPointA);
//    Serial.print(COMMA);
//    Serial.print(endPointB);
//    Serial.println(F("."));
    
    changeLength(startPointA, startPointB);
    if (density > 1)
    {
      drawSquarePixel(size, size, density, globalDrawDirection);
    }
    changeLength(endPointA, endPointB);
    
    outputAvailableMemory(); 
}

byte getRandomDrawDirection()
{
  return random(1, 5);
}

byte getAutoDrawDirection(long targetA, long targetB, long sourceA, long sourceB)
{
  byte dir = DIR_SE;
  
  // some bitchin triangles, I goshed-well love triangles.
  long diffA = sourceA - targetA;
  long diffB = sourceB - targetB;
  long hyp = sqrt(sq(diffA)+sq(diffB));
  
  float bearing = atan(hyp/diffA);
  
//  Serial.print("bearing:");
//  Serial.println(bearing);
//  
//  Serial.print(F("TargetA: "));
//  Serial.print(targetA);
//  Serial.print(F(", targetB: "));
//  Serial.print(targetB);
//  Serial.print(F(". SourceA: "));
//  Serial.print(sourceA);
//  Serial.print(F(", sourceB: "));
//  Serial.print(sourceB);
//  Serial.println(F("."));
  
  
  if (targetA<sourceA && targetB<sourceA)
  {
//    Serial.println(F("calculated NW"));
    dir = DIR_NW;
  }
  else if (targetA>sourceA && targetB>sourceB)
  {
//    Serial.println(F("calculated SE"));
    dir = DIR_SE;
  }
  else if (targetA<sourceA && targetB>sourceB)
  {
//    Serial.println(F("calculated SW"));
    dir = DIR_SW;
  }
  else if (targetA>sourceA && targetB<sourceB)
  {
//    Serial.println(F("calculated NE"));
    dir = DIR_NE;
  }
  else if (targetA==sourceA && targetB<sourceB)
  {
//    Serial.println(F("calc NE"));
    dir = DIR_NE;
  }
  else if (targetA==sourceA && targetB>sourceB)
  {
//    Serial.println(F("calc SW"));
    dir = DIR_SW;
  }
  else if (targetA<sourceA && targetB==sourceB)
  {
//    Serial.println(F("calc NW"));
    dir = DIR_NW;
  }
  else if (targetA>sourceA && targetB==sourceB)
  {
//    Serial.println(F("calc SE"));
    dir = DIR_SE;
  }
  else
  {
//    Serial.println("Not calculated - default SE");
  }

  return dir;
}

void drawScribblePixel() {
    int originA = asInt(inParam1);
    int originB = asInt(inParam2);
    int size = asInt(inParam3);
    int density = asInt(inParam4);
    
    int maxDens = maxDensity(penWidth, size);

    density = scaleDensity(density, 255, maxDens);
    drawScribblePixel(originA, originB, size*1.1, density);
    
    outputAvailableMemory(); 
}

void drawScribblePixel(int originA, int originB, int size, int density) {

//  int originA = accelA.currentPosition();
//  int originB = accelB.currentPosition();
  
  int lowLimitA = originA-(size/2);
  int highLimitA = lowLimitA+size;
  int lowLimitB = originB-(size/2);
  int highLimitB = lowLimitB+size;
  int randA;
  int randB;
  
  int inc = 0;
  int currSize = size;
  
  for (int i = 0; i <= density; i++)
  {
    randA = random(0, currSize);
    randB = random(0, currSize);
    changeLength(lowLimitA+randA, lowLimitB+randB);
    
    lowLimitA-=inc;
    highLimitA+=inc;
    currSize+=inc*2;
  }
}

int minSegmentSizeForPen(float penSize)
{
  float penSizeInSteps = penSize * stepsPerMM;

  int minSegSize = 1;
  if (penSizeInSteps >= 2.0)
    minSegSize = int(penSizeInSteps);
    
//  Serial.print(F("Min segment size for penSize "));
//  Serial.print(penSize);
//  Serial.print(F(": "));
//  Serial.print(minSegSize);
//  Serial.print(F(" steps."));
//  Serial.println();
  
  return minSegSize;
}

int maxDensity(float penSize, int rowSize)
{
  float rowSizeInMM = mmPerStep * rowSize;
//  Serial.print(F("rowsize in mm: "));
//  Serial.print(rowSizeInMM);
//  Serial.print(F(", mmPerStep: "));
//  Serial.print(mmPerStep);
//  Serial.print(F(", rowsize: "));
//  Serial.println(rowSize);
  
  float numberOfSegments = rowSizeInMM / penSize;
  int maxDens = 1;
  if (numberOfSegments >= 2.0)
    maxDens = int(numberOfSegments);
    
//  Serial.print("num of segments float:");
//  Serial.println(numberOfSegments);
//
//    
//  Serial.print(F("Max density: penSize: "));
//  Serial.print(penSize);
//  Serial.print(F(", rowSize: "));
//  Serial.print(rowSize);
//  Serial.println(maxDens);
  
  return maxDens;
}

int scaleDensity(int inDens, int inMax, int outMax)
{
  float reducedDens = (float(inDens) / float(inMax)) * float(outMax);
  reducedDens = outMax-reducedDens;
//  Serial.print(F("inDens:"));
//  Serial.print(inDens);
//  Serial.print(F(", inMax:"));
//  Serial.print(inMax);
//  Serial.print(F(", outMax:"));
//  Serial.print(outMax);
//  Serial.print(F(", reduced:"));
//  Serial.println(reducedDens);
  
  // round up if bigger than .5
  int result = int(reducedDens);
  if (reducedDens - (result) > 0.5)
    result ++;
  
  return result;
}

void drawSquarePixel(int length, int width, int density, byte drawDirection) 
{
  // work out how wide each segment should be
  int segmentLength = 0;

  if (density > 0)
  {
    // work out some segment widths
    int basicSegLength = length / density;
    int basicSegRemainder = length % density;
    float remainderPerSegment = float(basicSegRemainder) / float(density);
    float totalRemainder = 0.0;
    int lengthSoFar = 0;
    
//    Serial.print("Basic seg length:");
//    Serial.print(basicSegLength);
//    Serial.print(", basic seg remainder:");
//    Serial.print(basicSegRemainder);
//    Serial.print(", remainder per seg");
//    Serial.println(remainderPerSegment);
    
    for (int i = 0; i <= density; i++) 
    {
      totalRemainder += remainderPerSegment;

      if (totalRemainder >= 1.0)
      {
        totalRemainder -= 1.0;
        segmentLength = basicSegLength+1;
      }
      else
      {
        segmentLength = basicSegLength;
      }

      if (drawDirection == DIR_SE) {
        drawSquareWaveAlongA(width, segmentLength, density, i);
      }
      if (drawDirection == DIR_SW) {
        drawSquareWaveAlongB(width, segmentLength, density, i);
      }
      if (drawDirection == DIR_NW) {
        segmentLength = 0 - segmentLength; // reverse
        drawSquareWaveAlongA(width, segmentLength, density, i);
      }
      if (drawDirection == DIR_NE) {
        segmentLength = 0 - segmentLength; // reverse
        drawSquareWaveAlongB(width, segmentLength, density, i);
      }
      lengthSoFar += segmentLength;
    //      Serial.print("distance so far:");
    //      Serial.print(distanceSoFar);
      
      
      reportPosition();
    } // end of loop
  }
}


void drawSquareWaveAlongA(int waveAmplitude, int waveLength, int totalWaves, int waveNo)
{
  if (waveNo == 0) 
  { 
    // first one, half a line and an along
    Serial.println("First wave half");
    if (lastWaveWasTop) {
      moveB(waveAmplitude/2);
      moveA(waveLength);
    }
    else {
      moveB(0-(waveAmplitude/2));
      moveA(waveLength);
    }
    flipWaveDirection();
  }
  else if (waveNo == totalWaves) 
  { 
    // last one, half a line with no along
    if (lastWaveWasTop) {
      moveB(waveAmplitude/2);
    }
    else {
      moveB(0-(waveAmplitude/2));
    }
  }
  else 
  { 
    // intervening lines - full lines, and an along
    if (lastWaveWasTop) {
      moveB(waveAmplitude);
      moveA(waveLength);
    }
    else {
      moveB(0-waveAmplitude);
      moveA(waveLength);
    }
    flipWaveDirection();
  }
}

void drawSquareWaveAlongB(int waveAmplitude, int waveLength, int totalWaves, int waveNo)
{
  if (waveNo == 0) 
  { 
    // first one, half a line and an along
    if (lastWaveWasTop) {
      moveA(waveAmplitude/2);
      moveB(waveLength);
    }
    else {
      moveA(0-(waveAmplitude/2));
      moveB(waveLength);
    }
    flipWaveDirection();
  }
  else if (waveNo == totalWaves) 
  { 
    // last one, half a line with no along
    if (lastWaveWasTop) {
      moveA(waveAmplitude/2);
    }
    else {
      moveA(0-(waveAmplitude/2));
    }
  }
  else 
  { 
    // intervening lines - full lines, and an along
    if (lastWaveWasTop) {
      moveA(waveAmplitude);
      moveB(waveLength);
    }
    else {
      moveA(0-waveAmplitude);
      moveB(waveLength);
    }
    flipWaveDirection();
  }
}


void flipWaveDirection()
{
  if (lastWaveWasTop)
    lastWaveWasTop = false;
  else
    lastWaveWasTop = true;
}
void moveA(int dist)
{
  accelA.move(dist);
  while (accelA.distanceToGo() != 0)
    accelA.run();
}

void moveB(int dist)
{
  accelB.move(dist);
  while (accelB.distanceToGo() != 0)
    accelB.run();
}

void reportPosition()
{
  Serial.print(OUT_CMD_SYNC);
  Serial.print(accelA.currentPosition());
  Serial.print(COMMA);
  Serial.print(accelB.currentPosition());
  Serial.println(CMD_END);
  
//  int cX = getCartesianX();
//  int cY = getCartesianY(cX, accelA.currentPosition());
//  Serial.print(OUT_CMD_CARTESIAN);
//  Serial.print(cX*mmPerStep);
//  Serial.print(COMMA);
//  Serial.print(cY*mmPerStep);
//  Serial.println(CMD_END);
//
  outputAvailableMemory();
}


void setPosition()
{
  int targetA = asInt(inParam1);
  int targetB = asInt(inParam2);

  accelA.setCurrentPosition(targetA);
  accelB.setCurrentPosition(targetB);
  
  engageMotors();
  
  reportPosition();
}

void engageMotors()
{
  accelA.runToNewPosition(accelA.currentPosition()+4);
  accelB.runToNewPosition(accelB.currentPosition()+4);
  accelA.runToNewPosition(accelA.currentPosition()-4);
  accelB.runToNewPosition(accelB.currentPosition()-4);
}

void releaseMotors()
{
  penUp();
  motora.release();
  motorb.release();
}

double getCartesianXFP(double aPos, double bPos)
{
  double calcX = (pow(pageWidth, 2) - pow(bPos, 2) + pow(aPos, 2)) / (pageWidth*2);
  return calcX;  
}
double getCartesianYFP(double cX, double aPos) 
{
  double calcY = sqrt(pow(aPos,2)-pow(cX,2));
  return calcY;
}


int getCartesianX(float aPos, float bPos)
{
  int calcX = int((pow(pageWidth, 2) - pow(bPos, 2) + pow(aPos, 2)) / (pageWidth*2));
  return calcX;  
}

int getCartesianX() {
  int calcX = getCartesianX(accelA.currentPosition(), accelB.currentPosition());
  return calcX;  
}

int getCartesianY() {
  return getCartesianY(getCartesianX(), accelA.currentPosition());
}
int getCartesianY(int cX, float aPos) {
  int calcX = cX;
  int calcY = int(sqrt(pow(aPos,2)-pow(calcX,2)));
  return calcY;
}


void outputAvailableMemory()
{
  int avMem = availableMemory();
  if (avMem != availMem)
  {
    availMem = avMem;
    Serial.print(FREE_MEMORY_STRING);
    Serial.print(availMem);
    Serial.println(CMD_END);
  }
}

//from http://www.arduino.cc/playground/Code/AvailableMemory
int availableMemory() {
  uint8_t * heapptr, * stackptr;
  stackptr = (uint8_t *)malloc(4);
  heapptr = stackptr;
  free(stackptr);               
  stackptr = (uint8_t *)(SP);
  return stackptr - heapptr;
} 

//This function will write a 2 byte integer to the eeprom at the specified address and address + 1
void EEPROMWriteInt(int p_address, int p_value)
{
  Serial.print(F("Writing Int "));
  Serial.print(p_value);
  Serial.print(F(" to address "));
  Serial.println(p_address);

  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);
  EEPROM.write(p_address, lowByte);
  EEPROM.write(p_address + 1, highByte);
}

//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
unsigned int EEPROMReadInt(int p_address)
{
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);
  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}


