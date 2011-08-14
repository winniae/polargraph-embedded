#include <AFMotor.h>
#include <AccelStepper.h>
#include <Servo.h>

// Pen raising servo
Servo penHeight;
int const UP_POSITION = 90;
int const DOWN_POSITION = 180;
int const PEN_HEIGHT_SERVO_PIN = 9;
boolean isPenUp = true;

int motorStepsPerRev = 800;
float mmPerRev = 95;

float currentMaxSpeed = 600.0;
float currentAcceleration = 200.0;

AF_Stepper motora(motorStepsPerRev, 2);
AF_Stepper motorb(motorStepsPerRev, 1);

int startLengthMM = 800;

float mmPerStep = mmPerRev / motorStepsPerRev;
float stepsPerMM = motorStepsPerRev / mmPerRev;

int pageWidth = 712 * stepsPerMM;

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
static boolean drawingLeftToRight = true;

static int currentRow = 0;

const String READY = "READY";
const String RESEND = "RESEND";
const String ACK = "ACK";
const String DRAWING = "DRAWING";
static String readyString = READY;

String lastCommand = "";
boolean lastCommandConfirmed = false;

int totalDistanceMoved = 0;

void setup() 
{
  Serial.begin(57600);           // set up Serial library at 9600 bps
  Serial.println("DRAWBOT TEST!!");

  accelA.setMaxSpeed(currentMaxSpeed);
  accelA.setAcceleration(currentAcceleration);  
  accelB.setMaxSpeed(currentMaxSpeed);
  accelB.setAcceleration(currentAcceleration);
  
//  accelA.setMinPulseWidth(1);
//  accelB.setMinPulseWidth(1);

  float startLength = ((float) startLengthMM / (float) mmPerRev) * (float) motorStepsPerRev;
  accelA.setCurrentPosition(startLength);
  accelB.setCurrentPosition(startLength);

  // turn on servo
  penDown();
  
  drawingLeftToRight = true;

  readyString = READY;
  establishContact();
  delay(500);
  outputAvailableMemory();
}

void penUp()
{
  if (isPenUp == false)
  {
    penHeight.attach(PEN_HEIGHT_SERVO_PIN);
    for (int i=DOWN_POSITION; i>UP_POSITION; i--) {
      Serial.println(i);
      penHeight.write(i);
      delay(10);
    }
    penHeight.detach();
    isPenUp = true;
  }
}

void penDown()
{
  if (isPenUp == true)
  {
    penHeight.attach(PEN_HEIGHT_SERVO_PIN);
    for (int i=UP_POSITION; i<DOWN_POSITION; i++) {
      Serial.println(i);
      penHeight.write(i);
      delay(10);
    }
    penHeight.detach();
    isPenUp = false;
  }
}
void testPenHeight()
{
  delay(3000);
  penUp();
  delay(3000);
  penDown();
  delay(3000);
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
  String ack = "ACK," + command;
  Serial.println(ack);
}

void loop()
{
  // send ready
  // wait for instruction
  int idleTime = millis();
  while (!lastCommandConfirmed)
  {
    // idle
    String inS = "";

    // get incoming command
    while (inS.length() == 0)
    {
      int timeSince = millis() - idleTime;
      if (timeSince > 5000)
      {
        ready();
        idleTime = millis();
      }
      inS = readCommand();
    }

    if (inS.equals("EXECUTE")) // this is confirming the previous command
    {
      // this shit is on
      lastCommandConfirmed = true;
      Serial.print("Command confirmed:");
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
    Serial.println("Executing command.");
    executeCommand(lastCommand);
    lastCommand = "";
    lastCommandConfirmed = false;
    ready();
  }
  else
  {
    Serial.println("Command not parsed.");
    lastCommand = "";
    lastCommandConfirmed = false;
    ready();
  }
//  ready();
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
  if (inS.startsWith("CHANGELENGTH"))
  {
    changeLength();
  }
  else if (inS.startsWith("CHANGEPENWIDTH"))
  {
    changePenWidth();
  }
  else if (inS.startsWith("CHANGEMOTORSPEED"))
  {
    changeMotorSpeed();
  }
  else if (inS.startsWith("CHANGEMOTORACCEL"))
  {
    changeMotorAcceleration();
  }
  else if (inS.startsWith("DRAWPIXEL"))
  {
    // go to coordinates.
    drawSquarePixel();
  }  
  else if (inS.startsWith("DRAWSCRIBBLEPIXEL"))
  {
//    Serial.println(inS);
    // go to coordinates.
    drawScribblePixel();
  }  
  else if (inS.startsWith("DRAWRECT"))
  {
//    Serial.println(inS);
    // go to coordinates.
    drawRectangle();
  }
  else if (inS.startsWith("CHANGEDRAWINGDIRECTION"))
  {
//    Serial.println(inS);
    changeDrawingDirection();
  }
  else if (inS.startsWith("SETPOSITION"))
  {
//    Serial.println(inS);
    setPosition();
  }
  else if (inS.startsWith("TESTPATTERN"))
  {
//    Serial.println(inS);
    testPattern();
  }
  else if (inS.startsWith("TESTPENWIDTHSQUARE"))
  {
    testPenWidth();
  }
  else if (inS.startsWith("TESTPENWIDTHSCRIBBLE"))
  {
    testPenWidthScribble();
  }
  else if (inS.startsWith("PENDOWN"))
  {
    penDown();
  }
  else if (inS.startsWith("PENUP"))
  {
    penUp();
  }
  else
  {
    Serial.print("Sorry, ");
    Serial.print(inS);
    Serial.println(" isn't a command I recognise.");
    ready();
  }

//  Serial.println("After execute:");
//  outputAvailableMemory();
  
}

boolean parseCommand(String inS)
{
  String endChars = inS.substring(inS.length() - 4);
  if (endChars.equals(",END"))
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

float asInt(String inParam)
{
  char paramChar[inParam.length() + 1];
  inParam.toCharArray(paramChar, inParam.length() + 1);
  return atoi(paramChar);
}
float asFloat(String inParam)
{
  char paramChar[inParam.length() + 1];
  inParam.toCharArray(paramChar, inParam.length() + 1);
  return atof(paramChar);
}

void changeMotorSpeed()
{
  float speedChange = asFloat(inParam1);
  Serial.print("Speed changing: ");
  Serial.print(speedChange);
  currentMaxSpeed += speedChange;
  accelA.setMaxSpeed(currentMaxSpeed);
  accelB.setMaxSpeed(currentMaxSpeed);
  Serial.print(", to ");
  Serial.println(currentMaxSpeed);
 }
void changeMotorAcceleration()
{
  float speedChange = asFloat(inParam1);
  Serial.print("Acceleration changing: ");
  Serial.print(speedChange);
  currentAcceleration += speedChange;
  accelA.setAcceleration(currentAcceleration);
  accelB.setAcceleration(currentAcceleration);
  Serial.print(", to ");
  Serial.println(currentAcceleration);
 }

void changePenWidth()
{
  penWidth = asFloat(inParam1);
  Serial.print("Changed Pen width to ");
  Serial.print(penWidth);
  Serial.println("mm");
 }   

 void changeDrawingDirection() {
  rowAxis = inParam1;
    
  if (inParam2.equals(DIRECTION_STRING_LTR))
    drawingLeftToRight = true;
  else
    drawingLeftToRight = false;
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
    
//    Serial.print("Command:");
//    Serial.print(inParam1);
//    Serial.print(", p1:");
//    Serial.print(inParam1);
//    Serial.print(", p2:");
//    Serial.print(inParam2);
//    Serial.print(", p3:");
//    Serial.print(inParam3);
//    Serial.print(", p4:");
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
        drawSquarePixel(w, i, ltr);
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
    
    boolean ltr = true;
    
    float oldPenWidth = penWidth;
    int iterations = 0;
    
    for (float pw = startWidth; pw <= endWidth; pw+=incSize)
    {
      iterations++;
      penWidth = pw;
      int maxDens = maxDensity(penWidth, rowWidth);
      Serial.print("Penwidth test ");
      Serial.print(iterations);
      Serial.print(", pen width: ");
      Serial.print(penWidth);
      Serial.print(", max density: ");
      Serial.println(maxDens);
      drawSquarePixel(rowWidth, maxDens, true);
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
      Serial.print("Penwidth test ");
      Serial.print(iterations);
      Serial.print(", pen width: ");
      Serial.print(penWidth);
      Serial.print(", max density: ");
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

void drawSquarePixel() {
    int originA = asInt(inParam1);
    int originB = asInt(inParam2);
    int size = asInt(inParam3);
    int density = asInt(inParam4);

    int halfSize = size / 2;
    
    int startPoint;
    int endPoint;
    int endPoint2;

    int calcFullSize = halfSize * 2; // see if there's any rounding errors
    int offsetStart = size - calcFullSize;

    if (drawingLeftToRight) {
      startPoint = originA - (halfSize);
      startPoint -= offsetStart;
      endPoint = originA + (halfSize);
    }
    else {
      startPoint = originA + (halfSize);
      startPoint += offsetStart;
      endPoint = originA - (halfSize);
    }

    density = scaleDensity(density, 255, maxDensity(penWidth, size));
    changeLength(startPoint, originB);
    if (density > 1)
    {
      drawSquarePixel(size, density, drawingLeftToRight);
    }
    changeLength(endPoint, originB);
    
    outputAvailableMemory(); 
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
    
  Serial.print("Min segment size for penSize ");
  Serial.print(penSize);
  Serial.print(": ");
  Serial.print(minSegSize);
  Serial.println(" steps.");
  
  return minSegSize;
}

int maxDensity(float penSize, int rowSize)
{
  float rowSizeInMM = mmPerStep * rowSize;
  Serial.print("rowsize in mm: ");
  Serial.print(rowSizeInMM);
  Serial.print(", mmPerStep: ");
  Serial.print(mmPerStep);
  Serial.print(", rowsize: ");
  Serial.println(rowSize);
  
  float numberOfSegments = rowSizeInMM / penSize;
  int maxDens = 1;
  if (numberOfSegments >= 2.0)
    maxDens = int(numberOfSegments);
    
//  Serial.print("num of segments float:");
//  Serial.println(numberOfSegments);
//
//    
  Serial.print("Max density: penSize: ");
  Serial.print(penSize);
  Serial.print(", rowSize: ");
  Serial.print(rowSize);
  Serial.println(maxDens);
  
  return maxDens;
}

int scaleDensity(int inDens, int inMax, int outMax)
{
  float reducedDens = (float(inDens) / float(inMax)) * float(outMax);
  reducedDens = outMax-reducedDens;
  Serial.print("inDens:");
  Serial.print(inDens);
  Serial.print(", inMax:");
  Serial.print(inMax);
  Serial.print(", outMax:");
  Serial.print(outMax);
  Serial.print(", reduced:");
  Serial.println(reducedDens);
  
  // round up if bigger than .5
  int result = int(reducedDens);
  if (reducedDens - (result) > 0.5)
    result ++;
  
  return result;
}

void drawSquarePixel(int size, int density, boolean drawingLeftToRight) {
  
  // work out how wide each segment should be
  int segmentWidth = 0;
  int segmentWidthSoFar = 0;

  if (density < 1)
  {
    if (drawingLeftToRight)
      moveA(size);
    else
      moveA(0-size);
  }
  else
  {
    // work out some segment widths
    int basicSegWidth = size / density;
    int basicSegRemainder = size % density;
    float remainderPerSegment = float(basicSegRemainder) / float(density);
    float totalRemainder = 0.0;
    int distanceSoFar = 0;
    
//    Serial.print("Basic seg width:");
//    Serial.print(basicSegWidth);
//    Serial.print(", basic seg remainder:");
//    Serial.print(basicSegRemainder);
//    Serial.print(", remainder per seg");
//    Serial.println(remainderPerSegment);
    
    for (int i = 0; i <= density; i++) 
    {
//      Serial.print("segment #");
//      Serial.print(i);
      
      totalRemainder += remainderPerSegment;
//      Serial.print(", totalRemainder:");
//      Serial.print(totalRemainder);

      if (totalRemainder >= 1.0)
      {
        totalRemainder -= 1.0;
        segmentWidth = basicSegWidth+1;
      }
      else
      {
        segmentWidth = basicSegWidth;
      }

//      Serial.print("segment #");
//      Serial.print(i);
//      Serial.print(", SegmentWidth: ");
//      Serial.println(segmentWidth);

      
      if (!drawingLeftToRight) {
        segmentWidth = 0 - segmentWidth; // reverse
      }
      
      
      if (i == 0) 
      { // first one, half a line and an along
//        Serial.println("First one.");
        if (lastWaveWasTop) {
          moveB(size/2);
          moveA(segmentWidth);
        }
        else {
          moveB(0-(size/2));
          moveA(segmentWidth);
        }
        flipWaveDirection();
      }
      else if (i == density) 
      { // last one, half a line with no along
//        moveA(segmentWidth);
          
        if (lastWaveWasTop) {
          moveB(size/2);
        }
        else {
          moveB(0-(size/2));
        }
      }
      else 
      { // intervening lines - full lines, and an along
        if (lastWaveWasTop) {
          moveB(size);
          moveA(segmentWidth);
        }
        else {
          moveB(0-size);
          moveA(segmentWidth);
        }
        flipWaveDirection();
      }
      
      distanceSoFar += segmentWidth;
//      Serial.print("distance so far:");
//      Serial.print(distanceSoFar);
      
      reportPosition();
    }
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
  Serial.print("SYNC,");
  Serial.print(accelA.currentPosition());
  Serial.print(",");
  Serial.print(accelB.currentPosition());
  Serial.println(",END");
  outputAvailableMemory();
}


void setPosition()
{
  int targetA = asInt(inParam1);
  int targetB = asInt(inParam2);

  accelA.setCurrentPosition(targetA);
  accelB.setCurrentPosition(targetB);
  
  reportPosition();
}



void outputAvailableMemory()
{
  int avMem = availableMemory();
  if (avMem != availMem)
  {
    availMem = avMem;
    Serial.print(FREE_MEMORY_STRING);
    Serial.print(availMem);
    Serial.println(",END");
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

