//#include <AFMotorSPI.h>
#include <AFMotor.h>
#include <AccelStepper.h>
#include <avr/pgmspace.h>
#include <Servo.h>

// Pen raising servo
Servo penHeight;
int const UP_POSITION = 90;
int const DOWN_POSITION = 180;
int const PEN_HEIGHT_SERVO_PIN = 10;
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

int pageWidth = 712 * stepsPerMM; // previously 712

static String rowAxis = "A";
const int INLENGTH = 50;
const char INTERMINATOR = 10;

const String DIRECTION_STRING_LTR = "LTR";
const int SRAM_SIZE = 8196;
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

int roveMaxA = 0;
int roveMinA = 0;
int roveMaxB = 0;
int roveMinB = 0;

boolean roving = false;

const static String CMD_EXEC = "EXEC";
const static String CMD_ACK = "ACK";

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

const static String CMD_LOADMAGEFILE = "C23";

const static String CMD_STARTROVE = "C19";
const static String CMD_STOPROVE = "C20";
const static String CMD_SETROVEAREA = "C21";


const static String CMD_END = ",END";

struct ImageIndex {
  String imageName;
  long firstImageBlock;
  int headerOffset;
  int width;
  int height;
};

static ImageIndex currentImageIndex;


void setup() 
{
  Serial.begin(57600);           // set up Serial library at 9600 bps
  print_P(PSTR("POLARGRAPH ON!"));
  Serial.println();

  pinMode(53, OUTPUT); // necessary for SD card reading to work


  accelA.setMaxSpeed(currentMaxSpeed);
  accelA.setAcceleration(currentAcceleration);  
  accelB.setMaxSpeed(currentMaxSpeed);
  accelB.setAcceleration(currentAcceleration);
  
  accelA.setMinPulseWidth(10);
  accelB.setMinPulseWidth(10);

  float startLength = ((float) startLengthMM / (float) mmPerRev) * (float) motorStepsPerRev;
  accelA.setCurrentPosition(startLength);
  accelB.setCurrentPosition(startLength);

  drawingLeftToRight = true;
  
  //testServoRange();
  movePenUp();

  readyString = READY;
  establishContact();
  delay(500);
  outputAvailableMemory();
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
  for (int i=DOWN_POSITION; i>UP_POSITION; i--) {
    Serial.println(i);
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
  for (int i=UP_POSITION; i<DOWN_POSITION; i++) {
    Serial.println(i);
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
  print_P(PSTR(","));
  Serial.println(command);
}

void loop()
{
  // send ready
  // wait for instruction
  int idleTime = millis();
  while (!lastCommandConfirmed)
  {
    rove();
    // idle
    String inS = "";

    // get incoming command
    while (inS.length() == 0)
    {
      rove();
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
      println_P(PSTR("Command confirmed:"));
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
    println_P(PSTR("Executing command."));
    executeCommand(lastCommand);
    lastCommand = "";
    lastCommandConfirmed = false;
    ready();
  }
  else
  {
    println_P(PSTR("Command not parsed."));
    lastCommand = "";
    lastCommandConfirmed = false;
    ready();
  }
//  ready();
}

void rove()
{
  if (roving)
  {
    if (accelA.distanceToGo() != 0)
    {
      accelA.run();
    }
    else
    {
      // choose new position for accelA to head to
      int rand = random(roveMinA, roveMaxA);
      accelA.moveTo(rand);
    }

    if (accelB.distanceToGo() != 0)
    {
      accelB.run();
    }
    else
    {
      // choose new position for accelA to head to
      int rand = random(roveMinB, roveMaxB);
      accelB.moveTo(rand);
    }
    
    byte brightness = getPixelBrightness(accelA.currentPosition(), accelB.currentPosition());
    
  }
}

byte getPixelBrightness(float aPos, float bPos)
{
  if (isImageLoaded())
  {
    // work out cartesian coordinates
    int cX = getCartesianX() / stepsPerMM;
    int cY = getCartesianY(cX) / stepsPerMM;
    Serial.print("x:");
    Serial.print(cX);
    Serial.print(",y:");
    Serial.println(cY);
  }
  else
    return 255;
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
  else if (inS.startsWith(CMD_DRAWPIXEL))
  {
    // go to coordinates.
    drawSquarePixel();
  }  
  else if (inS.startsWith(CMD_DRAWSCRIBBLEPIXEL))
  {
//    Serial.println(inS);
    // go to coordinates.
    drawScribblePixel();
  }  
  else if (inS.startsWith(CMD_DRAWRECT))
  {
//    Serial.println(inS);
    // go to coordinates.
    drawRectangle();
  }
  else if (inS.startsWith(CMD_CHANGEDRAWINGDIRECTION))
  {
//    Serial.println(inS);
    changeDrawingDirection();
  }
  else if (inS.startsWith(CMD_SETPOSITION))
  {
//    Serial.println(inS);
    setPosition();
  }
  else if (inS.startsWith(CMD_TESTPATTERN))
  {
//    Serial.println(inS);
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
  else if (inS.startsWith(CMD_LOADMAGEFILE))
  {
    findImageFile();
  }
  else if (inS.startsWith(CMD_STARTROVE))
  {
    startRoving();
  }
  else if (inS.startsWith(CMD_STOPROVE))
  {
    stopRoving();
  }
  else if (inS.startsWith(CMD_SETROVEAREA))
  {
    setRoveArea();
  }
  else
  {
    print_P(PSTR("Sorry, "));
    Serial.print(inS);
    print_P(PSTR(" isn't a command I recognise."));
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

void startRoving()
{
  if (isRoveAreaSet())
  {
    print_P(PSTR("Started roving."));
    accelA.moveTo(roveMinA);
    accelB.moveTo(roveMinB);
    while (accelA.distanceToGo() != 0 && accelB.distanceToGo() != 0)
    {
      accelA.run();
      accelB.run();
      reportPosition();
    }
    roving = true;
    delay(2000);
  }
}
void stopRoving()
{
  roving = false;
  print_P(PSTR("Stopped roving."));
  outputAvailableMemory();
}

void setRoveArea()
{
  roveMinA = asInt(inParam1);
  roveMinB = asInt(inParam2);
  roveMaxA = asInt(inParam3);
  roveMaxB = asInt(inParam4);
  if (isRoveAreaSet())
  {
    print_P(PSTR("Rove area set: "));
    Serial.print(roveMaxA);
    print_P(PSTR(", "));
    Serial.print(roveMinA);
    print_P(PSTR(", "));
    Serial.print(roveMaxB);
    print_P(PSTR(", "));
    Serial.println(roveMinB);
  }
}

boolean isRoveAreaSet()
{
  if (roveMaxA == 0 
  || roveMinA == 0
  || roveMaxB == 0
  || roveMinB == 0)
    return false;
  else
    return true;
}

boolean isImageLoaded()
{
  if (currentImageIndex.firstImageBlock < 0)
    return false;
  else
    return true;
}

void findImageFile()
{
  /*
    file index commands looks like
    C23, (4)
    filename________ (8)
    ,END
  */
  String filename = inParam1;
  Serial.println(filename);
  
  initSD();
  boolean imageFound = findPolargraphImage(filename);
  print_P(PSTR("Image called "));
  Serial.print(filename);
  if (imageFound)
  {
    print_P(PSTR(" was found in block "));
    Serial.println(currentImageIndex.firstImageBlock);
  }
  else
  {
    print_P(PSTR(" could not be found on the card."));
    Serial.println();
    currentImageIndex.firstImageBlock = -1;
  }
}

void changeMotorSpeed()
{
  float speedChange = asFloat(inParam1);
  print_P(PSTR("Speed changing: "));
  Serial.print(speedChange);
  currentMaxSpeed += speedChange;
  accelA.setMaxSpeed(currentMaxSpeed);
  accelB.setMaxSpeed(currentMaxSpeed);
  print_P(PSTR(", to "));
  Serial.println(currentMaxSpeed);
 }
void changeMotorAcceleration()
{
  float speedChange = asFloat(inParam1);
  print_P(PSTR("Acceleration changing: "));
  Serial.print(speedChange);
  currentAcceleration += speedChange;
  accelA.setAcceleration(currentAcceleration);
  accelB.setAcceleration(currentAcceleration);
  print_P(PSTR(", to "));
  Serial.println(currentAcceleration);
 }

void changePenWidth()
{
  penWidth = asFloat(inParam1);
  print_P(PSTR("Changed Pen width to "));
  Serial.print(penWidth);
  print_P(PSTR("mm"));
  Serial.println();
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
      print_P(PSTR("Penwidth test "));
      Serial.print(iterations);
      print_P(PSTR(", pen width: "));
      Serial.print(penWidth);
      print_P(PSTR(", max density: "));
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
      print_P(PSTR("Penwidth test "));
      Serial.print(iterations);
      print_P(PSTR(", pen width: "));
      Serial.print(penWidth);
      print_P(PSTR(", max density: "));
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
  
  int inc = 1;
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
    
  print_P(PSTR("Min segment size for penSize "));
  Serial.print(penSize);
  print_P(PSTR(": "));
  Serial.print(minSegSize);
  print_P(PSTR(" steps."));
  Serial.println();
  
  return minSegSize;
}

int maxDensity(float penSize, int rowSize)
{
  float rowSizeInMM = mmPerStep * rowSize;
  print_P(PSTR("rowsize in mm: "));
  Serial.print(rowSizeInMM);
  print_P(PSTR(", mmPerStep: "));
  Serial.print(mmPerStep);
  print_P(PSTR(", rowsize: "));
  Serial.println(rowSize);
  
  float numberOfSegments = rowSizeInMM / penSize;
  int maxDens = 1;
  if (numberOfSegments >= 2.0)
    maxDens = int(numberOfSegments);
    
//  Serial.print("num of segments float:");
//  Serial.println(numberOfSegments);
//
//    
  print_P(PSTR("Max density: penSize: "));
  Serial.print(penSize);
  print_P(PSTR(", rowSize: "));
  Serial.print(rowSize);
  Serial.println(maxDens);
  
  return maxDens;
}

int scaleDensity(int inDens, int inMax, int outMax)
{
  float reducedDens = (float(inDens) / float(inMax)) * float(outMax);
  reducedDens = outMax-reducedDens;
  print_P(PSTR("inDens:"));
  Serial.print(inDens);
  print_P(PSTR(", inMax:"));
  Serial.print(inMax);
  print_P(PSTR(", outMax:"));
  Serial.print(outMax);
  print_P(PSTR(", reduced:"));
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
  print_P(PSTR("SYNC,"));
  Serial.print(accelA.currentPosition());
  print_P(PSTR(","));
  Serial.print(accelB.currentPosition());
  Serial.println(CMD_END);
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

int getCartesianX() {
  int calcX = int((pow(pageWidth, 2) - pow(accelB.currentPosition(), 2) + pow(accelA.currentPosition(), 2)) / (pageWidth*2));
  return calcX;  
}

int getCartesianY() {
  return getCartesianY(getCartesianX());
}
int getCartesianY(int cX) {
  int calcX = cX;
  int calcY = int(sqrt(pow(accelA.currentPosition(),2)-pow(calcX,2)));
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

void print_P(const char *data) {         // print prompts and messages from program flash to save SRAM
   while (pgm_read_byte(data) != 0x00)
     Serial.print(pgm_read_byte(data++));
}
void println_P(const char *data) {
  print_P(data);
  Serial.println();
}

void error_P(const char* message)        // report an error then hang, flashing the on-board led
{
  print_P(PSTR("Error  "));
  print_P(message);
  print_P(PSTR("\n<press reset>"));
  for(;;)
  {
    digitalWrite(13, (millis() / 512) & 1);
  }
}


/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/********************   FILE STUFF   *************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/

// Ports
int PIN_CS = PINB2;      // chip select
int PIN_MOSI = PINB3;    // master out slave in
int PIN_MISO = PINB4;    // master in slave out
int PIN_CLOCK = PINB5;   // clock

/********************** SPI SECTION BELOW **********************/

// SPI Variables
byte clr;     // dummy variable used to clear some of the SPI registers
byte spi_err; // SPI timeout flag, must be cleared manually

// send an SPI command, includes time out management
// returns spi_err: "0" is "no error"
byte spi_cmd(volatile char data) {
//  print_P(PSTR("spicmd:"));
//  Serial.println(data, HEX);
  
  spi_err = 0; // reset spi error
  SPDR = data; // start the transmission by loading the output byte into the spi data register
  int i = 0;
  while (!(SPSR & (1<<SPIF))) {
    i++;
    if (i >= 0xFF) {
      spi_err = 1;
      return(0x00);
    }
  }
  // returned value
  return(SPDR); 
}

// initialize SPI port 
void spi_initialize(void) {
  SPCR = (1<<SPE) | (1<<MSTR); // spi enabled, master mode
  clr = SPSR; // dummy read registers to clear previous results
  clr = SPDR;
}

/********************** SD CARD SECTION BELOW **********************/

// SD Card variables
const int blockSize = 512;          // block size (default 512 bytes)
byte vBlock[blockSize];        // set vector containing data that will be recorded on SD Card
byte vBuffer[16];

#define GO_IDLE_STATE 0x00     // resets the SD card
#define SEND_CSD 0x09          // sends card-specific data
#define SEND_CID 0x0A          // sends card identification 
const int READ_SINGLE_BLOCK = 0x11; // reads a block at byte address 
#define WRITE_BLOCK 0x18       // writes a block at byte address
#define SEND_OP_COND 0x29      // starts card initialization
#define APP_CMD 0x37           // prefix for application command 


// Send a SD command, num is the actual index, NOT OR'ed with 0x40. 
// arg is all four bytes of the argument
byte sdc_cmd(byte commandIndex, long arg) {
//  Serial.print(commandIndex, HEX);
//  Serial.print(", arg:");
//  Serial.println(arg);
  PORTB &= ~(1<<PIN_CS);   // assert chip select for the card
  spi_cmd(0xFF);           // dummy byte
  commandIndex |= 0x40;    // command token OR'ed with 0x40 
  spi_cmd(commandIndex);   // send command
  for (int i=3; i>=0; i--) {
    spi_cmd(arg>>(i*8));   // send argument in little endian form (MSB first)
  }
  spi_cmd(0x95);           // checksum valid for GO_IDLE_STATE, not needed thereafter, so we can hardcode this value
  spi_cmd(0xFF);           // dummy byte gives card time to process
  byte res = spi_cmd(0xFF);
//  print_P(PSTR("result in sdc:"));
//  Serial.print(res, DEC);
  return (res);  // query return value from card
}

// initialize SD card 
// retuns 1 if successful
byte sdc_initialize(void) {
  // set slow clock: 1/128 base frequency (125Khz in this case)
  SPCR |=  (1<<SPR1) | (1<<SPR0); // set slow clock: 1/128 base frequency (125Khz in this case)
  SPSR &= ~(1<<SPI2X);            // No doubled clock frequency
  // wake up SD card
  PORTB |=  (1<<PIN_CS);          // deasserts card for warmup
  PORTB |=  (1<<PIN_MOSI);        // set MOSI high
  for(byte i=0; i<10; i++) {
    spi_cmd(0xFF);                // send 10 times 8 pulses for a warmup (74 minimum)
  }
  // set idle mode
  byte retries=0;
  PORTB &= ~(1<<PIN_CS);          // assert chip select for the card
  while(sdc_cmd(GO_IDLE_STATE, 0) != 0x01) { // while SD card is not in iddle state
    retries++;
    if (retries >= 0xFF) {
      return(NULL); // timed out!
    }
    delay(5);
  }
  // at this stage, the card is in idle mode and ready for start up
  retries = 0;
  sdc_cmd(APP_CMD, 0); // startup sequence for SD cards 55/41
  while (sdc_cmd(SEND_OP_COND, 0) != 0x00) {
    retries++;
    if (retries >= 0xFF) {
      return(NULL); // timed out!
    }
    sdc_cmd(APP_CMD, 0); 
  }
  // set fast clock, 1/4 CPU clock frequency (4Mhz in this case)
  SPCR &= ~((1<<SPR1) | (1<<SPR0)); // Clock Frequency: f_OSC / 4 
  SPSR |=  (1<<SPI2X);              // Doubled Clock Frequency: f_OSC / 2 
  return (0x01); // returned value (success)
}

// clear block content
void sdc_clearVector(void) {
  for (int i=0; i<blockSize; i++) {
    vBlock[i] = 0;
  }
}

void dumpVector()
{
  print_P(PSTR("Chars"));
  Serial.println();
  for (int i = 0; i < blockSize; i++)
  {
    Serial.print(vBlock[i]);
    Serial.print(" ");
    if ((i+1) % 32 == 0)
      Serial.println();
  }
  print_P(PSTR("Decimals"));
  Serial.println();
  for (int i = 0; i < blockSize; i++)
  {
    Serial.print(vBlock[i], DEC);
    Serial.print(" ");
    if ((i+1) % 16 == 0)
      Serial.println();
  }
}

// get nbr of blocks on SD memory card from
long sdc_totalNbrBlocks(void) {
  sdc_readRegister(SEND_CSD);
  // compute size
  long C_Size = ((vBuffer[0x08] & 0xC0) >> 6) | ((vBuffer[0x07] & 0xFF) << 2) | ((vBuffer[0x06] & 0x03) << 10);
  long C_Mult = ((vBuffer[0x08] & 0x80) >> 7) | ((vBuffer[0x08] & 0x03) << 2);
  return ((C_Size+1) << (C_Mult+2)); 
}

// read SD card register content and store it in vBuffer
void sdc_readRegister(byte sentCommand) {
  byte retries=0x00;
  byte res=sdc_cmd(sentCommand, 0); 
  while(res != 0x00) { 
    delay(1);
    retries++;
    if (retries >= 0xFF) return; // timed out!
    res=spi_cmd(0xFF); // retry
  }  
  // wait for data token
  while (spi_cmd(0xFF) != 0xFE); 
  // read data
  for (int i=0; i<16; i++) {
    vBuffer[i] = spi_cmd(0xFF);
  }
  // read CRC (lost results in blue sky)
  spi_cmd(0xFF); // LSB
  spi_cmd(0xFF); // MSB
}

// write block on SD card 
// addr is the address in bytes (multiples of block size)
void sdc_writeBlock(long blockIndex) {
  byte retries=0;
  while(sdc_cmd(WRITE_BLOCK, blockIndex * blockSize) != 0x00) { 
    delay(1);
    retries++;
    if (retries >= 0xFF) return; // timed out!
  }
  spi_cmd(0xFF); // dummy byte (at least one)
  // send data packet (includes data token, data block and CRC)
  // data token
  spi_cmd(0xFE);
  // copy block data
  for (int i=0; i<blockSize; i++) {
    spi_cmd(vBlock[i]); 
  }
  // write CRC (lost results in blue sky)
  spi_cmd(0xFF); // LSB
  spi_cmd(0xFF); // MSB
  // wait until write is finished
  while (spi_cmd(0xFF) != 0xFF) delay(1); // kind of NOP
}

// read block on SD card and copy data in block vector
// retuns 1 if successful
void sdc_readBlock(long blockIndex) {
  byte retries = 0x00;
  byte res = sdc_cmd(READ_SINGLE_BLOCK,  (blockIndex * blockSize));
//  Serial.println(res);
  while(res != 0x00) { 
    delay(1);
    retries++;
    if (retries >= 0xFF) return; // timed out!
    res=spi_cmd(0xFF); // retry
  }
  // read data packet (includes data token, data block and CRC)
  // read data token
  while (spi_cmd(0xFF) != 0xFE); 
  // read data block
  for (int i=0; i<blockSize; i++) {
//    Serial.print("i:");
//    Serial.print(i);
    vBlock[i] = spi_cmd(0xFF); // read data
  }
  // read CRC (lost results in blue sky)
  spi_cmd(0xFF); // LSB
  spi_cmd(0xFF); // MSB
}

long getLongFromVector(long addr)
{
  // start address
  byte inBuf[4];
  for (int i = addr; i < addr+4; i++)
  {
    inBuf[i-addr] = vBlock[i];
  }
  long result = ((long)inBuf[0]) << 24;
  result |= ((long)inBuf[1]) << 16;
  result |= ((long)inBuf[2]) << 8;
  result |= inBuf[3];
  
  return result;
}

/********************** MAIN ROUTINES SECTION  BELOW **********************/

void initSD()
{
  // Set ports
  // Data in
  DDRB &= ~(1<<PIN_MISO);
  // Data out
  DDRB |=  (1<<PIN_CLOCK);
  DDRB |=  (1<<PIN_CS);
  DDRB |=  (1<<PIN_MOSI);  
  // Initialize SPI and SDC 
  spi_err=0;        // reset SPI error
  spi_initialize(); // initialize SPI port
  sdc_initialize(); // Initialize SD Card
  Serial.println();
  Serial.print(sdc_totalNbrBlocks(), DEC);
  print_P(PSTR(" blocks"));
  Serial.println();
  
}

boolean findPolargraphImage(String imageName)
{
  boolean fileFound = false;
  print_P(PSTR("Listing files:"));
  Serial.println();
  long totalBlocks = sdc_totalNbrBlocks();
  Serial.println(totalBlocks);
  for (int i = 0; i < totalBlocks; i++)
  {
    // get the block
    outputAvailableMemory();
    if (blockIsImage(i))
    {
      print_P(PSTR("Block is image!"));
      Serial.println();
      char* text_cstr = "                ";
      imageName.toCharArray(text_cstr, 17);
      if (blockIsImageName(text_cstr))
      {
        Serial.print(imageName);
        print_P(PSTR(" found. Block "));
        Serial.println(i);
        currentImageIndex.imageName=imageName;
        currentImageIndex.firstImageBlock = i;
        currentImageIndex.headerOffset = 64;
        currentImageIndex.width = getImageWidthFromBlock();
        currentImageIndex.height = getImageHeightFromBlock();
        fileFound = true;
        break;
      }
      else
      {
        print_P(PSTR("not matched."));
      }
    }
  }
}

int getImageWidthFromBlock()
{
  // get eigtht bytes to find the width
  char* text_cstr = "00000000";
  for (byte i = 48; i < 56; i++)
  {
    text_cstr[i-48] = vBlock[i];
  }
  
  int widthInt = atoi(text_cstr);
  Serial.print("width:");
  Serial.println(widthInt);
  return widthInt;
}

int getImageHeightFromBlock()
{
}

boolean blockIsImage(long block)
{
  sdc_readBlock(block);
  if (vBlock[0] == 'P' 
    && vBlock[1] == 'O'
    && vBlock[2] == 'L'
    && vBlock[3] == 'A'
    && vBlock[4] == 'R'
    && vBlock[5] == 'G'
    && vBlock[6] == 'R'
    && vBlock[7] == 'A'
    && vBlock[8] == 'P'
    && vBlock[9] == 'H'
    && vBlock[10] == '_'
    && vBlock[11] == 'I'
    && vBlock[12] == 'M'
    && vBlock[13] == 'A'
    && vBlock[14] == 'G'
    && vBlock[15] == 'E'
    )
  {
    // is an image, great
    for (int i = 0; i < 64; i++)
    {
      Serial.print(vBlock[i]);
    }
    Serial.println();
    return true;
  }
  else
    return false;
}

boolean blockIsImageName(char imgName[17])
{
  if (vBlock[32] == imgName[0] 
    && vBlock[33] == imgName[1]
    && vBlock[34] == imgName[2]
    && vBlock[35] == imgName[3]
    && vBlock[36] == imgName[4]
    && vBlock[37] == imgName[5]
    && vBlock[38] == imgName[6]
    && vBlock[39] == imgName[7]
    && vBlock[40] == imgName[8]
    && vBlock[41] == imgName[9]
    && vBlock[42] == imgName[10]
    && vBlock[43] == imgName[11]
    && vBlock[44] == imgName[12]
    && vBlock[45] == imgName[13]
    && vBlock[46] == imgName[14]
    && vBlock[47] == imgName[15]
    )
  {
    // is an image, great
    return true;
  }
  else
  {
    return false;
  }
}


