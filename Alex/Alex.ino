#include <math.h>
#include <serialize.h>

#include "packet.h"
#include "constants.h"

typedef enum {
  STOP=0,
  FORWARD=1,
  BACKWARD=2,
  LEFT=3,
  RIGHT=4
} TDirection;

volatile TDirection dir = STOP;

/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV      195

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.42

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  5   // Left forward pin
#define LR                  6   // Left reverse pin
#define RF                  11  // Right forward pin
#define RR                  10  // Right reverse pin

void forward(float dist, float speed);
void reverse(float dist, float speed);
void left(float angle, float speed);
void right(float angle, float speed);
void clearCounters();

// PI, for calculating turn circumference
#define PI                  3.141592654

// Alex Length & Breadth in cm    // TO BE UPDATED
#define ALEX_LENGTH         17.5
#define ALEX_BREADTH        11

// Alex's diagonal. Compute and stored once
const float alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));

// Alex's turning circumference, calculated once
const float alexCirc = PI * alexDiagonal;

// Colour Init
#define s0 A0       
#define s1 A1
#define out A2
#define s2 A3
#define s3 A4
#define trig 12 
#define echo 13

int color[3] = {0,0,0};
float distance;
float soundSpeed = 0.0345;

int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

float redColour = 0;
float greenColour = 0;
float blueColour = 0;

/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long rightReverseTicks;

// Left and right encoder ticks for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

// Variables to track whether we have moved a commanded distance
unsigned long deltaDist;
unsigned long newDist;

unsigned long deltaTicks;
unsigned long targetTicks;


/*
 * 
 * Alex Communication Routines.
 * 
 */
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;

  unsigned long paramArray[10] = {leftForwardTicks, rightForwardTicks, leftReverseTicks, rightReverseTicks,
  leftForwardTicksTurns, rightForwardTicksTurns, leftReverseTicksTurns, rightReverseTicksTurns, forwardDist, reverseDist};

  for (int i=0; i<10; i++)
    statusPacket.params[i] = paramArray[i];
  
  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprintf(char* format, ...) {
  va_list args;
  char buffer[128];
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
  DDRD &= ~(0b00001100);
  PORTD |= 0b00001100;
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  switch (dir) {
    case FORWARD:
      leftForwardTicks++;
      forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
      break;
    case BACKWARD:
      leftReverseTicks++;
      reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
      break;
    case LEFT:
      leftReverseTicksTurns++;
      break;
    case RIGHT:
      leftForwardTicksTurns++;
      break;
  }
}

void rightISR()
{
  switch (dir) {
    case FORWARD:
      rightForwardTicks++;
      break;
    case BACKWARD:
      rightReverseTicks++;
      break;
    case LEFT:
      rightForwardTicksTurns++;
      break;
    case RIGHT:
      rightReverseTicksTurns++;
      break;
  }
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  EICRA |= 0b00001010;
  EIMSK |= 0b00000011;
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.

ISR(INT0_vect)
{
  leftISR();
}

ISR(INT1_vect)
{
  rightISR();
}

// Implement INT0 and INT1 ISRs above.

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(9600);
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
  
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count=0;

  while(Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
}

/*
 * Alex's motor drivers.
 * 
 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B
   *    A2IN - Pin 6, PD6, OC0A
   *    B1IN - Pin 10, PB2, OC1B
   *    B2In - pIN 11, PB3, OC2A
   */
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
  
}

void setupColor() {
  pinMode(s0,OUTPUT);   
  pinMode(s1,OUTPUT);
  pinMode(s2,OUTPUT);
  pinMode(s3,OUTPUT);
  pinMode(out,INPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  digitalWrite(s0,HIGH);
  digitalWrite(s1,HIGH);
}

void distance_check() {
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  distance = pulseIn(echo, HIGH);
  distance *= soundSpeed / 2;
}

void moveObject() {
  clearCounters();
  const float threshold = 7.0;
  const float range = 0.1;
  const int speed = 60;

  distance_check();

  if (distance > (threshold + range))
    forward(distance - threshold, speed);
  else if (distance < (threshold - range))
    reverse(threshold - distance, speed);
  else
    stop();

  clearCounters();
}

void color_check() {
  digitalWrite(s2,LOW); 
  digitalWrite(s3,LOW);
  color[0] = pulseIn(out,LOW);
  delay(20);
                    
  digitalWrite(s2,LOW);
  digitalWrite(s3,HIGH);
  color[1] = pulseIn(out,LOW);
  delay(20);

  digitalWrite(s2,HIGH);
  digitalWrite(s3,HIGH);
  color[2] = pulseIn(out,LOW);
  delay(20);

  delay(200);  
}

int calchue(int color[3]) {
  float max = color[0];
  float min = color[0];

  for (int i = 1; i < 3; i++) {
    if (max < color[i])
      max = color[i];
    if (min > color[i])
      min = color[i];
  }

  if (max == color[0])
    return (color[2] - color[1]) / (max - min);
  else if (max == color[1])
    return 4.0 + (color[0] - color[2]) / (max - min);
  else if (max == color[2])
    return 2.0 + (color[1] - color[0]) / (max - min);
}

void sendColor()
{
  int hue = calchue(color);
  char colorNumStr[7];
  for (int i = 0; i < 3; i++) {
    sendMessage(itoa(color[i], colorNumStr, 10));
  }
  sendMessage(itoa(distance, colorNumStr, 10));
  sendMessage(itoa(hue, colorNumStr, 10));
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if (speed < 0.0)
    speed = 0;

  else if (speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  // Code to tell us how far to move
  if (!dist)
    deltaDist = 999999;
  else
    deltaDist = dist;
  
  newDist = forwardDist + deltaDist;

  dir = FORWARD;

  int val = pwmVal(speed);

  // For now we will ignore dist and move
  // forward indefinitely. We will fix this
  // in Week 9.

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  
  analogWrite(LF, val);
  analogWrite(RF, (int)(val*0.95));
  analogWrite(LR,0);
  analogWrite(RR, 0);
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
  // Code to tell us how far to move
  if (!dist)
    deltaDist = 999999;
  else
    deltaDist = dist;
  
  newDist = reverseDist + deltaDist;

  dir = BACKWARD;

  int val = pwmVal(speed);

  // For now we will ignore dist and 
  // reverse indefinitely. We will fix this
  // in Week 9.

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  analogWrite(LR, val);
  analogWrite(RR, val*0.95);
  analogWrite(LF, 0);
  analogWrite(RF, 0);
}

// New function to estimate number of wheel ticks
// needed to turn an angle
unsigned long computeDeltaTicks (float ang) {
  // We assume angular distance moved = linear distance moved by 1 wheel
  // revolution. Incorrect but simplifies calculation.
  // # of wheel revolutions to make 1 full 360 turn is alexCirc / WHEEL_CIRC
  // For ang degrees, (ang * alexCirc) / (360 * WHEEL_CIRC)
  // To convert to ticks, we multiply by COUNTS_PER_REV

  unsigned long ticks = (unsigned long) ((ang * alexCirc * COUNTS_PER_REV) / (360 * WHEEL_CIRC));

  return ticks;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{
  if(ang == 0)
    deltaTicks = 99999999;
  else {
    ang /= 3;
    deltaTicks = computeDeltaTicks(ang);
  }
  targetTicks = rightReverseTicksTurns + deltaTicks;
    // verify if rightReverseTicksTurns or leftForwardTicksTurns

  dir = RIGHT;

  int val = pwmVal(speed);

  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn right we reverse the right wheel and move
  // the left wheel forward.
  analogWrite(RR, val*0.95);
  analogWrite(LF, val);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  if(ang == 0)
    deltaTicks = 99999999;
  else {
    ang /= 3;
    deltaTicks = computeDeltaTicks(ang);
  }
  targetTicks = leftReverseTicksTurns + deltaTicks;

  dir = LEFT;

  int val = pwmVal(speed);

  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn left we reverse the left wheel and move
  // the right wheel forward.
  analogWrite(LR, val);
  analogWrite(RF, val*0.95);
  analogWrite(LF, 0);
  analogWrite(RR, 0);
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;

  analogWrite(LF, 0);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
  analogWrite(RR, 0);
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks=0;
  leftReverseTicks=0;
  leftForwardTicksTurns=0;
  leftReverseTicksTurns=0;
  rightForwardTicks=0;
  rightReverseTicks=0;
  rightForwardTicksTurns=0;
  rightReverseTicksTurns=0;
  leftRevs=0;
  rightRevs=0;
  forwardDist=0;
  reverseDist=0; 
}

// Intialize Alex's internal states
void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    // Stop command
    case COMMAND_STOP:
      sendOK();
      stop();
      break;

    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      sendOK();
      forward((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_REVERSE:
      sendOK();
      reverse((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_LEFT:
      sendOK();
      left((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_RIGHT:
      sendOK();
      right((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_GET_STATS:
      sendStatus();
      break;

    case COMMAND_CLEAR_STATS:
      sendOK();
      initializeState();
      break;

    case COMMAND_GET_COLOR:
      moveObject();
      color_check();
      sendColor();
      break;
        
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
     

        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setup() {
  // put your setup code here, to run once:

  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  sei();
  setupColor();
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

// Function to control movement

void movement() {
  if(deltaDist > 0) {
    if(dir == FORWARD) {
      if(forwardDist >= newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }

    else if(dir == BACKWARD) {
      if(reverseDist >= newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }

    else if(dir == STOP) {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }
}

// Function to control rotation

void turning() {
  if(deltaTicks >= 0) {
    if (dir == LEFT) {
      if (leftReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }

    else if (dir == RIGHT) {
      if (rightReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }

    else if (dir == STOP) {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }
}

void loop() {

  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else if (result == PACKET_BAD)
    sendBadPacket();
  else if(result == PACKET_CHECKSUM_BAD)
    sendBadChecksum();

  movement();
  turning();
}
