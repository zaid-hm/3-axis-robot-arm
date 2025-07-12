#include <Bluepad32.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
// #include "vectors.h"
Adafruit_PWMServoDriver sb1 = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN 100  // minimum pulse to move servo (gotten through testing)
#define SERVOMAX 550  // pulse to get to 180deg      (gotten through testing)

#define SR0 0
#define SR1 1
#define SR2 2
#define SR3 3

// for serial read
int receivedInt = 0;
int firstValue = 0;
int secondValue = 0;
int thirdValue = 0;
bool firstReceived = false;
bool secondReceived = false;

// controller values
int cX = 0;
int cY = 0;
int cZ = 0;
int grip = 0;

// current values for arm position
float xnow;
float ynow;
float znow;

// for non blocking delay
unsigned long xtime = 0;
unsigned long ytime = 0;
unsigned long ztime = 0;
// unsigned long time=0;

// maximum and minimum values for arm position
int xmax = 400;
int ymax = 400;
int zmax = 300;
int xmin = 30;
int ymin = -400;
int zmin = -30;


// arm dimentions
float T = 300;
float F = 250;
float H = 30;

int mdelay = 15;  //controls movement speed(lower is faster) // depricated

float offsets[3] = { 90, 0, 0 };

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

bool rumble = false;
void processGamepad(ControllerPtr ctl) {
  cX = ctl->axisY();
  cY = ctl->axisX();
  cZ = ctl->axisRY();
  grip = ctl->throttle();
  if (rumble) {
    ctl->playDualRumble(0, 250 /* durationMs */, 0x80, 0x40);
    rumble = false;
  }
  // Serial.printf("X:%d, Y:%d, Z:%d\n",cX,cY,cZ);
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}


int angleToPulse(int ang, bool invert = false) {
  int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
  if (invert)
    pulse = map(pulse, SERVOMIN, SERVOMAX, SERVOMAX, SERVOMIN);
  return pulse;
}



void m2points(int leg, float x1, float y1, float z1, float x2, float y2, float z2, int n) {
  float xp[n];
  float yp[n];
  float zp[n];
  xp[0] = x1;
  yp[0] = y1;
  zp[0] = z1;
  xp[n - 1] = x2;
  yp[n - 1] = y2;
  zp[n - 1] = z2;
  float xi = (x2 - x1) / (n - 1);
  float yi = (y2 - y1) / (n - 1);
  float zi = (z2 - z1) / (n - 1);
  
  for (int i = 1; i < n - 1; i++) {
    xp[i] = xp[i - 1] + xi;
    yp[i] = yp[i - 1] + yi;
    zp[i] = zp[i - 1] + zi;
  }
  for (int i = 1; i < n; i++) {
    moveArm(xp[i], yp[i], zp[i]);

    delay(mdelay);
  }
}

void moveArm(float x, float y, float z) {
  // Serial.printf("ma: %d, %d, %d\n", x, y, z);

  // z = -1 * z;
  float a1 = atan(y / (x + H));

  x = sqrt(pow(x + H, 2) + pow(y, 2)) - H;
  float L = sqrt(x * x + z * z);


  float a2 = acos((F * F + L * L - T * T) / (2 * F * L)) + atan(z / x);
  float a3 = acos((F * F + T * T - L * L) / (2 * F * T));
  a1 = a1 * 57.2958;
  a2 = a2 * 57.2958;
  a3 = a3 * 57.2958;

  sb1.setPWM(0, 0, angleToPulse(a1 + offsets[0], true));
  sb1.setPWM(1, 0, angleToPulse(a2 + offsets[1]));
  sb1.setPWM(2, 0, angleToPulse(a3 + offsets[2], true));
}

void setup() {
  Serial.begin(115200);
  sb1.begin();

  sb1.setPWMFreq(60);

  moveArm(150, 0, 10);
  xnow = 150;
  ynow = 0;
  znow = 10;

  delay(3000);

  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);
}

void loop() {
  Serial.printf("%f, %f, %f\n", xnow, ynow, znow);
  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();

  controllerMode();

  if (Serial.available() > 0) {
    char receivedChar = Serial.read();
    if (receivedChar != ',') {
      if (firstReceived == false) {
        receivedInt = receivedChar - '0';
        firstValue *= 10;
        firstValue += receivedInt;
      } else if (receivedChar != '\n' && secondReceived == false) {
        secondValue = secondValue * 10 + (receivedChar - '0');
      } else if (receivedChar != '\n' && secondReceived == true) {
        thirdValue = thirdValue * 10 + (receivedChar - '0');
      }

    } else {
      if (firstReceived == true)
        secondReceived = true;
      else
        firstReceived = true;
    }
    if (receivedChar == '\n') {
      
      moveArm(firstValue, secondValue, thirdValue);
      xnow = firstValue;
      ynow = secondValue;
      znow = thirdValue;
      
      firstValue = 0;
      secondValue = 0;
      thirdValue = 0;
      firstReceived = false;
      secondReceived = false;
    }
  }
}

void controllerMode() {

  unsigned long time = millis();

  byte maxDelay = 10;
  byte minDelay = 1;
  byte deadZone = 65;
  float gap = 1;

  int xpos;
  int ypos;
  int zpos;

  if (cX < 0) xpos = 1;
  else xpos = -1;
  if (cY < 0) ypos = -1;
  else ypos = 1;
  if (cZ < 0) zpos = 1;
  else zpos = -1;

  byte xDelay = map(abs(cX), deadZone, 512, maxDelay, minDelay);
  byte yDelay = map(abs(cY), deadZone, 512, maxDelay, minDelay);
  byte zDelay = map(abs(cZ), deadZone, 512, maxDelay, minDelay);
  int hold = map(grip, 0, 1023, 575, 325);


  if (time > xtime + xDelay) {
    xtime = time;
    if (abs(cX) > deadZone) xnow += xpos * gap;
  }

  if (time > ytime + yDelay) {
    ytime = time;
    if (abs(cY) > deadZone) ynow += ypos * gap;
  }

  if (time > ztime + zDelay) {
    ztime = time;
    if (abs(cZ) > deadZone) znow += zpos * gap;
  }

  if (xnow > xmax) {
    xnow = xmax;
    rumble = true;
  }
  if (ynow > ymax) {
    ynow = ymax;
    rumble = true;
  }
  if (znow > zmax) {
    znow = zmax;
    rumble = true;
  }

  if (xnow < xmin) {
    xnow = xmin;
    rumble = true;
  }
  if (ynow < ymin) {
    ynow = ymin;
    rumble = true;
  }
  if (znow < zmin) {
    znow = zmin;
    rumble = true;
  }

  moveArm(xnow, ynow, znow);
  sb1.setPWM(3, 0, hold);
}