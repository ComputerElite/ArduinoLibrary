#include <Servo.h>

Servo bigArm;
Servo smallArm;
Servo arm;
Servo base;

#define SERVO_BIG_ARM 2
#define SERVO_SMALL_ARM 3
#define SERVO_ARM 4
#define SERVO_BASE 5

#define JOYSTICK_SWITCH 7

#define VRx A1
#define VRy A0

char message[64];

int angles[] = {
  90,
  90,
  90,
  90
};


int lastAngles[] = {
  90,
  90,
  90,
  90
};

void setup() {
  bigArm.attach(SERVO_BIG_ARM);
  smallArm.attach(SERVO_SMALL_ARM);
  arm.attach(SERVO_ARM);
  base.attach(SERVO_BASE);
  pinMode(JOYSTICK_SWITCH, INPUT_PULLUP);
  Serial.begin(9600); 
}

int analogXPos = 0;
int analogYPos = 0;
double joystickX = 0.0;
double joystickY = 0.0;
double deadzone = 0.1;
long lastLoop = 0;

double robotX = 10.0;
double robotY = 5.0;
double robotZ = 0.0;
double robotRotate = 0.0;
double robotGripper = 0.0;

double joystickMultiplier = 35.0;

bool adjustYAxis = false;

double clamp(double value, double min, double max) {
  if(value < min) return min;
  if(value > max) return max;
  return value;
}
struct RobotArm {
  double length;
  double d1;
  double d2;
  double v1;
  double v2;
};
RobotArm gripper = {
  .length = 0.0,
  .d1 = 0,
  .d2 = 5,
  .v1 = 11,
  .v2 = 127
};
RobotArm smallArmRA = {
  .length = 15.4,
  .d1 = 183.5,
  .d2 = 259.0,
  .v1 = 28,
  .v2 = 99
};
RobotArm bigArmRA = {
  .length = 13.7,
  .d1 = 172.8,
  .d2 = 66.7,
  .v1 = 22,
  .v2 = 127
};
RobotArm baseRA = {
  .length = 0.0,
  .d1 = 80.1,
  .d2 = -38.0,
  .v1 = 11,
  .v2 = 127
};

double baseHeight = 9;


void XYZToAngles(double x, double y, double z, bool setBase = false) {
  /*
  // Debug info
  Serial.print("X ");
  Serial.print(x);
  Serial.print("  Y ");
  Serial.print(y);
  Serial.print("  Z ");
  Serial.print(z);
  Serial.print("  R ");
  Serial.print(robotRotate);
  Serial.print("  G ");
  Serial.println(robotGripper);
  */
  double rotate = atan(z/x);
  double xR = sqrt(x*x + z*z);
  
  double b = sqrt(xR*xR + (y - baseHeight)*(y - baseHeight));
  double c = bigArmRA.length;
  double a = smallArmRA.length;
  
  double alpha = acos((a*a-b*b-c*c)/(-2*b*c));
  double beta = acos((b*b-a*a-c*c)/(-2*a*c));
  
  double gamma2 = PI / 2 + asin((y-baseHeight)/x);
  double alpha2 = PI / 2 - gamma2;
  double epsylon = PI / 2 - alpha2 - alpha;
  double beta2 = PI / 2 - epsylon;
  double delta = PI + (PI - beta2 - beta);
  
  double bigArmAngle = RadiansToDegrees(epsylon)+90;
  double smallArmAngle = RadiansToDegrees(delta);
  double baseAngle = RadiansToDegrees(rotate);;
  
  angles[0] = ConvertAngleToServo(bigArmRA, bigArmAngle);
  angles[1] = ConvertAngleToServo(smallArmRA, smallArmAngle);
  if(setBase) angles[3] = ConvertAngleToServo(baseRA, baseAngle);

  CheckAngles();
}

double lastRobotX = 10.0;
double lastRobotY = 5.0;
double lastRobotZ = 0.0;
bool calculated = false;

void CheckAngles() {
  if(abs(angles[0] - lastAngles[0]) < 10 && abs(angles[1] - lastAngles[1]) < 10 || !calculated) {
    lastAngles[0] = angles[0];
    lastAngles[1] = angles[1];
    lastRobotX = robotX;
    lastRobotY = robotY;
    calculated = true;
  } else {
    angles[0] = lastAngles[0];
    angles[1] = lastAngles[1];
    robotX = lastRobotX;
    robotY = lastRobotY;
  }
}

void ReadAnalogStick() {
  analogXPos = analogRead(VRx);
  analogYPos = analogRead(VRy);

  joystickX = (static_cast<double>(analogXPos) - 512.0) / 512.0;
  if(abs(joystickX) <= deadzone) joystickX = 0.0;
  joystickY = (static_cast<double>(analogYPos) - 512.0) / 512.0;
  if(abs(joystickY) <= deadzone) joystickY = 0.0;

  if(adjustYAxis) {
    robotY += -joystickX * GetDeltaTime() * joystickMultiplier;
    robotGripper += joystickY * GetDeltaTime() * joystickMultiplier;
    robotGripper = clamp(robotGripper, 0, 5);
    angles[2] = ConvertAngleToServo(gripper, robotGripper);
  } else {
    robotX += joystickX * GetDeltaTime() * joystickMultiplier;
    robotRotate += joystickY * GetDeltaTime() * joystickMultiplier * 5.0;
    robotRotate = clamp(robotRotate, baseRA.d2, baseRA.d1);
    angles[3] = static_cast<int>(ConvertAngleToServo(baseRA, robotRotate));
  }

  XYZToAngles(robotX, robotY, robotZ, false);
}

bool lastButton0Pressed = false;
long button0PressTime = 0;
long button0ReleaseTime = 0;

void ReadButtons() {
  if(millis() < 1000) return; // ignore first second
  // Button 0
  bool button0Pressed = digitalRead(JOYSTICK_SWITCH) == HIGH;
  if(button0Pressed != lastButton0Pressed && !button0Pressed) {
    button0ReleaseTime = millis();
    adjustYAxis = !adjustYAxis;
  }
  if(button0Pressed != lastButton0Pressed && button0Pressed) {
    // Button 0 pressed
    button0PressTime = millis();
  }
  lastButton0Pressed = button0Pressed;
}


double RadiansToDegrees(double radians) {
  return radians * 180.0 / PI;
}


double GetDeltaTime() {
  return static_cast<double>(millis() - lastLoop) / 1000.0;
}

double ConvertAngleToServo(struct RobotArm arm, double degrees) {
  double a = (degrees - arm.d1) / (arm.d2 - arm.d1) * (arm.v2 - arm.v1) + arm.v1;
  return clamp(a, arm.v1, arm.v2);
}

void loop() {

  ReadFromSerial();
  bigArm.write(angles[0]);
  smallArm.write(angles[1]);
  arm.write(angles[2]);
  base.write(angles[3]);

  ReadButtons();
  ReadAnalogStick();
  lastLoop = millis();
}

int messagePos = 0;

// Call this function whenever we can read from the serial buffer
void ReadFromSerial() {
  //return;
  while (Serial.available() > 0)
  {

    //Read the next available byte in the serial receive buffer
    char inByte = Serial.read();
    if(inByte != '\n')  {
      message[messagePos] = inByte;
      messagePos++;
    } else {
      // Message done, handle
      message[messagePos] = '\0';
      HandleSerialMsg(message);
      messagePos = 0;
    }
  }
}

void HandleSerialMsg(char data[]) {
  char* d;
  int i = 0;
  char* cmd;
  char* arg1;

  d = strtok(data, ",");
  i = 0;
  while (d != NULL) {
    if (i == 0)
      cmd = d;
    else if (i == 1)
      arg1 = d;
    
    d = strtok(NULL, ",");
    i++;
  }

  angles[strtol(cmd, NULL, 10)] = arg1[0];
}