// Simple NeoPixel test.  Lights just a few pixels at a time so a
// 1m strip can safely be powered from Arduino 5V pin.  Arduino
// may nonetheless hiccup when LEDs are first connected and not
// accept code.  So upload code first, unplug USB, connect pixels
// to GND FIRST, then +5V and digital pin 6, then re-plug USB.
// A working strip will show a few pixels moving down the line,
// cycling between red, green and blue.  If you get no response,
// might be connected to wrong end of strip (the end wires, if
// any, are no indication -- look instead for the data direction
// arrows printed on the strip).

#include <Adafruit_NeoPixel.h>

#define PIN 2
#define ECHO 3
#define TRIGGER 4
#define N_LEDS 60

Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_LEDS, PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);

  strip.begin();
  Serial.begin(9600);
}

double d = 0;
double direction = .3;

long triggerDistance = 233;
bool isTriggered = false;

long lastTriggerTime = 0;

long colorA = 0x000000;
long colorB = 0x000000;
long currentColor = 0x000000;

long distance = 0;
long lastRead = 0;

void read() {
  long readDistance = readDistanceInCm();
  if(readDistance < 400) {
    distance = readDistance;
  }
  lastRead = millis();
  Serial.println(distance);
}

void loop() {
  if(millis() - lastRead > 50) {
    read();
  }

  if(distance < triggerDistance) {
    if(isTriggered) {
      lastTriggerTime = millis();
      colorA = currentColor;
      colorB = 0xFF0000;
    }
    isTriggered = false;
  } else {
    if(!isTriggered) {
      lastTriggerTime = millis();
      colorA = currentColor;
      colorB = 0x00FF00;
    }
    isTriggered = true;
  }
  if(millis() - lastTriggerTime > 3000 && isTriggered) {
      colorA = currentColor;
      colorB = 0xFFFFFF;
      lastTriggerTime = millis();
  }

  //SetStripTo(BytesToColor(255, 0, 0));
  currentColor = Lerp(colorA, colorB, getFractionalDeltaTime(lastTriggerTime) * 3.0);
  SetStripTo(currentColor);
}

double getFractionalDeltaTime(long time) {
  long deltaTime = millis() - time;
  return static_cast<double>(deltaTime) / 1000.0;
}

long BytesToColor(long r, long g, long b) {
  if(r > 255) r = 255;
  if(g > 255) g = 255;
  if(b > 255) b = 255;
  return (r << 16) | (g << 8) | b;
}

long Lerp(long color, long target, double a) {
  if(a > 1) a = 1;
  if(a < 0) a = 0;
  double startR = static_cast<double>((color >> 16) & 0xff); // Splits out new color into separate R, G, B
  double startG = static_cast<double>((color >> 8) & 0xff);
  double startB = static_cast<double>(color & 0xff);
  
  double targetR = static_cast<double>((target >> 16) & 0xff); // Splits out new color into separate R, G, B
  double targetG = static_cast<double>((target >> 8) & 0xff);
  double targetB = static_cast<double>(target & 0xff);

  uint16_t newR = (uint16_t)(startR + (targetR - startR) * a);
  uint16_t newG = (uint16_t)(startG + (targetG - startG) * a);
  uint16_t newB = (uint16_t)(startB + (targetB - startB) * a);

  return BytesToColor(newR, newG, newB);
}

void SetStripTo(long color) {
  for(int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

long readDistanceInCm() {
  digitalWrite(TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);

  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  long duration = pulseIn(ECHO, HIGH);

  // convert the time into a distance
  return microsecondsToCentimeters(duration);
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}
