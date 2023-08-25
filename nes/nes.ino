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

#define LATCH 3
#define CLOCK 2
#define DATA 4
#define N_LEDS 30
const int MAX_MESSAGE_LENGTH = 64;

void setup() {
  pinMode(DATA, INPUT);
  pinMode(CLOCK, OUTPUT);
  pinMode(LATCH, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  readButtons();
}

bool B = false;
bool Y = false;
bool Select = false;
bool Start = false;
bool Up = false;
bool Down = false;
bool Left = false;
bool Right = false;
bool A = false;
bool X = false;
bool L = false;
bool R = false;

bool state[12];
bool prevState[12];
char* values[12] = {
  "z",
  "a",
  "Shift_R", // right shift
  "Enter", // return
  "Up", // UP
  "Down", // Down
  "Left", // Left
  "Right", // Right
  "x",
  "s",
  "q",
  "w"
};


void readButtons() {
  digitalWrite(LATCH, HIGH);
  digitalWrite(LATCH, LOW);

  for(int i = 0; i < 12; i++) {
    state[i] = readData();
    if(state[i] != prevState[i]) {
      if(state[i]) {
        Serial.print("p");
        Serial.println(values[i]);
      } else {
        
        Serial.print("r");
        Serial.println(values[i]);
      }
    }
    PulseClock();
  }



  for(int i = 0; i < 12; i++) {
    prevState[i] = state[i];
  }
  digitalWrite(LED_BUILTIN, state[0] ? HIGH : LOW);
  delay(15);
}

bool readData() {
  return digitalRead(DATA) == LOW;
}

void PulseClock() {

  digitalWrite(CLOCK, HIGH);
  digitalWrite(CLOCK, LOW);
}