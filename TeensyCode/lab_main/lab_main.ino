/*
  lab_main  |  CMDP 4630  |  Teensy 4.1

  The core lab sketch. Upload it once, then add hardware as you go.

  BEFORE UPLOADING:  Tools > USB Type > "Serial + MIDI"

  WHAT IT SENDS TO THE COMPUTER
    Buttons on pins 0-7        ->  Notes 60-67   (press = note on, release = note off)
    Joystick X on A0           ->  CC 20         (0-127)
    Joystick Y on A1           ->  CC 21         (0-127)
    Variable resistor on A2    ->  CC 22         (0-127)

  WHAT IT LISTENS FOR (outputs)
    Note 72 from the computer  ->  LED on / off   (the Teensy's own LED, and an LED on pin 10)
    CC 31 from the computer    ->  brightness of the LED on pin 10
    CC 30 from the computer    ->  servo angle on pin 9
    The servo starts in the middle and waits there until a CC 30 arrives.

  WIRING THE OUTPUTS (optional)
    LED:    pin 10 -> 220 ohm resistor -> LED long leg.  LED short leg -> GND.
    Servo:  signal wire (yellow or orange) -> pin 9
            red wire -> VIN (5V from USB)
            brown or black wire -> GND

  WIRING REMINDERS
    Button:  one side to a pin (0-7), the other side to GND. No resistor needed.
    Joystick and sensors:  power from 3.3V, never 5V.
    An analog pin with nothing connected sends drifting junk values. Ignore it
    until you wire that sensor.
*/

#include <Bounce.h>     // comes with Teensy. Cleans up noisy button presses.
#include <PWMServo.h>   // comes with Teensy. Drives the servo.

// Change this 0 to a 1 to also print every message in the Serial Monitor.
#define DEBUG 1

const int MIDI_CHANNEL = 1;

// ---------------------------------------------------------------------------
// BUTTONS
// ---------------------------------------------------------------------------
const int NUM_BUTTONS = 8;
const int FIRST_NOTE = 60;        // pin 0 = note 60, pin 1 = note 61, and so on

// One Bounce object per pin. The 5 is "ignore flicker shorter than 5 ms".
Bounce buttons[NUM_BUTTONS] = {
  Bounce(0, 5), Bounce(1, 5), Bounce(2, 5), Bounce(3, 5),
  Bounce(4, 5), Bounce(5, 5), Bounce(6, 5), Bounce(7, 5)
};

// ---------------------------------------------------------------------------
// ANALOG INPUTS
// ---------------------------------------------------------------------------
const int NUM_ANALOG = 3;
const int analogPins[NUM_ANALOG] = { A0, A1, A2 };
const int ccNumbers[NUM_ANALOG]  = { 20, 21, 22 };

// 0.2 = smooth. Closer to 1.0 = faster but jumpier. Closer to 0 = slower, steadier.
const float SMOOTHING = 0.2;

float smoothed[NUM_ANALOG];       // the running, smoothed reading for each pin
int lastSent[NUM_ANALOG];         // the last value we sent, so we only send changes
elapsedMillis sinceAnalog;        // a stopwatch: how long since we last read the pins

// ---------------------------------------------------------------------------
// OUTPUTS: LED and servo
// ---------------------------------------------------------------------------
const int LED_PIN = 13;           // the LED built into the Teensy
const int EXT_LED_PIN = 10;       // an LED you wire yourself
const int LED_NOTE = 72;          // note on = lit, note off = dark
const int LED_CC = 31;            // 0-127 sets the brightness of the pin 10 LED

const int SERVO_PIN = 9;
const int SERVO_CC = 30;          // 0-127 sets the servo angle

PWMServo servo;

// ---------------------------------------------------------------------------
// HELPERS: send a note or a control change, and print it if DEBUG is on
// ---------------------------------------------------------------------------
void sendNote(int note, bool isOn) {
  if (isOn) {
    usbMIDI.sendNoteOn(note, 127, MIDI_CHANNEL);
  } else {
    usbMIDI.sendNoteOff(note, 0, MIDI_CHANNEL);
  }
#if DEBUG
  Serial.print("Note ");
  Serial.print(note);
  Serial.println(isOn ? " on" : " off");
#endif
}

void sendCC(int cc, int value) {
  usbMIDI.sendControlChange(cc, value, MIDI_CHANNEL);
#if DEBUG
  Serial.print("CC ");
  Serial.print(cc);
  Serial.print(" = ");
  Serial.println(value);
#endif
}

// ---------------------------------------------------------------------------
// These three run automatically when a message arrives FROM the computer.
// ---------------------------------------------------------------------------
void setLED(bool isOn) {
  digitalWrite(LED_PIN, isOn ? HIGH : LOW);
  analogWrite(EXT_LED_PIN, isOn ? 255 : 0);
}

void onNoteOn(byte channel, byte note, byte velocity) {
  if (note == LED_NOTE) setLED(velocity > 0);
}

void onNoteOff(byte channel, byte note, byte velocity) {
  if (note == LED_NOTE) setLED(false);
}

void onControlChange(byte channel, byte control, byte value) {
  if (control == SERVO_CC) {
    servo.write(map(value, 0, 127, 0, 180));        // 0-127 becomes 0-180 degrees
  }
  if (control == LED_CC) {
    analogWrite(EXT_LED_PIN, value * 2);            // 0-127 becomes 0-254 brightness
  }
}

// ---------------------------------------------------------------------------
// Read one analog pin, smooth it, and send a CC only if the value changed.
// ---------------------------------------------------------------------------
void readAnalog(int i) {
  int raw = analogRead(analogPins[i]);              // 0-1023

  // Move the smoothed value a fraction of the way toward the new reading.
  smoothed[i] += SMOOTHING * (raw - smoothed[i]);

  // Each MIDI step (0-127) covers 8 raw steps (0-1023). Only accept a new
  // MIDI value once the reading is clearly past the edge of the current step.
  // This stops a sensor sitting on a boundary from flickering between two values.
  float centerOfLast = lastSent[i] * 8 + 4;
  bool clearlyMoved = fabsf(smoothed[i] - centerOfLast) > 6;

  if (lastSent[i] < 0 || clearlyMoved) {
    int value = constrain((int)(smoothed[i] / 8), 0, 127);
    if (value != lastSent[i]) {
      sendCC(ccNumbers[i], value);
      lastSent[i] = value;
    }
  }
}

// ---------------------------------------------------------------------------
// SETUP: runs once when the Teensy powers up
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // INPUT_PULLUP: the pin reads HIGH on its own and LOW when connected to GND.
  for (int i = 0; i < NUM_BUTTONS; i++) {
    pinMode(i, INPUT_PULLUP);
  }

  pinMode(LED_PIN, OUTPUT);
  pinMode(EXT_LED_PIN, OUTPUT);
  servo.attach(SERVO_PIN);
  servo.write(90);                // start in the middle

  // Start each smoothed value at the current reading.
  for (int i = 0; i < NUM_ANALOG; i++) {
    smoothed[i] = analogRead(analogPins[i]);
    lastSent[i] = -1;             // -1 means "nothing sent yet"
  }

  // Tell the Teensy which functions to run when messages arrive from the computer.
  usbMIDI.setHandleNoteOn(onNoteOn);
  usbMIDI.setHandleNoteOff(onNoteOff);
  usbMIDI.setHandleControlChange(onControlChange);
}

// ---------------------------------------------------------------------------
// LOOP: runs over and over, thousands of times a second
// ---------------------------------------------------------------------------
void loop() {
  // 1. Buttons. Pressing connects the pin to GND, so the signal FALLS.
  for (int i = 0; i < NUM_BUTTONS; i++) {
    buttons[i].update();
    if (buttons[i].fallingEdge()) sendNote(FIRST_NOTE + i, true);    // pressed
    if (buttons[i].risingEdge())  sendNote(FIRST_NOTE + i, false);   // released
  }

  // 2. Analog inputs, every 5 milliseconds.
  if (sinceAnalog >= 5) {
    sinceAnalog = 0;
    for (int i = 0; i < NUM_ANALOG; i++) {
      readAnalog(i);
    }
  }

  // 3. Handle anything the computer sent us (this triggers the functions above).
  while (usbMIDI.read()) {
  }
}
