// PTZ Controller - USB joystick for driving a PTZ camera via a key mapper.
//
// Shows up in Windows as a game controller with X/Y axes and 7 buttons.
// Map the buttons/axes to the camera's keyboard controls with a tool such as JoyToKey.
//
// Board:   Arduino Pro Micro / Leonardo / Micro (ATmega32U4, native USB)
// Library: ArduinoJoystickLibrary by MHeironimus
//          https://github.com/MHeironimus/ArduinoJoystickLibrary
//
// Wiring (all buttons to GND, internal pull-ups, no resistors needed):
//   Joystick +5V -> VCC, GND -> GND, VRx -> A0, VRy -> A1, SW -> 8
//   Zoom +  -> 2      Button 1 -> 4      Button 3 -> 6
//   Zoom -  -> 3      Button 2 -> 5      Button 4 -> 7
//
// Windows button numbers follow btnPins[] order:
//   1 = Zoom +, 2 = Zoom -, 3-6 = Buttons 1-4, 7 = stick click

#include <Joystick.h>

// 7 buttons, no hat switch, X + Y axes only
Joystick_ js(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK,
             7, 0,                       // buttons, hat switches
             true, true, false,          // X, Y, Z
             false, false, false,        // Rx, Ry, Rz
             false, false,               // rudder, throttle
             false, false, false);       // accelerator, brake, steering

const int NUM_BTNS = 7;
const int btnPins[NUM_BTNS] = {2, 3, 4, 5, 6, 7, 8};  // Z+, Z-, B1-B4, stick click
const int DZ = 30;            // centre deadzone (ADC counts)
const int AXIS_HYST = 4;      // ignore axis changes smaller than this
const bool INVERT_X = false;  // flip if pan runs the wrong way once mounted
const bool INVERT_Y = false;  // flip if tilt runs the wrong way once mounted

bool last[NUM_BTNS];

int dz(int v) { return abs(v - 512) < DZ ? 512 : v; }

void setup() {
  for (int i = 0; i < NUM_BTNS; i++) pinMode(btnPins[i], INPUT_PULLUP);
  js.setXAxisRange(0, 1023);
  js.setYAxisRange(0, 1023);
  js.begin(false);            // manual sendState() = fewer USB reports
}

void loop() {
  bool changed = false;

  for (int i = 0; i < NUM_BTNS; i++) {
    bool pressed = digitalRead(btnPins[i]) == LOW;
    if (pressed != last[i]) {
      js.setButton(i, pressed);
      last[i] = pressed;
      changed = true;
    }
  }

  static int lx = -1, ly = -1;
  int rx = analogRead(A0), ry = analogRead(A1);
  if (INVERT_X) rx = 1023 - rx;
  if (INVERT_Y) ry = 1023 - ry;
  int x = dz(rx), y = dz(ry);
  if (abs(x - lx) > AXIS_HYST || abs(y - ly) > AXIS_HYST) {
    js.setXAxis(x);
    js.setYAxis(y);
    lx = x; ly = y;
    changed = true;
  }

  if (changed) js.sendState();
  delay(10);                  // ~10 ms poll doubles as debounce; raise to 15-20 if buttons bounce
}
