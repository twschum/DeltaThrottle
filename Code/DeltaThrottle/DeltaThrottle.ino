/*
 * All original work and credit to zdayton
 *
 * Expanded by twschum to include support for the NewHandle
 * design which integrates 3 buttons and a hat as specified
 * in the Board BOM (specifically, the operation of the hat
 *
 * Switches (3):
 *    C&K Components RS-032G05A3-SM RT
 *    Digi-Key Part Number	CKN10388CT-ND
 * Hat:
 *    E-Switch JS5208
 *    Digi-Key Part Number	EG4561-ND
 */

// HID Joystick state struct, defined in USBAPI.h
//JoyState_t joystickState;

const bool DEBUG = true;  // set to true to debug the raw values

float xZero, yZero, zZero;
float xValue, yValue, zValue;

// Geometry
float handle_rad = 1.75;   // end effector
float base_rad = 1.75;     // base
float pushrod_lng = 3.5;
float pivot_lng = 2.25;

// Configuration
float deadzone = 0.1;  // smaller values will be set to 0
int gain = 150;

// Pin assignments
#define BTN1 4
#define BTN2 5
#define BTN3 3

#define HAT1_UP     9
#define HAT1_LEFT   6
#define HAT1_DOWN   2
#define HAT1_RIGHT  7
#define HAT1_CENTER 8

#define ENABLE  14

// trigonometric constants
float sqrt3 = sqrt(3.0);
float pi = 3.141592653;
float sin120 = sqrt3/2.0;
float cos120 = -0.5;
float tan60 = sqrt3;
float sin30 = 0.5;
float tan30 = 1.0/sqrt3;

void setup()
{
    // analog inputs from the potentiometers
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);

    // digital button inputs with pullup resistor
    pinMode(BTN1, INPUT_PULLUP);
    pinMode(BTN2, INPUT_PULLUP);
    pinMode(BTN3, INPUT_PULLUP);
    pinMode(ENABLE, INPUT_PULLUP);

    pinMode(HAT1_UP,     INPUT_PULLUP);
    pinMode(HAT1_LEFT,   INPUT_PULLUP);
    pinMode(HAT1_DOWN,   INPUT_PULLUP);
    pinMode(HAT1_RIGHT,  INPUT_PULLUP);
    pinMode(HAT1_CENTER, INPUT_PULLUP);

    if (DEBUG) {
        Serial.begin(9600);
    }

    //getForwardKinematic();

    // calculate neutral position
    xZero = 0;
    yZero = 0;
    zZero = zValue;

    //joystickState.xAxis = 0;
    //joystickState.yAxis = 0;
    //joystickState.zAxis = 0;
}

// The delta kinematic math
// You shouldn't need to change anything here.
void getForwardKinematic()
{
    // p1 is the bottom joint, p2 is the top right, p3 is the top left
    int p1ADC = analogRead(A0);
    int p2ADC = analogRead(A1);
    int p3ADC = analogRead(A2);

    // get the angle of each joint in radians
    float theta1 = (p1ADC-60) * 0.003475;
    float theta2 = (p2ADC-60) * 0.003475;
    float theta3 = (p3ADC-60) * 0.003475;

    float t = base_rad-handle_rad;
    float y1 = -(t + pivot_lng*cos(theta1));
    float z1 = pivot_lng*sin(theta1);

    float y2 = (t + pivot_lng*cos(theta2))*sin30;
    float x2 = y2*tan60;
    float z2 = pivot_lng*sin(theta2);

    float y3 = (t + pivot_lng*cos(theta3))*sin30;
    float x3 = -y3*tan60;
    float z3 = pivot_lng*sin(theta3);

    float dnm = (y2-y1)*x3-(y3-y1)*x2;

    float w1 = y1*y1 + z1*z1;
    float w2 = x2*x2 + y2*y2 + z2*z2;
    float w3 = x3*x3 + y3*y3 + z3*z3;

    // x = (a1*z + b1)/dnm
    float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
    float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;

    // y = (a2*z + b2)/dnm;
    float a2 = -(z2-z1)*x3+(z3-z1)*x2;
    float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;

    // a*z^2 + b*z + c = 0
    float a = a1*a1 + a2*a2 + dnm*dnm;
    float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
    float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - pushrod_lng*pushrod_lng);

    // discriminant
    float d = b*b - 4.0*a*c;

    zValue = 0.5*(-b+sqrt(d))/a;
    xValue = (a1*zValue + b1)/dnm;
    yValue = (a2*zValue + b2)/dnm;

    if(false) {
        Serial.print("T1: ");
        Serial.println(theta1);
        Serial.print("T2: ");
        Serial.println(theta2);
        Serial.print("T3: ");
        Serial.println(theta3);
        Serial.print("Z1: ");
        Serial.println(z1);
        Serial.print("Z2: ");
        Serial.println(z2);
        Serial.print("Z3: ");
        Serial.println(z3);
        Serial.print("DNM: ");
        Serial.println(dnm);
    }
}

int getHatState()
{
    int hat = 0;
    hat |= !digitalRead(HAT1_UP) << 0;
    hat |= !digitalRead(HAT1_LEFT) << 1;
    hat |= !digitalRead(HAT1_DOWN) << 2;
    hat |= !digitalRead(HAT1_RIGHT) << 3;
    hat |= !digitalRead(HAT1_CENTER) << 4;

    switch (hat) {
        case 1: return 0;  // b00001  U (hat:0)
        case 2: return 2;  // b00010  L (hat:2)
        case 4: return 4;  // b00100  D (hat:4)
        case 8: return 6;  // b01000  R (hat:6)
        case 16: return 8; // b10000  x (hat:8) (center)
    }
}

char* hatStateName(int hat)
{
    // these values from USBAPI/HID
    switch (hat) {
        case 0: return "UP";
        case 2: return "LEFT";
        case 4: return "DOWN";
        case 6: return "RIGHT";
        case 8: return "CENTER";
    }
}

float applyDeadzone(float value)
{
    if (value < -1*deadzone) {
        value = value + deadzone;
    }
    else if (value > deadzone) {
        value = value - deadzone;
    }
    else {
        value = 0;
    }

    return value;
}

int last = 0;

void loop()
{
    // 10ms = 100 Hz polling
    delay(10);

    // analog read and delta math for xyz
    getForwardKinematic();

    // grab handle button states
    int btn1 = !digitalRead(BTN1);
    int btn2 = !digitalRead(BTN2);
    int btn3 = !digitalRead(BTN3);
    int hat1 = getHatState();

    // update xyz if enabled (0 otherwise)
    if (!digitalRead(ENABLE)) {

        // subtract zero position
        xValue -= xZero;
        yValue -= yZero;
        zValue -= zZero;

        // apply deadzone modifiers
        xValue = applyDeadzone(xValue);
        yValue = applyDeadzone(yValue);
        zValue = applyDeadzone(zValue);

        // apply gain
        xValue *= gain;
        yValue *= gain;
        zValue *= gain;

        // constrain outputs to +- 100
        xValue = constrain(xValue,-100,100);
        yValue = constrain(yValue,-100,100);
        zValue = constrain(zValue,-100,100);
    }
    else {
        // when disabled, force zeros
        xValue = 0;
        yValue = 0;
        zValue = 0;
    }

    // map outputs to 8 bit values, update JoystickSt
    //joystickState.xAxis = map(xValue, -100, 100, 0, 255);
    //joystickState.yAxis = map(yValue, -100, 100, 0, 255);
    //joystickState.zAxis = map(zValue, -100, 100, 0, 255);

    // write button and hat states to JoystickSt
    //joystickState.buttons = btn1 | (btn2<<1) | (btn3<<2);
    //joystickState.hatSw1 = hat1;
    

//    if(btn1) {
//      if (last == 0) {
//        Serial.print('0');
//      }
//      else {
//        Serial.print('-');
//      }
//    }
//    else if (last == 1) {
//      Serial.println("x");
//    }
//    last = btn1;

    if (DEBUG) {
        Serial.print("X: ");
        Serial.println(xValue);
        Serial.print("Raw: ");
        Serial.println( analogRead(A0) );
        Serial.print("Y: ");
        Serial.println(yValue);
        Serial.print("Z: ");
        Serial.println(zValue);
        Serial.print("B1: ");
        Serial.println(btn1);
        Serial.print("B2: ");
        Serial.println(btn2);
        Serial.print("B3: ");
        Serial.println(btn3);
        Serial.print("EN: ");
        Serial.println(!digitalRead(ENABLE));
        Serial.print("H1: ");
        Serial.println(hatStateName(hat1));
    }

    // Send to USB
    //Joystick.setState(&joystickState);

    if (DEBUG) {
        delay(1000);
    }
}
