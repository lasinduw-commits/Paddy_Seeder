#include <PPMReader.h>
#include <Servo.h>

Servo ESC;                          // Creating an object ESC from class Servo

byte interruptPin = 3;              // Receiver signal pin connected to D3 interrupt pin
byte channelAmount = 6;             // 6 channels
PPMReader ppm(interruptPin, channelAmount);

const int SSR_PIN = 4;              // SSR enable for BLDC Motor power on D4 
const int ESC_PWM_PIN = 5;          // PWM for ESC signal on D5
const int R_EN_PIN = 7;             // Rotary Motors enable on D7
const int R_PWM_PIN = 6;            // PWM for Rotary Motors speed control on D6
const int V_EN_PIN = 10;            // Vibration Motors enable on D10
const int V_PWM_PIN = 11;           // PWM for Vibration Motors on D11
const int BAT_6S_SW = 8;            // 6S Battery switch on D8
const int BAT_3S_SW = 9;            // 3S Battery switch on D9
const int MIN_THROTTLE = 40;        // Minimum "angle" value for BLDC Motors' Speed Control

int THROTTLE = 0;                   // Counter variable for BLDC Motors' ramp up
int THROTTLE_INP = MIN_THROTTLE;    // Input Throttle value from remote
int THROTTLE_LAST = MIN_THROTTLE;   // Remote Input Throttle value in last loop
int MODE = 1000;                    // Initial Working Mode (Off)
int R_SPD = 0;                      // Initial PWM value for Rotary Motor Speed 

bool GO = false;                    // Variable to confirm if the BLDC Motors have come up to minimum speed

// Function to stop everything
void stop()
{
  digitalWrite(SSR_PIN, 0);
  ESC.write(0);
  digitalWrite(R_EN_PIN, 0);
  digitalWrite(V_EN_PIN, 0);
  THROTTLE = 0;
  THROTTLE_INP = MIN_THROTTLE;
  THROTTLE_LAST = MIN_THROTTLE;
  GO = false;
}

// Function to switch on Rotary and Vibration Motors 
void RV()
{
  if (ppm.latestValidChannelValue(4, 1000) > 1950)                          // Channel 4 is the master enable for Rotary & Vibration Motors
  {
    R_SPD = map(ppm.latestValidChannelValue(6, 1000), 1000, 2000, 0, 255);  // Channel 6 for Rotary Motors' Speed
    if ((GO) && (ppm.latestValidChannelValue(2, 1000) > 1950))              // Channel 2 is the Rotary Motors' Enable
    {
      digitalWrite(R_EN_PIN, 1);
      analogWrite(R_PWM_PIN, R_SPD);
    } else
      {
        digitalWrite(R_EN_PIN, 0);
      }
    if ((GO) && (ppm.latestValidChannelValue(3, 1000) > 1950))              // Channel 3 is the Vibration Motors' Enable
    {
      digitalWrite(V_EN_PIN, 1);
      digitalWrite(V_PWM_PIN, 1);
    } else
      {
        digitalWrite(V_EN_PIN, 0);
      }
  } else
    {
      digitalWrite(R_EN_PIN, 0);
      digitalWrite(V_EN_PIN, 0);
    }
}

void setup() 
{

  // Baud Rate
  Serial.begin(115200);       
  
  // Declaring control pins as outputs
  pinMode(SSR_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(V_EN_PIN, OUTPUT);
  pinMode(V_PWM_PIN, OUTPUT); 

  // ESC frequency range (microseconds)
  ESC.attach(ESC_PWM_PIN, 1000, 2000);   

  // Declaring switch signals as sourcing inputs
  pinMode(BAT_6S_SW, INPUT_PULLUP);
  pinMode(BAT_3S_SW, INPUT_PULLUP); 

}

void loop()
{

  // Reading PPM values from receiver
  MODE = ppm.latestValidChannelValue(1, 1000);                                              // Channel 1 for Working Mode (Off, On, Off)
  THROTTLE_INP = map(ppm.latestValidChannelValue(5, 1000), 1000, 2000, MIN_THROTTLE, 180);  // Channel 5 for BLDC Motors Speed

  if ((digitalRead(BAT_6S_SW) == 0) && (digitalRead(BAT_3S_SW) == 0) && (MODE > 1450) && (MODE < 1550))
  {
    if ((THROTTLE_INP <= (MIN_THROTTLE + 1)) && (!GO))
    {
      digitalWrite(SSR_PIN, 1);
      ESC.write(0);
      delay(1500);
      for (THROTTLE = 0; THROTTLE < (THROTTLE_INP + 1); THROTTLE++) 
      {
        MODE = ppm.latestValidChannelValue(1, 1000);
        if ((digitalRead(BAT_6S_SW) == 0) && (digitalRead(BAT_3S_SW) == 0) && (MODE > 1450) && (MODE < 1550))
        {
          // Gradually getting the BLDC Motors to (minimum) speed
          ESC.write(THROTTLE);
          delay(50);
          if (THROTTLE == MIN_THROTTLE)
          {
            GO = true;
          }
        } else
          {
            stop();
            break;
          }
        RV();
        THROTTLE_INP = map(ppm.latestValidChannelValue(5, 1000), 1000, 2000, MIN_THROTTLE, 180);
      }
    } else if ((THROTTLE_INP > THROTTLE_LAST) && (GO))
      {
        for (THROTTLE = THROTTLE_LAST; THROTTLE < (THROTTLE_INP + 1); THROTTLE++) 
        {
          MODE = ppm.latestValidChannelValue(1, 1000);
          if ((digitalRead(BAT_6S_SW) == 0) && (digitalRead(BAT_3S_SW) == 0) && (MODE > 1450) && (MODE < 1550))
          {
            // Gradually getting the BLDC Motors to speed
            ESC.write(THROTTLE);
            delay(50);
          } else
            {
              stop();
              break;
            }
          RV();
          THROTTLE_INP = map(ppm.latestValidChannelValue(5, 1000), 1000, 2000, MIN_THROTTLE, 180);
        }
      } else if (GO)
        {
          ESC.write(THROTTLE_INP);
          RV();
        }
    if ((digitalRead(BAT_6S_SW) == 0) && (digitalRead(BAT_3S_SW) == 0) && (MODE > 1450) && (MODE < 1550))
    {
      THROTTLE_LAST = THROTTLE_INP;
    }
  } else 
    {
      stop();
    }

}