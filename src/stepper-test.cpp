/*
 * Author: Ben Peart
 * Test application for TMCStepper and FastAccelStepper libraries
 */

#define DEBUG
#define TMCSTEPPER
// #define DEBUG_STEPPER
#define XBOX
// #define XBOX_SERIAL_PLOTTER
// #define DEBUG_XBOX_CONTROLLER

#include "debug.h"
#include <FastAccelStepper.h>
#ifdef TMCSTEPPER
#include <TMCStepper.h>
#endif
#ifdef XBOX
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

void Xbox_setup();
void Xbox_loop();
#endif

#define stepperEnablePin 19 // Enable

#define DRIVER_ADDRESS_LEFT 0b00  // TMC2209 Driver address according to MS1=LOW and MS2=LOW
#define stepperLeftDirPin 25      // Direction
#define stepperLeftStepPin 26     // Step
#define stepperLeftUStepPin1 18   // MS1
#define stepperLeftUStepPin2 05   // MS2
#define DRIVER_ADDRESS_RIGHT 0b01 // TMC2209 Driver address according to MS1=HIGH and MS2=LOW
#define stepperRightDirPin 32     // Direction
#define stepperRightStepPin 33    // Step
#define stepperRightUStepPin1 04  // MS1
#define stepperRightUStepPin2 27  // MS2

#define SERIAL_BAUD_RATE 500000
#define MAX_CURRENT 2000       // in mA; should match capabilities of stepper motor
#define SPEED_INCREMENT 1000   // in steps/s
#define MAX_ACCELERATION 20000 // in steps/s2
#define MAX_THROTTLE 30000
#define MAX_STEERING 25000
#define DEADZONE_RADIUS 1500 // deadzone radius

#define SERIAL2_PORT Serial2 // TMC2208/TMC2224 HardwareSerial port
#define SERIAL2_RX_PIN 16    // Specify Serial2 RX pin as the default has changed
#define SERIAL2_TX_PIN 17    // Specify Serial2 TX pin as the default has changed

#define R_SENSE 0.11f // Match to your driver \
                       // SilentStepStick series use 0.11 \
                       // UltiMachine Einsy and Archim2 boards use 0.2 \
                       // Panucatt BSD2660 uses 0.1 \
                       // Watterott TMC5160 uses 0.075

// Stepper driver
#ifdef TMCSTEPPER
TMC2209Stepper tmcDriverLeft(&SERIAL2_PORT, R_SENSE, DRIVER_ADDRESS_LEFT);   // Hardware Serial
TMC2209Stepper tmcDriverRight(&SERIAL2_PORT, R_SENSE, DRIVER_ADDRESS_RIGHT); // Hardware Serial
#endif

// Stepper controller
FastAccelStepperEngine stepperEngine = FastAccelStepperEngine(); // this should be a singleton
FastAccelStepper *stepperLeft = NULL;
FastAccelStepper *stepperRight = NULL;

void setup()
{
  Serial.begin(230400);

#ifdef TMCSTEPPER
  // use TMC2209 pins MS1 and MS2 to set the correct address for Serial control
  pinMode(stepperLeftUStepPin1, OUTPUT);
  pinMode(stepperLeftUStepPin2, OUTPUT);
  digitalWrite(stepperLeftUStepPin1, LOW);
  digitalWrite(stepperLeftUStepPin2, LOW);
  pinMode(stepperRightUStepPin1, OUTPUT);
  pinMode(stepperRightUStepPin2, OUTPUT);
  digitalWrite(stepperRightUStepPin1, HIGH);
  digitalWrite(stepperRightUStepPin2, LOW);

  // setup a TMC2209 stepper driver
  SERIAL2_PORT.begin(SERIAL_BAUD_RATE, SERIAL_8N1, SERIAL2_RX_PIN, SERIAL2_TX_PIN); // Initialize HW UART drivers with the correct RX/TX pins
  tmcDriverLeft.begin();                                                            // Initiate pins and registeries
  tmcDriverRight.begin();                                                           // Initiate pins and registeries
  tmcDriverLeft.toff(5);                                                            // Enables driver in software (0 = driver off, 1-15 = driver enabled)
  tmcDriverRight.toff(5);                                                           // Enables driver in software (0 = driver off, 1-15 = driver enabled)
  tmcDriverLeft.rms_current(MAX_CURRENT);                                           // Set stepper current to MAX_CURRENT in mA
  tmcDriverRight.rms_current(MAX_CURRENT);                                          // Set stepper current to MAX_CURRENT in mA

  // configure microstepping
  tmcDriverLeft.mstep_reg_select(true);  // Microstep resolution selected by MRES register
  tmcDriverRight.mstep_reg_select(true); // Microstep resolution selected by MRES register
  tmcDriverLeft.microsteps(16);          // Set microsteps per step
  tmcDriverRight.microsteps(16);         // Set microsteps per step

  // enable CoolStep to save up to 75% of energy
  tmcDriverLeft.TCOOLTHRS(0xFFFF);  // Set threshold for CoolStep
  tmcDriverRight.TCOOLTHRS(0xFFFF); // Set threshold for CoolStep
                                    //    tmcDriverLeft.THIGH(0);                  // Set high threshold (not avaialble with TMC2209)
                                    //    tmcDriverRight.THIGH(0);                 // Set high threshold (not avaialble with TMC2209)
  tmcDriverLeft.SGTHRS(10);         // Set StallGuard threshold
  tmcDriverRight.SGTHRS(10);        // Set StallGuard threshold
  tmcDriverLeft.semin(5);           // Minimum current adjustment min = 1, max = 15
  tmcDriverRight.semin(5);          // Minimum current adjustment min = 1, max = 15
  tmcDriverLeft.semax(2);           // Maximum current adjustment min = 0, max = 15, 0-2 recommended
  tmcDriverRight.semax(2);          // Maximum current adjustment min = 0, max = 15, 0-2 recommended
  tmcDriverLeft.sedn(0b01);         // Current down step speed
  tmcDriverRight.sedn(0b01);        // Current down step speed

  // StealthChop2 guarantees that the motor is absolutely quiet in standstill and in slow motion
  // SpreadCycle provides high dynamics (reacting at once to a change of motor velocity) and highest peak velocity at low vibration
  tmcDriverLeft.en_spreadCycle(true);  // Toggle spreadCycle on TMC2208/2209/2224. false = StealthChop (low speed) / true = SpreadCycle (faster speeds)
  tmcDriverRight.en_spreadCycle(true); // Toggle spreadCycle on TMC2208/2209/2224. false = StealthChop (low speed) / true = SpreadCycle (faster speeds)
  tmcDriverLeft.pwm_autoscale(true);   // Needed for stealthChop
  tmcDriverRight.pwm_autoscale(true);  // Needed for stealthChop
#else
  // set micro stepping to 16
  pinMode(stepperLeftUStepPin1, OUTPUT);
  pinMode(stepperLeftUStepPin2, OUTPUT);
  digitalWrite(stepperLeftUStepPin1, HIGH);
  digitalWrite(stepperLeftUStepPin2, HIGH);
  pinMode(stepperRightUStepPin1, OUTPUT);
  pinMode(stepperRightUStepPin2, OUTPUT);
  digitalWrite(stepperRightUStepPin1, HIGH);
  digitalWrite(stepperRightUStepPin2, HIGH);
#endif

  // setup a stepper controller
  stepperEngine.init();
  stepperLeft = stepperEngine.stepperConnectToPin(stepperLeftStepPin); // uses the ESP32 Motor Control Pulse Width Modulator (MCPWM)for the first 3 stepper motors
                                                                       //  stepperLeft = stepperEngine.stepperConnectToPin(stepperLeftStepPin, DRIVER_RMT); // use the ESP32 Remote Control Transceiver (RMT)
  if (stepperLeft)
  {
    DB_PRINTLN("stepperConnectToPin stepperLeftStepPin success");
    stepperLeft->setDirectionPin(stepperLeftDirPin);
    stepperLeft->setEnablePin(stepperEnablePin);
    stepperLeft->setAutoEnable(true);

    // If auto enable/disable need delays, just add (one or both):
    // stepperLeft->setDelayToEnable(50);
    // stepperLeft->setDelayToDisable(1000);

    stepperLeft->setSpeedInHz(0); // the parameter is steps/s !!!
    stepperLeft->setAcceleration(MAX_ACCELERATION);
  }
  stepperRight = stepperEngine.stepperConnectToPin(stepperRightStepPin); // uses the ESP32 Motor Control Pulse Width Modulator (MCPWM)for the first 3 stepper motors
                                                                         //  stepperRight = stepperEngine.stepperConnectToPin(stepperRightStepPin, DRIVER_RMT); // use the ESP32 Remote Control Transceiver (RMT)
  if (stepperRight)
  {
    DB_PRINTLN("stepperConnectToPin stepperRightStepPin success");
    stepperRight->setDirectionPin(stepperRightDirPin);
    stepperRight->setEnablePin(stepperEnablePin);
    stepperRight->setAutoEnable(true);

    // If auto enable/disable need delays, just add (one or both):
    // stepperRight->setDelayToEnable(50);
    // stepperRight->setDelayToDisable(1000);

    stepperRight->setSpeedInHz(0); // the parameter is steps/s !!!
    stepperRight->setAcceleration(MAX_ACCELERATION);
  }

#ifdef XBOX
  Xbox_setup();
#endif // XBOX
}

void loop()
{
  static uint32_t last_time = 0;

#ifdef XBOX
  Xbox_loop();
#endif // XBOX

#ifdef TMCSTEPPER
  // test the serial connections to the TMC2209
  uint8_t result = tmcDriverLeft.test_connection();
  if (result != 0)
  {
    DB_PRINTF("tmcDriverLeft.test_connection failed with error code: %d\n", result);
    return;
  }
  // test the serial connections to the TMC2209
  result = tmcDriverRight.test_connection();
  if (result != 0)
  {
    //    DB_PRINTF("tmcDriverRight.test_connection failed with error code: %d\n", result);
    //    return;
  }
#endif

  if (Serial.available())
  {
    String command = Serial.readString();

    switch (command[0])
    {
#ifdef TMCSTEPPER
    // TMC2209 driver settings
    case 'u':
    {
      int value = command.substring(1).toInt();
      tmcDriverLeft.rms_current(value);
      DB_PRINTF("rms_current_left(0-2000mA): %d\n", tmcDriverLeft.rms_current());
      tmcDriverRight.rms_current(value);
      DB_PRINTF("rms_current_right(0-2000mA): %d\n", tmcDriverRight.rms_current());
      break;
    }

    case 'c':
    {
      if (command.charAt(1) == '+')
      {
        DB_PRINTF("enableCoolStep\n");
        tmcDriverLeft.semin(constrain(1, 1, 15));
        tmcDriverLeft.semax(constrain(0, 0, 15));
        tmcDriverRight.semin(constrain(1, 1, 15));
        tmcDriverRight.semax(constrain(0, 0, 15));
      }
      if (command.charAt(1) == '-')
      {
        DB_PRINTF("disableCoolStep\n");
        tmcDriverLeft.semin(0);
        tmcDriverRight.semin(0);
      }
      break;
    }

    case 's':
      if (command.charAt(1) == '+')
      {
        tmcDriverLeft.en_spreadCycle(true); // true = SpreadCycle (faster speeds)
        DB_PRINTF("SpreadCycle_left (high speed) = %s\n", tmcDriverLeft.en_spreadCycle() ? "true" : "false");
        tmcDriverRight.en_spreadCycle(true); // true = SpreadCycle (faster speeds)
        DB_PRINTF("SpreadCycle_right (high speed) = %s\n", tmcDriverRight.en_spreadCycle() ? "true" : "false");
      }
      if (command.charAt(1) == '-')
      {
        tmcDriverLeft.en_spreadCycle(false); // false = StealthChop (low speed)
        DB_PRINTF("StealthChop_left (low speed) = %s\n", tmcDriverLeft.en_spreadCycle() ? "false" : "true");
        tmcDriverRight.en_spreadCycle(false); // false = StealthChop (low speed)
        DB_PRINTF("StealthChop_right (low speed) = %s\n", tmcDriverRight.en_spreadCycle() ? "false" : "true");
      }
      break;

    case 'f':
      if (command.charAt(1) == 'n')
      {
        DB_PRINTF("freewheel(Normal operation)\n");
        tmcDriverLeft.freewheel(0);
        tmcDriverRight.freewheel(0);
      }
      if (command.charAt(1) == 'f')
      {
        DB_PRINTF("freewheel(Freewheeling)\n");
        tmcDriverLeft.freewheel(1);
        tmcDriverRight.freewheel(1);
      }
      if (command.charAt(1) == 's')
      {
        DB_PRINTF("freewheel(Coil shorted using LS drivers (STRONG_BRAKING))\n");
        tmcDriverLeft.freewheel(2);
        tmcDriverRight.freewheel(2);
      }
      if (command.charAt(1) == 'b')
      {
        DB_PRINTF("freewheel(Coil shorted using HS drivers (BRAKING))\n");
        tmcDriverLeft.freewheel(3);
        tmcDriverRight.freewheel(3);
      }
      break;
#endif
    // stepper controller commands
    case 'F':
    {
      int8_t resultLeft, resultRight;
      resultLeft = stepperLeft->runForward();
      //            digitalWrite(stepperLeftDirPin, HIGH);
      resultRight = stepperRight->runForward();
      DB_PRINTF("runForward Left:%s, Right:%s\n", resultLeft ? "ERR" : "OK", resultRight ? "ERR" : "OK");
      break;
    }

    case 'B':
    {
      int8_t resultLeft, resultRight;
      resultLeft = stepperLeft->runBackward();
      //            digitalWrite(stepperLeftDirPin, LOW);
      resultRight = stepperRight->runBackward();
      DB_PRINTF("runBackward Left:%s, Right:%s\n", resultLeft ? "ERR" : "OK", resultRight ? "ERR" : "OK");
      break;
    }

    case '0':
    {
      stepperLeft->stopMove();
      stepperRight->stopMove();
      DB_PRINTLN("stopMove");
      break;
    }

    case 'G':
    { // EXAMPLE G0 G20000 G-20000 G3200
      int value = command.substring(1).toInt();
      stepperLeft->moveTo(value);
      stepperRight->moveTo(value);
      DB_PRINTF("MoveTo absolute position: %d\n", value);
      break;
    }

    case 'R':
    { // EXAMPLE R0 R20000 R-20000
      int value = command.substring(1).toInt();
      stepperLeft->move(value);
      stepperRight->move(value);
      DB_PRINTF("Move to relative position: %d\n", value);
      break;
    }

    case 'M':
    {
      // Make sure we stop before changing the micro stepping
      stepperLeft->stopMove();
      stepperRight->stopMove();
      while (stepperLeft->getCurrentSpeedInUs())
        ;

      // EXAMPLE M16 (0, 2, 4, 8, 16, 32, 64, 128, 256)
      int value = command.substring(1).toInt();
#ifdef TMCSTEPPER
      tmcDriverLeft.microsteps(value);
      tmcDriverRight.microsteps(value);
      DB_PRINTF("microsteps_left: 1/%d\n", tmcDriverLeft.microsteps());
      DB_PRINTF("microsteps_right: 1/%d\n", tmcDriverRight.microsteps());
#else
      // manually set stepper*UStepPin* pins high/low
      switch (value)
      {
      case 8:
        digitalWrite(stepperLeftUStepPin1, LOW);
        digitalWrite(stepperLeftUStepPin2, LOW);
        digitalWrite(stepperRightUStepPin1, LOW);
        digitalWrite(stepperRightUStepPin2, LOW);
        DB_PRINTF("microsteps: 1/%d\n", value);
        break;

      case 16:
        digitalWrite(stepperLeftUStepPin1, HIGH);
        digitalWrite(stepperLeftUStepPin2, HIGH);
        digitalWrite(stepperRightUStepPin1, HIGH);
        digitalWrite(stepperRightUStepPin2, HIGH);
        DB_PRINTF("microsteps: 1/%d\n", value);
        break;

      case 32:
        digitalWrite(stepperLeftUStepPin1, HIGH);
        digitalWrite(stepperLeftUStepPin2, LOW);
        digitalWrite(stepperRightUStepPin1, HIGH);
        digitalWrite(stepperRightUStepPin2, LOW);
        DB_PRINTF("microsteps: 1/%d\n", value);
        break;

      case 64:
        digitalWrite(stepperLeftUStepPin1, LOW);
        digitalWrite(stepperLeftUStepPin2, LOW);
        digitalWrite(stepperRightUStepPin1, HIGH);
        digitalWrite(stepperRightUStepPin2, HIGH);
        DB_PRINTF("microsteps: 1/%d\n", value);
        break;
      }
#endif // TMCSTEPPER
      break;
    }

    // control motor speed and acceleration
    case '+':
      stepperLeft->setSpeedInHz(stepperLeft->getSpeedInMilliHz() / 1000 + SPEED_INCREMENT);
      stepperRight->setSpeedInHz(stepperRight->getSpeedInMilliHz() / 1000 + SPEED_INCREMENT);
      stepperLeft->applySpeedAcceleration();
      stepperRight->applySpeedAcceleration();
      DB_PRINTF("setSpeedInHz_left: %d\n", stepperLeft->getSpeedInMilliHz() / 1000);
      DB_PRINTF("setSpeedInHz_right: %d\n", stepperRight->getSpeedInMilliHz() / 1000);
      break;

    case '-':
      stepperLeft->setSpeedInHz(stepperLeft->getSpeedInMilliHz() / 1000 - SPEED_INCREMENT);
      stepperRight->setSpeedInHz(stepperRight->getSpeedInMilliHz() / 1000 - SPEED_INCREMENT);
      stepperLeft->applySpeedAcceleration();
      stepperRight->applySpeedAcceleration();
      DB_PRINTF("setSpeedInHz_left: %d\n", stepperLeft->getSpeedInMilliHz() / 1000);
      DB_PRINTF("setSpeedInHz_right: %d\n", stepperRight->getSpeedInMilliHz() / 1000);
      break;

    case 'S':
    { // EXAMPLE S20000
      int value = command.substring(1).toInt();
      stepperLeft->setSpeedInHz(value);
      stepperRight->setSpeedInHz(value);
      stepperLeft->applySpeedAcceleration();
      stepperRight->applySpeedAcceleration();
      DB_PRINTF("setSpeed in steps/s: %d\n", value);
      DB_PRINTF("getMaxSpeed_left in steps/s: %d\n", stepperLeft->getMaxSpeedInHz());
      DB_PRINTF("getMaxSpeed_right in steps/s: %d\n", stepperRight->getMaxSpeedInHz());
      break;
    }

    case 'A':
    { // EXAMPLE A10000
      int value = command.substring(1).toInt();
      stepperLeft->setAcceleration(value);
      stepperRight->setAcceleration(value);
      stepperLeft->applySpeedAcceleration();
      stepperRight->applySpeedAcceleration();
      DB_PRINTF("setAcceleration_left: %d\n", stepperLeft->getAcceleration());
      DB_PRINTF("setAcceleration_right: %d\n", stepperRight->getAcceleration());
      break;
    }
    }
  }

  uint32_t ms = millis();
  if ((ms - last_time) > 2000)
  {
    last_time = ms;

#ifdef DEBUG_STEPPER
    // output stepper controller data
    DB_PRINTF(">getCurrentPosition_left: %d", stepperLeft->getCurrentPosition());
    DB_PRINTF(">getCurrentPosition_right: %d", stepperRight->getCurrentPosition());
    DB_PRINTF(">getCurrentSpeedInHz_left: %d", stepperLeft->getCurrentSpeedInMilliHz() / 1000);
    DB_PRINTF(">getCurrentSpeedInHz_right: %d", stepperRight->getCurrentSpeedInMilliHz() / 1000);
    DB_PRINTF(">getAcceleration_left: %d", stepperLeft->getAcceleration());
    DB_PRINTF(">getAcceleration_right: %d", stepperRight->getAcceleration());
#endif // DEBUG_STEPPER
#ifdef TMCSTEPPER
    // output TMC driver data
    DB_PRINTF("StallGuard_left: %d", tmcDriverLeft.SG_RESULT());
    DB_PRINTF(">StallGuard_right: %d", tmcDriverRight.SG_RESULT());
    DB_PRINTF(">cs_actual_left: %d", tmcDriverLeft.cs2rms(tmcDriverLeft.cs_actual()));
    DB_PRINTF(">cs_actual_right: %d", tmcDriverRight.cs2rms(tmcDriverLeft.cs_actual()));
#endif
    DB_PRINTLN();
  }
}

#ifdef XBOX

// only bind to my xbox controller
// XboxSeriesXControllerESP32_asukiaaa::Core xboxController("9c:aa:1b:f2:66:3d");

// bind to any xbox controller
XboxSeriesXControllerESP32_asukiaaa::Core xboxController;

void Xbox_setup()
{
#ifdef DEBUG_XBOX_CONTROLLER
  DB_PRINTLN("Starting NimBLE Client");
#endif
  xboxController.begin();
}

void Xbox_loop()
{
  // if not connected, reconnect (this is _very_ expensive!), else respond to controller input
  xboxController.onLoop();
  if (xboxController.isConnected())
  {
    if (xboxController.isWaitingForFirstNotification())
    {
#ifdef DEBUG_XBOX_CONTROLLER
      DB_PRINTLN("waiting for first notification");
#endif
    }
    else
    {
#ifdef DEBUG_XBOX_CONTROLLER
      static int first = 1;
      if (first)
      {
        first = 0;
        DB_PRINTLN("Address: " + xboxController.buildDeviceAddressStr());
        DB_PRINT(xboxController.xboxNotif.toString());
      }
#endif
      // normalize the controller input to the range of 0 to 1 then scale
      float car_speed_forward = ((float)xboxController.xboxNotif.trigRT / XboxControllerNotificationParser::maxTrig);
      float car_speed_reverse = ((float)xboxController.xboxNotif.trigLT / XboxControllerNotificationParser::maxTrig);

      // subtract the requested reverse speed from the requested forward speed in case both triggers are requesting different values
      float throttle = (car_speed_forward - car_speed_reverse) * MAX_THROTTLE;

      // the steering is based off the left horizontal joystick
      // convert the range from 0 <-> maxJoy to -1.0 <-> 1.0
      float steering = (float)(xboxController.xboxNotif.joyLHori - (XboxControllerNotificationParser::maxJoy / 2)) / (XboxControllerNotificationParser::maxJoy / 2);

      // We use an exponential damping function on steering to smooth the center band
      if (steering > 0)
        steering = (steering * steering + 0.5 * steering) * MAX_STEERING;
      else
        steering = (-steering * steering + 0.5 * steering) * MAX_STEERING;

      // if within the dead zone, zero it out
      if (steering > -DEADZONE_RADIUS && steering < DEADZONE_RADIUS)
        steering = 0;

      int speed_left_hz = throttle + steering;
      int speed_right_hz = throttle - steering;

#ifdef XBOX_SERIAL_PLOTTER
      DB_PRINT(", throttle:");
      DB_PRINT(throttle);
      DB_PRINT(", steering:");
      DB_PRINT(steering);
      DB_PRINT(", speed_left_hz:");
      DB_PRINT(speed_left_hz);
      DB_PRINT(", speed_right_hz:");
      DB_PRINTLN(speed_right_hz);
#endif // XBOX_SERIAL_PLOTTER

      if (speed_left_hz < 0)
      {
        speed_left_hz = -speed_left_hz;
        stepperLeft->runBackward();
      }
      else if (speed_left_hz > 0)
      {
        stepperLeft->runForward();
      }
      else
      {
        stepperLeft->stopMove();
      }
      if (speed_right_hz < 0)
      {
        speed_right_hz = -speed_right_hz;
        stepperRight->runBackward();
      }
      else if (speed_right_hz > 0)
      {
        stepperRight->runForward();
      }
      else
      {
        stepperRight->stopMove();
      }
      stepperLeft->setSpeedInHz(speed_left_hz);
      stepperRight->setSpeedInHz(speed_right_hz);
      stepperLeft->applySpeedAcceleration();
      stepperRight->applySpeedAcceleration();
    }
  }
  else
  {
#ifdef DEBUG_XBOX_CONTROLLER
    DB_PRINTLN("not connected");
#endif
    // To prevent rebooting when a controller turns on but doesn't connect we could
    // start with an empty address and reboot if failed connections. Once connected,
    // save the address in preferences (with a WebUI to clear it) and use that without
    // the reboot logic on future boots.
    if (xboxController.getCountFailedConnection() > 3)
    {
      ESP.restart();
    }
  }
}
#endif // XBOX
