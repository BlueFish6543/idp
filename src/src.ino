#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

Servo servo;

int leftLineDetectorPin = A2; // to be changed
int rightLineDetectorPin = A1; // to be changed
int servoPin; // to be changed
int IRPhototransistorPin; // to be changed

typedef struct {
  int x;
  int y;
  int theta;
} Pose;

class Robot {
  private:
    enum State {
      START_TO_TUNNEL, // from start to tunnel
      TUNNEL_TO_FINISH, // from tunnel to finish
      SERVICE_TO_TUNNEL, // from service area to tunnel
      TUNNEL_TO_SERVICE, // from tunnel to service area
      SEARCH, // searching for robots
      RETURN_TO_TUNNEL // got a robot and returning to tunnel
    };

    // Constants
    static const int MAX_SERVO_ANGLE; // to be changed
    static const int DISTANCE_THRESHOLD; // to be changed
    static const int NO_IR_SIGNAL_FOUND = -1000; // to be changed
    static const int MOVE_FORWARD_CALIBRATION_CONSTANT; // to be changed
    static const int TURN_CALIBRATION_CONSTANT; // to be changed
    static const int NORMAL_MOTOR_SPEED = 75; // to be changed
    static const int TURN_DELAY = 100; // to be changed
    static const int IR_SCAN_DELAY = 10; // to be changed
    int LEFT_THRESHOLD = 150; // to be changed
    int RIGHT_THRESHOLD = 300; // to be changed
    static const int ADAPTIVE_THRESHOLD_OFFSET = 150; // to be changed
    static const int SERVO_ROTATION_STEP = 5; // to be changed
    static const int IR_SCAN_WINDOW_LENGTH = 30; // to be changed
    static const int IR_PEAK_THRESHOLD; // to be changed

    State state; // current state of the robot
    Pose pose; // current pose of the robot
    int distanceToObjectInFront;

    int prevLeftSensor = 0;
    int prevRightSensor = 0;

    int servoAngle = 0;
    int servoSweepDirection = SERVO_ROTATION_STEP;

    bool allServicingRobotsCollected = false;

    void updatePoseForward() {
      pose.x += MOVE_FORWARD_CALIBRATION_CONSTANT * cos(pose.theta);
      pose.y += MOVE_FORWARD_CALIBRATION_CONSTANT * sin(pose.theta);
    }

    void updatePoseTurn() {
      pose.theta += TURN_CALIBRATION_CONSTANT;
    }

    void goForward() {
      leftMotor->setSpeed(NORMAL_MOTOR_SPEED);
      leftMotor->run(FORWARD);
      rightMotor->setSpeed(NORMAL_MOTOR_SPEED);
      rightMotor->run(FORWARD);
    }

    void moveForward() {
      // moves robot forward by a single step
      // to be implemented
      updatePoseForward();
    }

    void turnLeft() {
      leftMotor->setSpeed(0);
      leftMotor->run(RELEASE);
      rightMotor->setSpeed(NORMAL_MOTOR_SPEED);
      rightMotor->run(FORWARD);
      delay(TURN_DELAY);
    }

    void turnRight() {
      leftMotor->setSpeed(NORMAL_MOTOR_SPEED);
      leftMotor->run(FORWARD);
      rightMotor->setSpeed(0);
      rightMotor->run(RELEASE);
      delay(TURN_DELAY);
    }
    
    void stopMoving() {
      leftMotor->setSpeed(0);
      leftMotor->run(RELEASE);
      rightMotor->setSpeed(0);
      rightMotor->run(RELEASE);
    }

    void turnByAngle(int angle) {
      // turns robot by set angle, angle can be positive or negative
      // to be implemented
      updatePoseTurn();
    }

    bool leftDetectorOnLine() {
      if (!prevLeftSensor) {
        return analogRead(leftLineDetectorPin) > LEFT_THRESHOLD;
      } else {
        return analogRead(leftLineDetectorPin) > LEFT_THRESHOLD;
      }
    }

    bool rightDetectorOnLine() {
      if (!prevRightSensor) {
        return analogRead(rightLineDetectorPin) > RIGHT_THRESHOLD;
      } else {
        return analogRead(rightLineDetectorPin) > RIGHT_THRESHOLD;
      }
    }

    void adaptLeftLineThreshold() {
      int current = analogRead(leftLineDetectorPin);
      LEFT_THRESHOLD = current + ADAPTIVE_THRESHOLD_OFFSET;
    }

    void adaptRightLineThreshold() {
      int current = analogRead(rightLineDetectorPin);
      RIGHT_THRESHOLD = current + ADAPTIVE_THRESHOLD_OFFSET;
    }

    void followLine() {
      /* This function should handle the entire line following process from start to finish. */
      int numIgnores;
      int counter = 0;
      prevLeftSensor = false;
      prevRightSensor = false;

      switch (state) {
        case START_TO_TUNNEL:
          numIgnores = 2;
          // first ignore for the start box, and second ignore for the junction when the tracks merge
          break;
        // to be added on
      }

//      adaptLeftLineThreshold();
//      adaptRightLinethreshold();
      goForward();

      /* The following while loop is meant to handle the junctions by ignoring a set number
       *  of instances where both sensors return high.
       */
      while (counter <= numIgnores) {
        while (!leftDetectorOnLine() || !rightDetectorOnLine()) {          
          if (leftDetectorOnLine()) {
            turnLeft();
            goForward();
            prevLeftSensor = true;
            prevRightSensor = false;
            continue;
          }
          
          if (rightDetectorOnLine()) {
            turnRight();
            goForward();
            prevLeftSensor = false;
            prevRightSensor = true;
            continue;
          }
          
          prevLeftSensor = false;
          prevRightSensor = false;
        }

        if (!prevLeftSensor || !prevRightSensor) {
          prevLeftSensor = true;
          prevRightSensor = true;
          counter++;
        }
      }

      // Note: may need different logic to handle when to stop depending on the state

      stopMoving(); // presumably reached the end of tunnel
    }

    void moveServo() {
      servoAngle = servoAngle + servoSweepDirection;
      servo.write(servoAngle);
    }

    void sweepServo() {
      if (abs(servoAngle) == MAX_SERVO_ANGLE) {
        servoSweepDirection *= -1;
      }
      moveServo();
    }

    bool IRSignalDetected() {
      /* Current method of implementation:
       *  If a robot is present, the IR receiver gives downward peaks.
       *  We observe a window of time, and say that there is a peak if the difference in
       *  value between adjacent readings is greater than IR_PEAK_THRESHOLD.
       *  Note that program will pause while the receiver is monitoring input.
       */
      int prevValue, currValue;
      for (int i = 0; i < IR_SCAN_WINDOW_LENGTH; i++) {
        currValue = analogRead(IRPhototransistorPin);
        if (i == 0) {
          prevValue = currValue;
          continue; 
        }
        if (abs(currValue - prevValue) > IR_PEAK_THRESHOLD) {
          return true;
        }
        prevValue = currValue;
      }
      return false;
    }

    int scanIR() {
      /* Returns the angle relative to the robot's heading if an IR signal is found,
       *  otherwise returns NO_IR_SIGNAL_FOUND.
       */
      bool signalDetected = false;
      for (int i = 0; i < MAX_SERVO_ANGLE / SERVO_ROTATION_STEP; i++) {
        signalDetected = signalDetected || IRSignalDetected();
      }
      // TODO
      
      sweepServo();
      bool signalDetected = IRSignalDetected();
      if (IRSignalDetected()) {
        return servoAngle;
      } else {
        return NO_IR_SIGNAL_FOUND;
      }
    }

    bool requiresServicing() {
      // to be implemented
    }

    void lightUpLED() {
      // to be implemented
    }

    void locateRobot() {
      
    }

    void findRobot() {
      int signalDirection = NO_IR_SIGNAL_FOUND;
      
      while (signalDirection == NO_IR_SIGNAL_FOUND || (!requiresServicing() && !allServicingRobotsCollected) { 
        signalDirection = scanIR();
      }

      // IR receiver should have found robot here
      turnByAngle(servoAngle);
      

      // Target should be in front of robot at this point
      lightUpLED();
      collectRobot();
    }

    void collectRobot() {
      // to be implemented
    }

    void goBackToTunnel() {
      // to be implemented
    }

    /* TODO:
     *  Make LEDs light up
     *  Predetermined route for robot
     *  OpenCV
     *  More robust line tracking algorithm (threshold)
     *  Servo code
     *  IR signal detector
     */
};

void setup() {
  AFMS.begin();
  servo.attach(servoPin);
  Serial.begin(9600);
}

void loop() {
  
}
