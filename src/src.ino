#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

int leftLineDetectorPin = A2; // to be changed
int rightLineDetectorPin = A1; // to be changed

int IR_SENSOR; // to be replaced

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
    static const int NO_IR_SIGNAL_FOUND; // to be changed
    static const int MOVE_FORWARD_CALIBRATION_CONSTANT; // to be changed
    static const int TURN_CALIBRATION_CONSTANT; // to be changed
    static const int NORMAL_MOTOR_SPEED = 75; // to be changed
    static const int TURN_DELAY = 100; // to be changed
    int LEFT_THRESHOLD = 150; // to be changed
    int RIGHT_THRESHOLD = 300; // to be changed
    static const int ADAPTIVE_THRESHOLD_OFFSET = 150; // to be changed

    State state; // current state of the robot
    Pose pose; // current pose of the robot
    int distanceToObjectInFront;

    int prevLeftSensor = 0;
    int prevRightSensor = 0;

    int servoAngle; // to be replaced
    int servoSweepDirection;

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
      // to be implemented
    }

    void sweepServo() {
      if (servoAngle == MAX_SERVO_ANGLE) {
        // change servoSweepDirection
      }
      moveServo();
      // update servoAngle
    }

    int scanIR() {
      /* Returns the angle relative to the robot's heading if an IR signal is found,
       *  otherwise returns NO_IR_SIGNAL_FOUND.
       */
      sweepServo();
      if (IR_SENSOR) {
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

    void findRobot() {
      while (distanceToObjectInFront > DISTANCE_THRESHOLD) { 
        int signalDirection = scanIR();
        if ((signalDirection != NO_IR_SIGNAL_FOUND) && (requiresServicing() || allServicingRobotsCollected)) {
          turnByAngle(servoAngle);
        }
        moveForward();

        /* Might need to confirm that distanceToObjectInFront
         *  is indeed decreasing.
         */
      }

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
  Serial.begin(9600);
}

void loop() {
  
}
