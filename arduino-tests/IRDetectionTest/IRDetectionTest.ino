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
int servoPin = 9; // to be changed
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
    static const int MAX_SERVO_ANGLE = 180; // to be changed
    static const int DISTANCE_THRESHOLD = 10; // to be changed
    static const int NO_IR_SIGNAL_FOUND = -1000; // to be changed
    static const int MOVE_FORWARD_CALIBRATION_CONSTANT; // to be changed
    static const int TURN_CALIBRATION_CONSTANT; // to be changed
    static const int NORMAL_MOTOR_SPEED = 100; // to be changed
    static const int TURN_DELAY = 100; // to be changed
    static const int SERVO_SCAN_DELAY = 10; // to be changed
    int LEFT_THRESHOLD = 150; // to be changed
    int RIGHT_THRESHOLD = 300; // to be changed
    static const int ADAPTIVE_THRESHOLD_OFFSET = 150; // to be changed
    static const int SERVO_ROTATION_STEP = 2; // to be changed
    static const int IR_SCAN_WINDOW_LENGTH = 50; // to be changed
    static const int IR_SPIKE_THRESHOLD = 50; // to be changed
    static const int IR_SPIKE_COUNTER_THRESHOLD = 2; // to be changed
    static const int DEFAULT_MOVE_FORWARD_DURATION = 5; // in seconds, to be changed
    
    State state; // current state of t        he robot
    Pose pose; // current pose of the robot
    int distanceToObjectInFront = 1000; // may need to use an interrupt for this

    int prevLeftSensor = 0;
    int prevRightSensor = 0;

    int servoAngle = 0;
    int servoSweepDirection = SERVO_ROTATION_STEP;

    bool allServicingRobotsCollected = false;

    void updatePoseForward(int duration) {
      pose.x += MOVE_FORWARD_CALIBRATION_CONSTANT * duration * cos(pose.theta);
      pose.y += MOVE_FORWARD_CALIBRATION_CONSTANT * duration * sin(pose.theta);
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

    void moveForward(int duration) {
      // duration in seconds
      leftMotor->setSpeed(NORMAL_MOTOR_SPEED);
      leftMotor->run(FORWARD);
      rightMotor->setSpeed(NORMAL_MOTOR_SPEED);
      rightMotor->run(FORWARD);
      
      int counter = 0;
      while ((distanceToObjectInFront > DISTANCE_THRESHOLD) && (counter < duration * 10)) {
        // robot will stop if it detects object in front
        delay(100);
        counter++;
      }
      stopMoving();
      updatePoseForward(duration);
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

    int scanIR() {
      /* Sweeps servo to scan for IR signals. Returns the angle of the IR signal if a
       *  signal is found, otherwise returns NO_IR_SIGNAL_FOUND.
       */
      servoAngle = 0; // reset
      int spikeCounter, currValue;
      bool spikeDetected = false;
      int firstAngle, lastAngle;
      
      while (servoAngle <= MAX_SERVO_ANGLE) {
        servo.write(servoAngle);
        delay(SERVO_SCAN_DELAY);
        spikeCounter = 0;
        
        for (int i = 0; i < IR_SCAN_WINDOW_LENGTH; i++) {
          currValue = analogRead(IRPhototransistorPin);
          if (currValue <= IR_SPIKE_THRESHOLD) {
            spikeCounter++;
          }
          if (spikeCounter >= IR_SPIKE_COUNTER_THRESHOLD) {
            if (!spikeDetected) {
              spikeDetected = true;
              firstAngle = servoAngle;
            }
            lastAngle = servoAngle;
            break;
          }
        }

        servoAngle += SERVO_ROTATION_STEP;
      }

      if (!spikeDetected) {
        return NO_IR_SIGNAL_FOUND;
      } else {
        return ((firstAngle + lastAngle) / 2);
      }
    }

    bool requiresServicing() {
      // to be implemented
      return true;
    }

    void lightUpLED() {
      // to be implemented
    }

    void moveRobotToNextLocation() {
      // to be implemented
    }

    void locateRobot() {
      /* Locates target robot and turns towards it. */
      int signalDirection = scanIR();
      
      while (signalDirection == NO_IR_SIGNAL_FOUND || (!requiresServicing() && !allServicingRobotsCollected)) {
        moveRobotToNextLocation();
        signalDirection = scanIR();
      }

      // Signal has been found
      turnByAngle(servoAngle);
    }

    void findRobot() {
      while (distanceToObjectInFront > DISTANCE_THRESHOLD) {
        locateRobot();
        moveForward(DEFAULT_MOVE_FORWARD_DURATION);
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
     */
     
  public:
    void start() {
      state = SEARCH;
      findRobot();
      // to be continued
    }
};

const int Robot::MAX_SERVO_ANGLE;
const int Robot::DISTANCE_THRESHOLD;
const int Robot::NO_IR_SIGNAL_FOUND;
const int Robot::MOVE_FORWARD_CALIBRATION_CONSTANT;
const int Robot::TURN_CALIBRATION_CONSTANT;
const int Robot::NORMAL_MOTOR_SPEED;
const int Robot::TURN_DELAY;
const int Robot::SERVO_SCAN_DELAY;
const int Robot::ADAPTIVE_THRESHOLD_OFFSET;
const int Robot::SERVO_ROTATION_STEP;
const int Robot::IR_SCAN_WINDOW_LENGTH;
const int Robot::IR_SPIKE_THRESHOLD;
const int Robot::IR_SPIKE_COUNTER_THRESHOLD;
const int Robot::DEFAULT_MOVE_FORWARD_DURATION;

void setup() {
  AFMS.begin();
  servo.attach(servoPin);
  Serial.begin(9600);
}

void loop() {
  Robot(robot);
  robot.start();
}
