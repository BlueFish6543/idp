#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

int leftLineDetectorPin = A2; // to be changed
int rightLineDetectorPin = A1; // to be changed

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
    static const int NORMAL_MOTOR_SPEED = 75; // to be changed
    static const int TURN_DELAY = 100; // to be changed
    static const int LINE_DETECTION_THRESHOLD_LOW = 500; // to be changed
    static const int LINE_DETECTION_THRESHOLD_HIGH = 700; // to be changed

    State state; // current state of the robot

    bool prevLeftSensor = 0;
    bool prevRightSensor = 0;
    
    void goForward() {
      leftMotor->setSpeed(NORMAL_MOTOR_SPEED);
      leftMotor->run(FORWARD);
      rightMotor->setSpeed(NORMAL_MOTOR_SPEED);
      rightMotor->run(FORWARD);
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

    bool leftDetectorOnLine() {
      if (!prevLeftSensor) {
        return analogRead(leftLineDetectorPin) > LINE_DETECTION_THRESHOLD_HIGH;
      } else {
        return analogRead(leftLineDetectorPin) > LINE_DETECTION_THRESHOLD_LOW;
      }
    }

    bool rightDetectorOnLine() {
      if (!prevRightSensor) {
        return analogRead(rightLineDetectorPin) > LINE_DETECTION_THRESHOLD_HIGH;
      } else {
        return analogRead(rightLineDetectorPin) > LINE_DETECTION_THRESHOLD_LOW;
      }
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
      
      goForward();
      Serial.println("Running");

      /* The following while loop is meant to handle the junctions by ignoring a set number
       *  of instances where both sensors return high.
       */
      while (counter <= numIgnores) {
        while (!leftDetectorOnLine() || !rightDetectorOnLine()) {
          Serial.print(analogRead(leftLineDetectorPin));
          Serial.print(" ");
          Serial.print(analogRead(rightLineDetectorPin));
          Serial.print(" ");
          Serial.print(leftDetectorOnLine());
          Serial.print(" ");
          Serial.print(rightDetectorOnLine());
          Serial.print(" ");
          Serial.print(prevLeftSensor);
          Serial.print(" ");
          Serial.print(prevRightSensor);
          Serial.print(" ");
          Serial.println(counter);
          
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
        
        Serial.print(analogRead(leftLineDetectorPin));
        Serial.print(" ");
        Serial.print(analogRead(rightLineDetectorPin));
        Serial.print(" ");
        Serial.print(leftDetectorOnLine());
        Serial.print(" ");
        Serial.print(rightDetectorOnLine());
        Serial.print(" ");
        Serial.print(prevLeftSensor);
        Serial.print(" ");
        Serial.print(prevRightSensor);
        Serial.print(" ");
        Serial.println(counter);

        if (!prevLeftSensor || !prevRightSensor) {
          prevLeftSensor = true;
          prevRightSensor = true;
          counter++;
        }
      }

      // Note: may need different logic to handle when to stop depending on the state

      stopMoving(); // presumably reached the end of tunnel
    }

  public:
    void startLineTrackingTest() {
      state = START_TO_TUNNEL;
      followLine();
    }
};

void setup() {
  AFMS.begin();
  Serial.begin(9600);
}

void loop() {
  delay(2000);
  Robot(robot);
  robot.startLineTrackingTest();
  exit(0);
}
