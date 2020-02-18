int LEFT_SENSOR; // to be replaced
int RIGHT_SENSOR; // to be replaced
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

    State state; // current state of the robot
    Pose pose; // current pose of the robot
    int distanceToObjectInFront;

    int prevLeftSensor;
    int prevRightSensor;

    int servoAngle; // to be replaced
    int servoSweepDirection;

    void updatePoseForward() {
      pose.x += MOVE_FORWARD_CALIBRATION_CONSTANT * cos(pose.theta);
      pose.y += MOVE_FORWARD_CALIBRATION_CONSTANT * sin(pose.theta);
    }

    void updatePoseTurn() {
      pose.theta += TURN_CALIBRATION_CONSTANT;
    }

    void goForward() {
      // goes forward indefinitely
      // to be implemented
    }

    void moveForward() {
      // moves robot forward by a single step
      // to be implemented
      updatePoseForward();
    }

    void turnLeft() {
      // turns robot left by a single step
      // to be implemented
    }

    void turnRight() {
      // turns robot right by a single step
      // to be implemented
    }

    void turnByAngle(int angle) {
      // turns robot by set angle, angle can be positive or negative
      // to be implemented
      updatePoseTurn();
    }

    void stopMoving() {
      // stops robot
      // to be implemented
    }

    void followLine() {
      /* This function should handle the entire line following process from start to finish. */
      int numIgnores;
      int counter = 0;
      prevLeftSensor = 0;
      prevRightSensor = 0;

      switch (state) {
        case START_TO_TUNNEL:
          numIgnores = 2;
          break;
        // to be added on
      }
      
      goForward();

      /* The following while loop is meant to handle the junctions by ignoring a set number
       *  of instances where both sensors return high.
       */
      while (counter <= numIgnores) {
        while (!LEFT_SENSOR || !RIGHT_SENSOR) {
          if (LEFT_SENSOR) {
            turnLeft();
            goForward();
            prevLeftSensor = 1;
            continue;
          }
          
          if (RIGHT_SENSOR) {
            turnRight();
            goForward();
            prevRightSensor = 1;
            continue;
          }
          
          prevLeftSensor = 0;
          prevRightSensor = 0;
        }

        if (!prevLeftSensor || !prevRightSensor) {
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

    void findRobot() {
      while (distanceToObjectInFront > DISTANCE_THRESHOLD) { 
        int signalDirection = scanIR();
        if (signalDirection != NO_IR_SIGNAL_FOUND) {
          turnByAngle(servoAngle);
        }
        moveForward();

        /* Needs additional logic to determine whether robot needs serving or
         *  recharging, and also might need to confirm that distanceToObjectInFront
         *  is indeed decreasing.
         */
      }
    }

    /* TODO:
     *  Make LEDs light up
     *  Predetermined route for robot
     *  OpenCV
     */
};

void setup() {
  Serial.begin(9600);
}

void loop() {
  
}
