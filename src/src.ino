#define MAX_SERVO_ANGLE 0 // to be replaced
#define DISTANCE_THRESHOLD 0 // to be replaced

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

    State state; // current state of the robot
    Pose pose; // current pose of the robot
    int distanceToObjectInFront;

    int prevLeftSensor;
    int prevRightSensor;

    int servoAngle; // to be replaced
    int servoSweepDirection;

    // TODO: updatePose()

    void goForward() {
      // to be implemented
    }

    void turnLeft() {
      // to be implemented
    }

    void turnRight() {
      // to be implemented
    }

    void turnByAngle(int angle) {
      // to be implemented
    }

    void stopMoving() {
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

    void scanIR() {
      sweepServo();
      if (IR_SENSOR) {
        turnByAngle(servoAngle);
        goForward();
      }
    }

    void findRobot() {
      while (distanceToObjectInFront > DISTANCE_THRESHOLD) { 
        scanIR();
      }
    }
};

void setup() {
  Serial.begin(9600);
}

void loop() {
  
}
