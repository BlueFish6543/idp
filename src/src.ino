int LEFT_SENSOR; // to be replaced
int RIGHT_SENSOR; // to be replaced

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

    void goForward() {
      // to be implemented
    }

    void turnLeft() {
      // to be implemented
    }

    void turnRight() {
      // to be implemented
    }

    void stopMoving() {
      // to be implemented
    }

    void followLine() {
      goForward();

      while (!LEFT_SENSOR && !RIGHT_SENSOR) {
        if (LEFT_SENSOR) {
          turnLeft();
          continue;
        }
        if (RIGHT_SENSOR) {
          turnRight();
          continue;
        }
      }

      /* This logic needs to be improved:
       *  Needs logic to handle the junction correctly.
       *  May need different logic to know when to stop depending on the state.
       */

      stopMoving(); // presumably reached the end of tunnel
    }
};

void setup() {
  Serial.begin(9600);
}

void loop() {
  
}
