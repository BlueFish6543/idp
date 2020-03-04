#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>

int status = WL_IDLE_STATUS;
char ssid[] = "galaxy-s8";
char pass[] = "QuarkZero";
int keyIndex = 0; // (needed only for WEP)

unsigned int localPort = 2390;
IPAddress ip(192, 168, 43, 170);

char packetBuffer[255];

WiFiUDP Udp;

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

void setupWiFi() {
  while (!Serial) {
    ; // wait for serial port to connect, needed for native USB port only
  }

  // Check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    exit(1);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // Attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // Wait for connection:
    delay(3000);
  }
  Serial.println("Connected to WiFi");
  printWiFiStatus();

  Serial.println("\nStarting connection to server...");
  // If you get a connection, report back via serial:
  Udp.begin(localPort);
}

void printWiFiStatus() {
  // Print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // Print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // Print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

int readPacket() {
  while (true) {
    // If there's data available, read a packet
    int packetSize = Udp.parsePacket();
    if (packetSize) {
      Serial.print("Received packet of size ");
      Serial.println(packetSize);
      Serial.print("From ");
      IPAddress remoteIp = Udp.remoteIP();
      Serial.print(remoteIp);
      Serial.print(", port ");
      Serial.println(Udp.remotePort());
  
      // read the packet into packetBufffer
      int len = Udp.read(packetBuffer, 255);
      if (len > 0) {
        packetBuffer[len] = 0;
      }
      int value = atoi(packetBuffer);
      Serial.println("Contents:");
      Serial.println(value);
  
      return value;
    }
  }
}

void acknowledge() {
  char message[] = "Connection established";
  Udp.beginPacket(ip, localPort);
  Udp.write(message);
  Udp.endPacket();
}

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
    static const int MOVE_FORWARD_CALIBRATION_CONSTANT = 6000; // milliseconds to traverse half the table = 360 pixels
    static const int TURN_CALIBRATION_CONSTANT = 4800; // milliseconds to make a complete revolution
    static const int NORMAL_MOTOR_SPEED = 200; // to be changed
    static const int TURN_DELAY = 100; // to be changed
    int LEFT_THRESHOLD = 700; // to be changed
    int RIGHT_THRESHOLD = 700; // to be changed
    static const int ADAPTIVE_THRESHOLD_OFFSET = 150; // to be changed
    static const int X_CENTRE = 378; // hardcoded value
    static const int Y_CENTRE = 340; // hardcoded value
    static const int DISTANCE_OFFSET = 50; // to be changed
    static const int THETA_THRESHOLD = 100; // to be changed
    static const int SENTINEL_VALUE = -1000;
    
    State state; // current state of the robot
    Pose pose; // current pose of the robot
    int distanceToObjectInFront; // may need to use an interrupt for this

    int prevLeftSensor = 0;
    int prevRightSensor = 0;

    int servoAngle = 0;
    int coordinateCounter = 0;

    int targetCoordinates[8]; // holds (x, y) coordinates of the targets
    const int numCoordinates = sizeof(targetCoordinates) / sizeof(*targetCoordinates);

    int actionHistory[4] = {SENTINEL_VALUE, SENTINEL_VALUE, SENTINEL_VALUE, SENTINEL_VALUE}; // stores history of actions

    bool allServicingRobotsCollected = false;

    void obtainTargetCoordinates() {
      for (int i = 0; i < numCoordinates; i++) {
        int value = readPacket();
        targetCoordinates[i] = value;
      }
    }

    void goForward() {
      leftMotor->setSpeed(NORMAL_MOTOR_SPEED);
      rightMotor->setSpeed(NORMAL_MOTOR_SPEED);
      leftMotor->run(FORWARD);
      rightMotor->run(FORWARD);
    }

    void moveForward(int distance) {
      // distance in pixels
      long delayTime = (long) MOVE_FORWARD_CALIBRATION_CONSTANT * (long) distance / 360L;
      goForward();
      delay(delayTime);      
      stopMoving();
    }

    void turnLeft() {
      leftMotor->setSpeed(NORMAL_MOTOR_SPEED);
      leftMotor->run(BACKWARD);
      rightMotor->setSpeed(NORMAL_MOTOR_SPEED);
      rightMotor->run(FORWARD);
      delay(TURN_DELAY);
    }

    void turnRight() {
      leftMotor->setSpeed(NORMAL_MOTOR_SPEED);
      leftMotor->run(FORWARD);
      rightMotor->setSpeed(NORMAL_MOTOR_SPEED);
      rightMotor->run(BACKWARD);
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
      long delayTime = (long) TURN_CALIBRATION_CONSTANT * (long) abs(angle) / 360L;
      if (angle > 0) {
        turnRight();
        delay(delayTime);
        stopMoving();
      } else if (angle < 0) {
        turnLeft();
        delay(delayTime);
        stopMoving();
      }
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
      bool ignoreRightDetector = false;

      switch (state) {
        case START_TO_TUNNEL:
          numIgnores = 2;
          // first ignore for the start box, and second ignore for the junction when the tracks merge
          break;
          
        case TUNNEL_TO_SERVICE:
          numIgnores = 1;
          // ignore for the junction
          break;

        case SERVICE_TO_TUNNEL:
          numIgnores = 1;
          // ignore for the junction
          break;

        case TUNNEL_TO_FINISH:
          numIgnores = 1;
          // ignore for the junction
          break;
      }

//      adaptLeftLineThreshold();
//      adaptRightLinethreshold();
      goForward();

      /* The following while loop is meant to handle the junctions by ignoring a set number
       *  of instances where both sensors return high.
       */
      while (counter <= numIgnores) {
        while (!leftDetectorOnLine() || !rightDetectorOnLine()) {
          ignoreRightDetector = false;

          if (leftDetectorOnLine()) {
            turnLeft();
            goForward();
            prevLeftSensor = true;
            prevRightSensor = false;
            continue;
          }
          
          if (!ignoreRightDetector && rightDetectorOnLine()) {
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
          if (state == TUNNEL_TO_FINISH) {
            ignoreRightDetector = true;
          }
          counter++;
        }
      }

      // Note: may need different logic to handle when to stop depending on the state

      stopMoving(); // presumably reached the end of tunnel
    }

    int getTheta(int x, int y) {
      // Assume x, y are to the front of the tunnel
      int theta = atan2(Y_CENTRE - y, X_CENTRE - x) * 180 / PI;
      Serial.println(theta);
      return theta;
    }

    int getDistance(int x, int y) {
      long distanceSquared = pow((long) X_CENTRE - x, 2) + pow((long) Y_CENTRE - y, 2);
      long distance = sqrt(distanceSquared);
      Serial.println(distance);
      return max((int) distance - DISTANCE_OFFSET, 0);
    }

    bool requiresServicing() {
      // to be implemented
    }

    void lightUpLED() {
      // to be implemented
    }

    void moveRobotToDeadZone(int x, int y, int theta) {
      if (y > Y_CENTRE) {
        turnByAngle(-90);
        actionHistory[0] = -90;
        moveForward(y - Y_CENTRE);
        actionHistory[1] = y - Y_CENTRE;
        turnByAngle(theta + 90);
        actionHistory[2] = theta + 90;
        moveForward(max(0, x - X_CENTRE - DISTANCE_OFFSET));
        actionHistory[3] = max(0, x - X_CENTRE - DISTANCE_OFFSET);
        
      } else if (y < Y_CENTRE) {
        turnByAngle(90);
        actionHistory[0] = 90;
        moveForward(Y_CENTRE - y);
        actionHistory[1] = Y_CENTRE - y;
        turnByAngle(theta - 90);
        actionHistory[2] = theta - 90;
        moveForward(max(0, x - X_CENTRE - DISTANCE_OFFSET));
        actionHistory[3] = max(0, x - X_CENTRE - DISTANCE_OFFSET);
      }
    }

    void findRobot() {
      Serial.println("Finding robot");
      int x = targetCoordinates[coordinateCounter];
      int y = targetCoordinates[coordinateCounter + 1];
      coordinateCounter += 2;
      int theta = getTheta(x, y);

      if (abs(theta) > THETA_THRESHOLD) {
        moveRobotToDeadZone(x, y, theta);
      } else {
        int distance = getDistance(x, y);
        turnByAngle(theta);
        actionHistory[0] = theta;
        moveForward(distance);
        actionHistory[1] = distance;
      }

      /* Need to check whether it needs servicing or recharging */
      
      // Target should be in front of robot at this point
      lightUpLED();
      collectRobot();
      turnByAngle(180); // to be removed 
    }

    void collectRobot() {
      // to be implemented
    }

    void goBackToTunnel() {
      if (actionHistory[3] != SENTINEL_VALUE) {
        moveForward(actionHistory[3]);
        turnByAngle(-1 * actionHistory[2]);
      }
      moveForward(actionHistory[1]);
      turnByAngle(-1 * actionHistory[0]);

      // Reset
      for (int i = 0; i < 3; i++) {
        actionHistory[i] = SENTINEL_VALUE;
      }
    }

    void dropOffRobot() {
      // to be implemented
    }

    /* TODO:
     *  Make LEDs light up
     *  Predetermined route for robot
     *  OpenCV
     */
     
  public:
    void start() {
      acknowledge();
      obtainTargetCoordinates();
      
      state = START_TO_TUNNEL;
      followLine();

      for (int i = 0; i < numCoordinates / 2; i++) {
        if (targetCoordinates[coordinateCounter] == 0) {
          break;
        }
        
        state = SEARCH;
        findRobot();
        
        state = RETURN_TO_TUNNEL;
        goBackToTunnel();

        if (i == numCoordinates / 2) {
          state = TUNNEL_TO_FINISH;
          followLine();
          moveForward(75);
          
        } else {
          state = TUNNEL_TO_SERVICE;
          followLine();
          dropOffRobot();
          
          state = SERVICE_TO_TUNNEL;
          followLine();
        }     
      }
    }
};

const int Robot::MAX_SERVO_ANGLE;
const int Robot::MOVE_FORWARD_CALIBRATION_CONSTANT;
const int Robot::TURN_CALIBRATION_CONSTANT;
const int Robot::NORMAL_MOTOR_SPEED;
const int Robot::TURN_DELAY;
const int Robot::ADAPTIVE_THRESHOLD_OFFSET;
const int Robot::X_CENTRE;
const int Robot::Y_CENTRE;
const int Robot::DISTANCE_OFFSET;
const int Robot::THETA_THRESHOLD;
const int Robot::SENTINEL_VALUE;

void setup() {
  AFMS.begin();
  servo.attach(servoPin);
  Serial.begin(9600);
  setupWiFi();
}

void loop() {
  Robot(robot);
  robot.start();
  exit(0);
}
