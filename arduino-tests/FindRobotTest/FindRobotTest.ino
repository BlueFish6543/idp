#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>

#define PI 3.14159265

int status = WL_IDLE_STATUS;
char ssid[] = "galaxy-s8";
char pass[] = "QuarkZero";
int keyIndex = 0; // (needed only for WEP)

unsigned int localPort = 2390;

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
    static const int DISTANCE_THRESHOLD; // to be changed
    static const int NO_IR_SIGNAL_FOUND = -1000; // to be changed
    static const int MOVE_FORWARD_CALIBRATION_CONSTANT = 8543; // milliseconds to traverse half the table = 360 pixels
    static const int TURN_CALIBRATION_CONSTANT = 4800; // milliseconds to make a complete revolution
    static const int NORMAL_MOTOR_SPEED = 200; // to be changed
    static const int TURN_DELAY = 100; // to be changed
    int LEFT_THRESHOLD = 150; // to be changed
    int RIGHT_THRESHOLD = 300; // to be changed
    static const int ADAPTIVE_THRESHOLD_OFFSET = 150; // to be changed
    static const int X_CENTRE = 365; // hardcoded value
    static const int Y_CENTRE = 339; // hardcoded value
    
    State state; // current state of the robot
    Pose pose; // current pose of the robot
    int distanceToObjectInFront; // may need to use an interrupt for this

    int prevLeftSensor = 0;
    int prevRightSensor = 0;

    int servoAngle = 0;
    int coordinateCounter = 0;

    int targetCoordinates[4]; // holds (x, y) coordinates of the targets
    const int numCoordinates = sizeof(targetCoordinates) / sizeof(*targetCoordinates);

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
      return (int) distance;
    }

    bool requiresServicing() {
      // to be implemented
    }

    void lightUpLED() {
      // to be implemented
    }

    void moveRobotToNextLocation() {
      // to be implemented
    }

    void findRobot() {
      Serial.println("Finding robot");
      int x = targetCoordinates[coordinateCounter];
      int y = targetCoordinates[coordinateCounter + 1];
      coordinateCounter += 2;
      int theta = getTheta(x, y);
      int distance = getDistance(x, y);
      
      turnByAngle(theta);
      moveForward(distance);
      
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
      obtainTargetCoordinates();
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
const int Robot::ADAPTIVE_THRESHOLD_OFFSET;
const int Robot::X_CENTRE;
const int Robot::Y_CENTRE;

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
