#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>

/* Wi-Fi information for the phone's hotspot to which the Arduino
 *  and the computer are connected to.
 */
int status = WL_IDLE_STATUS;
char ssid[] = "galaxy-s8";
char pass[] = "QuarkZero";
int keyIndex = 0; // (needed only for WEP)

/* Default UDP port to send data over and IP address of the computer */
unsigned int localPort = 2390;
IPAddress ip(192, 168, 43, 170);

char packetBuffer[255];

WiFiUDP Udp;

/* Motors for the robot */
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

Servo servo;

/* Pins on the Arduino */
int leftLineDetectorPin = A1;
int rightLineDetectorPin = A2;
int servoPin = 9;
int amberLEDPin = 13;

int amberLEDState = LOW;

/* Connects to the phone's Wi-Fi hotspot. */
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

/* Prints the Wi-Fi status. */
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

/* Reads data sent from the computer. */
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

/* Sends a message from the Arduino to the computer when the Arduino
 *  starts up to let the computer know it's ready to receive data.
 */
void acknowledge() {
  char message[] = "Connection established";
  Udp.beginPacket(ip, localPort);
  Udp.write(message);
  Udp.endPacket();
}

void sendMessage(String message) {
  char charMessage[1024];
  message.toCharArray(charMessage, 1024);
  Udp.beginPacket(ip, localPort);
  Udp.write(charMessage);
  Udp.endPacket();
}

/* The main class for the robot. */
class Robot {
  private:
    /* Stores the current state of the robot. */
    enum State {
      START_TO_TUNNEL, // from start to tunnel
      TUNNEL_TO_FINISH, // from tunnel to finish
      SERVICE_TO_TUNNEL, // from service area to tunnel
      TUNNEL_TO_SERVICE, // from tunnel to service area
      SEARCH, // searching for robots
      RETURN_TO_TUNNEL // got a robot and returning to tunnel
    };

    // Constants
    static const int MOVE_FORWARD_CALIBRATION_CONSTANT = 6000; // milliseconds to traverse half the table = 360 pixels
    static const long TURN_CALIBRATION_CONSTANT = 4635; // milliseconds to make a complete revolution
    static const int NORMAL_MOTOR_SPEED = 200; // maximum possible value is 255
    static const int LINE_FOLLOWING_SPEED = 120; // for line following
    static const int TURN_DELAY = 100; // during line following, each turnLeft or turnRight command executes by this number of milliseconds
    int LEFT_THRESHOLD = 700; // for line following, detector is on line if value is above the threshold
    int RIGHT_THRESHOLD = 700; // for line following, detector is on line if value is above the threshold
    static const int ADAPTIVE_THRESHOLD_OFFSET = 200; // for adaptive thresholding for line following, currently not used
    static const int X_CENTRE = 395; // hardcoded value of x coordinate of the end of the tunnel
    static const int Y_CENTRE = 340; // hardcoded value of y coordinate of the end of the tunnel
    static const int DISTANCE_OFFSET = 82; // robot should stop this number of pixels from the target
    static const int THETA_THRESHOLD = 100; // robot uses a different algorithm to move towards target if theta is above this
    static const int SENTINEL_VALUE = -1000; // sentinel value for out-of-range data
    static const int MOVE_BACKWARD_TIME = 750; // milliseconds for moving backward after dropping off robot
    static const int GO_TO_FINISH_ANGLE = 2; // angle to turn when returning to finish
    static const int GO_TO_FINISH_DISTANCE = 600; // distance in pixels to move forward when returning to finish
    static const int BLINK_INTERVAL = 250; // milliseconds between change in amber LED state
    
    State state; // current state of the robot

    int prevLeftSensor = 0; // whether left line sensor was previously on line (1) or not (0)
    int prevRightSensor = 0; // whether right line sensor was previously on line (1) or not (0)

    int coordinateCounter = 0; // which coordinate (i.e. target) the robot is currently finding

    int targetCoordinates[8]; // holds (x, y) coordinates of the targets, assume a maximum of 4 coordinates
    const int numCoordinates = 8;
    int robotOrientation; // holds orientation of the robot once it exits the tunnel and stops

    unsigned long previousMillis = millis();

    int actionHistory[4] = {SENTINEL_VALUE, SENTINEL_VALUE, SENTINEL_VALUE, SENTINEL_VALUE}; // stores history of actions

    void obtainTargetCoordinates() {
      for (int i = 0; i < numCoordinates; i++) {
        int value = readPacket();
        sendMessage(String("Obtained coordinate: "));
        sendMessage(String(value));
        targetCoordinates[i] = value;
      }
    }

    void obtainRobotOrientation() {
      robotOrientation = readPacket();    
      sendMessage(String("Obtained orientation: "));
      sendMessage(String(robotOrientation));
    }

    void goForward() {
      leftMotor->setSpeed(LINE_FOLLOWING_SPEED);
      rightMotor->setSpeed(LINE_FOLLOWING_SPEED);
      leftMotor->run(FORWARD);
      rightMotor->run(FORWARD);
    }

    void moveBackward(int delayTime) {
      leftMotor->setSpeed(LINE_FOLLOWING_SPEED);
      rightMotor->setSpeed(LINE_FOLLOWING_SPEED);
      leftMotor->run(BACKWARD);
      rightMotor->run(BACKWARD);
      int startTime = millis();
      while ((millis() - startTime) < delayTime) {
        updateLED();
      }
      stopMoving();
    }

    void goForwardQuick() {
      leftMotor->setSpeed(NORMAL_MOTOR_SPEED);
      rightMotor->setSpeed(NORMAL_MOTOR_SPEED);
      leftMotor->run(FORWARD);
      rightMotor->run(FORWARD);
    }

    void moveForward(int distance) {
      // distance in pixels
      long delayTime = (long) MOVE_FORWARD_CALIBRATION_CONSTANT * (long) distance / 360L;
      goForwardQuick();
      int startTime = millis();
      while ((millis() - startTime) < delayTime) {
        updateLED();
      }
      stopMoving();
    }

    void turnLeft() {
      leftMotor->setSpeed(LINE_FOLLOWING_SPEED / 3);
      leftMotor->run(BACKWARD);
      rightMotor->setSpeed(LINE_FOLLOWING_SPEED);
      rightMotor->run(FORWARD);
      delay(TURN_DELAY);
    }

    void turnRight() {
      leftMotor->setSpeed(LINE_FOLLOWING_SPEED);
      leftMotor->run(FORWARD);
      rightMotor->setSpeed(LINE_FOLLOWING_SPEED / 3);
      rightMotor->run(BACKWARD);
      delay(TURN_DELAY);
    }

    void turnLeftSingle() {
      leftMotor->setSpeed(0);
      leftMotor->run(RELEASE);
      rightMotor->setSpeed(LINE_FOLLOWING_SPEED);
      rightMotor->run(FORWARD);
      delay(TURN_DELAY);
    }

    void turnRightSingle() {
      leftMotor->setSpeed(LINE_FOLLOWING_SPEED);
      leftMotor->run(FORWARD);
      rightMotor->setSpeed(0);
      rightMotor->run(RELEASE);
      delay(TURN_DELAY);
    }

    void turnLeftDouble() {
      leftMotor->setSpeed(NORMAL_MOTOR_SPEED);
      leftMotor->run(BACKWARD);
      rightMotor->setSpeed(NORMAL_MOTOR_SPEED);
      rightMotor->run(FORWARD);
      delay(TURN_DELAY);
    }

    void turnRightDouble() {
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
      sendMessage(String("Turning: "));
      sendMessage(String(angle));
      long delayTime = TURN_CALIBRATION_CONSTANT * (long) abs(angle) / 360L;
      if (angle > 0) {
        turnRightDouble();
        int startTime = millis();
        while ((millis() - startTime) < delayTime) {
          updateLED();
        }
        stopMoving();
      } else if (angle < 0) {
        turnLeftDouble();
        int startTime = millis();
        while ((millis() - startTime) < delayTime) {
          updateLED();
        }
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
      int counterStep = 0;
      prevLeftSensor = false;
      prevRightSensor = false;
      bool ignoreLeftDetector = false;
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
      }

      if (state == START_TO_TUNNEL) {
        adaptLeftLineThreshold();
        adaptRightLineThreshold();
      }
      Serial.println(LEFT_THRESHOLD);
      Serial.println(RIGHT_THRESHOLD);
      delay(2000);
      
      goForward();

      /* The following while loop is meant to handle the junctions by ignoring a set number
       *  of instances where both sensors return high.
       */
      while (counter <= numIgnores) {
        updateLED();
        
        if (ignoreLeftDetector && leftDetectorOnLine()) {
          turnRight();
          goForward();
          continue;
        }

        if (ignoreRightDetector && rightDetectorOnLine()) {
          turnLeft();
          goForward();
          continue;
        }
        
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

          ignoreLeftDetector = false;
          ignoreRightDetector = false;
          counterStep += 1;
          updateLED();

          if (!ignoreLeftDetector && leftDetectorOnLine()) {
            if (state == START_TO_TUNNEL || state == SERVICE_TO_TUNNEL) {
              turnLeftSingle();
            } else {
              turnLeft();
            }
            turnLeft();
            goForward();
            prevLeftSensor = true;
            prevRightSensor = false;
            continue;
          }
          
          if (!ignoreRightDetector && rightDetectorOnLine()) {
            if (state == START_TO_TUNNEL || state == SERVICE_TO_TUNNEL) {
              turnRightSingle();
            } else {
              turnRight();
            }
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
          
          if (state == TUNNEL_TO_SERVICE) {
            ignoreLeftDetector = true;
          }

          if (counterStep > 50) {
            counter++;
            counterStep = 0;
            sendMessage(String(counter));
          }
        }
      }
      stopMoving(); // presumably reached the end
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

    void moveRobotToDeadZone(int x, int y) {
      if (y > Y_CENTRE) {
        turnByAngle(-90 - robotOrientation);
        actionHistory[0] = -90;
        moveForward(y - Y_CENTRE);
        actionHistory[1] = y - Y_CENTRE;
        delay(500);
        turnByAngle(-90);
        actionHistory[2] = -90;
        moveForward(max(0, x - X_CENTRE - DISTANCE_OFFSET));
        actionHistory[3] = max(0, x - X_CENTRE - DISTANCE_OFFSET);
        
      } else if (y < Y_CENTRE) {
        turnByAngle(90 - robotOrientation);
        actionHistory[0] = 90;
        moveForward(Y_CENTRE - y);
        actionHistory[1] = Y_CENTRE - y;
        delay(500);
        turnByAngle(90);
        actionHistory[2] = 90;
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
      sendMessage(String(robotOrientation));
      sendMessage(String(theta));
      sendMessage(String(theta - robotOrientation));

      if (abs(theta) > THETA_THRESHOLD) {
        moveRobotToDeadZone(x, y);
      } else {
        int distance = getDistance(x, y);
        sendMessage(String(distance));
        turnByAngle(theta - robotOrientation);
        actionHistory[0] = theta;
        moveForward(distance);
        actionHistory[1] = distance;
      }

      /* Need to check whether it needs servicing or recharging */
      
      // Target should be in front of robot at this point
      digitalWrite(amberLEDPin, HIGH);
      openSweeper();
      delay(1500);
      collectRobot();
      delay(1500);
    }

    void openSweeper() {
      int pos;
      for (pos = 90; pos >= 20; pos -= 1) { // goes from 0 degrees to 90 degrees
        // in steps of 1 degree
        servo.write(pos);
        delay(5);
      }
    }

    void closeSweeper() {
      int pos;
      for (pos = 20; pos <= 90; pos += 1) { // goes from 0 degrees to 90 degrees
        // in steps of 1 degree
        servo.write(pos);
        delay(2);
      }
    }

    void collectRobot() {
      turnByAngle(193);
      closeSweeper();
      delay(500);
    }

    void updateLED() {
      unsigned long currentMillis = millis();
      if ((currentMillis - previousMillis) >= BLINK_INTERVAL) {
        previousMillis = currentMillis;
        
        if (amberLEDState == LOW) {
          amberLEDState = HIGH;
        } else {
          amberLEDState = LOW;
        }
        digitalWrite(amberLEDPin, amberLEDState);
      }
    }

    void goBackToTunnel() {
      if (actionHistory[3] != SENTINEL_VALUE) {
        moveForward(actionHistory[3]);
        delay(500);
        turnByAngle(-1 * actionHistory[2]);
      }
      moveForward(actionHistory[1]);
      delay(500);
      turnByAngle(-1 * actionHistory[0]);

      // Reset
      for (int i = 0; i < 4; i++) {
        actionHistory[i] = SENTINEL_VALUE;
      }
    }

    void dropOffRobot() {
      turnByAngle(190);
      delay(1000);
      openSweeper();
      moveBackward(MOVE_BACKWARD_TIME);
      moveForward(50);
      delay(1000);
      turnByAngle(-190);
      moveBackward(MOVE_BACKWARD_TIME);
      closeSweeper();
      delay(500);
      turnByAngle(190);
      delay(100);
    }

    void goToFinish() {
      turnByAngle(GO_TO_FINISH_ANGLE);
      moveForward(GO_TO_FINISH_DISTANCE);
      stopMoving();
    }
     
  public:
    void start() {
      acknowledge();
      obtainTargetCoordinates();
      
      state = START_TO_TUNNEL;
      followLine();
      delay(1000);

      acknowledge();
      obtainRobotOrientation();

      for (int i = 0; i < numCoordinates / 2; i++) {
        state = SEARCH;
        findRobot();
        
        state = RETURN_TO_TUNNEL;
        goBackToTunnel();
        delay(1000);

        state = TUNNEL_TO_SERVICE;
        followLine();    
        dropOffRobot();

        if ((i == (numCoordinates / 2) - 1) || (targetCoordinates[coordinateCounter] == 0)) {
          state = TUNNEL_TO_FINISH;
          goToFinish();
          return;
          
        } else {
          state = SERVICE_TO_TUNNEL;
          followLine();
          acknowledge();
          obtainRobotOrientation();
        }
      }
    }
};

const int Robot::MOVE_FORWARD_CALIBRATION_CONSTANT;
const long Robot::TURN_CALIBRATION_CONSTANT;
const int Robot::NORMAL_MOTOR_SPEED;
const int Robot::TURN_DELAY;
const int Robot::ADAPTIVE_THRESHOLD_OFFSET;
const int Robot::X_CENTRE;
const int Robot::Y_CENTRE;
const int Robot::DISTANCE_OFFSET;
const int Robot::THETA_THRESHOLD;
const int Robot::SENTINEL_VALUE;
const int Robot::MOVE_BACKWARD_TIME;
const int Robot::GO_TO_FINISH_ANGLE;
const int Robot::GO_TO_FINISH_DISTANCE;

void setup() {
  AFMS.begin();
  servo.attach(servoPin);
  pinMode(amberLEDPin, OUTPUT);
  Serial.begin(9600);
  setupWiFi();
}

void loop() {
  servo.write(90);
  Robot(robot);
  robot.start();
  exit(0);
}
