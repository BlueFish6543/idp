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

/* Buffer to hold received messages */
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

/* Sends messages from the Arduino to the computer for debugging purposes. */
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
    static const long TURN_CALIBRATION_CONSTANT = 4635; // milliseconds to make a complete revolution of 360 degrees
    static const int NORMAL_MOTOR_SPEED = 200; // maximum possible value is 255
    static const int LINE_FOLLOWING_SPEED = 175; // for line following
    static const int TURN_DELAY = 100; // during line following, each turnLeft or turnRight command executes by this number of milliseconds
    int LEFT_THRESHOLD = 700; // for line following, detector is on line if value is above the threshold, will be overwritten by adaptive thresholding
    int RIGHT_THRESHOLD = 700; // for line following, detector is on line if value is above the threshold, will be overwritten by adaptive thresholding
    static const int ADAPTIVE_THRESHOLD_OFFSET = 200; // for adaptive thresholding for line following
    static const int X_CENTRE = 389; // hardcoded value of x coordinate of the end of the tunnel
    static const int Y_CENTRE = 340; // hardcoded value of y coordinate of the end of the tunnel
    static const int DISTANCE_OFFSET = 82; // robot should stop this number of pixels from the target
    static const int THETA_THRESHOLD = 100; // robot uses a different algorithm to move towards target if theta is above this
    static const int SENTINEL_VALUE = -1000; // sentinel value for out-of-range data
    static const int MOVE_BACKWARD_TIME = 750; // milliseconds for moving backward after dropping off robot
    static const int GO_TO_FINISH_ANGLE = 4; // angle to turn when returning to finish
    static const int GO_TO_FINISH_DISTANCE = 600; // distance in pixels to move forward when returning to finish
    
    State state; // current state of the robot

    int prevLeftSensor = 0; // whether left line sensor was previously on line (1) or not (0)
    int prevRightSensor = 0; // whether right line sensor was previously on line (1) or not (0)

    int coordinateCounter = 0; // which coordinate (i.e. target) the robot is currently finding

    int targetCoordinates[8]; // holds (x, y) coordinates of the targets, assume a maximum of 4 coordinates
    const int numCoordinates = 8;
    int robotOrientation; // holds orientation of the robot once it exits the tunnel and stops

    int actionHistory[4] = {SENTINEL_VALUE, SENTINEL_VALUE, SENTINEL_VALUE, SENTINEL_VALUE}; // stores history of actions

    /* Obtains target coordinates from the remote computer. */
    void obtainTargetCoordinates() {
      for (int i = 0; i < numCoordinates; i++) {
        int value = readPacket();
        sendMessage(String("Obtained coordinate: "));
        sendMessage(String(value));
        targetCoordinates[i] = value;
      }
    }

    /* Obtains orientation of the robot from the remote computer once the robot has exited the tunnel. */
    void obtainRobotOrientation() {
      robotOrientation = readPacket();
      sendMessage(String("Obtained orientation: "));
      sendMessage(String(robotOrientation));
    }

    /* Goes forward indefinitely.*/
    void goForward() {
      leftMotor->setSpeed(LINE_FOLLOWING_SPEED);
      rightMotor->setSpeed(LINE_FOLLOWING_SPEED);
      leftMotor->run(FORWARD);
      rightMotor->run(FORWARD);
    }

    /* Moves backward for delayTime milliseconds. Used when dropping off robots. */
    void moveBackward(int delayTime) {
      leftMotor->setSpeed(LINE_FOLLOWING_SPEED);
      rightMotor->setSpeed(LINE_FOLLOWING_SPEED);
      leftMotor->run(BACKWARD);
      rightMotor->run(BACKWARD);
      delay(delayTime);
      stopMoving();
    }

    /* Goes forward indefinitely, at a quicker speed. */
    void goForwardQuick() {
      leftMotor->setSpeed(NORMAL_MOTOR_SPEED);
      rightMotor->setSpeed(NORMAL_MOTOR_SPEED);
      leftMotor->run(FORWARD);
      rightMotor->run(FORWARD);
    }

    /* Moves forward for a set amount of distance. */
    void moveForward(int distance) {
      // distance in pixels
      long delayTime = (long) MOVE_FORWARD_CALIBRATION_CONSTANT * (long) distance / 360L;
      goForwardQuick();
      delay(delayTime);      
      stopMoving();
    }

    /* Turns left by a single 'step'. Left wheel is set to rotate backwards at one third the speed
     *  of the right wheel rotating forwards.
     */
    void turnLeft() {
      leftMotor->setSpeed(LINE_FOLLOWING_SPEED / 3);
      leftMotor->run(BACKWARD);
      rightMotor->setSpeed(LINE_FOLLOWING_SPEED);
      rightMotor->run(FORWARD);
      delay(TURN_DELAY);
    }

    /* Turns right by a single 'step'. Right wheel is set to rotate backwards at one third the speed
     *  of the left wheel rotating forwards.
     */
    void turnRight() {
      leftMotor->setSpeed(LINE_FOLLOWING_SPEED);
      leftMotor->run(FORWARD);
      rightMotor->setSpeed(LINE_FOLLOWING_SPEED / 3);
      rightMotor->run(BACKWARD);
      delay(TURN_DELAY);
    }

    /* Same as turnLeft(), but left wheel does not rotate backwards. */
    void turnLeftSingle() {
      leftMotor->setSpeed(0);
      leftMotor->run(RELEASE);
      rightMotor->setSpeed(LINE_FOLLOWING_SPEED);
      rightMotor->run(FORWARD);
      delay(TURN_DELAY);
    }

    /* Same as turnRight(), but right wheel does not rotate backwards. */
    void turnRightSingle() {
      leftMotor->setSpeed(LINE_FOLLOWING_SPEED);
      leftMotor->run(FORWARD);
      rightMotor->setSpeed(0);
      rightMotor->run(RELEASE);
      delay(TURN_DELAY);
    }

    /* Similar to turnLeft(), but both wheels rotate in the appropriate directions 
     *  at the same speed and at a quicker speed.
     */
    void turnLeftDouble() {
      leftMotor->setSpeed(NORMAL_MOTOR_SPEED);
      leftMotor->run(BACKWARD);
      rightMotor->setSpeed(NORMAL_MOTOR_SPEED);
      rightMotor->run(FORWARD);
      delay(TURN_DELAY);
    }

    /* Similar to turnRight(), but both wheels rotate in the appropriate directions
     *  at the same speed and at a quicker speed.
     */
    void turnRightDouble() {
      leftMotor->setSpeed(NORMAL_MOTOR_SPEED);
      leftMotor->run(FORWARD);
      rightMotor->setSpeed(NORMAL_MOTOR_SPEED);
      rightMotor->run(BACKWARD);
      delay(TURN_DELAY);
    }
    
    /* Stops the robot. */
    void stopMoving() {
      leftMotor->setSpeed(0);
      leftMotor->run(RELEASE);
      rightMotor->setSpeed(0);
      rightMotor->run(RELEASE);
    }

    /* Turns robot by a set angle. Positive angle is clockwise, negative is anticlockwise. */
    void turnByAngle(int angle) {
      sendMessage(String("Turning: "));
      sendMessage(String(angle));
      long delayTime = TURN_CALIBRATION_CONSTANT * (long) abs(angle) / 360L;
      if (angle > 0) {
        turnRightDouble();
        delay(delayTime);
        stopMoving();
      } else if (angle < 0) {
        turnLeftDouble();
        delay(delayTime);
        stopMoving();
      }
    }

    /* Returns true if left line detector is detecting high, i.e. on the line, and
     *  false otherwise.
     */
    bool leftDetectorOnLine() {
      if (!prevLeftSensor) {
        return analogRead(leftLineDetectorPin) > LEFT_THRESHOLD;
      } else {
        return analogRead(leftLineDetectorPin) > LEFT_THRESHOLD;
      }
    }

    /* Returns true if right line detector is detecting high, i.e. on the line, and
     *  false otherwise.
     */
    bool rightDetectorOnLine() {
      if (!prevRightSensor) {
        return analogRead(rightLineDetectorPin) > RIGHT_THRESHOLD;
      } else {
        return analogRead(rightLineDetectorPin) > RIGHT_THRESHOLD;
      }
    }

    /* Sets the left line detector threshold to be ADAPTIVE_THRESHOLD_OFFSET higher
     *  than the current read value.
     */
    void adaptLeftLineThreshold() {
      int current = analogRead(leftLineDetectorPin);
      LEFT_THRESHOLD = current + ADAPTIVE_THRESHOLD_OFFSET;
    }

    /* Sets the right line detector threshold to be ADAPTIVE_THRESHOLD_OFFSET higher
     *  than the current read value.
     */
    void adaptRightLineThreshold() {
      int current = analogRead(rightLineDetectorPin);
      RIGHT_THRESHOLD = current + ADAPTIVE_THRESHOLD_OFFSET;
    }

    void followLine() {
      /* This function should handle the entire line following process from start to finish. */
      int numIgnores;
      int counter = 0;
      int counterStep = 50;
      prevLeftSensor = false;
      prevRightSensor = false;
      bool ignoreLeftDetector = false;
      bool ignoreRightDetector = false;

      /* The robot will ignore numIgnores instances when both line detectors are on the line. */
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
        // adapt line detector thresholds at the start
        adaptLeftLineThreshold();
        adaptRightLineThreshold();
      }
      Serial.println(LEFT_THRESHOLD);
      Serial.println(RIGHT_THRESHOLD);
      delay(2000);
      
      goForward();

      /* The following while loop is meant to handle the junctions by ignoring a set number
       *  of instances where both sensors return high (i.e. on the line). The robot will break
       *  out of this while loop only when it has reached the end of the line following sequence.
       */
      while (counter <= numIgnores) {
        if (ignoreLeftDetector && leftDetectorOnLine()) {
          turnRight();
          goForwardQuick();
          continue;
        }

        if (ignoreRightDetector && rightDetectorOnLine()) {
          turnLeftSingle();
          goForward();
          continue;
        }

        /* The robot will break out of this inner while loop when both detectors return
         *  high (i.e. on the line). Note that different turning speeds are used depending
         *  on which state the robot is in; this was determined experimentally.
         */
        while (!leftDetectorOnLine() || !rightDetectorOnLine()) {
//          Printing for debugging purposes
//          Serial.print(analogRead(leftLineDetectorPin));
//          Serial.print(" ");
//          Serial.print(analogRead(rightLineDetectorPin));
//          Serial.print(" ");
//          Serial.print(leftDetectorOnLine());
//          Serial.print(" ");
//          Serial.print(rightDetectorOnLine());
//          Serial.print(" ");
//          Serial.print(prevLeftSensor);
//          Serial.print(" ");
//          Serial.print(prevRightSensor);
//          Serial.print(" ");
//          Serial.println(counter);

          ignoreLeftDetector = false;
          ignoreRightDetector = false;
          counterStep += 1;

          if (!ignoreLeftDetector && leftDetectorOnLine()) {
            if (state == START_TO_TUNNEL) {
              turnLeftSingle();
            } else {
              turnLeft();
            }
            goForwardQuick();
            prevLeftSensor = true;
            prevRightSensor = false;
            continue;
          }
          
          if (!ignoreRightDetector && rightDetectorOnLine()) {
            if (state == START_TO_TUNNEL) {
              turnRightSingle();
            } else {
              turnRight();
            }
            goForwardQuick();
            prevLeftSensor = false;
            prevRightSensor = true;
            continue;
          }
          
          prevLeftSensor = false;
          prevRightSensor = false;
        }

//        Printing for debugging purposes
//        Serial.print(analogRead(leftLineDetectorPin));
//        Serial.print(" ");
//        Serial.print(analogRead(rightLineDetectorPin));
//        Serial.print(" ");
//        Serial.print(leftDetectorOnLine());
//        Serial.print(" ");
//        Serial.print(rightDetectorOnLine());
//        Serial.print(" ");
//        Serial.print(prevLeftSensor);
//        Serial.print(" ");
//        Serial.print(prevRightSensor);
//        Serial.print(" ");
//        Serial.println(counter);

        if (!prevLeftSensor || !prevRightSensor) {
          prevLeftSensor = true;
          prevRightSensor = true;
          
          // Ignore the appropriate sensors for better junction handling
          if (state == TUNNEL_TO_SERVICE) {
            ignoreLeftDetector = true;
          } else if (state == SERVICE_TO_TUNNEL) {
            ignoreRightDetector = true;
          }

          if (counterStep > 50) {
            // Increment the ignore counter if a certain time interval has passed since the previous increment
            counter++;
            counterStep = 0;
            sendMessage(String(counter));
          }
        } // inner while
      } // outer while
      stopMoving(); // presumably reached the end
    }

    /* Returns the angle that the robot needs to turn by in order to reach the target
     *  located at pixel coordinates (x, y).
     */
    int getTheta(int x, int y) {
      // Assume x, y are to the front of the tunnel
      int theta = atan2(Y_CENTRE - y, X_CENTRE - x) * 180 / PI;
      Serial.println(theta);
      return theta;
    }

    /* Returns the distance in pixels that the robot needs to move forward by in order
     *  to reach the target located at pixel coordinates (x, y).
     */
    int getDistance(int x, int y) {
      long distanceSquared = pow((long) X_CENTRE - x, 2) + pow((long) Y_CENTRE - y, 2);
      long distance = sqrt(distanceSquared);
      Serial.println(distance);
      if (distance > 325) {
        distance += 5;
      }
      return max((int) distance - DISTANCE_OFFSET, 0);
    }

    /* Moves AGV to pick up a target located in the dead zone. The robot will turn by
     *  90 degrees twice. Note that the orientation of the AGV is taken into account
     *  at the first turning, because the AGV may not be facing straight after it exits
     *  the tunnel. Each action is stored into the actionHistory array.
     */
    void moveRobotToDeadZone(int x, int y) {
      if (y > Y_CENTRE) {
        turnByAngle(-90 - robotOrientation);
        actionHistory[0] = -90;
        moveForward(y - Y_CENTRE + 15);
        actionHistory[1] = y - Y_CENTRE + 15;
        delay(200);
        turnByAngle(-90);
        actionHistory[2] = -90;
        moveForward(max(0, x - X_CENTRE - DISTANCE_OFFSET - 20));
        actionHistory[3] = max(0, x - X_CENTRE - DISTANCE_OFFSET - 20);
        
      } else if (y < Y_CENTRE) {
        turnByAngle(90 - robotOrientation);
        actionHistory[0] = 90;
        moveForward(Y_CENTRE - y + 15);
        actionHistory[1] = Y_CENTRE - y + 15;
        delay(200);
        turnByAngle(90);
        actionHistory[2] = 90;
        moveForward(max(0, x - X_CENTRE - DISTANCE_OFFSET - 20));
        actionHistory[3] = max(0, x - X_CENTRE - DISTANCE_OFFSET - 20);
      }
    }

    /* Finds and moves towards target robot, and picks it up. */
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
        // Target is considered to be in the dead zone if angle > THETA_THRESHOLD
        moveRobotToDeadZone(x, y);
      } else {
        int distance = getDistance(x, y);
        sendMessage(String(distance));
        turnByAngle(theta - robotOrientation); // compensate for possible skewness of AGV
        actionHistory[0] = theta; // record action history
        moveForward(distance);
        actionHistory[1] = distance; // record action history
      }
      
      // Target should be in front of robot at this point
      openSweeper();
      delay(500);
      collectRobot();
      delay(500);
    }

    /* Opens the sweeper mechanism. */
    void openSweeper() {
      int pos;
      for (pos = 90; pos >= 20; pos -= 1) {
        // in steps of 1 degree
        servo.write(pos);
        delay(5);
      }
    }

    /* Closes the sweeper mechanism. */
    void closeSweeper() {
      int pos;
      for (pos = 20; pos <= 90; pos += 1) {
        // in steps of 1 degree
        servo.write(pos);
        delay(2);
      }
    }

    /* Turns and sweeps target robot onto the AGV. */
    void collectRobot() {
      turnByAngle(194); // 194 degrees was an observed value that works, should theoretically by 180 degrees
      closeSweeper();
    }

    /* Returns to the tunnel after picking up a target robot. */
    void goBackToTunnel() {
      if (actionHistory[3] != SENTINEL_VALUE) {
        // Target was in the dead zone and therefore we had two turns
        moveForward(actionHistory[3]);
        delay(200);
        turnByAngle(-1 * actionHistory[2]);
      }
      moveForward(actionHistory[1]);
      delay(200);
      turnByAngle(-1 * actionHistory[0]);
      
      // Reset actionHistory
      for (int i = 0; i < 4; i++) {
        actionHistory[i] = SENTINEL_VALUE;
      }
    }

    /* Drops off robot at the designated area. */
    void dropOffRobot() {
      turnByAngle(190);
      delay(200);
      openSweeper();
      moveBackward(MOVE_BACKWARD_TIME);
      moveForward(50);
      delay(200);
      turnByAngle(-190);
      moveBackward(MOVE_BACKWARD_TIME);
      closeSweeper();
      delay(200);
      turnByAngle(190);
      delay(100);
    }

    /* Goes to the finish area after dropping off the last robot. */
    void goToFinish() {
      turnByAngle(GO_TO_FINISH_ANGLE);
      moveForward(GO_TO_FINISH_DISTANCE);
      stopMoving();
    }
     
  public:
    /* This function is called and executes the entire routine. */
    void start() {
      acknowledge();
      obtainTargetCoordinates();
      
      state = START_TO_TUNNEL;
      followLine();

      acknowledge();
      obtainRobotOrientation();

      for (int i = 0; i < numCoordinates / 2; i++) {
        state = SEARCH;
        findRobot();
        
        state = RETURN_TO_TUNNEL;
        goBackToTunnel();
        delay(200);

        state = TUNNEL_TO_SERVICE;
        followLine();    
        dropOffRobot();

        if ((i == (numCoordinates / 2) - 1) || (targetCoordinates[coordinateCounter] == 0)) {
          // All available robots have been collected
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

// Define static variables
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
  Serial.begin(9600);
  setupWiFi();
}

void loop() {
  servo.write(90); // ensure the sweeper is closed at the start
  Robot(robot);
  robot.start();
  exit(0);
}
