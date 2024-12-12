#include <ESP8266WiFi.h>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>

// ********** USER CONFIGURATION **********
const char* ssid = "Server";
const char* password = "apple123";
const char* serverIP = "192.168.43.64"; // Replace with your server's IP
const int serverPort = 12345;
// ****************************************

// Motor Control Pins
#define IN1 D8
#define IN2 D7
#define IN3 D4
#define IN4 D3

// Ultrasonic Sensor Pins
#define TRIG_PIN D6
#define ECHO_PIN D5

WiFiClient client;
MPU6050 mpu;

// Variables
volatile long duration = 0;
volatile bool echoReceived = false;
long distance = -1;
unsigned long lastMeasureTime = 0;

uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
float estimated_yaw = 0.0;
float estimated_pitch = 0.0;
float estimated_roll = 0.0;

// Moving Average Filter for yaw
#define YAW_BUFFER_SIZE 10
float yawBuffer[YAW_BUFFER_SIZE];
int yawBufferIndex = 0;
float yawSum = 0.0;

ICACHE_RAM_ATTR void echoISR() {
  static unsigned long startTime = 0;
  if (digitalRead(ECHO_PIN) == HIGH) {
    startTime = micros();
  } else {
    duration = micros() - startTime;
    echoReceived = true;
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("ESP8266 Car Initialized.");

  // Initialize Motor Control Pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopCar();

  // Initialize Ultrasonic Sensor Pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoISR, CHANGE);

  // Connect to Wi-Fi
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected.");
  Serial.print("IP: "); Serial.println(WiFi.localIP());

  // Connect to TCP server
  Serial.print("Connecting to server ");
  Serial.print(serverIP); Serial.print(":"); Serial.println(serverPort);
  while (!client.connect(serverIP, serverPort)) {
    Serial.println("Connection to server failed, retrying...");
    delay(2000);
  }
  Serial.println("Connected to server.");

  // Initialize MPU6050 with DMP
  Wire.begin(D2, D1);  // SDA to D2, SCL to D1
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }

  uint8_t devStatus = mpu.dmpInitialize();
  mpu.setXAccelOffset(-2291);
  mpu.setYAccelOffset(-1602);
  mpu.setZAccelOffset(1228);
  mpu.setXGyroOffset(87);
  mpu.setYGyroOffset(-72);
  mpu.setZGyroOffset(8);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    Serial.println("DMP ready");
    mpu.setRate(19);  // ~10 Hz
  } else {
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }

  // Initialize yaw buffer
  for (int i = 0; i < YAW_BUFFER_SIZE; i++) {
    yawBuffer[i] = 0.0;
  }
}

void loop() {
  // Non-blocking distance measurement
  if (millis() - lastMeasureTime >= 500) {
    lastMeasureTime = millis();
    if (echoReceived) {
      if (duration != 0) {
        distance = (duration / 2) / 29.1;
      } else {
        distance = -1;
      }
      echoReceived = false;
    }
    // Trigger new measurement
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
  }

  // Read MPU6050 data
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    float raw_yaw = ypr[0] * 180 / PI;
    estimated_pitch = ypr[1] * 180 / PI;
    estimated_roll = ypr[2] * 180 / PI;
    if (raw_yaw < 0) raw_yaw += 360.0;
    float current_yaw = raw_yaw;

    // Apply moving average filter to yaw
    yawSum -= yawBuffer[yawBufferIndex];
    yawBuffer[yawBufferIndex] = current_yaw;
    yawSum += current_yaw;
    yawBufferIndex = (yawBufferIndex + 1) % YAW_BUFFER_SIZE;
    estimated_yaw = yawSum / YAW_BUFFER_SIZE;
  }

  // Check if data is available from server
  if (client.available()) {
    String cmd = client.readStringUntil('\n');
    cmd.trim();
    Serial.print("Command from server: "); Serial.println(cmd);
    handleCommand(cmd);
  }

  // Periodically send sensor data
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 1000) {
    lastSend = millis();
    String data = "DIST:" + String(distance) + " P:" + String(estimated_pitch,2) + 
                  " R:" + String(estimated_roll,2) + " Y:" + String(estimated_yaw,2) + "\n";
    client.print(data);
  }

  // If disconnected, try to reconnect
  if (!client.connected()) {
    Serial.println("Disconnected from server, retrying...");
    while (!client.connect(serverIP, serverPort)) {
      Serial.println("Failed to reconnect. Retrying...");
      delay(2000);
    }
    Serial.println("Reconnected to server.");
  }
}

// Motor Control Functions
void stopCar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Car Stopped.");
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  //Serial.println("Moving Forward.");
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  //Serial.println("Moving Backward.");
}

void turnLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  //Serial.println("Turning Left.");
}

void turnRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  //Serial.println("Turning Right.");
}

void rotateAroundLeft() {
  digitalWrite(IN1, HIGH);  // Right Motor Forward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);   
  digitalWrite(IN4, LOW);
  //Serial.println("Rotating Around Left Wheels.");
}

void rotateAroundRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);   
  digitalWrite(IN4, HIGH);
  //Serial.println("Rotating Around Right Wheels.");
}

// Move forward by a specified distance (in cm) based on ultrasonic readings
void moveForwardByDistance(float distToMove) {
  // Wait for a valid distance reading
  if (distance < 0) {
    Serial.println("No valid distance reading. Can't move by distance.");
    return;
  }

  float startDist = distance;
  float targetDist = startDist - distToMove;
  if (targetDist < 0) targetDist = 0; // Prevent going negative if no obstacle?

  Serial.print("Moving forward by "); Serial.print(distToMove); Serial.println(" cm...");
  Serial.print("StartDist: "); Serial.println(startDist);
  Serial.print("TargetDist: "); Serial.println(targetDist);

  unsigned long startTime = millis();
  const float THRESHOLD = 2.0; // cm threshold
  while (true) {
    // Move forward
    moveForward();

    // Check if we got close to the target
    if (distance > 0 && fabs(distance - targetDist) <= THRESHOLD) {
      stopCar();
      Serial.println("Reached target distance.");
      break;
    }

    // Timeout to prevent endless loop
    if (millis() - startTime > 15000) { // 15 seconds timeout
      stopCar();
      Serial.println("Timeout reached, stopping.");
      break;
    }

    delay(100);
  }
}

// Handle Commands
void handleCommand(String cmd) {
  if (cmd == "F") {
    moveForward();
  } else if (cmd == "B") {
    moveBackward();
  } else if (cmd == "L") {
    turnLeft();
  } else if (cmd == "R") {
    turnRight();
  } else if (cmd == "S") {
    stopCar();
  } else if (cmd == "RL") {
    rotateAroundLeft();
  } else if (cmd == "RR") {
    rotateAroundRight();
  } else if (cmd.startsWith("ROT:")) {
    cmd.replace("ROT:", "");
    cmd.trim();
    float angle = cmd.toFloat();
    Serial.print("Rotating by angle: "); Serial.println(angle);
    rotateToAngle(angle);
  } else if (cmd.startsWith("MOVE:")) {
    cmd.replace("MOVE:", "");
    cmd.trim();
    float distVal = cmd.toFloat();
    moveForwardByDistance(distVal);
  } else {
    Serial.println("Unknown Command.");
  }
}

// Normalize angle to 0-360 range
float normalizeAngle(float angle) {
  while (angle < 0) angle += 360.0;
  while (angle >= 360.0) angle -= 360.0;
  return angle;
}

// Get current yaw
float getCurrentYaw() {
  return estimated_yaw;
}

// Rotate to a specific angle (relative)
void rotateToAngle(float relativeAngle) {
  float startYaw = getCurrentYaw();
  float targetYaw = startYaw + relativeAngle;
  targetYaw = normalizeAngle(targetYaw);

  float cwDistance = (startYaw > targetYaw) ? (startYaw - targetYaw) : (startYaw + 360.0 - targetYaw);
  float ccwDistance = 360.0 - cwDistance;

  bool rotateClockwise = (cwDistance < ccwDistance);
  const float THRESHOLD = 2.0; // degrees
  unsigned long startTime = millis();

  while (true) {
    float current = getCurrentYaw();
    float diff = targetYaw - current;
    diff = normalizeAngle(diff);

    if (fabs(diff) < THRESHOLD || fabs(diff - 360.0) < THRESHOLD) {
      stopCar();
      Serial.println("Reached target angle.");
      break;
    }

    if (millis() - startTime > 15000) {
      stopCar();
      Serial.println("Rotation timeout - stopping.");
      break;
    }

    if (rotateClockwise) {
      rotateAroundLeft(); 
    } else {
      rotateAroundRight();
    }

    delay(100);
  }
}
