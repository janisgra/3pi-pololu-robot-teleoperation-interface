/**
 * @file main.cpp
 * @brief Pololu 3pi+ 32U4 Robot - Teleoperation Controller
 * @author Janis
 * @version 1.0.0
 * 
 * Receives JSON commands via ESP32 WiFi bridge and executes motor control,
 * sensor reading, and position feedback.
 * 
 * IMPORTANT: The OLED display will NOT work when WiFi teleoperation is enabled.
 * Serial1 (pins 0/1) is shared between display and UART communication.
 * 
 * See docs/message-format-definition.md for protocol details.
 */

#include <Arduino.h>
#include <Pololu3piPlus32U4.h>
#include <Pololu3piPlus32U4IMU.h>

using namespace Pololu3piPlus32U4;

// ============================================================================
// DEFINITIONS
// ============================================================================

// Communication Configuration
#define USE_SERIAL1                     // Comment out to use USB Serial (debug only)
#define SERIAL_BAUD             115200  // Must match ESP32 bridge

// Robot Physical Constants (3pi+ 32U4 Standard Edition)
#define WHEEL_DIAMETER_MM       32.0f   // 32mm wheels
#define WHEEL_BASE_MM           85.0f   // Distance between wheels
#define ENCODER_CPR             12.0f   // Counts per motor revolution
#define GEAR_RATIO              29.86f  // 30:1 MP motors (actual 29.86:1)

// Derived Constants
#define COUNTS_PER_REV          (ENCODER_CPR * GEAR_RATIO)  // ~358 counts/wheel rev
#define MM_PER_COUNT            ((PI * WHEEL_DIAMETER_MM) / COUNTS_PER_REV)

// Motor Limits
#define MAX_SPEED               400     // Maximum motor speed value

// IMU Configuration (LSM6DS33)
#define GYRO_SENSITIVITY        0.00875f  // dps per LSB at 245 dps range

// PID Constants for Heading Correction
#define KP_HEADING              0.8f
#define KI_HEADING              0.0f
#define KD_HEADING              0.1f

// Line-Follow Configuration
// Sensor readLineBlack() returns 0-4000 (centre = 2000).
// PID gains match the proven Pololu reference algorithm (integer-based):
//   power_difference = proportional/KP_DIV + integral/KI_DIV + derivative*KD_NUM/KD_DIV
// The reference uses proportional/20, integral/10000, derivative*3/2.
#define LINE_CENTRE             2000
#define LF_KP_DIVISOR           20      // proportional = error / 20
#define LF_KI_DIVISOR           10000   // integral = sum(error) / 10000
#define LF_KD_NUMERATOR         3       // derivative = (error-last) * 3/2
#define LF_KD_DIVISOR           2
#define LF_INTEGRAL_LIMIT       10000L  // anti wind-up clamp on raw integral sum
#define LF_MAX_CORRECTION       200     // max motor differential (caps wild swings)
#define LINE_LOST_THRESHOLD     100     // sensor sum below this = line lost
#define LINE_SEARCH_THRESHOLD   500     // sensor sum to confirm line during search
#define LINE_SEARCH_SPEED       80      // motor speed during search phase
#define LINE_REPORT_INTERVAL_MS 250     // how often to stream line telemetry
#define CALIBRATION_SPEED       55      // motor speed during calibration spin

// Timing
#define HEARTBEAT_INTERVAL_MS   2000
#define DISPLAY_UPDATE_MS       250

// Line Sensors
#define LINE_SENSOR_COUNT       5

// Display Mode (auto-set based on serial choice)
#ifdef USE_SERIAL1
    #define espSerial           Serial1
    #define DISPLAY_ENABLED     false   // Display shares pins 0/1 with Serial1
#else
    #define espSerial           Serial
    #define DISPLAY_ENABLED     true
#endif

// ============================================================================
// HARDWARE OBJECTS
// ============================================================================

OLED display;
Buzzer buzzer;
Motors motors;
Encoders encoders;
BumpSensors bumpSensors;
LineSensors lineSensors;
IMU imu;

// ============================================================================
// STATE STRUCTURES
// ============================================================================

struct PrecisionMove {
    bool active = false;
    char direction = 0;
    int16_t baseSpeed = 0;
    bool useDistance = false;
    float targetDistance = 0;
    uint32_t targetDuration = 0;
    int32_t startEncLeft = 0;
    int32_t startEncRight = 0;
    float startHeading = 0;
    float headingIntegral = 0;
    float lastHeadingError = 0;
    uint32_t startTime = 0;
    uint32_t lastUpdateTime = 0;
} precisionMove;

struct LineFollow {
    bool active = false;
    int16_t baseSpeed = 0;
    bool useDistance = false;
    float targetDistance = 0;
    uint32_t targetDuration = 0;
    int32_t startEncLeft = 0;
    int32_t startEncRight = 0;
    long integral = 0;                  // raw sum of error (integer, like reference)
    int lastProportional = 0;           // previous error (integer, like reference)
    uint32_t startTime = 0;
    uint32_t lastUpdateTime = 0;
    uint32_t lastReportTime = 0;
    bool lineLost = false;
    uint32_t lineLostTime = 0;          // millis when line was lost
    bool searchPhase = false;           // true = drive straight until line found
    float searchMaxDist = 0;            // max mm to search before giving up
    float searchHeading = 0;            // heading to maintain during search
} lineFollow;

struct Position {
    float x = 0;          // mm, forward is +X
    float y = 0;          // mm, left is +Y
    float heading = 0;    // degrees, CCW positive
    int32_t lastEncLeft = 0;
    int32_t lastEncRight = 0;
} position;

// ============================================================================
// GLOBAL STATE
// ============================================================================

int16_t currentLeftSpeed = 0;
int16_t currentRightSpeed = 0;
uint32_t moveEndTime = 0;
bool moveActive = false;
bool lastBumpLeft = false;
bool lastBumpRight = false;
bool imuAvailable = false;
float gyroZBias = 0;
uint32_t lastImuUpdate = 0;
uint32_t lastHeartbeatTime = 0;
uint32_t lastDisplayUpdate = 0;
uint32_t commandCount = 0;
uint32_t heartbeatCount = 0;
bool displayNeedsUpdate = true;
bool lineSensorsCalibrated = false;
uint16_t lineSensorValues[LINE_SENSOR_COUNT];
char rxBuffer[128];
uint8_t rxIndex = 0;

// ============================================================================
// JSON PARSING (minimal for ATmega32U4 flash savings)
// ============================================================================

int32_t parseIntValue(const char* json, const char* key) {
    const char* p = strstr(json, key);
    if (!p) return 0;
    p = strchr(p, ':');
    if (!p) return 0;
    while (*++p == ' ');
    return atol(p);
}

bool parseBoolValue(const char* json, const char* key) {
    const char* p = strstr(json, key);
    if (!p) return false;
    p = strchr(p, ':');
    return p && strstr(p, "true");
}

bool getCommand(const char* json, char* cmd, size_t maxLen) {
    const char* p = strstr(json, "\"cmd\"");
    if (!p) return false;
    p = strchr(p, ':');
    if (!p) return false;
    p = strchr(p, '"');
    if (!p) return false;
    p++;
    size_t i = 0;
    while (*p && *p != '"' && i < maxLen - 1) cmd[i++] = *p++;
    cmd[i] = '\0';
    return i > 0;
}

bool parseStringValue(const char* json, const char* key, char* value, size_t maxLen) {
    const char* p = strstr(json, key);
    if (!p) return false;
    p = strchr(p, ':');
    if (!p) return false;
    p = strchr(p, '"');
    if (!p) return false;
    p++;
    size_t i = 0;
    while (*p && *p != '"' && i < maxLen - 1) value[i++] = *p++;
    value[i] = '\0';
    return i > 0;
}

// ============================================================================
// IMU FUNCTIONS
// ============================================================================

void initIMU() {
    Wire.begin();
    if (imu.init()) {
        imu.enableDefault();
        imuAvailable = true;
        long sum = 0;
        for (int i = 0; i < 100; i++) {
            imu.read();
            sum += imu.g.z;
            delay(5);
        }
        gyroZBias = sum / 100.0f;
        Serial.println(F("IMU initialized"));
    } else {
        Serial.println(F("IMU init failed"));
    }
}

void updateHeadingFromGyro() {
    if (!imuAvailable) return;
    uint32_t now = micros();
    if (lastImuUpdate == 0) { lastImuUpdate = now; return; }
    float dt = (now - lastImuUpdate) / 1000000.0f;
    lastImuUpdate = now;
    imu.read();
    position.heading += (imu.g.z - gyroZBias) * GYRO_SENSITIVITY * dt;
    while (position.heading > 180) position.heading -= 360;
    while (position.heading < -180) position.heading += 360;
}

// ============================================================================
// POSITION TRACKING
// ============================================================================

void updatePositionFromEncoders() {
    int32_t encLeft = encoders.getCountsLeft();
    int32_t encRight = encoders.getCountsRight();
    float distCenter = ((encLeft - position.lastEncLeft) + (encRight - position.lastEncRight)) * MM_PER_COUNT / 2.0f;
    position.lastEncLeft = encLeft;
    position.lastEncRight = encRight;
    float headingRad = position.heading * PI / 180.0f;
    position.x += distCenter * cos(headingRad);
    position.y += distCenter * sin(headingRad);
}

void resetPosition() {
    position.x = 0;
    position.y = 0;
    position.heading = 0;
    position.lastEncLeft = encoders.getCountsLeft();
    position.lastEncRight = encoders.getCountsRight();
    lastImuUpdate = 0;
}

// ============================================================================
// PRECISION MOVE
// ============================================================================

void startPrecisionMove(char dir, int16_t speedPercent, float distMM, uint32_t durMS) {
    precisionMove.active = true;
    precisionMove.direction = dir;
    precisionMove.baseSpeed = constrain(speedPercent * MAX_SPEED / 100, 1, MAX_SPEED);
    precisionMove.useDistance = (distMM > 0);
    precisionMove.targetDistance = distMM;
    precisionMove.targetDuration = durMS;
    precisionMove.startEncLeft = encoders.getCountsLeft();
    precisionMove.startEncRight = encoders.getCountsRight();
    precisionMove.startHeading = position.heading;
    precisionMove.headingIntegral = 0;
    precisionMove.lastHeadingError = 0;
    precisionMove.startTime = millis();
    precisionMove.lastUpdateTime = millis();
    
    int16_t leftSpeed = precisionMove.baseSpeed;
    int16_t rightSpeed = precisionMove.baseSpeed;
    
    switch (dir) {
        case 's': leftSpeed = -leftSpeed; rightSpeed = -rightSpeed; break;
        case 'a': leftSpeed = -leftSpeed; break;
        case 'd': rightSpeed = -rightSpeed; break;
    }
    
    motors.setSpeeds(leftSpeed, rightSpeed);
    currentLeftSpeed = leftSpeed;
    currentRightSpeed = rightSpeed;
    moveActive = true;
}

void updatePrecisionMove() {
    if (!precisionMove.active) return;
    
    uint32_t now = millis();
    float dt = (now - precisionMove.lastUpdateTime) / 1000.0f;
    precisionMove.lastUpdateTime = now;
    
    int32_t dLeft = abs(encoders.getCountsLeft() - precisionMove.startEncLeft);
    int32_t dRight = abs(encoders.getCountsRight() - precisionMove.startEncRight);
    float distTraveled = (dLeft + dRight) * MM_PER_COUNT / 2.0f;
    
    bool complete = precisionMove.useDistance 
        ? (distTraveled >= precisionMove.targetDistance)
        : (now - precisionMove.startTime >= precisionMove.targetDuration);
    
    if (complete) {
        motors.setSpeeds(0, 0);
        currentLeftSpeed = currentRightSpeed = 0;
        precisionMove.active = moveActive = false;
        
        espSerial.print(F("{\"type\":\"pmove_done\",\"dir\":\""));
        espSerial.print(precisionMove.direction);
        espSerial.print(F("\",\"dist\":"));
        espSerial.print(distTraveled, 1);
        espSerial.print(F(",\"dur\":"));
        espSerial.print(now - precisionMove.startTime);
        espSerial.print(F(",\"pos\":{\"x\":"));
        espSerial.print(position.x, 1);
        espSerial.print(F(",\"y\":"));
        espSerial.print(position.y, 1);
        espSerial.print(F(",\"h\":"));
        espSerial.print(position.heading, 1);
        espSerial.print(F("},\"ts\":"));
        espSerial.print(now);
        espSerial.println(F("}"));
        return;
    }
    
    // PID heading correction for straight moves
    if (imuAvailable && (precisionMove.direction == 'w' || precisionMove.direction == 's')) {
        float headingError = precisionMove.startHeading - position.heading;
        while (headingError > 180) headingError -= 360;
        while (headingError < -180) headingError += 360;
        
        precisionMove.headingIntegral = constrain(precisionMove.headingIntegral + headingError * dt, -50, 50);
        float derivative = dt > 0 ? (headingError - precisionMove.lastHeadingError) / dt : 0;
        precisionMove.lastHeadingError = headingError;
        
        float correction = KP_HEADING * headingError + KI_HEADING * precisionMove.headingIntegral + KD_HEADING * derivative;
        
        int16_t leftSpeed = precisionMove.baseSpeed * (precisionMove.direction == 's' ? -1 : 1);
        int16_t rightSpeed = leftSpeed;
        
        motors.setSpeeds(
            constrain(leftSpeed + (int16_t)correction, -MAX_SPEED, MAX_SPEED),
            constrain(rightSpeed - (int16_t)correction, -MAX_SPEED, MAX_SPEED)
        );
    }
}

// ============================================================================
// LINE-FOLLOW MOVE
// ============================================================================

/**
 * Start a line-following move.
 * The robot drives forward along the black line using the 5 downward-facing
 * IR reflectance sensors for lateral PID correction.  Terminates by
 * distance (encoder counts) or duration, whichever is specified.
 */
void startLineFollow(int16_t speedPercent, float distMM, uint32_t durMS) {
    if (!lineSensorsCalibrated) {
        // Refuse to line-follow without calibration -- the raw values
        // would be meaningless.
        espSerial.println(F("{\"type\":\"error\",\"error\":\"line_not_calibrated\"}"));
        return;
    }

    lineFollow.active = true;
    lineFollow.baseSpeed = constrain(speedPercent * MAX_SPEED / 100, 1, MAX_SPEED);
    lineFollow.useDistance = (distMM > 0);
    lineFollow.targetDistance = distMM;
    lineFollow.targetDuration = durMS;
    lineFollow.startEncLeft = encoders.getCountsLeft();
    lineFollow.startEncRight = encoders.getCountsRight();
    lineFollow.integral = 0;
    lineFollow.lastProportional = 0;
    lineFollow.startTime = millis();
    lineFollow.lastUpdateTime = millis();
    lineFollow.lastReportTime = millis();
    lineFollow.lineLost = false;
    lineFollow.lineLostTime = 0;
    lineFollow.searchPhase = false;
    lineFollow.searchMaxDist = 0;

    // Start at baseSpeed; PID will modulate immediately on first update
    motors.setSpeeds(lineFollow.baseSpeed, lineFollow.baseSpeed);
    currentLeftSpeed = currentRightSpeed = lineFollow.baseSpeed;
    moveActive = true;
}

/**
 * Per-loop update for line-follow mode.
 * Reads the calibrated line sensors, computes a weighted position
 * (0-4000, centre = 2000), and applies an integer PID correction to the
 * motor differential.  Matches the Pololu reference algorithm exactly.
 * Also checks for completion and line-lost conditions.
 */
void updateLineFollow() {
    if (!lineFollow.active) return;

    // --- Line search phase: drive straight slowly until sensors find line ---
    if (lineFollow.searchPhase) {
        int32_t dL = abs(encoders.getCountsLeft() - lineFollow.startEncLeft);
        int32_t dR = abs(encoders.getCountsRight() - lineFollow.startEncRight);
        float sd = (dL + dR) * MM_PER_COUNT / 2.0f;

        lineSensors.readLineBlack(lineSensorValues);
        uint32_t ss = 0;
        for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) ss += lineSensorValues[i];

        if (ss >= LINE_SEARCH_THRESHOLD) {
            // Line found -- switch to PID tracking, reset counters
            lineFollow.searchPhase = false;
            lineFollow.startEncLeft = encoders.getCountsLeft();
            lineFollow.startEncRight = encoders.getCountsRight();
            lineFollow.startTime = millis();
            lineFollow.lastUpdateTime = millis();
            lineFollow.lastReportTime = 0;
            lineFollow.integral = 0;
            lineFollow.lastProportional = 0;
            espSerial.print(F("{\"type\":\"event\",\"event\":\"search_ok\",\"d\":"));
            espSerial.print(sd, 1);
            espSerial.println(F("}"));
            return;
        }
        if (sd >= lineFollow.searchMaxDist) {
            motors.setSpeeds(0, 0);
            currentLeftSpeed = currentRightSpeed = 0;
            lineFollow.active = moveActive = false;
            espSerial.println(F("{\"type\":\"lf_done\",\"reason\":\"no_line\"}"));
            return;
        }
        // Simple heading correction during search (P-only)
        float hErr = lineFollow.searchHeading - position.heading;
        while (hErr > 180) hErr -= 360;
        while (hErr < -180) hErr += 360;
        int16_t corr = constrain((int16_t)(hErr * KP_HEADING), -30, 30);
        motors.setSpeeds(LINE_SEARCH_SPEED + corr, LINE_SEARCH_SPEED - corr);
        currentLeftSpeed = LINE_SEARCH_SPEED + corr;
        currentRightSpeed = LINE_SEARCH_SPEED - corr;
        return;
    }

    uint32_t now = millis();

    // --- Check completion by distance or duration ---
    int32_t dLeft = abs(encoders.getCountsLeft() - lineFollow.startEncLeft);
    int32_t dRight = abs(encoders.getCountsRight() - lineFollow.startEncRight);
    float distTraveled = (dLeft + dRight) * MM_PER_COUNT / 2.0f;

    bool complete = lineFollow.useDistance
        ? (distTraveled >= lineFollow.targetDistance)
        : (now - lineFollow.startTime >= lineFollow.targetDuration);

    if (complete) {
        motors.setSpeeds(0, 0);
        currentLeftSpeed = currentRightSpeed = 0;
        lineFollow.active = moveActive = false;

        espSerial.print(F("{\"type\":\"lf_done\",\"dist\":"));
        espSerial.print(distTraveled, 1);
        espSerial.print(F(",\"dur\":"));
        espSerial.print(now - lineFollow.startTime);
        espSerial.print(F(",\"pos\":{\"x\":"));
        espSerial.print(position.x, 1);
        espSerial.print(F(",\"y\":"));
        espSerial.print(position.y, 1);
        espSerial.print(F(",\"h\":"));
        espSerial.print(position.heading, 1);
        espSerial.print(F("},\"ts\":"));
        espSerial.print(now);
        espSerial.println(F("}"));
        return;
    }

    // --- Read line position ---
    // readLineBlack() returns 0-4000, where 2000 = centred on line.
    // Uses the calibrated min/max values from a prior calibrate command.
    uint16_t linePos = lineSensors.readLineBlack(lineSensorValues);

    // Check if all sensors see very little (line lost)
    uint32_t sensorSum = 0;
    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++)
        sensorSum += lineSensorValues[i];

    if (sensorSum < LINE_LOST_THRESHOLD) {
        if (!lineFollow.lineLost) {
            lineFollow.lineLost = true;
            lineFollow.lineLostTime = now;
            espSerial.print(F("{\"type\":\"event\",\"event\":\"line_lost\",\"ts\":"));
            espSerial.print(now);
            espSerial.println(F("}"));
        }
        // Keep driving straight with last correction for a brief period
        // in case the robot is crossing a gap (e.g. sync marker transition).
        // After 500 ms of no line, stop.
        if (now - lineFollow.lineLostTime > 500) {
            motors.setSpeeds(0, 0);
            currentLeftSpeed = currentRightSpeed = 0;
            lineFollow.active = moveActive = false;
            espSerial.print(F("{\"type\":\"lf_done\",\"reason\":\"line_lost\",\"dist\":"));
            espSerial.print(distTraveled, 1);
            espSerial.print(F(",\"dur\":"));
            espSerial.print(now - lineFollow.startTime);
            espSerial.print(F(",\"pos\":{\"x\":"));
            espSerial.print(position.x, 1);
            espSerial.print(F(",\"y\":"));
            espSerial.print(position.y, 1);
            espSerial.print(F(",\"h\":"));
            espSerial.print(position.heading, 1);
            espSerial.print(F("},\"ts\":"));
            espSerial.print(now);
            espSerial.println(F("}"));
            return;
        }
        // Drive straight while line is temporarily lost
        motors.setSpeeds(lineFollow.baseSpeed, lineFollow.baseSpeed);
        currentLeftSpeed = currentRightSpeed = lineFollow.baseSpeed;
        return;
    }

    // Line is visible -- clear lost state
    if (lineFollow.lineLost) {
        lineFollow.lineLost = false;
        espSerial.print(F("{\"type\":\"event\",\"event\":\"line_found\",\"ts\":"));
        espSerial.print(now);
        espSerial.println(F("}"));
    }

    // --- PID on line position (integer-based, matches Pololu reference) ---
    // error > 0  =>  line is to the RIGHT of centre
    // error < 0  =>  line is to the LEFT  of centre
    int proportional = (int)linePos - LINE_CENTRE;
    int derivative   = proportional - lineFollow.lastProportional;
    lineFollow.integral += proportional;
    lineFollow.integral  = constrain(lineFollow.integral,
                                     -LF_INTEGRAL_LIMIT, LF_INTEGRAL_LIMIT);
    lineFollow.lastProportional = proportional;

    // power_difference > 0  =>  robot must turn RIGHT (slow right motor)
    // This matches the Pololu reference convention.
    int powerDiff = proportional / LF_KP_DIVISOR
                  + lineFollow.integral / LF_KI_DIVISOR
                  + derivative * LF_KD_NUMERATOR / LF_KD_DIVISOR;

    // Clamp correction so it can never exceed baseSpeed (no reverse)
    int maxCorr = min((int)lineFollow.baseSpeed, (int)LF_MAX_CORRECTION);
    powerDiff = constrain(powerDiff, -maxCorr, maxCorr);

    // Apply: positive powerDiff => slow RIGHT motor, speed up LEFT motor
    // This steers the robot to the RIGHT, toward the line.
    int16_t leftSpeed, rightSpeed;
    if (powerDiff > 0) {
        leftSpeed  = lineFollow.baseSpeed;
        rightSpeed = lineFollow.baseSpeed - powerDiff;
    } else {
        leftSpeed  = lineFollow.baseSpeed + powerDiff;  // powerDiff is negative
        rightSpeed = lineFollow.baseSpeed;
    }

    motors.setSpeeds(
        constrain(leftSpeed, 0, MAX_SPEED),
        constrain(rightSpeed, 0, MAX_SPEED));
    currentLeftSpeed = leftSpeed;
    currentRightSpeed = rightSpeed;

    // --- Periodic telemetry ---
    if (now - lineFollow.lastReportTime >= LINE_REPORT_INTERVAL_MS) {
        lineFollow.lastReportTime = now;
        espSerial.print(F("{\"type\":\"lf_status\",\"line\":"));
        espSerial.print(linePos);
        espSerial.print(F(",\"err\":"));
        espSerial.print(proportional);
        espSerial.print(F(",\"sns\":["));
        for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) {
            if (i > 0) espSerial.print(F(","));
            espSerial.print(lineSensorValues[i]);
        }
        espSerial.print(F("],\"spd\":["));
        espSerial.print(leftSpeed);
        espSerial.print(F(","));
        espSerial.print(rightSpeed);
        espSerial.print(F("],\"dist\":"));
        espSerial.print(distTraveled, 1);
        espSerial.print(F(",\"pos\":{\"x\":"));
        espSerial.print(position.x, 1);
        espSerial.print(F(",\"y\":"));
        espSerial.print(position.y, 1);
        espSerial.print(F(",\"h\":"));
        espSerial.print(position.heading, 1);
        espSerial.print(F("},\"ts\":"));
        espSerial.print(now);
        espSerial.println(F("}"));
    }
}

// ============================================================================
// RESPONSE FUNCTIONS
// ============================================================================

void sendAck(const char* cmd) {
    espSerial.print(F("{\"type\":\"ack\",\"cmd\":\""));
    espSerial.print(cmd);
    espSerial.print(F("\",\"ts\":"));
    espSerial.print(millis());
    espSerial.println(F("}"));
}

void sendError(const char* error) {
    espSerial.print(F("{\"type\":\"error\",\"error\":\""));
    espSerial.print(error);
    espSerial.print(F("\",\"ts\":"));
    espSerial.print(millis());
    espSerial.println(F("}"));
}

void sendStatus() {
    bumpSensors.read();
    lineSensors.read(lineSensorValues);
    
    espSerial.print(F("{\"type\":\"status\",\"battery\":"));
    espSerial.print(readBatteryMillivolts());
    espSerial.print(F(",\"enc\":["));
    espSerial.print(encoders.getCountsLeft());
    espSerial.print(F(","));
    espSerial.print(encoders.getCountsRight());
    espSerial.print(F("],\"bump\":["));
    espSerial.print(bumpSensors.leftIsPressed() ? 1 : 0);
    espSerial.print(F(","));
    espSerial.print(bumpSensors.rightIsPressed() ? 1 : 0);
    espSerial.print(F("],\"line\":["));
    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) {
        if (i > 0) espSerial.print(F(","));
        espSerial.print(lineSensorValues[i]);
    }
    espSerial.print(F("],\"mtr\":["));
    espSerial.print(currentLeftSpeed);
    espSerial.print(F(","));
    espSerial.print(currentRightSpeed);
    espSerial.print(F("],\"ts\":"));
    espSerial.print(millis());
    espSerial.println(F("}"));
}

void sendHeartbeat() {
    heartbeatCount++;
    espSerial.print(F("{\"type\":\"heartbeat\",\"count\":"));
    espSerial.print(heartbeatCount);
    espSerial.print(F(",\"battery\":"));
    espSerial.print(readBatteryMillivolts());
    espSerial.print(F(",\"mtr\":["));
    espSerial.print(currentLeftSpeed);
    espSerial.print(F(","));
    espSerial.print(currentRightSpeed);
    espSerial.print(F("],\"ts\":"));
    espSerial.print(millis());
    espSerial.println(F("}"));
}

// ============================================================================
// COMMAND PROCESSING
// ============================================================================

void processCommand(const char* json) {
    char cmd[16];
    if (!getCommand(json, cmd, sizeof(cmd))) { sendError("no_cmd"); return; }
    
    commandCount++;
    ledGreen(1);
    
    if (strcmp(cmd, "move") == 0) {
        int16_t left = constrain(parseIntValue(json, "\"left\""), -MAX_SPEED, MAX_SPEED);
        int16_t right = constrain(parseIntValue(json, "\"right\""), -MAX_SPEED, MAX_SPEED);
        uint32_t duration = parseIntValue(json, "\"duration\"");
        
        motors.setSpeeds(left, right);
        currentLeftSpeed = left;
        currentRightSpeed = right;
        moveEndTime = duration > 0 ? millis() + duration : 0;
        moveActive = duration > 0;
        
        espSerial.print(F("{\"type\":\"ack\",\"cmd\":\"move\",\"set_l\":"));
        espSerial.print(left);
        espSerial.print(F(",\"set_r\":"));
        espSerial.print(right);
        espSerial.print(F(",\"ts\":"));
        espSerial.print(millis());
        espSerial.println(F("}"));
    }
    else if (strcmp(cmd, "stop") == 0) {
        motors.setSpeeds(0, 0);
        currentLeftSpeed = currentRightSpeed = 0;
        moveActive = precisionMove.active = lineFollow.active = false;
        sendAck("stop");
    }
    else if (strcmp(cmd, "pmove") == 0) {
        char dirStr[4];
        if (!parseStringValue(json, "\"dir\"", dirStr, sizeof(dirStr))) { sendError("no_dir"); return; }
        char dir = dirStr[0];
        if (dir != 'w' && dir != 'a' && dir != 's' && dir != 'd') { sendError("bad_dir"); return; }
        
        int16_t speed = constrain(parseIntValue(json, "\"speed\""), 1, 100);
        if (speed == 0) speed = 50;
        float dist = parseIntValue(json, "\"dist\"");
        uint32_t dur = parseIntValue(json, "\"dur\"");
        if (dist <= 0 && dur <= 0) { sendError("no_dist_dur"); return; }
        
        startPrecisionMove(dir, speed, dist, dur);
        
        espSerial.print(F("{\"type\":\"ack\",\"cmd\":\"pmove\",\"dir\":\""));
        espSerial.print(dir);
        espSerial.print(F("\",\"speed\":"));
        espSerial.print(speed);
        espSerial.print(dist > 0 ? F(",\"dist\":") : F(",\"dur\":"));
        espSerial.print(dist > 0 ? (int)dist : (int)dur);
        espSerial.print(F(",\"imu\":"));
        espSerial.print(imuAvailable ? F("true") : F("false"));
        espSerial.print(F(",\"ts\":"));
        espSerial.print(millis());
        espSerial.println(F("}"));
    }
    else if (strcmp(cmd, "getpos") == 0) {
        espSerial.print(F("{\"type\":\"position\",\"x\":"));
        espSerial.print(position.x, 1);
        espSerial.print(F(",\"y\":"));
        espSerial.print(position.y, 1);
        espSerial.print(F(",\"h\":"));
        espSerial.print(position.heading, 1);
        espSerial.print(F(",\"enc\":["));
        espSerial.print(encoders.getCountsLeft());
        espSerial.print(F(","));
        espSerial.print(encoders.getCountsRight());
        espSerial.print(F("],\"ts\":"));
        espSerial.print(millis());
        espSerial.println(F("}"));
    }
    else if (strcmp(cmd, "resetpos") == 0) {
        resetPosition();
        encoders.getCountsAndResetLeft();
        encoders.getCountsAndResetRight();
        sendAck("resetpos");
    }
    else if (strcmp(cmd, "setheading") == 0) {
        // Set heading to a specific value without resetting x/y.
        // Useful for correcting gyro drift after line-follow.
        float h = parseIntValue(json, "\"h\"");
        position.heading = h;
        lastImuUpdate = 0;  // re-sync gyro dt
        espSerial.print(F("{\"type\":\"ack\",\"cmd\":\"setheading\",\"h\":"));
        espSerial.print(position.heading, 1);
        espSerial.print(F(",\"ts\":"));
        espSerial.print(millis());
        espSerial.println(F("}"));
    }
    else if (strcmp(cmd, "imu") == 0) {
        if (imuAvailable) {
            imu.read();
            espSerial.print(F("{\"type\":\"imu\",\"avail\":true,\"gyro\":["));
            espSerial.print(imu.g.x); espSerial.print(F(",")); espSerial.print(imu.g.y);
            espSerial.print(F(",")); espSerial.print(imu.g.z);
            espSerial.print(F("],\"heading\":"));
            espSerial.print(position.heading, 1);
            espSerial.print(F(",\"ts\":"));
            espSerial.print(millis());
            espSerial.println(F("}"));
        } else {
            espSerial.println(F("{\"type\":\"imu\",\"avail\":false}"));
        }
    }
    else if (strcmp(cmd, "led") == 0) {
        if (strstr(json, "\"red\"")) ledRed(parseBoolValue(json, "\"red\""));
        if (strstr(json, "\"yellow\"")) ledYellow(parseBoolValue(json, "\"yellow\""));
        if (strstr(json, "\"green\"")) ledGreen(parseBoolValue(json, "\"green\""));
        sendAck("led");
    }
    else if (strcmp(cmd, "buzzer") == 0) {
        uint16_t freq = parseIntValue(json, "\"frequency\"");
        uint16_t dur = parseIntValue(json, "\"duration\"");
        buzzer.playFrequency(freq ? freq : 440, dur ? dur : 200, 15);
        sendAck("buzzer");
    }
    else if (strcmp(cmd, "status") == 0) {
        sendStatus();
    }
    else if (strcmp(cmd, "calibrate") == 0) {
        // Calibration spin: sweep sensors over line, matching Pololu reference
        // pattern (80 steps x 20ms = 1.6s, gentle speed = 40).
        for (int i = 0; i < 80; i++) {
            if (i < 20 || i >= 60)
                motors.setSpeeds(-CALIBRATION_SPEED, CALIBRATION_SPEED);
            else
                motors.setSpeeds(CALIBRATION_SPEED, -CALIBRATION_SPEED);
            lineSensors.calibrate();
            delay(20);
        }
        motors.setSpeeds(0, 0);
        lineSensorsCalibrated = true;

        // Reset heading and position so odometry starts fresh after
        // the calibration spin (otherwise heading is at a random angle).
        resetPosition();
        encoders.getCountsAndResetLeft();
        encoders.getCountsAndResetRight();

        sendAck("calibrate");
    }
    else if (strcmp(cmd, "linefollow") == 0) {
        // Line-follow move: drive forward using line sensors for correction
        // JSON: {"cmd":"linefollow","speed":50,"dist":800}
        //   or: {"cmd":"linefollow","speed":50,"dur":5000}
        int16_t speed = constrain(parseIntValue(json, "\"speed\""), 1, 100);
        if (speed == 0) speed = 50;
        float dist = parseIntValue(json, "\"dist\"");
        uint32_t dur = parseIntValue(json, "\"dur\"");
        if (dist <= 0 && dur <= 0) { sendError("no_dist_dur"); return; }

        startLineFollow(speed, dist, dur);

        // Optional search phase: drive straight until line found before PID
        float search = parseIntValue(json, "\"search\"");
        if (search > 0) {
            lineFollow.searchPhase = true;
            lineFollow.searchMaxDist = search;
            lineFollow.searchHeading = position.heading;
            motors.setSpeeds(LINE_SEARCH_SPEED, LINE_SEARCH_SPEED);
            currentLeftSpeed = currentRightSpeed = LINE_SEARCH_SPEED;
        }

        espSerial.print(F("{\"type\":\"ack\",\"cmd\":\"linefollow\",\"speed\":"));
        espSerial.print(speed);
        espSerial.print(dist > 0 ? F(",\"dist\":") : F(",\"dur\":"));
        espSerial.print(dist > 0 ? (int)dist : (int)dur);
        espSerial.print(F(",\"calibrated\":"));
        espSerial.print(lineSensorsCalibrated ? F("true") : F("false"));
        espSerial.print(F(",\"ts\":"));
        espSerial.print(millis());
        espSerial.println(F("}"));
    }
    else if (strcmp(cmd, "ping") == 0) {
        // Latency benchmarking: echo back with robot-side timestamp
        uint32_t seq = parseIntValue(json, "\"seq\"");
        uint32_t clientTs = parseIntValue(json, "\"ts\"");
        espSerial.print(F("{\"type\":\"pong\",\"seq\":"));
        espSerial.print(seq);
        espSerial.print(F(",\"client_ts\":"));
        espSerial.print(clientTs);
        espSerial.print(F(",\"robot_ts\":"));
        espSerial.print(millis());
        espSerial.println(F("}"));
    }
    else sendError("unknown_cmd");
}

// ============================================================================
// SETUP & LOOP
// ============================================================================

void setup() {
    Serial.begin(115200);
#ifdef USE_SERIAL1
    Serial1.begin(SERIAL_BAUD);
    Serial.println(F("3pi+ Teleoperation - Display DISABLED (pins 0/1 used for UART)"));
#endif
    
    initIMU();
    bumpSensors.calibrate();
    
    buzzer.playFrequency(440, 100, 15);
    delay(150);
    buzzer.playFrequency(880, 100, 15);
    
    motors.setSpeeds(100, 100);
    delay(30);
    motors.setSpeeds(0, 0);
    
    resetPosition();
    delay(500);
    
    espSerial.print(F("{\"type\":\"event\",\"event\":\"startup\",\"imu\":"));
    espSerial.print(imuAvailable ? F("true") : F("false"));
    espSerial.println(F(",\"ts\":0}"));
    
    Serial.println(F("Ready"));
}

void loop() {
    uint32_t now = millis();
    
    updateHeadingFromGyro();
    updatePositionFromEncoders();
    updatePrecisionMove();
    updateLineFollow();
    
    while (espSerial.available()) {
        char c = espSerial.read();
        if (c == '\n') {
            if (rxIndex > 0) {
                rxBuffer[rxIndex] = '\0';
                processCommand(rxBuffer);
                rxIndex = 0;
            }
        } else if (c != '\r' && rxIndex < sizeof(rxBuffer) - 1) {
            rxBuffer[rxIndex++] = c;
        }
    }
    
    static uint32_t ledOffTime = 0;
    if (ledOffTime && now >= ledOffTime) { ledGreen(0); ledOffTime = 0; }
    if (commandCount > 0 && !ledOffTime) ledOffTime = now + 50;
    
    if (moveActive && !precisionMove.active && now >= moveEndTime) {
        motors.setSpeeds(0, 0);
        currentLeftSpeed = currentRightSpeed = 0;
        moveActive = false;
    }
    
    if (now - lastHeartbeatTime >= HEARTBEAT_INTERVAL_MS) {
        sendHeartbeat();
        lastHeartbeatTime = now;
        static bool yellow = false;
        ledYellow(yellow = !yellow);
    }
    
    bumpSensors.read();
    if (bumpSensors.leftIsPressed() && !lastBumpLeft) {
        espSerial.print(F("{\"type\":\"event\",\"event\":\"bump\",\"side\":\"left\",\"ts\":"));
        espSerial.print(now);
        espSerial.println(F("}"));
        motors.setSpeeds(0, 0);
        currentLeftSpeed = currentRightSpeed = 0;
        moveActive = precisionMove.active = lineFollow.active = false;
    }
    if (bumpSensors.rightIsPressed() && !lastBumpRight) {
        espSerial.print(F("{\"type\":\"event\",\"event\":\"bump\",\"side\":\"right\",\"ts\":"));
        espSerial.print(now);
        espSerial.println(F("}"));
        motors.setSpeeds(0, 0);
        currentLeftSpeed = currentRightSpeed = 0;
        moveActive = precisionMove.active = lineFollow.active = false;
    }
    lastBumpLeft = bumpSensors.leftIsPressed();
    lastBumpRight = bumpSensors.rightIsPressed();
}
