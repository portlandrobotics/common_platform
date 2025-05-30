//***********************************
// PARTS Common Robot Platform
// Test Program  				 
//***********************************

// Define constants to enable or disable various debug prints and features.
#define PRINT_MOTION 0        // Control print statements related to motion
#define PID_ENABLED 0         // Enable or disable PID control
#define PRINT_RAMP_INFO 0     // Control print statements related to ramping information
#define PRINT_PID 1           // Enable print statements for PID debugging
#define PRINT_ERROR 1         // Enable print statements for error reporting
#define MOTORS_ENABLED 1      // Enable or disable motor operation
#define PRINT_CURRENTS 1      // Enable print statements for current monitoring
#define PRINT_ENCODER_SPEED 1 // Enable print statements for encoder speed

// Constants for motor control
const int MAX_PWM      = 100;        // Maximum PWM value for motor speed
const int MAX_RAMP_MS  = 2500;       // Maximum ramp time in milliseconds
const int MAX_RAMP_PWM = 64;         // Maximum PWM value during ramp-up

// Pin assignments for the robot's hardware components
const int PIN_XD_EN   =  2;  // Pin for XD enable
const int PIN_RD_PWM1 =  3;  // Right drive motor, PWM pin 1
const int PIN_LD_PWM1 =  4;  // Left drive motor, PWM pin 1
const int PIN_RD_PWM2 =  5;  // Right drive motor, PWM pin 2
const int PIN_LD_PWM2 =  6;  // Left drive motor, PWM pin 2
const int PIN_LENCB   =  7;  // Left encoder, pin B
const int PIN_LENCA   =  8;  // Left encoder, pin A
const int PIN_US_TRIG =  9;  // Ultrasonic sensor trigger pin
const int PIN_VBAT    = 14;  // Battery voltage pin
const int PIN_US_ECHO = 15;  // Ultrasonic sensor echo pin
const int PIN_LD_OCM  = 20;  // Left drive overcurrent monitor pin
const int PIN_RD_OCM  = 21;  // Right drive overcurrent monitor pin
const int PIN_RENCB   = 22;  // Right encoder, pin B
const int PIN_RENCA   = 23;  // Right encoder, pin A

#include <Wire.h> // Include the Wire library for I2C communication

// Define aliases for easier understanding of the code
#define RENC 0  // Right Encoder
#define RMOT RENC // Right Motor, same as right encoder
#define LENC 1  // Left Encoder
#define LMOT LENC // Left Motor, same as left encoder

// Arrays to hold encoder counts and speeds
long encCount[2] = {0, 0};           // Array to store encoder counts for right and left wheels
float lastEncSpeed[2] = {0, 0};      // Array to store last recorded speeds for right and left encoders
const char *wheelName[2] = {"Right", "Left"}; // Array to store wheel names for easy reference
const int motorDir[2] = {1, -1};     // Array to set motor direction, 1 for forward, -1 for reverse
int motorPWMPin1[2] = {PIN_RD_PWM1, PIN_LD_PWM1}; // Array of PWM pins for each motor
int motorPWMPin2[2] = {PIN_RD_PWM2, PIN_LD_PWM2}; // Secondary PWM pins for each motor

// Definition of the PidControl class
class PidControl {
    public:
    float P = 0.5, I = .1, D = .1; // PID coefficients
    float lastError; // Last error value
    float errorSum;  // Sum of errors for integral calculation
    float limitSum;  // Limit for integral sum to prevent integral windup
    long errorCount; // Counter for the number of errors

    // Constructor for the PidControl class
    PidControl() {
        Serial.println("In PidControl constructor"); // Print a message when a PidControl object is created
    }

    // Function to update PID control
    float update(float error) {
        // Update integral sum with current error
        errorSum += I * error;

        // Prevent integral wind-up by limiting the integral sum
        limitSum = 255;
        if (errorSum > limitSum) {
            errorSum = limitSum;
        }
        else if (errorSum < -limitSum) {
            errorSum = -limitSum;
        }

        // Calculate PID output: Proportional + Integral + Derivative
        float rv = P * error + errorSum + D * (error - lastError);

        // Conditional compile for printing PID values
#if PRINT_PID
        Serial.print("P: ");
        Serial.print(P * error);
        Serial.print(" I: ");
        Serial.print(I * error);
        Serial.print(" D: ");
        Serial.print(D * (error - lastError));
        Serial.print(" rv: ");
        Serial.print(rv);
        Serial.print(" errorSum: ");
        Serial.print(errorSum);
        Serial.print(" limitSum: ");
        Serial.println(limitSum);
#endif

        lastError = error; // Update lastError with current error

        errorCount++; // Increment error count
        return rv; // Return the PID output value
    }
};

// Instantiate two PID controllers, one for each motor
PidControl pid[2];

// Function to read motion sensor data from an MPU-6050 sensor
void dumpMotion() {
    Wire.beginTransmission(0x68);  // Start communication with MPU-6050
    Wire.write(0x3b);              // Request data starting from register 0x3b
    Wire.endTransmission();        // End transmission
    Wire.requestFrom(0x68,14);     // Request 14 bytes of data from MPU-6050

    int16_t data[] = {0,0,0,0,0,0,0}; // Array to hold the sensor data
    int i=0;                          // Index for reading data
    while(Wire.available()) {         // While there is data available from MPU-6050
        auto num = Wire.read();       // Read a byte
        // If i is odd, combine current byte with previous to form a 16-bit number
        if(i % 2)
            data[i/2] |= num;
        else // If i is even, store current byte as high byte of 16-bit number
            data[i/2] |= num << 8;
        i++;
    }

    // Print accelerometer, temperature, and gyro data
    Serial.print("Accel:");
    for(int j = 0; j < 7; j++) {
        if(j < 3) { // Accelerometer data (converted to g's)
            Serial.print(' ');
            Serial.print(data[j] / 16384.);
        }
        else if (j == 3) { // Temperature data (converted to degrees Celsius)
            Serial.print('\t');
            Serial.print("Temp = ");
            Serial.print(data[j] / 340. + 36.53);
            Serial.print("\tGyro:");
        }
        else { // Gyroscope data (converted to degrees/s)
            Serial.print(" ");
            Serial.print(data[j] / 131.);
        }
    }
    Serial.println();
}

// Setup function, runs once at the beginning
void setup() {
    // Enable internal pullup resistors for encoder pins
    pinMode(PIN_LENCA, INPUT_PULLUP);
    pinMode(PIN_LENCB, INPUT_PULLUP);
    pinMode(PIN_RENCA, INPUT_PULLUP);
    pinMode(PIN_RENCB, INPUT_PULLUP);

    // Set motor driver pins to output mode
    pinMode(PIN_XD_EN, OUTPUT);
    pinMode(PIN_RD_PWM1, OUTPUT);
    pinMode(PIN_LD_PWM1, OUTPUT);
    pinMode(PIN_RD_PWM2, OUTPUT);
    pinMode(PIN_LD_PWM2, OUTPUT);

    // Initialize motors to off state
    digitalWrite(PIN_XD_EN, HIGH);
    analogWrite(PIN_RD_PWM1,0);
    analogWrite(PIN_RD_PWM2,0);
    analogWrite(PIN_LD_PWM1,0);
    analogWrite(PIN_LD_PWM2,0);

    // Initialize MPU-6050 sensor
    Wire.begin(); // Start I2C communication
    // Reset MPU-6050
    Wire.beginTransmission(0x68);
    Wire.write(0x6b); 
    Wire.write(0x80); 
    auto rv = Wire.endTransmission(true);
    Serial.print("setup returned ");
    Serial.println(rv);

    // Wake up MPU-6050
    Wire.begin();
    Wire.beginTransmission(0x68);
    Wire.write(0x6b);
    Wire.write(0x00);
    rv = Wire.endTransmission(true);
    Serial.print("setup returned ");
    Serial.println(rv);
}

// Main loop function, runs repeatedly
void loop() {
    // Variables for encoder reading and timing
    static int8_t lastEnc[2] = {0, 0};        // Last encoder values
    static unsigned long lastEncTime[2] = {0, 0}; // Time of last encoder reading
    static int dir[2] = {1, 1};               // Direction of rotation
    static bool first = true;                 // Flag for the first loop iteration
    unsigned long nowMicros = micros();       // Current time in microseconds
    const unsigned long nowMillis = millis(); // Current time in milliseconds
    static long dt[2] = {0, 0};               // Time difference between encoder readings
    int8_t enc[2] = {0, 0};                   // Current encoder values
    float speed[2];                           // Speeds of the wheels

    #if PID_ENABLED
        float goalSpeed[2] = {0.1, 0.1};          // Target speeds for PID control
    #endif
        int i;

    // Read encoder values
    if (digitalRead(PIN_LENCB) == HIGH)
        enc[LENC] |= 1;
    if (digitalRead(PIN_LENCA) == HIGH) {
        enc[LENC] ^= 3;
    }
    if (digitalRead(PIN_RENCA) == HIGH)
        enc[RENC] |= 1;
    if (digitalRead(PIN_RENCB) == HIGH) {
        enc[RENC] ^= 3;
    }

    // Handle the first loop iteration
    if (first) {
        first = false;
        lastEnc[0] = enc[0];
        lastEnc[1] = enc[1];
    }

    // Process encoder readings
    for (i = 0; i < 2; i++) {
        if (enc[i] != lastEnc[i]) {
            // Calculate time since last encoder reading
            dt[i] = nowMicros - lastEncTime[i];
            lastEncTime[i] = nowMicros;

            // Determine direction of rotation
            dir[i] = enc[i] - lastEnc[i] - 2;
            lastEnc[i] = enc[i];

            if (dir[i] < -2)
                dir[i] += 4;

            // Update encoder count
            encCount[i] += dir[i];

            // Calculate wheel speed (in meters per second)
            speed[i] = 153. / dt[i]; // Convert from micrometers/microsecond to m/s
            lastEncSpeed[i] = speed[i] * dir[i];
        }
    }

    #if PRINT_ENCODER_SPEED
        // Print distance and speed information for each wheel
        for (i = 0; i < 2; i++) {
            static unsigned long nextPrintRead[2] = {0, 0}; // Next time to print

            // Check if it's time to print for this wheel
            if (nowMillis > nextPrintRead[i]) {
                Serial.print(wheelName[i]); // Print wheel name
                Serial.print(" ");
                Serial.print(dt[i] / 1e3); // Print time interval in milliseconds
                Serial.print(" msec, dir ");
                Serial.print(dir[i]); // Print direction of wheel
                Serial.print(" total ");
                Serial.print(encCount[i] * 153e-6); // Print total distance in meters
                Serial.print(" meters, speed ");
                Serial.print(lastEncSpeed[i]); // Print speed in meters per second
                Serial.println();
                nextPrintRead[i] = nowMillis + 1000; // Schedule next print
            }
        }
    #endif

    #if PRINT_MOTION
        // Display accelerometer reading periodically
        static long nextMotionDump = 0;
        if(nextMotionDump < nowMillis) {
            dumpMotion(); // Call function to dump motion sensor data
            nextMotionDump = nowMillis + 10000; // Schedule next dump
        }
    #endif

    static int setspeed[2] = {0, 0}; // Array to store speed setpoints for motors

    #if PID_ENABLED
        // Closed loop control of motor speed using PID
        static unsigned long lastPidUpdate = 0;

        // Update PID control at regular intervals
        if(nowMillis > lastPidUpdate) {
            lastPidUpdate = nowMillis + 100; // Schedule next PID update

            for (i = 0; i < 2; i++) {
                float error = 1 * (lastEncSpeed[i] - goalSpeed[i]); // Calculate error
                float r = pid[i].update(error); // Update PID controller
                setspeed[i] = 255 * min(max(r, -1), 1); // Calculate speed setpoint

                #if PRINT_ERROR
                Serial.print("PID: error ");
                Serial.print(error); // Print PID error
                Serial.print(" result ");
                Serial.println(r); // Print PID result
                #endif
            }
        }
    }
    #else
        // Open loop ramp of motor speed
        const unsigned long loopTime = nowMillis % MAX_RAMP_MS;
        static unsigned long lastLoopTime = 0;
        static int driveDir = 1; // Direction of ramping

        // Handle loop time wrap-around and switch directions
        if (loopTime < lastLoopTime) {
            driveDir *= -1;
        }

        // Ramp motor speed up and down
        for (i = 0; i < 2; i++) {
            if (loopTime < MAX_RAMP_MS / 2) {
                setspeed[i] = loopTime * MAX_RAMP_PWM / (MAX_RAMP_MS / 2);
            }
            else if(loopTime >= MAX_RAMP_MS / 2 && loopTime < MAX_RAMP_MS) {
                setspeed[i] = (MAX_RAMP_MS - loopTime) * MAX_RAMP_PWM / (MAX_RAMP_MS / 2);
            }
            setspeed[i] *= driveDir; // Apply direction to speed

            #if PRINT_RAMP_INFO
            Serial.print(wheelName[i]); // Print wheel name
            Serial.print(" driveDir: ");
            Serial.print(driveDir * motorDir[i]); // Print drive direction
            Serial.print(" speed: ");
            Serial.print(setspeed[i]); // Print speed setpoint
            Serial.print(" time: ");
            Serial.println(loopTime); // Print current loop time
            #endif
        }
        lastLoopTime = loopTime; // Update last loop time
    #endif

    #if MOTORS_ENABLED
        // Set motor PWM for both motors
        for (i = 0; i < 2; i++) {
            // Limit the speed setpoint to maximum PWM value
            if (setspeed[i] > MAX_PWM) {
                setspeed[i] = MAX_PWM;
            }
            else if (setspeed[i] < -MAX_PWM) {
                setspeed[i] = -MAX_PWM;
            }

            // Control motor direction and speed using PWM
            if (setspeed[i] * motorDir[i] >= 0) {
                analogWrite(motorPWMPin1[i], setspeed[i] * motorDir[i]);
                analogWrite(motorPWMPin2[i], 0);
            } else {
                analogWrite(motorPWMPin1[i], 0);
                analogWrite(motorPWMPin2[i], -setspeed[i] * motorDir[i]);
            }
        }
    #else
        // Print motor outputs when motors are disabled
        Serial.print("Right output: ");
        Serial.print(setspeed[RMOT]);
        Serial.print(" Left output: ");
        Serial.println(setspeed[LMOT]);
    #endif

    #if PRINT_CURRENTS
        // Print motor currents at regular intervals
        static auto nextCurrentRead = nowMillis;
        if (nowMillis > nextCurrentRead) {
            // Read current sensor value for right motor
            auto ocm = analogRead(PIN_RD_OCM);
            static auto lastOcmR = 0;
            if (lastOcmR != 0 || ocm != 0) {
                Serial.print("Right motor current: ");
                Serial.print(ocm * 64 / 10); // Convert to mA
                Serial.println(" mA");
            }
            lastOcmR = ocm;

            // Read current sensor value for left motor
            ocm = analogRead(PIN_LD_OCM);
            static auto lastOcmL = 0;
            if (lastOcmL != 0 || ocm != 0) {
                Serial.print("Left motor current: ");
                Serial.print(ocm * 64 / 10); // Convert to mA
                Serial.println(" mA");
            }
            lastOcmL = ocm;

            nextCurrentRead = nowMillis + 1000; // Schedule next current read
        }
    #endif
}

