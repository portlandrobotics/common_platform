//***********************************
// PARTS Common Robot Platform
// Test Program  				 
//***********************************

#define PRINT_MOTION 0
#define PID_ENABLED 0
#define PRINT_RAMP_INFO 0
#define PRINT_PID 1
#define PRINT_ERROR 1
#define MOTORS_ENABLED 1
#define PRINT_CURRENTS 1
#define PRINT_ENCODER_SPEED 1

// set this to maximum allowed motor speed (255 is ultimate maximum)
const int MAX_PWM	  = 100;
const int MAX_RAMP_MS = 2500;
const int MAX_RAMP_PWM = 64;

const int PIN_XD_EN   =  2;
const int PIN_RD_PWM1 =  3;
const int PIN_LD_PWM1 =  4;
const int PIN_RD_PWM2 =  5;
const int PIN_LD_PWM2 =  6;
const int PIN_LENCB   =  7;
const int PIN_LENCA   =  8;
const int PIN_US_TRIG =  9;
const int PIN_VBAT    = 14;
const int PIN_US_ECHO = 15;
const int PIN_LD_OCM  = 20;
const int PIN_RD_OCM  = 21;
const int PIN_RENCB   = 22;
const int PIN_RENCA   = 23;

#include <Wire.h>

#define RENC 0
#define RMOT RENC
#define LENC 1
#define LMOT LENC

long encCount[2] = {0, 0};
float lastEncSpeed[2] = {0, 0};
const char *wheelName[2] = {"Right", "Left"};
const int motorDir[2] = {1, -1};
int motorPWMPin1[2] = {PIN_RD_PWM1, PIN_LD_PWM1};
int motorPWMPin2[2] = {PIN_RD_PWM2, PIN_LD_PWM2};

class PidControl {
	public:
	float P = 0.5, I = .1, D = .1;
	float lastError;
	float errorSum;
	float limitSum;
	long errorCount;

	PidControl() {
		Serial.println("In PidControl constructor");
		}
	float update(float error) {
		// update integral
		errorSum += I * error;

		// prevent wind-up of integral, which can result in runaway and limit cycles if poorly tuned
		limitSum = 255;
		if (errorSum > limitSum) {
			errorSum = limitSum;
		}
		else if (errorSum < -limitSum) {
			errorSum = -limitSum;
		}

		// do classic proportional / integral / differential control calculation
		float rv = P * error + errorSum + D * (error - lastError);

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

		lastError = error;

		errorCount++;
		return rv;
	}
};

PidControl pid[2];

void dumpMotion() {
	Wire.beginTransmission(0x68);
	Wire.write(0x3b);
	Wire.endTransmission();
	Wire.requestFrom(0x68,14);

	int16_t data[] = {0,0,0,0,0,0,0};
	int i=0;
	while(Wire.available()) {
		auto num = Wire.read();
		//Serial.println(num);
		if(i % 2)
			data[i/2] |= num;
		else
			data[i/2] |= num << 8;
		i++;
	}
	Serial.print("Accel:");
	for(int j = 0; j < 7; j++) {
		if(j < 3) {
			Serial.print(' ');
			Serial.print(data[j] / 16384.);
		}
		else if (j == 3) {
			Serial.print('\t');
			Serial.print("Temp = ");
			Serial.print(data[j] / 340. + 36.53);
			Serial.print("\tGyro:");
		}
		else {
			Serial.print(" ");
			Serial.print(data[j] / 131.);
		}
	}
	Serial.println();

}

void setup() {

	// Enable encoder pullups
	pinMode(PIN_LENCA, INPUT_PULLUP);
	pinMode(PIN_LENCB, INPUT_PULLUP);
	pinMode(PIN_RENCA, INPUT_PULLUP);
	pinMode(PIN_RENCB, INPUT_PULLUP);

	// Enable motor drive outputs
	pinMode(PIN_XD_EN, OUTPUT);
	pinMode(PIN_RD_PWM1, OUTPUT);
	pinMode(PIN_LD_PWM1, OUTPUT);
	pinMode(PIN_RD_PWM2, OUTPUT);
	pinMode(PIN_LD_PWM2, OUTPUT);

	// Make sure motors are off to start
	digitalWrite(PIN_XD_EN, HIGH);
	analogWrite(PIN_RD_PWM1,0);
	analogWrite(PIN_RD_PWM2,0);
	analogWrite(PIN_LD_PWM1,0);
	analogWrite(PIN_LD_PWM2,0);


	Wire.begin();
	Wire.beginTransmission(0x68);
	Wire.write(0x6b);
	Wire.write(0x80);
	auto rv = Wire.endTransmission(true);
	Serial.print("setup returned ");
	Serial.println(rv);

	Wire.begin();
	Wire.beginTransmission(0x68);
	Wire.write(0x6b);
	Wire.write(0x00);
	rv = Wire.endTransmission(true);
	Serial.print("setup returned ");
	Serial.println(rv);

}

void loop() {
	static int8_t lastEnc[2] = {0, 0};
	static unsigned long lastEncTime[2] = {0, 0};
	static int dir[2] = {1, 1};
	static bool first = true;
	unsigned long nowMicros = micros();
	const unsigned long nowMillis = millis();
	static long dt[2] = {0, 0};
	int8_t enc[2] = {0, 0};
	float speed[2];
#if PID_ENABLED
	float goalSpeed[2] = {0.1, 0.1};
#endif
	int i;

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

	if (first) {
		first = false;
		lastEnc[0] = enc[0];
		lastEnc[1] = enc[1];
	}

	for (i = 0; i < 2; i++) {
		if (enc[i] != lastEnc[i]) {
			// 1440 counts per revolution
			// 70mm diameter
			// 1 revolution = Pi * 70mm = 220 mm / rev
			// 1 count = Pi * 70mm / 1440 = 153 um / count

			dt[i] = nowMicros - lastEncTime[i];
			lastEncTime[i] = nowMicros;

			dir[i] = enc[i] - lastEnc[i] - 2;
			lastEnc[i] = enc[i];

			if (dir[i] < -2)
				dir[i] += 4;

			encCount[i] += dir[i];

			speed[i] = 153. / dt[i]; // micrometers / microsecond => m/s
			lastEncSpeed[i] = speed[i] * dir[i];
		}
	}

#if PRINT_ENCODER_SPEED
	// print distance and speed

	for (i = 0; i < 2; i++) {
		static unsigned long nextPrintRead[2] = {0, 0};

		if (nowMillis > nextPrintRead[i]) {
			Serial.print(wheelName[i]);
			Serial.print(" ");
			Serial.print(dt[i] / 1e3);
			Serial.print(" msec, dir ");
			Serial.print(dir[i]);
			Serial.print(" total ");
			Serial.print(encCount[i] * 153e-6);
			Serial.print(" meters, speed ");
			Serial.print(lastEncSpeed[i]);
			Serial.println(" m/s");
			nextPrintRead[i] = nowMillis + 1000;
		}
	}
#endif

#if PRINT_MOTION
	// display accelerometer reading

	static long nextMotionDump = 0;
	if(nextMotionDump < nowMillis) {
		dumpMotion();
		nextMotionDump = nowMillis + 10000;
	}
#endif

	static int setspeed[2] = {0, 0};

#if PID_ENABLED
	// closed loop control of motor speed

	static unsigned long lastPidUpdate = 0;

	if(nowMillis > lastPidUpdate) {
		lastPidUpdate = nowMillis + 100;

		for (i = 0; i < 2; i++) {
			float error = 1 * (lastEncSpeed[i] - goalSpeed[i]);
			float r = pid[i].update(error);
			setspeed[i] = 255 * min(max(r, -1), 1);

#if PRINT_ERROR
			Serial.print("PID: error ");
			Serial.print(error);
			Serial.print(" result ");
			Serial.println(r);
#endif
		}
	}

#else
	// open loop ramp of motor speed

	const unsigned long loopTime = nowMillis % MAX_RAMP_MS;
	static unsigned long lastLoopTime = 0;
	static int driveDir = 1;

	if (loopTime < lastLoopTime) {
		// looped around, so switch directions
		driveDir *= -1;
	}
	for (i = 0; i < 2; i++) {
		if (loopTime < MAX_RAMP_MS / 2) {
			setspeed[i] = loopTime * MAX_RAMP_PWM / (MAX_RAMP_MS / 2);
		}
		else if(loopTime >= MAX_RAMP_MS / 2 && loopTime < MAX_RAMP_MS) {
			setspeed[i] = (MAX_RAMP_MS - loopTime) * MAX_RAMP_PWM / (MAX_RAMP_MS / 2);
		}
		setspeed[i] *= driveDir;
#if PRINT_RAMP_INFO 	
		Serial.print(wheelName[i]);
		Serial.print("driveDir: ");
		Serial.print(driveDir * motorDir[i]);
		Serial.print(" speed: ");
		Serial.print(setspeed[i]);
		Serial.print(" time: ");
		Serial.println(loopTime); 
#endif    
	}
	lastLoopTime = loopTime;
#endif

#if MOTORS_ENABLED
	// set motor PWM for both motors

	for (i = 0; i < 2; i++) {
		if (setspeed[i] > MAX_PWM) {
			setspeed[i] = MAX_PWM;
		}
		else if (setspeed[i] < -MAX_PWM) {
			setspeed[i] = -MAX_PWM;
		}
		if (setspeed[i] * motorDir[i] >= 0) {
			analogWrite(motorPWMPin1[i], setspeed[i] * motorDir[i]);
			analogWrite(motorPWMPin2[i], 0);
		} else {
			analogWrite(motorPWMPin1[i], 0);
			analogWrite(motorPWMPin2[i], -setspeed[i] * motorDir[i]);
		}
	}
#else
	serial.print("Right output: ");
	serial.print(setspeed[RMOT]);
	serial.print(" Left output: ");
	serial.println(setspeed[LMOT]);
#endif

#if PRINT_CURRENTS
	// print motor currents

	static auto nextCurrentRead = nowMillis;
	if (nowMillis > nextCurrentRead) {

		auto ocm = analogRead(PIN_RD_OCM);
		// .5V per amp, 3.3V ref, 1024 counts, so 3.3V / 1024 counts / .5V/A = 6.4mA/count
		//Serial.print(ocm);
		static auto lastOcmR = 0;
		if (lastOcmR != 0 || ocm != 0) {
			Serial.print("Right motor current: ");
			Serial.print(ocm * 64 / 10);
			Serial.println(" mA");
		}
		lastOcmR = ocm;

		ocm = analogRead(PIN_LD_OCM);
		// .5V per amp, 3.3V ref, 1024 counts, so 3.3V / 1024 counts / .5V/A = 6.4mA/count
		//Serial.print(ocm);
		static auto lastOcmL = 0;
		if (lastOcmL != 0 || ocm != 0) {
			Serial.print("Left motor current: ");
			Serial.print(ocm * 64 / 10);
			Serial.println(" mA");
		}
		lastOcmL = ocm;

		nextCurrentRead = nowMillis + 1000;
	}
#endif
}
