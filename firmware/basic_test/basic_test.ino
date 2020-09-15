
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

long leftCount = 0;
long rightCount = 0;
float lastEncSpeed = 0;

class PidControl {
  public:
  float P=1,I=.1,D=.1;
  float lastError;
  float errorSum;
  long errorCount;
  PidControl() {}
  float update(float error) {
    float rv = P*error + I * errorSum + D * (error-lastError);
    lastError=error;
    errorSum = errorSum * 0.9 + error * 0.1;
    errorCount++;
    return rv;
  }
};

PidControl leftPid;

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
    if(i%2)
      data[i/2] |= num;
    else
      data[i/2] |= num << 8;
    i++;
  }
  Serial.print("Accel:");
  for(int j=0;j<7;j++) {
    if(j<3) {
      Serial.print(' ');
      Serial.print(data[j]/16384.);
    }
    else if (j==3) {
      Serial.print('\t');
      Serial.print("Temp = ");
      Serial.print(data[j]/340.+36.53);
      Serial.print("\tGyro:");
    }
    else {
      Serial.print(" ");
      Serial.print(data[j]/131.);
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
#if 1
  return;  
#endif

  const auto nowTime = millis();
  const unsigned long loopTime = nowTime % 300000;

  static long nextMotionDump = 0;
  if(nextMotionDump < nowTime) {
    dumpMotion();
    nextMotionDump = nowTime + 10000;
  }

  int setspeed=0;
#if 0
  static long lastPidUpdate = 0;
  if(nowTime > lastPidUpdate) {
    float r = leftPid.update(lastEncSpeed-.1);
    setspeed = 255*min(max(r,-1),1);
    
    #if 0
    
    Serial.print("PID: error ");
    Serial.print(lastSpeed-.1);
    Serial.print(" result ");
    Serial.println(r);
    #endif
    lastPidUpdate = nowTime+100;
  }
#endif

  if(loopTime < 5000) {
    setspeed = loopTime * 0x80 / 5000;
    setspeed = 0xff;
  }
  else if(loopTime > 15000 && loopTime < 20000)
    setspeed = ((long)loopTime - 15000) * -0x80 / 5000;

  if(setspeed >= 0) {
    analogWrite(PIN_LD_PWM1, setspeed);
    analogWrite(PIN_LD_PWM2, 0);
  } else {
    analogWrite(PIN_LD_PWM1, 0);
    analogWrite(PIN_LD_PWM2, -setspeed);
  }

#if 0
  static float lastEncSpeed = 0;
  if(lastSpeed != speed && 0) {
    Serial.write("jeff ");
    Serial.print(speed);
    Serial.write("\n");
    lastSpeed=speed;
  }
  #endif

  static auto nextCurrentRead = nowTime;
  if(nowTime > nextCurrentRead) {
    auto ocm = analogRead(PIN_LD_OCM);
    // .5V per amp, 3.3V ref, 1024 counts, so 3.3V / 1024 counts / .5V/A = 6.4mA/count
    //Serial.print(ocm);
    static auto lastOcm = 0;
    if(lastOcm != 0 || ocm != 0) {
      Serial.print("Left motor current: ");
      Serial.print(ocm*64/10);
      Serial.println(" mA");
    }
    lastOcm = ocm;
    nextCurrentRead = nowTime + 1000;
  }

  int8_t enc = 0;
  if(digitalRead(PIN_LENCA) == HIGH)
    enc |= 1;
  if(digitalRead(PIN_LENCB)==HIGH) {
    enc ^= 3;
  }

  static auto lastEnc = enc;
  static auto lastEncTime = micros();
  if(enc != lastEnc) {
    // 1440 counts per revolution
    // 70mm diameter
    // 1 revolution = Pi * 70mm = 220 mm / rev
    // 1 count = Pi * 70mm / 1440 = 153 um / count
    
    auto now = micros();
    auto dt = now-lastEncTime;
    auto dir = enc-lastEnc-2;
    if(dir<-2)
      dir +=4;
    leftCount += dir;

    static auto nextPrintRead = nowTime;
    if(nowTime > nextPrintRead) {
      float speed = 153./dt; // micrometers / microsecond => m/s
      lastEncSpeed=speed;
      Serial.print(dt/1e3);
      Serial.print(" ms, dir ");
      Serial.print(dir);
      Serial.print(" total ");
      Serial.print(leftCount*153e-6);
      Serial.print(" meters, speed ");
      Serial.print(speed);
      Serial.println(" m/s");
      nextPrintRead = nowTime + 1000;
    }
    lastEnc = enc;
    lastEncTime=now;
  }
  
}
