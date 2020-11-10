
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
    errorSum += error;
    errorCount++;
    return rv;
  }
};

//PidControl leftPid;

class MotorControl {
private:
  uint32_t lastEncTime;
  int8_t lastEnc;
  int32_t counter;
  float lastSpeed;
  int32_t lastCounter;
  float mpwm;
public:
  PidControl pid;
public:
  void readCurrent() {
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
  }
  void readSensor() {
    
      int8_t enc = 0;
      if(digitalRead(PIN_LENCA) == HIGH)
        enc |= 1;
      if(digitalRead(PIN_LENCB)==HIGH)
        enc ^= 3;
      
    
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
        counter += dir;
        float speed = 153./dt; // micrometers / microsecond => m/s
        
        lastSpeed=speed;
        lastEnc=enc;
        lastEncTime=now;
        
#if 0
        Serial.print("enc ");
        Serial.print(enc);
        Serial.print(" ");
        Serial.print(dir);
        Serial.print(" ");
        Serial.println(speed);
#endif
      }
  }
  void motionUpdate() {
    //float P=.1,I=0,D=0;
    float target=.1;

    float err = target-lastSpeed;
    //mpwm += err * P;
    mpwm += pid.update(err);
    //Serial.print(" ");
    //Serial.println(counter-lastCounter);
    lastCounter=counter;
    
    int setspeed = mpwm*255;
    if(setspeed > 255)
      setspeed=255;
     if(setspeed< -255)
      setspeed=-255;

          static int i=0;
    if(i++>100) {
    Serial.print("update ");
    Serial.print(lastSpeed);
    Serial.print(" ");
    Serial.println(err);
      Serial.println(setspeed);
      i=0;
    }

      
     if(setspeed >= 0) {
      analogWrite(PIN_LD_PWM1, setspeed);
      analogWrite(PIN_LD_PWM2, 0);
    } else {
      analogWrite(PIN_LD_PWM1, 0);
      analogWrite(PIN_LD_PWM2, -setspeed);
    }
  }
};

MotorControl leftMotor;
void setup() {
  leftMotor.pid.P = 1;
  leftMotor.pid.I = 0;
  leftMotor.pid.D = 0;
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

  leftMotor.readSensor();
  const auto nowMicros = micros();
  static auto nextMotionUpdate = nowMicros;
  if (nowMicros >= nextMotionUpdate) {
    leftMotor.motionUpdate();
    nextMotionUpdate += 10000; // 10ms or 100Hz
  }
}
