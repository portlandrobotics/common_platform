
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
  float targetSpeed;
  int32_t lastCounter;
  float mpwm;
  const int ciPinCurrent;
  const int ciPinEncA;
  const int ciPinEncB;
  const int ciPwmA;
  const int ciPwmB;
  int msgcnt=0;
  bool sawEdge=false;
public:
  PidControl pid;
  MotorControl(int pinCurrent, int pinEncA, int pinEncB, int pinPwmA, int pinPwmB) :
    ciPinCurrent(pinCurrent),ciPinEncA(pinEncA),ciPinEncB(pinEncB),
    ciPwmA(pinPwmA),ciPwmB(pinPwmB) {};

  int32_t getCounter() { return counter; }
  float getSpeed() { return lastSpeed; }
  float getTargetSpeed() { return targetSpeed; }
  void setTargetSpeed(float speed) {
    if(targetSpeed != speed) {
      targetSpeed=speed;
      pid.errorSum=0;
      Serial.print("nsp ");
      Serial.println(speed);
    }
  }
  void resetCounter() { counter=0; lastCounter=0; }

  void readCurrent() {
    auto ocm = analogRead(ciPinCurrent);
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
      if(digitalRead(ciPinEncA) == HIGH)
        enc |= 1;
      if(digitalRead(ciPinEncB)==HIGH)
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
        float speed = 153./dt * dir; // micrometers / microsecond => m/s
        
        lastSpeed=speed;
        lastEnc=enc;
        lastEncTime=now;
        sawEdge=true;
        
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
    if(!sawEdge) {
      lastSpeed=0;
    }
    sawEdge = false;
    //float P=.1,I=0,D=0;
    //float target=.1;

    float err = targetSpeed-lastSpeed;
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

    //static 
    if(msgcnt++>100 && setspeed != 0) {
      Serial.print(' ');
      Serial.print((ciPinCurrent==PIN_LD_OCM)?'L':'R');
      //Serial.print(ciPinCurrent);
      Serial.print(" update ");
      Serial.print(targetSpeed);
      Serial.print(" ");
      Serial.print(lastSpeed);
      Serial.print(" ");
      Serial.print(err);
      Serial.print(" ");
      //Serial.println(setspeed);
      char message[512];
      sprintf(message,"U2 %f %d, %f - %f",mpwm,setspeed,targetSpeed,lastSpeed);
      Serial.println(message);
      msgcnt=0;
    }
    if(targetSpeed==0) {
      mpwm=0;
      setspeed=0;
    }
      
     if(setspeed >= 0) {
      analogWrite(ciPwmA, setspeed);
      analogWrite(ciPwmB, 0);
    } else {
      analogWrite(ciPwmA, 0);
      analogWrite(ciPwmB, -setspeed);
    }
  }

  
};

class Point {
public:
  float x,y;
  Point() {}
  Point(float ix, float iy):x(ix),y(iy) {}
  Point(const Point &o) {x=o.x;y=o.y;}
  Point& operator+=(const Point &o) {
    x+=o.x;
    y+=o.y;
    return *this;
  }
};
class Motion {
  public:
  Point location;
  float theta;
  int32_t lastLeft,lastRight;
  float track=0.142;//meters
  float meterspercount=0.000153;

  void reset() {
    location.x=0;
    location.y=0;
    theta=0;
    lastLeft=0;
    lastRight=0;
  }

  void update(int32_t newLeft,int32_t newRight) {
    float AL = (newLeft-lastLeft)*meterspercount;
    float AR = (newRight-lastRight)*meterspercount;
    char pbuf[512];
    sprintf(pbuf,"move %f,%f",AL,AR);
    Serial.println(pbuf);
    //Serial.println(AL);
    lastLeft=newLeft;
    lastRight=newRight;

    //float theta = (AR-AL)/track;
    if (AL==AR) {
      // straight
      location.x += AL * sin(theta);
      location.y += AL * cos(theta);
    }
    else {
      float radius = track/2*(AR+AL)/(AR-AL);
      //Serial.println(radius);
      float insttheta = (AR-AL)/track;
      //Serial.println(insttheta);
      Point RC(location);
      RC.x -= radius * sin(theta);
      RC.y += radius * cos(theta);
      Point newloc(RC);
      newloc.x += cos(insttheta)*(location.x-RC.x) - sin(insttheta) * (location.y-RC.y);
      newloc.y += sin(insttheta)*(location.x-RC.x) + cos(insttheta) * (location.y-RC.y);
      location=newloc;
      theta += insttheta;
      //todo: normalize to 0<=theta<2*PI
    }
    if(AL || AR) {
      print();
    }
  }
  void print() {
    char buf[256];
    sprintf(buf,"Pose: %f,%f %f",location.x,location.y,theta*180/M_PI);
    Serial.println(buf);
  }
};


//globals

//left motor is backwards
MotorControl leftMotor (PIN_LD_OCM,PIN_LENCB,PIN_LENCA,PIN_LD_PWM2,PIN_LD_PWM1);
MotorControl rightMotor(PIN_RD_OCM,PIN_RENCA,PIN_RENCB,PIN_RD_PWM1,PIN_RD_PWM2);
PidControl differential;

Motion motion;

void setup() {
  leftMotor.pid.P = .02;
  leftMotor.pid.I = 0;
  leftMotor.pid.D = 0;
  rightMotor.pid.P = .04;
  rightMotor.pid.I = 0;
  rightMotor.pid.D = 0;
  differential.P=.1;
  differential.I=.0;
  differential.D=.0;
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

int32_t leftTarget=0;
int32_t rightTarget=0;
bool cmdDrive=false;
float topspeed=.1;
float targetangle=0;
bool move = false;

void parseCommand(const char * const cmd) {
  if(strchr("LRD",cmd[0])) {
    float tmp = atof(cmd+1);
    if(cmd[0]=='L') {
      tmp = (tmp/180.)*M_PI*motion.track/2./motion.meterspercount;
      leftTarget  -= tmp;
      rightTarget += tmp;
      move=false;
    }
    else if(cmd[0]=='R') {
      tmp = (tmp/180.)*M_PI*motion.track/2./motion.meterspercount;
      leftTarget  += tmp;
      rightTarget -= tmp;
      move=false;
    }
    else if(cmd[0]=='D') {
      tmp /= motion.meterspercount;
      leftTarget  += tmp;
      rightTarget += tmp;
      targetangle=motion.theta;
      move=true;
    }
  }
  else if(cmd[0]=='M') {
    cmdDrive = ! cmdDrive;
  }
  else if(cmd[0]=='X') {
    leftMotor.resetCounter();
    rightMotor.resetCounter();
    leftTarget=0;
    rightTarget=0;
    motion.reset();
  }
  else if(cmd[0]=='P') {
    float tmp = atof(cmd+1);
    //leftMotor.pid.P=tmp;
    //rightMotor.pid.P=tmp;
    differential.P=tmp;
  }
  else if(cmd[0]=='I') {
    float tmp = atof(cmd+1);
    //leftMotor.pid.P=tmp;
    //rightMotor.pid.P=tmp;
    differential.I=tmp;
  }
  else if(cmd[0]=='V') {
    topspeed=atof(cmd+1);
  }
  else {
    Serial.println("Error");
    return;
  }
  
  Serial.println("OK");
}

void printStatus() {
  char buf[128];
  sprintf(buf,"LT %ld, LC %ld, RT %ld, RC %ld %f",
    leftTarget,leftMotor.getCounter(),
    rightTarget,rightMotor.getCounter(),leftMotor.pid.P);
  Serial.println(buf);
}

void drivecontrol(MotorControl * mc, int target,float corr) {
  float newSpeed=0;
  if(cmdDrive) {
    //check position against target
    auto dLeft = target - mc->getCounter();
    if(dLeft > 100)
      newSpeed=topspeed+corr;
    else if(dLeft > 10)
      newSpeed=topspeed+corr;//(.02);
    else if(dLeft > -10)
      newSpeed=(0);
    else if(dLeft > -100)
      newSpeed= -topspeed+corr;//-.02;
    else
      newSpeed= -topspeed+corr;
  } else {
    newSpeed=0;
  }
  mc->setTargetSpeed(newSpeed);
}

void loop() {
  //static int32_t x=0,y=0;
  //motion.print();
  //motion.update(x+=100,y+=101);


  static char cmdbuf[16];
  static size_t cmdbufpos=0;
  static float corr=0;
#if 0
  return;  
#endif

  leftMotor.readSensor();
  rightMotor.readSensor();
  const auto nowMicros = micros();
  static auto nextMotionUpdate = nowMicros;
  if (nowMicros >= nextMotionUpdate) {
    motion.update(leftMotor.getCounter(),rightMotor.getCounter());
    if(move) {
      corr=differential.update(motion.theta-targetangle);
      //Serial.println(motion.theta-targetangle);
      if(abs(corr)>0.01) {
        Serial.print("corr: ");
        Serial.println(corr);
      }
    } else {
      corr =0;
    }
    leftMotor.motionUpdate();
    rightMotor.motionUpdate();
    nextMotionUpdate += 100000; // 10ms or 100Hz
  }

  if(corr < -topspeed)
    corr=-topspeed;
  else if(corr > topspeed)
    corr=topspeed;
  drivecontrol(&leftMotor,leftTarget,corr);
  drivecontrol(&rightMotor,rightTarget,-corr);
  static auto nextStatusUpdate = nowMicros;
  if(nowMicros >= nextStatusUpdate) {
    printStatus();
    motion.print();
    nextStatusUpdate = nowMicros+1000000;
  }

  static auto nextVoltageUpdate = nowMicros;
  if(nowMicros >= nextVoltageUpdate) {
    Serial.print("Voltage: ");
     Serial.println(analogRead(PIN_VBAT));
     nextVoltageUpdate = nowMicros + 10000000;
  }

  while (Serial.available()) {
    char c = Serial.read();
    if(c == '\n' || c == '\r') {
      cmdbuf[cmdbufpos]=0;
      parseCommand(cmdbuf);
      cmdbufpos=0;
    }
    else if(cmdbufpos < (sizeof(cmdbuf)-1)) {
      cmdbuf[cmdbufpos++]=c;
    }
  }
}
