
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

#define PRINT_MOTION 1


#include <Arduino.h>
#if PRINT_MOTION
#include <Wire.h>
#endif
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

#define LED_PIN 13

#define RENC 0
#define RMOT RENC
#define LENC 1
#define LMOT LENC

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

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
#if 1
      SerialUSB1.print("nsp ");
      SerialUSB1.println(speed);
#endif
    }
  }
  void resetCounter() { counter=0; lastCounter=0; }

  void readCurrent() {
    auto ocm = analogRead(ciPinCurrent);
    // .5V per amp, 3.3V ref, 1024 counts, so 3.3V / 1024 counts / .5V/A = 6.4mA/count
    //SerialUSB1.print(ocm);
    static auto lastOcm = 0;
    if(lastOcm != 0 || ocm != 0) {
//      SerialUSB1.print("Left motor current: ");
//      SerialUSB1.print(ocm*64/10);
//      SerialUSB1.println(" mA");
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
        SerialUSB1.print("enc ");
        SerialUSB1.print(enc);
        SerialUSB1.print(" ");
        SerialUSB1.print(dir);
        SerialUSB1.print(" ");
        SerialUSB1.println(speed);
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
    //SerialUSB1.print(" ");
    //SerialUSB1.println(counter-lastCounter);
    lastCounter=counter;

    int setspeed = mpwm*255;
    if(setspeed > 255)
      setspeed=255;
     if(setspeed< -255)
      setspeed=-255;

    //static
    if(msgcnt++>100 && setspeed != 0) {
      //SerialUSB1.print(' ');
      //SerialUSB1.print((ciPinCurrent==PIN_LD_OCM)?'L':'R');
      //SerialUSB1.print(ciPinCurrent);
      //SerialUSB1.print(" update ");
      //SerialUSB1.print(targetSpeed);
      //SerialUSB1.print(" ");
      //SerialUSB1.print(lastSpeed);
      //SerialUSB1.print(" ");
      //SerialUSB1.print(err);
      //SerialUSB1.print(" ");
      //SerialUSB1.println(setspeed);
      char message[512];
      sprintf(message,"U2 %f %d, %f - %f",mpwm,setspeed,targetSpeed,lastSpeed);
      //SerialUSB1.println(message);
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
  float wheelRadius=0.035; // wheel radius in meters
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
    // SerialUSB1.println(pbuf);
    //SerialUSB1.println(AL);
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
      //SerialUSB1.println(radius);
      float insttheta = (AR-AL)/track;
      //SerialUSB1.println(insttheta);
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
    // if(AL || AR) {
    //   print();
    // }
  }
  void print() {
    char buf[256];
    sprintf(buf,"Pose: %f,%f %f",location.x,location.y,theta*180/M_PI);
    SerialUSB1.println(buf);
  }
};

void error_loop()
{
	while (1)
	{
		digitalWrite(LED_PIN, !digitalRead(LED_PIN));
		delay(100);
	}
}

//twist message cb
void subscription_callback(const void *msgin)
{
	const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
	const float linear_vel = msg->linear.x;
	const float angular_vel = msg->angular.z;
	digitalWrite(LED_PIN, (msg->linear.x == 0) ? LOW : HIGH);
 SerialUSB1.println("TWIST");
	// Do differential drive inverse kinematics
	// goalSpeed[0] = linear_vel + 0.5 * angular_vel * WHEEL_BASE;  // right wheel
	// goalSpeed[1] = linear_vel - 0.5 * angular_vel * WHEEL_BASE;  // Left wheel
  parseTwist(msg);
}


//globals

//left motor is backwards
MotorControl leftMotor (PIN_LD_OCM,PIN_LENCB,PIN_LENCA,PIN_LD_PWM2,PIN_LD_PWM1);
MotorControl rightMotor(PIN_RD_OCM,PIN_RENCA,PIN_RENCB,PIN_RD_PWM1,PIN_RD_PWM2);
PidControl differential;

Motion motion;

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

	// create subscriber
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"cmd_vel"));

	// create executor
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));


  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&subscriber, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
	set_microros_transports();
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LOW);

	delay(2000);
  state = WAITING_AGENT;

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
//  SerialUSB1.print("setup returned ");
//  SerialUSB1.println(rv);

  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6b);
  Wire.write(0x00);
  rv = Wire.endTransmission(true);
//  SerialUSB1.print("setup returned ");
//  SerialUSB1.println(rv);

}

int32_t leftTargetDistance=0;
int32_t rightTargetDistance=0;
float targetHeading=0;
bool cmdDrive=true;
float targetLinearVelocity = 0.0;
float targetAngularVelocity = 0.0;
bool move = false;

void parseTwist(const geometry_msgs__msg__Twist *msg) {
	const float linear_vel = msg->linear.x;
	const float angular_vel = msg->angular.z;
      targetLinearVelocity = msg->linear.x;
      SerialUSB1.print("TLV");SerialUSB1.println(targetLinearVelocity);
      targetAngularVelocity = msg->angular.z;
      SerialUSB1.print("TAV");SerialUSB1.println(targetAngularVelocity);
      move=false;
  if (linear_vel != 0) {
      move=true;
  } else {
      move=false;
  }
}

void parseCommand(const char * const cmd) {
  if(strchr("LRD",cmd[0])) {
    float tmp = atof(cmd+1);
    if(cmd[0]=='L') {
      tmp = (tmp/180.)*M_PI*motion.track/2./motion.meterspercount;
      leftTargetDistance  -= tmp;
      rightTargetDistance += tmp;
      move=false;
    }
    else if(cmd[0]=='R') {
      tmp = (tmp/180.)*M_PI*motion.track/2./motion.meterspercount;
      leftTargetDistance  += tmp;
      rightTargetDistance -= tmp;
      move=false;
    }
    else if(cmd[0]=='D') {
      tmp /= motion.meterspercount;
      leftTargetDistance  += tmp;
      rightTargetDistance += tmp;
      targetHeading=motion.theta;
      move=true;
    }
  }
  else if(cmd[0]=='M') {
    cmdDrive = ! cmdDrive;
  }
  else if(cmd[0]=='X') {
    leftMotor.resetCounter();
    rightMotor.resetCounter();
    leftTargetDistance=0;
    rightTargetDistance=0;
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
    targetLinearVelocity=atof(cmd+1);
  }
  else {
    SerialUSB1.println("Error");
    return;
  }

  SerialUSB1.println("OK");
}

void printStatus() {
  char buf[128];
  sprintf(buf,"LT %ld, LC %ld, RT %ld, RC %ld %f",
    leftTargetDistance,leftMotor.getCounter(),
    rightTargetDistance,rightMotor.getCounter(),leftMotor.pid.P);
  SerialUSB1.println(buf);
  sprintf(buf, "LV %f, RV %f", leftMotor.getSpeed(), rightMotor.getSpeed());
  SerialUSB1.println(buf);
}

// Update wheel RPM for each wheel using differential drive kinematics
void driveControl() {
  float leftVelocity =  targetLinearVelocity - 0.5*targetAngularVelocity*motion.track/motion.wheelRadius;
  float rightVelocity = targetLinearVelocity + 0.5*targetAngularVelocity*motion.track/motion.wheelRadius;
  leftMotor.setTargetSpeed(leftVelocity);
  rightMotor.setTargetSpeed(rightVelocity);
}

// void drivecontrol(MotorControl * mc, int target,float corr) {
//   float newSpeed=0;
//   if(cmdDrive) {
//     //check position against target
//     auto dLeft = target - mc->getCounter();
//     if(dLeft > 100)
//       newSpeed=targetLinearVelocity+corr;
//     else if(dLeft > 10)
//       newSpeed=targetLinearVelocity+corr;//(.02);
//     else if(dLeft > -10)
//       newSpeed=(0);
//     else if(dLeft > -100)
//       newSpeed= -targetLinearVelocity+corr;//-.02;
//     else
//       newSpeed= -targetLinearVelocity+corr;
//   } else {
//     newSpeed=0;
//   }
//   mc->setTargetSpeed(newSpeed);
// }

void loop()
{
  // static int32_t x=0,y=0;
  // motion.print();
  // motion.update(x+=100,y+=101);

  static char cmdbuf[16];
  static size_t cmdbufpos = 0;
  static float corr = 0;
#if 0
  return;
#endif

  leftMotor.readSensor();
  rightMotor.readSensor();
  const auto nowMicros = micros();
  static auto nextMotionUpdate = nowMicros;
  if (nowMicros >= nextMotionUpdate)
  {
    motion.update(leftMotor.getCounter(), rightMotor.getCounter());
//     if (move)
//     {
//       corr = differential.update(motion.theta - targetHeading);
//       // SerialUSB1.println(motion.theta-targetangle);
//       if (abs(corr) > 0.01)
//       {
// #if 1
//         SerialUSB1.print("corr: ");
//         SerialUSB1.println(corr);
// #endif
//       }
//     }
//     else
//     {
//       corr = 0;
//     }
    leftMotor.motionUpdate();
    rightMotor.motionUpdate();
    nextMotionUpdate += 100000; // 10ms or 100Hz
  }

  // if (corr < -targetLinearVelocity)
  //   corr = -targetLinearVelocity;
  // else if (corr > targetLinearVelocity)
  //   corr = targetLinearVelocity;
  // drivecontrol(&leftMotor, leftTargetDistance, corr);
  // drivecontrol(&rightMotor, rightTargetDistance, -corr);
  driveControl();
  static auto nextStatusUpdate = nowMicros;
  if (nowMicros >= nextStatusUpdate)
  {
    printStatus();
    motion.print();
    nextStatusUpdate = nowMicros + 1000000;
  }

  static auto nextVoltageUpdate = nowMicros;
  if (nowMicros >= nextVoltageUpdate)
  {
    SerialUSB1.print("Voltage: ");
    SerialUSB1.println(analogRead(PIN_VBAT));
    nextVoltageUpdate = nowMicros + 10000000;
  }

  while (SerialUSB1.available())
  {
    char c = SerialUSB1.read();
    if (c == '\n' || c == '\r')
    {
      cmdbuf[cmdbufpos] = 0;
      parseCommand(cmdbuf);
      cmdbufpos = 0;
    }
    else if (cmdbufpos < (sizeof(cmdbuf) - 1))
    {
      cmdbuf[cmdbufpos++] = c;
    }
  }

  switch (state)
  {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
    break;
  case AGENT_AVAILABLE:
    state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT)
    {
      destroy_entities();
    };
    break;
  case AGENT_CONNECTED:
    // EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
    if (state == AGENT_CONNECTED)
    {
      rclc_executor_spin_some(&executor, RCL_US_TO_NS(10));
    }
    break;
  case AGENT_DISCONNECTED:
    destroy_entities();
    state = WAITING_AGENT;
    break;
  default:
    break;
  }

  if (state == AGENT_CONNECTED)
  {
    digitalWrite(LED_PIN, 1);
  }
  else
  {
    digitalWrite(LED_PIN, 0);
  }
}
