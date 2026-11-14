#include "RosInterface.h"
#include "RobotState.h"

#if ROS
// Override weak clock_gettime from micro_ros_arduino to match the 64-bit time_t
// ABI expected by Jazzy's precompiled libmicroros.a (built with GCC 11.3) when
// compiling with Teensyduino 1.57.x (GCC 5.4, 32-bit time_t).
#if defined(__GNUC__) && (__GNUC__ < 10)
struct timespec64_abi {
  int64_t tv_sec;
  int32_t tv_nsec;
};

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp) {
  (void)unused;
  static uint32_t last_micros = 0;
  static uint32_t rollover_count = 0;

  uint32_t m = micros();
  if (m < last_micros) {
    rollover_count++;
  }
  last_micros = m;

  uint64_t total_us = (static_cast<uint64_t>(rollover_count) << 32) | m;
  struct timespec64_abi *tp64 = reinterpret_cast<struct timespec64_abi *>(tp);
  tp64->tv_sec = static_cast<int64_t>(total_us / 1000000ULL);
  tp64->tv_nsec = static_cast<int32_t>((total_us % 1000000ULL) * 1000ULL);
  return 0;
}
#endif

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

RosAgentStatus currentRosAgentStatus = WAITING_AGENT;

// ROS-specific Macros
#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      SERIAL_OUT.printf("ROS init failed on line %d: %d\n", __LINE__,          \
                        (int)temp_rc);                                         \
      return false;                                                            \
    }                                                                          \
  }

#define EXECUTE_EVERY_N_MS(MS, X)                                              \
  do {                                                                         \
    static volatile int64_t init = -1;                                         \
    if (init == -1) {                                                          \
      init = uxr_millis();                                                     \
    }                                                                          \
    if (uxr_millis() - init > MS) {                                            \
      X;                                                                       \
      init = uxr_millis();                                                     \
    }                                                                          \
  } while (0)

// Twist message callback
void subscription_callback(const void *msgin, void *context) {
  const geometry_msgs__msg__Twist *msg =
      (const geometry_msgs__msg__Twist *)msgin;
  RobotState *robotState_ptr = (RobotState *)context;

  // Directly update robot state from twist message
  robotState_ptr->targetLinearVelocity = msg->linear.x;
  robotState_ptr->targetAngularVelocity = msg->angular.z;
  robotState_ptr->move =
      (robotState_ptr->targetLinearVelocity != 0.0f ||
       robotState_ptr->targetAngularVelocity != 0.0f); // Update move flag

#if PRINT_MOVES
  SERIAL_OUT.print("Twist Received: LinX=");
  SERIAL_OUT.print(robotState_ptr->targetLinearVelocity);
  SERIAL_OUT.print(" AngZ=");
  SERIAL_OUT.println(robotState_ptr->targetAngularVelocity);
#endif
}

// Create ROS entities
bool create_entities(void *context) {
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(
      rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  RCCHECK(rclc_subscription_init_default(
      &subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription_with_context(
      &executor, &subscriber, &msg, &subscription_callback, context,
      ON_NEW_DATA));
  return true;
}

// Destroy ROS entities
void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&subscriber, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// Handle agent connection state machine
void handleRosAgentState(void *context) {
  RobotState *robotState_ptr = (RobotState *)context;

  switch (currentRosAgentStatus) {

  case WAITING_AGENT:
    // Ping to discover agent (100ms timeout, 2 attempts)
    EXECUTE_EVERY_N_MS(500, {
      if (rmw_uros_ping_agent(100, 2) == RMW_RET_OK) {
        currentRosAgentStatus = AGENT_AVAILABLE;
        SERIAL_OUT.println("ROS Agent Available");
      }
    });
    break;

  case AGENT_AVAILABLE:
    if (create_entities(context)) {
      currentRosAgentStatus = AGENT_CONNECTED;
      SERIAL_OUT.println("ROS Agent Connected");
    } else {
      destroy_entities();
      currentRosAgentStatus = WAITING_AGENT;
      SERIAL_OUT.println("ROS entity creation failed, returning to WAITING_AGENT");
    }
    break;

  case AGENT_CONNECTED: {
    EXECUTE_EVERY_N_MS(500, {
      if (rmw_uros_ping_agent(100, 2) != RMW_RET_OK) {
        currentRosAgentStatus = AGENT_DISCONNECTED;
        SERIAL_OUT.println("ROS Agent Disconnected");
      }
    });
    if (currentRosAgentStatus == AGENT_CONNECTED) {
      // Non-blocking spin so main loop (including motor control) runs smoothly
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    }
  } break;

  case AGENT_DISCONNECTED:
    destroy_entities();
    currentRosAgentStatus = WAITING_AGENT;
    // Reset speeds when disconnected
    robotState_ptr->targetLinearVelocity = 0.0f;
    robotState_ptr->targetAngularVelocity = 0.0f;
    robotState_ptr->move = false;
    SERIAL_OUT.println("Entities destroyed, returning to WAITING_AGENT");
    break;

  default:
    break;
  }

  // Update onboard LED to show connection status
  digitalWrite(LED_PIN, (currentRosAgentStatus == AGENT_CONNECTED) ? HIGH : LOW);
}

void setup_ROS() {
  Serial.begin(115200);
  set_microros_transports();
}
#endif
