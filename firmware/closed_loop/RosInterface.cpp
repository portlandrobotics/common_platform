#include "RosInterface.h"
#include "RobotState.h"

#if ROS
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
      error_loop("Failed status on line %d: %d. Aborting.\n", __LINE__,        \
                 (int)temp_rc);                                                \
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

// ROS-specific function: Error loop
void error_loop(const char *msg, int line, int rc) {
  SERIAL_OUT.printf("ROS Error at %s:%d code %d\n", __FILE__, line, rc);
  while (1) {
    // TODO: Figure out what to do with LED_PIN
    // digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Twist message callback
void subscription_callback(const void *msgin, void *context) {
  const geometry_msgs__msg__Twist *msg =
      (const geometry_msgs__msg__Twist *)msgin;
  RobotState *robotState_ptr = (RobotState *)context;

  // Directly update robot state from twist message
  robotState_ptr->targetLinearVelocity = msg->linear.x;
  robotState_ptr->targetAngularVelocity = msg->angular.z;
  robotState_ptr->cmdDrive =
      (robotState_ptr->targetLinearVelocity != 0.0f ||
       robotState_ptr->targetAngularVelocity != 0.0f); // Update move flag

  // TODO: Figure out what to do with LED_PIN
  // digitalWrite(LED_PIN, (robotState.targetLinearVelocity == 0 &&
  //                        robotState.targetAngularVelocity == 0)
  //                           ? LOW
  //                           : HIGH);

#if PRINT_MOVES
  SERIAL_OUT.print("Twist Received: LinX=");
  SERIAL_OUT.print(robotState.targetLinearVelocity);
  SERIAL_OUT.print(" AngZ=");
  SERIAL_OUT.println(robotState.targetAngularVelocity);
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
    // Use ping to discover the agent when not connected
    EXECUTE_EVERY_N_MS(500, currentRosAgentStatus =
                                (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                    ? AGENT_AVAILABLE
                                    : WAITING_AGENT;);
    break;
  case AGENT_AVAILABLE:
    // Agent is available, so create all the ROS entities
    currentRosAgentStatus =
        (true == create_entities(context)) ? AGENT_CONNECTED : WAITING_AGENT;
    if (currentRosAgentStatus == WAITING_AGENT) {
      destroy_entities(); // Clean up if creation failed
    };
    break;
  case AGENT_CONNECTED: {
    EXECUTE_EVERY_N_MS(200, currentRosAgentStatus =
                                (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                    ? AGENT_CONNECTED
                                    : AGENT_DISCONNECTED;);
    if (currentRosAgentStatus == AGENT_CONNECTED) {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
  } break;
  case AGENT_DISCONNECTED:
    // Clean up all entities and return to waiting for the agent
    destroy_entities();
    currentRosAgentStatus = WAITING_AGENT;
    // Reset speeds when disconnected
    robotState_ptr->targetLinearVelocity = 0.0f;
    robotState_ptr->targetAngularVelocity = 0.0f;
    robotState_ptr->move = false;
    break;
  default:
    break;
  }
}
void setup_ROS() { set_microros_transports(); }
#endif
