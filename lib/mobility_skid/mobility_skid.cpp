#include <esp_log.h>
#include <esp_timer.h>

#include "picorosso.h"
#include "mobility_skid_config.h"
#include "mobility_skid.h"

#include "sabertooth.h"
#include <ESP32Encoder.h>
#include "fpid.h"
#include "odom_helper.h"

#define TSK_MINIMAL_STACK_SIZE (1024)
#define CONTROL_TASK_NAME "skid_control_task"
#define CONTROL_TASK_STACK_SIZE (TSK_MINIMAL_STACK_SIZE * 8)
#define CONTROL_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define REPORT_TASK_NAME "skid_report_task"
#define REPORT_TASK_STACK_SIZE (TSK_MINIMAL_STACK_SIZE * 8)
#define REPORT_TASK_PRIORITY (tskIDLE_PRIORITY + 1)
#define PUBLISHER_BUF_SIZE 1024 //TODO

static const char *TAG = "skid";

static uint8_t publisher_buf[PUBLISHER_BUF_SIZE]; // pre-allocated buffer for serialization

static Sabertooth sabertooth;

static unsigned long set_control_time_ms = 0;
static unsigned long control_cb_time_ms = 0;

static ESP32Encoder encoder_rr_lft;
static ESP32Encoder encoder_fr_lft;
static ESP32Encoder encoder_fr_rgt;
static ESP32Encoder encoder_rr_rgt;
// static unsigned long last_encoder_us = 0UL;

static Fpid pid_lft(-126, 126, PID_LEFT_KF, PID_LEFT_KP, PID_LEFT_KI, PID_LEFT_KD);
static Fpid pid_rgt(-126, 126, PID_RIGHT_KF, PID_RIGHT_KP, PID_RIGHT_KI, PID_RIGHT_KD);

static OdomHelper odom;

static double state_position[4];
static double state_velocity[4];
static double state_efforts[4];

static const char *joint_names[] = {"fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint", "rr_wheel_joint"};
static ros_JointState msg_joint_state = {
    .header = {.frame_id = (char *)"base_link"},
    .name = {.data = (char **)joint_names, .n_elements = 4},
    .position = {.data = state_position, .n_elements = 4},
    .velocity = {.data = state_velocity, .n_elements = 4},
    .effort = {.data = state_efforts, .n_elements = 4},
};
static picoros_publisher_t publisher_joint = {
    .topic = {
        .name = (char *)"joint_states",
        .type = ROSTYPE_NAME(ros_JointState),
        .rihs_hash = ROSTYPE_HASH(ros_JointState),
    },
};

static void cmd_vel_cb(uint8_t *rx_data, size_t data_len);
picoros_subscriber_t subscription_cmd_vel = {
    .topic = {
        .name = (char *)"cmd_vel",
        .type = ROSTYPE_NAME(ros_Odometry),
        .rihs_hash = ROSTYPE_HASH(ros_Odometry),
    },
    .user_callback = cmd_vel_cb,
};

// static float current_linear;
// static float current_angular;

// static float target_v_lft;
// static float target_v_rgt;
static float target_linear;
static float target_angular;

static float current_v_lft;
static float current_v_rgt;

// static float last_dt_s;

static float wheel_angular_rr_lft;
static float wheel_angular_fr_lft;
static float wheel_angular_fr_rgt;
static float wheel_angular_rr_rgt;

static int64_t enc_count_rr_lft = 0;
static int64_t enc_count_fr_lft = 0;
static int64_t enc_count_fr_rgt = 0;
static int64_t enc_count_rr_rgt = 0;

MobilitySkid::MobilitySkid() {};

static void set_target_velocities(float linear, float angular)
{
  if (linear > MAX_SPEED)
    target_linear = MAX_SPEED;
  else if (linear < -MAX_SPEED)
    target_linear = -MAX_SPEED;
  else
    target_linear = linear;

  if (angular > MAX_TURNSPEED)
    target_angular = MAX_TURNSPEED;
  else if (angular < -MAX_TURNSPEED)
    target_angular = -MAX_TURNSPEED;
  else
    target_angular = angular;
}

static void compute_movement(float time_step)
{
  int64_t count_fr_lft;
  int64_t count_fr_rgt;
  int64_t count_rr_lft;
  int64_t count_rr_rgt;
  {
    // volatile DisableInterruptsGuard interrupt;
    count_fr_lft = encoder_fr_lft.getCount() * ENCODER_lft_MULT;
    count_fr_rgt = encoder_fr_rgt.getCount() * ENCODER_rgt_MULT;
    count_rr_lft = encoder_rr_lft.getCount() * ENCODER_lft_MULT;
    count_rr_rgt = encoder_rr_rgt.getCount() * ENCODER_rgt_MULT;
  }

  // rad/s
  float tics__to__rad_s = TICKS_TO_RAD / time_step;

  wheel_angular_fr_lft = tics__to__rad_s * (count_fr_lft - enc_count_fr_lft);
  wheel_angular_fr_rgt = tics__to__rad_s * (count_fr_rgt - enc_count_fr_rgt);
  wheel_angular_rr_lft = tics__to__rad_s * (count_rr_lft - enc_count_rr_lft);
  wheel_angular_rr_rgt = tics__to__rad_s * (count_rr_rgt - enc_count_rr_rgt);

  enc_count_fr_lft = count_fr_lft;
  enc_count_fr_rgt = count_fr_rgt;
  enc_count_rr_lft = count_rr_lft;
  enc_count_rr_rgt = count_rr_rgt;

  current_v_lft = WHEEL_RADIUS * (wheel_angular_rr_lft + wheel_angular_fr_lft) / 2; // FIXME
  current_v_rgt = WHEEL_RADIUS * (wheel_angular_rr_rgt + wheel_angular_fr_rgt) / 2;

  float current_linear = (current_v_lft + current_v_rgt) / 2;                         // m/s
  float current_angular = atan((current_v_rgt - current_v_lft) / LR_WHEELS_DISTANCE); // rad/s

  odom.update_pos(current_linear, 0.0, current_angular, time_step);

  /*
  D_print(current_linear);
  D_print(" m/s | rad/s ");
  D_println(current_angular);
  //  */
}

static void cmd_vel_cb(uint8_t *rx_data, size_t data_len)
{
  set_control_time_ms = pdTICKS_TO_MS(xTaskGetTickCount());

  ros_TwistStamped msg_cmd_vel = {};
  if (ps_deserialize(rx_data, &msg_cmd_vel, data_len))
  {
    // TODO verify msg_cmd_vel.header.stamp
    /*
    D_print("cmd_vel: ");
    D_print(msg_cmd_vel.linear.x);
    D_print(" ");
    D_println(msg_cmd_vel.angular.z);
    */
    set_target_velocities(
        msg_cmd_vel.twist.linear.x,
        msg_cmd_vel.twist.angular.z);
  }
  else
  {
    ESP_LOGE(TAG, "cmd_vel message deserialization error");
  }
}

static void report_task(void *)
{
  for (;;)
  {
    TickType_t last_wake_time = xTaskGetTickCount();

    msg_joint_state.velocity.data[0] = wheel_angular_fr_lft;
    msg_joint_state.velocity.data[1] = wheel_angular_fr_rgt;
    msg_joint_state.velocity.data[2] = wheel_angular_rr_lft;
    msg_joint_state.velocity.data[3] = wheel_angular_rr_rgt;

    msg_joint_state.position.data[0] = TICKS_TO_RAD * enc_count_fr_lft;
    msg_joint_state.position.data[1] = TICKS_TO_RAD * enc_count_fr_rgt;
    msg_joint_state.position.data[2] = TICKS_TO_RAD * enc_count_rr_lft;
    msg_joint_state.position.data[3] = TICKS_TO_RAD * enc_count_rr_rgt;

    // z_clock_t now = z_clock_now();
    // msg_joint_state.header.stamp.sec = (int32_t)now.tv_sec;
    // msg_joint_state.header.stamp.nanosec = (uint32_t)now.tv_nsec;
    PicoRosso::set_timestamp(msg_joint_state.header.stamp);

    pr_publish_buf(publisher_joint, msg_joint_state, publisher_buf, sizeof(publisher_buf));

    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(PERIOD_REPORT_MS));
  }
}

static void control_task(void *)
{
  for (;;)
  {
    TickType_t last_wake_time = xTaskGetTickCount();

    uint64_t now_ms = pdTICKS_TO_MS(last_wake_time);
    uint64_t time_step_ms = now_ms - control_cb_time_ms;
    control_cb_time_ms = now_ms;
    float time_step = time_step_ms / 1000.0;

    compute_movement(time_step);

    if (((now_ms - set_control_time_ms) > STOP_TIMEOUT_MS))
    {
      // sabertooth watchdog must have stopped the motor, set as stopped and skip
      MobilitySkid::stop();
    }
    else
    {
      // reverse kinematics
      float v_diff = LR_WHEELS_OFFSET * target_angular;
      float target_v_lft = target_linear - v_diff;
      float target_v_rgt = target_linear + v_diff;

      float power_lft = pid_lft.compute(target_v_lft, current_v_lft, time_step);
      float power_rgt = pid_rgt.compute(target_v_rgt, current_v_rgt, time_step);

      sabertooth.motor(1, MOTOR_OUT_lft_MULT * (int8_t)power_lft);
      sabertooth.motor(2, MOTOR_OUT_rgt_MULT * (int8_t)power_rgt);
    }

    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(PERIOD_CONTROL_MS));
  }
}

void MobilitySkid::stop()
{
  set_target_velocities(0.0, 0.0);
  pid_lft.reset_errors(0);
  pid_rgt.reset_errors(0);
  sabertooth.stop();
}

// TODO
void MobilitySkid::set_motor_enable(bool enable)
{
}

bool MobilitySkid::setup()
{
  ESP_LOGD(TAG, "Setting up...");

  if (!odom.setup("odom", "tf", "odom", "base_footprint"))
  {
    ESP_LOGE(TAG, "failure initializing odom_helper.");
    return false;
  }

  /* Initialize the motor driver */
  ESP_LOGD(TAG, "setup: sabertooth... ");
  // SABERTOOTH_SERIAL.begin(9600, SERIAL_8N1, -1, SABERTOOTH_TX_PIN);
  sabertooth.init(128, SABERTOOTH_SERIAL, SABERTOOTH_TX_PIN);
  sabertooth.drive(0);
  sabertooth.turn(0);
  sabertooth.setTimeout(STOP_TIMEOUT_MS);

  // ESP32Encoder::useInternalWeakPullResistors = puType::down;
  //  Enable the weak pull up resistors
  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  ESP_LOGD(TAG, "setup: encoders... ");
#if defined(USE_FULL_QUADRATURE)
  encoder_rr_lft.attachFullQuad(ENCODER_rr_lft_PIN_A, ENCODER_rr_lft_PIN_B);
  encoder_fr_lft.attachFullQuad(ENCODER_fr_lft_PIN_A, ENCODER_fr_lft_PIN_B);
  encoder_fr_rgt.attachFullQuad(ENCODER_fr_rgt_PIN_A, ENCODER_fr_rgt_PIN_B);
  encoder_rr_rgt.attachFullQuad(ENCODER_rr_rgt_PIN_A, ENCODER_rr_rgt_PIN_B);
#elif
  encoder_rr_lft.attachSingleEdge(ENCODER_rr_lft_PIN_A, ENCODER_rr_lft_PIN_B);
  encoder_fr_lft.attachSingleEdge(ENCODER_fr_lft_PIN_A, ENCODER_fr_lft_PIN_B);
  encoder_fr_rgt.attachSingleEdge(ENCODER_fr_rgt_PIN_A, ENCODER_fr_rgt_PIN_B);
  encoder_rr_rgt.attachSingleEdge(ENCODER_rr_rgt_PIN_A, ENCODER_rr_rgt_PIN_B);
#endif

  encoder_rr_lft.clearCount();
  encoder_fr_lft.clearCount();
  encoder_fr_rgt.clearCount();
  encoder_rr_rgt.clearCount();
  // last_encoder_us = micros();

  ESP_LOGD(TAG, "setup: control...");

  ESP_LOGI(TAG, "Declaring publisher on [%s]", publisher_joint.topic.name);
  picoros_publisher_declare(&PicoRosso::node, &publisher_joint);

  ESP_LOGI(TAG, "Declaring subscriber on [%s]", subscription_cmd_vel.topic.name);
  picoros_subscriber_declare(&PicoRosso::node, &subscription_cmd_vel);

  control_cb_time_ms = pdMS_TO_TICKS(xTaskGetTickCount());
  // PicoRosso::timer.every(PERIOD_CONTROL_MS, &control_cb);
  xTaskCreate(
      control_task,
      CONTROL_TASK_NAME,
      CONTROL_TASK_STACK_SIZE,
      NULL,
      CONTROL_TASK_PRIORITY,
      NULL);

  // PicoRosso::timer.every(PERIOD_REPORT_MS, &report_cb);
  xTaskCreate(
      report_task,
      REPORT_TASK_NAME,
      REPORT_TASK_STACK_SIZE,
      NULL,
      REPORT_TASK_PRIORITY,
      NULL);

  ESP_LOGD(TAG, "Setting up done.");
  return true;
}
