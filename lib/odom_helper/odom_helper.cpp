#include <esp_log.h>
#include "picorosso.h"
#include "odom_helper.h"
#include <cmath>

#define TSK_MINIMAL_STACK_SIZE (1024)
#define ODOM_TASK_NAME "odom_helper_task"
#define ODOM_TASK_STACK_SIZE (TSK_MINIMAL_STACK_SIZE * 8)
#define ODOM_TASK_PRIORITY (tskIDLE_PRIORITY + 2)

static const char *TAG = "odom";

odom_t OdomHelper::pos;
odom_t OdomHelper::vel;

// Odometry message and publisher
static ros_Odometry msg_odom;
picoros_publisher_t publisher_odom = {
    .topic = {
        .name = NULL,
        .type = ROSTYPE_NAME(ros_Odometry),
        .rihs_hash = ROSTYPE_HASH(ros_Odometry),
    },
};

static ros_TransformStamped msg_transform;
static ros_TFMessage msg_tf = {
    .transforms = {
        .data = &msg_transform,
        .n_elements = 1,
    },
};
picoros_publisher_t publisher_tf = {
    .topic = {
        .name = NULL,
        .type = ROSTYPE_NAME(ros_TFMessage),
        .rihs_hash = ROSTYPE_HASH(ros_TFMessage),
    },
};

static void euler_to_quat(float roll, float pitch, float yaw, float *q)
{
  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);

  q[0] = cy * cp * cr + sy * sp * sr;
  q[1] = cy * cp * sr - sy * sp * cr;
  q[2] = sy * cp * sr + cy * sp * cr;
  q[3] = sy * cp * cr - cy * sp * sr;
}

OdomHelper::OdomHelper()
{
  // we don't change these anymore
  msg_odom.pose.covariance[0] = 0.0001;
  msg_odom.pose.covariance[7] = 0.0001;
  msg_odom.pose.covariance[35] = 0.0001;

  msg_odom.twist.covariance[0] = 0.0001;
  msg_odom.twist.covariance[7] = 0.0001;
  msg_odom.twist.covariance[35] = 0.0001;
}

void OdomHelper::set(float x, float y, float phi)
{
  OdomHelper::pos.x = x;
  OdomHelper::pos.y = y;
  OdomHelper::pos.phi = phi;
}

void OdomHelper::reset()
{
  set(0.0, 0.0, 0.0);
}

void OdomHelper::update_pos(float vx, float vy, float vphi, float dt)
{
  OdomHelper::vel.x = vx;
  OdomHelper::vel.y = vy;
  OdomHelper::vel.phi = vphi;

  OdomHelper::pos.phi += vphi * dt;
  float phi_y = OdomHelper::pos.phi + M_PI_2; // pi/2

  float dx = vx * dt;
  float dy = vy * dt;

  OdomHelper::pos.x += cos(OdomHelper::pos.phi) * dx;
  OdomHelper::pos.y += sin(OdomHelper::pos.phi) * dx;
  OdomHelper::pos.x += cos(phi_y) * dy;
  OdomHelper::pos.y += sin(phi_y) * dy;
}

static void odom_helper_task(void *pvParameters)
{
  TickType_t last_wake_time = xTaskGetTickCount();

  for (;;)
  {
    z_clock_t now = z_clock_now();

    // robot's position in x,y,phi
    msg_odom.pose.pose.position.x = OdomHelper::pos.x;
    msg_odom.pose.pose.position.y = OdomHelper::pos.y;

    // calculate robot's heading in quaternion angle
    // ROS has a function to calculate yaw in quaternion angle
    float q[4];
    euler_to_quat(0, 0, OdomHelper::pos.phi, q);
    // robot's heading in quaternion
    msg_odom.pose.pose.orientation.x = q[1];
    msg_odom.pose.pose.orientation.y = q[2];
    msg_odom.pose.pose.orientation.z = q[3];
    msg_odom.pose.pose.orientation.w = q[0];

    // speed from encoders
    msg_odom.twist.twist.linear.x = OdomHelper::vel.x;
    msg_odom.twist.twist.linear.y = OdomHelper::vel.y;
    msg_odom.twist.twist.angular.z = OdomHelper::vel.phi;

    //msg_odom.header.stamp.sec = (int32_t)now.tv_sec;
    //msg_odom.header.stamp.nanosec = (uint32_t)now.tv_nsec;
    PicoRosso::set_timestamp(msg_odom.header.stamp, now);

    pr_publish(publisher_odom, msg_odom);

    msg_transform.transform.translation.x = msg_odom.pose.pose.position.x;
    msg_transform.transform.translation.y = msg_odom.pose.pose.position.y;
    msg_transform.transform.translation.z = msg_odom.pose.pose.position.z;
    msg_transform.transform.rotation = msg_odom.pose.pose.orientation;

    //msg_transform.header.stamp.sec = (int32_t)now.tv_sec;
    //msg_transform.header.stamp.nanosec = (uint32_t)now.tv_nsec;
    PicoRosso::set_timestamp(msg_odom.header.stamp, now);

    // RCNOCHECK(rcl_publish(&pdescriptor_tf.publisher, &msg_tf, NULL));
    pr_publish(publisher_tf, msg_tf);

    // pause the task per defined wait period
    vTaskDelayUntil(&last_wake_time, ODOM_INTERVAL_MS / portTICK_PERIOD_MS);
  }
}

bool OdomHelper::setup(const char *topic_odom,
                       const char *topic_tf,
                       const char *frame_id,
                       const char *child_frame_id)
{
  ESP_LOGD(TAG, "Setting up...");

  ESP_LOGI(TAG, "Declaring publisher on [%s]", topic_odom);
  publisher_odom.topic.name = topic_odom;
  msg_odom.header.frame_id = (char *)frame_id;
  msg_odom.child_frame_id = (char *)child_frame_id;
  picoros_publisher_declare(&PicoRosso::node, &publisher_odom);

  ESP_LOGI(TAG, "Declaring publisher on [%s]", topic_tf);
  publisher_tf.topic.name = (char *)topic_tf;
  msg_transform.header.frame_id = (char *)frame_id;
  msg_transform.child_frame_id = (char *)child_frame_id;
  picoros_publisher_declare(&PicoRosso::node, &publisher_tf);

  //PicoRosso::timer.every(ODOM_INTERVAL_MS, &report_cb);
  xTaskCreate(
      odom_helper_task,
      ODOM_TASK_NAME,
      ODOM_TASK_STACK_SIZE,
      NULL,
      ODOM_TASK_PRIORITY,
      NULL);

  ESP_LOGD(TAG, "Setting up done.");
  return true;
}
