#ifndef __ODOM_HELPER_H
#define __ODOM_HELPER_H

#define ODOM_INTERVAL_MS 100

typedef struct
{
  float x;
  float y;
  float phi;
} odom_t;

class OdomHelper
{
public:
  OdomHelper();
  static bool setup(const char *topic_odom = "odom",
                    const char *topic_tf = "tf",
                    const char *frame_id = "odom",
                    const char *child_frame_id = "base_link");

  static void update_pos(float vx, float vy, float vphi, float dt);
  static void set(float x, float y, float phi);
  static void reset();

  static odom_t pos;
  static odom_t vel;
};

#endif // __ODOM_HELPER_H
