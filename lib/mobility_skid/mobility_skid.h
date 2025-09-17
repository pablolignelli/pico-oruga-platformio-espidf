#ifndef __MOBILITY_SKID_H
#define __MOBILITY_SKID_H

class MobilitySkid
{
public:
  MobilitySkid();

  static bool setup();

  static void set_motor_enable(bool enable); // TODO
  static void stop();
};

#endif // __MOBILITY_SKID_H
