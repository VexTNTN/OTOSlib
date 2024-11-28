#include "main.h"
#include <cstdint>
#include <cstring>
#include <string>

#include "OTOS.hpp"

OTOS otos(1);

void initialize()
{
  otos.calibrate();
  otos.setOffset({2.75, -7.5, 0});
  otos.setAngularScaler(0.923076923);
  otos.setLinearScaler(0.988739357);
  otos.resetTracking();
  otos.setPose({0, 0, 0});
}

void opcontrol()
{
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  while (true)
  {
    auto pose = otos.getPose();

    printf("x:%f y:%f theta:%f \n", pose.x, pose.y, pose.theta);

    pros::delay(10);
  }
}