#pragma once
#include <cstring>

struct Pose
{
  float x;
  float y;
  float theta;
};

class OTOS
{
public:
  OTOS(int port);
  ~OTOS();
  void calibrate();
  void resetTracking();
  Pose getPose();
  void setPose(Pose pose);
  void setOffset(Pose pose);
  void setLinearScaler(float scaler);
  void setAngularScaler(float scaler);

private:
  int getData(char *buffer, int size);
  void sendCommand(char command, void *data, int length);
  pros::Serial *serial;
  Pose lastRead;
};