#pragma once
#include <cstring>

struct Pose {
  float x;
  float y;
  float theta;
};

/**
 * @brief A class to interface with the OTOS sensor
 */
class OTOS {
public:
  /**
   * @brief Construct a new smart OTOS sensor object
   *
   * @param port
   */
  OTOS(int port);

  /**
   * @brief Destroy the smart OTOS sensor object
   */
  ~OTOS();

  /**
   * @brief Initialize and Calibrate the sensor (MUST BE CALLED BEFORE ANY OTHER
   * FUNCTION). Also blocking for 20ms
   */
  void calibrate();

  /**
   * @brief Reset the tracking of the sensor
   */
  void resetTracking();

  /**
   * @brief Get the Pose object
   *
   * @return Pose
   */
  Pose getPose();

  /**
   * @brief Set the Pose object
   *
   * @param pose
   */
  void setPose(Pose pose);

  /**
   * @brief Sets the offset of the sensor from the robot's rotation center
   *
   * @param pose
   */
  void setOffset(Pose pose);

  /**
   * @brief Sets the linear scale of the sensr (use this to tune the sensors
   * performance)
   *
   * @param scaler
   */
  void setLinearScaler(float scaler);

  /**
   * @brief Sets the angular scale of the sensor (use this to tune the sensors
   * performance)
   *
   * @param scaler
   */
  void setAngularScaler(float scaler);

  // Only true once good data has been read from the sensor
  bool isCalibrated = false;

private:
  /**
   * @brief Get the Data object
   *
   * @param buffer
   * @param size
   * @return int
   */
  int getData(char *buffer, int size);

  /**
   * @brief Send a command to the sensor
   *
   * @param command
   * @param data
   * @param length
   */
  void sendCommand(char command, void *data, int length);

  pros::Serial *serial;
  Pose lastRead;
  const int port;
};