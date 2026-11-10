#ifndef RESCUE_ROBOT_CONFIG_H_
#define RESCUE_ROBOT_CONFIG_H_

namespace robot_config {

// Validated Webots device names and sensor response profiles.
constexpr const char *kLeftMotorName = "left wheel motor";
constexpr const char *kRightMotorName = "right wheel motor";
constexpr const char *kLeftEncoderName = "left wheel sensor";
constexpr const char *kRightEncoderName = "right wheel sensor";
constexpr const char *kCompassName = "compass";
constexpr const char *kGpsName = "gps";
constexpr const char *kFrontCameraName = "camera_f";
constexpr const char *kSphericalCameraName = "camera_s";

constexpr int kDistanceSensorCount = 16;
constexpr double kFrontLaserProfileSaturationResponse = 1020.0;
constexpr double kOtherLaserProfilesSaturationResponse = 1024.0;
constexpr double kFrontLaserProfileNearSaturationM = 0.08;
constexpr double kOtherLaserProfilesNearSaturationM = 0.2;
constexpr double kDistanceSensorLongRangeM = 1.0;
constexpr double kDistanceSensorShortRangeM = 0.4;
constexpr double kMaximumWheelSpeedRadPerSec = 10.0;
constexpr double kWheelRadiusM = 0.0825;
constexpr double kWheelSeparationM = 0.3606;

constexpr bool isLongRangeDistanceSensor(int index) {
  return index == 3 || index == 12;
}
constexpr bool usesFrontLaserLookupProfile(int index) {
  return (index >= 0 && index <= 2) || (index >= 13 && index <= 15);
}
constexpr double distanceSensorMaximumRangeM(int index) {
  return isLongRangeDistanceSensor(index) ? kDistanceSensorLongRangeM
                                          : kDistanceSensorShortRangeM;
}
constexpr double distanceSensorNearSaturationM(int index) {
  return usesFrontLaserLookupProfile(index)
             ? kFrontLaserProfileNearSaturationM
             : kOtherLaserProfilesNearSaturationM;
}
constexpr double distanceSensorSaturationResponse(int index) {
  return usesFrontLaserLookupProfile(index)
             ? kFrontLaserProfileSaturationResponse
             : kOtherLaserProfilesSaturationResponse;
}

}  // namespace robot_config
#endif
