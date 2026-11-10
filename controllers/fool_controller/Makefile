CXX_SOURCES = fool_controller.cpp \
              sensors/SensorManager.cpp \
              sensors/DistanceSensorCalibration.cpp \
              control/MotorController.cpp \
              ../../experiments/exp_001_sensor_validation/src/Exp001Logger.cpp

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
