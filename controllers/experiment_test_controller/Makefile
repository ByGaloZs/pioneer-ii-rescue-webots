CXX_SOURCES = experiment_test_controller.cpp \
              ../fool_controller/sensors/SensorManager.cpp \
              ../fool_controller/sensors/DistanceSensorCalibration.cpp \
              ../fool_controller/control/MotorController.cpp \
              ../../experiments/exp_002_visual_ranging/src/Exp002Logger.cpp

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
