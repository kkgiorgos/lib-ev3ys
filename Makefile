THIS_LIB_NAME := lib-ev3ys
THIS_LIB_OBJS := 
THIS_LIB_CXXOBJS := ev3cxx.o ev3cxx_time.o ev3ys_timer.o ev3ys_motor.o ev3ys_math.o ev3ys_motor_trajectory.o ev3ys_motor_controller.o ev3ys_chassis.o ev3ys_lineFollower.o

include $(EV3RT_SDK_LIB_DIR)/../Makefile.slib
