#include <iostream>
#include <array>
#include "gtest/gtest.h"

#define private public //not the best practise.  This may cause problems
#include "controller/controller.h"

void setup_control_object(controller::Controller &cntrl);
void test_velocities_vs_expected_velocities(controller::Controller &cntrl, std::array<double,3> expectedV1Velocities);
void test_chasing_velocities_greater_than_boat_velocity(controller::Controller &cntrl);