#include <iostream>
#include <array>
#include "gtest/gtest.h"

#define private public //not the best practise.  This may cause problems
#include "controller/controller.h"

void setup_control_object(controller::Controller &cntrl);