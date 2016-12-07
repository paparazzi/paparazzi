#include <stdio.h>

extern "C" {
#include "test.h"
}

#include "math.hpp"

using namespace matrix;

template class Vector<float, 5>;

void test_init(void)
{
  // define an euler angle (Body 3(yaw)-2(pitch)-1(roll) rotation)
  float roll = 0.1f;
  float pitch = 0.2f;
  float yaw = 0.3f;
  Eulerf euler(roll, pitch, yaw);


  // convert to quaternion from euler
  Quatf q_nb(euler);
  /*
  // convert to DCM from quaternion
  Dcmf dcm(q_nb);

  // you can assign a rotation instance that already exist to another rotation instance, e.g.
  dcm = euler;

  // you can also directly create a DCM instance from euler angles like this
  dcm = Eulerf(roll, pitch, yaw);
  */

}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
