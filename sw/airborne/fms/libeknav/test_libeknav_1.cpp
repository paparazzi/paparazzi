

#include <iostream>
#include <iomanip>

#include <Eigen/Core>

// import most common Eigen types 
USING_PART_OF_NAMESPACE_EIGEN

int main(int, char *[]) {

  std::cout << "hello world" << std::endl;

  Matrix3f m3;
  m3 << 1, 2, 3, 4, 5, 6, 7, 8, 9;

  return 0;

}

