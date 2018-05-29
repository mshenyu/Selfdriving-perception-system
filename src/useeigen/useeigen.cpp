#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
using Eigen::MatrixXd;
int main()
{
  // Eigen::Matrix<float, 4, 4> m(2,2);
  // m(0,0) = 3;
  // m(1,0) = 2.5;
  // m(0,1) = -1;
  // m(1,1) = m(1,0) + m(0,1);

  Eigen::Matrix4f transform;
  transform  <<  0.953047, -0.00288613, -0.30281, -0.489196,
                0.00405324, 0.999987, 0.00322592, 0.00450199,
                0.302796, -0.00430181, 0.953046, -0.0730386,
                0,			0,			0,			1;
// transform(0,0) = 0.953047;
// transform(0,1) = 

  std::cout << transform << std::endl;
}