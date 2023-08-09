#include <eigen3/Eigen/Dense>
#include <iostream>
int main() {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    std::cout << pose.matrix() << std::endl;
    return 0;
}