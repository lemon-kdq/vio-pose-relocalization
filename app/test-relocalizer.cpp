#include <vio-pose-relocalization/vio-pose-relocalization.hpp>
typedef VioPoseRelocalization::SE3 SE3F;
typedef VioPoseRelocalization::SO3 SO3F;
typedef VioPoseRelocalization::V3 V3F;

int main() {
  VioPoseRelocalization::GetInstance().RunUnitTest();

  for(size_t i = 0; i < 100; i++) {
    VioPoseRelocalization::GetInstance().UpdateVioPose(i,SE3F::Identity());
  
    if ( i > 0 && i % 10 == 0) {
      SE3F reloc_pose = SE3F::Identity();
      SO3F quat;
      quat =  Eigen::AngleAxisf(0.03, Eigen::Vector3f::UnitZ())
                       * Eigen::AngleAxisf(0., Eigen::Vector3f::UnitY())
                       * Eigen::AngleAxisf(0., Eigen::Vector3f::UnitX());
      V3F t(i*0.1,0.,0.);
      reloc_pose.translation() = t;
      reloc_pose.linear() = quat.toRotationMatrix();
      std::cout << i << " - reloc-pose = \n" << reloc_pose.matrix() << std::endl;

      VioPoseRelocalization::GetInstance().UpdateRelocalizePose(i,reloc_pose);
    }
    SE3F fuse_pose = VioPoseRelocalization::GetInstance().GetCurPose();
    std::cout << i << " - pose = \n" << fuse_pose.matrix() << std::endl;
  }


  return 0;
}