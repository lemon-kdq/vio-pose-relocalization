#include <vio-pose-relocalization/vio-pose-relocalization.hpp>
typedef VioPoseRelocalization<float>::SE3 SE3F;
int main() {
  VioPoseRelocalization<float> vio_reloc(200,AlignFreedom::DOF6);
  SE3F cur_pose = vio_reloc.GetCurPose();
  std::cout << "cur_pose = \n" << cur_pose.matrix() << std::endl;
  for (size_t i = 0 ; i < 100; i++) {
    SE3F pose = SE3F::Identity();
    vio_reloc.UpdateVioPose(i,pose);
    std::cout << "pose size :" << vio_reloc.RawVioPoseSize() << std::endl;
  }
  float p[7] = {1.,2.,3.,1.,0.,0.,1.0};
  SE3F T_p = vio_reloc.Pose2SE3(p);
  std::cout << " p : \n" <<  T_p.matrix() << std::endl;
  vio_reloc.UpdateRelocalizePose(0.,T_p,RelocalizeStatus::OK);
  std::map<double,SE3F> err_pose = vio_reloc.GetErrPose();
  for(auto iter = err_pose.begin(); iter != err_pose.end(); iter++) {
    std::cout << iter->second.matrix() << std::endl;
    std::cout << "euler : " << iter->second.rotation().eulerAngles(2,1,0) * 57.3 << std::endl;
  }
  return 0;
}