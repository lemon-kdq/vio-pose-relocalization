#include <iostream>
#include <eigen3/Eigen/Dense>
#include <map>

enum RelocalizeStatus {
 OK,
 Bad
};

enum AlignFreedom {
  DOF3,    //只对齐位置
  DOF4,    //对齐位置和航向
  DOF6     //对齐所有自由度
};




template <typename T>
class VioPoseRelocalization {
 public:
  typedef Eigen::Transform<T,3,Eigen::Isometry> SE3;
  typedef Eigen::Vector3<T> V3;
  typedef Eigen::Quaternion<T> SO3;
  VioPoseRelocalization(double max_allowed_timestamp_s,AlignFreedom aligned_dof) 
        : max_allowed_timestamp_s_(max_allowed_timestamp_s),
          algin_dof_(aligned_dof) {
    aligned_Twc_.setIdentity();
  }

  SE3 GetCurPose() const {
    return aligned_Twc_;
  }
  
  void UpdateRelocalizePose(double timestamp_s,const SE3& reloc_Twc,
        RelocalizeStatus status) {
    if (MatchedVioPose(timestamp_s)) {
     SE3 err_Tww = CalculateReloPoseErr(timestamp_s,reloc_Twc);
     err_Tww_[timestamp_s] = err_Tww;
    }
  }

  void UpdateVioPose(double timestamp_s,const SE3& pose) {
    vio_Twc_[timestamp_s] = pose;
    if (!err_Tww_.empty()) {
      SE3 err_Tww = err_Tww_.rbegin()->second;
      aligned_Twc_ = err_Tww * pose;
    } else {
      aligned_Twc_ = pose;
    }
    while (!vio_Twc_.empty()) {
      double oldest_timestamp_s = vio_Twc_.begin()->first;
      if ((timestamp_s - oldest_timestamp_s) > max_allowed_timestamp_s_) {
        vio_Twc_.erase(vio_Twc_.begin());
      } else {
        break;
      }
    }
  }
  size_t RawVioPoseSize() const {
    return vio_Twc_.size();
  }

  SE3 Pose2SE3(const T pose[]) {
    SE3 T_pose;
    T_pose.setIdentity();
    V3 t_pose(pose[0],pose[1],pose[2]);
    SO3 q_pose(pose[3],pose[4],pose[5],pose[6]);
    q_pose.normalize();
    T_pose.pretranslate(t_pose);
    T_pose.rotate(q_pose);
    return T_pose;
  }

  std::map<double,SE3>& GetErrPose() {
    return err_Tww_;
  }

 private:
  AlignFreedom algin_dof_;
  SE3 aligned_Twc_;
  std::map<double,SE3> err_Tww_;
  std::map<double,SE3> vio_Twc_;   //raw Vio Pose : Camera Frame -> World Frame
  double max_allowed_timestamp_s_;
 private:

  
  //reloc_Twc * vio_Tcw = Tww'
  SE3 CalculateReloPoseErr(double timestamp_s,const SE3& reloc_Twc) {
    SE3 err_pose;
    err_pose.setIdentity();
    SE3 vio_Twc = vio_Twc_[timestamp_s];
    switch (algin_dof_)
    {
    case DOF3:
      {
        err_pose.translation() = reloc_Twc.translation() - vio_Twc.translation();
      }
      break;
    case DOF4:
      {
        
        V3 reloc_Twc_euler = SE3ToEuler(reloc_Twc);
        Eigen::Matrix3<T>  reloc_tmp;
        T yaw = reloc_Twc_euler[0];
        reloc_tmp << cos(yaw),-sin(yaw),0.,sin(yaw),cos(yaw),0.,0.0,0.0,1.0;
        SE3 reloc_Twc_yaw;
        reloc_Twc_yaw.setIdentity();
        reloc_Twc_yaw.pretranslate(reloc_Twc.translation());
        reloc_Twc_yaw.rotate(reloc_tmp);

        V3 vio_Twc_euler = SE3ToEuler(vio_Twc);
        Eigen::Matrix3<T>  vio_tmp;
        yaw = vio_Twc_euler[0];
        vio_tmp << cos(yaw),-sin(yaw),0.,sin(yaw),cos(yaw),0.,0.0,0.0,1.0;
        SE3 vio_Twc_yaw;
        vio_Twc_yaw.setIdentity();
        vio_Twc_yaw.pretranslate(vio_Twc.translation());
        vio_Twc_yaw.rotate(vio_tmp);
        
        err_pose = reloc_Twc_yaw * vio_Twc_yaw.inverse();
      }
      break;
    case DOF6:
      {
        err_pose = reloc_Twc * vio_Twc.inverse();
      }
      break;
    default:
      break;
    }
    return err_pose;
  }

  bool MatchedVioPose(double reloc_timestamp) {
    if (vio_Twc_.count(reloc_timestamp)) {
      return true;
    } else {
      return false;
    }
  }
 
 V3 SE3ToEuler(const SE3& pose) {
   return pose.rotation().eulerAngles(2,1,0);
 }
  
};