#include "iostream"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "map"
#include "vector"
#define POSE_THRESHOLD 0.3
#define YAW_THRESHOLD 5.0
#define RAD2DEG 57.29578
enum RelocalizeStatus {
    OK,
    Bad
};

enum AlignFreedom {
    DOF3,    //只对齐位置
    DOF4,    //对齐位置和航向
    DOF6     //对齐所有自由度
};




class VioPoseRelocalization {
public:
    typedef Eigen::Isometry3f SE3;
    typedef Eigen::Vector3f V3;
    typedef Eigen::Quaternionf SO3;
    typedef Eigen::Matrix3f SO3Mat;

    class SE3Slerp {
     public:
      SE3Slerp() {
        SE3 s_se3 = SE3::Identity();
        SE3 t_se3 = SE3::Identity();
        reset(s_se3,t_se3,10);
      }
      SE3Slerp(SE3 s_se3,SE3 t_se3,int num_step) {
        reset(s_se3,t_se3,num_step);
      }
      void reset(SE3 s_se3,SE3 t_se3,int num_step) {
        se3_src_ = s_se3;
        se3_tar_ = t_se3;
        num_step_ = num_step;
        count_ = 1;
      }
      SE3 GetSlerpPose() {
        float alpha = float(count_) / num_step_;
        if (count_ < num_step_)
          count_++;
        SE3 T_interp = GetSlerpPose(se3_src_,se3_tar_,alpha);
        return T_interp;
      }

      static SE3 GetSlerpPose(SE3 se3_src,SE3 se3_tar,float alpha) {
        V3 t_interp = (1.0 - alpha) * se3_src.translation() + alpha * se3_tar.translation();
        SO3 q_src(se3_src.linear());
        SO3 q_tar(se3_tar.linear());
        SO3 q_interp = q_src.slerp(alpha,q_tar);
        SE3 T_interp;
        T_interp.setIdentity();
        T_interp.translation() = t_interp;
        T_interp.linear() = q_interp.toRotationMatrix();
        return T_interp;
      }
     private:
      SE3 se3_src_;
      SE3 se3_tar_;
      int num_step_;
      int count_;
    };

    static VioPoseRelocalization& GetInstance() {
      static VioPoseRelocalization instance;
      return instance;
    }

    void InitParameters(double max_allowed_timestamp_s,AlignFreedom aligned_dof,int slide_window_size = 3,int interp_steps = 3) {
      max_allowed_timestamp_s_ = max_allowed_timestamp_s;
      aligned_dof_ = aligned_dof;
      slide_window_size_ = slide_window_size;
      interp_steps_ = interp_steps;
    }

    SE3 GetCurPose() const {
      return aligned_Twc_;
    }

    void UpdateRelocalizePose(double timestamp_s,const SE3& reloc_Twc,
                              RelocalizeStatus status = RelocalizeStatus::OK) {
      if (MatchedVioPose(timestamp_s)) {
        SE3 err_Tww = CalculateReloPoseErr(timestamp_s,reloc_Twc);
        err_Tww_.push_back(err_Tww);
        last_target_err_Tww_ = target_err_Tww_;
        bool same_flg = false;
        target_err_Tww_ = SlideWindowFilter(err_Tww_,target_err_Tww_,slide_window_size_,same_flg);
        if (!same_flg) {
          se3_slerper_.reset(last_target_err_Tww_,target_err_Tww_,interp_steps_);
        }
      }
    }

    void UpdateVioPose(double timestamp_s,const SE3& pose) {
      vio_Twc_[timestamp_s] = pose;
      if (!err_Tww_.empty()) {
        SE3 err_Tww = se3_slerper_.GetSlerpPose();
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

    void SE3toFloatArray(SE3 pose,float pose_array[]) const {
      pose_array[0] = pose.translation()[0];
      pose_array[1] = pose.translation()[1];
      pose_array[2] = pose.translation()[2];
      Eigen::Matrix3f R = pose.rotation();
      Eigen::Quaternionf quat(R);
      pose_array[3] = quat.w();
      pose_array[4] = quat.x();
      pose_array[5] = quat.y();
      pose_array[6] = quat.z();
    }

    SE3 Pose2SE3(const float pose[]) const {
      SE3 T_pose;
      T_pose.setIdentity();
      V3 t_pose(pose[0],pose[1],pose[2]);
      SO3 q_pose(pose[3],pose[4],pose[5],pose[6]);
      q_pose.normalize();
      T_pose.pretranslate(t_pose);
      T_pose.rotate(q_pose);
      return T_pose;
    }

    SE3 Array2SE3(const std::array<float,16>& pose) const {
      SE3 T_pose;
      T_pose.setIdentity();
      V3 t_pose(pose[3],pose[7],pose[11]);
      SO3Mat R_pose;
      R_pose << pose[0],pose[1],pose[2],
              pose[4],pose[5],pose[6],
              pose[8],pose[9],pose[10];
      T_pose.pretranslate(t_pose);
      T_pose.rotate(R_pose);
      return T_pose;
    }

    std::vector<SE3>& GetErrPose() {
      return err_Tww_;
    }

    void RunUnitTest() {
      RunSE3Slerp();
      RunSE3Average();
    }

    static V3 SE3ToEuler(const SE3& pose) {
      return pose.rotation().eulerAngles(2,1,0);
    }
   
    static SE3 CalculateSE3Error(SE3 src_pose,SE3 tar_pose,AlignFreedom aligned_dof) {
      SE3 err_pose;
      err_pose.setIdentity();
      switch (aligned_dof)
      {
        case DOF3:
        {
          err_pose.translation() = tar_pose.translation() - src_pose.translation();
        }
        break;
        case DOF4:
        {

          V3 reloc_Twc_euler = SE3ToEuler(tar_pose);
          Eigen::Matrix3f  reloc_tmp;
          float yaw = reloc_Twc_euler[0];
          reloc_tmp << cos(yaw),-sin(yaw),0.,sin(yaw),cos(yaw),0.,0.0,0.0,1.0;
          SE3 reloc_Twc_yaw;
          reloc_Twc_yaw.setIdentity();
          reloc_Twc_yaw.pretranslate(tar_pose.translation());
          reloc_Twc_yaw.rotate(reloc_tmp);

          V3 vio_Twc_euler = SE3ToEuler(src_pose);
          Eigen::Matrix3f  vio_tmp;
          yaw = vio_Twc_euler[0];
          vio_tmp << cos(yaw),-sin(yaw),0.,sin(yaw),cos(yaw),0.,0.0,0.0,1.0;
          SE3 vio_Twc_yaw;
          vio_Twc_yaw.setIdentity();
          vio_Twc_yaw.pretranslate(src_pose.translation());
          vio_Twc_yaw.rotate(vio_tmp);
          err_pose = reloc_Twc_yaw * vio_Twc_yaw.inverse();
        }
        break;
        case DOF6:
        {
          err_pose = tar_pose * src_pose.inverse();
        }
        break;
        default:
          break;
      }
      return err_pose;
    }

private:
    VioPoseRelocalization(double max_allowed_timestamp_s = 10.,AlignFreedom aligned_dof = DOF4,int slide_window_size = 3,int interp_steps = 3)
            : max_allowed_timestamp_s_(max_allowed_timestamp_s),
              aligned_dof_(aligned_dof),
              slide_window_size_(slide_window_size),
              interp_steps_(interp_steps) {
      aligned_Twc_.setIdentity();
      target_err_Tww_.setIdentity();
      last_target_err_Tww_.setIdentity();
    }

    VioPoseRelocalization(const VioPoseRelocalization&) = delete;
    VioPoseRelocalization& operator=(const VioPoseRelocalization&) = delete;

    AlignFreedom aligned_dof_;
    SE3 aligned_Twc_;
    std::vector<SE3> err_Tww_;
    SE3 target_err_Tww_,last_target_err_Tww_;
    std::map<double,SE3> vio_Twc_;   //raw Vio Pose : Camera Frame -> World Frame
    double max_allowed_timestamp_s_;
    SE3Slerp se3_slerper_;
    int slide_window_size_,interp_steps_;
private:


    //reloc_Twc * vio_Tcw = Tww'
    SE3 CalculateReloPoseErr(double timestamp_s,const SE3& reloc_Twc) {
      SE3 vio_Twc = vio_Twc_[timestamp_s];
      return CalculateSE3Error(vio_Twc,reloc_Twc,aligned_dof_);
    }

    bool MatchedVioPose(double reloc_timestamp) {
      if (vio_Twc_.count(reloc_timestamp)) {
        return true;
      } else {
        return false;
      }
    }



    SE3 SlideWindowFilter(std::vector<SE3>& history_err_Tww,SE3 last_target_err_Tww,int window_size,bool& same_flg) {
      SE3 cur_target_err_Tww;
      int total_size = 0;
      while(history_err_Tww.size() > window_size) {
        history_err_Tww.erase(history_err_Tww.begin());
      }
      SE3 cur_pose = history_err_Tww[0];
      for(size_t i = 1; i < history_err_Tww.size(); i++) {
        cur_pose = SE3Slerp::GetSlerpPose(cur_pose,history_err_Tww[i],0.5);
      }
      SE3 err_pose = CalculateSE3Error(last_target_err_Tww,cur_pose,aligned_dof_);
      V3 euler = SE3ToEuler(err_pose);
      float yaw = std::fabs(euler[0]) * RAD2DEG;
      if (yaw > YAW_THRESHOLD || err_pose.translation().norm() > POSE_THRESHOLD) {
        same_flg = false;
        return cur_pose;
      } else {
        same_flg = true;
        return last_target_err_Tww;
      }
    }


    void RunSE3Average() {

      SE3 T1 = SE3::Identity();
      SE3 T2 = SE3::Identity();

      // 设置T1和T2的平移部分
      T1.translation() << 1.0, 2.0, 3.0;
      T2.translation() << 4.0, 5.0, 6.0;

      // 设置T1和T2的旋转部分，例如使用旋转矩阵
      SO3 rotation_matrix1;
      rotation_matrix1 = Eigen::AngleAxisf(0.1, Eigen::Vector3f::UnitX())
                       * Eigen::AngleAxisf(0.2, Eigen::Vector3f::UnitY())
                       * Eigen::AngleAxisf(0.3, Eigen::Vector3f::UnitZ());
      T1.linear() = rotation_matrix1.toRotationMatrix();

      SO3 rotation_matrix2;
      rotation_matrix2 = Eigen::AngleAxisf(0.4, Eigen::Vector3f::UnitX())
                       * Eigen::AngleAxisf(0.5, Eigen::Vector3f::UnitY())
                       * Eigen::AngleAxisf(0.6, Eigen::Vector3f::UnitZ());
      T2.linear() = rotation_matrix2.toRotationMatrix();
      
      SE3 T = SE3Slerp::GetSlerpPose(T1,T2,0.5);
      std::cout << T.translation().transpose() << std::endl;
      V3 euler = T.linear().eulerAngles(0,1,2);
      std::cout << euler.transpose() << std::endl;

  }

    void RunSE3Slerp() {

      SE3 T1 = SE3::Identity();
      SE3 T2 = SE3::Identity();

      // 设置T1和T2的平移部分
      T1.translation() << 1.0, 2.0, 3.0;
      T2.translation() << 4.0, 5.0, 6.0;

      // 设置T1和T2的旋转部分，例如使用旋转矩阵
      SO3 rotation_matrix1;
      rotation_matrix1 = Eigen::AngleAxisf(0.1, Eigen::Vector3f::UnitX())
                       * Eigen::AngleAxisf(0.2, Eigen::Vector3f::UnitY())
                       * Eigen::AngleAxisf(0.3, Eigen::Vector3f::UnitZ());
      T1.linear() = rotation_matrix1.toRotationMatrix();

      SO3 rotation_matrix2;
      rotation_matrix2 = Eigen::AngleAxisf(0.4, Eigen::Vector3f::UnitX())
                       * Eigen::AngleAxisf(0.5, Eigen::Vector3f::UnitY())
                       * Eigen::AngleAxisf(0.6, Eigen::Vector3f::UnitZ());
      T2.linear() = rotation_matrix2.toRotationMatrix();
      SE3Slerp se3_slerper(T1,T2,10);

      for(size_t i = 0; i < 11; i++)
      {
        SE3 T_interp = se3_slerper.GetSlerpPose();
        std::cout << T_interp.matrix() << std::endl;
        V3 euler = T_interp.linear().eulerAngles(0,1,2);
        std::cout << euler.transpose() << std::endl;
      }


  }
};