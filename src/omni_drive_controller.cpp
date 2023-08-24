#include <cmath>
#include <omni_drive_controller/omni_drive_controller.h>
#include <tf/transform_datatypes.h>
#include <urdf/urdfdom_compatibility.h>
#include <urdf_parser/urdf_parser.h>
#include <eigen3/Eigen/Dense>

namespace omni_drive_controller{

    OmniDriveController::OmniDriveController()
    : open_loop_(false),
      base_frame_id_("base_footprint"),
      odom_frame_id_("odom"),
      enable_odom_tf_(true),
      publish_cmd_(false),
      publish_wheel_joint_controller_state_(false),
      wheel_joints_size_(0),
      robot_x_(0.0),
      robot_y_(0.0),
      robot_theta_(0.0)
    {

    }

    bool OmniDriveController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh)
    {
        const std::string complete_ns = controller_nh.getNamespace();
        std::size_t id = complete_ns.find_last_of("/");
        name_ = complete_ns.substr(id + 1);


        wheel_joints_size_ = 3;
        std::vector<std::string> wheel_names;
        wheel_names.resize(wheel_joints_size_,  "");
        wheel_joints_.resize(wheel_joints_size_);


        std::string wheel_name;
        if(!controller_nh.getParam("wheel1", wheel_names[0]))
        {
            ROS_ERROR_STREAM_NAMED(name_, "can't find the paramset param for wheel1.");
            return false;
        }
        if(!controller_nh.getParam("wheel2", wheel_names[1]))
        {
            ROS_ERROR_STREAM_NAMED(name_, "set param for wheel2.");
            return false;
        }
        if(!controller_nh.getParam("wheel3", wheel_names[2]))
        {
            ROS_ERROR_STREAM_NAMED(name_, "set param for wheel3.");
            return false;
        }
        ROS_INFO_NAMED(name_, "Find 3 wheels from configuration and parameter [%s, %s, %s].", wheel_names[0].c_str(), wheel_names[1].c_str(), wheel_names[2].c_str());

        // Odometry
        double publish_rate;
        controller_nh.param("publish_rate", publish_rate, 50.0);
        ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at " << publish_rate << "Hz.");
        publish_period_ = ros::Duration(1.0 / publish_rate);

        controller_nh.param("open_loop", open_loop_, open_loop_);


        controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);
        ROS_INFO_STREAM_NAMED(name_, "Velocity commands will be considered old if they are older than " << cmd_vel_timeout_ << "s.");

        controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
        ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

        controller_nh.param("odom_frame_id", odom_frame_id_, odom_frame_id_);
        ROS_INFO_STREAM_NAMED(name_, "Odometry frame_id set to " << odom_frame_id_);

        controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
        ROS_INFO_STREAM_NAMED(name_, "Publishing to tf is " << (enable_odom_tf_?"enabled":"disabled"));


        controller_nh.param("publish_cmd", publish_cmd_, publish_cmd_);
        controller_nh.param("publish_wheel_joint_controller_state", publish_wheel_joint_controller_state_, publish_wheel_joint_controller_state_);

        controller_nh.getParam("wheel_separation", wheel_separation_);
        controller_nh.getParam("wheel_radius", wheel_radius_);

        ROS_INFO_STREAM_NAMED(name_, "Odometry params : wheel separation " << wheel_separation_ << ", wheel radius "  << wheel_radius_);
        setOdomPubFields(root_nh, controller_nh);

        if(publish_cmd_)
        {
            cmd_vel_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>(controller_nh, "cmd_vel_out", 100));
        }

        if(publish_wheel_joint_controller_state_)
        {
            controller_state_pub_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointTrajectoryControllerState>(controller_nh, "wheel_joint_controller_state", 100));
            const size_t num_wheels = wheel_names.size();

            controller_state_pub_->msg_.joint_names.resize(num_wheels);

            controller_state_pub_->msg_.desired.positions.resize(num_wheels);
            controller_state_pub_->msg_.desired.velocities.resize(num_wheels);
            controller_state_pub_->msg_.desired.accelerations.resize(num_wheels);
            controller_state_pub_->msg_.desired.effort.resize(num_wheels);

            controller_state_pub_->msg_.actual.positions.resize(num_wheels);
            controller_state_pub_->msg_.actual.velocities.resize(num_wheels);
            controller_state_pub_->msg_.actual.accelerations.resize(num_wheels);
            controller_state_pub_->msg_.actual.effort.resize(num_wheels);

            controller_state_pub_->msg_.error.positions.resize(num_wheels);
            controller_state_pub_->msg_.error.velocities.resize(num_wheels);
            controller_state_pub_->msg_.error.accelerations.resize(num_wheels);
            controller_state_pub_->msg_.error.effort.resize(num_wheels);

            for (size_t i = 0; i < num_wheels; ++i)
            {
                controller_state_pub_->msg_.joint_names[i] = wheel_names[i];
            }

            vel_wheel_previous_.resize(num_wheels, 0.0);
        }

        // Get the joint object to use in the realtime loop
        for (size_t i = 0; i < wheel_joints_size_; ++i)
        {
            wheel_joints_[i] = hw->getHandle(wheel_names[i]);
            ROS_INFO_STREAM_NAMED(name_, "Adding wheel with joint name: [" << wheel_names[i] << "] ...");
        }

        sub_command_ = controller_nh.subscribe("cmd_vel", 1, &OmniDriveController::cmdVelCallback, this);
        return true;
    }

    void OmniDriveController::update(const ros::Time& time, const ros::Duration& period)
    {
        // update and publish odometry
        double wv1 = wheel_joints_[0].getVelocity();
        double wv2 = wheel_joints_[1].getVelocity();
        double wv3 = wheel_joints_[2].getVelocity();


        ros::Duration dt = time - time_previous_;
        Eigen::Matrix3d Rtheta, J1f, J2;
        Rtheta <<  cos(robot_theta_), sin(robot_theta_), 0,
                  -sin(robot_theta_), cos(robot_theta_), 0,
                   0,                 0,                 1;
        J1f << sin(M_PI/3),  -cos(M_PI/3),  -wheel_separation_,
               0,            -cos(M_PI),    -wheel_separation_,
               sin(-M_PI/3), -cos(-M_PI/3), -wheel_separation_;
        J2 << 2 * wheel_radius_, 0,                 0,
              0,                 2 * wheel_radius_, 0,
              0,                 0,                 2 * wheel_radius_;

        Eigen::Vector3d wv(wv1, wv2, wv3);
        Eigen::Vector3d rv_result = Rtheta.inverse() * J1f.inverse() * J2 * wv;

        // accumulated the position of robot.
        robot_x_ += rv_result[0] * (double)dt.toSec();
        robot_y_ += rv_result[1] * (double)dt.toSec();
        robot_theta_ += rv_result[2] * (double)dt.toSec();

        // publish odometry and tf
        const geometry_msgs::Quaternion orientation(tf::createQuaternionMsgFromYaw(robot_theta_));

        if (odom_pub_->trylock())
        {
            odom_pub_->msg_.header.stamp = time;
            odom_pub_->msg_.pose.pose.position.x = robot_x_;
            odom_pub_->msg_.pose.pose.position.y = robot_y_;
            odom_pub_->msg_.pose.pose.orientation = orientation;
            odom_pub_->msg_.twist.twist.linear.x  = rv_result[0];
            odom_pub_->msg_.twist.twist.linear.y  = rv_result[1];
            odom_pub_->msg_.twist.twist.angular.z = rv_result[2];
            odom_pub_->unlockAndPublish();
        }

        // Publish tf /odom frame
        if (enable_odom_tf_ && tf_odom_pub_->trylock())
        {
            geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
            odom_frame.header.stamp = time;
            odom_frame.transform.translation.x = robot_x_;
            odom_frame.transform.translation.y = robot_y_;
            odom_frame.transform.rotation = orientation;
            tf_odom_pub_->unlockAndPublish();
        }

        //=====================================================================
        // move the robot
        Commands curr_cmd = *(command_.readFromRT());
        const double timeout_dt = (time - curr_cmd.stamp).toSec();

        // Brake if cmd_vel has timeout:
        if (timeout_dt > cmd_vel_timeout_)
        {
            curr_cmd.lin_x = 0.0;
            curr_cmd.lin_y = 0.0;
            curr_cmd.ang = 0.0;
        }

        // Publish limited velocity:
        if (publish_cmd_ && cmd_vel_pub_ && cmd_vel_pub_->trylock())
        {
            cmd_vel_pub_->msg_.header.stamp = time;
            cmd_vel_pub_->msg_.twist.linear.x = curr_cmd.lin_x;
            cmd_vel_pub_->msg_.twist.linear.y = curr_cmd.lin_y;
            cmd_vel_pub_->msg_.twist.angular.z = curr_cmd.ang;
            cmd_vel_pub_->unlockAndPublish();
        }

        // Compute wheels velocities:
        double vel_wheel[3] = {0.0, 0.0, 0.0};

        double x_dot = curr_cmd.lin_x;
        double y_dot = curr_cmd.lin_y;
        double theta_dot = curr_cmd.ang;

        double u1 = cos(0) * x_dot + sin(0) * y_dot;
        double u2 = sin(0) * x_dot - cos(0) * y_dot;

        vel_wheel[0] = (sqrt(3) / (4.0 * wheel_radius_)) * u1 + (1.0 / (4.0 * wheel_radius_)) * u2 - (wheel_separation_ / (2.0 * wheel_radius_)) * theta_dot;
        vel_wheel[1] = (-1.0 / (2.0 * wheel_radius_)) * u2 - (wheel_separation_ / (2.0 * wheel_radius_)) * theta_dot;
        vel_wheel[2] = (-1.0 * sqrt(3) / (4.0 * wheel_radius_)) * u1 + (1.0 / (4.0 * wheel_radius_)) * u2 - (wheel_separation_ / (2.0 * wheel_radius_)) * theta_dot;

        for(size_t i = 0; i < wheel_joints_size_; i++)
        {
            wheel_joints_[i].setCommand(vel_wheel[i]);
        }

        time_previous_ = time;
    }

    void OmniDriveController::starting(const ros::Time& time)
    {
        brake();

        ROS_INFO("start the omni_drive_controller.");
        last_state_publish_time_ = time;
        time_previous_ = ros::Time::now();

        reset_odometry();
    }

    void OmniDriveController::stopping(const ros::Time& /*time*/)
    {
        ROS_INFO("stop the controller.");
        brake();
    }

    void OmniDriveController::brake()
    {
        for(size_t i = 0; i < wheel_joints_size_; i++)
        {
            wheel_joints_[i].setCommand(0.0);
        }
    }

    void OmniDriveController::reset_odometry()
    {
        robot_x_ = 0.0;
        robot_y_ = 0.0;
        robot_theta_ = 0.0;
    }

    void OmniDriveController::cmdVelCallback(const geometry_msgs::Twist& command)
    {
        if(isRunning())
        {
            if (!allow_multiple_cmd_vel_publishers_ && sub_command_.getNumPublishers() > 1)
            {
                ROS_ERROR_STREAM_THROTTLE_NAMED(1.0, name_, "Detected " << sub_command_.getNumPublishers()
                    << " publishers. Only 1 publisher is allowed. Going to brake.");
                brake();
                return;
            }

            command_struct_.lin_x = command.linear.x;
            command_struct_.lin_y = command.linear.y;
            command_struct_.ang = command.angular.z;
            command_struct_.stamp = ros::Time::now();

            ROS_DEBUG_STREAM_NAMED(name_,
                             "Added values to command. "
                             << "Ang: "   << command_struct_.ang << ", "
                             << "Lin_x: "   << command_struct_.lin_x << ", "
                             << "Lin_y: "   << command_struct_.lin_y << ", "
                             << "Stamp: " << command_struct_.stamp);

            command_.writeFromNonRT(command_struct_);
        }
        else
        {
            ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
        }
    }

    void OmniDriveController::setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
    {
        // Get and check params for covariances
        XmlRpc::XmlRpcValue pose_cov_list;
        controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
        ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(pose_cov_list.size() == 6);
        for (int i = 0; i < pose_cov_list.size(); ++i)
        ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

        XmlRpc::XmlRpcValue twist_cov_list;
        controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
        ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(twist_cov_list.size() == 6);
        for (int i = 0; i < twist_cov_list.size(); ++i)
        ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

        // Setup odometry realtime publisher + odom message constant fields
        odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
        odom_pub_->msg_.header.frame_id = odom_frame_id_;
        odom_pub_->msg_.child_frame_id = base_frame_id_;
        odom_pub_->msg_.pose.pose.position.z = 0;
        odom_pub_->msg_.pose.covariance = {
            static_cast<double>(pose_cov_list[0]), 0., 0., 0., 0., 0.,
            0., static_cast<double>(pose_cov_list[1]), 0., 0., 0., 0.,
            0., 0., static_cast<double>(pose_cov_list[2]), 0., 0., 0.,
            0., 0., 0., static_cast<double>(pose_cov_list[3]), 0., 0.,
            0., 0., 0., 0., static_cast<double>(pose_cov_list[4]), 0.,
            0., 0., 0., 0., 0., static_cast<double>(pose_cov_list[5]) };
        odom_pub_->msg_.twist.twist.linear.y  = 0;
        odom_pub_->msg_.twist.twist.linear.z  = 0;
        odom_pub_->msg_.twist.twist.angular.x = 0;
        odom_pub_->msg_.twist.twist.angular.y = 0;
        odom_pub_->msg_.twist.covariance = {
            static_cast<double>(twist_cov_list[0]), 0., 0., 0., 0., 0.,
            0., static_cast<double>(twist_cov_list[1]), 0., 0., 0., 0.,
            0., 0., static_cast<double>(twist_cov_list[2]), 0., 0., 0.,
            0., 0., 0., static_cast<double>(twist_cov_list[3]), 0., 0.,
            0., 0., 0., 0., static_cast<double>(twist_cov_list[4]), 0.,
            0., 0., 0., 0., 0., static_cast<double>(twist_cov_list[5]) };
        tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
        tf_odom_pub_->msg_.transforms.resize(1);
        tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
        tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
        tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;
    }
}

