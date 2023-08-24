#include <control_msgs/JointTrajectoryControllerState.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/TwistStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/tfMessage.h>


namespace omni_drive_controller{

class OmniDriveController
    :public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
    public:
        OmniDriveController();

        bool init(hardware_interface::VelocityJointInterface* hw,
              ros::NodeHandle& root_nh,
              ros::NodeHandle &controller_nh);

        void update(const ros::Time& time, const ros::Duration& period);
        void starting(const ros::Time& time);
        void stopping(const ros::Time& /*time*/);

    private:
        void brake();
        void reset_odometry();
        void cmdVelCallback(const geometry_msgs::Twist& command);
        void setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

    private:
        std::string name_;

        /// Odometry related:
        ros::Duration publish_period_;
        ros::Time last_state_publish_time_;
        bool open_loop_;

        /// Hardware handles:
        std::vector<hardware_interface::JointHandle> wheel_joints_;

        // Previous time
        ros::Time time_previous_;

        /// Previous velocities from the encoders:
        std::vector<double> vel_wheel_previous_;

        struct Commands
        {
            double lin_x;
            double lin_y;
            double ang;
            ros::Time stamp;

            Commands(): lin_x(0.0), lin_y(0.0), ang(0.0), stamp(0.0) {}
        };

        /// Velocity command related:
        realtime_tools::RealtimeBuffer<Commands> command_;
        Commands command_struct_;
        ros::Subscriber sub_command_;

        /// Publish executed commands
        std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped> > cmd_vel_pub_;

        /// Odometry related:
        std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
        std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;


        double robot_x_;
        double robot_y_;
        double robot_theta_;

        /// Controller state publisher
        std::shared_ptr<realtime_tools::RealtimePublisher<control_msgs::JointTrajectoryControllerState> > controller_state_pub_;

        /// Wheel separation, wrt the midpoint of the wheel width:
        double wheel_separation_;

        /// Wheel radius (assuming it's the same for all wheels):
        double wheel_radius_;

        /// Timeout to consider cmd_vel commands old:
        double cmd_vel_timeout_;

        /// Whether to allow multiple publishers on cmd_vel topic or not:
        bool allow_multiple_cmd_vel_publishers_;

        /// Frame to use for the robot base:
        std::string base_frame_id_;

        /// Frame to use for odometry and odom tf:
        std::string odom_frame_id_;

        /// Whether to publish odometry to tf or not:
        bool enable_odom_tf_;

        /// Number of wheel joints:
        size_t wheel_joints_size_;

        bool publish_cmd_;
        bool publish_wheel_joint_controller_state_;
};

PLUGINLIB_EXPORT_CLASS(omni_drive_controller::OmniDriveController, controller_interface::ControllerBase);
}