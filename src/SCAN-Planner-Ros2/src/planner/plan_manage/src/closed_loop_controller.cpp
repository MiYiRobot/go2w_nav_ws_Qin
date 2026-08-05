#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

#include "bspline_opt/uniform_bspline.h"

namespace scan_planner
{
class ClosedLoopController : public rclcpp::Node
{
public:
  ClosedLoopController() : Node("closed_loop_controller")
  {
    time_forward_ = declare_parameter<double>("time_forward", 0.8);
    heading_error_threshold_ = declare_parameter<double>("heading_error_threshold", 0.8);
    kp_pos_ = declare_parameter<double>("kp_pos", 0.8);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);
    
    declare_parameter<std::string>("start_navigation_topic", "/start_navigation");
    const auto start_navigation_topic = get_parameter("start_navigation_topic").as_string();
    declare_parameter<std::string>("stop_navigation_topic", "/stop_navigation");
    const auto stop_navigation_topic = get_parameter("stop_navigation_topic").as_string();


    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(  
        "planning/bspline", 10,     //B-spline轨迹订阅
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),     // 订阅机器人位姿odom话题
        std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);      //cmd发布
    start_navigation_sub_ = create_subscription<std_msgs::msg::Bool>(     //接收启动命令
      start_navigation_topic,rclcpp::QoS(10).reliable(),
      std::bind(&ClosedLoopController::startNavigationCallback,this,std::placeholders::_1)
    );
    // stop_navigation_sub_ = create_subscription<std_msgs::msg::Bool>(      //接收停止命令
    //   stop_navigation_topic,rclcpp::QoS(10).reliable(),
    //   std::bind(&ClosedLoopController::stopNavigationCallback,this,std::placeholders::_1)
    // );


    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>("planning/go2_execution_frozen", 10); //角度过大暂停跟踪轨迹
    cmd_timer_ = create_wall_timer(std::chrono::milliseconds(10),     // 100Hz的控制频率
                                   std::bind(&ClosedLoopController::cmdCallback, this));
    last_update_time_ = now();
    RCLCPP_INFO(get_logger(), "Closed-loop controller ready");
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;
  
  //角度归一化
  static double normalizeAngle(double angle)  
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  //限制二维速度大小
  static Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
  {
    const double norm = value.norm();
    return (norm <= max_norm || norm < 1e-6) ? value : value / norm * max_norm;
  }
  //估计当前机器人朝向
  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des) const
  {
    const double t_look = std::min(traj_duration_, t_cur + time_forward_);
    Eigen::Vector3d direction = traj_[0].evaluateDeBoorT(t_look) - pos_des;
    if (direction.head<2>().squaredNorm() < 1e-4)
      direction = traj_[1].evaluateDeBoorT(t_cur);
    return direction.head<2>().squaredNorm() < 1e-4
        ? odom_yaw_ : std::atan2(direction.y(), direction.x());
  }
  //发送停止速度，只允许yaw旋转
  void publishStop(double yaw_rate = 0.0)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(yaw_rate, -max_vyaw_, max_vyaw_);
    cmd_vel_pub_->publish(cmd);
  }
  //告诉规划器机器人是否因为角度误差太大暂停执行
  void publishExecutionFrozen(bool frozen)
  {
    std_msgs::msg::Bool msg;
    msg.data = frozen;
    execution_frozen_pub_->publish(msg);  
  }

  //获取b样条轨迹控制点
  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");
      return;
    }
    //读取控制点
    Eigen::MatrixXd points(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
      points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
    //创建B样条连续轨迹
    UniformBspline position(points, msg->order, 0.1);
    position.setKnot(knots);
    traj_ = {position, position.getDerivative()};
    traj_.push_back(traj_[1].getDerivative());
    traj_duration_ = traj_[0].getTimeSum();
    traj_id_ = msg->traj_id;
    exec_time_ = 0.0;
    last_update_time_ = now();
    receive_traj_ = true;
    if(!navigation_enable_)
    {
        RCLCPP_WARN(get_logger(),"Received trajectory %lld, but navigation is not enabled. Ignoring.", static_cast<long long>(traj_id_));
        return;
    }
    RCLCPP_INFO(get_logger(), "Received trajectory %lld, duration %.3fs",
                static_cast<long long>(traj_id_), traj_duration_);
  }

  //获取定位
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)  
  {
    odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    odom_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    have_odom_ = true;
  }

  void cmdCallback()
  {
    if (!receive_traj_ || !have_odom_)
    {
      publishExecutionFrozen(false);
      publishStop();
      return;
    }
    //没有启动导航
    if(!navigation_enable_)
    {
        publishExecutionFrozen(true);
        publishStop();
        return;
    }

    const auto current_time = now();
    double dt = (current_time - last_update_time_).seconds();
    if (dt < 0.0 || dt > 0.2) dt = 0.0;
    const double t_eval = std::min(exec_time_, traj_duration_);
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval); //计算期望位置
    const double yaw_error = normalizeAngle(estimateDesiredYaw(t_eval, pos_des) - odom_yaw_); //计算yaw偏差
    const double yaw_command = std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);
    if (std::abs(yaw_error) > heading_error_threshold_) //偏差过大只旋转不前进
    {
      publishExecutionFrozen(true);
      publishStop(yaw_command);
      last_update_time_ = current_time;
      return;
    }

    publishExecutionFrozen(false);
    exec_time_ = std::min(traj_duration_, exec_time_ + dt);   //b样条轨迹时间
    last_update_time_ = current_time;
    pos_des = traj_[0].evaluateDeBoorT(exec_time_); //计算期望位置
    const Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(exec_time_);   //根据b样条轨迹时间获取目标期望速度
    const Eigen::Vector2d pos_error(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y()); //计算位置误差
    const Eigen::Vector2d vel_world = clampNorm(  //计算世界速度：轨迹前馈（期望速度）+P反馈
        Eigen::Vector2d(vel_des.x(), vel_des.y()) + kp_pos_ * pos_error,
        std::max(max_vx_, max_vy_));
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    geometry_msgs::msg::Twist command;  //世界坐标系转成机器人自身坐标系
    command.linear.x = std::clamp(c * vel_world.x() + s * vel_world.y(), -max_vx_, max_vx_);
    command.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
    command.angular.z = yaw_command;
    if (exec_time_ >= traj_duration_ && pos_error.norm() < finish_dist_)
      command = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(command);
  }


  void startNavigationCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if(!msg->data)
    {
      RCLCPP_INFO(get_logger(),"Select only display the trajectory, but not execute it.222222222222222222222");
      navigation_enable_ = false;
      return;
    }
    navigation_enable_ = true;

    //通知规划器解除冻结
    // publishExecutionFrozen(false);

    RCLCPP_INFO(get_logger(),"Navigation started1111111111111111111111111111");
  }

  // void stopNavigationCallback(const std_msgs::msg::Bool::SharedPtr msg)
  // {
  //   if(!msg->data)
  //   {
  //     return;
  //   }

  //   RCLCPP_WARN(get_logger(),"Navigation stopped0000000000000000000000000000000");
  //   navigation_enable_ = false;
   
  //   //冻结规划端
  //   // publishExecutionFrozen(true);

  //   //立即停车
  //   // publishStop();

  // }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_navigation_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_navigation_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  bool receive_traj_{false};
  bool have_odom_{false};
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  std::int64_t traj_id_{0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  double exec_time_{0.0};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  double time_forward_, heading_error_threshold_, kp_pos_, kp_yaw_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_;
  bool navigation_enable_{false};
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}
