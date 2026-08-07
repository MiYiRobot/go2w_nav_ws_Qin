
#include <plan_manage/scan_replan_fsm.h>
#include <cmath>
#include <stdexcept>
#include <limits>

namespace
{
  template <typename T>
  T load_parameter(rclcpp::Node *node, const std::string &name, const T &default_value)
  {
    if (!node->has_parameter(name)) node->declare_parameter<T>(name, default_value);
    return node->get_parameter(name).get_value<T>();
  }
} // namespace

namespace scan_planner
{

  void SCANReplanFSM::init(rclcpp::Node *node)
  {
    node_ = node;
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    rviz_height_ready_ = false;   
    go2_execution_frozen_ = false;
    flag_escape_emergency_ = true;
    need_hover_stop_ = false;
    replan_fail_count_ = 0;
    navigation_started_ = false;
    last_freeze_update_time_ = node_->now();

    /*  fsm param  */
    navi_mode_ = load_parameter<int>(node_, "fsm.navi_mode", -1);
    replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_replan", -1.0);
    no_replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_no_replan", -1.0);
    planning_horizon_ = load_parameter<double>(node_, "fsm.planning_horizon", -1.0);
    point_global_dist_ = load_parameter<double>(node_, "fsm.point_global_dist", 0.4);
    emergency_time_ = load_parameter<double>(node_, "fsm.emergency_time", 1.0);
    enable_fail_safe_ = load_parameter<bool>(node_, "fsm.fail_safe", true);
    max_replan_fail_count_ = load_parameter<int>(node_, "fsm.max_replan_fail_count", 1000);
    self_inflation_z_up_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_up", 0.0);
    self_inflation_z_down_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_down", 0.0);
    self_double_cylinder_radius_ = load_parameter<double>(node_, "grid_map.double_cylinder_radius", 0.0);
    self_double_cylinder_offset_ = load_parameter<double>(node_, "grid_map.double_cylinder_offset", 0.0);
    body_height_ = load_parameter<double>(node_, "grid_map.body_height", 0.0);
    self_inflation_frame_id_ = load_parameter<std::string>(node_, "grid_map.frame_id", "world");
    start_navigation_topic_ = load_parameter<std::string>(node_, "start_navigation_topic", "/start_navigation");

    // declare_parameter<std::string>("start_navigation_topic", "/start_navigation");
    // const auto start_navigation_topic = get_parameter("start_navigation_topic").as_string();

    //预设航点初始化
    if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      const auto flat_waypoints = load_parameter<std::vector<double>>(node_, "fsm.waypoints", {});
      if (flat_waypoints.empty() || flat_waypoints.size() % 3 != 0)
        throw std::runtime_error("navi_mode=2 requires non-empty fsm.waypoints with x,y,z triples");
      waypoint_num_ = static_cast<int>(flat_waypoints.size() / 3);
      preset_waypoints_.resize(waypoint_num_);
      for (int i = 0; i < waypoint_num_; i++)
      {
        preset_waypoints_[i] = Eigen::Vector3d(flat_waypoints[3 * i], flat_waypoints[3 * i + 1],
                                               flat_waypoints[3 * i + 2]);
      }
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(node_));       //rviz可视化模块
    planner_manager_.reset(new SCANPlannerManager);               //规划器模块
    planner_manager_->initPlanModules(node_, visualization_);     //规划器初始化

    /* callback */
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),   //100Hz的状态机执行频率
                                           std::bind(&SCANReplanFSM::execFSMCallback, this));
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),   //20Hz的安全检查频率
                                             std::bind(&SCANReplanFSM::checkCollisionCallback, this));
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(  //订阅定位数据
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&SCANReplanFSM::odometryCallback, this, std::placeholders::_1));
    go2_execution_frozen_sub_ = node_->create_subscription<std_msgs::msg::Bool>(  //订阅规划器是否因为角度误差过大而暂停执行
        "planning/go2_execution_frozen", 10,
        std::bind(&SCANReplanFSM::go2ExecutionFrozenCallback, this, std::placeholders::_1));
    start_navigation_sub_ = node_->create_subscription<std_msgs::msg::Bool>(     //接收启动命令
      start_navigation_topic_,rclcpp::QoS(10).reliable(),
      std::bind(&SCANReplanFSM::startNavigationCallback,this,std::placeholders::_1));

    bspline_pub_ = node_->create_publisher<scan_planner_msgs::msg::Bspline>("planning/bspline", 10);  //b样条轨迹发布
    data_disp_pub_ = node_->create_publisher<scan_planner_msgs::msg::DataDisp>("planning/data_display", 100); //调试信息
    self_inflation_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "self_inflation", rclcpp::QoS(1).reliable().transient_local());

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)   //mode1：rviz目标
      goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "move_base_simple/goal", 1,
          std::bind(&SCANReplanFSM::rvizGoalCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)   //mode3:参考路径
      path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
          "initial_path", rclcpp::QoS(1).transient_local().reliable(), std::bind(&SCANReplanFSM::pathCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::PRESET_TARGET)    //mode2:预设目标
      RCLCPP_INFO(node_->get_logger(), "Preset waypoint mode will start after the first odometry message");
    else
      throw std::runtime_error("fsm.navi_mode must be 1, 2, or 3");
  }

  //根据航点规划全局轨迹
  void SCANReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps = preset_waypoints_;     //复制预设航点

    for (size_t i = 0; i < wps.size(); i++)   //在rviz中显示所有航点
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    active_waypoints_ = wps;  //保存当前航点
    current_wp_ = 0;      //设置航点索引
    trigger_ = true;      //开启触发规划
    init_pt_ = odom_pos_;  //记录机器人当前位置作为起点

    if (planNextWaypoint())   //规划全局轨迹，标志位设置
    {
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory to first preset waypoint");
    }
  }

  //接收rviz发来的目标点
  void SCANReplanFSM::rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
    if (!msg)
      return;

    if (!rviz_height_ready_)  //是否收到定位中的高度信息
    {
      RCLCPP_WARN(node_->get_logger(), "Ignore RViz goal before receiving initial body pose");
      return;
    }

    auto path = std::make_shared<nav_msgs::msg::Path>();    //获取rviz发来的目标点
    path->header = msg->header;
    path->poses.push_back(*msg);
    waypointCallback(path);     //根据路径点生成全局轨迹
  }

  // 生成全局轨迹、处理终点位置、改变标志位、状态转换、Rviz可视化显示
  void SCANReplanFSM::waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Empty waypoint message; ignoring");
      return;
    }

    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;   //记录机器人当前位置

    //根据起点（当前位置和速度）和终点（传进来的目标点）生成全局轨迹
    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, rviz_goal_height_;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
      success = adjustGlobalTargetIfOccupied(); //检查终点是否位于安全区域，如果不在则沿着轨迹往回找一个安全点作为新的终点

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0); //Rviz显示目标点

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;  // 轨迹显示采样时间
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);   // 计算采样点数
      vector<Eigen::Vector3d> gloabl_traj(i_end);  //创建轨迹点数组
      for (int i = 0; i < i_end; i++)   //获取全局轨迹点
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      //设置终点速度为0及是否有目标点标志位
      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/                   //状态转换
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");   
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0); //轨迹可视化
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory");
    }
  }

  // 规划航点序列的全局轨迹，检查终点是否安全，设置标志位，Rviz可视化显示
  bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    //异常检查
    if (waypoints.size() < 2)
    {
      // RCLCPP_WARN(node_->get_logger(), "No waypoint supplied for global trajectory");
      RCLCPP_WARN(node_->get_logger(), "[planGlobalTrajByWaypoints] Reference path requires at least two points.");
      return false;
    }
    //设置终点为航点序列的最后一个点
    end_pt_ = waypoints.back();
    std::vector<Eigen::Vector3d> reference_waypoints(waypoints.begin() +1, waypoints.end());

    //rviz显示所有航点（一个大球）
    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    // 在航点中插点、时间分配、Minimum Snap优化，生成全局轨迹
    // bool success = planner_manager_->planGlobalTrajWaypoints( 
    //     odom_pos_,
    //     odom_vel_,
    //     Eigen::Vector3d::Zero(),
    //     waypoints,
    //     Eigen::Vector3d::Zero(),
    //     Eigen::Vector3d::Zero());

      bool success = planner_manager_->planGlobalTrajWaypoints( 
      waypoints.front(),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      reference_waypoints,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero());

      //  bool success = planner_manager_->planGlobalTrajWaypoints( 
      // odom_pos_,
      // Eigen::Vector3d::Zero(),
      // Eigen::Vector3d::Zero(),
      // reference_waypoints,
      // Eigen::Vector3d::Zero(),
      // Eigen::Vector3d::Zero());

    if (!success) //生成失败报异常，返回false
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from waypoints");
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())  //检查轨迹终点位置是否安全
      return false;

    constexpr double step_size_t = 0.1;   //采样时间：0.1s
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t); //计算采样数量
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++) //连续轨迹离散化
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;  //设置有目标点标志位
    have_new_target_ = true;  //设置有新目标点标志位
    //rviz显示轨迹和终点
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, static_cast<int>(waypoints.size()) - 1);

    return true;
  }


  //规划当前航点的全局轨迹，检查终点是否安全，设置标志位，Rviz可视化显示
  bool SCANReplanFSM::planNextWaypoint()
  {
    //处理异常情况
    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size())
    {
      RCLCPP_WARN(node_->get_logger(), "[navi_mode=%d] No active waypoint to plan", navi_mode_);
      return false;
    }

    end_pt_ = active_waypoints_[current_wp_];   //设置当前目标点
    setStartStateFromOdomOrCurrentTraj();   //设置起点状态

    bool success = planner_manager_->planGlobalTraj(  //生成全局轨迹 
        start_pt_,
        start_vel_,
        start_acc_,
        end_pt_,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)   //全局规划失败就返回false
    {
      RCLCPP_ERROR(node_->get_logger(), "[navi_mode=%d] Unable to generate trajectory to waypoint %d",
                   navi_mode_, current_wp_ + 1);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())  //检查目标点是否被占用
      return false;

    constexpr double step_size_t = 0.1;     //设置采样间隔，0.1s取一个点
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t); //计算轨迹总时间
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);    //用来保存采样点
    for (int i = 0; i < i_end; i++)   //采样轨迹点
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();     //最终目标速度设为0，即每个点停一下
    have_target_ = true;    //有目标
    have_new_target_ = true;    //有新目标
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);     //rviz显示全局路径
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, current_wp_);   //在目标点显示一个球
    RCLCPP_INFO(node_->get_logger(), "[navi_mode=%d] Planning to waypoint %d/%zu: [%.2f, %.2f, %.2f]",  //打印相关信息
                navi_mode_, current_wp_ + 1, active_waypoints_.size(), end_pt_(0), end_pt_(1), end_pt_(2));

    return true;
  }

  //是否为模式2
  bool SCANReplanFSM::isWaypointSequenceMode() const
  {
    return navi_mode_ == NAVI_MODE::PRESET_TARGET;
  }

  //检查全局轨迹终点是否落入膨胀障碍，如果是则沿着已经生成的轨迹从终点往回找一个安全点作为新的终点
  bool SCANReplanFSM::adjustGlobalTargetIfOccupied()
  {
    //获取地图和全局轨迹数据
    auto map = planner_manager_->grid_map_;
    auto &global_data = planner_manager_->global_data_;
    const double duration = global_data.global_duration_;
    if (!map || duration < 1e-3)
      return true;

    constexpr double sample_dt = 0.05;
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));  //计算采样点数量
    const Eigen::Vector3d final_pt = global_data.global_traj_.evaluate(duration);   //检查最终目标点
    const Eigen::Vector3d final_prev = global_data.global_traj_.evaluate(duration * (sample_num - 1) / sample_num); //获取最终目标点前一个点
    const int final_occ = map->getInflateOccupancy(final_pt, estimateYawFromSegment(final_prev, final_pt)); //检查最终目标点是否被占用
    if (final_occ <= 0) //没被占用返回true
      return true;

    //被占用了，寻找全局轨迹上离终点最近的一个未被占用的点作为新的终点
    for (int i = sample_num; i >= 0; --i)     //从终点往回找
    {
      const double t = duration * i / sample_num; //计算采样时间
      const double prev_t = duration * std::max(0, i - 1) / sample_num;
      const Eigen::Vector3d pt = global_data.global_traj_.evaluate(t);  //计算轨迹位置
      const Eigen::Vector3d prev_pt = global_data.global_traj_.evaluate(prev_t);

      if (map->getInflateOccupancy(pt, estimateYawFromSegment(prev_pt, pt)) == 0) //判断这个点是否被占用
      {
        const Eigen::Vector3d raw_end = end_pt_;
        end_pt_ = pt; //修改目标点
        global_data.global_duration_ = t; //修改全局轨迹时间
        global_data.last_progress_time_ = std::min(global_data.last_progress_time_, t); //修改进度时间
        RCLCPP_WARN(node_->get_logger(),
                    "Target [%.2f, %.2f, %.2f] is occupied; using [%.2f, %.2f, %.2f]",
                    raw_end(0), raw_end(1), raw_end(2), end_pt_(0), end_pt_(1), end_pt_(2));
        return true;
      }
    }
    //整条轨迹都没有安全点，打印数据返回false
    RCLCPP_ERROR(node_->get_logger(),
                 "Target is occupied and no collision-free point was found on the global trajectory");
    return false;
  }


  void SCANReplanFSM::startNavigationCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
      navigation_started_ = msg->data;
      if(navigation_started_)     //导航启动，开始规划
      {  
          if(!pending_waypoints_.empty())
          {
            trigger_ = true;    //设置导航触发标志位
            bool success = planGlobalTrajByWaypoints(pending_waypoints_);    //根据路径点生成全局轨迹，设置标志位，rviz可视化显示
            //FSM状态机切换
            if (success)
            {
              /*** FSM ***/
              if (exec_state_ == WAIT_TARGET)   //如果当前状态是等待目标点，则切换到生成新轨迹状态
              {
                changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
              }
              else if (exec_state_ == EXEC_TRAJ)    //如果当前状态是执行轨迹，则切换到重新规划轨迹状态
              {
                changeFSMExecState(REPLAN_TRAJ, "TRIG");
              }
              else if (exec_state_ == EMERGENCY_STOP)   
              {
                changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
              }

              RCLCPP_INFO(node_->get_logger(), "Reference path accepted!!!!!!!!!!!!!!!!!!!!!!!!!!");
            }
            else
            {
              RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from reference path~~~~~~~~~~~~~~~~~");
            }
          }
          else
          {
              RCLCPP_WARN(node_->get_logger(), "No path received yet");
          }
      }
      else      //停止规划，进入急停状态
      {
          RCLCPP_INFO(node_->get_logger(), "Navigation stopped");

          //停止时可以进入急停状态
          changeFSMExecState(EMERGENCY_STOP, "STOP NAVIGATION");
      }

  }



  //接收全局参考路径，生成全局轨迹，切换FSM状态和标志位
  void SCANReplanFSM::pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    //检查路径有效性
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Received empty initial_path; ignoring");
      return;
    }

    if(!have_odom_)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "[pathCallback] No odometry yet, cannot plan global trajectory.");
      return;
    }

    end_pt_ << msg->poses.back().pose.position.x, msg->poses.back().pose.position.y, msg->poses.back().pose.position.z + body_height_;  //设置终点为路径的最后一个点

    std::vector<Eigen::Vector3d> new_waypoints;   //创建waypoints保存路径点
    new_waypoints.reserve(msg->poses.size() + 1);     //预分配空间

    // constexpr double min_dist = 0.4;
    // 以机器人当前位置作为参考路径起点
    Eigen::Vector3d robot_pos = odom_pos_;
    new_waypoints.push_back(robot_pos);

    Eigen::Vector3d last_wp = robot_pos;

    // 检查外部路径点起点与机器人当前位置是否一致
    Eigen::Vector3d first_path_pt;
    first_path_pt << msg->poses.front().pose.position.x, msg->poses.front().pose.position.y, msg->poses.front().pose.position.z + body_height_;
    if((first_path_pt - robot_pos).norm() > 1.0)
    {
      RCLCPP_WARN(node_->get_logger(), "Reference path start point is far from robot position: [%.2f, %.2f, %.2f] vs [%.2f, %.2f, %.2f]",
                  first_path_pt(0), first_path_pt(1), first_path_pt(2),
                  robot_pos(0), robot_pos(1), robot_pos(2));
    }
    
    for(const auto &pose_stamped : msg->poses)
    {
      Eigen::Vector3d wp;
      wp(0) = pose_stamped.pose.position.x;
      wp(1) = pose_stamped.pose.position.y;
      wp(2) = pose_stamped.pose.position.z + body_height_; // Adjust for body height     ？z轴注意点，看输入的是加上body_height_还是没加上

      if((wp - last_wp).norm() >= point_global_dist_)
      {
        new_waypoints.push_back(wp);
        last_wp = wp;
      }
    }

    if((new_waypoints.back() - end_pt_).norm() > 1e-6)
      new_waypoints.push_back(end_pt_);  //确保终点在航点序列中

    pending_waypoints_ = new_waypoints;  //保存路径点
    RCLCPP_INFO(node_->get_logger(), "Received reference path with %zu waypoints", pending_waypoints_.size());

  }

  // 根据定位信息更新当前位置、速度、姿态
  void SCANReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_)
    {
      rviz_goal_height_ = odom_pos_(2);   //更新机器人高度
      rviz_height_ready_ = true;
      RCLCPP_INFO(node_->get_logger(), "Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);
    }

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
    publishSelfInflationMarker();   //可视化碰撞圆柱体
    if (navi_mode_ == NAVI_MODE::PRESET_TARGET && !preset_started_)
    {
      preset_started_ = true;
      planGlobalTrajbyGivenWps();   //根据航点规划全局轨迹
    }
  }

  void SCANReplanFSM::go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    go2_execution_frozen_ = msg->data;
  }

  //同步局部轨迹的执行开始时间
  void SCANReplanFSM::updateLocalTrajTimeFreeze()
  {
    const rclcpp::Time now = node_->now();
    double dt = (now - last_freeze_update_time_).seconds();
    last_freeze_update_time_ = now;

    if (dt <= 0.0 || dt > 0.2)
      return;

    LocalTrajData *info = &planner_manager_->local_data_;   //取当前局部规划信息
    if (go2_execution_frozen_ && info->start_time_.seconds() > 1e-5) //机器人被冻结了且局部轨迹已经开始执行
      info->start_time_ += rclcpp::Duration::from_seconds(dt);  //同步局部轨迹的执行开始时间
  }

  double SCANReplanFSM::getOdomYaw() const
  {
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0);
    if (heading.head<2>().squaredNorm() < 1e-8)
      return 0.0;
    return std::atan2(heading(1), heading(0));
  }

  //根据两点计算期望两点之间的偏航角
  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    if (diff.squaredNorm() < 1e-8)
      return getOdomYaw();
    return std::atan2(diff(1), diff(0));
  }

  //在rviz中发布机器人自身膨胀模型的Marker，用来可视化规划器认为机器人占据的空间（两个圆柱）
  void SCANReplanFSM::publishSelfInflationMarker()
  {
    const double radius = std::max(0.0, self_double_cylinder_radius_);
    const double z_up = std::max(0.0, self_inflation_z_up_);
    const double z_down = std::max(0.0, self_inflation_z_down_);
    const double height = std::max(1e-3, z_up + z_down);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "self_inflation";
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = height;
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    Eigen::Vector3d center = odom_pos_;
    center(2) += 0.5 * (z_up - z_down);

    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);
    Eigen::Vector3d front = center + self_double_cylinder_offset_ * heading;
    Eigen::Vector3d rear = center - self_double_cylinder_offset_ * heading;

    marker.id = 0;
    marker.pose.position.x = front(0);
    marker.pose.position.y = front(1);
    marker.pose.position.z = front(2);
    self_inflation_pub_->publish(marker);

    marker.id = 1;
    marker.pose.position.x = rear(0);
    marker.pose.position.y = rear(1);
    marker.pose.position.z = rear(2);
    self_inflation_pub_->publish(marker);
  }

  //状态改变并打印
  void SCANReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continuously_called_times_++;
    else
      continuously_called_times_ = 1;   //统计当前状态进入次数

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }
  
  //返回当前状态连续调用次数
  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  //打印当前状态
  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  //状态机执行代码,100Hz
  void SCANReplanFSM::execFSMCallback()
  {
    updateLocalTrajTimeFreeze();    //同步时间

    //打印状态（1s打印一次）
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)  //没定位
      {
        return;
      }
      if (!trigger_)    //没触发规划
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)  //没目标点
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:    //第一次规划
    { 
      setStartStateFromOdomOrCurrentTraj();   //设置规划起始状态，更新start_pt_、start_vel_、start_acc_,并进行一系列安全检查

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)   //第一次进入该状态
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);    //局部重规划（主要代码部分）
      if (success)  //成功则进入下一状态
      {

        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else  //不成功则重新规划
      {
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj())      //重新生成轨迹
      {
        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_; //获取局部轨迹数据
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();  //计算当前规划执行时间
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);  //计算机器人在轨迹上的理论位置

      if (isWaypointSequenceMode() &&   //是否是模式2
          current_wp_ + 1 < (int)active_waypoints_.size() &&  //后面是否还存在航点
          (end_pt_ - odom_pos_).norm() < 0.5) //机器人距离当前航点距离小于0.5，即认为机器人到达当前航点
      {
        current_wp_++;
        if (planNextWaypoint()) //规划下一个航点
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        return;
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)   //判断当前轨迹是否执行完毕
      {
        if (isWaypointSequenceMode() && current_wp_ + 1 < (int)active_waypoints_.size())  //还有航点未到达
        {
          current_wp_++;
          if (planNextWaypoint()) //规划下一个航点
          {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            return;
          }
          replan_fail_count_++;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }

        if (isWaypointSequenceMode()) //最后一个航点都到达了，清空航点
        {
          active_waypoints_.clear();
          current_wp_ = 0;
        }

        have_target_ = false;

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)  //离全局轨迹终点距离过小了
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)  //距离局部轨迹起点过近了，不需要重复规划
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);   //发布紧急停止轨迹
      }
      else    //等待差不多停下来时再切换状态
      {
        if(navigation_started_)   //导航启动了，导航没启动就发布停止路径完了啥也不干
        {
          if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1)     // 重新开始规划
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");  
          else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1) //等待新的目标后再重新规划，重规划失败太多次才进入这里
          {
            RCLCPP_INFO(node_->get_logger(),
                        "Exiting EMERGENCY_STOP; switching to WAIT_TARGET for a new target");
            need_hover_stop_ = false;
            have_target_ = false;
            trigger_ = false;
            changeFSMExecState(WAIT_TARGET, "EMERGENCY_EXIT");
          }
        }
        else
        {
          // RCLCPP_INFO(node_->get_logger(),"Stop!!! Wait for the start flag!");
        }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    finishProcess();  //判断是否连续重规划失败次数超过阈值，超过则进入紧急停止状态

    data_disp_.header.stamp = node_->now();
    data_disp_.a = exec_state_;
    data_disp_.b = replan_fail_count_;
    data_disp_pub_->publish(data_disp_);  //发布调试信息
  }

  //连续重规划失败次数超过阈值则进入紧急停止状态
  void SCANReplanFSM::finishProcess()
  {
    if (replan_fail_count_ >= max_replan_fail_count_)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Replan failed %d times; emergency stop and wait for a new target", replan_fail_count_);
      replan_fail_count_ = 0;
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
    }
  }

  //重新生成全局轨迹和局部轨迹，返回是否生成成功
  bool SCANReplanFSM::planFromCurrentTraj()
  {
    LocalTrajData *info = &planner_manager_->local_data_;   //获取当前局部轨迹
    rclcpp::Time time_now = node_->now();
    double t_cur = (time_now - info->start_time_).seconds();   //计算机器人处于轨迹中的哪个时间点
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    //--------------------------------add------------------------------------------------------
    if(navi_mode_ == NAVI_MODE::REFERENCE_PATH)
    {
      // start_pt_ = odom_pos_;     //获取当前轨迹的位置信息
      start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);     //获取当前轨迹的位置信息
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);     //获取当前轨迹的速度
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur); //获取当前轨迹的加速度

      // Eigen::Vector2d to_goal = local_target_pt_.head<2>() - odom_pos_.head<2>();  //计算当前位置与局部目标点的向量
      // if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)  //如果当前位置到目标点的向量与当前速度方向相反，则将速度和加速度置为0
      // {
      //   start_vel_.setZero();
      //   start_acc_.setZero();
      // }

      bool success = callReboundReplan(false,false);    //局部重规划，在已有的轨迹上重新生成轨迹
      if(!success)
      {
        success = callReboundReplan(true,false);  //重新生成一条新的局部轨迹，生成普通五次多项式轨迹
        if(!success)
        {
          success = callReboundReplan(true,true); //重新生成一条新的局部轨迹，生成随机插点轨迹
          if(!success)
            return false;
        }
      }
        return true;
    }

    //--------------------------------add------------------------------------------------------
    start_pt_ = odom_pos_;  //根据当前位置设置新的轨迹起点
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);     //获取当前轨迹的速度
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur); //获取当前轨迹的加速度

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();  //计算当前位置到目标点的向量
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)  //如果当前位置到目标点的向量与当前速度方向相反，则将速度和加速度置为0
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }

    if (!planner_manager_->planGlobalTraj(  //重新生成当前位置到目标点的全局轨迹
            start_pt_,
            start_vel_,
            start_acc_,
            end_pt_,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()))
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[navi_mode=%d] Unable to refresh global trajectory from odom to current target", navi_mode_);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())  //目标点碰撞占据检查
      return false;

    bool success = callReboundReplan(true, false);    //局部重规划，重新生成局部轨迹，生成普通五次多项式轨迹
    if (!success)
    {
      success = callReboundReplan(true, true);      //生成随机插点轨迹
      if (!success)
        return false;
    }

    return true;
  }

  //设置下一次规划时的起始状态
  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    //记录当前odom状态
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    //获取当前局部轨迹信息
    LocalTrajData *info = &planner_manager_->local_data_; 
    if (info->start_time_.seconds() < 1e-5 || info->duration_ <= 1e-5)
      return;

      //记录当前时间在轨迹中的位置
    const double raw_t_cur = (node_->now() - info->start_time_).seconds();
    if (raw_t_cur < -1e-3 || raw_t_cur > info->duration_ + 0.2)
      return;

    const double t_cur = std::min(std::max(raw_t_cur, 0.0), info->duration_); //限值
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);   //计算当前轨迹的速度
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur); //计算当前轨迹的加速度

    //检查速度方向是否朝向目标点，如果不是则将速度和加速度置为0
    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }
  }

  //50ms检查一次轨迹是否碰撞
  void SCANReplanFSM::checkCollisionCallback()
  {
    updateLocalTrajTimeFreeze();      //同步时间

    LocalTrajData *info = &planner_manager_->local_data_; //获取局部轨迹
    auto map = planner_manager_->grid_map_;   //获取地图

    if (exec_state_ == WAIT_TARGET || info->start_time_.seconds() < 1e-5)  //没有目标和局部刚启动时直接返回
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (node_->now() - info->start_time_).seconds();  //计算当前轨迹执行时间
    double t_2_3 = info->duration_ * 2 / 3;     //计算轨迹的2/3时间点
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      //只检测前2/3轨迹
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t);  //计算轨迹位置
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(std::min(t + time_step, info->duration_));  //计算轨迹下一位置
      if (map->getInflateOccupancy(pos, estimateYawFromSegment(pos, pos_next)))   //检查机器人前后两个圆柱模型是否落入膨胀障碍物地图中
      {
        if (planFromCurrentTraj()) // Make a chance  重新生成轨迹，如果成功则继续执行轨迹
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time    距离障碍轨迹过近，直接进入紧急停止状态
          {
            RCLCPP_WARN(node_->get_logger(), "Obstacle discovered; emergency stop in %.3fs", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else    //距离障碍物还有时间，重规划
          {
            //ROS_WARN("current traj in collision, replan.");
            RCLCPP_WARN(node_->get_logger(), "current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  //重规划，更新局部轨迹，发布局部轨迹消息
  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();   //获取局部目标点，更新local_target_pt_、local_target_vel_

    bool plan_success =     //生成局部轨迹
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;   //标志位清除

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_;   //获取最新局部轨迹，前面reboundReplan已经更新过了

      /* publish traj */
      scan_planner_msgs::msg::Bspline bspline;    //创建要发布的轨迹消息
      bspline.order = 3;
      bspline.start_time = info->start_time_;     
      bspline.traj_id = info->traj_id_;           

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();   //获取局部轨迹的控制点
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)  //保存为ROS point
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      //获取节点向量
      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)  //保存
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_->publish(bspline); //发布局部轨迹消息

      visualization_->displayOptimalTraj(info->position_traj_, 0);  //rviz可视化
    }

    return plan_success;
  }

  //生成停止轨迹并发布
  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);    //生成停止轨迹，更新停止轨迹的local_data

    auto info = &planner_manager_->local_data_;   //获取最新局部轨迹，前面EmergencyStop已经更新过了

    /* publish traj */
    scan_planner_msgs::msg::Bspline bspline;    //创建要发布的轨迹消息
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_->publish(bspline); //发布停止轨迹消息

    return true;
  }

  //从全局规划中悬着一个适合作为本次局部规划终点的局部目标
  void SCANReplanFSM::getLocalTarget()
  {

    const double max_vel = planner_manager_->pp_.max_vel_; //获取最大速度
    const double max_acc = planner_manager_->pp_.max_acc_; //获取最大加速度
    const double duration = planner_manager_->global_data_.global_duration_;     //获取全局轨迹数据
 
    double t_step = max_vel > 1e-6 ? planning_horizon_ / 20.0 / max_vel : 0.01; //计算采样时间步长，确保不会除以零 3.5/20/0.5--->0.35s检查一遍
    t_step = std::max(t_step, 0.01); // Ensure a minimum time step of 0.01s


    // const double search_start_t = std::min(std::max(global_data.last_progress_time_, 0.0), duration);   //确定搜索起点
    // double projection_t = search_start_t;   //初始化投影时间
    // double min_dist_to_start = std::numeric_limits<double>::max();    //初始化最近距离

    double t_proj = 0.0;
    double min_dist_to_start = 9999.0;
    for(double t = 0.0; t < duration; t+= t_step)
    {
      const Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t); //获取当前时间点轨迹位置
      // const double dist_to_start = (pos_t - odom_pos_).norm(); //计算机器人与轨迹点距离（投影距离）
      const double dist_to_start = (pos_t - start_pt_).norm(); //计算机器人与轨迹点距离（投影距离）
      if(dist_to_start < min_dist_to_start) //判断是否是最近点
      {
        min_dist_to_start = dist_to_start; //更新最近点
        t_proj = t;
      }
    }

    double  target_t = duration; // Initialize target_t to the end of the global trajectory
    double total_dist = 0.0; //累计轨迹距离
    bool target_found = false; //是否找到合适的局部目标点
    Eigen::Vector3d prev_pos = planner_manager_->global_data_.getPosition(t_proj); //获取机器人在全局轨迹上的投影位置
    local_target_pt_ = end_pt_; //初始化局部目标，默认为全局终点

    //遍历全局轨迹
    for (double t = t_proj; t < duration; t += t_step)
    {
      const Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t); //获取当前时间点轨迹位置
      total_dist += (pos_t - prev_pos).norm(); //累计轨迹距离
      if (total_dist >= planning_horizon_) 
      {
        local_target_pt_ = pos_t; //更新局部目标点
        target_t = t; //更新局部目标点对应时间
        target_found = true; //标记找到局部目标点
        break;
      }
      prev_pos = pos_t; //更新前一个轨迹点
    }
    planner_manager_->global_data_.last_progress_time_ = target_found ? target_t : duration; //更新机器人当前走到哪里

    // //检查局部目标点是否碰撞
    auto targetOccupancy = [&](const Eigen::Vector3d &pt) {
      return planner_manager_->grid_map_->getInflateOccupancy(pt, estimateYawFromSegment(odom_pos_, pt));
    };


    if (targetOccupancy(local_target_pt_) != 0) //如果局部目标点在障碍里面
    {
      bool found_free_target = false;
      double adjusted_t = target_t;

      //寻找附近安全点
      for (double dt = 0.0; dt <= planner_manager_->global_data_.global_duration_; dt += t_step)
      {
        double t_forward = target_t + dt;  //向前找
        if (t_forward <= planner_manager_->global_data_.global_duration_)
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_forward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_forward;
            found_free_target = true;
            break;
          }
        }

        double t_backward = target_t - dt;  //向后找
        if (t_backward >= std::max(0.0, t_proj))
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_backward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_backward;
            found_free_target = true;
            break;
          }
        }
      }

      if (found_free_target)
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target was adjusted to a nearby collision-free point");
        target_t = adjusted_t;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target is in collision and no nearby free target was found");
      }
    }
    //计算目标速度
    // if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    if ((end_pt_ - local_target_pt_).norm() < (max_vel * max_vel) / (2 * max_acc))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();    //距离太小停车
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(target_t); //沿着global轨迹继续运动
      if(local_target_vel_.norm() > max_vel)
        local_target_vel_ = local_target_vel_.normalized() * max_vel;  //限速
      // cout << "AA" << endl;
    }
  }

} // namespace scan_planner
