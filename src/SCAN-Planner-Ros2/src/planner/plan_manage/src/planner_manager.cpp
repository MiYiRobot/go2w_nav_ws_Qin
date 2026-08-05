// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <chrono>
#include <thread>

namespace scan_planner
{
  namespace
  {
    // 给一条已有的三维路径点重新分配z高度，使z坐标沿着路径的水平距离线性变化
    void applyLinearZReference(std::vector<Eigen::Vector3d> &points, const double start_z, const double target_z)
    {
      if (points.empty())
        return;

      if (points.size() == 1) //只有一个点时直接设置
      {
        points.front()(2) = start_z;
        return;
      }

      std::vector<double> accumulated_xy_length(points.size(), 0.0); //用来保存从起点走到当前的点的水平距离
      for (size_t i = 1; i < points.size(); ++i)  //计算xy累计距离
      {
        accumulated_xy_length[i] = accumulated_xy_length[i - 1] +
                                   (points[i].head<2>() - points[i - 1].head<2>()).norm();
      }

      const double total_xy_length = accumulated_xy_length.back();    //总水平长度
      for (size_t i = 0; i < points.size(); ++i)  
      {
        const double ratio = total_xy_length > 1e-6   //给每个点计算比例
                                 ? accumulated_xy_length[i] / total_xy_length
                                 : static_cast<double>(i) / static_cast<double>(points.size() - 1);
        points[i](2) = start_z + ratio * (target_z - start_z);    //z轴线性插值
      }

      //强制修正起点和终点z轴坐标
      points.front()(2) = start_z;
      points.back()(2) = target_z;
    }
  } // namespace

  // SECTION interfaces for setup and query

  SCANPlannerManager::SCANPlannerManager() {}

  SCANPlannerManager::~SCANPlannerManager() { std::cout << "des manager" << std::endl; }

  void SCANPlannerManager::initPlanModules(rclcpp::Node *node, PlanningVisualization::Ptr vis)
  {
    node_ = node;
    /* read algorithm parameters */
    const auto get_double = [node](const std::string &name, double default_value) {
      if (!node->has_parameter(name)) node->declare_parameter<double>(name, default_value);
      return node->get_parameter(name).as_double();
    };
    pp_.max_vel_ = get_double("manager.max_vel", -1.0);
    pp_.max_acc_ = get_double("manager.max_acc", -1.0);
    pp_.max_jerk_ = get_double("manager.max_jerk", -1.0);
    pp_.vel_tolerance_ = get_double("optimization.vel_tolerance", 1.0);
    pp_.acc_tolerance_ = get_double("optimization.acc_tolerance", 1.0);
    pp_.feasibility_tolerance_ = get_double("manager.feasibility_tolerance", 0.0);
    pp_.ctrl_pt_dist = get_double("manager.control_points_distance", -1.0);
    pp_.planning_horizon_ = get_double("manager.planning_horizon", 5.0);

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    grid_map_->initMap(node_);    //在这里订阅 "sensor_pose" 、"cloud"和"body_pose"等话题，发布灰度图信息

    //初始化Bspline优化器
    bspline_optimizer_rebound_.reset(new BsplineOptimizer);
    bspline_optimizer_rebound_->setParam(node_);
    bspline_optimizer_rebound_->setEnvironment(grid_map_);
    bspline_optimizer_rebound_->a_star_.reset(new AStar);
    bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;
  }

  // !SECTION

  // SECTION rebond replanning

  //根据机器人当前状态和局部目标点，在当前三维占据地图中生成一条满足避障、平滑、速度和加速度约束的局部B-spline轨迹，并保存给控制器执行，成功构建并保存轨迹返回true
  //flag_polyInit：是否需要重新生成初始轨迹   flag_randomPolyTraj：是否需要随机插点生成初始轨迹，true为随机插点生成，flase则为普通五次多项式轨迹
  bool SCANPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {

    static int count = 0; // 记录重规划调用次数
    std::cout << endl
              << "[rebo replan]: -------------------------------------" << count++ << std::endl;
    cout.precision(3);    //设置浮点数输出精度，小数点后3位
    cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
         << endl;

    //判断是否已经接近目标
    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      continuous_failures_count_++;
      return false;
    }

    auto t_start = std::chrono::steady_clock::now();    //开始时间
    double t_init = 0.0, t_opt = 0.0, t_refine = 0.0;     //初始化时间、优化时间、精炼时间

    /*** STEP 1: INIT ***/
    // 记录轨迹中相邻两点之间的时间：给多点时间裕值，接近目标点就给更多裕值
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives; //轨迹采样点及边界导数
    static bool flag_first_call = true, flag_force_polynomial = false;  //是否第一次进入、是否强制生成多项式轨迹
    bool flag_regenerate = false;   //是否需要重新生成轨迹
    do    //生成一条合适的初始轨迹
    {     
      //清空上一轮数据
      point_set.clear();
      start_end_derivatives.clear(); 
      flag_regenerate = false;

      //判断是否需要重新生成初始轨迹：第一次调用、外部要求、强制重新生成
      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        // 重置flag
        flag_first_call = false;
        flag_force_polynomial = false;

        PolynomialTraj gl_traj;     //创建全局临时轨迹对象

        double dist = (start_pt - local_target_pt).norm();    //计算起点到目标点的距离
        //计算初始轨迹时间：距离小于最大加速距离，采用匀加速模型；小于最大加速距离，采用加速+匀速+减速模型
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        //两种Polynomial轨迹生成方式：1.单段轨迹；2.随机插入点生成多段轨迹
        if (!flag_randomPolyTraj) 
        {
          //普通五次多项式
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else  //随机插点轨迹：目的是改变初始估计形状
        {
          Eigen::Vector3d horizon_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();  //横向偏移方向
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizon_dir)).normalized();    //叉乘得到垂直偏移方向 
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +       //生成随机插入点：在起点和终点中间，沿着横向和垂直方向偏移（失败次数越多，随机范围越大）
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizon_dir * 0.8 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989);
          Eigen::MatrixXd pos(3, 3);    //三点生成min snap轨迹
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        //采样Polynomial轨迹，开始将连续轨迹离散化为离散点集
        double t;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();    //清空上一轮采样点
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);  //记录上一个采样点
          for (t = 0; t < time; t += ts)    //逐时间采样
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)   //两个采样点之间的距离不能过大
            {
              flag_too_far = true;  //距离过大则重新采样
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);  //保存采样点
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        //保存边界速度和加速度
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));  //起点速度
        start_end_derivatives.push_back(local_target_vel);        //终点速度
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));  //起点加速度
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));  //终点加速度
      }
      else // Initial path generated from previous trajectory. 已经有一条轨迹了，基于上一条轨迹继续延伸
      {

        double t;
        double t_cur = (node_->now() - local_data_.start_time_).seconds(); //计算当前机器人执行这条局部轨迹多久了

        vector<double> pseudo_arc_length;    //定义弧长距离数组，保存轨迹累计距离
        vector<Eigen::Vector3d> segment_point;    //用来保存旧轨迹采样点
        pseudo_arc_length.push_back(0.0);     //初始化弧长
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)  //从当前估计继续采样
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t)); //获取旧轨迹位置
          if (t > t_cur)
          {   //计算累计距离
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;    //得到旧轨迹最后时间

        //判断是否需要连接新目标
        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts) //距离过远则补一段轨迹
        {     //生成旧轨迹末端到新目标的一段平滑连接
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          //把连接段加入采样
          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              RCLCPP_ERROR(node_->get_logger(), "pseudo_arc_length is empty; aborting replan");
              continuous_failures_count_++;
              return false;
            }
          }
        }

        //根据弧长重新采样控制点
        double sample_length = 0;     //当前采样距离
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do  
        {   //重新均匀采样控制点
          cps_dist /= 1.5;
          point_set.clear();  //清除缓存
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              //线性插值获得点
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);   //加入最终目标
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        //保存边界条件
        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        //检查轨迹是否异常长
        if (point_set.size() > pp_.planning_horizon_ / pp_.ctrl_pt_dist * 3) // The initial path is abnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    //修正z轴高度，使z坐标沿着路径的水平距离线性变化
    applyLinearZReference(point_set, start_pt(2), local_target_pt(2));

    //B-spline参数化：将离散点集转换为B样条轨迹
    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    //A*路径规划初始化：在B样条轨迹基础上进行A*路径规划，得到一条可行的路径
    vector<vector<Eigen::Vector3d>> a_star_paths;
    a_star_paths = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

    //记录初始化时间
    t_init = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    //可视化初始轨迹
    static int vis_id = 0;
    visualization_->displayInitPathList(point_set, 0.2, 0);
    visualization_->displayAStarList(a_star_paths, vis_id);

    t_start = std::chrono::steady_clock::now();   //记录优化开始时间

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);  //B样条轨迹优化:使轨迹满足平滑、避障
    cout << "first_optimize_step_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success) //优化失败直接返回False
    {
      // visualization_->displayOptimalList( ctrl_pts, vis_id );
      continuous_failures_count_++;
      return false;
    }
    //visualization_->displayOptimalList( ctrl_pts, vis_id );

    //记录优化时间
    t_opt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
    t_start = std::chrono::steady_clock::now();

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts); //构造最终的B样条轨迹对象
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);    //设置动力学限制

    //检查动力学可行性
    double ratio;
    bool flag_step_2_success = true;
    if (!pos.checkFeasibility(ratio, false))  
    {
      cout << "Need to reallocate time." << endl;
      //时间重分配
      Eigen::MatrixXd optimal_control_points;
      flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);  // 拉长时间
      if (flag_step_2_success)  //优化成功则重新生成轨迹
        pos = UniformBspline(optimal_control_points, 3, ts);
    }
    
    //最终检查：检查时间优化是否成功，最终速度、加速度是否满足约束
    if (!flag_step_2_success || !checkDynamicFeasibility(pos))
    {
      printf("\033[34mThis refined trajectory is unsafe or dynamically infeasible. Skip publishing it.\n\033[0m");
      continuous_failures_count_++;
      return false;
    }

    t_refine = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    // save planned results
    updateTrajInfo(pos, node_->now());  //保存最终轨迹

    cout << "total time:\033[42m" << (t_init + t_opt + t_refine)
         << "\033[0m,optimize:" << (t_init + t_opt) << ",refine:" << t_refine << endl;

    // success. YoY
    continuous_failures_count_ = 0;
    return true;
  }

  //生成停止轨迹，并保存到局部规划轨迹数据中
  bool SCANPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++) // 6个控制点，停在当前位置
    {
      control_points.col(i) = stop_pos;
    }
    //创建一个3阶B样条轨迹，控制点间隔为1个周期
    updateTrajInfo(UniformBspline(control_points, 3, 1.0), node_->now());

    return true;
  }

  // 把离散的导航目标点，经过插值、时间分配和Minimum Snap优化，转换成一条连续可执行的全局参考轨迹
  bool SCANPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory
    //异常检查
    if (waypoints.empty())
      return false;

    vector<Eigen::Vector3d> points;   //用来保存路径点
    points.push_back(start_pos);      //先加入起点

    //把所有路径加入points中
    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    //计算总长度
    double total_len = 0;
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      total_len += (points[i + 1] - points[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;     //插入点
    double dist_thresh = max(total_len / 8, 4.0); //相邻两个轨迹点不能超过四米，或者总长度的1/8

    //插点
    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm(); //判断两点距离

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1; //计算应该插几个点

        //线性插值插点
        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());    //加入终点

    // for ( int i=0; i<inter_points.size(); i++ )
    // {
    //   cout << inter_points[i].transpose() << endl;
    // }

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);   //生成位置矩阵
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)  //时间分配
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    //起点和终点时间给多点，缓启动
    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    //生成多段Minimum Snap轨迹
    PolynomialTraj gl_traj;   //用来保存全局轨迹
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = node_->now();
    global_data_.setGlobalTraj(gl_traj, time_now);  //保存轨迹和时间到全局轨迹

    return true;
  }

  /* 
    根据起点和终点生成多段minSnap全局轨迹（没避障） 
    param：起点位置、起点速度、起点加速度、终点位置、终点速度、终点加速度
    return: 是否成功生成全局轨迹
  */
  bool SCANPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory 构造路径点

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far 中间插点
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;
    //相邻两个轨迹点不能超过四米
    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1; //计算应该插几个点

        for (int j = 1; j < id_num; ++j) 
        {
          Eigen::Vector3d inter_pt =                // 线性插值
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);   //转换成矩阵
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)  //估计每段时间
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    //起点和终点时间给多点，缓启动
    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    //生成多段Minimum Snap轨迹
    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = node_->now();
    global_data_.setGlobalTraj(gl_traj, time_now);  //保存轨迹和时间到全局轨迹 

    return true;
  }

  //拿已有的 B-spline 初始轨迹 → 调整时间参数和控制点 → 重新优化控制点 → 输出更平滑、更安全的 B-spline
  bool SCANPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_rebound_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  // 保存局部规划轨迹结果
  void SCANPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  // 检查轨迹是否满足动力学约束
  bool SCANPlannerManager::checkDynamicFeasibility(UniformBspline position_traj)
  {
    UniformBspline vel_traj = position_traj.getDerivative();
    UniformBspline acc_traj = vel_traj.getDerivative();
    const double duration = position_traj.getTimeSum();
    const double sample_dt = std::max(0.01, std::min(0.05, duration / 50.0));
    const double vel_limit = pp_.max_vel_ + pp_.vel_tolerance_;
    const double acc_limit = pp_.max_acc_ + pp_.acc_tolerance_;

    for (double t = 0.0; t < duration + 1e-6; t += sample_dt)
    {
      const double tc = std::min(t, duration);
      Eigen::Vector3d vel = vel_traj.evaluateDeBoorT(tc);
      if (vel.norm() > vel_limit)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Dynamic feasibility failed: velocity at t=%.3f is %.3f > %.3f",
                    tc, vel.norm(), vel_limit);
        return false;
      }

      Eigen::Vector3d acc = acc_traj.evaluateDeBoorT(tc);
      if (acc.norm() > acc_limit)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Dynamic feasibility failed: acceleration at t=%.3f is %.3f > %.3f",
                    tc, acc.norm(), acc_limit);
        return false;
      }
    }

    return true;
  }

  //改变 B-spline 的执行时间（速度），然后重新计算一组等时间间隔的控制点，使新的 B-spline 保持原轨迹形状，同时满足新的时间约束
  void SCANPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace scan_planner
