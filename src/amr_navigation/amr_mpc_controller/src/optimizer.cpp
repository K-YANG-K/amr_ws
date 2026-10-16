#include "amr_mpc_controller/optimizer.hpp"

#include <cmath>
#include <algorithm>
#include <functional>

#include "glog/logging.h"
#include "geometry_msgs/msg/point.hpp"
#include "amr_costmap_2d/cost_values.hpp"
#include "amr_util/node_utils.hpp"

namespace mpc
{

// ══════════════════════════════════════════════════════════════════════
// 初始化：注册 ROS 参数、设置初始状态
// ══════════════════════════════════════════════════════════════════════
void Optimizer::initialize(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
    std::shared_ptr<amr_costmap_2d::Costmap2DROS> costmap_ros)
{
    parent_ = parent;
    name_ = name;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();

    // 初始化碰撞检测器（直接操作 costmap 底层指针）
    collision_checker_ = amr_costmap_2d::FootprintCollisionChecker<amr_costmap_2d::Costmap2D *>(costmap_);

    auto node = parent_.lock();
    if (!node) {
        LOG(ERROR) << "Optimizer: 父节点为空，初始化失败";
        return;
    }

    // ── 声明并加载所有 MPC 参数 ──────────────────────────────────
    // 运动约束
    amr_util::declare_parameter_if_not_declared(node, name_ + ".acc_x_limit", rclcpp::ParameterValue(0.5));
    amr_util::declare_parameter_if_not_declared(node, name_ + ".acc_y_limit", rclcpp::ParameterValue(0.5));
    amr_util::declare_parameter_if_not_declared(node, name_ + ".acc_theta_limit", rclcpp::ParameterValue(0.5));

    // 速度边界
    amr_util::declare_parameter_if_not_declared(node, name_ + ".min_vel_x", rclcpp::ParameterValue(-0.5));
    amr_util::declare_parameter_if_not_declared(node, name_ + ".min_vel_y", rclcpp::ParameterValue(-0.5));
    amr_util::declare_parameter_if_not_declared(node, name_ + ".min_vel_theta", rclcpp::ParameterValue(-0.5));
    amr_util::declare_parameter_if_not_declared(node, name_ + ".max_vel_x", rclcpp::ParameterValue(0.5));
    amr_util::declare_parameter_if_not_declared(node, name_ + ".max_vel_y", rclcpp::ParameterValue(0.5));
    amr_util::declare_parameter_if_not_declared(node, name_ + ".max_vel_theta", rclcpp::ParameterValue(0.5));
    amr_util::declare_parameter_if_not_declared(node, name_ + ".max_vel_trans", rclcpp::ParameterValue(0.5));

    // 代价权重
    amr_util::declare_parameter_if_not_declared(node, name_ + ".w_trans", rclcpp::ParameterValue(0.5));
    amr_util::declare_parameter_if_not_declared(node, name_ + ".w_orient", rclcpp::ParameterValue(0.5));
    amr_util::declare_parameter_if_not_declared(node, name_ + ".w_control", rclcpp::ParameterValue(0.5));
    amr_util::declare_parameter_if_not_declared(node, name_ + ".w_terminal", rclcpp::ParameterValue(0.5));

    // 算法参数
    amr_util::declare_parameter_if_not_declared(node, name_ + ".low_pass_gain", rclcpp::ParameterValue(0.5));
    amr_util::declare_parameter_if_not_declared(node, name_ + ".opt_tolerance", rclcpp::ParameterValue(1e-5));
    amr_util::declare_parameter_if_not_declared(node, name_ + ".prediction_horizon", rclcpp::ParameterValue(0.5));
    amr_util::declare_parameter_if_not_declared(node, name_ + ".control_steps", rclcpp::ParameterValue(3));
    amr_util::declare_parameter_if_not_declared(node, name_ + ".waiting_time", rclcpp::ParameterValue(3.0));

    // 读取参数值
    node->get_parameter(name_ + ".acc_x_limit", acc_x_limit_);
    node->get_parameter(name_ + ".acc_y_limit", acc_y_limit_);
    node->get_parameter(name_ + ".acc_theta_limit", acc_theta_limit_);

    node->get_parameter(name_ + ".min_vel_x", min_vel_x_);
    node->get_parameter(name_ + ".min_vel_y", min_vel_y_);
    node->get_parameter(name_ + ".min_vel_theta", min_vel_theta_);
    node->get_parameter(name_ + ".max_vel_x", max_vel_x_);
    node->get_parameter(name_ + ".max_vel_y", max_vel_y_);
    node->get_parameter(name_ + ".max_vel_theta", max_vel_theta_);
    node->get_parameter(name_ + ".max_vel_trans", max_vel_trans_);

    node->get_parameter(name_ + ".w_trans", w_trans_);
    node->get_parameter(name_ + ".w_orient", w_orient_);
    node->get_parameter(name_ + ".w_control", w_control_);
    node->get_parameter(name_ + ".w_terminal", w_terminal_);

    node->get_parameter(name_ + ".low_pass_gain", low_pass_gain_);
    node->get_parameter(name_ + ".opt_tolerance", opt_tolerance_);
    node->get_parameter(name_ + ".prediction_horizon", prediction_horizon_);
    node->get_parameter(name_ + ".control_steps", control_steps_);
    node->get_parameter(name_ + ".waiting_time", waiting_time_max_);

    // ── 初始化求解器状态 ────────────────────────────────────────
    last_control_ = {0.0, 0.0, 0.0};
    initial_guess_.assign(control_steps_ * 3, 0.0);
    old_goal_ = geometry_msgs::msg::Pose();
    collision_ = false;
    collision_footprint_ = false;
    waiting_time_ = 0.0;
    last_time_initialized_ = false;

    dt_ = prediction_horizon_ / control_steps_;

    LOG(INFO) << "Optimizer 初始化完成: control_steps=" << control_steps_
              << ", dt=" << dt_ << "s, horizon=" << prediction_horizon_ << "s";
}

// ══════════════════════════════════════════════════════════════════════
// NLopt 约束数据结构体
// ══════════════════════════════════════════════════════════════════════
struct ConstraintData {
    Optimizer * self;   // 指向 Optimizer 实例
    int index;          // 当前约束对应的控制步序号 i
};

// ══════════════════════════════════════════════════════════════════════
// NLopt 目标函数回调（C 接口 → C++ 成员函数）
// ══════════════════════════════════════════════════════════════════════
double objectiveWrapper(const std::vector<double> & x,
                        std::vector<double> & grad, void * data)
{
    (void)grad;  // SLSQP 使用有限差分计算梯度，不需要手动提供
    auto * self = static_cast<Optimizer *>(data);
    return self->objective(x, grad);
}

// ══════════════════════════════════════════════════════════════════════
// NLopt 不等式约束回调：√(vx² + vy²) ≤ max_vel_trans
// 约束函数 g(x) ≥ 0 形式：max_vel_trans - √(vx²+vy²) ≥ 0
// ══════════════════════════════════════════════════════════════════════
double constraintWrapper(const std::vector<double> & x,
                         std::vector<double> & grad, void * data)
{
    (void)grad;
    auto * cd = static_cast<ConstraintData *>(data);
    int idx = cd->index;
    double vx = x[0 + idx * 3];
    double vy = x[1 + idx * 3];
    return cd->self->max_vel_trans_ - std::sqrt(vx * vx + vy * vy);
}

// ══════════════════════════════════════════════════════════════════════
// 四元数 → 偏航角
// ══════════════════════════════════════════════════════════════════════
double Optimizer::getYaw(const geometry_msgs::msg::Quaternion & q)
{
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

// ══════════════════════════════════════════════════════════════════════
// 目标函数 J(x) —— MPC 代价函数
//
// 决策变量 x = [vx₀ vy₀ ω₀, vx₁ vy₁ ω₁, ..., vxₙ₋₁ vyₙ₋₁ ωₙ₋₁]
// 共 N × 3 维，N = control_steps_
//
// 状态传播（全向轮运动学模型，在局部坐标系下）：
//   z_{k+1} = z_k + ω_k * dt
//   x_{k+1} = x_k + (vx_k * cos(z_k) - vy_k * sin(z_k)) * dt
//   y_{k+1} = y_k + (vx_k * sin(z_k) + vy_k * cos(z_k)) * dt
//
// 代价由三部分组成：
//   i)  追踪代价：每个预测步上，预测位姿与 carrot 点的位置误差 + 姿态误差
//   ii) 控制代价：每个预测步上，预测控制量与当前速度的偏差（平滑性）
//   iii) 终端代价：预测终点与最终目标点的误差 × w_terminal 放大
// ══════════════════════════════════════════════════════════════════════
double Optimizer::objective(const std::vector<double> & x,
                            std::vector<double> & grad)
{
    (void)grad;  // SLSQP 使用有限差分

    double cost_total = 0.0;
    double px = 0.0, py = 0.0, pz = 0.0;  // 局部坐标系下的预测状态

    // ── i) 追踪代价 + ii) 控制代价（逐步累加）─────────────────
    for (int i = 0; i < control_steps_; ++i) {
        // 状态传播
        pz += x[2 + 3 * i] * dt_;
        px += (x[0 + 3 * i] * std::cos(pz) - x[1 + 3 * i] * std::sin(pz)) * dt_;
        py += (x[0 + 3 * i] * std::sin(pz) + x[1 + 3 * i] * std::cos(pz)) * dt_;

        // 追踪代价：位置误差² + 姿态误差²（直接用平方和，省掉 sqrt）
        double dx = carrot_x_ - px;
        double dy = carrot_y_ - py;
        double step_dist_error_sq = dx * dx + dy * dy;
        double step_orient_error = target_yaw_ - pz;

        cost_total += (w_trans_ * step_dist_error_sq +
                       w_orient_ * step_orient_error * step_orient_error) /
                      control_steps_;

        // 控制平滑代价：与当前速度的偏差
        double dvx = current_vx_ - x[0 + 3 * i];
        double dvy = current_vy_ - x[1 + 3 * i];
        double dvz = current_vtheta_ - x[2 + 3 * i];
        cost_total += w_control_ * (dvx * dvx + dvy * dvy + dvz * dvz) /
                      control_steps_;
    }

    // ── iii) 终端代价：预测终点与最终目标点的偏差 ─────────────
    double dx_term = goal_x_ - px;
    double dy_term = goal_y_ - py;
    double term_dist_error_sq = dx_term * dx_term + dy_term * dy_term;
    double term_orient_error = final_yaw_ - pz;
    cost_total += (w_trans_ * term_dist_error_sq +
                   w_orient_ * term_orient_error * term_orient_error) *
                  w_terminal_;

    return cost_total;
}

// ══════════════════════════════════════════════════════════════════════
// 碰撞检测：沿预测轨迹逐步检查
//
// 两重检查：
//   1) 中心点检查：预测位置的 cell cost ≥ LETHAL_OBSTACLE
//   2) 足迹检查：预测位姿下，机器人 footprint 覆盖的 cell 是否有障碍
//
// 注意：使用的是全局坐标系下的运动学传播（odom 坐标系），
// 因为 costmap 在全局坐标系下索引。
// ══════════════════════════════════════════════════════════════════════
bool Optimizer::collisionCheck(const std::vector<double> & x,
                               const geometry_msgs::msg::PoseStamped & amr_pose,
                               const amr_costmap_2d::Footprint & footprint)
{
    if (footprint.empty()) {
        return false;
    }

    double odom_yaw = getYaw(amr_pose.pose.orientation);
    double pos_x = amr_pose.pose.position.x;
    double pos_y = amr_pose.pose.position.y;

    // 先检查当前位置的 footprint 是否已在障碍物中
    double current_fp_cost = collision_checker_.footprintCostAtPose(
        pos_x, pos_y, odom_yaw, footprint);
    if (current_fp_cost >= static_cast<double>(amr_costmap_2d::LETHAL_OBSTACLE)) {
        LOG(WARNING) << "当前位姿足迹已在障碍物中，停车";
        return true;
    }

    // 逐步检查预测轨迹
    for (int i = 0; i < control_steps_; ++i) {
        // 全局坐标系下的状态传播
        odom_yaw += x[2 + 3 * i] * dt_;
        pos_x += (x[0 + 3 * i] * std::cos(odom_yaw) -
                  x[1 + 3 * i] * std::sin(odom_yaw)) * dt_;
        pos_y += (x[0 + 3 * i] * std::sin(odom_yaw) +
                  x[1 + 3 * i] * std::cos(odom_yaw)) * dt_;

        // 检查 1：中心点是否在障碍物 cell 上
        unsigned int mx, my;
        if (costmap_->worldToMap(pos_x, pos_y, mx, my)) {
            unsigned char cell_cost = costmap_->getCost(mx, my);
            if (cell_cost >= amr_costmap_2d::LETHAL_OBSTACLE) {
                LOG(WARNING) << "第 " << i << " 步预测点碰撞"
                             << " (cost=" << static_cast<int>(cell_cost) << ")，停车";
                return true;
            }
        }

        // 检查 2：预测位姿下的机器人 footprint 是否碰撞
        double fp_cost = collision_checker_.footprintCostAtPose(
            pos_x, pos_y, odom_yaw, footprint);
        if (fp_cost >= static_cast<double>(amr_costmap_2d::LETHAL_OBSTACLE)) {
            LOG(WARNING) << "第 " << i << " 步足迹碰撞"
                         << " (cost=" << fp_cost << ")，停车";
            return true;
        }
    }

    return false;
}

// ══════════════════════════════════════════════════════════════════════
// 初值热启动：将上一帧的解向前平移一个步长
//
// 变换逻辑：
//   new_guess[0..N-2] = old_solution[1..N-1]   （向前滑动）
//   new_guess[N-1]    = old_solution[0]          （末尾重复第一个）
//
// 这样每次优化的初值更接近真实最优解，加速收敛。
// ══════════════════════════════════════════════════════════════════════
void Optimizer::initialGuessUpdate(std::vector<double> & init_guess,
                                   const std::vector<double> & guess)
{
    for (int i = 0; i < control_steps_ - 1; ++i) {
        init_guess[0 + 3 * i] = guess[3 + 3 * i];
        init_guess[1 + 3 * i] = guess[4 + 3 * i];
        init_guess[2 + 3 * i] = guess[5 + 3 * i];
    }
    int last = control_steps_ - 1;
    init_guess[0 + 3 * last] = guess[0];
    init_guess[1 + 3 * last] = guess[1];
    init_guess[2 + 3 * last] = guess[2];
}

// ══════════════════════════════════════════════════════════════════════
// 主函数：计算控制量
//
// 完整流程：
//   1. 检测新目标 → 重置状态
//   2. 构建 NLopt SLSQP 优化问题
//      - 目标函数：追踪 + 平滑 + 终端
//      - 变量边界：各速度分量上下界
//      - 不等式约束：√(vx²+vy²) ≤ max_vel_trans
//   3. 求解 → 得到最优控制序列 x*
//   4. 后处理：低通滤波 → 加速度限幅 → 碰撞检测
//   5. 碰撞停障 → 等待超时后恢复
//   6. 更新热启动初值
// ══════════════════════════════════════════════════════════════════════
geometry_msgs::msg::TwistStamped Optimizer::getControl(
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::PoseStamped & carrot_pose,
    const geometry_msgs::msg::PoseStamped & amr_pose,
    const geometry_msgs::msg::Twist & speed)
{
    geometry_msgs::msg::TwistStamped cmd_speed;
    cmd_speed.header.frame_id = costmap_ros_->getBaseFrameID();
    cmd_speed.header.stamp = rclcpp::Clock().now();

    auto node = parent_.lock();
    if (!node) {
        return cmd_speed;
    }

    // ── 1. 计算时间间隔 ────────────────────────────────────────
    rclcpp::Time now = node->now();
    double delta_t = 0.0;
    if (last_time_initialized_) {
        delta_t = (now - last_time_).seconds();
        if (delta_t < 0.0) delta_t = 0.0;  // 防止时钟跳变
    }
    last_time_ = now;
    last_time_initialized_ = true;

    // ── 2. 检测新目标 → 重置求解器状态 ─────────────────────────
    if (old_goal_.position.x != goal_pose.position.x ||
        old_goal_.position.y != goal_pose.position.y ||
        old_goal_.orientation.z != goal_pose.orientation.z ||
        old_goal_.orientation.w != goal_pose.orientation.w) {
        std::fill(initial_guess_.begin(), initial_guess_.end(), 0.0);
        std::fill(last_control_.begin(), last_control_.end(), 0.0);
        waiting_time_ = 0.0;
        collision_ = false;
        collision_footprint_ = false;
    }

    // ── 3. 准备优化上下文 ──────────────────────────────────────
    // 这些变量在目标函数 objective() 中被读取
    target_yaw_ = getYaw(carrot_pose.pose.orientation);
    final_yaw_ = getYaw(goal_pose.orientation);
    current_vx_ = speed.linear.x;
    current_vy_ = speed.linear.y;
    current_vtheta_ = speed.angular.z;
    carrot_x_ = carrot_pose.pose.position.x;
    carrot_y_ = carrot_pose.pose.position.y;
    goal_x_ = goal_pose.position.x;
    goal_y_ = goal_pose.position.y;

    dt_ = prediction_horizon_ / control_steps_;
    int n_vars = control_steps_ * 3;

    // ── 4. 构建 NLopt SLSQP 优化问题 ───────────────────────────
    nlopt::opt opt(nlopt::LD_SLSQP, n_vars);

    // 目标函数
    opt.set_min_objective(objectiveWrapper, this);

    // 收敛容差
    opt.set_ftol_rel(opt_tolerance_);
    opt.set_xtol_rel(1e-8);

    // 变量边界：每个控制步的速度分量上下界
    std::vector<double> lb(n_vars), ub(n_vars);
    for (int i = 0; i < control_steps_; ++i) {
        lb[0 + 3 * i] = min_vel_x_;        ub[0 + 3 * i] = max_vel_x_;
        lb[1 + 3 * i] = min_vel_y_;        ub[1 + 3 * i] = max_vel_y_;
        lb[2 + 3 * i] = min_vel_theta_;    ub[2 + 3 * i] = max_vel_theta_;
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);

    // 不等式约束：每步的平移合速度不超过 max_vel_trans
    // g_i(x) = max_vel_trans - √(vx_i² + vy_i²) ≥ 0
    std::vector<ConstraintData> constraint_data(control_steps_);
    for (int i = 0; i < control_steps_; ++i) {
        constraint_data[i].self = this;
        constraint_data[i].index = i;
        opt.add_inequality_constraint(constraintWrapper, &constraint_data[i], 1e-8);
    }

    // ── 5. 求解优化问题 ────────────────────────────────────────
    std::vector<double> x = initial_guess_;  // 拷贝初值
    double min_cost = 0.0;
    bool success = false;

    try {
        nlopt::result result = opt.optimize(x, min_cost);
        success = (result == nlopt::SUCCESS ||
                   result == nlopt::FTOL_REACHED ||
                   result == nlopt::XTOL_REACHED ||
                   result == nlopt::MAXEVAL_REACHED);
        if (!success) {
            LOG(WARNING) << "NLopt 返回非完全成功状态: " << static_cast<int>(result);
        }
    } catch (const std::exception & e) {
        LOG(ERROR) << "NLopt 优化异常: " << e.what();
        // 求解失败时输出零速度
        cmd_speed.twist.linear.x = 0.0;
        cmd_speed.twist.linear.y = 0.0;
        cmd_speed.twist.angular.z = 0.0;
        return cmd_speed;
    }

    // ── 6. 后处理：低通滤波 ────────────────────────────────────
    // cmd = x*[0] * gain + last_cmd * (1 - gain)
    // 目的：抑制帧间控制量突变，提高平顺性
    double filtered_vx = x[0] * low_pass_gain_ + last_control_[0] * (1.0 - low_pass_gain_);
    double filtered_vy = x[1] * low_pass_gain_ + last_control_[1] * (1.0 - low_pass_gain_);
    double filtered_vz = x[2] * low_pass_gain_ + last_control_[2] * (1.0 - low_pass_gain_);

    // ── 7. 碰撞检测 ────────────────────────────────────────────
    auto footprint = costmap_ros_->getRobotFootprint();
    bool collision_now = collisionCheck(x, amr_pose, footprint);

    // 同时检测当前位姿的 footprint
    double odom_yaw = getYaw(amr_pose.pose.orientation);
    double fp_at_current = collision_checker_.footprintCostAtPose(
        amr_pose.pose.position.x, amr_pose.pose.position.y, odom_yaw, footprint);
    bool footprint_collision_now =
        (fp_at_current >= static_cast<double>(amr_costmap_2d::LETHAL_OBSTACLE));

    if (collision_now || footprint_collision_now) {
        // ── 碰撞 → 停车等待 ──────────────────────────────────
        collision_ = true;
        waiting_time_ += delta_t;
        LOG(WARNING) << "检测到障碍物，停车等待。已等待 " << waiting_time_
                     << "s / " << waiting_time_max_ << "s";

        cmd_speed.twist.linear.x = 0.0;
        cmd_speed.twist.linear.y = 0.0;
        cmd_speed.twist.angular.z = 0.0;

        // 等待超时后清除碰撞标记，允许机器人继续
        if (waiting_time_ >= waiting_time_max_) {
            collision_ = false;
            waiting_time_ = 0.0;
            LOG(INFO) << "等待时间到期，清除碰撞标记，允许继续行驶";
        }
    } else {
        // ── 无碰撞 → 加速度限幅后输出 ─────────────────────────
        // 上限：cmd ≤ last_cmd + acc_limit * dt
        double temp_vx = std::min(filtered_vx, last_control_[0] + acc_x_limit_ * delta_t);
        double temp_vy = std::min(filtered_vy, last_control_[1] + acc_y_limit_ * delta_t);
        double temp_vz = std::min(filtered_vz, last_control_[2] + acc_theta_limit_ * delta_t);

        // 下限：cmd ≥ last_cmd - acc_limit * dt
        cmd_speed.twist.linear.x  = std::max(temp_vx, last_control_[0] - acc_x_limit_ * delta_t);
        cmd_speed.twist.linear.y  = std::max(temp_vy, last_control_[1] - acc_y_limit_ * delta_t);
        cmd_speed.twist.angular.z = std::max(temp_vz, last_control_[2] - acc_theta_limit_ * delta_t);
    }

    // ── 8. 保存状态 ────────────────────────────────────────────
    last_control_[0] = cmd_speed.twist.linear.x;
    last_control_[1] = cmd_speed.twist.linear.y;
    last_control_[2] = cmd_speed.twist.angular.z;

    // ── 9. 更新热启动初值 ──────────────────────────────────────
    if (success) {
        initialGuessUpdate(initial_guess_, x);
    } else {
        initial_guess_ = x;  // 即使未完全收敛，也保留结果作为暖启动
    }

    old_goal_ = goal_pose;

    return cmd_speed;
}

// ══════════════════════════════════════════════════════════════════════
// 动态参数更新
//
// 由 MPCController::dynamicParametersCallback 调用，
// 支持运行时通过 ROS 参数服务修改 MPC 参数，无需重启。
// ══════════════════════════════════════════════════════════════════════
void Optimizer::updateParameter(const std::string & name, double value)
{
    std::lock_guard<std::mutex> lock(param_mutex_);

    // 去掉命名空间前缀，提取参数短名
    std::string short_name = name;
    auto pos = name.rfind('.');
    if (pos != std::string::npos) {
        short_name = name.substr(pos + 1);
    }

    if (short_name == "acc_x_limit")           acc_x_limit_ = value;
    else if (short_name == "acc_y_limit")      acc_y_limit_ = value;
    else if (short_name == "acc_theta_limit")  acc_theta_limit_ = value;
    else if (short_name == "min_vel_x")        min_vel_x_ = value;
    else if (short_name == "min_vel_y")        min_vel_y_ = value;
    else if (short_name == "min_vel_theta")    min_vel_theta_ = value;
    else if (short_name == "max_vel_x")        max_vel_x_ = value;
    else if (short_name == "max_vel_y")        max_vel_y_ = value;
    else if (short_name == "max_vel_theta")    max_vel_theta_ = value;
    else if (short_name == "max_vel_trans")    max_vel_trans_ = value;
    else if (short_name == "w_trans")          w_trans_ = value;
    else if (short_name == "w_orient")         w_orient_ = value;
    else if (short_name == "w_control")        w_control_ = value;
    else if (short_name == "w_terminal")       w_terminal_ = value;
    else if (short_name == "low_pass_gain")    low_pass_gain_ = value;
    else if (short_name == "opt_tolerance")    opt_tolerance_ = value;
    else if (short_name == "prediction_horizon") {
        prediction_horizon_ = value;
        dt_ = prediction_horizon_ / control_steps_;
    }
    else if (short_name == "control_steps") {
        control_steps_ = static_cast<int>(value);
        dt_ = prediction_horizon_ / control_steps_;
        // 步数变化时需要重新分配状态向量
        initial_guess_.assign(control_steps_ * 3, 0.0);
        last_control_ = {0.0, 0.0, 0.0};
    }
    else if (short_name == "waiting_time")     waiting_time_max_ = value;
}

}  // namespace mpc
