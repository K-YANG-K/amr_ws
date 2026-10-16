#pragma once

#include <string>
#include <memory>
#include <vector>
#include <mutex>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "amr_costmap_2d/costmap_2d_ros.hpp"
#include "amr_costmap_2d/footprint_collision_checker.hpp"
#include "nlopt.hpp"

namespace mpc
{

// 前向声明 NLopt 回调函数（C 风格回调，需要 friend 访问私有成员）
double objectiveWrapper(const std::vector<double> & x,
                        std::vector<double> & grad, void * data);
struct ConstraintData;
double constraintWrapper(const std::vector<double> & x,
                         std::vector<double> & grad, void * data);

class Optimizer
{
    friend double mpc::objectiveWrapper(const std::vector<double> & x,
                                        std::vector<double> & grad, void * data);
    friend double mpc::constraintWrapper(const std::vector<double> & x,
                                         std::vector<double> & grad, void * data);
    friend struct mpc::ConstraintData;
public:
    Optimizer() = default;
    ~Optimizer() = default;

    void initialize(
        rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
        std::shared_ptr<amr_costmap_2d::Costmap2DROS> costmap_ros);

    geometry_msgs::msg::TwistStamped getControl(
        const geometry_msgs::msg::Pose & goal_pose,
        const geometry_msgs::msg::PoseStamped & carrot_pose,
        const geometry_msgs::msg::PoseStamped & amr_pose,
        const geometry_msgs::msg::Twist & speed);

    /// 运行时动态更新 MPC 参数
    void updateParameter(const std::string & name, double value);

protected:
    // ── ROS / Costmap 接口 ──────────────────────────────────────
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
    std::shared_ptr<amr_costmap_2d::Costmap2DROS> costmap_ros_;
    amr_costmap_2d::Costmap2D * costmap_;
    std::string name_;

    // 足迹碰撞检测器（复用 costmap 中已有的实现）
    amr_costmap_2d::FootprintCollisionChecker<amr_costmap_2d::Costmap2D *> collision_checker_;

    // ── MPC 运动约束参数（全部支持运行时动态修改）──────────────
    double acc_x_limit_;       // x 方向加速度上限
    double acc_y_limit_;       // y 方向加速度上限
    double acc_theta_limit_;   // 角加速度上限

    double min_vel_x_;         // x 速度下界
    double min_vel_y_;         // y 速度下界
    double min_vel_theta_;     // 角速度下界
    double max_vel_x_;         // x 速度上界
    double max_vel_y_;         // y 速度上界
    double max_vel_theta_;     // 角速度上界
    double max_vel_trans_;     // 平移合速度上限 √(vx²+vy²)

    // ── 代价函数权重（全部支持运行时动态修改）──────────────────
    double w_trans_;           // 位置追踪误差权重
    double w_orient_;          // 姿态追踪误差权重
    double w_control_;         // 控制量平滑权重
    double w_terminal_;        // 终端代价放大系数

    // ── 算法参数 ────────────────────────────────────────────────
    double low_pass_gain_;     // 低通滤波增益 (0~1)
    double opt_tolerance_;     // 优化器收敛容差
    double prediction_horizon_;// 预测时域 [秒]
    int    control_steps_;     // 控制步数 N
    double waiting_time_max_;  // 停障最大等待时间 [秒]

    // ── 求解器状态（跨迭代持久化）───────────────────────────────
    std::vector<double> last_control_;   // 上一帧实际输出的控制量 [vx, vy, vθ]
    std::vector<double> initial_guess_;  // 当前帧优化初值 (N×3)
    geometry_msgs::msg::Pose old_goal_;  // 上一帧的目标位姿（用于检测新目标）
    bool    collision_;                  // 当前是否处于碰撞停止状态
    bool    collision_footprint_;        // 当前足迹是否在障碍物中
    double  waiting_time_;               // 累计等待时间 [秒]
    rclcpp::Time last_time_;             // 上一帧时间戳
    bool    last_time_initialized_;      // 时间是否已初始化

    // 步长时间间隔 dt = prediction_horizon / control_steps
    double dt_;

    // ── 优化上下文（每次 getControl 前设置，供目标函数读取）────
    double target_yaw_;        // carrot 点的期望偏航角
    double final_yaw_;         // 最终目标点的期望偏航角
    double current_vx_, current_vy_, current_vtheta_;  // 当前机器人速度
    double carrot_x_, carrot_y_;  // carrot 点在局部坐标系下的坐标
    double goal_x_, goal_y_;      // 最终目标点在局部坐标系下的坐标

    // ── 辅助函数 ────────────────────────────────────────────────
    /// 从四元数提取偏航角
    static double getYaw(const geometry_msgs::msg::Quaternion & q);
    /// 初值热启动：将上一帧解算结果平移一个步长作为新初值
    void initialGuessUpdate(std::vector<double> & init_guess, const std::vector<double> & guess);
    /// 目标函数 J(x)：追踪代价 + 控制代价 + 终端代价
    double objective(const std::vector<double> & x, std::vector<double> & grad);
    /// 碰撞检测：沿预测轨迹逐步检查 cell 和 footprint 是否碰撞
    bool collisionCheck(const std::vector<double> & x,
                        const geometry_msgs::msg::PoseStamped & amr_pose,
                        const amr_costmap_2d::Footprint & footprint);

    // 参数修改互斥锁
    std::mutex param_mutex_;
};

}  // namespace mpc
