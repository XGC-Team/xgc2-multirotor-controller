#pragma once

#include <ros/ros.h>
#include <ros1_utils/topic_stats.h>

#include <Eigen/Dense>  // 用于 MpcTrajectoryState
#include <cstdint>
#include <string>

namespace px4_multirotor_controller {

// ============ 业务相关类型ID定义 ============

// 控制模式枚举（用于 Custom1State MPC轨迹跟踪）
enum class ControlMode {
    PX4_CASCADE_PID = 0,  // PX4串级PID：发送 pos+vel+acc（前馈），type_mask=0b111111000111
    PURE_SLIDING_MODE = 1,  // 纯滑模控制：只发送 acc=u_h+u_f，type_mask=0b011111000111
    HYBRID_CONTROL = 2  // 混合控制：发送 pos+vel+acc（滑模），type_mask=0b111111000111
};

// 跟踪后端选择。默认 legacy_mpc_lifter 保持现有 Custom1State 行为。
enum class TrackingBackend { LEGACY_MPC_LIFTER = 0, NMPC_ATTITUDE_RATE = 1 };

// 控制器配置参数
struct ControllerConfig {
    double takeoff_altitude{1.5};  // 起飞目标高度（米）
    bool skip_takeoff_init_disarm{false};  // 是否跳过起飞初始化阶段的 DISARM 与 ALTCTL 安全门
    double planning_period{0.1};  // MPC 离散规划周期（秒），默认 10Hz

    // ========== MPC轨迹跟踪控制模式 ==========
    ControlMode control_mode{ControlMode::PX4_CASCADE_PID};  // 默认使用PX4控制
    TrackingBackend tracking_backend{TrackingBackend::LEGACY_MPC_LIFTER};

    // 滑模控制器参数（用于 PURE_SLIDING_MODE 和 HYBRID_CONTROL 模式）
    // 理论参考：论文公式(26)(27)(28)
    struct SlidingModeParams {
        double k1{3.0};       // 控制增益
        double k2{3.0};       // 滑模参数（影响收敛速度）
        double epsilon{0.5};  // 边界层厚度（减小抖振）
    } sliding_mode;

    // ========== 偏航角控制开关 ==========
    bool enable_yaw_control{false};  // 是否启用偏航角控制（false=忽略偏航角）

    // ========== UAV NMPC 后端参数 ==========
    struct UavNmpcParams {
        double control_period{0.01};
        double prediction_horizon{1.0};
        double gravity{9.8066};
        double hover_thrust_ratio{0.5};
        double min_hover_thrust{0.05};
        double max_hover_thrust{0.95};
        double specific_thrust_min{0.0};
        double specific_thrust_max{20.373};
        bool hover_thrust_enabled{false};
        double hover_thrust_timeout{0.5};
        double solve_timeout{0.03};
        double result_timeout{0.1};
        double reference_timeout{0.5};
        double reference_start_delay{0.2};
        double reference_duration{60.0};
        double reference_radius{3.0};
        double reference_line_speed{3.0};
        double reference_height{3.0};
        double reference_z_amplitude{1.0};
        double reference_z_frequency{0.5};
        double reference_entry_duration{5.0};

        bool enable_timing_log{true};
        double log_period{1.0};
    } nmpc;

    // 安全检查限制
    struct SafetyLimits {
        // 传感器超时阈值（秒）
        double timeout_local_pos{1.0};
        double timeout_local_velocity{1.0};
        double timeout_imu{0.5};
        double timeout_state{2.0};
        double timeout_battery{5.0};

        // 位置跳变检测（安全检查）
        double position_jump_threshold{0.3};  // 位置跳变阈值（米）

        // 通信质量阈值
        double max_dt_local_pos{0.2};  // 最大帧间隔（秒）
        double max_dt_local_velocity{0.2};
        double max_dt_imu{0.1};
        double max_dt_state{0.5};

        double max_jitter_local_pos{0.05};  // 最大抖动（秒）
        double max_jitter_local_velocity{0.05};
        double max_jitter_imu{0.02};
        double max_jitter_state{0.1};

        // 地理围栏（米）
        double fence_x_min{-10.0}, fence_x_max{10.0};
        double fence_y_min{-10.0}, fence_y_max{10.0};
        double fence_z_min{0.0}, fence_z_max{3.0};

        // 速度限制（m/s）
        double max_velocity_xy{5.0};
        double max_velocity_z{2.0};

        // 控制饱和检测（m/s²）
        double acc_saturation_xy{3.0};
        double acc_saturation_z{3.0};

        // 电池（百分比，0.0-1.0）
        double battery_low{0.3};        // 30%
        double battery_critical{0.15};  // 15%
    } safety;
};

// 状态类型ID常量（业务相关，定义飞行状态）
// 使用 constexpr 保证编译时常量
namespace state_type {
constexpr uint32_t HealthMonitor = 100;  // 并行健康检查逻辑状态
constexpr uint32_t DebugMonitor = 101;   // 并行调试/观测输出逻辑状态

// 启动序列状态
constexpr uint32_t SelfCheck = 1;  // 自检中
constexpr uint32_t Ready = 2;      // 就绪
constexpr uint32_t Normal = 8;     // 正常飞行父状态

// 起飞序列状态
constexpr uint32_t TakeoffInit = 3;             // 起飞初始化
constexpr uint32_t TakeoffOffboardRequest = 4;  // 请求OFFBOARD模式
constexpr uint32_t TakeoffArmRequest = 5;       // 请求ARM解锁
constexpr uint32_t TakeoffAscending = 6;        // 上升中
constexpr uint32_t Takeoff = 10;                // 起飞父状态

// 飞行状态
constexpr uint32_t Hover = 7;    // 悬停
constexpr uint32_t Landing = 9;  // 降落

// 特殊状态
constexpr uint32_t Custom1 = 11;  // 自定义状态1
}  // namespace state_type

namespace region_type {
constexpr uint32_t HEALTH = 1;
constexpr uint32_t CONTROL = 2;
constexpr uint32_t DEBUG = 3;
}  // namespace region_type

// 事件类型ID常量（业务相关，定义具体事件）
// 使用 constexpr 保证编译时常量
namespace event_type {
// 未知事件
constexpr uint32_t UNKNOWN = 0;

// 外部输入事件
constexpr uint32_t CMD_TAKEOFF = 1;
constexpr uint32_t CMD_LAND = 2;
constexpr uint32_t CMD_HOVER = 3;
constexpr uint32_t CMD_CUSTOM1 = 6;

constexpr uint32_t INPUT_LOCAL_POSITION_UPDATED = 50;
constexpr uint32_t INPUT_LOCAL_VELOCITY_UPDATED = 51;
constexpr uint32_t INPUT_IMU_UPDATED = 52;
constexpr uint32_t INPUT_FCU_STATE_UPDATED = 53;
constexpr uint32_t INPUT_BATTERY_UPDATED = 54;
constexpr uint32_t INPUT_VRPN_POSE_UPDATED = 55;
constexpr uint32_t INPUT_VRPN_TWIST_UPDATED = 56;
constexpr uint32_t INPUT_MPC_TRAJECTORY_UPDATED = 57;
constexpr uint32_t INPUT_HOVER_THRUST_UPDATED = 58;
constexpr uint32_t INPUT_REFERENCE_TRAJECTORY_UPDATED = 59;
constexpr uint32_t INPUT_NMPC_SOLVE_SUCCEEDED = 60;
constexpr uint32_t INPUT_NMPC_SOLVE_FAILED = 61;
constexpr uint32_t INPUT_NMPC_SOLVE_TIMED_OUT = 62;
constexpr uint32_t INPUT_UAV_STATE_ESTIMATE_UPDATED = 63;
constexpr uint32_t INPUT_REFERENCE_TRAJECTORY_LOST = 64;

// 内部条件事件
constexpr uint32_t ALTITUDE_REACHED = 10;
constexpr uint32_t POSITION_REACHED = 11;
constexpr uint32_t TIMEOUT = 12;
constexpr uint32_t TOUCHDOWN = 13;        // 触地（降落完成）
constexpr uint32_t LANDING_TIMEOUT = 14;  // 降落超时
constexpr uint32_t TAKEOFF_TIMEOUT = 15;  // 起飞超时
constexpr uint32_t ALTCTL_READY = 16;  // 定高模式就绪（接收到足够的ALTCTL状态帧）
constexpr uint32_t OFFBOARD_READY = 17;  // OFFBOARD模式就绪（接收到足够的OFFBOARD状态帧）
constexpr uint32_t ARM_READY = 18;       // ARM解锁就绪（接收到足够的ARM状态帧）
constexpr uint32_t REFERENCE_TRAJECTORY_FINISHED = 19;  // 跟踪参考正常结束，保护性退回悬停

// 运行时安全检查事件（ID 20-49）- 边沿触发，仅在状态变化时发布
// 传感器超时
constexpr uint32_t SAFE_TIMEOUT_LOCAL_POS = 20;
constexpr uint32_t SAFE_TIMEOUT_LOCAL_VELOCITY = 21;
constexpr uint32_t SAFE_TIMEOUT_IMU = 22;
constexpr uint32_t SAFE_TIMEOUT_STATE = 23;
constexpr uint32_t SAFE_TIMEOUT_BATTERY = 24;
constexpr uint32_t SAFE_TIMEOUT_VRPN_POSE = 25;
constexpr uint32_t SAFE_TIMEOUT_VRPN_TWIST = 26;
constexpr uint32_t SAFE_TIMEOUT_UAV_STATE_ESTIMATE = 27;
constexpr uint32_t SAFE_UAV_STATE_ESTIMATE_UNUSABLE = 28;

// 通信质量 - dt（帧间隔）
constexpr uint32_t SAFE_DEGRADED_DT_LOCAL_POS = 30;
constexpr uint32_t SAFE_DEGRADED_DT_LOCAL_VELOCITY = 31;
constexpr uint32_t SAFE_DEGRADED_DT_IMU = 32;
constexpr uint32_t SAFE_DEGRADED_DT_STATE = 33;

// 通信质量 - jitter（抖动）
constexpr uint32_t SAFE_DEGRADED_JITTER_LOCAL_POS = 34;
constexpr uint32_t SAFE_DEGRADED_JITTER_LOCAL_VELOCITY = 35;
constexpr uint32_t SAFE_DEGRADED_JITTER_IMU = 36;
constexpr uint32_t SAFE_DEGRADED_JITTER_STATE = 37;

// 地理围栏和速度
constexpr uint32_t SAFE_GEOFENCE_VIOLATION = 40;
constexpr uint32_t SAFE_VELOCITY_XY_EXCEEDED = 41;
constexpr uint32_t SAFE_VELOCITY_Z_EXCEEDED = 42;

// 控制饱和和电池
constexpr uint32_t SAFE_CONTROL_SATURATION_XY = 43;
constexpr uint32_t SAFE_CONTROL_SATURATION_Z = 44;
constexpr uint32_t SAFE_LOW_BATTERY = 45;
constexpr uint32_t SAFE_CRITICAL_BATTERY = 46;

// 位置跳变检测
constexpr uint32_t SAFE_POSITION_JUMP = 47;  // 本地位置跳变

// VRPN 质量事件
constexpr uint32_t SAFE_VRPN_EFFECTIVE_FREQ_LOW = 48;  // VRPN有效频率过低
constexpr uint32_t SAFE_VRPN_POSITION_JUMP = 49;       // VRPN位置跳变
}  // namespace event_type

namespace output_event_type {
constexpr uint32_t REQUEST_ARMING = 10000;
constexpr uint32_t REQUEST_KILL = 10001;
constexpr uint32_t REQUEST_MODE = 10002;
constexpr uint32_t PUBLISH_SETPOINT = 10004;
constexpr uint32_t PUBLISH_ATTITUDE_RATE_TARGET = 10005;
constexpr uint32_t REQUEST_NMPC_SOLVE = 10006;
constexpr uint32_t PUBLISH_CONTROLLER_STATUS = 10007;
constexpr uint32_t PUBLISH_SENSOR_STATS = 10008;
constexpr uint32_t PUBLISH_STATE_MACHINE_EVENTS = 10009;
constexpr uint32_t PRINT_SENSOR_DEBUG = 10010;
constexpr uint32_t PUBLISH_TRACKING_ERROR = 10012;
constexpr uint32_t PUBLISH_REFERENCE_TRAJECTORY_ACTIVATION = 10013;
}  // namespace output_event_type

// 转移优先级常量
namespace transition_priority {
// 紧急事件：最高优先级，安全相关
constexpr int EMERGENCY = 100;  // 紧急停止、严重故障
constexpr int CRITICAL = 90;    // 传感器失效、电池低等

// 用户命令：高优先级，响应用户操作
constexpr int USER_COMMAND = 80;  // 降落命令等关键操作
constexpr int COMMAND = 50;       // 一般用户命令

// 自动条件：低优先级，自动化转移
constexpr int AUTOMATIC = 20;  // 高度到达、位置到达
constexpr int DEFAULT = 0;     // 默认优先级
}  // namespace transition_priority

// ============ 数据结构 ============

// 传感器数据（聚合）
// 包含所有传感器测量值，字段名与 ROS 消息保持一致
struct SensorData {
    // 控制状态 (m, m/s, quaternion, rad/s) - 来自 estimator_vrpn_px4_rotor_state/RigidStateEstimate
    double x{0.0}, y{0.0}, z{0.0};

    // 线速度 (m/s)
    double vx{0.0}, vy{0.0}, vz{0.0};

    // 姿态四元数 - 来自 geometry_msgs/PoseStamped 或 sensor_msgs/Imu
    // 使用四元数避免欧拉角奇异点
    double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};

    // 角速度 (rad/s)
    double wx{0.0}, wy{0.0}, wz{0.0};
    uint8_t uav_state_estimator_state{0};
    uint32_t uav_state_estimator_flags{0};
    double uav_state_estimate_stamp{0.0};

    // 悬停推力估计 - 来自 hover_thrust_estimator/hover_thrust/estimate_state
    double hover_thrust_estimate{0.0};
    double hover_thrust_estimate_stamp{0.0};
    bool hover_thrust_estimate_available{false};
    uint32_t hover_thrust_estimate_flags{0};

    // 电池数据 - 来自 sensor_msgs/BatteryState
    double battery_percentage{0.0};  // 电量百分比 (0.0-1.0)

    // ========== 飞控状态（来自 mavros_msgs/State）==========
    bool fcu_connected{false};     // 飞控连接状态
    bool fcu_armed{false};         // 电机解锁状态
    bool fcu_guided{false};        // 引导模式
    bool fcu_manual_input{false};  // 遥控器输入状态（检测RC接管）
    std::string fcu_mode{""};      // 当前飞行模式（如 "OFFBOARD"）
    uint8_t fcu_system_status{0};  // 系统状态码（MAV_STATE 原始值）

    // 话题统计数据
    using TopicStats = ros1_utils::TopicStats;
    TopicStats uav_state_estimate_stats, local_pos_stats, local_velocity_stats, imu_stats,
        state_stats, battery_stats;

    // MAVROS 本地位置，仅用于一致性检查/诊断，不作为控制状态源
    double local_x{0.0}, local_y{0.0}, local_z{0.0};
    double local_qx{0.0}, local_qy{0.0}, local_qz{0.0}, local_qw{1.0};
    double local_vx{0.0}, local_vy{0.0}, local_vz{0.0};

    // VRPN 原始数据（独立于 mavros）
    // 位置 (m) - 来自 pose 话题
    double vrpn_x{0.0}, vrpn_y{0.0}, vrpn_z{0.0};
    double vrpn_qx{0.0}, vrpn_qy{0.0}, vrpn_qz{0.0}, vrpn_qw{1.0};

    // 速度 (m/s) 和角速度 (rad/s) - 来自 twist 话题
    double vrpn_vx{0.0}, vrpn_vy{0.0}, vrpn_vz{0.0};
    double vrpn_wx{0.0}, vrpn_wy{0.0}, vrpn_wz{0.0};

    // VRPN 话题统计
    TopicStats vrpn_pose_stats;   // pose 话题统计
    TopicStats vrpn_twist_stats;  // twist 话题统计

    SensorData() = default;
};

// 目标设定点
// 字段名与 mavros_msgs/PositionTarget 保持一致
struct Setpoint {
    // 位置目标 (m) - 对应 position.{x,y,z}
    double x{0.0}, y{0.0}, z{0.0};

    // 速度目标 (m/s) - 对应 velocity.{x,y,z}
    double vx{0.0}, vy{0.0}, vz{0.0};

    // 加速度目标 (m/s^2) - 对应 acceleration_or_force.{x,y,z}
    double ax{0.0}, ay{0.0}, az{0.0};

    // 姿态目标（四元数）- 对应 orientation.{x,y,z,w}
    // 注意：MAVROS PositionTarget 不支持四元数姿态，仅支持 yaw
    // 这里保留四元数用于内部计算，发布时转换为 yaw
    double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};

    // 偏航角速度目标 (rad/s) - 对应 yaw_rate
    double yaw_rate{0.0};

    // 类型掩码：控制哪些字段被忽略
    // 位定义（与 MAVROS PositionTarget.type_mask 完全兼容）:
    //   bit 0 (1):    IGNORE_PX       - 忽略 x
    //   bit 1 (2):    IGNORE_PY       - 忽略 y
    //   bit 2 (4):    IGNORE_PZ       - 忽略 z
    //   bit 3 (8):    IGNORE_VX       - 忽略 vx
    //   bit 4 (16):   IGNORE_VY       - 忽略 vy
    //   bit 5 (32):   IGNORE_VZ       - 忽略 vz
    //   bit 6 (64):   IGNORE_AFX      - 忽略 ax
    //   bit 7 (128):  IGNORE_AFY      - 忽略 ay
    //   bit 8 (256):  IGNORE_AFZ      - 忽略 az
    //   bit 9 (512):  FORCE           - 强制设定点
    //   bit 10 (1024): IGNORE_YAW     - 忽略 yaw
    //   bit 11 (2048): IGNORE_YAW_RATE - 忽略 yaw_rate
    //
    // 示例：
    //   位置控制: type_mask = 0b111111111000 = 3576
    //             (忽略速度、加速度、偏航角速度，仅使用位置和偏航)
    //   速度控制: type_mask = 0b111111000111 = 4039
    //             (忽略位置、加速度、偏航角速度，仅使用速度和偏航)
    //   加速度+偏航控制: type_mask = 0b100000111111 = 2111
    //             (忽略位置、速度、偏航角速度，仅使用加速度和偏航)
    uint16_t type_mask{0};

    // 坐标系：对应 coordinate_frame
    // mavros_msgs::PositionTarget 定义的坐标系常量：
    //   FRAME_LOCAL_NED = 1   - 本地北-东-地坐标系（实际是ENU，MAVROS会转换）
    //   FRAME_LOCAL_OFFSET_NED = 7 - 相对于当前位置的偏移
    //   FRAME_BODY_NED = 8    - 机体坐标系
    //   FRAME_BODY_OFFSET_NED = 9 - 相对于当前机体的偏移
    // 默认值：FRAME_LOCAL_NED (1)
    uint8_t coordinate_frame{1};

    Setpoint() = default;
};

// UAV NMPC 输出：body-rate + normalized thrust -> mavros/setpoint_raw/attitude
struct AttitudeRateTarget {
    double body_rate_x{0.0};
    double body_rate_y{0.0};
    double body_rate_z{0.0};
    double thrust{0.0};

    AttitudeRateTarget() = default;
};

// MPC轨迹状态（用于连续轨迹生成）
// 基于状态空间模型：h_i(t) = e^{A*Δt} * E * δ_i(k) + ∫ e^{A*(t-s)} * B * τ_i(k)
// ds
struct MpcTrajectoryState {
    // 离散规划结果（来自 MPC 规划周期）
    Eigen::Vector3d position_k;      // δ_p(k)：离散位置偏移
    Eigen::Vector3d velocity_k;      // δ_v(k)：离散速度偏移
    Eigen::Vector3d acceleration_k;  // τ_i(k)：控制输入（加速度）

    // 时间戳
    ros::Time planning_time;  // k*ε：MPC规划时刻

    // 姿态和控制字段
    double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};  // 四元数
    double yaw_rate{0.0};                       // 偏航角速度
    uint16_t type_mask{0};                      // 控制掩码
    uint8_t coordinate_frame{1};                // 坐标系

    // 状态标志
    bool is_valid{false};           // 数据是否有效
    bool new_data_received{false};  // 保留字段：MPC帧切换由 active/pending 缓存控制

    MpcTrajectoryState()
        : position_k(Eigen::Vector3d::Zero()),
          velocity_k(Eigen::Vector3d::Zero()),
          acceleration_k(Eigen::Vector3d::Zero()) {}
};

}  // namespace px4_multirotor_controller
