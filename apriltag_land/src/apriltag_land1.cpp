#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/RCIn.h>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <vector>
#include <geometry_msgs/TwistStamped.h>

using namespace std;
using namespace Eigen;

// 控制参数
#define ERR_DEADZONE 0.03    // 误差死区阈值(单位：米)
#define MIN_LANDING_ERROR 0.1 // 降落最小位置误差(米)
#define SAFE_DESCENT_ALT 0.5  // 安全下降高度(米)

// LQR控制参数
const double DT = 0.1;       // 控制周期(秒)
const double MAX_ACCEL = 2.0;// 最大加速度(m/s²)
const double MAX_VEL = 1.5;  // 最大速度(m/s)

// 全局变量声明
double value0 = 0, value1 = 0, value2 = 0, value3 = 0, value4 = 0, value5 = 0;
double vel_lit = 1.0;  // 速度限制值
double HIGHT = 0, vel_z = 0, x_err = 0, y_err = 0, cam_angle = 0;
double Kp = 0, Ki = 0, Kd = 0;
double x_move = 0, y_move = 0, z_err = 0;
bool land_mode = false;

// LQR控制器参数
const double Q_pos = 10.0;   // 位置误差权重
const double Q_vel = 5.0;    // 速度误差权重
const double R_value = 1.0;  // 控制输入权重

// 系统状态
mavros_msgs::State current_state;
mavros_msgs::PositionTarget setpoint;
bool marker_found = false, flag_land = false;
vector<int> current_target_id(1);
char mode;
double init_x_take_off = 0, init_y_take_off = 0, init_z_take_off = 0;
float detec_x = 0, detec_y = 0, detec_z = 0;
double angle1 = 0, roll, pitch, yaw = 0, target_yaw = 0;
double err_yaw = 0, err_yaw_err = 0, err_yaw0 = 0, diff_angle = 0;
double x_xz, y_xz;

// 订阅的无人机当前位置数据
geometry_msgs::PoseStamped local_pos;
geometry_msgs::TwistStamped local_vel;  // 新增：无人机当前速度

// LQR控制器类
class LQRController {
private:
    MatrixXd A;  // 系统矩阵
    MatrixXd B;  // 输入矩阵
    MatrixXd Q;  // 状态权重矩阵
    MatrixXd R;  // 控制权重矩阵
    MatrixXd K;  // LQR增益矩阵
    double dt;   // 采样时间
    
    // 求解黎卡提方程得到LQR增益
    void solveRiccati() {
        // 初始化P为Q
        MatrixXd P = Q;
        MatrixXd P_next = Q;
        
        // 迭代求解离散黎卡提方程
        const int ITERATIONS = 100;
        for (int i = 0; i < ITERATIONS; ++i) {
            P_next = Q + A.transpose() * P * A - 
                    (A.transpose() * P * B) * 
                    (R + B.transpose() * P * B).inverse() * 
                    (B.transpose() * P * A);
            
            // 检查收敛
            if ((P_next - P).norm() < 1e-6) {
                break;
            }
            P = P_next;
        }
        
        // 计算LQR增益
        K = (R + B.transpose() * P_next * B).inverse() * 
            B.transpose() * P_next * A;
    }
    
public:
    LQRController(double time_step = DT) : dt(time_step) {
        // 初始化系统矩阵 (状态: [x, y, z, vx, vy, vz])
        A = MatrixXd::Identity(6, 6);
        A.block<3, 3>(0, 3) = MatrixXd::Identity(3, 3) * dt;
        
        // 初始化输入矩阵
        B = MatrixXd::Zero(6, 3);
        B.block<3, 3>(3, 0) = MatrixXd::Identity(3, 3) * dt;
        
        // 初始化权重矩阵
        Q = MatrixXd::Zero(6, 6);
        Q.diagonal() << Q_pos, Q_pos, Q_pos*2, Q_vel, Q_vel, Q_vel*2;  // z轴权重更高
        
        R = MatrixXd::Identity(3, 3) * R_value;
        
        // 求解黎卡提方程得到增益矩阵
        solveRiccati();
    }

    // 计算最优控制
    Vector3d computeControl(const Vector3d& current_pos, 
                           const Vector3d& current_vel,
                           const Vector3d& target_pos,
                           const Vector3d& target_vel = Vector3d::Zero()) {
        // 构建状态向量 [位置误差, 速度误差]
        VectorXd state(6);
        state << current_pos - target_pos,  // 位置误差
                 current_vel - target_vel;  // 速度误差
        
        // 计算控制输入 (u = -Kx)
        Vector3d control = -K * state;
        
        // 限制控制输入在安全范围内
        for (int i = 0; i < 3; ++i) {
            if (control[i] > MAX_ACCEL) control[i] = MAX_ACCEL;
            else if (control[i] < -MAX_ACCEL) control[i] = -MAX_ACCEL;
        }
        
        return control;
    }
};

// 全局LQR控制器实例
LQRController lqr;

// 回调函数声明
void state_cb(const mavros_msgs::State::ConstPtr& msg);
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);  // 新增：速度回调
void yaw_cb(const sensor_msgs::Imu::ConstPtr& msg);
void rcin_cb(const mavros_msgs::RCIn::ConstPtr& msg);
void apriltag_cb(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

// 控制函数声明
void pos_xz(double x, double y, double z);
void vel_xz(float x, float y);
void cam_xz(float xa, float ya);

// 回调函数实现
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    local_pos = *msg;
}

// 新增：速度回调函数
void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    local_vel = *msg;
}

void yaw_cb(const sensor_msgs::Imu::ConstPtr& msg) {
    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    angle1 = yaw * 180.0 / M_PI;  // 转换为度
}

void rcin_cb(const mavros_msgs::RCIn::ConstPtr& msg) {
    // 确保参数已初始化
    if(value0 == 0 || value2 == 0 || value3 == 0 || value5 == 0) return;
    
    double vx, vy;
    if(!marker_found && mode == 'm') {
        vx = abs(msg->channels[1] - value2) * 1.00 / abs(value0 - value2) * 1 - 0.5; 
        vy = abs(msg->channels[3] - value5) * 1.00 / abs(value5 - value3) * 1 - 0.5; 
        if(abs(msg->channels[1] - value1) < 50) vx = 0;
        if(abs(msg->channels[3] - value5) < 50) vy = 0;

        double rotation_angle = angle1 * (M_PI / 180.0);
        tf::Quaternion q;
        q.setRPY(0, 0, rotation_angle);

        tf::Vector3 point(vx, vy, 0);
        tf::Matrix3x3 rotation_matrix(q);
        tf::Vector3 rotated_point = rotation_matrix * point;
    }
}

void apriltag_cb(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
    marker_found = false;
    if (!msg->detections.empty()) {
        for (const auto& marker : msg->detections) {			
            if(!marker.id.empty() && marker.id[0] == current_target_id[0]) {       
                marker_found = true;
                detec_x = marker.pose.pose.pose.position.x;
                detec_y = marker.pose.pose.pose.position.y;
                detec_z = marker.pose.pose.pose.position.z;
                break;
            }
        }
    }
}

// 坐标转换函数
void pos_xz(double x, double y, double z) {
    double rotation_angle = angle1 * (M_PI / 180.0);
    tf::Quaternion q;
    q.setRPY(0, 0, rotation_angle);

    tf::Vector3 point(x, y, z);
    tf::Matrix3x3 rotation_matrix(q);
    tf::Vector3 rotated_point = rotation_matrix * point;

    x_xz = rotated_point.x();
    y_xz = rotated_point.y();
}

void vel_xz(float x, float y) {
    double rotation_angle = angle1 * (M_PI / 180.0);
    tf::Quaternion q;
    q.setRPY(0, 0, rotation_angle);

    tf::Vector3 point(x, y, 0);
    tf::Matrix3x3 rotation_matrix(q);
    tf::Vector3 rotated_point = rotation_matrix * point;

    // 调用LQR控制器
    Vector3d current_pos(local_pos.pose.position.x, local_pos.pose.position.y, local_pos.pose.position.z);
    Vector3d current_vel(local_vel.twist.linear.x, local_vel.twist.linear.y, local_vel.twist.linear.z);
    
    // 计算目标位置（当前位置加上期望位移）
    Vector3d target_pos = current_pos + Vector3d(rotated_point.x(), rotated_point.y(), 0);
    
    // 使用LQR计算加速度指令
    Vector3d accel_cmd = lqr.computeControl(current_pos, current_vel, target_pos);
    
    // 转换为速度指令（积分一步）
    double CtrOutx = current_vel.x() + accel_cmd.x() * DT;
    double CtrOuty = current_vel.y() + accel_cmd.y() * DT;
    
    // 限幅处理
    CtrOutx = max(min(CtrOutx, vel_lit), -vel_lit);
    CtrOuty = max(min(CtrOuty, vel_lit), -vel_lit);

    ROS_INFO("vel_x:%f, vel_y:%f", CtrOutx, CtrOuty);

    setpoint.velocity.x = CtrOutx;
    setpoint.velocity.y = CtrOuty;
}

void cam_xz(float xa, float ya) {
    double xb = xa * cos(cam_angle * (M_PI / 180.0)) - ya * sin(cam_angle * (M_PI / 180.0));
    double yb = -xa * sin(cam_angle * (M_PI / 180.0)) - ya * cos(cam_angle * (M_PI / 180.0)); 

    vel_xz(xb, yb);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "apriltag_land_lqr");
    ros::NodeHandle nh;
    
    // 订阅器
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pos_cb);
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 10, local_vel_cb);  // 新增：速度订阅
    ros::Subscriber apriltag_sub = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 10, apriltag_cb);
    ros::Subscriber yaw_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 10, yaw_cb);
    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>("mavros/rc/in", 10, rcin_cb);

    // 发布器和服务客户端
    ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // 参数获取
    nh.getParam("HIGHT", HIGHT);
    nh.getParam("vel_z", vel_z);
    nh.getParam("/cam/x_err", x_err);
    nh.getParam("/cam/y_err", y_err);
    nh.getParam("/cam/R", cam_angle);
    nh.getParam("Kp", Kp);
    nh.getParam("Ki", Ki);
    nh.getParam("Kd", Kd);
    nh.getParam("vel_lit", vel_lit);
    nh.getParam("x_move", x_move);
    nh.getParam("y_move", y_move);
    nh.getParam("z_err", z_err);
    nh.getParam("land_mode", land_mode);
    nh.getParam("/channle1/value0", value0);
    nh.getParam("/channle1/value1", value1);
    nh.getParam("/channle1/value2", value2);
    nh.getParam("/channle2/value3", value3);
    nh.getParam("/channle2/value4", value4);
    nh.getParam("/channle2/value5", value5);

    // 获取目标AprilTag ID
    nh.getParam("target_id", current_target_id[0]);

    ros::Rate rate(10);  // LQR控制可以稍低的频率运行

    // 等待连接
    while(ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // 初始化setpoint
    setpoint.header.stamp = ros::Time::now();
    setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    setpoint.type_mask = 
        mavros_msgs::PositionTarget::IGNORE_VX |
        mavros_msgs::PositionTarget::IGNORE_VY |
        mavros_msgs::PositionTarget::IGNORE_VZ |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::FORCE |
        mavros_msgs::PositionTarget::IGNORE_YAW;
    setpoint.position.x = 0;
    setpoint.position.y = 0;
    setpoint.position.z = 0;

    // 发送初始setpoint
    for(int i = 100; ros::ok() && i > 0; --i) {
        setpoint.header.stamp = ros::Time::now();
        setpoint_pub.publish(setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    // 设置模式和服务
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::SetMode offb_setPS_mode;
    offb_setPS_mode.request.custom_mode = "POSCTL";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time detec_time = ros::Time::now();

    mode = 't';
    int sametimes = 0;
    angle1 = yaw;
    target_yaw = yaw;

    // 主循环
    while(ros::ok()) {
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_setPS_mode) &&
                offb_setPS_mode.response.mode_sent) {
                ROS_INFO("Position control enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    ROS_INFO("UAV armed");
                    init_x_take_off = local_pos.pose.position.x;
                    init_y_take_off = local_pos.pose.position.y;
                    init_z_take_off = local_pos.pose.position.z;
                }
                last_request = ros::Time::now();
            } else {
                // 定义current_height在switch外部，避免跳转错误
                double current_height = local_pos.pose.position.z - init_z_take_off;
                
                switch(mode) {
                    case 't': {  // 起飞模式
                        setpoint.position.x = init_x_take_off;
                        setpoint.position.y = init_y_take_off;
                        setpoint.position.z = init_z_take_off + HIGHT;
                        
                        if(local_pos.pose.position.z > init_z_take_off + HIGHT - 0.2) {	
                            if (sametimes > 10) {
                                mode = 'p';
                                last_request = ros::Time::now();
                            }
                            else sametimes++;
                        }
                        else sametimes = 0;
                        break;
                    }
                    
                    case 'p': {  // 定位模式
                        pos_xz(x_move, y_move, init_z_take_off + HIGHT);
                        setpoint.position.x = init_x_take_off + x_xz;
                        setpoint.position.y = init_y_take_off + y_xz;

                        if(marker_found) {
                            mode = 'm';
                            detec_time = ros::Time::now();
                        }
                        if(abs(local_pos.pose.position.x - (init_x_take_off + x_xz)) < 0.1 && 
                           abs(local_pos.pose.position.y - (init_y_take_off + y_xz)) < 0.1) { 
                            mode = 'm';
                            detec_time = ros::Time::now();
                        }
                        break;
                    }
                    
                    case 'm': {  // 跟踪模式
                        if(marker_found) {	
                            setpoint.type_mask = 
                                mavros_msgs::PositionTarget::IGNORE_PX |
                                mavros_msgs::PositionTarget::IGNORE_PY |
                                mavros_msgs::PositionTarget::IGNORE_PZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::FORCE |
                                mavros_msgs::PositionTarget::IGNORE_YAW;
                                
                            cam_xz(detec_x, detec_y);
                            setpoint.velocity.z = -vel_z;
                            flag_land = true;
                            detec_time = ros::Time::now();
                        } else {
                            if((ros::Time::now() - detec_time).toSec() > 15.0) {
                                mode = 'l';
                                last_request = ros::Time::now();
                            }
                            setpoint.velocity.z = 0;
                        }
                        
                        if(current_height < z_err + 1.5 && land_mode) 
                            setpoint.velocity.z = 0;
                        if(current_height < z_err + 0.3 && flag_land && !land_mode) {
                            mode = 'l';
                            last_request = ros::Time::now();
                        }
                        break;
                    }
                    
                    case 'l': {  // 降落模式
                        offb_set_mode.request.custom_mode = "AUTO.LAND";
                        if (current_state.mode != "AUTO.LAND" && 
                            (ros::Time::now() - last_request > ros::Duration(5.0))) {
                            if (set_mode_client.call(offb_set_mode) && 
                                offb_set_mode.response.mode_sent) {
                                ROS_INFO("Landing mode enabled");
                            }
                            last_request = ros::Time::now();
                        }
                        
                        // 检查是否完成降落
                        if (current_height < MIN_LANDING_ERROR) {
                            ROS_INFO("Landing completed!");
                            // 执行安全上锁
                            arm_cmd.request.value = false;
                            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                                ROS_INFO("UAV disarmed");
                            }
                        }
                        break;
                    }
                }
            }
        }

        // 偏航控制
        err_yaw = target_yaw - yaw;
        // 处理角度环绕问题
        if (err_yaw > M_PI) err_yaw -= 2 * M_PI;
        else if (err_yaw < -M_PI) err_yaw += 2 * M_PI;
        
        err_yaw_err = err_yaw - err_yaw0;
        err_yaw0 = err_yaw;
        setpoint.yaw_rate = 0.01 * err_yaw + 0.002 * err_yaw_err;

        setpoint.header.stamp = ros::Time::now();
        setpoint_pub.publish(setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

