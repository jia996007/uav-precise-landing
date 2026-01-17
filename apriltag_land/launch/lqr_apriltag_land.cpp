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

using namespace std;

// 控制参数
#define ERR_DEADZONE 0.03    // 误差死区阈值(单位：米)

// 全局变量声明
double value0 = 0, value1 = 0, value2 = 0, value3 = 0, value4 = 0, value5 = 0;
double vel_lit = 1.0;  // 速度限制值
double HIGHT = 0, vel_z = 0, x_err = 0, y_err = 0, cam_angle = 0;
double Kp = 0, Ki = 0, Kd = 0;
double x_move = 0, y_move = 0, z_err = 0;
bool land_mode = false;
double MIN_ERROR = 0.1;
double err_yaw = 0, err_yaw_err = 0, err_yaw0 = 0, diff_angle = 0;

// LQR控制器参数
double Q_pos = 1.0;      // 位置误差权重
double Q_vel = 0.1;      // 速度误差权重
double R_value = 0.01;   // 控制输入权重
const double CTRL_LIM_FINE = 1.0; // 控制输出限幅值

// 历史误差记录
static double errx_old_Last = 0;
static double erry_old_Last = 0;

// 控制输出
double CtrOutx = 0;
double CtrOuty = 0;

// 系统状态
mavros_msgs::State current_state;
mavros_msgs::PositionTarget setpoint;
bool marker_found = false, flag_land = false;
std::vector<int> current_target_id(1);
char mode;
double init_x_take_off = 0, init_y_take_off = 0, init_z_take_off = 0;
float detec_x = 0, detec_y = 0, detec_z = 0;
double angle1 = 0, roll, pitch, yaw = 0, target_yaw = 0;
double errx_Now = 0, erry_Now = 0;
double x_xz, y_xz;

// 订阅的无人机当前位置数据
geometry_msgs::PoseStamped local_pos;

// 回调函数声明
void state_cb(const mavros_msgs::State::ConstPtr& msg);
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
void yaw_cb(sensor_msgs::Imu msg);
void rcin_cb(const mavros_msgs::RCIn::ConstPtr& msg);
void apriltag_cb(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

// 控制函数声明
//void vel_lqr(double x_err, double y_err);
//void pos_xz(double x, double y, double z);
//void vel_xz(float x, float y);
//void cam_xz(float xa, float ya);

// 回调函数实现
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    local_pos = *msg;
}

void yaw_cb(sensor_msgs::Imu msg) {
    // 实现yaw角计算
}

void rcin_cb(const mavros_msgs::RCIn::ConstPtr& msg) {
    double vx, vy;
    if(!marker_found && mode == 'm') {
        vx = abs(msg->channels[1] - value2) * 1.00 / abs(value0 - value2) * 1 - 0.5; 
        vy = abs(msg->channels[3] - value5) * 1.00 / abs(value5 - value3) * 1 - 0.5; 
        if(abs(msg->channels[1] - value1) < 50) vx = 0;
        if(abs(msg->channels[3] - value5) < 50) vy = 0;

        double rotation_angle = (angle1) * (M_PI / 180.0);
        tf::Quaternion q;
        q.setRPY(0, 0, rotation_angle);

        tf::Vector3 point(vx, vy, 0);
        tf::Matrix3x3 rotation_matrix(q);
        tf::Vector3 rotated_point = rotation_matrix * point;
    }
}

void apriltag_cb(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
    int count = msg->detections.size();
    if(count != 0) {
        for(int i = 0; i < count; i++) {
            apriltag_ros::AprilTagDetection marker = msg->detections[i];			
            if(!marker.id.empty() && marker.id[0] == current_target_id[0]) {       
                marker_found = true;
                detec_x = marker.pose.pose.pose.position.x;
                detec_y = marker.pose.pose.pose.position.y;
                detec_z = marker.pose.pose.pose.position.z; 
            }
        }
    } else {
        marker_found = false;
    }
}

// LQR控制器实现
void vel_lqr(double x_err, double y_err) {
    // 系统采样时间（20Hz控制频率）
    const double dt = 0.05;

    // 系统矩阵（连续时间系统）
    Eigen::Matrix2d A;
    A << 0, 1,
         0, 0;

    Eigen::Matrix<double, 2, 1> B;
    B << 0,
         1;

    // 权重矩阵
    Eigen::Matrix2d Q;
    Q << Q_pos, 0,
         0, Q_vel;

    // 控制权重矩阵（必须是正定矩阵）
    Eigen::Matrix<double, 1, 1> R;
    R << R_value;  // R现在是1x1矩阵

    // 初始化Riccati方程的解
    Eigen::Matrix2d P = Eigen::Matrix2d::Zero();
    
    // Riccati方程迭代求解参数
    const double tolerance = 1e-6;
    const int max_iter = 1000;

    // Riccati方程迭代求解
    for (int i = 0; i < max_iter; ++i) {
        // 修正矩阵运算维度
        Eigen::Matrix2d P_next = A.transpose() * P + P * A - 
                               P * B * R.inverse() * B.transpose() * P + Q;
        
        // 检查收敛
        if ((P_next - P).norm() < tolerance) {
            P = P_next;
            break;
        }
        P = P_next;
    }

    // 计算反馈增益矩阵 K = R^-1 * B^T * P
    // 修正维度问题：R是1x1，B是2x1，P是2x2
    Eigen::Matrix<double, 1, 2> K = R.inverse() * B.transpose() * P;

    // ==== x轴控制 ====
    double vel_x = (x_err - errx_old_Last) / dt;
    Eigen::Vector2d state_x;
    state_x << x_err, vel_x;
    CtrOutx = -(K * state_x)(0);

    // ==== y轴控制 ====
    double vel_y = (y_err - erry_old_Last) / dt;
    Eigen::Vector2d state_y;
    state_y << y_err, vel_y;
    CtrOuty = -(K * state_y)(0);

    // 更新历史误差
    errx_old_Last = x_err;
    erry_old_Last = y_err;

    // 输出限幅
    CtrOutx = std::max(std::min(CtrOutx, CTRL_LIM_FINE), -CTRL_LIM_FINE);
    CtrOuty = std::max(std::min(CtrOuty, CTRL_LIM_FINE), -CTRL_LIM_FINE);
}

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
    double rotation_angle = (angle1) * (M_PI / 180.0);
    tf::Quaternion q;
    q.setRPY(0, 0, rotation_angle);

    tf::Vector3 point(x, y, 0);
    tf::Matrix3x3 rotation_matrix(q);
    tf::Vector3 rotated_point = rotation_matrix * point;

    // 调用LQR控制器
    vel_lqr(rotated_point.x(), rotated_point.y());
    
    if(CtrOutx > vel_lit) CtrOutx = vel_lit;
    if(CtrOutx < -vel_lit) CtrOutx = -vel_lit;
    if(CtrOuty > vel_lit) CtrOuty = vel_lit;
    if(CtrOuty < -vel_lit) CtrOuty = -vel_lit;

    ROS_INFO("vel_x:%f,vel_y:%f", CtrOutx, CtrOuty);

    setpoint.velocity.x = CtrOutx;
    setpoint.velocity.y = CtrOuty;
}

void cam_xz(float xa, float ya) {
    double xb, yb;
    xb = xa * cos(cam_angle * (M_PI / 180.0)) - ya * sin(cam_angle * (M_PI / 180.0));
    yb = -xa * sin(cam_angle * (M_PI / 180.0)) - ya * cos(cam_angle * (M_PI / 180.0)); 

    vel_xz(xb, yb);
}



int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "apriltag_land");
    ros::NodeHandle nh;
    
    // 订阅器
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pos_cb);
    ros::Subscriber apriltag_sub = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 10, apriltag_cb);
    ros::Subscriber yaw_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 10, yaw_cb);
    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>("mavros/rc/in", 10, rcin_cb);

    // 发布器和服务客户端
    ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20);

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
    nh.getParam("MIN_ERROR", MIN_ERROR);
    nh.getParam("/channle1/value0", value0);
    nh.getParam("/channle1/value1", value1);
    nh.getParam("/channle1/value2", value2);
    nh.getParam("/channle2/value3", value3);
    nh.getParam("/channle2/value4", value4);
    nh.getParam("/channle2/value5", value5);

    ROS_INFO("x_err:%d,y_err:%d", x_err, y_err);
    ROS_INFO("HIGHT:%f,vel_z:%f,R:%f", HIGHT, vel_z, cam_angle);
    ROS_INFO("kp:%f,ki:%f,kd:%f,x:%f,y:%f", Kp, Ki, Kd, x_move, y_move);

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
                ROS_INFO("POSTION PROTECTED");
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
                switch(mode) {
                    case 't':
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
                    case 'p':
                        pos_xz(x_move, y_move, init_z_take_off + HIGHT);
                        setpoint.position.x = init_x_take_off + x_xz;
                        setpoint.position.y = init_y_take_off + y_xz;

                        if(marker_found) {
                            mode = 'm';
                            detec_time = ros::Time::now();
                        }
                        if(local_pos.pose.position.x > init_x_take_off + x_xz - 0.1 && 
                           local_pos.pose.position.x < init_x_take_off + x_xz + 0.1 &&
                           local_pos.pose.position.y > init_y_take_off + y_xz - 0.1 && 
                           local_pos.pose.position.y < init_y_take_off + y_xz + 0.1) { 
                            mode = 'm';
                            detec_time = ros::Time::now();
                        }
                        break;
                    case 'm':
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
                            if(ros::Time::now() - detec_time > ros::Duration(15.0)) {
                                mode = 'l';
                                last_request = ros::Time::now();
                            }
                            setpoint.velocity.z = 0;
                        }
                        if(local_pos.pose.position.z < init_z_take_off + z_err + 1.5 && land_mode == 1) 
                            setpoint.velocity.z = 0;
                        if(local_pos.pose.position.z < init_z_take_off + z_err + 0.3 && flag_land == true && land_mode == 0) {
                            mode = 'l';
                            last_request = ros::Time::now();
                        }
                        break;
                    case 'l':
                        offb_set_mode.request.custom_mode = "AUTO.LAND";
                        if (current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                                ROS_INFO("AUTO.LAND enabled");
                            }
                            last_request = ros::Time::now();
                        }
                        break;
                }
            }
        }

        // 计算偏航角速度
        err_yaw = target_yaw - yaw;
        err_yaw_err = err_yaw - err_yaw0;
        err_yaw0 = err_yaw;
        diff_angle = 0.01 * err_yaw + 0.002 * err_yaw_err;
        setpoint.yaw_rate = diff_angle;

        setpoint_pub.publish(setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}





