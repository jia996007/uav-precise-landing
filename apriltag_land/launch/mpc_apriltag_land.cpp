#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <mavros_msgs/CommandBool.h> 
#include <mavros_msgs/SetMode.h>     
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>  
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/RCIn.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>

using namespace std;
using namespace Eigen;

// 控制参数 - 重点优化降落相关参数
#define ERR_DEADZONE      0.03    
#define SAFE_DESCENT_ALT  2.5     // 提高安全下降高度阈值到1.5米
#define POS_STABLE_COUNT  8       // 进一步降低稳定计数阈值，更容易满足
#define MIN_LANDING_ERROR 0.15    // 适当增大降落误差阈值，更容易满足
#define LOW_PASS_ALPHA    0.6     
#define GROUND_THRESHOLD  0.2     // 地面阈值高度，低于此值直接触发落地逻辑

// MPC参数
const int HORIZON = 8;           
const double DT = 0.15;          
const double MAX_ACCEL = 1.0;    // 进一步降低最大加速度，接近地面时更稳定
const double MAX_VEL = 0.6;      // 降低最大速度，提高接近地面时的稳定性

// 全局变量
bool marker_found = false, flag_land = false;
vector<int> current_target_id(1);
char mode;
double init_x_take_off = 0, init_y_take_off = 0, init_z_take_off = 0;
float detec_x = 0, detec_y = 0, detec_z = 0;
double angle1 = 0, roll, pitch, yaw = 0, target_yaw = 0;
double err_yaw = 0, err_yaw0 = 0, err_yaw_err = 0;
double x_xz, y_xz;
int position_stable_counter = 0;  
double last_cmd_vx = 0, last_cmd_vy = 0;  

// 参数变量
int x_err = 0, y_err = 0, land_mode;
double vel_z, cam_angle, HIGHT, vel_lit, Kp, Ki, Kd;
double x_move, y_move, z_err, MIN_ERROR;
int value0, value1, value2, value3, value4, value5;
bool auto_land_activated = false;  // 跟踪自动降落是否激活

// 系统状态
mavros_msgs::State current_state;
mavros_msgs::PositionTarget setpoint;
geometry_msgs::PoseStamped local_pos;
geometry_msgs::TwistStamped local_vel;  

// MPC控制器类
class MPCController {
private:
    MatrixXd Q;  
    MatrixXd R;  
    double dt;   
    
public:
    MPCController(double time_step = DT) : dt(time_step) {
        Q = MatrixXd::Identity(6, 6);
        Q.diagonal() << 8.0, 8.0, 12.0, 1.5, 1.5, 3.0;
        
        R = MatrixXd::Identity(3, 3) * 0.8;
    }

    Vector3d computeControl(const Vector3d& current_pos, 
                           const Vector3d& current_vel,
                           const Vector3d& target_pos,
                           double current_height) {
        MatrixXd A(6, 6);
        A << 1, 0, 0, dt, 0, 0,
             0, 1, 0, 0, dt, 0,
             0, 0, 1, 0, 0, dt,
             0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 1, 0,
             0, 0, 0, 0, 0, 1;
             
        MatrixXd B(6, 3);
        B << 0.5*dt*dt, 0, 0,
             0, 0.5*dt*dt, 0,
             0, 0, 0.5*dt*dt,
             dt, 0, 0,
             0, dt, 0,
             0, 0, dt;
        
        vector<Vector3d> candidates = generateControlCandidates();
        
        VectorXd state(6);
        state << current_pos, current_vel;
        VectorXd target_state(6);
        target_state << target_pos, 0, 0, 0;
        
        double min_cost = numeric_limits<double>::max();
        Vector3d best_control = Vector3d::Zero();
        
        double height_factor = (current_height > SAFE_DESCENT_ALT) ? 1.0 : 
                              (current_height / SAFE_DESCENT_ALT);
        double pos_weight = 1.0 + (1.0 - height_factor) * 3.0;
        
        for (const auto& u : candidates) {
            VectorXd predicted_state = state;
            double cost = 0.0;
            bool valid = true;
            
            for (int i = 0; i < HORIZON; i++) {
                predicted_state = A * predicted_state + B * u;
                
                Vector3d pred_vel = predicted_state.tail(3);
                if (pred_vel.norm() > MAX_VEL) {
                    valid = false;
                    break;
                }
                
                // 接近地面时增加惩罚，但允许最终接触地面
                if (predicted_state[2] < init_z_take_off + GROUND_THRESHOLD) {
                    cost += 3.0;  // 降低接近地面的惩罚，允许落地
                }
                
                VectorXd error = predicted_state - target_state;
                cost += pos_weight * error.head(3).transpose() * Q.topLeftCorner(3,3) * error.head(3);
                cost += error.tail(3).transpose() * Q.bottomRightCorner(3,3) * error.tail(3);
                cost += u.transpose() * R * u;
            }
            
            if (valid && cost < min_cost) {
                min_cost = cost;
                best_control = u;
            }
        }
        
        return best_control;
    }
    
private:
    vector<Vector3d> generateControlCandidates() {
        vector<Vector3d> candidates;
        const int steps = 4;
        double step = 2 * MAX_ACCEL / (steps - 1);
        
        for (int i = 0; i < steps; i++) {
            for (int j = 0; j < steps; j++) {
                double ax = -MAX_ACCEL + i * step;
                double ay = -MAX_ACCEL + j * step;
                
                if (sqrt(ax*ax + ay*ay) > MAX_ACCEL * 1.1)
                    continue;
                    
                candidates.emplace_back(ax, ay, 0);
            }
        }
        
        return candidates;
    }
};

MPCController mpc;

// 回调函数声明
void state_cb(const mavros_msgs::State::ConstPtr& msg);
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
void yaw_cb(const sensor_msgs::Imu::ConstPtr& msg);
void rcin_cb(const mavros_msgs::RCIn::ConstPtr& msg);
void apriltag_cb(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

// 控制函数声明
void pos_xz(double x, double y, double z);
void vel_mpc(float x, float y);
void cam_xz(float xa, float ya);
double computeDescentVelocity(double current_height, double pos_error);

// 回调函数实现
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
    // 监控自动降落模式是否激活
    if (current_state.mode == "AUTO.LAND") {
        auto_land_activated = true;
        ROS_INFO("AUTO.LAND mode is active");
    } else {
        auto_land_activated = false;
    }
}

void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    local_pos = *msg;
}

void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    static bool first_run = true;
    static geometry_msgs::TwistStamped filtered_vel;
    
    if (first_run) {
        filtered_vel = *msg;
        first_run = false;
    } else {
        filtered_vel.twist.linear.x = 0.7 * filtered_vel.twist.linear.x + 0.3 * msg->twist.linear.x;
        filtered_vel.twist.linear.y = 0.7 * filtered_vel.twist.linear.y + 0.3 * msg->twist.linear.y;
        filtered_vel.twist.linear.z = 0.7 * filtered_vel.twist.linear.z + 0.3 * msg->twist.linear.z;
    }
    
    local_vel = filtered_vel;
}

void yaw_cb(const sensor_msgs::Imu::ConstPtr& msg) {
    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    angle1 = yaw * 180.0 / M_PI;
}

void rcin_cb(const mavros_msgs::RCIn::ConstPtr& msg) {
    double vx, vy;
    // RC通道7（索引6）大于1800时强制进入降落模式（备用机制）
    if (msg->channels.size() > 6 && msg->channels[6] > 1800) {
        mode = 'l';
        ROS_WARN("Force landing triggered by RC");
    }
    
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

        setpoint.velocity.x = rotated_point.x() * 0.5;
        setpoint.velocity.y = rotated_point.y() * 0.5;
    }
}

void apriltag_cb(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
    static const float alpha = 0.2;
    marker_found = false;
    if (!msg->detections.empty()) {
        for (const auto& marker : msg->detections) {			
            if(!marker.id.empty() && marker.id[0] == current_target_id[0]) {       
                marker_found = true;
                detec_x = alpha * marker.pose.pose.pose.position.x + (1 - alpha) * detec_x;
                detec_y = alpha * marker.pose.pose.pose.position.y + (1 - alpha) * detec_y;
                detec_z = alpha * marker.pose.pose.pose.position.z + (1 - alpha) * detec_z;
                
                ROS_DEBUG("Marker detected - x: %.3f, y: %.3f, z: %.3f", detec_x, detec_y, detec_z);
                break;
            }
        }
    }
    
    if (!marker_found) {
        ROS_DEBUG("Marker not found");
    }
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

void vel_mpc(float x, float y) {
    double rotation_angle = angle1 * (M_PI / 180.0);
    tf::Quaternion q;
    q.setRPY(0, 0, rotation_angle);

    tf::Vector3 point(x, y, 0);
    tf::Matrix3x3 rotation_matrix(q);
    tf::Vector3 rotated_point = rotation_matrix * point;

    Vector3d current_pos(local_pos.pose.position.x, 
                        local_pos.pose.position.y, 
                        local_pos.pose.position.z);
    Vector3d current_vel(local_vel.twist.linear.x, 
                        local_vel.twist.linear.y, 
                        local_vel.twist.linear.z);
    
    Vector3d target_pos = current_pos + Vector3d(rotated_point.x() * 0.8, rotated_point.y() * 0.8, 0);
    
    double current_height = local_pos.pose.position.z - init_z_take_off;
    ROS_DEBUG("Current height: %.3f", current_height);
    
    Vector3d accel_cmd = mpc.computeControl(current_pos, current_vel, target_pos, current_height);
    
    double cmd_vx = current_vel.x() + accel_cmd.x() * DT;
    double cmd_vy = current_vel.y() + accel_cmd.y() * DT;
    
    cmd_vx = LOW_PASS_ALPHA * cmd_vx + (1 - LOW_PASS_ALPHA) * last_cmd_vx;
    cmd_vy = LOW_PASS_ALPHA * cmd_vy + (1 - LOW_PASS_ALPHA) * last_cmd_vy;
    
    last_cmd_vx = cmd_vx;
    last_cmd_vy = cmd_vy;
    
    double vel_scale = (current_height > SAFE_DESCENT_ALT) ? 1.0 : 
                      (current_height / SAFE_DESCENT_ALT);
    double scaled_vel_lit = vel_lit * max(vel_scale, 0.2);
    
    cmd_vx = max(min(cmd_vx, scaled_vel_lit), -scaled_vel_lit);
    cmd_vy = max(min(cmd_vy, scaled_vel_lit), -scaled_vel_lit);
    
    if (abs(cmd_vx) < ERR_DEADZONE) cmd_vx = 0;
    if (abs(cmd_vy) < ERR_DEADZONE) cmd_vy = 0;
    
    double pos_error = sqrt(pow(rotated_point.x(), 2) + pow(rotated_point.y(), 2));
    ROS_DEBUG("Position error: %.3f", pos_error);
    
    // 优化计数器逻辑：低高度时降低稳定性要求
    if (current_height < GROUND_THRESHOLD * 2) {
        // 接近地面时，即使有稍大误差也计数
        if (pos_error < MIN_LANDING_ERROR * 1.5) {
            position_stable_counter++;
        } else if (position_stable_counter > 0) {
            position_stable_counter--;
        }
    } else {
        if (pos_error < MIN_LANDING_ERROR) {
            position_stable_counter++;
        } else if (position_stable_counter > 0) {
            position_stable_counter--;
        }
    }

    ROS_INFO("MPC: vel_x=%.3f, vel_y=%.3f, error=%.3f, stable=%d, height=%.3f", 
             cmd_vx, cmd_vy, pos_error, position_stable_counter, current_height);

    setpoint.velocity.x = cmd_vx;
    setpoint.velocity.y = cmd_vy;
}

void cam_xz(float xa, float ya) {
    double xb = xa * cos(cam_angle * (M_PI / 180.0)) - ya * sin(cam_angle * (M_PI / 180.0));
    double yb = -xa * sin(cam_angle * (M_PI / 180.0)) - ya * cos(cam_angle * (M_PI / 180.0)); 

    xb -= x_err;
    yb -= y_err;
    
    vel_mpc(xb, yb);
}

// 优化下降速度计算，确保能落地
double computeDescentVelocity(double current_height, double pos_error) {
    // 极低高度时强制缓慢下降
    if (current_height < GROUND_THRESHOLD) {
        return -0.1;  // 缓慢下降接触地面
    }
    
    // 低高度时（接近地面）放宽误差限制
    double error_factor;
    if (current_height < SAFE_DESCENT_ALT) {
        error_factor = min(1.0, max(0.2, 1.0 - pos_error * 2.0));  // 降低误差对下降的影响
    } else {
        error_factor = min(1.0, max(0.3, 1.0 - pos_error * 3.0));
    }
    
    double height_factor = (current_height > SAFE_DESCENT_ALT) ? 1.0 : 
                          (current_height / SAFE_DESCENT_ALT * 0.6 + 0.4);
    
    double descent_vel = -vel_z * error_factor * height_factor * 1.2;  // 略微增加下降速度
    
    // 限制最大下降速度，但允许接近地面时稍快
    double max_descent = (current_height < SAFE_DESCENT_ALT) ? -0.35 : -0.3;
    return max(descent_vel, max_descent);
}

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "apriltag_land_mpc");
    ros::NodeHandle nh;
    
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pos_cb);
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 10, local_vel_cb);
    ros::Subscriber apriltag_sub = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 10, apriltag_cb);
    ros::Subscriber yaw_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 10, yaw_cb);
    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>("mavros/rc/in", 10, rcin_cb);

    ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

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
    nh.getParam("target_id", current_target_id[0]);

    ROS_INFO("MPC Drone Landing Initializing...");
    ROS_INFO("Target tag ID: %d", current_target_id[0]);
    ROS_INFO("Flight parameters: HIGHT=%.1f, vel_z=%.1f", HIGHT, vel_z);
    ROS_INFO("Landing thresholds: SAFE_DESCENT=%.1f, GROUND=%.1f", SAFE_DESCENT_ALT, GROUND_THRESHOLD);

    ros::Rate rate(1/DT);

    ros::Time connect_timeout = ros::Time::now() + ros::Duration(30.0);
    while(ros::ok() && !current_state.connected && ros::Time::now() < connect_timeout) {
        ros::spinOnce();
        rate.sleep();
    }
    
    if (!current_state.connected) {
        ROS_ERROR("Failed to connect to MAVROS after 30 seconds");
        return -1;
    }

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

    for(int i = 100; ros::ok() && i > 0; --i) {
        setpoint.header.stamp = ros::Time::now();
        setpoint_pub.publish(setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "AUTO.LAND";

    mavros_msgs::SetMode offb_setPS_mode;
    offb_setPS_mode.request.custom_mode = "POSCTL";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time detec_time = ros::Time::now();
    ros::Time stable_land_time = ros::Time::now();
    ros::Time auto_land_start_time = ros::Time::now();  // 跟踪自动降落开始时间

    mode = 't';
    int sametimes = 0;
    angle1 = yaw;
    target_yaw = yaw;

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
                double current_height = local_pos.pose.position.z - init_z_take_off;
                ROS_DEBUG("Current height: %.3f", current_height);
                
                double pos_error = sqrt(pow(local_pos.pose.position.x - (init_x_take_off + x_xz), 2) +
                                       pow(local_pos.pose.position.y - (init_y_take_off + y_xz), 2));
                
                switch(mode) {
                    case 't': {  
                        setpoint.type_mask =
                            mavros_msgs::PositionTarget::IGNORE_VX |
                            mavros_msgs::PositionTarget::IGNORE_VY |
                            mavros_msgs::PositionTarget::IGNORE_VZ |
                            mavros_msgs::PositionTarget::IGNORE_AFX |
                            mavros_msgs::PositionTarget::IGNORE_AFY |
                            mavros_msgs::PositionTarget::IGNORE_AFZ |
                            mavros_msgs::PositionTarget::FORCE |
                            mavros_msgs::PositionTarget::IGNORE_YAW;
                            
                        setpoint.position.x = init_x_take_off;
                        setpoint.position.y = init_y_take_off;
                        setpoint.position.z = init_z_take_off + HIGHT;
                        
                        if(local_pos.pose.position.z > init_z_take_off + HIGHT - 0.3 &&
                           pos_error < 0.15) {
                            if (sametimes > 15) {
                                mode = 'p';
                                last_request = ros::Time::now();
                                ROS_INFO("Takeoff completed, entering positioning mode");
                            }
                            else sametimes++;
                        }
                        else sametimes = 0;
                        break;
                    }
                    
                    case 'p': {  
                        setpoint.type_mask =
                            mavros_msgs::PositionTarget::IGNORE_VX |
                            mavros_msgs::PositionTarget::IGNORE_VY |
                            mavros_msgs::PositionTarget::IGNORE_VZ |
                            mavros_msgs::PositionTarget::IGNORE_AFX |
                            mavros_msgs::PositionTarget::IGNORE_AFY |
                            mavros_msgs::PositionTarget::IGNORE_AFZ |
                            mavros_msgs::PositionTarget::FORCE |
                            mavros_msgs::PositionTarget::IGNORE_YAW;
                            
                        pos_xz(x_move, y_move, init_z_take_off + HIGHT);
                        setpoint.position.x = init_x_take_off + x_xz;
                        setpoint.position.y = init_y_take_off + y_xz;
                        setpoint.position.z = init_z_take_off + HIGHT;

                        if(marker_found) {
                            mode = 'm';
                            detec_time = ros::Time::now();
                            ROS_INFO("Marker found, entering tracking mode");
                        }
                        else if(pos_error < 0.1) {
                            if ((ros::Time::now() - stable_land_time).toSec() > 1.5) {
                                mode = 'm';
                                detec_time = ros::Time::now();
                                ROS_INFO("Position stabilized, entering tracking mode");
                            }
                        } else {
                            stable_land_time = ros::Time::now();
                        }
                        break;
                    }
                    
                    case 'm': {  
                        setpoint.type_mask =
                            mavros_msgs::PositionTarget::IGNORE_PX |
                            mavros_msgs::PositionTarget::IGNORE_PY |
                            mavros_msgs::PositionTarget::IGNORE_PZ |
                            mavros_msgs::PositionTarget::IGNORE_AFX |
                            mavros_msgs::PositionTarget::IGNORE_AFY |
                            mavros_msgs::PositionTarget::IGNORE_AFZ |
                            mavros_msgs::PositionTarget::FORCE |
                            mavros_msgs::PositionTarget::IGNORE_YAW;
                            
                        if(marker_found) {	
                            cam_xz(detec_x, detec_y);
                            
                            setpoint.velocity.z = computeDescentVelocity(current_height, pos_error);
                            
                            flag_land = true;
                            detec_time = ros::Time::now();
                            
                            // 优化降落触发条件，将高度条件调整为2.5米
                            bool height_condition = current_height < SAFE_DESCENT_ALT || current_height < 2.5;
                            bool error_condition = pos_error < MIN_LANDING_ERROR * 1.3;
                            bool stable_condition = position_stable_counter > POS_STABLE_COUNT;
                            // 降低强制降落的高度阈值，更早触发
                            bool force_land_condition = current_height < GROUND_THRESHOLD *5;  // 改为1.0米
                            
                            ROS_DEBUG("Landing conditions - height: %d, error: %d, stable: %d, force: %d", 
                                      height_condition, error_condition, stable_condition, force_land_condition);
                              
                            if((error_condition && stable_condition && height_condition) || force_land_condition) {
                                if ((ros::Time::now() - stable_land_time).toSec() > 0.3) {  // 进一步缩短稳定等待时间
                                    ROS_INFO("Position stable, entering landing mode");
                                    mode = 'l';
                                    last_request = ros::Time::now();
                                    auto_land_start_time = ros::Time::now();
                                }
                            } else {
                                stable_land_time = ros::Time::now();
                            }
                        } else {
                            if((ros::Time::now() - detec_time).toSec() > 15.0) {
                                mode = 'p';
                                last_request = ros::Time::now();
                                ROS_WARN("Marker lost, returning to positioning mode");
                            }
                            setpoint.velocity.z = 0;
                            setpoint.velocity.x = 0;
                            setpoint.velocity.y = 0;
                        }
                        
                        // 调整位置误差大时的处理逻辑，低高度时允许更大误差
                        if(current_height < z_err + 0.5 && flag_land) {
                            double error_threshold = (current_height < SAFE_DESCENT_ALT) ? 
                                                    MIN_LANDING_ERROR * 2.0 : MIN_LANDING_ERROR * 1.5;
                            
                            if (pos_error >= error_threshold) {
                                setpoint.velocity.z = 0;
                                ROS_WARN("Position error too large, stopping descent");
                            }
                        }
                        break;
                    }
                    
                    case 'l': {  // 强化自动降落逻辑
                        offb_set_mode.request.custom_mode = "AUTO.LAND";
                        
                        // 持续发送自动降落指令，直到确认模式切换成功
                        if (!auto_land_activated) {
                            if (ros::Time::now() - last_request > ros::Duration(0.5)) {  // 高频重试
                                if (set_mode_client.call(offb_set_mode) && 
                                    offb_set_mode.response.mode_sent) {
                                    ROS_INFO("Sent AUTO.LAND command");
                                } else {
                                    ROS_WARN("Failed to send AUTO.LAND command, retrying...");
                                }
                                last_request = ros::Time::now();
                            }
                        }
                        
                        // 如果自动降落模式激活但长时间（10秒）未落地，强制降低
                        if (auto_land_activated && current_height > GROUND_THRESHOLD) {
                            if (ros::Time::now() - auto_land_start_time > ros::Duration(10.0)) {
                                ROS_WARN("Auto-land taking too long, forcing final descent");
                                setpoint.type_mask =
                                    mavros_msgs::PositionTarget::IGNORE_PX |
                                    mavros_msgs::PositionTarget::IGNORE_PY |
                                    mavros_msgs::PositionTarget::IGNORE_VX |
                                    mavros_msgs::PositionTarget::IGNORE_VY |
                                    mavros_msgs::PositionTarget::IGNORE_AFX |
                                    mavros_msgs::PositionTarget::IGNORE_AFY |
                                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                                    mavros_msgs::PositionTarget::FORCE |
                                    mavros_msgs::PositionTarget::IGNORE_YAW;
                                setpoint.position.z = init_z_take_off + 0.1;  // 设定极低目标高度
                                setpoint_pub.publish(setpoint);
                            }
                        }
                        
                        // 触地检测：如果高度低于地面阈值且速度接近零，提示降落完成
                        if (current_height < GROUND_THRESHOLD && 
                            abs(local_vel.twist.linear.z) < 0.05) {
                            ROS_INFO("Landing completed! Current height: %.3f", current_height);
                        }
                        break;
                    }
                }
            }
        }

        // 偏航控制
        err_yaw = target_yaw - yaw;
        if (err_yaw > M_PI) err_yaw -= 2 * M_PI;
        else if (err_yaw < -M_PI) err_yaw += 2 * M_PI;
        
        err_yaw_err = err_yaw - err_yaw0;
        err_yaw0 = err_yaw;
        setpoint.yaw_rate = 0.008 * err_yaw + 0.003 * err_yaw_err;
        setpoint.yaw_rate = std::max(std::min(setpoint.yaw_rate, 0.15f), -0.15f);

        setpoint.header.stamp = ros::Time::now();
        setpoint_pub.publish(setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

