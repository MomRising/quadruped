use std::sync::{Arc, Mutex};
use std::time::Duration;
use std::thread;
use nalgebra::{Matrix3x4, Point3, Vector3 as naVector3};

use rclrs::{RclrsError, Clock, ClockType, Context};
use std_msgs::msg::{Float32, String as StringMsg};
use geometry_msgs::msg::{Twist, Vector3};
use sensor_msgs::msg::JointState;
use builtin_interfaces::msg::Time as TimeMsg;
use std_srvs::srv::{SetBool, SetBool_Request, SetBool_Response};
// use cus_interfaces::msg::Base;

mod controller;
mod sensor;
mod drive;
//机械结构
const TIME_STEP: f32 = 0.02;
const LENGTH: f32 = 212.0;
const WIDTH: f32 = 110.0;
const LINKS: [f32; 3] = [29.5, 130.0, 139.0];
//默认姿态
const DEF_VEL:f32 = 150.0;
const DEF_HEIGHT: f32 = 150.0;
const DEF_POS: Matrix3x4<f32> = Matrix3x4::<f32>::new(
    LENGTH / 2.0, - LENGTH / 2.0, LENGTH / 2.0, - LENGTH / 2.0, 
    WIDTH / 2.0, - WIDTH / 2.0, - WIDTH / 2.0, WIDTH / 2.0,
    0.0, 0.0, 0.0, 0.0,
    );//坐标基点为主体重心的垂点，x轴为前进方向，y轴为前进方向左侧，z轴向上
const _DEF_POS: [Point3<f32>; 4] = [
    Point3::new(LENGTH / 2.0, WIDTH / 2.0, 0.0),
    Point3::new(-LENGTH / 2.0, -WIDTH / 2.0, 0.0),
    Point3::new(LENGTH / 2.0, -WIDTH / 2.0, 0.0),
    Point3::new(-LENGTH / 2.0, WIDTH / 2.0, 0.0),
];
//////////////////////////////////////////////////////////////////////////////
#[cfg(test)]
mod tests {
    use super::*; // 导入上层模块中的所有公共项
    use controller::ik::ik;
    #[test]
    fn test1() {
        ik(DEF_HEIGHT, naVector3::new(0.0, 0.0, 0.0), DEF_POS);
    }
}
//////////////////////////////////////////////////////////////////////////////
fn main() -> Result<(), RclrsError> {
    let context = rclrs::Context::new(std::env::args())?;
    let robot = Arc::new(RobotModule::new(&context, false)?);//false控制真实机器人
    match robot.sim {
        //物理节点控制
        false => {
            let robot_ = Arc::clone(&robot);
            thread::spawn(move || -> Result<(),RclrsError>{
                loop {
                    let next_mode = robot_.mode_cmd.lock().unwrap().clone().unwrap();
                    let next_vel = robot_.vel_cmd.lock().unwrap().as_ref().unwrap().linear.clone();
                    let body_angle = robot_.state.lock().unwrap().body_angle.clone();
                    let now_foot_pos = robot_.state.lock().unwrap().foot_pos.clone();
                    let next_foot_pos = robot_.controller.lock().unwrap().control(next_mode.data.as_str(), next_vel, now_foot_pos);
                    robot_.state.lock().unwrap().foot_pos = next_foot_pos;
                    let next_angle = controller::ik::ik(DEF_HEIGHT, body_angle, next_foot_pos);
                    drive::stm32(next_angle);

                    thread::sleep(Duration::from_secs_f32(TIME_STEP));
                }
            });
        }
        //仿真节点控制
        true => {
            let robot_sim = Arc::clone(&robot);
            thread::spawn(move || -> Result<(),RclrsError>{
                loop {
                    robot_sim.sim_joint_publish()?;
                    let now = Clock::new(ClockType::SteadyTime).0.now().to_ros_msg().unwrap();

                    let mut temp = robot_sim.sim_joint.lock().unwrap();
                    if let Some(msg) = temp.as_mut() {
                        msg.header.stamp = TimeMsg{sec: now.sec, nanosec: now.nanosec};
                        println!("{:?}", msg.header.stamp);
                    }
                    if let Some(msg) = temp.as_mut() {
                        msg.position[0] += 1.0;
                        println!("{:?}", msg.position);
                    }
                    thread::sleep(Duration::from_secs_f32(TIME_STEP));
                }
            });
        }
    }
    rclrs::spin(Arc::clone(&robot.node))
}

struct RobotState {
    pub _mode: String,
    pub foot_pos: Matrix3x4<f32>,
    pub _height: f32,
    pub body_angle: naVector3<f32>,//roll, pitch, yaw
}
impl RobotState {
    pub fn new() -> Self {
        RobotState {
            _mode: String::from("trot"),
            foot_pos: DEF_POS,
            _height: DEF_HEIGHT,
            body_angle: naVector3::<f32>::new(0.0, 0.0, 0.0),
        }
    }
}

struct RobotController {
    trot: controller::trot::Trot,
}
impl RobotController {
    pub fn new() -> Self {
        RobotController {
            trot: controller::trot::Trot::new(),
        }
    }
    pub fn control(&mut self, mode_cmd: &str, vel_cmd: Vector3, foot_pos: Matrix3x4<f32>) -> Matrix3x4<f32> {
        let vel_cmd_x = vel_cmd.x as f32;
        let vel_cmd_y = vel_cmd.y as f32;
        let vel_cmd_z = vel_cmd.z as f32;
        let vel = vec![vel_cmd_x, vel_cmd_y, vel_cmd_z];
        let next_foot_pos: Matrix3x4<f32> = match mode_cmd {
            "trot" => self.trot.plan(vel, foot_pos),
            _ => Matrix3x4::<f32>::zeros(),
        };
        next_foot_pos
    }
}

pub struct RobotModule {
    _name: String,
    sim: bool,

    mode_cmd:Arc<Mutex<Option<StringMsg>>>,
    vel_cmd:Arc<Mutex<Option<Twist>>>,
    _height_cmd:Arc<Mutex<Option<Float32>>>,
    _led: Arc<Mutex<Option<bool>>>,

    state: Arc<Mutex<RobotState>>,
    controller: Arc<Mutex<RobotController>>,

    node:Arc<rclrs::Node>,
    _base_pub:Arc<rclrs::Publisher<StringMsg>>,
    _vel_sub:Arc<rclrs::Subscription<Twist>>,
    _led_ser:Arc<rclrs::Service<SetBool>>,
    
    sim_joint:Arc<Mutex<Option<JointState>>>,
    sim_joint_publisher:Arc<rclrs::Publisher<JointState>>,
}

impl RobotModule {
    pub fn new(context:&Context, is_sim: bool) -> Result<Self,RclrsError> {
        let _name = String::from("Rupper");
        let sim = is_sim;

        let mode_cmd = Arc::new(Mutex::new(Some(StringMsg{data: String::from("trot")})));
        let vel_cmd = Arc::new(Mutex::new(Some(
            Twist{
            linear: geometry_msgs::msg::Vector3{x: DEF_VEL as f64, y: 0.0, z: 0.0},//初始默认速度修改
            angular: geometry_msgs::msg::Vector3{x: 0.0, y: 0.0, z: 0.0},
        })));
        let height_cmd = Arc::new(Mutex::new(Some(Float32{data: DEF_HEIGHT})));
        let state = Arc::new(Mutex::new(RobotState::new()));
        let controller = Arc::new(Mutex::new(RobotController::new()));
        let led = Arc::new(Mutex::new(Some(false)));

        let node = rclrs::create_node(&context, "base_node")?;
        let _base_pub = node.create_publisher("base_topic", rclrs::QOS_PROFILE_DEFAULT)?;

        let vel_clone = Arc::clone(&vel_cmd);
        let _vel_sub = node.create_subscription("velocity_topic", rclrs::QOS_PROFILE_SENSOR_DATA,
            move |vel:Twist| {
                *vel_clone.lock().unwrap() = Some(vel);
            })?;

        let led_clone = Arc::clone(&led);
        let _led_ser = node.create_service("led_server",
            move |_request_header: &rclrs::rmw_request_id_t, request: SetBool_Request| {
                *led_clone.lock().unwrap() = Some(request.data);
                set_led();
                SetBool_Response {
                    success: true,
                    message: String::from("LED set successfully")}
            })?;

        let now = Clock::new(ClockType::SteadyTime).0.now().to_ros_msg().unwrap();
        let sim_joint = Arc::new(Mutex::new(Some(
            JointState{
                header: std_msgs::msg::Header{
                    stamp: TimeMsg { sec: now.sec, nanosec: now.nanosec },
                    frame_id: String::from("")},
                name: vec![String::from("left_wheel_joint"),String::from("right_wheel_joint")],
                position: vec![0.0, 0.0],
                velocity: vec![0.0, 0.0],
                effort: vec![0.0, 0.0],
            })));
        // println!("sim_joint {:?}",sim_joint);
        let sim_joint_publisher = node.create_publisher("joint_states", rclrs::QOS_PROFILE_DEFAULT)?;

        Ok(Self {
            _name, sim,
            mode_cmd, vel_cmd, _height_cmd: height_cmd, state, controller, _led: led,
            sim_joint, sim_joint_publisher,
            node, _base_pub, _vel_sub, _led_ser,
        })
    }

    fn sim_joint_publish(&self) -> Result<(),RclrsError> {
        if let Some(msg) = &*self.sim_joint.lock().unwrap() {
            self.sim_joint_publisher.publish(msg)?;
        };
        Ok(())
    }

    // fn sim_set_velocity(&self, vel: Vec<f64>) -> Result<(),RclrsError> {
    //     let temp = self.sim_joint.lock().unwrap().as_mut().unwrap();
    //     *temp.velocity = vel;
    //     Ok(())
    // }
    // fn change(& mut self) {
    //     self._name = Some("test".to_string());
    // }
}

fn set_led() {
    todo!("turn over led status")
}