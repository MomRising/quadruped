use std::f32::consts::PI;

use nalgebra::{Matrix3x4, Matrix4, Vector3};
#[path = "./ik.rs"] mod ik;

use crate::{DEF_POS, FREQUENCY, TIME_STEP};
const PHASE_TICK: u32 = (FREQUENCY * 1.0) as u32;//一秒内周期数
const PHASE_TICK_HALF: u32 = PHASE_TICK / 2;
const PHASE_TICK_THIRD: u32 = PHASE_TICK - STANCE_TICK;
const PHASE_TIME: f32 = (PHASE_TICK as f32) * TIME_STEP;//0.8s
const DUTY: f32 = 0.8;//可修改，支撑相占空比
const SWING_TICK: u32 = ((1.0 - DUTY) * PHASE_TICK as f32) as u32;
const STANCE_TICK: u32 = PHASE_TICK / 2 - SWING_TICK;

// const DEF_POS: nalgebra::Matrix<>

/* 
    from above
    1 ---- 4        x
    |      |     y__|
    |      |
    3 ---- 2

 */

pub struct Trot {
    vel: Vec<f32>,
    phase_tick: u32,
    phase_time: f32,
    duty: f32,
    swing_tick: u32,
    stance_tick: u32,
    ticks: u32,
    start_pos: Matrix3x4<f32>,
    end_pos_sub_x_or_y: f32,

    _phase: Matrix4<u32>,
}
impl Trot {
    pub fn new() -> Self {
        Self {
            vel: vec![0.0, 0.0, 0.0],
            phase_tick: PHASE_TICK,
            phase_time: PHASE_TIME,
            duty: DUTY,
            swing_tick: SWING_TICK,
            stance_tick: STANCE_TICK,
            ticks: 0u32,
            start_pos: DEF_POS,
            end_pos_sub_x_or_y: (PHASE_TICK as f32) * DUTY * TIME_STEP * 1.0/5.0,//1/5为设定步幅在默认位置前的部分

            _phase: Matrix4::new(0, 1, 1, 1,
                                0, 1, 1, 1, 
                                1, 1, 0, 1, 
                                1, 1, 0, 1),
        }
    }
    pub fn plan(&mut self, vel_cmd: Vec<f32>, foot_pos: Matrix3x4<f32>) -> Matrix3x4<f32>{
        self.ticks = self.ticks % PHASE_TICK; // 0 ~ PHASE_TICK
        match self.ticks {
            0 | PHASE_TICK_HALF => {
                self.start_pos = foot_pos;
                self.vel = vel_cmd;
            },
            _ => {},
        };
        let next_foot_pos = match self.ticks {
            0..SWING_TICK => {Matrix3x4::<f32>::from_columns(&[
                self.swing(&self.vel, self.ticks, self.start_pos.column(0).into(), DEF_POS.column(0).into()),
                self.swing(&self.vel, self.ticks, self.start_pos.column(1).into(), DEF_POS.column(1).into()),
                self.stance(&self.vel, foot_pos.column(2).into()),
                self.stance(&self.vel, foot_pos.column(3).into())
                ])
            },
            tick if tick >= SWING_TICK && tick < PHASE_TICK_HALF => {Matrix3x4::<f32>::from_columns(&[//范围判断防止占空比0.5时无此区间
                self.stance(&self.vel, foot_pos.column(0).into()),
                self.stance(&self.vel, foot_pos.column(1).into()),
                self.stance(&self.vel, foot_pos.column(2).into()),
                self.stance(&self.vel, foot_pos.column(3).into()),
                ])
            },
            PHASE_TICK_HALF..PHASE_TICK_THIRD => {Matrix3x4::<f32>::from_columns(&[
                self.stance(&self.vel, foot_pos.column(0).into()),
                self.stance(&self.vel, foot_pos.column(1).into()),
                self.swing(&self.vel, self.ticks - PHASE_TICK_HALF, self.start_pos.column(2).into(), DEF_POS.column(2).into()),
                self.swing(&self.vel, self.ticks - PHASE_TICK_HALF, self.start_pos.column(3).into(), DEF_POS.column(3).into())
                ])
            },
            tick if tick >= PHASE_TICK_THIRD && tick < PHASE_TICK => {Matrix3x4::<f32>::from_columns(&[
                self.stance(&self.vel, foot_pos.column(0).into()),
                self.stance(&self.vel, foot_pos.column(1).into()),
                self.stance(&self.vel, foot_pos.column(2).into()),
                self.stance(&self.vel, foot_pos.column(3).into()),
                ])
            },
            _ => {
                Matrix3x4::<f32>::zeros()
            },
        };
        self.ticks += 1;
        // println!("ticks: {}, next_foot_pos: {:?}", self.ticks, next_foot_pos);
        next_foot_pos
    }
    pub fn stance(&self, vel: &Vec<f32>, foot_pos: Vector3<f32>) -> Vector3<f32> {
        let delta_x = - vel[0] * TIME_STEP;
        let delta_y = - vel[1] * TIME_STEP;
        let delta_z = 0.0;
        let delta_xyz = Vector3::new(delta_x, delta_y, delta_z);
        let delta_yaw = ik::rot(0.0, 0.0, 0.0);
        // let delta_yaw = ik::rot(0.0, 0.0, - vel.angular.z as f32 * TIME_STEP);
        let next_foot_pos = delta_xyz + delta_yaw * foot_pos;
        next_foot_pos
    }
    pub fn swing(&self, vel: &Vec<f32>, done_ticks: u32, start_foot_pos: Vector3<f32>, def_foot_pos: Vector3<f32>) -> Vector3<f32>{
        let next_ticks = done_ticks + 1;
        //作为变量
        let sigma = 2.0 * PI * (next_ticks as f32) / (SWING_TICK as f32);
        //30设定抬腿高度
        let pos_z = 30.0 * (1.0 - sigma.cos()) / 2.0;
        //设定摆动相终点位置
        let end_x = vel[0] * self.end_pos_sub_x_or_y + def_foot_pos[0];
        let end_y = vel[1] * self.end_pos_sub_x_or_y + def_foot_pos[1];
        let start_x = start_foot_pos[0];
        let start_y = start_foot_pos[1];
        //z终点为零
        let pos_x = (end_x - start_x) * (sigma - sigma.sin()) / (2.0 * PI) + start_x;
        let pos_y = (end_y - start_y) * (sigma - sigma.sin()) / (2.0 * PI) + start_y;
        let next_foot_pos = Vector3::new(pos_x, pos_y, pos_z);
        next_foot_pos
    }
}
