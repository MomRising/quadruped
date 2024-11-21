use std::f32::consts::PI;

use nalgebra::{distance, point, Matrix3, Matrix3x4, Rotation3, Vector3};
use crate::{LINKS, _DEF_POS};

pub fn ik(height: f32, body_angle: Vector3<f32>, foot_pos: Matrix3x4<f32>) -> [[f32;3];4]{
    //整体移至重心重合原点，身体不旋转，足端反向旋转
    let shoulder = _DEF_POS;
    let mut down_foot_pos = foot_pos;
    for h in down_foot_pos.row_mut(2).iter_mut() {
        *h -= height;//高度下降
    }
    //足端反向旋转
    let down_foot_pos = rot(- body_angle[0], - body_angle[1], - body_angle[2]) * down_foot_pos;
    let foot: Vec<Vec<f32>> = down_foot_pos.column_iter().map(|col| col.iter().cloned().collect()).collect();
    // println!("foot: {:?}, shoulder: {:?}", foot, shoulder);
    
    let mut next_angle: [[f32;3];4] = [[0.0_f32;3];4];
    for num in 0..4 {
        //计算肩部x轴夹角
        let dyz = distance(&point![0.0, foot[num][1], foot[num][2]], &point![0.0, shoulder[num][1], shoulder[num][2]]);//yoz面,肩关节和足端距离
        let lyz = (dyz.powi(2) - LINKS[0].powi(2)).sqrt();//yoz面,上关节和足端距离
        let gamma_yz = - ((foot[num][1] - shoulder[num][1]) / (foot[num][2] - shoulder[num][2])).atan();
        let gamma_link = (LINKS[0] as f32).atan2(lyz);
        let gamma = match num {
            0 | 3 => gamma_yz - gamma_link,
            1 | 2 => gamma_yz + gamma_link,
            _ => 0.0
        };
        //下关节旋转角
        let lxz = ((foot[num][0]  - shoulder[num][0]).powi(2) + lyz.powi(2)).sqrt();//xoz面上关节点和足端距离
        let delta_n = (lxz.powi(2) - LINKS[1].powi(2) - LINKS[2].powi(2)) / (2.0 * LINKS[1] as f32);
        let beta = - (delta_n / LINKS[2] as f32).acos();
        //上关节旋转角
        let alpha_xz = - ((foot[num][0]  - shoulder[num][0]) / lyz).atan();
        let alpha_link = (((LINKS[1] as f32) + delta_n) / lxz).acos();
        let alpha = alpha_xz + alpha_link;
        
        next_angle[num][0] = gamma;
        next_angle[num][1] = alpha;
        next_angle[num][2] = - beta;//由于连杆结构，取正角度便于计算
    }
    ////////////////////////////////
    // let gamma = next_angle[0][0];
    // let alpha = next_angle[0][1];
    // let beta = next_angle[0][2];
    // println!("gamma:{},alpha:{},beta:{}",gamma*180.0/PI,alpha*180.0/PI,beta*180.0/PI);
    // println!("next_angle:{:?}", next_angle);
    ///////////////////////////////
    next_angle
}

pub fn rot(roll: f32, pitch: f32, yaw: f32) -> Matrix3<f32> {
    let rot = Rotation3::from_euler_angles(roll, pitch, yaw);
    rot.into_inner()
}
//函数待做
//整体上下移动，身体在空间内任意位置