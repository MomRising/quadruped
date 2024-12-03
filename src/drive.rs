use std::f32::consts::{FRAC_PI_2, FRAC_PI_4, PI};
// use serialport::SerialPort;

use crate::TIME_STEP;
const ANGLE_CENTER: [f32;3] = [0.0, FRAC_PI_4, FRAC_PI_2];//机械校准。数组0，1，2分别代表肩，大腿，小腿
const ANGLE_MAX: [f32;3] = [//范围均可调整
    FRAC_PI_2 - 0.2,//舵机臂机械碰撞极限角度
    FRAC_PI_2,
    PI - FRAC_PI_4];
const ANGLE_RANGE: [[f32;2];3] = [
[ANGLE_CENTER[0] - ANGLE_MAX[0]/2.0, ANGLE_CENTER[0] + ANGLE_MAX[0]/2.0],//肩关节不为0.0的话，需改动不同腿
[ANGLE_CENTER[1] - ANGLE_MAX[1]/2.0, ANGLE_CENTER[1] + ANGLE_MAX[1]/2.0],
[ANGLE_CENTER[2] - ANGLE_MAX[2]/2.0, ANGLE_CENTER[2] + ANGLE_MAX[2]/2.0]];
const ANGLE_BETWEEN_ALPHA_BETA: f32 = 0.3;//两腿间夹角
const PWM_ERR: [[i32;3];4] = [[55, -65, -40],[-10, -10, 0],[50, -35, 35],[10, 10, 0],];//矫正
#[cfg(test)]
mod tests {
    use std::f32::consts::{FRAC_2_PI, FRAC_PI_3};

    use super::*; 
    #[test]
    fn test2() {
        // stm32([[0.0, FRAC_PI_2, FRAC_PI_2];4]);
        // stm32([[0.0, FRAC_PI_4, FRAC_PI_2];4]);
    }
}
pub fn stm32(mut angle: [[f32;3];4]) -> [u8;43] {
    let mut cmd:[u8;43] = [
        0x55, 0x55,
        0x29, 0x03,//数据长度，指令代码
        0x0C, 0x14, 0x00,//数量，时间低八位，高八位
        0x00, 0xE8, 0x03, 0x01, 0xE8, 0x03, 0x02, 0xE8, 0x03,//序号，位置低八位，高八位
        0x03, 0xE8, 0x03, 0x04, 0xE8, 0x03, 0x05, 0xE8, 0x03,
        0x06, 0xE8, 0x03, 0x07, 0xE8, 0x03, 0x08, 0xE8, 0x03,
        0x09, 0xE8, 0x03, 0x0A, 0xE8, 0x03, 0x0B, 0xE8, 0x03
    ];
    // let mut cmd:[u8;16] = [
    //     0x55, 0x55,
    //     0x0E, 0x03,//数据长度，指令代码
    //     0x03, 0x14, 0x00,//数量，时间低八位，高八位
    //     0x00, 0xE8, 0x03, 0x01, 0xE8, 0x03, 0x02, 0xE8, 0x03];

    // 时间设定
    let done_time = to_hex((TIME_STEP * 1000.0) as u32);
    (cmd[5], cmd[6]) = (done_time[0], done_time[1]);
    //角度设定
    for leg in 0..4{
        angle[leg][2] = angle[leg][1] + (PI - angle[leg][2]);
        if (angle[leg][2] - angle[leg][1]) < ANGLE_BETWEEN_ALPHA_BETA {
            angle[leg][2] = angle[leg][1] + ANGLE_BETWEEN_ALPHA_BETA;//大腿和小腿舵机间角度不低于设定值，不然上下腿会碰撞
        }
        for joint in 0..3 {
            let pwm = manage(angle[leg][joint], leg, joint);
            let angle_hex = to_hex(pwm as u32);
            (cmd[8 + leg * 9 + joint * 3], cmd[9 + leg * 9 + joint * 3]) = (angle_hex[0], angle_hex[1]);
        }
    }
    cmd
}
fn manage(angle: f32, leg: usize, joint: usize)  -> i32 {//角度映射到pwm频率
    let angle_processed = match angle {
        x if x < ANGLE_RANGE[joint][0] => ANGLE_RANGE[joint][0],
        x if x > ANGLE_RANGE[joint][1] => ANGLE_RANGE[joint][1],
        _ => angle,
    };
    let mut pwm = ((angle_processed - ANGLE_CENTER[joint]) / PI * 2000.0 + 1500.0) as i32;
    if let (0,2)|(1,0)|(1,1)|(2,1)|(3,0)|(3,2) = (leg, joint){
        pwm = 3000 - pwm//轴朝右和朝后的舵机pwm输出关于1500对称。
    }
    pwm += PWM_ERR[leg][joint];//先对称，后修正
    pwm
}
fn to_hex(num: u32) -> [u8;2] {
    let hex = num.to_be_bytes();//转化为[0,0,高八位，低八位]
    [hex[3], hex[2]]
}