use aerso::*;
use aerso::types::*;

/// Quadcopter Geometry
const S: f64 = 0.5;  // Area of body
const C_D: f64 = 1.28;  // Body drag coefficient
const BOOM_LENGTH: f64 = 2.0;  // Length of Boom 
const MAX_THRUST: f64 = 35.0;  // Max Thrust [N]
const BODY_MASS: f64 = 5.0;  // Mass of aircraft's Body [Kg]
const MOTOR_MASS: f64 = 1.0;  // Mass of aircraft's motor [Kg] 

pub struct Thrust;
impl AeroEffect<[f64;4]> for Thrust {
    fn get_effect(&self, _airstate: AirState, _rates: Vector3, inputstate: &[f64;4]) -> (Force,Torque) {
        let mut thrust = inputstate.iter().sum();
        thrust *= - MAX_THRUST;
        
        let torque_0_x;
        let torque_0_y;
        let torque_1_x;
        let torque_1_y;
        let torque_2_x;
        let torque_2_y;
        let torque_3_x;
        let torque_3_y;
        
        if inputstate[0] != 0.0 {
            torque_0_x = MAX_THRUST * inputstate[0].powf(-0.5) * BOOM_LENGTH;
            torque_0_y = MAX_THRUST * inputstate[0].powf(-0.5) * BOOM_LENGTH;
        }
        else{
            torque_0_x = 0.0;
            torque_0_y = 0.0;
        }

        if inputstate[1] != 0.0 {
            torque_1_x = MAX_THRUST * -inputstate[1].powf(-0.5) * BOOM_LENGTH;
            torque_1_y = MAX_THRUST * inputstate[1].powf(-0.5) * BOOM_LENGTH;
        }
        else {
            torque_1_x = 0.0;
            torque_1_y = 0.0;
        }

        if inputstate[2] != 0.0 {
            torque_2_x = MAX_THRUST * -inputstate[2].powf(-0.5) * BOOM_LENGTH;
            torque_2_y = MAX_THRUST * -inputstate[2].powf(-0.5) * BOOM_LENGTH;
        }
        else {
            torque_2_x = 0.0;
            torque_2_y = 0.0;
        }

        if inputstate[3] != 0.0 {
            torque_3_x = MAX_THRUST * inputstate[3].powf(-0.5) * BOOM_LENGTH;
            torque_3_y = MAX_THRUST * -inputstate[3].powf(-0.5) * BOOM_LENGTH;
        }
        else {
            torque_3_x = 0.0;
            torque_3_y = 0.0;
        }
        let torque_x = torque_0_x + torque_1_x + torque_2_x + torque_3_x;
        let torque_y = torque_0_y + torque_1_y + torque_2_y + torque_3_y;
        (Force::body(0.0,0.0,thrust),Torque::body(torque_x, torque_y, 0.0))
    }
}

pub struct Drag;

    impl<I> AeroEffect<I> for Drag {
        fn get_effect(&self, airstate: AirState, _rates: Vector3, _inputstate: &I) -> (Force,Torque) {
            let drag = airstate.q * S * C_D;
            let magnitude = airstate.u.powf(2.0) + airstate.v.powf(2.0) + airstate.w.powf(2.0);
            let magnitude = magnitude.sqrt();
            let drag_x;
            let drag_y;
            let drag_z;
            if magnitude != 0.0 {
                drag_x = -drag * (airstate.u / magnitude).abs();
                drag_y = -drag * (airstate.v / magnitude).abs();
                drag_z = -drag * (airstate.w / magnitude).abs();
            }
            else {
                drag_x = 0.0;
                drag_y = 0.0;
                drag_z = 0.0;
            }
            (Force::body(drag_x, drag_y, drag_z),Torque::body(0.0,0.0,0.0))
        }
    }

pub fn aircraft_mass() -> f64 {
    BODY_MASS + 4.0 * MOTOR_MASS
}

pub fn aircraft_inertia() -> Matrix3 {
    let mut inertia_array = Matrix3::identity();
    let boom_ang: f64 = 45.0;
    let r: f64 = boom_ang.sin() * BOOM_LENGTH;
    let inertia = MOTOR_MASS * r.powf(2.0);
    inertia_array[(0, 0)] = inertia;
    inertia_array[(1, 1)] = inertia;
    inertia_array[(2, 2)] = inertia;
    inertia_array
}