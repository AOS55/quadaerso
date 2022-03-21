extern crate aerso;
use crate::aircraft;
use argmin::prelude::*;
use aerso::*;
use aerso::types::*;

pub struct TargetPosition{
    pub x: f64,
    pub y: f64,
    pub z: f64, 
    pub condition: Conditions,
    pub delta_t: f64,
}

pub struct Conditions{
    pub pos: Vector3,
    pub vel: Vector3,
    pub att: UnitQuaternion,
    pub rate: Vector3
}

impl ArgminOp for TargetPosition {
    type Param = Vec<f64>;
    type Output = f64;
    type Float = f64;
    type Hessian = ();
    type Jacobian = ();
    // Apply cost function to parameter p
    fn apply(&self, p: &Self::Param) -> Result<Self::Output, Error> {
        Ok(calculate_cost(p.to_vec(), self.x, self.y, self.z, &self.condition, self.delta_t))
    }
}

pub fn calculate_cost(p: Vec<f64>, x_tar: f64, y_tar: f64, z_tar: f64, conditions: &Conditions, delta_t: f64) -> f64 {
    let pos = step_one(p, conditions, delta_t);
    let x_diff = x_tar - pos[0];
    let y_diff = y_tar - pos[1];
    let z_diff = z_tar - pos[2];
    let tot_diff = x_diff.powf(2.0) + y_diff.powf(2.0) + z_diff.powf(2.0);
    tot_diff.powf(0.5)  // return the L2 norm of the function
}

pub fn step_one(thrust: Vec<f64>, condition: &Conditions, delta_t: f64) -> Vec<f64> {
    let k_body = Body::new(aircraft::aircraft_mass(), aircraft::aircraft_inertia(), condition.pos, condition.vel, condition.att, condition.rate);
    let a_body = AeroBody::new(k_body);
    let mut vehicle = AffectedBody {
        body: a_body,
        effectors: vec![Box::new(aircraft::Thrust), Box::new(aircraft::Drag)]
    };
    vehicle.step(delta_t, &[thrust[0], thrust[1], thrust[2], thrust[3]]);
    let pos = vehicle.position();
    vec![pos[0], pos[1], pos[2]]
}