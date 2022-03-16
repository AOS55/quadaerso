use argmin::prelude::*;

struct TargetPosition{
    x: f64,
    y: f64,
    z: f64
}

impl ArgminOp for TargetPosition {
    type Param = Vec<f64>;
    type Output = f64;
    type Float = f64;

    // Apply cost function to parameter p
    fn apply(&self, p: &self::Param) -> Result<Self::Output, Error> {
        Ok(calculate_cost(p, self.x, self.y, self.z))
    }
}

fn calculate_cost(p: vec!<f64; 4>, x_tar: f64, y_tar: f64, z_tar: f64) {
    let pos = step_one(p);
    let tar_pos = vec![x_tar, y_tar, z_tar];
    pos - tar_pos
}

fn step_one(thrust) {
    vehicle.step(delta_t, &thrust);
    vehicle.position()
}