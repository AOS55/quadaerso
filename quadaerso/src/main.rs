#![warn(clippy::all)]

extern crate aerso;
extern crate matplotrust;
mod aircraft;
mod mpc;

use aerso::*;
use aerso::types::*;
use argmin::prelude::*;
use argmin::solver::linesearch::MoreThuenteLineSearch;
use argmin::solver::gradientdescent::SteepestDescent;
use argmin::core::{Error, Executor, ObserverMode};

fn run(cost: mpc::TargetPosition) -> Result<(), Error> {
    let init_param: Vec<f64> = vec![0.0, 0.0, 0.0, 0.0];
    let linesearch = MoreThuenteLineSearch::new();
    let solver = SteepestDescent::new(linesearch);
    let res = Executor::new(cost, solver, init_param)
        .add_observer(ArgminSlogLogger::term(), ObserverMode::Always)
        .max_iters(10)
        .run()?;
    Ok(())
}

fn main() {
    
    let initial_position = Vector3::zeros();
    let initial_velocity = Vector3::zeros();
    let initial_attitude = UnitQuaternion::from_euler_angles(0.0,0.0,0.0);
    let initial_rates = Vector3::zeros();
    
    let k_body = Body::new(aircraft::aircraft_mass(), aircraft::aircraft_inertia(), initial_position, initial_velocity, initial_attitude, initial_rates);

    let a_body = AeroBody::new(k_body);
    
    let mut vehicle = AffectedBody {
        body: a_body,
        effectors: vec![Box::new(aircraft::Thrust), Box::new(aircraft::Drag)],
        };
    
    let delta_t = 0.01;
    let mut time = 0.0;
    
    while time < 0.1 {
        let condition = mpc::Conditions {pos: vehicle.position().clone(),
                                         vel: vehicle.velocity().clone(),
                                         att: vehicle.attitude().clone(),
                                         rate: vehicle.rates().clone()};
        let cost = mpc::TargetPosition {x: 0.0, y: 0.0, z: 0.0, condition, delta_t};
        let res = run(cost);
        println!("Res is: {:?}", res);
        vehicle.step(delta_t, &[0.1, 0.1, 0.1, 0.1]);
        time += delta_t;
        println!("position: {}", vehicle.position());
        // let airstate = vehicle.body.get_airstate();
        // println!("alpha: {}, beta: {}, airspeed: {}, dyn_pressure: {}", airstate.alpha, airstate.beta, airstate.airspeed, airstate.q);
    }
}