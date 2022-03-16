#![warn(clippy::all)]

extern crate aerso;
extern crate matplotrust;
mod aircraft;

fn main() {
    
    use aerso::*;
    use aerso::types::*;
    use matplotrust::*;
    
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
    let mut x_pos = Vec::new();
    let mut y_pos = Vec::new();
    let mut z_pos = Vec::new();
    let mut u_vel = Vec::new();
    let mut v_vel = Vec::new();
    let mut w_vel = Vec::new();
    let mut alpha_vec = Vec::new();
    let mut time_vec = Vec::new();
    while time < 0.1 {
        vehicle.step(delta_t, &[0.1, 0.1, 0.1, 0.1]);
        time += delta_t;
        println!("position: {}",vehicle.position());
        x_pos.push(vehicle.position()[0]);
        y_pos.push(vehicle.position()[1]);
        z_pos.push(vehicle.position()[2]);
        u_vel.push(vehicle.velocity()[0]);
        v_vel.push(vehicle.velocity()[1]);
        w_vel.push(vehicle.velocity()[2]);
        time_vec.push(time);
        let airstate = vehicle.body.get_airstate();
        println!("alpha: {}, beta: {}, airspeed: {}, dyn_pressure: {}", airstate.alpha, airstate.beta,airstate.airspeed, airstate.q);
        alpha_vec.push(airstate.alpha);
    }
    let x_time = &time_vec;
    let y_time = &time_vec;
    let z_time = &time_vec;
    let u_time = &time_vec;
    let v_time = &time_vec;
    let w_time = &time_vec;
    let alpha_time = &time_vec;
    let xp = line_plot::<f64, f64>(x_time.to_vec(), x_pos, None);
    let yp = line_plot::<f64, f64>(y_time.to_vec(), y_pos, None);
    let zp = line_plot::<f64, f64>(z_time.to_vec(), z_pos, None);
    let up = line_plot::<f64, f64>(u_time.to_vec(), u_vel, None);
    let vp = line_plot::<f64, f64>(v_time.to_vec(), v_vel, None);
    let wp = line_plot::<f64, f64>(w_time.to_vec(), w_vel, None);
    let alphap = line_plot::<f64, f64>(alpha_time.to_vec(), alpha_vec, None);
    let mut figure = Figure::new();
    // figure.add_plot(xp.clone());
    // figure.add_plot(yp.clone());
    //figure.add_plot(zp.clone());
    // figure.add_plot(alphap.clone());
    figure.add_plot(up.clone());
    figure.add_plot(vp.clone());
    figure.add_plot(wp.clone());
    figure.save("./plot.png", Some("/opt/miniconda3/envs/d2p-env/bin/python"));
}