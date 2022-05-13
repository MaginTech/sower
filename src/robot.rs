extern crate nalgebra as na;

use crate::math::*;
use crate::link::*;

// robot structure.
#[derive(Clone, PartialEq)]
pub struct Robot{
    name: String,
    id: u32,
    dof: u32,

    // link
    links : Box<[Link]>,

    mass: f64,

    inertia: na::DMatrix<f64>,
    bias: na::DVector<f64>,

    // general coordinate
    gen_coord: na::DVector<f64>,
    gen_veloc: na::DVector<f64>,
    gen_accel: na::DVector<f64>,
    gen_force: na::DVector<f64>,

    // center of gravity
    cog: na::Vector3<f64>,
}

impl Default for Robot {
    fn default() -> Self { 
        Self{
            name: Default::default(),
            id: Default::default(),
            dof: 1,
            links : Default::default(),
            mass: 1.0,
            inertia: na::DMatrix::from_element(1, 1, 0.),
            bias: na::DVector::from_element(1, 0.),
            gen_coord: na::DVector::from_element(1, 0.),
            gen_veloc: na::DVector::from_element(1, 0.),
            gen_accel: na::DVector::from_element(1, 0.),
            gen_force: na::DVector::from_element(1, 0.),
            cog: na::Vector3::new(0., 0., 0.),
        }
    }
}

impl Robot{

    pub fn set_gen_coord(&mut self, _q : na::DVector<f64>){
    }

    pub fn set_gen_veloc(&mut self, _dq : na::DVector<f64>){
    }

    pub fn set_gen_accel(&mut self, _ddq : na::DVector<f64>){
    }

    pub fn set_gen_force(&mut self, _f : na::DVector<f64>){
    }

    pub fn update_frame(&mut self){
        self.links[0].update_link_frame();
    }

    pub fn update_twist_vel(&mut self){
        self.links[0].update_twist_vel();
    }

    pub fn update_twist_acc(&mut self){
        self.links[0].update_twist_acc();
    }
}