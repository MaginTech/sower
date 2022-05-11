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
    root : Option<Box<Link>>,

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
            root: Default::default(), 
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
    pub fn update_frame(&mut self){
        if let Some(r) =  &mut self.root { 
            r.update_link_frame();
        }
    }

    pub fn update_twist_vel(&mut self){
        if let Some(r) =  &mut self.root { 
            r.update_twist_vel();
        }
    }

    pub fn update_twist_acc(&mut self){
        if let Some(r) =  &mut self.root { 
            r.update_twist_acc();
        }
    }
}