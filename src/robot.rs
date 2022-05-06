extern crate nalgebra as na;

use crate::math::*;
use crate::link::*;

// robot structure.
#[derive(Clone, PartialEq)]
pub struct Robot{
    name: String,
    id: u32,
    dof : u32,

    // link
    root : Option<Box<Link>>,

    // general coordinate
    gen_coord: na::DVector<f64>,
    gen_veloc: na::DVector<f64>,
    gen_accel: na::DVector<f64>,
    gen_force: na::DVector<f64>,
}

impl Default for Robot {
    fn default() -> Self { 
        Self{
            name: Default::default(),
            id: Default::default(),
            dof: 1,
            root: Default::default(), 
            gen_coord: na::DVector::from_element(1, 0.),
            gen_veloc: na::DVector::from_element(1, 0.),
            gen_accel: na::DVector::from_element(1, 0.),
            gen_force: na::DVector::from_element(1, 0.),
        }
    }
}