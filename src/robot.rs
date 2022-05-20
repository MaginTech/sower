extern crate nalgebra as na;

use crate::math::*;
use crate::link::*;

// robot structure.
#[derive(Clone, PartialEq)]
pub struct Robot{
    name: String,
    id: usize,
    dof: usize,
    link_num: usize,
    joint_num: usize,
    joint_dof: usize,

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
            link_num : 1,
            joint_num : 1,
            joint_dof : 1,
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
    fn link(&mut self, i : usize) -> &Link{
        &self.links[i]
    }

    pub fn set_gen_coord(&mut self, q : na::DVector<f64>){
        if let JointType::Free = self.links[0].joint_type(){

        }

        let mut indx = 0;
        for n in 1..self.link_num {
            let mut pos = na::DVector::from_element(self.links[n].joint_dof(), 0.);
            for m in 0..self.links[n].joint_dof()-1 {
                pos[m] = q[indx + m];
            }
            self.links[n].set_joint_pos( pos );
            indx += self.links[n].joint_dof();
        }
    }

    pub fn set_gen_veloc(&mut self, dq : na::DVector<f64>){
        if let JointType::Free = self.links[0].joint_type(){
            
        }

        let mut indx = 0;
        for n in 1..self.link_num {
            let mut vel = na::DVector::from_element(self.links[n].joint_dof(), 0.);
            for m in 0..self.links[n].joint_dof()-1 {
                vel[m] = dq[indx + m];
            }
            self.links[n].set_joint_vel( vel );
            indx += self.links[n].joint_dof();
        }
    }

    pub fn set_gen_accel(&mut self, ddq : na::DVector<f64>){
        if let JointType::Free = self.links[0].joint_type(){
            
        }

        let mut indx = 0;
        for n in 1..self.link_num {
            let mut acc = na::DVector::from_element(self.links[n].joint_dof(), 0.);
            for m in 0..self.links[n].joint_dof()-1 {
                acc[m] = ddq[indx + m];
            }
            self.links[n].set_joint_acc( acc );
            indx += self.links[n].joint_dof();
        }
    }

    pub fn set_gen_force(&mut self, f : na::DVector<f64>){
        if let JointType::Free = self.links[0].joint_type(){
            
        }

        let mut indx = 0;
        for n in 1..self.link_num {
            let mut tau = na::DVector::from_element(self.links[n].joint_dof(), 0.);
            for m in 0..self.links[n].joint_dof()-1 {
                tau[m] = f[indx + m];
            }
            self.links[n].set_joint_torque( tau );
            indx += self.links[n].joint_dof();
        }
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