// extern crate ndarray;
// extern crate ndarray_linalg;

// use ndarray::prelude::*;

extern crate nalgebra as na;

use crate::math::*;

#[derive(Clone, PartialEq)]
pub enum JointType {
    Fix,
    Free,
    Revolute,
    Prismatic, 
}

#[derive(Clone, PartialEq)]
pub enum JointMode {
    Active,
    Passive,
}

/// joint structure.
#[derive(Clone, PartialEq)]
pub struct Joint{
    joint_type: JointType,
    mode: JointMode,
    dof: usize,
    // joint displacement
    // position: Array1<f64>
    pos: na::DVector<f64>,
    vel: na::DVector<f64>,
    acc: na::DVector<f64>,
    torque: na::DVector<f64>,
}

impl Default for Joint {
    fn default() -> Self { 
        Self{
            joint_type: JointType::Revolute,
            mode: JointMode::Active,
            dof: 1,
            pos : na::DVector::from_element(1, 0.),
            vel :  na::DVector::from_element(1, 0.),
            acc :  na::DVector::from_element(1, 0.),
            torque :  na::DVector::from_element(1, 0.),
        }
    }
}

impl Joint{
    pub fn set_pos(&mut self, pos : na::DVector<f64>){
        self.pos = pos;
    }

    pub fn set_vel(&mut self, vel : na::DVector<f64>){
        self.vel = vel;
    }

    pub fn set_acc(&mut self, acc : na::DVector<f64>){
        self.acc = acc;
    }

    pub fn set_torque(&mut self, torque : na::DVector<f64>){
        self.torque = torque;
    }
}

#[test]
fn test_joint_position(){
    let joint = Joint { 
        pos : na::DVector::from_element(1, 1.5),
        ..Default::default() };
    let ref_value = na::DVector::from_element(1, 1.5);
    assert_eq!(ref_value, joint.pos);
}

#[derive(Clone, PartialEq)]
enum LinkType {
    Rigid,
}

impl Default for LinkType {
    fn default() -> Self { LinkType::Rigid }
}

/// link structure.
#[derive(Clone, PartialEq)]
pub struct Link{
    name: String,
    id: u32,
    link_type: LinkType,

    mass: f64,
    
    // arm vector of link
    // arm_vec: Array1<f64>,
    arm_vec: na::Vector3<f64>,

    // arm rotation of link
    // arm_rot: Array2<f64>,
    arm_rot: na::Matrix3<f64>,

    joint: Joint,

    parent: Option<Box<Link>>,
    child: Option<Box<Link>>,
    sibling: Option<Box<Link>>,
    
    // position from root link
    // position: Array1<f64>,
    position: na::Vector3<f64>,

    // rotation matrix from root link
    // rotation: Array2<f64>,
    rotation: na::Matrix3<f64>,

    // // twist velocity from root link
    // twist_vel: Array1<f64>,
    // // twist accelaration from root link
    // twist_acc: Array1<f64>,

    // center of gravity
    cog: na::Vector3<f64>,
}

impl Default for Link {
    fn default() -> Self { 
        Self{
            name: Default::default(),
            id: Default::default(),
            link_type: LinkType::Rigid,
            mass: 0.,
            arm_vec: na::Vector3::new(0., 0., 0.),
            arm_rot: na::Matrix3::new
                (1., 0., 0.,
                 0., 1., 0.,
                 0., 0., 1.),
            joint: Default::default(),
            parent: None,
            child: None,
            sibling: None,
            position: na::Vector3::new(0., 0., 0.),
            rotation: na::Matrix3::new
                (1., 0., 0.,
                 0., 1., 0.,
                 0., 0., 1.),
            cog: na::Vector3::new(0., 0., 0.),
        }
    }
}

impl Link{
    // fn mut_child(&mut self) -> Option<Box<Link>> {
    //     self.child.borrow_mut()
    // }
    pub fn joint_dof(&self) -> usize{
        self.joint.dof
    }

    pub fn joint_type(&self) -> &JointType{
        &self.joint.joint_type
    }

    pub fn set_joint_pos(&mut self, pos : na::DVector<f64>){
        self.joint.set_pos(pos);
    }

    pub fn set_joint_vel(&mut self, pos : na::DVector<f64>){
        self.joint.set_vel(pos);
    }

    pub fn set_joint_acc(&mut self, pos : na::DVector<f64>){
        self.joint.set_acc(pos);
    }

    pub fn set_joint_torque(&mut self, torque : na::DVector<f64>){
        self.joint.set_torque(torque);
    }

    pub fn position(&self) -> na::Vector3<f64> {
        self.position
    }

    pub fn rotation(&self) -> na::Matrix3<f64> {
        self.rotation
    }

    pub fn update_link_frame(&mut self){
        if let Some(p) =  &self.parent { 
            match self.joint_type(){
                JointType::Fix => 
                {
                    println!("to be implemented");
                },
                JointType::Free =>
                {
                    println!("to be implemented");
                },
                JointType::Revolute => 
                {
                    let rot = rot_mat(na::Vector3::z(), self.joint.pos[0]);
                    self.rotation = p.rotation * p.arm_rot * rot; 
                    self.position = p.position + p.rotation * p.arm_vec;
                },
                JointType::Prismatic => 
                {
                    self.rotation = p.rotation * p.arm_rot; 
                    let pos =  self.rotation * self.joint.pos[0] * na::Vector3::z();
                    self.position = p.position + p.rotation * p.arm_vec + pos;
                },
            }
        }

        if let Some(c) =  &mut self.child { 
            c.update_link_frame();
        }

        if let Some(s) =  &mut self.sibling { 
            s.update_link_frame();
        }
    }

    pub fn update_twist_vel(&mut self){
        if let Some(p) =  &self.parent { 
            match self.joint_type(){
                JointType::Fix => 
                {
                    println!("to be implemented");
                },
                JointType::Free =>
                {
                    println!("to be implemented");
                },
                JointType::Revolute => 
                {
                    println!("to be implemented");
                },
                JointType::Prismatic => 
                {
                    println!("to be implemented");
                },
            }
        }

        if let Some(c) =  &mut self.child { 
            c.update_twist_vel();
        }

        if let Some(s) =  &mut self.sibling { 
            s.update_twist_vel();
        }
    }

    pub fn update_twist_acc(&mut self){
        if let Some(p) =  &self.parent { 
            match self.joint_type(){
                JointType::Fix => 
                {
                    println!("to be implemented");
                },
                JointType::Free =>
                {
                    println!("to be implemented");
                },
                JointType::Revolute => 
                {
                    println!("to be implemented");
                },
                JointType::Prismatic => 
                {
                    println!("to be implemented");
                },
            }
        }

        if let Some(c) =  &mut self.child { 
            c.update_twist_acc();
        }

        if let Some(s) =  &mut self.sibling { 
            s.update_twist_acc();
        }
    }
}

#[test]
fn test_link_position(){
    let link = Link { 
        position : na::Vector3::new(0., 1., 2.),
        ..Default::default() };

    let ref_value = na::Vector3::new(0., 1., 2.);
    assert_eq!(ref_value, link.position());
}

#[test]
fn test_update_link_frame() {
    let parent = Link { 
        name : "0".to_string(),
        id : 0,
        arm_vec: na::Vector3::new(1., 0., 2.),
        arm_rot: na::Matrix3::new
        (2_f64.sqrt()/2., -2_f64.sqrt()/2., 0.,
         2_f64.sqrt()/2.,  2_f64.sqrt()/2., 0.,
         0., 0., 1.),
        position: na::Vector3::new(0., 1., 0.5),
        ..Default::default() 
    };

    let joint = Joint {
        pos : na::DVector::from_element(1, std::f64::consts::PI / 4.),
        ..Default::default()
    };

    let mut link = Link {
        name : "1".to_string(),
        id : 1,
        joint : joint,
        position: na::Vector3::new(0., 1., 0.),
        parent : Some(Box::new(parent)), 
        ..Default::default()
    };

    link.update_link_frame();

    assert_eq!(na::Vector3::new(1., 1., 2.5), link.position);

    let mat = na::Matrix3::new
        (0., -1., 0.,
         1.,  0., 0.,
         0.,  0., 1.);
    let abs_difference = (mat - link.rotation).abs();
    assert!(abs_difference.norm() < 1e-10);
}

#[test]
fn test_update_twist_vel(){
    let parent = Link { 
        name : "0".to_string(),
        id : 0,
        arm_vec: na::Vector3::new(1., 0., 2.),
        arm_rot: na::Matrix3::new
        (2_f64.sqrt()/2., -2_f64.sqrt()/2., 0.,
         2_f64.sqrt()/2.,  2_f64.sqrt()/2., 0.,
         0., 0., 1.),
        position: na::Vector3::new(0., 1., 0.5),
        ..Default::default() 
    };

    let joint = Joint {
        pos : na::DVector::from_element(1, std::f64::consts::PI / 4.),
        ..Default::default()
    };

    let mut link = Link {
        name : "1".to_string(),
        id : 1,
        joint : joint,
        position: na::Vector3::new(0., 1., 0.),
        parent : Some(Box::new(parent)), 
        ..Default::default()
    };

    link.update_twist_vel();
}

#[test]
fn test_update_twist_acc(){
    let parent = Link { 
        name : "0".to_string(),
        id : 0,
        arm_vec: na::Vector3::new(1., 0., 2.),
        arm_rot: na::Matrix3::new
        (2_f64.sqrt()/2., -2_f64.sqrt()/2., 0.,
         2_f64.sqrt()/2.,  2_f64.sqrt()/2., 0.,
         0., 0., 1.),
        position: na::Vector3::new(0., 1., 0.5),
        ..Default::default() 
    };

    let joint = Joint {
        pos : na::DVector::from_element(1, std::f64::consts::PI / 4.),
        ..Default::default()
    };

    let mut link = Link {
        name : "1".to_string(),
        id : 1,
        joint : joint,
        position: na::Vector3::new(0., 1., 0.),
        parent : Some(Box::new(parent)), 
        ..Default::default()
    };

    link.update_twist_acc();
}