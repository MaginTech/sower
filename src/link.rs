extern crate ndarray;
extern crate ndarray_linalg;

use ndarray::prelude::*;

#[derive(Clone, PartialEq)]
pub enum JointType {
    Fix,
    Free,
    Revolute,
    Prismatic, 
}

impl Default for JointType {
    fn default() -> Self { JointType::Revolute }
}

#[derive(Clone, PartialEq)]
pub enum JointMode {
    Active,
    Passive,
}

impl Default for JointMode {
    fn default() -> Self { JointMode::Active }
}

/// joint structure.
#[derive(Clone, PartialEq, Default)]
pub struct Joint{
    joint_type: JointType,
    mode: JointMode,

    // joint displacement
    position: Array1<f64>
}

impl Joint{
    pub fn position(&self) -> &Array1<f64> {
        &self.position
    }
}

#[test]
fn test_joint_position(){
    let joint = Joint { 
        position : array![1.5,],
        ..Default::default() };

    assert_eq!(array![1.5,], joint.position());
}

#[derive(Clone, PartialEq)]
enum LinkType {
    Rigid,
}

impl Default for LinkType {
    fn default() -> Self { LinkType::Rigid }
}

/// link structure.
#[derive(Clone, PartialEq, Default)]
pub struct Link{
    name: String,
    id: u32,
    link_type: LinkType,
    
    // arm vector of link
    arm_vec: Array1<f64>,
    // arm rotation of link
    arm_rot: Array2<f64>,

    joint: Joint,

    parent: Option<Box<Link>>,
    child: Option<Box<Link>>,
    brother: Option<Box<Link>>,
    
    // position from root link
    position: Array1<f64>,
    // rotation matrix from root link
    rotation: Array2<f64>,

    // // twist velocity from root link
    // twist_vel: Array1<f64>,
    // // twist accelaration from root link
    // twist_acc: Array1<f64>,
}

impl Link{
    pub fn position(&self) -> &Array1<f64> {
        &self.position
    }

    pub fn rotation(&self) -> &Array2<f64> {
        &self.rotation
    }

    pub fn update_link_frame(l: Link){
    // if child
    //     match Link.type{
    //         fix =>  println!("to be implement");,
    //         free =>  println!("to be implement");,
    //         revolute => 
    //             child.position = position + arm_pos;
    //             let rot = arm_rot * rot_mat(joit.position(0));
    //             child.rotation = arm_rot * rotation;,
    //         prismatic => println!("to be implement"),
    //     }
    }
}

#[test]
fn test_link_position(){
    let link = Link { 
        position : array![0., 1., 2.],
        ..Default::default() };

    assert_eq!(array![0., 1., 2.], link.position());
}

#[test]
fn test_update_link_frame() {
    let mut child_joint = Joint {
        joint_type : JointType::Revolute,
        mode : JointMode::Active,
        position : array![5.],
    };

    let mut child = Link {
        name : "1".to_string(),
        id : 1,
        link_type : LinkType::Rigid,
        arm_vec : array![0., 1., 0.],
        arm_rot : array![[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]],
        joint : child_joint,
        parent : None, 
        child : None, 
        brother : None, 
        position : array![0., 1., 0.],
        rotation : array![[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]],
    };

    let mut link_joint = Joint {
        joint_type : JointType::Revolute,
        mode : JointMode::Active,
        position : array![5.],
    };

    let mut link = Link {
        name : "0".to_string(),
        id : 0,
        link_type : LinkType::Rigid,
        arm_vec : array![0., 1., 0.],
        arm_rot : array![[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]],
        joint : link_joint,
        parent : None, 
        child : Some(Box::new(child)), 
        brother : None, 
        position : array![0., 1., 0.],
        rotation : array![[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]],
    };

    // assert_eq!(array![0., 1., 0.], link.child);
}
