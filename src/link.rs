// extern crate ndarray;
// extern crate ndarray_linalg;

// use ndarray::prelude::*;

extern crate nalgebra as na;

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

    // joint displacement
    // position: Array1<f64>
    position: na::DVector<f64>
}

impl Default for Joint {
    fn default() -> Self { 
        Self{
            joint_type: JointType::Revolute,
            mode: JointMode::Active,
            position : na::DVector::from_element(1, 0.)
        }
    }
}

 impl Joint{

}

#[test]
fn test_joint_position(){
    let joint = Joint { 
        position : na::DVector::from_element(1, 1.5),
        ..Default::default() };
    let ref_value = na::DVector::from_element(1, 1.5);
    assert_eq!(ref_value, joint.position);
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
    
    // arm vector of link
    // arm_vec: Array1<f64>,
    arm_vec: na::Vector3<f64>,

    // arm rotation of link
    // arm_rot: Array2<f64>,
    arm_rot: na::Matrix3<f64>,

    joint: Joint,

    parent: Option<Box<Link>>,
    child: Option<Box<Link>>,
    brother: Option<Box<Link>>,
    
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
}

impl Default for Link {
    fn default() -> Self { 
        Self{
            name: Default::default(),
            id: Default::default(),
            link_type: LinkType::Rigid,
            arm_vec: na::Vector3::new(0., 0., 0.),
            arm_rot: na::Matrix3::new
                (1., 0., 0.,
                 0., 1., 0.,
                 0., 0., 1.),
            joint: Default::default(),
            parent: None,
            child: None,
            brother: None,
            position: na::Vector3::new(0., 0., 0.),
            rotation: na::Matrix3::new
                (1., 0., 0.,
                 0., 1., 0.,
                 0., 0., 1.),
        }
    }
}

impl Link{
    // fn mut_child(&mut self) -> Option<Box<Link>> {
    //     self.child.borrow_mut()
    // }

    fn joint_type(&self) -> &JointType{
        &self.joint.joint_type
    }

    pub fn position(&self) -> na::Vector3<f64> {
        self.position
    }

    pub fn rotation(&self) -> na::Matrix3<f64> {
        self.rotation
    }

    pub fn update_link_framemut(&mut self){
        let parent = &self.parent;
         if let Some(p) = parent { 
            match self.joint_type(){
                JointType::Fix => println!("to be implement"),
                JointType::Free => println!("to be implement"),
                JointType::Revolute => 
                {
                    self.position = p.position + p.arm_vec;
                    // let rot = arm_rot * rot_mat(joit.position);
                    let rot = na::Matrix3::<f64>::identity();
                    self.rotation = rot * p.arm_rot *  p.rotation;
                },
                JointType::Prismatic => println!("to be implement"),
            }
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
        position: na::Vector3::new(0., 1., 0.),
        ..Default::default() 
    };

    let mut joint = Joint {
        position : na::DVector::from_element(1, 1.5),
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

    // if let Some(p) = link.parent { 
    //     assert_eq!(na::Vector3::new(0., 1., 0.), p.position());
    // }

    assert_eq!(na::Vector3::new(0., 0., 0.), link.position);
}
