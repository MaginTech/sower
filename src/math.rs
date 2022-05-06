extern crate nalgebra as na;

#[allow(dead_code)]
pub fn rot_mat(axis : na::Vector3<f64>, angle : f64 ) -> na::Matrix3<f64> {
    let n = skew_symmetric(axis);
    let rot = na::Matrix3::<f64>::identity() 
            + n * angle.sin()
            + n * n * (1. - angle.sin());
    rot
}

#[test]
fn test_rot_mat(){
    let axis = na::Vector3::<f64>::new(0., 0., 1.);
    let ang = std::f64::consts::PI / 4.;
    
    let mat = na::Matrix3::new
        (2_f64.sqrt()/2., -2_f64.sqrt()/2., 0.,
         2_f64.sqrt()/2.,  2_f64.sqrt()/2., 0.,
         0., 0., 1.);
    
    let abs_difference = (mat - rot_mat(axis, ang)).abs();
    assert!(abs_difference.norm() < 1e-10);
}


#[allow(dead_code)]
pub fn skew_symmetric(vec : na::Vector3<f64>) -> na::Matrix3<f64> {
    let mat =
        na::Matrix3::new
            (0., -vec[2], vec[1],
             vec[2], 0., -vec[0],
             -vec[1], vec[0], 0.);
    mat
}

#[test]
fn test_skew_symmetric(){
    let vec = na::Vector3::<f64>::new(2.5, 3.1, 0.4);
    
    let mat = na::Matrix3::new
        (0., -0.4, 3.1,
         0.4, 0., -2.5,
         -3.1, 2.5, 0.);

    assert_eq!(mat, skew_symmetric(vec));
}