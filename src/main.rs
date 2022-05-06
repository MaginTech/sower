extern crate ndarray;
extern crate ndarray_linalg;

use ndarray::prelude::*;
use ndarray_linalg::Solve;

extern crate nalgebra as na;

use std::time::Instant;
// mod base;
mod math;
// mod link;

fn main() {
    // base::write();

    const N:u32 = 1_000;

    /*ndarray*/
    let a: Array2<f64> = array![[3., 2., -1.], [2., -2., 4.], [-2., 1., -2.]];
    let b: Array1<f64> = array![1., -2., 0.];
    let c: Array1<f64> = array![1., -2., 0.];
    let d: Array1<f64> = array![1., -2., 0.];

    let istart = Instant::now();
    let mut istop = istart;
    
    // for _ in 1..N {
    //     let _x = a.solve_into(b).unwrap();
    // }
    let x = a.solve_into(b).unwrap();

    istop = Instant::now();
    println!("std::time:Instant::now() overhead = {:?}",
        istop.duration_since(istart));

    println!("x={}",x);

    let y = c + d;
    println!("y={}",y);
    // println!("b={}",b);

    /*nalgebra*/
    let v = na::Vector4::new(1, 2, 3, 4);

    let m = na::Matrix3x4::new(11, 12, 13, 14,
                               21, 22, 23, 24,
                               31, 32, 33, 34);
    let u = na::Matrix3x4::zeros();

    let x = (m + u) * v;
    println!("x={}",x);
}