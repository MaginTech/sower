use std::path::Path;

#[allow(dead_code)]
fn file_exist(file: &str) -> bool {
    Path::new(file).exists()
}

#[test]
fn test_file_exist(){
    assert!(file_exist("file name"));
}

#[allow(dead_code)]
pub fn read_urdf(file: &str) -> bool{
    if !file_exist(file) {
        return false;
    }

    true
}

#[test]
fn test_read_urdf(){
    assert!(read_urdf("file name"));
}