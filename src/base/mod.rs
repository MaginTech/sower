fn file_exist(_file: &str) -> bool {
    false
}

#[test]
fn test_file_exist(){
    assert!(file_exist("file name"));
}

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