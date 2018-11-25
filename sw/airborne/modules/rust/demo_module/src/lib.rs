#![no_std]
#![feature(alloc)]
#![feature(extern_crate_item_prelude)]
extern crate alloc;
extern crate rust_alloc_chibios;

use alloc::vec::Vec;

#[no_mangle]
pub extern "C" fn rust_function() {
    let mut v = Vec::new();
    v.push(1);
}

#[no_mangle]
pub extern "C" fn rust_periodic() {
    let mut v = Vec::new();
    v.push(1);
    v.push(2);
    v.pop();
}



#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
