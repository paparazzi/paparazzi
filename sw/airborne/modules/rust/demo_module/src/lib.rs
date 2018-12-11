#![no_std]
#![feature(alloc)]
#![feature(extern_crate_item_prelude)]
#![feature(lang_items)]

#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
include!(concat!(env!("OUT_DIR"), "/airframe.rs"));
//include!(concat!(env!("OUT_DIR"), "/state.rs"));

extern crate alloc;

#[cfg(feature = "use_std")]
extern crate rust_alloc_sim;

#[cfg(feature = "not_std")]
extern crate rust_alloc_chibios;

use alloc::vec::Vec;

#[no_mangle]
pub extern "C" fn rust_function() {
    let mut v = Vec::new();
    v.push(SERVO_AILERON_LEFT);
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
