#![no_std]
#![feature(alloc)]
#![feature(extern_crate_item_prelude)]
#![feature(lang_items)]
extern crate alloc;

#[cfg(feature = "use_std")]
extern crate rust_alloc_sim;

#[cfg(feature = "not_std")]
extern crate rust_alloc_chibios;

use alloc::vec::Vec;

extern "C" {
    #[link_name = "\u{1}actuators"]
    pub static mut actuators: [i16; 6usize];
}

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

    unsafe { actuators[0] = 42; }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
