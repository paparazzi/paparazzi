#![feature(lang_items, core_intrinsics)]
#![feature(start)]
#![feature(alloc)]
#![feature(alloc_system)]
#![feature(allocator_api)]
#![feature(extern_crate_item_prelude)] 
#![feature(alloc_error_handler)]
 
#![no_std]
use core::panic::PanicInfo;
extern crate alloc;
extern crate alloc_system;
 
use alloc_system::System;
 
#[global_allocator]
static GLOBAL: System = System;
 
#[lang = "eh_personality"]
#[no_mangle]
pub extern "C" fn rust_eh_personality() {}
 
#[lang = "eh_unwind_resume"]
#[no_mangle]
pub extern "C" fn rust_eh_unwind_resume() {}
 
#[no_mangle]
#[allow(non_snake_case)]
pub extern "C" fn _Unwind_Resume() {}

#[panic_handler]
#[no_mangle]
pub extern fn panic(_info: &PanicInfo) -> ! {
	loop{}
}

// See https://github.com/rust-lang/rust/issues/51540
#[alloc_error_handler]
fn foo(_: core::alloc::Layout) -> ! {
    loop {}
}
