#![no_std]
#![feature(alloc)]
#![feature(extern_crate_item_prelude)]
#![feature(alloc_error_handler)]
#![feature(lang_items)] 

#[allow(unused_extern_crates)] // NOTE(allow) bug rust-lang/rust#53964
extern crate panic_halt; // panic handler

extern crate alloc;

use core::alloc::{GlobalAlloc, Layout};
use core::ptr;

#[lang = "eh_personality"]
#[no_mangle]
pub extern "C" fn rust_eh_personality() {}


// See https://github.com/rust-lang/rust/issues/51540
#[alloc_error_handler]
fn foo(_: core::alloc::Layout) -> ! {
    loop {}
}


extern "C" {
	pub fn chHeapFree(p: *mut u8);
    pub fn chHeapStatus(heapp: *mut u8, sizep: *mut usize, sizep2: *mut usize) -> usize;
    /// Note: we have to call `chHeapAllocAligned` instead of `chHeapAlloc` because
    /// paparazzi optimizes the `chHeapAlloc` call and doesn't export it.
    /// As a result we have to guess the alignment. I am using a cfg feature
    /// to distinguish between 32-bit and 16-bit targets
    pub fn chHeapAllocAligned(heapp: *mut u8, size: usize, align: u8) -> *mut u8;
}

pub struct RustChibiOsAllocator;

unsafe impl GlobalAlloc for RustChibiOsAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
            #[cfg(feature = "arch32")]
            {
                chHeapAllocAligned(ptr::null_mut(), layout.size(), 8)
            }
            #[cfg(feature = "arch16")]
            {
                chHeapAllocAligned(ptr::null_mut(), layout.size(), 4)
            }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, _layout: Layout) {
            chHeapFree(ptr)
    }
}

#[global_allocator]
static GLOBAL: RustChibiOsAllocator = RustChibiOsAllocator;
