extern "C" {
    #[link_name = "\u{1}actuators_delay_time"]
    pub static mut actuators_delay_time: u32;
}
extern "C" {
    #[link_name = "\u{1}actuators_delay_done"]
    pub static mut actuators_delay_done: bool;
}
extern "C" {
    #[link_name = "\u{1}actuators"]
    pub static mut actuators: [i16; 6usize];
}
extern "C" {
    #[link_name = "\u{1}actuators_pprz"]
    pub static mut actuators_pprz: [i16; 6usize];
}
