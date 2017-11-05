/*
 * Copyright (C) 2017  Michal Podhradsky <mpodhradsky@galois.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */
/// Key generator for two pairs of asymmetric keys
/// Generates two header files:
/// 	- keys_uav.h is used for the autopilot
/// 	- keys_gcs.h is used by the GCS
///
extern crate rusthacl;
extern crate rand;

use rusthacl::*;

use rand::Rng;
use rand::os::OsRng;
use std::error::Error;
use std::io::prelude::*;
use std::fs::File;
use std::path::Path;
use std::env;

const KEY_LEN: usize = 32;

static HEAD: &'static str = "#ifndef PPRZLINK_KEYS_H\n#define PPRZLINK_KEYS_H\n";
static TAIL: &'static str = "#endif /* PPRZLINK_KEYS_H */\n";

fn print_array(name: &str, b: &[u8]) -> String {
    let mut data = String::from("#define ");
    data += name;
    data += " {";
    for byte in b {
        let val = format!("{},", byte);
        data += &val;
    }
    data += "}\n";
    return data;
}

fn main() {
    // get the destination directory
    let dir = match env::args().nth(1) {
        Some(val) => val + "/",
        None => String::new(),  
    };
    

    // generate public and private keys
    let mut rng = match OsRng::new() {
        Ok(gen) => gen,
        Err(e) => panic!("Failed to obtain OS RNG: {}", e),
    };

    let mut q_a: [u8; KEY_LEN] = [0; KEY_LEN]; // private A
    let mut p_a: [u8; KEY_LEN] = [0; KEY_LEN]; // public A
    let mut q_b: [u8; KEY_LEN] = [0; KEY_LEN]; // private B
    let mut p_b: [u8; KEY_LEN] = [0; KEY_LEN]; // public B

    rng.fill_bytes(&mut q_a);
    rng.fill_bytes(&mut q_b);

    ed25519_secret_to_public(&mut p_a, &q_a).unwrap();
    ed25519_secret_to_public(&mut p_b, &q_b).unwrap();

    {
        // UAV KEYS
        // open file to write keys to
        let path = dir.clone() + "keys_uav.h";
        let path = Path::new(&path);
        let display = path.display();

        // Open a file in write-only mode, returns `io::Result<File>`
        let mut file = match File::create(&path) {
            Err(why) => panic!("couldn't create {}: {}", display, why.description()),
            Ok(file) => file,
        };

        // convert keys to writable format
        let mut keys = String::new();
        keys += &print_array("GCS_PUBLIC", &p_a);
        keys += &print_array("UAV_PUBLIC", &p_b);
        keys += &print_array("UAV_PRIVATE", &q_b);


        // Write to the file
        match file.write_all(HEAD.as_bytes()) {
            Err(why) => panic!("couldn't write to {}: {}", display, why.description()),
            Ok(_) => (),
        }

        // write the keys
        match file.write_all(keys.as_bytes()) {
            Err(why) => panic!("couldn't write to {}: {}", display, why.description()),
            Ok(_) => (),
        }

        // Write to the file
        match file.write_all(TAIL.as_bytes()) {
            Err(why) => panic!("couldn't write to {}: {}", display, why.description()),
            Ok(_) => (),
        }
    }

    {
        // GCS KEYS
        // open file to write keys to
        let path = dir.clone() + "keys_gcs.h";
        let path = Path::new(&path);
        let display = path.display();

        // Open a file in write-only mode, returns `io::Result<File>`
        let mut file = match File::create(&path) {
            Err(why) => panic!("couldn't create {}: {}", display, why.description()),
            Ok(file) => file,
        };

        // convert keys to writable format
        let mut keys = String::new();
        keys += &print_array("GCS_PUBLIC", &p_a);
        keys += &print_array("GCS_PRIVATE", &q_a);
        keys += &print_array("UAV_PUBLIC", &p_b);

        // Write to the file
        match file.write_all(HEAD.as_bytes()) {
            Err(why) => panic!("couldn't write to {}: {}", display, why.description()),
            Ok(_) => (),
        }

        // write the keys
        match file.write_all(keys.as_bytes()) {
            Err(why) => panic!("couldn't write to {}: {}", display, why.description()),
            Ok(_) => (),
        }

        // Write to the file
        match file.write_all(TAIL.as_bytes()) {
            Err(why) => panic!("couldn't write to {}: {}", display, why.description()),
            Ok(_) => (),
        }
    }
}
