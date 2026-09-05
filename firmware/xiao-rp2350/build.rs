//! Linker-script setup for the XIAO RP2350 image.

use std::env;
use std::path::PathBuf;

fn main() {
    let Some(out_dir) = env::var_os("OUT_DIR") else {
        panic!("Cargo did not set OUT_DIR");
    };
    let output = PathBuf::from(out_dir).join("memory.x");
    if let Err(error) = std::fs::write(&output, include_bytes!("memory.x")) {
        panic!("failed to write {}: {error}", output.display());
    }
    let Some(parent) = output.parent() else {
        panic!("memory.x output path has no parent");
    };
    println!("cargo:rustc-link-search={}", parent.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
