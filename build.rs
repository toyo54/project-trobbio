use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

    // Copy link.x into OUT_DIR and expose OUT_DIR as a link-search path.
    // cargo:rustc-link-search PROPAGATES to dependent crates (unlike
    // cargo:rustc-link-arg, which does not), so this is what lets any
    // downstream binary find `-Tlink.x` by name without ever holding a
    // physical copy of the file — the content stays single-sourced here,
    // in src/link.x, instead of being copy-pasted into every consumer.
    fs::write(out_dir.join("link.x"), include_bytes!("src/link.x")).unwrap();
    println!("cargo:rustc-link-search={}", out_dir.display());
    println!("cargo:rerun-if-changed=src/link.x");

    // This crate's own src/main.rs still needs the flag emitted directly
    // (Scenario A: cloning this repo and building it as-is) — the search
    // path above makes the file discoverable, this line tells the linker
    // to actually apply it. A downstream binary depending on this crate
    // from crates.io needs this exact line in ITS OWN build.rs too, since
    // rustc-link-arg never propagates across a dependency edge — but
    // needs no copy of link.x itself, thanks to the search path.
    println!("cargo:rustc-link-arg=-Tlink.x");

    // rust-lld is invoked directly here (not via a gcc frontend), so
    // linker flags go through WITHOUT the "-Wl," passthrough prefix —
    // that prefix is for gcc/clang forwarding args to their linker, and
    // rust-lld chokes on it as one unrecognized literal argument.
    // This flag makes the linker print which input sections don't match
    // any rule in link.x's SECTIONS block (RISC-V small-data variants,
    // .eh_frame, etc.) instead of silently placing them wherever its
    // default orphan logic decides — which is what was fragmenting the
    // flash image into more segments than the ROM bootloader tolerates.
    println!("cargo:rustc-link-arg=--orphan-handling=warn");
}
