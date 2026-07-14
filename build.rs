// build.rs
fn main() {
    // memory.x isn't a .rs file, so cargo has no built-in reason to notice
    // when it changes. Without this, editing memory.x and rebuilding can
    // silently relink against a stale cached binary.
    println!("cargo:rerun-if-changed=memory.x");

    // Tell rustc to pass `-Tlink.x` to the linker so the RISC-V layout is applied
    println!("cargo:rustc-link-arg=-Tlink.x");
}
