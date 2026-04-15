use std::env;
use std::path::PathBuf;

fn main() {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let epio_root = manifest_dir.join("third_party/epio");
    let apio_root = manifest_dir.join("third_party/apio");
    let epio_src = epio_root.join("src");
    let epio_include = epio_root.join("include");
    let apio_include = apio_root.join("include");

    for p in [&epio_src, &epio_include, &apio_include] {
        assert!(
            p.exists(),
            "missing: {}\n\nRun `git submodule update --init --recursive` \
             in the workspace root.",
            p.display()
        );
    }

    let shim_dir = manifest_dir.join("src/c_shim");

    // ---- Compile libepio.a (epio + our C shim) ----------------------------
    let mut build = cc::Build::new();
    build
        .compiler("clang")
        .include(&epio_include)
        .include(&apio_include)
        .include(&shim_dir)
        .define("APIO_EMULATION", "1")
        .flag_if_supported("-Wno-unused-parameter")
        .flag_if_supported("-Wno-unused-function")
        .flag_if_supported("-Wno-sign-compare");

    for entry in std::fs::read_dir(&epio_src).expect("read epio/src") {
        let path = entry.unwrap().path();
        if path.extension().and_then(|e| e.to_str()) == Some("c") {
            println!("cargo:rerun-if-changed={}", path.display());
            build.file(path);
        }
    }

    let shim_src = shim_dir.join("trace_gen_core.c");
    println!("cargo:rerun-if-changed={}", shim_src.display());
    build.file(&shim_src);

    build.compile("epio");

    // ---- Generate Rust bindings for epio.h --------------------------------
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

    let epio_bindings = bindgen::Builder::default()
        .header(manifest_dir.join("wrapper.h").to_string_lossy())
        .clang_arg(format!("-I{}", epio_include.display()))
        .clang_arg(format!("-I{}", apio_include.display()))
        .clang_arg("-DAPIO_EMULATION=1")
        .allowlist_function("epio_.*")
        .allowlist_type("epio_.*")
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate()
        .expect("bindgen failed to generate bindings for epio.h");

    epio_bindings
        .write_to_file(out_dir.join("epio_bindings.rs"))
        .expect("failed to write epio_bindings.rs");

    // ---- Generate Rust bindings for trace_gen_core.h ----------------------
    let shim_header = shim_dir.join("trace_gen_core.h");
    let shim_bindings = bindgen::Builder::default()
        .header(shim_header.to_string_lossy())
        .allowlist_function("trace_gen_.*")
        .allowlist_type("trace_gen_.*")
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate()
        .expect("bindgen failed to generate bindings for trace_gen_core.h");

    shim_bindings
        .write_to_file(out_dir.join("shim_bindings.rs"))
        .expect("failed to write shim_bindings.rs");

    println!("cargo:rerun-if-changed=wrapper.h");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed={}", shim_header.display());
}
