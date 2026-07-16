use std::path::Path;

use postcard_bindgen::{PackageInfo, generate_bindings, python};
use shared_types::{CmdFromPC, TelemFromPC, TelemToPC};

fn main() {
    let mut output_dir = std::env::current_dir().unwrap();
    output_dir.push("synnax-client-py");
    output_dir.push("build");

    python::build_package(
        &output_dir,
        PackageInfo {
            name: "servo_reg_com".into(),
            version: "0.1.0".try_into().unwrap(),
        },
        python::GenerationSettings::enable_all(),
        generate_bindings!(CmdFromPC, TelemToPC, TelemFromPC),
    )
    .unwrap();
    print!("Python bindings generated!");
}
