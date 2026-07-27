use std::env;
use std::process::Command;

fn main() {
    println!("cargo:rerun-if-env-changed=GIT_COMMIT_SHA");
    println!("cargo:rerun-if-changed=.git/HEAD");

    let commit = env::var("GIT_COMMIT_SHA")
        .ok()
        .filter(|value| !value.trim().is_empty())
        .or_else(|| {
            Command::new("git")
                .args(["rev-parse", "HEAD"])
                .output()
                .ok()
                .filter(|output| output.status.success())
                .and_then(|output| String::from_utf8(output.stdout).ok())
                .map(|value| value.trim().to_string())
                .filter(|value| !value.is_empty())
        })
        .unwrap_or_else(|| "unknown".to_string());

    println!("cargo:rustc-env=GIT_COMMIT_SHA={commit}");
}
