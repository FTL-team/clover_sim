use std::{
    env,
    path::{Path, PathBuf},
};

pub fn get_local_path(sub: impl AsRef<Path>) -> PathBuf {
    env::current_dir()
        .expect("Cannot get current directory")
        .join(sub)
}
