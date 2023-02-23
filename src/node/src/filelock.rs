use std::path::PathBuf;

#[derive(Debug)]
pub struct FileLock {
    pub path: PathBuf,
}

impl FileLock {
    // Lock is synchronys to ensure no race conditions  (at least locally)
    pub fn lock(path: &PathBuf) -> Result<Self, std::io::Error> {
        let lock_path = path.join("_.lock");

        let f = std::fs::File::options()
            .create_new(true)
            .write(true)
            .read(true)
            .open(&lock_path)?;
        f.sync_all()?;
        Ok(FileLock { path: lock_path })
    }
}

impl Drop for FileLock {
    #[allow(unused_must_use)]
    fn drop(&mut self) {
        std::fs::remove_file(&self.path);
    }
}
