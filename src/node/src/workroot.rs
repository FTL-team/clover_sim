use cloversim_lib::err::NodeResult;
use std::{fs::Permissions, path::PathBuf, sync::Arc};

use crate::errhelper::IoNodeError;

use super::clean::Cleaner;

pub struct WorkRoot {
    path: PathBuf,
    cleaner: Option<Arc<Cleaner>>,
}

pub struct WorkEntry {
    path: PathBuf,
    root: Arc<WorkRoot>,
}

impl WorkEntry {
    pub fn path(&self) -> &PathBuf {
        &self.path
    }

    pub fn get(&self, name: &str) -> WorkEntry {
        WorkEntry {
            path: self.path.join(name),
            root: self.root.clone(),
        }
    }

    pub async fn write(&self, contents: impl AsRef<[u8]>) -> NodeResult<()> {
        tokio::fs::write(&self.path, contents)
            .await
            .map_err(IoNodeError::create_fs_errmap(&self.path))
    }

    pub async fn read(&self) -> NodeResult<Vec<u8>> {
        tokio::fs::read(&self.path)
            .await
            .map_err(IoNodeError::create_fs_errmap(&self.path))
    }

    pub async fn create(&self) -> NodeResult<tokio::fs::File> {
        tokio::fs::File::create(&self.path)
            .await
            .map_err(IoNodeError::create_fs_errmap(&self.path))
    }

    pub async fn create_dir_uniq(&self) -> NodeResult<()> {
        tokio::fs::create_dir(&self.path)
            .await
            .map_err(IoNodeError::create_fs_errmap(&self.path))
    }

    pub async fn create_dir(&self) -> NodeResult<()> {
        match tokio::fs::create_dir(&self.path).await {
            Ok(()) => Ok(()),
            Err(ref e) if e.kind() == std::io::ErrorKind::AlreadyExists => Ok(()),
            Err(e) => Err(IoNodeError::create_fs_error(&self.path, e)),
        }
    }

    pub async fn set_permissions(&self, permissions: Permissions) -> NodeResult<()> {
        tokio::fs::set_permissions(self.path(), permissions)
            .await
            .map_err(IoNodeError::create_fs_errmap(self.path()))
    }
}

impl WorkRoot {
    pub async fn new(root: PathBuf, cleaner: Arc<Cleaner>) -> NodeResult<WorkRoot> {
        tokio::fs::create_dir(&root)
            .await
            .map_err(IoNodeError::create_fs_errmap(&root))?;
        Ok(WorkRoot {
            path: root,
            cleaner: Some(cleaner),
        })
    }

    pub fn get(self: &Arc<Self>, name: &str) -> WorkEntry {
        WorkEntry {
            path: self.path.join(name),
            root: self.clone(),
        }
    }

    pub fn as_entry(self: &Arc<Self>) -> WorkEntry {
        WorkEntry {
            path: self.path.clone(),
            root: self.clone(),
        }
    }
}

impl Drop for WorkRoot {
    fn drop(&mut self) {
        let path = self.path.clone();
        if let Some(cleaner) = &self.cleaner {
            cleaner.clone().add_task(tokio::spawn(async move {
                tokio::fs::remove_dir_all(&path)
                    .await
                    .map_err(IoNodeError::create_fs_errmap(&path))?;
                Ok(())
            }))
        }
    }
}
