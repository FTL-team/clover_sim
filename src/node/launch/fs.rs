use std::path::PathBuf;
use sys_mount::{Mount, UnmountDrop, UnmountFlags};

use crate::node::{
    err::{NodeError, NodeResult},
    local_files::get_local_path,
    workroot::WorkEntry,
    workspace::Workspace,
};

pub trait OverlayEntry: Send + Sync {
    fn get_path(&self) -> PathBuf;
}

pub trait UpperOverlayEntry: OverlayEntry {
    fn get_work(&self) -> PathBuf;
}

pub struct BaseOverlayEntry {
    _mount: UnmountDrop<Mount>,
    work_entry: WorkEntry,
}

impl BaseOverlayEntry {
    pub async fn new(work_entry: WorkEntry) -> NodeResult<Self> {
        let sqsh_path = get_local_path("base.sqsh");
        if !sqsh_path.exists() {
            return Err(NodeError::NoBaseFSFound);
        }

        let target_path = work_entry.path();
        tokio::fs::create_dir(target_path)
            .await
            .map_err(NodeError::create_fs_errmap(target_path))?;

        tokio::task::spawn_blocking(move || {
            let squash_mount = Mount::builder()
                .fstype("squashfs")
                .mount_autodrop(sqsh_path, work_entry.path(), UnmountFlags::DETACH)
                .map_err(NodeError::create_io_errmap("mount<base.sqsh>"))?;

            Ok(BaseOverlayEntry {
                _mount: squash_mount,
                work_entry,
            })
        })
        .await?
    }
}

impl OverlayEntry for BaseOverlayEntry {
    fn get_path(&self) -> PathBuf {
        self.work_entry.path().clone()
    }
}

pub struct TmpfsOverlayEntry {
    _mount: UnmountDrop<Mount>,
    work_entry: WorkEntry,
}

impl TmpfsOverlayEntry {
    pub async fn new(work_entry: WorkEntry) -> NodeResult<Self> {
        let target_path = work_entry.path();
        let fs_path = target_path.join("fs");
        let work_path = target_path.join("overlay_work");

        tokio::fs::create_dir(target_path)
            .await
            .map_err(NodeError::create_fs_errmap(target_path))?;

        let entry = tokio::task::spawn_blocking(move || -> NodeResult<Self> {
            let squash_mount = Mount::builder()
                .fstype("tmpfs")
                .mount_autodrop("tmpfs", work_entry.path(), UnmountFlags::DETACH)
                .map_err(NodeError::create_io_errmap("mount<tmpfs>"))?;

            Ok(Self {
                _mount: squash_mount,
                work_entry,
            })
        })
        .await??;

        tokio::fs::create_dir(&fs_path)
            .await
            .map_err(NodeError::create_fs_errmap(&fs_path))?;

        tokio::fs::create_dir(&work_path)
            .await
            .map_err(NodeError::create_fs_errmap(&work_path))?;

        Ok(entry)
    }
}

impl OverlayEntry for TmpfsOverlayEntry {
    fn get_path(&self) -> PathBuf {
        self.work_entry.path().join("fs")
    }
}

impl UpperOverlayEntry for TmpfsOverlayEntry {
    fn get_work(&self) -> PathBuf {
        self.work_entry.path().join("overlay_work")
    }
}

pub struct WorkspaceOverlayEntry {
    work_path: PathBuf,
    workspace: Workspace,
}

impl WorkspaceOverlayEntry {
    pub async fn new(workspace: Workspace) -> NodeResult<Self> {
        let target_path = &workspace.path;
        let work_path = target_path.join("overlay_work");

        if let Err(err) = tokio::fs::create_dir(&work_path).await {
            match err.kind() {
                std::io::ErrorKind::AlreadyExists => {}
                _ => return Err(NodeError::create_fs_error(&work_path, err)),
            }
        }

        Ok(Self {
            work_path,
            workspace,
        })
    }
}

impl OverlayEntry for WorkspaceOverlayEntry {
    fn get_path(&self) -> PathBuf {
        self.workspace.path.join("fs")
    }
}

impl UpperOverlayEntry for WorkspaceOverlayEntry {
    fn get_work(&self) -> PathBuf {
        self.work_path.clone()
    }
}
