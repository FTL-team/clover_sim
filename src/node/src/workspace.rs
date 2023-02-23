use cloversim_lib::err::{InvalidWorkspaceKind, NodeError, NodeResult};
use serde::{Deserialize, Serialize};
use std::{
    self,
    fs::File,
    path::{Path, PathBuf},
    process::Stdio,
};

use crate::{
    errhelper::{IoNodeError, JoinNodeError},
    filelock::FileLock,
};

use super::cmd::execute_cmd;

#[derive(Debug, Deserialize, Serialize)]
pub struct WorkspaceSerialized {
    pub name: String,
}

#[derive(Debug)]
pub struct Workspace {
    pub name: String,
    pub path: PathBuf,
    pub lock: FileLock,
}

const EXPORT_NAME: &str = "export.tar.gz";

const CLEAN_DIRS: &[&str] = &[
    "fs/var/log",
    "fs/var/cache",
    "fs/home/clover/.ros/log",
    "fs/home/clover/.cache",
    "_.lock",
    "overlay_work",
    ".overlay_work",
    EXPORT_NAME,
];

static WORKSPACES_DIR: &str = "workspaces";

impl Workspace {
    fn path_from_name(name: &str) -> PathBuf {
        return Path::new(WORKSPACES_DIR).join(name);
    }

    fn create_workspace_lock(path: &PathBuf) -> NodeResult<FileLock> {
        match FileLock::lock(path) {
            Ok(lock) => Ok(lock),
            Err(err) => {
                return Err(match err.kind() {
                    std::io::ErrorKind::AlreadyExists => NodeError::WorkspaceIsUsed,
                    _ => IoNodeError::create_fs_error(&path.join("_.lock"), err),
                })
            }
        }
    }

    async fn load_workspace_meta(path: &PathBuf, name: &str) -> NodeResult<WorkspaceSerialized> {
        let meta_path = path.join("workspace.yml");
        let meta_path_string = String::from(meta_path.to_str().unwrap_or("???"));

        let workspace_serialized = match tokio::fs::read_to_string(&meta_path).await {
            Ok(serialized) => serialized,
            Err(err) => {
                return Err(match err.kind() {
                    std::io::ErrorKind::NotFound => {
                        NodeError::WorkspaceNotFound(String::from(name))
                    }
                    _ => NodeError::IoError(meta_path_string, err.to_string()),
                })
            }
        };

        match serde_yaml::from_str::<WorkspaceSerialized>(workspace_serialized.as_str()) {
            Ok(w) => Ok(w),
            Err(err) => Err(NodeError::FailedToParseFile(
                meta_path_string,
                err.to_string(),
            )),
        }
    }

    pub async fn list() -> NodeResult<Vec<String>> {
        tokio::task::spawn_blocking(|| {
            let ws_path = PathBuf::from(WORKSPACES_DIR);
            let err_mapper = IoNodeError::create_fs_errmap(&ws_path);
            let dir_contents = std::fs::read_dir(&ws_path).map_err(&err_mapper)?;

            dir_contents
                .into_iter()
                .map(|x| {
                    Ok(x.map_err(&err_mapper)?
                        .file_name()
                        .into_string()
                        .expect("Non utf8 filename"))
                })
                .collect()
        })
        .await
        .map_err(|x| JoinNodeError::from(x))?
    }

    pub async fn load(dirname: &str) -> NodeResult<Self> {
        let path = Self::path_from_name(dirname);
        let meta = Self::load_workspace_meta(&path, dirname).await?;
        let lock = Self::create_workspace_lock(&path)?;

        if meta.name != dirname {
            return Err(NodeError::InvalidWorkspace(
                InvalidWorkspaceKind::DirnameNotMatches,
            ));
        }

        Ok(Workspace {
            name: meta.name,
            lock,
            path,
        })
    }

    async fn save(&self) -> NodeResult<()> {
        let path = Self::path_from_name(&self.name).join("workspace.yml");
        let path_string = String::from(path.to_str().unwrap_or("???"));

        let serialized = match serde_yaml::to_string(&WorkspaceSerialized {
            name: self.name.clone(),
        }) {
            Ok(w) => w,
            Err(err) => return Err(NodeError::FailedToParseFile(path_string, err.to_string())),
        };

        match tokio::fs::write(&path, serialized).await {
            Ok(_) => Ok(()),
            Err(err) => Err(NodeError::IoError(path_string, err.to_string())),
        }
    }

    pub async fn new(name: String) -> NodeResult<Self> {
        let path = Self::path_from_name(&name);
        if let Err(err) = tokio::fs::create_dir(&path).await {
            return Err(match err.kind() {
                std::io::ErrorKind::AlreadyExists => NodeError::WorkspaceAlreadyExists(name),
                _ => NodeError::IoError(
                    String::from(path.to_str().unwrap_or("???")),
                    err.to_string(),
                ),
            });
        }

        if let Err(err) = tokio::fs::create_dir(path.join("fs")).await {
            return Err(NodeError::IoError(
                String::from(path.join("fs").to_str().unwrap_or("???")),
                err.to_string(),
            ));
        }

        let lock = Self::create_workspace_lock(&path)?;

        let w = Self { name, lock, path };
        w.save().await?;

        Ok(w)
    }

    pub async fn remove(self) -> NodeResult<()> {
        let Workspace { path, .. } = self;
        tokio::fs::remove_dir_all(&path)
            .await
            .map_err(IoNodeError::create_fs_errmap(&path))?;
        Ok(())
    }

    pub async fn export(&self, to: File) -> NodeResult<()> {
        let mut cmd = tokio::process::Command::new("tar");
        for to_exclude in CLEAN_DIRS.iter() {
            cmd.arg("--exclude").arg(to_exclude);
        }
        cmd.arg("-zc").arg(".");
        cmd.current_dir(&self.path);
        cmd.stdout(to);
        cmd.stderr(Stdio::inherit());

        let res = cmd
            .spawn()
            .map_err(IoNodeError::create_errmap("<tar export>"))?
            .wait()
            .await
            .map_err(IoNodeError::create_errmap("<tar export>"))?;

        if !res.success() {
            return Err(NodeError::CmdError(
                String::from("<tar export>"),
                String::from("cannot get output"),
            ));
        }

        execute_cmd(cmd, "tar export").await?;
        Ok(())
    }

    pub async fn import_workspace(from: File) -> NodeResult<String> {
        let target_extract =
            PathBuf::from(WORKSPACES_DIR).join(format!("__import_{:04x}", rand::random::<u16>()));

        tokio::fs::create_dir(&target_extract)
            .await
            .map_err(IoNodeError::create_fs_errmap(&target_extract))?;

        let mut cmd = tokio::process::Command::new("tar");
        cmd.arg("-zxf").arg("-").arg("-C").arg(&target_extract);
        cmd.stdin(from);

        let extract_closure = async {
            execute_cmd(cmd, "tar import").await?;

            let meta = Self::load_workspace_meta(&target_extract, "<import_ws>").await?;
            let final_path = Self::path_from_name(meta.name.as_str());
            if final_path.exists() {
                return Err(NodeError::WorkspaceAlreadyExists(meta.name));
            }

            Ok((meta, final_path))
        };

        let (meta, final_path) = match extract_closure.await {
            Ok(o) => o,
            Err(e) => {
                tokio::fs::remove_dir_all(&target_extract)
                    .await
                    .map_err(IoNodeError::create_fs_errmap(&target_extract))?;

                return Err(e);
            }
        };

        tokio::fs::rename(target_extract, &final_path)
            .await
            .map_err(IoNodeError::create_fs_errmap(&final_path))?;

        Ok(meta.name)
    }

    pub async fn duplicate(&self, newname: String) -> NodeResult<()> {
        let newws = Self::new(newname).await?;

        let mut cmd = tokio::process::Command::new("cp");
        cmd.arg("-rp").arg(self.path.join("fs")).arg(newws.path);
        cmd.stdin(Stdio::null());
        execute_cmd(cmd, "cp workspace").await?;
        Ok(())
    }

    pub async fn clean(&self) -> NodeResult<()> {
        for dir in CLEAN_DIRS {
            if *dir != "_.lock" {
                let target_path = self.path.join(dir);
                match tokio::fs::remove_dir_all(&target_path).await {
                    Ok(_) => (),
                    Err(e) => match e.kind() {
                        std::io::ErrorKind::NotFound => (), // If we didn't find it it is fine
                        _ => return Err(IoNodeError::create_fs_error(&target_path, e)),
                    },
                };
            }
        }

        Ok(())
    }
}
