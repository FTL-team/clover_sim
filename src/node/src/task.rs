use std::{io, path::PathBuf};

use cloversim_lib::err::{NodeError, NodeResult};
use futures::future::join_all;
use serde::Deserialize;

use crate::errhelper::{IoNodeError, JoinNodeError};

use super::local_files::get_local_path;

#[derive(Deserialize)]
struct TaskPackageXml {
    description: String,
    version: String,
    name: String,
}

pub struct Task {
    pub fullpath: PathBuf,
    pub relpath: String,
    pub name: String,
    pub description: String,
    pub version: String,
}

impl Task {
    pub async fn load(fullpath: PathBuf) -> NodeResult<Task> {
        let tasks_path = get_local_path("tasks");
        let relpath = fullpath
            .strip_prefix(&tasks_path)
            .expect("Task is not in tasks")
            .to_string_lossy()
            .to_string();

        let package_xml = fullpath.join("package.xml");
        let package_xml = match tokio::fs::read(&package_xml).await {
            Ok(data) => data,
            Err(ref e) if matches!(e.kind(), io::ErrorKind::NotFound) => {
                return Err(NodeError::TaskNotFound(relpath))
            }
            Err(e) => return Err(IoNodeError::create_fs_error(&package_xml, e)),
        };

        let info: Result<TaskPackageXml, _> =
            serde_xml_rs::de::from_str(&String::from_utf8_lossy(&package_xml));
        let info = match info {
            Ok(info) => info,
            Err(err) => return Err(NodeError::InvalidTask(err.to_string())),
        };

        Ok(Task {
            relpath,
            fullpath,

            name: info.name,
            description: info.description,
            version: info.version,
        })
    }

    pub async fn load_rel(relpath: String) -> Result<Task, NodeError> {
        let tasks_path = get_local_path("tasks");
        Self::load(tasks_path.join(relpath)).await
    }

    fn find_all_paths() -> NodeResult<Vec<PathBuf>> {
        let tasks_path = get_local_path("tasks");
        let mut paths_to_check = Vec::new();
        paths_to_check.push(tasks_path);
        let mut tasks = Vec::new();
        loop {
            let current_path = paths_to_check.pop();
            match current_path {
                Some(current_path) => {
                    if current_path.join("package.xml").exists() {
                        tasks.push(current_path);
                    } else {
                        let errmap = IoNodeError::create_fs_errmap(&current_path);
                        let dir = current_path.read_dir().map_err(&errmap)?;
                        for entry in dir.into_iter() {
                            let entry = entry.map_err(&errmap)?;
                            let entry_type = entry.file_type().map_err(&errmap)?;
                            if entry_type.is_dir() {
                                paths_to_check.push(entry.path());
                            }
                        }
                    }
                }
                None => break,
            }
        }

        Ok(tasks)
    }

    pub async fn list() -> NodeResult<Vec<Task>> {
        let tasks = tokio::task::spawn_blocking(Self::find_all_paths)
            .await
            .map_err(JoinNodeError::from)??;
        let tasks = tasks.into_iter().map(Self::load);
        let tasks: NodeResult<Vec<Task>> = join_all(tasks).await.into_iter().collect();
        tasks
    }
}
