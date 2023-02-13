use std::{path::PathBuf, sync::Arc};

use super::{
    err::NodeResult, launch::launch::launch, simhandle::SimulatorHandle, task::Task,
    workspace::Workspace,
};
use async_trait::async_trait;

macro_rules! rpc_runner {
    ($code:expr) => {
        tokio::spawn($code).await?
    };
}

pub struct TaskInfo {
    pub relpath: String,
    pub name: String,
    pub description: String,
    pub version: String,
}

#[async_trait]
pub trait NodeRpc {
    async fn create_workspace(&self, name: String) -> NodeResult<()>;
    async fn remove_workspace(&self, name: String) -> NodeResult<()>;
    async fn list_workspaces(&self) -> NodeResult<Vec<String>>;

    async fn export_workspace(&self, name: String) -> NodeResult<String>;
    async fn import_workspace(&self, from: PathBuf) -> NodeResult<String>;

    async fn duplicate_workspace(&self, name: String, newname: String) -> NodeResult<()>;
    async fn clean_workspace(&self, name: String) -> NodeResult<()>;

    async fn list_tasks(&self) -> NodeResult<Vec<TaskInfo>>;

    async fn launch(&self, ws_name: String, task_path: String) -> NodeResult<()>;
    fn get_sim_handle(&self) -> Arc<SimulatorHandle>;
}

pub struct LocalRpc {
    launch: Arc<SimulatorHandle>,
}

impl LocalRpc {
    pub fn new() -> Self {
        LocalRpc {
            launch: Arc::new(SimulatorHandle::new()),
        }
    }
}

#[async_trait]
impl NodeRpc for LocalRpc {
    async fn create_workspace(&self, name: String) -> NodeResult<()> {
        rpc_runner!(async {
            Workspace::new(name).await?;
            Ok(())
        })
    }

    async fn list_workspaces(&self) -> NodeResult<Vec<String>> {
        rpc_runner!(async { Workspace::list().await })
    }

    async fn remove_workspace(&self, name: String) -> NodeResult<()> {
        rpc_runner!(async move {
            let ws = Workspace::load(name.as_str()).await?;
            ws.remove().await?;
            Ok(())
        })
    }

    async fn export_workspace(&self, name: String) -> NodeResult<String> {
        rpc_runner!(async move {
            let ws = Workspace::load(name.as_str()).await?;
            let exported = ws.export().await?;
            Ok(String::from(
                exported
                    .to_str()
                    .expect("Path contains invalid(non-utf8) chars"),
            ))
        })
    }

    async fn import_workspace(&self, from: PathBuf) -> NodeResult<String> {
        rpc_runner!(async move { Workspace::import_workspace(from).await })
    }

    async fn duplicate_workspace(&self, name: String, newname: String) -> NodeResult<()> {
        rpc_runner!(async move {
            let ws = Workspace::load(name.as_str()).await?;
            ws.duplicate(newname).await?;
            Ok(())
        })
    }

    async fn clean_workspace(&self, name: String) -> NodeResult<()> {
        rpc_runner!(async move {
            let ws = Workspace::load(name.as_str()).await?;
            ws.clean().await?;
            Ok(())
        })
    }

    async fn launch(&self, ws_name: String, task_path: String) -> NodeResult<()> {
        let launch_handle = self.launch.clone();
        rpc_runner!(async move {
            let ws = Workspace::load(ws_name.as_str()).await?;
            let task = Task::load_rel(task_path).await?;
            launch(launch_handle, ws, task).await?;

            Ok(())
        })
    }

    fn get_sim_handle(&self) -> Arc<SimulatorHandle> {
        self.launch.clone()
    }

    async fn list_tasks(&self) -> NodeResult<Vec<TaskInfo>> {
        let tasks = Task::list().await?;
        Ok(tasks
            .into_iter()
            .map(|task| TaskInfo {
                relpath: task.relpath,
                name: task.name,
                description: task.description,
                version: task.version,
            })
            .collect::<Vec<_>>())
    }
}
