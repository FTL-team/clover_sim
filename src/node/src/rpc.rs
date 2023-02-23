use std::{fs::File, sync::Arc};

use super::errhelper::JoinNodeError;
use super::{launch::launch::launch, simhandle::SimulatorHandle, task::Task, workspace::Workspace};
use async_broadcast::Sender;
use async_trait::async_trait;
use cloversim_lib::err::NodeError;
use cloversim_lib::{
    err::NodeResult,
    launch_event::LaunchEvent,
    rpc::{NodeRpc, NodeStatus, TaskInfo},
};

macro_rules! rpc_runner {
    ($code:expr) => {
        tokio::spawn($code)
            .await
            .map_err(|e| JoinNodeError::from(e))?
    };
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
    async fn status(&self) -> NodeResult<NodeStatus> {
        Ok(NodeStatus {
            is_ok: true,
            launch_status: self.launch.get_status(),
        })
    }

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

    async fn export_workspace(&self, name: String, to: File) -> NodeResult<()> {
        rpc_runner!(async move {
            let ws = Workspace::load(name.as_str()).await?;
            ws.export(to).await?;
            Ok(())
        })
    }

    async fn import_workspace(&self, from: File) -> NodeResult<String> {
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

    async fn launch(&self, ws_name: String, task_path: String) -> NodeResult<()> {
        let launch_handle = self.launch.clone();
        rpc_runner!(async move {
            let ws = Workspace::load(ws_name.as_str()).await?;
            let task = Task::load_rel(task_path).await?;

            launch(launch_handle.clone(), ws, task).await?;

            Ok(())
        })
    }

    async fn create_launch_channel(&self) -> NodeResult<Sender<LaunchEvent>> {
        Ok(self.launch.create_new_sender())
    }

    async fn path_task_file(&self, file: String) -> NodeResult<String> {
        self.launch.get_task_file(&file)
    }

    async fn read_task_file(&self, file: String) -> NodeResult<Vec<u8>> {
        let launch_handle = self.launch.clone();
        rpc_runner!(async move {
            let path = launch_handle.get_task_file(&file)?;

            tokio::fs::read(&path)
                .await
                .map_err(|e| NodeError::IoError(path, e.to_string()))
        })
    }
}
