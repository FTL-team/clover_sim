use crate::{err::NodeResult, launch_event::LaunchEvent};
use async_broadcast::Sender;
use async_trait::async_trait;
use serde::{Deserialize, Serialize};
use std::fs::File;
use ts_rs::TS;

#[derive(Serialize, Deserialize, TS)]
#[ts(export)]
pub struct TaskInfo {
    pub relpath: String,
    pub name: String,
    pub description: String,
    pub version: String,
}

#[derive(Debug, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct NodeStatus {
    pub is_ok: bool,
    pub launch_status: bool,
}

#[async_trait]
pub trait NodeRpc: Sync + Send {
    async fn status(&self) -> NodeResult<NodeStatus>;

    async fn create_workspace(&self, name: String) -> NodeResult<()>;
    async fn remove_workspace(&self, name: String) -> NodeResult<()>;
    async fn list_workspaces(&self) -> NodeResult<Vec<String>>;

    async fn export_workspace(&self, name: String, to: File) -> NodeResult<()>;
    async fn import_workspace(&self, from: File) -> NodeResult<String>;

    async fn duplicate_workspace(&self, name: String, newname: String) -> NodeResult<()>;
    async fn clean_workspace(&self, name: String) -> NodeResult<()>;

    async fn list_tasks(&self) -> NodeResult<Vec<TaskInfo>>;

    async fn launch(&self, ws_name: String, task_path: String) -> NodeResult<()>;

    async fn create_launch_channel(&self) -> NodeResult<Sender<LaunchEvent>>;
    async fn path_task_file(&self, file: String) -> NodeResult<String>;
    async fn read_task_file(&self, file: String) -> NodeResult<Vec<u8>>;
}
