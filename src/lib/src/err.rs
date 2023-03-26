use serde::{Deserialize, Serialize};
use std::any::Any;
use ts_rs::TS;

#[derive(Clone, Debug, Deserialize, Serialize, TS)]
#[ts(export)]
pub enum InvalidWorkspaceKind {
    DirnameNotMatches,
}

#[derive(Clone, Debug, Deserialize, Serialize, TS)]
#[ts(export)]
pub enum NodeError {
    InternalError(String),
    IoError(String, String),
    CmdError(String, String),
    TaskCancelled,
    InvalidFilename,
    FailedToParseFile(String, String),
    AlreadyLocked,

    WorkspaceIsUsed,
    WorkspaceNotFound(String),
    WorkspaceAlreadyExists(String),
    InvalidWorkspace(InvalidWorkspaceKind),

    NoBaseFSFound,
    NoX11Display(String),
    VirglError(String),
    ContainerCreateFailed(String),
    ContainersError(Vec<NodeError>),

    AnotherSimmulationRunning,
    NoSimulatorRunning,
    InvalidTaskFile,

    InvalidTask(String),
    TaskNotFound(String),

    InvalidInstallation,

    HttpError(String),
}

impl NodeError {
    pub fn from_panic_any(e: Box<dyn Any + Send>) -> Self {
        let e = match e.downcast::<String>() {
            Ok(err) => return NodeError::InternalError(*err),
            Err(e) => e,
        };

        match e.downcast::<&str>() {
            Ok(err) => return NodeError::InternalError(String::from(*err)),
            Err(e) => e,
        };

        NodeError::InternalError(String::from("Unknown error"))
    }
}

// impl From<JoinError> for NodeError {
//     fn from(e: JoinError) -> Self {
//         if e.is_cancelled() {
//             return NodeError::TaskCancelled;
//         }
//         match e.try_into_panic() {
//             Ok(e) => NodeError::from_panic_any(e),
//             Err(e) => NodeError::InternalError(e.to_string()),
//         }
//     }
// }

// impl NodeError {
//     pub fn create_fs_error(path: &PathBuf, err: io::Error) -> NodeError {
//         NodeError::IoError(
//             String::from(path.to_str().unwrap_or("???")),
//             err.to_string(),
//         )
//     }

//     pub fn create_fs_errmap<'a>(path: &'a PathBuf) -> impl Fn(io::Error) -> NodeError + 'a {
//         |err| Self::create_fs_error(path, err)
//     }

//     pub fn create_io_errmap<'a>(source: &'a str) -> impl Fn(io::Error) -> NodeError + 'a {
//         move |err|  NodeError::IoError(
//             String::from(source),
//             err.to_string(),
//         )
//     }
// }

pub type NodeResult<T> = Result<T, NodeError>;
