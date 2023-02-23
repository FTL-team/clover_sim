use std::{io, path::PathBuf};

use cloversim_lib::err::NodeError;
use tokio::task::JoinError;

pub struct JoinNodeError(NodeError);

impl From<JoinError> for JoinNodeError {
    fn from(e: JoinError) -> Self {
        if e.is_cancelled() {
            return Self(NodeError::TaskCancelled);
        }
        Self(match e.try_into_panic() {
            Ok(e) => NodeError::from_panic_any(e),
            Err(e) => NodeError::InternalError(e.to_string()),
        })
    }
}

impl From<JoinNodeError> for NodeError {
    fn from(e: JoinNodeError) -> Self {
        e.0
    }
}

pub struct IoNodeError(NodeError);

impl IoNodeError {
    pub fn create_fs_error(path: &PathBuf, err: io::Error) -> NodeError {
        NodeError::IoError(
            String::from(path.to_str().unwrap_or("???")),
            err.to_string(),
        )
    }

    pub fn create_fs_errmap<'a>(path: &'a PathBuf) -> impl Fn(io::Error) -> NodeError + 'a {
        |err| Self::create_fs_error(path, err)
    }

    pub fn create_errmap<'a>(source: &'a str) -> impl Fn(io::Error) -> NodeError + 'a {
        move |err| NodeError::IoError(String::from(source), err.to_string())
    }
}

impl From<IoNodeError> for NodeError {
    fn from(err: IoNodeError) -> Self {
        err.0
    }
}
