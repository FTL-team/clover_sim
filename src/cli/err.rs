use rustyline_async::ReadlineError;
use tokio::{sync::{mpsc::error::SendError, broadcast::error::RecvError}, task::JoinError};

use crate::node::err::NodeError;

#[derive(Debug)]
pub enum CliError {
    ReadlineError(String),
    InternalError(String),
    Node(NodeError),
    Io(String),
}

impl From<JoinError> for CliError {
    fn from(e: JoinError) -> Self {
        CliError::InternalError(e.to_string())
    }
}

impl<T> From<SendError<T>> for CliError {
    fn from(e: SendError<T>) -> Self {
        CliError::InternalError(e.to_string())
    }
}

impl From<RecvError> for CliError {
  fn from(e: RecvError) -> Self {
      CliError::InternalError(e.to_string())
  }
}

impl From<ReadlineError> for CliError {
    fn from(e: ReadlineError) -> Self {
        CliError::ReadlineError(e.to_string())
    }
}

impl From<NodeError> for CliError {
  fn from(e: NodeError) -> Self {
      CliError::Node(e)
  }
}

impl From<std::io::Error> for CliError {
  fn from(e: std::io::Error) -> Self {
      CliError::Io(e.to_string())
  }
}
