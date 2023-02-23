use crate::errhelper::JoinNodeError;
use cloversim_lib::err::NodeResult;
use std::sync::{Arc, Mutex};
use tokio::task::JoinHandle;

pub struct Cleaner {
    to_wait_on: Mutex<Vec<JoinHandle<NodeResult<()>>>>,
}

impl Cleaner {
    pub fn new() -> Self {
        Self {
            to_wait_on: Mutex::new(Vec::new()),
        }
    }

    pub fn add_task(self: Arc<Self>, task: JoinHandle<NodeResult<()>>) {
        self.to_wait_on
            .lock()
            .expect("Cleanup lock is dead")
            .push(task)
    }

    pub async fn wait_clean(self: Arc<Self>) -> NodeResult<()> {
        loop {
            let task = self.to_wait_on.lock().expect("Cleanup lock is dead").pop();
            if let Some(task) = task {
                task.await.map_err(|x| JoinNodeError::from(x))??;
            } else {
                break;
            }
        }

        Ok(())
    }
}
