use std::{
    path::PathBuf,
    sync::{
        atomic::{AtomicBool, Ordering},
        Mutex,
    },
};

use async_broadcast::{broadcast, InactiveReceiver, Receiver, Sender};
use cloversim_lib::{
    err::{NodeError, NodeResult},
    launch_event::LaunchEvent,
};

pub struct SimulatorHandle {
    event_broadcast: Sender<LaunchEvent>,
    _rx_keeper: InactiveReceiver<LaunchEvent>, // prevents channel from being closed
    task_files: Mutex<Option<PathBuf>>,
    running: AtomicBool,
}

impl SimulatorHandle {
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        let (mut event_broadcast, core_recv) = broadcast(32);
        // dbg!("Created handle");
        event_broadcast.set_overflow(true);
        Self {
            event_broadcast,
            _rx_keeper: core_recv.deactivate(),
            task_files: Mutex::new(None),
            running: AtomicBool::new(false),
        }
    }

    pub fn create_new_sender(&self) -> Sender<LaunchEvent> {
        self.event_broadcast.clone()
    }

    pub fn create_reciever(&self) -> Receiver<LaunchEvent> {
        self.event_broadcast.new_receiver()
    }

    pub fn broadcast_event(&self, event: LaunchEvent) -> NodeResult<()> {
        self.event_broadcast
            .try_broadcast(event)
            .map_err(|e| NodeError::InternalError(e.to_string()))?;
        Ok(())
    }

    pub fn set_task_files(&self, new_path: PathBuf) {
        *self.task_files.lock().expect("Failed to lock") = Some(new_path);
    }

    pub fn get_task_file(&self, file: &str) -> NodeResult<String> {
        let task_lock = self.task_files.lock().expect("Failed to lock");
        let task_path = task_lock.as_ref().ok_or(NodeError::NoSimulatorRunning)?;
        let path = task_path.join(file);
        if !path.starts_with(task_path) {
            return Err(NodeError::InvalidTaskFile);
        }
        Ok(path.to_string_lossy().to_string())
    }

    pub fn start(&self) -> bool {
        !self.running.fetch_or(true, Ordering::Relaxed)
    }

    pub fn stop(&self) -> bool {
        self.running.fetch_and(false, Ordering::Relaxed)
    }

    pub fn get_status(&self) -> bool {
        self.running.load(Ordering::Relaxed)
    }
}
