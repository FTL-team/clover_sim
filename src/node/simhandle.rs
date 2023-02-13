use std::{path::PathBuf, sync::Mutex};

use tokio::sync::broadcast;

use super::{
    err::{NodeError, NodeResult},
    score::TaskScore,
};

#[derive(Clone, Debug)]
pub enum SimulatorAction {
    Start,
    Stop,
    Restart,
}

#[derive(Clone, Debug)]
pub enum LaunchEvent {
    Poweroff,
    ContainerExited(String),
    ContainerReady(String),
    ContainerStarted(String),
    ContainerFailed(String, NodeError),

    ControlSimulator(SimulatorAction),

    TaskScores(Box<TaskScore>),
    RefreshScores,

    SetRand(String),
    Rand(String),
    RefreshRand,

    Finished,
}

impl LaunchEvent {
    pub fn format_main(&self) -> Option<String> {
        Some(match self {
            LaunchEvent::Poweroff => format!("Exiting"),
            LaunchEvent::ContainerExited(name) => format!("Container {} exited", name),
            LaunchEvent::ContainerReady(name) => format!("Container {} is ready", name),
            LaunchEvent::ContainerStarted(name) => format!("Container {} started", name),
            LaunchEvent::ContainerFailed(name, err) => {
                format!("Container {} failed with err {:#?}", name, err)
            }
            LaunchEvent::ControlSimulator(targ) => format!(
                "{} simulator",
                match targ {
                    SimulatorAction::Start => "Starting",
                    SimulatorAction::Stop => "Stopping",
                    SimulatorAction::Restart => "Restarting",
                }
            ),
            LaunchEvent::Finished => format!("Finished"),

            LaunchEvent::SetRand(rand) => format!("New randomization: {}", rand),
            LaunchEvent::Rand(rand) => format!("Current randomization: {}", rand),

            LaunchEvent::RefreshScores | LaunchEvent::TaskScores(_) => return None,
            LaunchEvent::RefreshRand  => return None,
        })
    }

    pub fn format_source(&self) -> String {
        match self {
            LaunchEvent::Poweroff | LaunchEvent::Finished => String::from("global"),

            LaunchEvent::ContainerReady(c)
            | LaunchEvent::ContainerExited(c)
            | LaunchEvent::ContainerStarted(c)
            | LaunchEvent::ContainerFailed(c, _) => c.to_string(),

            LaunchEvent::ControlSimulator(_) | LaunchEvent::TaskScores(_) => {
                String::from("simulator")
            }
            LaunchEvent::RefreshScores | LaunchEvent::RefreshRand => String::from("internal"),

            LaunchEvent::SetRand(_) | LaunchEvent::Rand(_) => String::from("rand"),
        }
    }
}

pub struct SimulatorHandle {
    event_broadcast: broadcast::Sender<LaunchEvent>,
    task_files: Mutex<Option<PathBuf>>,
}

impl SimulatorHandle {
    pub fn new() -> Self {
        let event_broadcast = broadcast::channel::<LaunchEvent>(32).0;

        Self {
            event_broadcast,
            task_files: Mutex::new(None),
        }
    }

    pub fn create_event_reciever(&self) -> broadcast::Receiver<LaunchEvent> {
        self.event_broadcast.subscribe()
    }

    pub fn broadcast_event(&self, event: LaunchEvent) -> NodeResult<()> {
        self.event_broadcast
            .send(event)
            .map_err(|e| NodeError::InternalError(e.to_string()))?;
        Ok(())
    }

    pub fn set_task_files(&self, new_path: PathBuf) {
        *self.task_files.lock().expect("Failed to lock") = Some(new_path);
    }

    pub fn get_task_file(&self, file: &str) -> Option<String> {
        match self.task_files.lock().expect("Failed to lock").as_ref() {
            Some(path) => Some(path.join(file).to_string_lossy().to_string()),
            None => None,
        }
    }
}
