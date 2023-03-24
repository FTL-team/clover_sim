use serde::{Deserialize, Serialize};
use ts_rs::TS;

use crate::{err::NodeError, score::TaskScore};

#[derive(Clone, Debug, Serialize, Deserialize, TS)]
#[ts(export)]
pub enum SimulatorAction {
    Start,
    Stop,
    Restart,
}

#[derive(Clone, Debug, Serialize, Deserialize, TS)]
#[ts(export)]
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
            LaunchEvent::Poweroff => String::from("Exiting"),
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
            LaunchEvent::Finished => String::from("Finished"),

            LaunchEvent::SetRand(rand) => format!("New randomization: {}", rand),
            LaunchEvent::Rand(rand) => format!("Current randomization: {}", rand),

            LaunchEvent::RefreshScores | LaunchEvent::TaskScores(_) => return None,
            LaunchEvent::RefreshRand => return None,
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
