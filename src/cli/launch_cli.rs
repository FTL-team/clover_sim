use super::err::CliError;
use crate::node::{
    score::TaskScore,
    simhandle::{LaunchEvent, SimulatorAction, SimulatorHandle},
};
use crossterm::style::{Color, Stylize};
use rustyline_async::{Readline, ReadlineError};
use std::{io::Write, sync::Arc};
use tokio::{select, sync::mpsc::Receiver};

const SOURCE_COLORS: [Color; 7] = [
    Color::DarkGreen,
    Color::Magenta,
    Color::Yellow,
    Color::Cyan,
    Color::DarkYellow,
    Color::DarkBlue,
    Color::Green,
];

pub struct LaunchCli {}

const HELP: &str = "Available commands:
help     - show this help
exit     - shutdowns cloversim
start    - start simulator (gazebo, clover, etc.)
stop     - stop simulator (gazebo, clover, etc.)
restart  - restart the simulator (gazebo, clover, etc.)
rget     - get current randomzation
rset     - set randomization to value from second argument
rnew     - generate new random randomzation
score    - show current task scoring
tpath    - get path of file in task (README.md by default)
tread    - read file in task (README.md by default)";

impl LaunchCli {
    pub async fn run(
        handle: Arc<SimulatorHandle>,
        mut ctrl_rx: Receiver<bool>,
    ) -> Result<(), CliError> {
        let (mut readline, mut stdout) = Readline::new(String::from("> "))?;
        let mut event_rx = handle.create_event_reciever();
        let mut read_more = true;
        let mut should_print_scores = false;
        let mut should_print_rand = false;

        loop {
            select! {
                line = readline.readline(), if read_more => {
                    match line {
                        Ok(line) => {
                            let line = line.trim();
                            writeln!(stdout, "> {}", line)?;
                            readline.add_history_entry(line.to_owned());
                            let parts: Vec<&str> = line.split(' ').collect();
                            let event = Self::map_cmd_to_event(&parts);

                            match parts[0] {
                                "help" => {
                                    writeln!(stdout, "{}", HELP)?;
                                }

                                "tpath" => {
                                    let file = *parts.get(1).unwrap_or(&"README.md");
                                    if let Some(file) = handle.get_task_file(file) {
                                        writeln!(stdout, "Path to task file: {}", file)?;
                                    } else {
                                        writeln!(stdout, "No task found")?;
                                    }
                                }

                                "tread" => {
                                    let file = *parts.get(1).unwrap_or(&"README.md");
                                    if let Some(file) = handle.get_task_file(file) {
                                        match tokio::fs::read(&file).await {
                                            Ok(contents) => {
                                                let contents = String::from_utf8_lossy(&contents);
                                                if file.ends_with(".md") {
                                                    writeln!(stdout, "{}", termimad::text(&contents))?;
                                                } else {
                                                    writeln!(stdout, "{}", contents)?;
                                                }
                                            },
                                            Err(err) =>  writeln!(stdout, "Failed to read: {}", err)?,
                                        };
                                    } else {
                                        writeln!(stdout, "No task found")?;
                                    }
                                }

                                "score" => {
                                    should_print_scores = true;
                                    handle.broadcast_event(LaunchEvent::RefreshScores)?;
                                }

                                "rget" => {
                                    should_print_rand = true;
                                    handle.broadcast_event(LaunchEvent::RefreshRand)?;
                                }

                                _ => {
                                    if let Some(event) = event {
                                        handle.broadcast_event(event)?;
                                    } else {
                                        writeln!(stdout, "Unknown command: {}", parts[0])?;
                                    }
                                }
                            };
                        }
                        Err(ReadlineError::Interrupted) => {
                            writeln!(stdout, "")?;
                            read_more = false;
                            handle.broadcast_event(LaunchEvent::Poweroff)?;
                        }
                        Err(ReadlineError::Eof) => read_more = false,
                        Err(ReadlineError::Closed) => read_more = false,
                        Err(err) => {
                            dbg!("Error during readline", err);
                        }
                    }
                }


                event = event_rx.recv() => {
                    let event = event?;
                    if let LaunchEvent::Rand(_) = &event {
                        if !should_print_rand {
                            continue
                        }
                    }
                    
                    if let LaunchEvent::TaskScores(score) = &event {
                        if should_print_scores {
                            Self::print_score(&mut stdout, score)?;
                        }
                    }


                    if let LaunchEvent::Poweroff = event {
                        read_more = false;
                    }

                    if let Some(event_fmt) = event.format_main() {
                        let source = event.format_source();
                        let source_hash: usize = source.chars().map(|c| c as usize).sum();
                        let source = format!(" {:<10} ", source).with(SOURCE_COLORS[source_hash % 7]);
                        writeln!(stdout, "[{}] {}", source, event_fmt)?;
                    }

                    if !read_more {
                        readline.flush()?;
                    }
                }

                _ = ctrl_rx.recv() => {
                    writeln!(stdout, "")?;
                    readline.flush()?;
                    break;
                }
            }
        }

        Ok(())
    }

    fn map_cmd_to_event(parts: &Vec<&str>) -> Option<LaunchEvent> {
        Some(match parts[0] {
            "exit" => LaunchEvent::Poweroff,

            "start" => LaunchEvent::ControlSimulator(SimulatorAction::Start),
            "stop" => LaunchEvent::ControlSimulator(SimulatorAction::Stop),
            "restart" => LaunchEvent::ControlSimulator(SimulatorAction::Restart),

            "rset" => LaunchEvent::SetRand(parts[1..].join(" ")),
            "rnew" => LaunchEvent::SetRand(rand::random::<u32>().to_string()),

            _ => {
                return None;
            }
        })
    }

    fn print_score(writer: &mut dyn Write, score: &TaskScore) -> std::io::Result<()> {
        let flat = score.create_flat_repr(0);

        let scores_str = flat
            .iter()
            .map(|(_, task)| {
                (
                    format!("{:.2}", task.score),
                    format!("{:.2}", task.max_score),
                )
            })
            .collect::<Vec<_>>();

        let name_size = flat
            .iter()
            .map(|(level, task)| level * 2 + task.name.len())
            .max()
            .unwrap_or(0);
        let score_size = scores_str
            .iter()
            .map(|(score, _)| score.len())
            .max()
            .unwrap_or(0);
        let maxscore_size = scores_str
            .iter()
            .map(|(_, max_score)| max_score.len())
            .max()
            .unwrap_or(0);

        for ((level, task), (score, max_score)) in flat.iter().zip(scores_str.iter()) {
            let res = if task.failed {
                String::from("failed").red()
            } else {
                let color = if task.score == 0.0 {
                    Color::AnsiValue(248)
                } else if task.score == task.max_score {
                    Color::Green
                } else {
                    Color::White
                };

                format!(
                    "{:>scol$} / {:>mscol$}",
                    score,
                    max_score,
                    scol = score_size,
                    mscol = maxscore_size
                )
                .with(color)
            };
            writeln!(
                writer,
                "{:<lcol$}{:<ncols$}  {}",
                "",
                task.name,
                res,
                lcol = level * 2,
                ncols = name_size - level * 2
            )?;
        }

        Ok(())
    }
}
