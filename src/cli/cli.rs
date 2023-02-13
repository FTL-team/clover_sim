use std::path::PathBuf;

use crate::node::{err::NodeError, rpc};
use clap::{Parser, Subcommand};
use tokio::{sync::mpsc, task::JoinHandle};

use super::{err::CliError, launch_cli::LaunchCli};

/// Tool to easily launch clover simulations
#[derive(Debug, Parser)]
#[command(author, version, about, long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Debug, Subcommand)]
enum Commands {
    #[command(alias = "ws", about = "Manipulate workspaces")]
    Workspace(WorkspaceCli),

    #[command(about = "Launch simulator")]
    Launch {
        #[arg(short, long, default_value_t = String::from("base_task"))]
        task: String,

        ws: String,
    },

    #[command(about = "Manipulate tasks")]
    Task(TaskCli),
}

#[derive(Debug, Parser)]
struct WorkspaceCli {
    #[command(subcommand)]
    command: WorkspaceCommands,
}

#[derive(Debug, Subcommand)]
enum WorkspaceCommands {
    #[command(about = "Create new workspace")]
    Create { name: String },
    #[command(about = "Remove workspace")]
    Remove { name: String },

    #[command(about = "List workspaces")]
    List,

    #[command(about = "Export workspace to tar")]
    Export { name: String },
    #[command(about = "Import workspace from tar")]
    Import { path: PathBuf },

    #[command(about = "Duplicate workspace")]
    Duplicate { name: String, newname: String },

    #[command(about = "Clean workspace from unused files")]
    Clean { name: String },
}

#[derive(Debug, Parser)]
struct TaskCli {
    #[command(subcommand)]
    command: TaskCommands,
}

#[derive(Debug, Subcommand)]
enum TaskCommands {
    #[command(about = "List tasks")]
    List,
}

async fn execute_command(cli: Cli, rpc: &dyn rpc::NodeRpc) -> Result<(), CliError> {
    match cli.command {
        Commands::Workspace(cmd) => match cmd.command {
            WorkspaceCommands::Create { name } => {
                rpc.create_workspace(name.clone()).await?;
                println!("Workspace {} created successfully", name);
                Ok(())
            }
            WorkspaceCommands::Remove { name } => {
                rpc.remove_workspace(name.clone()).await?;
                println!("Workspace {} removed successfully", name);
                Ok(())
            }

            WorkspaceCommands::List => {
                for ws in rpc.list_workspaces().await?.into_iter() {
                    println!("{}", ws);
                }
                Ok(())
            }

            WorkspaceCommands::Export { name } => {
                let exported_path = rpc.export_workspace(name.clone()).await?;
                let target_name = format!("{}_exported.tar.gz", name);
                tokio::fs::rename(&exported_path, &target_name)
                    .await
                    .map_err(NodeError::create_fs_errmap(&PathBuf::from(&exported_path)))?;
                println!("Workspace {} exported to {}", name, target_name);
                Ok(())
            }

            WorkspaceCommands::Import { path } => {
                let path_str = String::from(path.to_str().unwrap_or("???"));
                let name = rpc.import_workspace(path).await?;
                println!("Imported from {} workspace {}", path_str, name);
                Ok(())
            }

            WorkspaceCommands::Duplicate { name, newname } => {
                rpc.duplicate_workspace(name.clone(), newname.clone())
                    .await?;
                println!("Workspace {} duplicated into {}", name, newname);
                Ok(())
            }

            WorkspaceCommands::Clean { name } => {
                rpc.clean_workspace(name.clone()).await?;
                println!("Workspace {} cleaned", name);
                Ok(())
            }
        },
        Commands::Launch { task, ws } => {
            let sim_handle = rpc.get_sim_handle();
            let (ctrl_tx, ctrl_rx) = mpsc::channel(16);
            let ctrl_task: JoinHandle<Result<(), CliError>> =
                tokio::spawn(LaunchCli::run(sim_handle, ctrl_rx));

            let res = rpc.launch(ws, task).await;
            ctrl_tx.send(true).await?;
            ctrl_task.await??;
            res.map_err(|x| x.into())
        }
        Commands::Task(cmd) => match cmd.command {
            TaskCommands::List => {
                let tasks = rpc.list_tasks().await?;
                let relpath_width = tasks.iter().map(|x| x.relpath.len()).max().unwrap_or(0);

                for task in tasks.into_iter() {
                    println!(
                        "{:width$} : {}",
                        task.relpath,
                        task.description,
                        width = relpath_width
                    );
                }
                Ok(())
            }
        },
    }
}

pub async fn run(rpc: &dyn rpc::NodeRpc) {
    let parsed = Cli::parse();
    let res: Result<(), CliError> = execute_command(parsed, rpc).await;

    if let Err(err) = res {
        eprintln!("{:#?}", err);
    }
}
