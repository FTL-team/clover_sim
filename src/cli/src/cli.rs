use std::{
    fs::File,
    future::Future,
    path::{Path, PathBuf},
    sync::Arc,
};

use clap::{Parser, Subcommand};
use cloversim_lib::{err::NodeError, rpc::NodeRpc};
use cloversim_repc::RemoteRpc;
use tokio::{sync::mpsc, task::JoinHandle};

use crate::launch_cli::LaunchCli;

use super::err::CliError;

/// Tool to easily launch clover simulations
#[derive(Debug, Parser)]
#[command(author, version, about, long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,

    #[arg(short, long, help = "Connect to remote coversim instance")]
    remote: Option<String>,
}

#[derive(Debug, Subcommand)]
enum Commands {
    #[command(alias = "ws", about = "Manipulate workspaces")]
    Workspace(WorkspaceCli),

    #[command(about = "Launch simulator")]
    Launch {
        #[arg(short, long, default_value_t = String::from("base_task"), help="Select task to launch")]
        task: String,

        ws: String,
    },

    #[command(about = "Manipulate tasks")]
    Task(TaskCli),

    #[command(about = "Run node for remote connection")]
    Node,
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

async fn execute_command(cli: Cli, rpc: Arc<dyn NodeRpc>) -> Result<(), CliError> {
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
                let target_name = format!("{}_exported.tar.gz", name);
                let path = PathBuf::from(target_name.clone());
                rpc.export_workspace(name.clone(), File::create(path)?)
                    .await?;
                println!("Workspace {} exported to {}", name, target_name);
                Ok(())
            }

            WorkspaceCommands::Import { path } => {
                let path_str = String::from(path.to_str().unwrap_or("???"));
                let name = rpc
                    .import_workspace(match path.exists() {
                        true => File::open(path)?,
                        false => File::create(path)?,
                    })
                    .await?;
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
            let (ctrl_tx, ctrl_rx) = mpsc::channel(16);
            let cli_rpc = rpc.clone();
            let ctrl_task: JoinHandle<Result<(), CliError>> = tokio::spawn(async move {
                let res = LaunchCli::run(cli_rpc, ctrl_rx).await;
                if let Err(e) = &res {
                    eprintln!("Failed to run control cli {:#?}", e);
                }
                res
            });

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
        Commands::Node => {
            cloversim_reps::run(rpc.clone()).await;
            Ok(())
        }
    }
}

pub async fn run(get_local_rpc: fn() -> Result<Arc<dyn NodeRpc>, NodeError>) {
    let parsed = Cli::parse();

    let rpc = match &parsed.remote {
        Some(addr) => RemoteRpc::connect(addr.clone())
            .await
            .map(|rpc| Arc::new(rpc) as Arc<dyn NodeRpc>),
        None => get_local_rpc(),
    };

    let rpc = match rpc {
        Err(err) => {
            eprintln!("{:#?}", err);
            return;
        }
        Ok(rpc) => rpc,
    };

    let res: Result<(), CliError> = execute_command(parsed, rpc).await;

    if let Err(err) = res {
        eprintln!("{:#?}", err);
    }
}
