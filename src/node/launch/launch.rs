use std::{
    env,
    os::unix::prelude::PermissionsExt,
    sync::{
        atomic::{AtomicBool, AtomicUsize, Ordering},
        Arc,
    },
};

use futures::future;
use rand::random;
use tokio::sync::broadcast;

use crate::node::{
    clean::Cleaner,
    err::{NodeError, NodeResult},
    launch::{
        container::{Container, ContainerOptions},
        ros::ServiceMode,
    },
    local_files::get_local_path,
    score::TaskScore,
    simhandle::{LaunchEvent, SimulatorAction, SimulatorHandle},
    task::Task,
    workroot::{WorkEntry, WorkRoot},
    workspace::Workspace,
};

use super::{
    base::BaseBind,
    fs::{BaseOverlayEntry, OverlayEntry},
    network::Network,
    x11::X11,
};

pub struct Launch {
    base_entry: BaseOverlayEntry,
    net: Network,
    x11: X11,
    binds: Vec<BaseBind>,

    task_name: String,
    score_entry: WorkEntry,
    rand_entry: WorkEntry,

    work_root: Arc<WorkRoot>,
    id: u16,
    handle: Arc<SimulatorHandle>,

    total_containers: usize,
    containers_running: AtomicUsize,
    containers_ready: AtomicUsize,
    poweroff_in_progress: AtomicBool,

    _cleaner: Arc<Cleaner>,
}

impl Launch {
    pub async fn create(
        handle: Arc<SimulatorHandle>,
        cleaner: Arc<Cleaner>,
        task: Task,
        total_containers: usize,
    ) -> NodeResult<Launch> {
        let id = random::<u16>();

        let common_containers_path = env::current_dir()
            .expect("Couldn't get current working directory")
            .join("containers");

        if let Err(e) = tokio::fs::create_dir(&common_containers_path).await {
            match e.kind() {
                std::io::ErrorKind::AlreadyExists => (),
                _ => return Err(NodeError::create_fs_error(&common_containers_path, e)),
            }
        }

        let work_root = WorkRoot::new(
            common_containers_path.join(format!("{:04x}", id)),
            cleaner.clone(),
        )
        .await?;
        let work_root = Arc::new(work_root);

        let base_entry = BaseOverlayEntry::new(work_root.get("basefs")).await?;
        let net = Network::new(work_root.get("hosts"), id, cleaner.clone()).await?;
        let x11 = X11::new(work_root.as_entry()).await?;

        if !get_local_path("sim/cloversim").exists() {
            return Err(NodeError::InvalidInstallation);
        }

        let score_entry = work_root.get("task_score.json");
        score_entry.create().await?;

        let rand_entry = work_root.get("task_rand");
        rand_entry.create().await?;

        tokio::fs::set_permissions(score_entry.path(), std::fs::Permissions::from_mode(0o666))
            .await
            .map_err(NodeError::create_fs_errmap(score_entry.path()))?;

        let mut binds = vec![
            BaseBind::RO(net.get_hosts_path(), String::from("/etc/hosts")),
            BaseBind::RO(
                get_local_path("sim/cloversim"),
                String::from("/home/clover/catkin_ws/src/cloversim"),
            ),
            BaseBind::RO(
                task.fullpath.clone(),
                format!("/home/clover/catkin_ws/src/{}", task.name),
            ),
        ];
        binds.append(&mut x11.get_binds());

        handle.set_task_files(task.fullpath);

        Ok(Launch {
            binds,
            base_entry,
            net,
            x11,

            task_name: task.name,
            score_entry,
            rand_entry,

            handle,
            work_root,
            id,

            total_containers,
            containers_running: AtomicUsize::new(0),
            containers_ready: AtomicUsize::new(0),
            poweroff_in_progress: AtomicBool::new(false),

            _cleaner: cleaner,
        })
    }

    pub async fn add_container(&self, options: ContainerOptions) -> NodeResult<()> {
        if self.poweroff_in_progress.load(Ordering::Relaxed) {
            return Ok(());
        }

        let name = options.name.clone();
        self.containers_running.fetch_add(1, Ordering::Relaxed);

        let run_res = Container::run(self, options).await;
        self.containers_running.fetch_sub(1, Ordering::Relaxed);

        if let Err(e) = &run_res {
            self.broadcast_event(LaunchEvent::ContainerFailed(name, e.clone()))?;
        }
        self.poweroff()?;

        run_res
    }

    pub async fn launch(
        handle: Arc<SimulatorHandle>,
        task: Task,
        containers: Vec<ContainerOptions>,
    ) -> NodeResult<()> {
        let cleaner = Arc::new(Cleaner::new());

        let launch = Launch::create(handle, cleaner.clone(), task, containers.len()).await?;

        let containers = future::join_all(containers.into_iter().map(|c| launch.add_container(c)));
        let containers = async {
            let res: Vec<NodeError> = containers
                .await
                .into_iter()
                .filter_map(|x| match x {
                    Ok(_) => None,
                    Err(e) => Some(e),
                })
                .collect();
            launch.broadcast_event(LaunchEvent::Finished)?;
            if res.len() == 0 {
                Ok(())
            } else {
                Err(NodeError::ContainersError(res))
            }
        };

        let (res, _) = tokio::join!(containers, launch.run());
        drop(launch);
        let clean_res = cleaner.wait_clean().await;

        res?;
        clean_res?;

        Ok(())
    }

    pub fn overlay_entries<'a>(&'a self) -> Vec<&'a dyn OverlayEntry> {
        vec![&self.base_entry]
    }

    pub fn work_root(&self) -> Arc<WorkRoot> {
        self.work_root.clone()
    }

    pub fn network<'a>(&'a self) -> &'a Network {
        &self.net
    }

    pub fn x11<'a>(&'a self) -> &'a X11 {
        &self.x11
    }

    pub fn get_id(&self) -> u16 {
        self.id
    }

    pub fn bind_entries(&self) -> Vec<BaseBind> {
        self.binds.clone()
    }

    pub fn get_task_name(&self) -> &str {
        self.task_name.as_str()
    }

    pub fn get_score_entry<'a>(&'a self) -> &'a WorkEntry {
        &self.score_entry
    }

    pub fn create_event_reciever(&self) -> broadcast::Receiver<LaunchEvent> {
        self.handle.create_event_reciever()
    }

    pub fn broadcast_event(&self, event: LaunchEvent) -> NodeResult<()> {
        self.handle.broadcast_event(event)
    }

    pub fn poweroff(&self) -> NodeResult<()> {
        if !self.poweroff_in_progress.fetch_or(true, Ordering::Relaxed) {
            self.broadcast_event(LaunchEvent::Poweroff)?;
        }
        Ok(())
    }

    pub async fn run(&self) -> NodeResult<()> {
        let mut reciever = self.create_event_reciever();

        loop {
            let event = reciever.recv().await;
            match event {
                Ok(event) => {
                    // println!("Event={:#?}", event);
                    match event {
                        LaunchEvent::ContainerExited(_) => {
                            self.poweroff()?;
                        }
                        LaunchEvent::ContainerFailed(_, _) => {
                            self.poweroff()?;
                        }
                        LaunchEvent::ContainerReady(_) => {
                            let cur_ready = self
                                .containers_ready
                                .fetch_add(1, std::sync::atomic::Ordering::AcqRel)
                                + 1;

                            if cur_ready == self.total_containers {
                                // println!("Everyone is ready");
                                self.broadcast_event(LaunchEvent::ControlSimulator(
                                    SimulatorAction::Start,
                                ))?;
                            }
                        }
                        LaunchEvent::Finished => break,
                        LaunchEvent::Poweroff => {
                            self.poweroff_in_progress.store(true, Ordering::Relaxed)
                        }
                        LaunchEvent::RefreshScores => {
                            let scores = self.score_entry.read().await?;
                            if let Ok(scores) =
                                serde_json::from_str::<TaskScore>(&String::from_utf8_lossy(&scores))
                            {
                                self.broadcast_event(LaunchEvent::TaskScores(Box::new(scores)))?;
                            }
                        }
                        LaunchEvent::RefreshRand => {
                            let rand = self.rand_entry.read().await?;
                            let rand = String::from_utf8_lossy(&rand);
                            self.broadcast_event(LaunchEvent::Rand(rand.to_string()))?;
                        }
                        LaunchEvent::SetRand(newrand) => {
                            self.rand_entry.write(newrand).await?;
                        }
                        _ => {}
                    };
                }
                Err(e) => return Err(NodeError::InternalError(e.to_string())),
            }
        }
        Ok(())
    }
}

pub async fn launch(handle: Arc<SimulatorHandle>, ws: Workspace, task: Task) -> NodeResult<()> {
    Launch::launch(
        handle,
        task,
        vec![
            ContainerOptions {
                name: String::from("cloversim"),
                prefered_ip: Some(2),
                mode: ServiceMode::Simulator,
                workspace: None,
            },
            ContainerOptions {
                name: String::from("clover0"),
                prefered_ip: None,
                mode: ServiceMode::Copter,
                workspace: Some(ws),
            },
        ],
    )
    .await
}
