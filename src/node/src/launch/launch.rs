use std::{
    env,
    fs::Permissions,
    os::unix::prelude::PermissionsExt,
    sync::{
        atomic::{AtomicBool, AtomicUsize, Ordering},
        Arc,
    },
    time::Duration,
};

use async_broadcast::Receiver;
use cloversim_lib::{
    err::{NodeError, NodeResult},
    launch_event::{LaunchEvent, SimulatorAction},
    score::TaskScore,
};
use futures::future;
use rand::random;
use tokio::{select, time::interval};

use crate::{
    clean::Cleaner,
    launch::{
        container::{Container, ContainerOptions},
        ros::ServiceMode,
    },
    local_files::get_local_path,
    simhandle::SimulatorHandle,
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
    map_entry: WorkEntry,

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

        let work_root = WorkRoot::new(
            common_containers_path.join(format!("{:04x}", id)),
            cleaner.clone(),
        )
        .await?;

        let work_root = Arc::new(work_root);
        work_root.as_entry().create_dir().await?;

        let base_entry = BaseOverlayEntry::new(work_root.get("basefs")).await?;
        let net = Network::new(work_root.get("hosts"), id, cleaner.clone()).await?;
        let x11 = X11::new(work_root.as_entry()).await?;

        if !get_local_path("sim/cloversim").exists() {
            return Err(NodeError::InvalidInstallation);
        }

        let score_entry = work_root.get("task_score.json");
        score_entry.create().await?;
        score_entry
            .set_permissions(Permissions::from_mode(0o666))
            .await?;

        let rand_entry = work_root.get("task_rand");
        rand_entry.write("0").await?;

        let map_entry = work_root.get("map.txt");
        map_entry.create().await?;
        map_entry
            .set_permissions(Permissions::from_mode(0o666))
            .await?;

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

        Ok(Launch {
            binds,
            base_entry,
            net,
            x11,

            task_name: task.name,
            score_entry,
            rand_entry,
            map_entry,

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
            if res.is_empty() {
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

    pub fn overlay_entries(&self) -> Vec<&'_ dyn OverlayEntry> {
        vec![&self.base_entry]
    }

    pub fn work_root(&self) -> Arc<WorkRoot> {
        self.work_root.clone()
    }

    pub fn network(&self) -> &'_ Network {
        &self.net
    }

    pub fn x11(&self) -> &'_ X11 {
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

    pub fn get_score_entry(&self) -> &'_ WorkEntry {
        &self.score_entry
    }

    pub fn get_rand_entry(&self) -> &'_ WorkEntry {
        &self.rand_entry
    }

    pub fn get_map_entry(&self) -> &'_ WorkEntry {
        &self.map_entry
    }

    pub fn create_event_reciever(&self) -> Receiver<LaunchEvent> {
        self.handle.create_reciever()
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
        let mut task_update_interval = interval(Duration::from_millis(250));
        let mut scores = String::from("");

        loop {
            select! {
                event = reciever.recv() => {
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
                                    scores = String::from(String::from_utf8_lossy(&self.score_entry.read().await?));
                                    if let Ok(scores) =
                                        serde_json::from_str::<TaskScore>(&scores)
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
                _ = task_update_interval.tick()  => {
                    let new_scores = String::from(String::from_utf8_lossy(&self.score_entry.read().await?));
                    if new_scores == scores {
                        continue
                    }
                    scores = new_scores;
                    if let Ok(scores) = serde_json::from_str::<TaskScore>(&scores)
                    {
                        self.broadcast_event(LaunchEvent::TaskScores(Box::new(scores)))?;
                    }
                }
            }
        }
        Ok(())
    }
}

pub async fn launch(handle: Arc<SimulatorHandle>, ws: Workspace, task: Task) -> NodeResult<()> {
    if !handle.start() {
        return Err(NodeError::AnotherSimmulationRunning);
    }
    handle.set_task_files(task.fullpath.clone());

    let res = Launch::launch(
        handle.clone(),
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
    .await;

    handle.stop();
    res
}
