use crate::node::{
    cmd::execute_cmd,
    err::{NodeError, NodeResult},
    launch::{ros::create_simulator_service, sdnotif::SdNotifyMessage},
    simhandle::{LaunchEvent, SimulatorAction},
    workspace::Workspace,
};

use super::{
    base::{BaseBind, BaseContainer, BaseContainerOptions},
    fs::{TmpfsOverlayEntry, UpperOverlayEntry, WorkspaceOverlayEntry},
    launch::Launch,
    network::ContainerNetwork,
    ros::ServiceMode,
    sdnotif::SdNotifyListener,
};

pub struct ContainerOptions {
    pub name: String,
    pub prefered_ip: Option<u8>,
    pub mode: ServiceMode,
    pub workspace: Option<Workspace>,
}

pub struct Container<'a> {
    _launch: &'a Launch,
    name: String,
    _net: ContainerNetwork,
    base: BaseContainer,
}

impl<'a> Container<'a> {
    pub async fn run(launch: &'a Launch, options: ContainerOptions) -> NodeResult<()> {
        let mut event_reciever = launch.create_event_reciever();

        let entry = launch.work_root().get(options.name.as_str());
        tokio::fs::create_dir(entry.path())
            .await
            .map_err(NodeError::create_fs_errmap(entry.path()))?;

        let top_entry: Box<dyn UpperOverlayEntry> = match options.workspace {
            Some(workspace) => Box::new(WorkspaceOverlayEntry::new(workspace).await?),
            None => Box::new(TmpfsOverlayEntry::new(entry.get("overlay_tmp")).await?),
        };
        let net =
            ContainerNetwork::new(launch.network(), options.name.as_str(), options.prefered_ip)
                .await?;

        let hostname_entry = entry.get("hostname");
        tokio::fs::write(hostname_entry.path(), &options.name)
            .await
            .map_err(NodeError::create_fs_errmap(hostname_entry.path()))?;

        let xauthority_entry = launch
            .x11()
            .create_container_xauth(options.name.as_str(), &entry)
            .await?;

        let (simservice_entry, buildservice_entry) =
            create_simulator_service(&entry, &options.mode, launch.get_task_name()).await?;

        let mut sd_notify = SdNotifyListener::new(entry.get("sd_notify"))?;

        let mut bind_entries = launch.bind_entries();
        bind_entries.push(BaseBind::RO(
            hostname_entry.path().clone(),
            String::from("/etc/hostname"),
        ));
        bind_entries.push(BaseBind::RO(
            xauthority_entry.path().clone(),
            String::from("/home/clover/.Xauthority"),
        ));
        bind_entries.push(BaseBind::RO(
            simservice_entry.path().clone(),
            String::from("/etc/systemd/system/cloversim.service"),
        ));
        bind_entries.push(BaseBind::RO(
            buildservice_entry.path().clone(),
            String::from("/etc/systemd/system/build_cloversim.service"),
        ));
        bind_entries.push(BaseBind::RW(
            sd_notify.get_path().clone(),
            String::from("/run/host/notify"),
        ));
        if matches!(options.mode, ServiceMode::Simulator) {
            bind_entries.push(BaseBind::RW(
                launch.get_score_entry().path().clone(),
                String::from("/home/clover/task_score.json"),
            ));
            bind_entries.push(BaseBind::RW(
                launch.get_rand_entry().path().clone(),
                String::from("/home/clover/task_randomization"),
            ));
        }

        let base = BaseContainer::start(BaseContainerOptions {
            layers: launch.overlay_entries(),
            upper: top_entry.as_ref(),
            name: options.name.clone(),
            root: entry,
            net: Some(net.get_ns_path()),
            target: vec![String::from("/sbin/init")],
            binds: bind_entries,
            id: launch.get_id(),
        })
        .await?;

        let mut container = Container {
            base,
            _net: net,
            name: options.name,
            _launch: launch,
        };
        launch.broadcast_event(LaunchEvent::ContainerStarted(container.name.clone()))?;

        loop {
            tokio::select! {
                event = event_reciever.recv() => {
                    let event = event.map_err(|x| NodeError::InternalError(x.to_string()))?;
                    let ev_res = container.handle_event(event).await;
                    if let Err(e) = ev_res {
                        launch.broadcast_event(LaunchEvent::ContainerFailed(container.name.clone(), e))?;
                    }
                },
                res = container.wait() => {
                    launch.broadcast_event(LaunchEvent::ContainerExited(container.name.clone()))?;
                    return res;
                }
                notify_event = sd_notify.recv() => {
                    if let Ok(notify_event) = notify_event {
                        match notify_event {
                            SdNotifyMessage::Ready => launch.broadcast_event(LaunchEvent::ContainerReady(container.name.clone()))?,
                        };
                    }
                }
            }
        }
    }

    pub async fn wait(&mut self) -> NodeResult<()> {
        self.base.wait().await?;
        Ok(())
    }

    pub async fn poweroff(&self) -> NodeResult<()> {
        self.base.poweroff().await
    }

    pub async fn handle_event(&self, event: LaunchEvent) -> NodeResult<()> {
        match event {
            LaunchEvent::Poweroff => self.poweroff().await,

            LaunchEvent::ControlSimulator(action) => {
                let (mut cmd, st) = self.base.tokio_exec();
                cmd.arg(st)
                    .arg("systemctl")
                    .arg(match action {
                        SimulatorAction::Start => "start",
                        SimulatorAction::Stop => "stop",
                        SimulatorAction::Restart => "restart",
                    })
                    .arg("cloversim.service");
                execute_cmd(cmd, "<control simulator>").await?;
                Ok(())
            }

            _ => Ok(()),
        }
    }
}
