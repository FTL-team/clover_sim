use crate::node::{err::NodeResult, workroot::WorkEntry};

pub enum ServiceMode {
    Copter,
    Simulator,
}

impl ServiceMode {
    fn launch_name(&self) -> &'static str {
        match self {
            &ServiceMode::Copter => "copter.launch",
            &ServiceMode::Simulator => "simulator.launch",
        }
    }
}

fn create_ros_service(options: impl AsRef<str>, command: impl AsRef<str>) -> String {
    format!(
        "[Unit]
{}
    
[Service]
EnvironmentFile=/etc/environment
RemainAfterExit=yes
User=1000
Group=1000
ExecStart=\"/bin/bash\" \"-ic\" \". /etc/profile; . ~/.bashrc; {}\"
    ",
        options.as_ref(),
        command.as_ref()
    )
}

pub async fn create_simulator_service(
    entry: &WorkEntry,
    mode: &ServiceMode,
    task_name: &str,
) -> NodeResult<(WorkEntry, WorkEntry)> {
    let simulator_service = create_ros_service(
        "Description=Start simulator
After=roscore.target
Wants=roscore.target
After=build_cloversim.service
Wants=build_cloversim.service",
        format!(
            "mkfifo /tmp/cloversim_inp; cat /tmp/cloversim_inp | roslaunch --wait cloversim {}",
            mode.launch_name()
        ),
    );
    let simulator_target = entry.get("cloversim.service");
    simulator_target.write(simulator_service).await?;

    let build_service = create_ros_service("Description=Build cloversim and task

[Service]
Type=oneshot",
format!("cd ~/catkin_ws; mkdir build_cloversim; catkin_make -DCATKIN_WHITELIST_PACKAGES=\\\"{};cloversim\\\" --build build_cloversim", task_name));
    let build_target = entry.get("build_cloversim.service");
    build_target.write(build_service).await?;

    Ok((simulator_target, build_target))
}
