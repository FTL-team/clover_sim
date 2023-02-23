use cloversim_lib::err::{NodeError, NodeResult};
use std::{
    env, fs::Permissions, os::unix::prelude::PermissionsExt, path::PathBuf, process::Stdio,
    time::Duration,
};
use tokio::{
    process::{Child, Command},
    time::timeout,
};

use crate::{cmd::simple_cmd, local_files::get_local_path, workroot::WorkEntry};

use super::base::BaseBind;

pub struct X11 {
    virgl_server: Child,
    xauth_info: Vec<u8>,
    binds: Vec<BaseBind>,
}

impl X11 {
    pub async fn new(work_root: WorkEntry) -> NodeResult<Self> {
        let virgl_entry = work_root.get("virgl_sock");

        let virgl_root_path = get_local_path("virgl");

        let errlog = work_root.get("virgl_errlog");
        let errlog_file = errlog.create().await?;
        let errlog_file = errlog_file.into_std().await;

        let mut virgl_cmd = Command::new(virgl_root_path.join("virgl_test_server"));
        virgl_cmd.arg("--use-egl-surfaceless").arg(format!(
            "--socket-path={}",
            virgl_entry.path().to_string_lossy()
        ));

        virgl_cmd.env("LD_LIBRARY_PATH", virgl_root_path);
        virgl_cmd
            .stdout(Stdio::null())
            .stdin(Stdio::null())
            .stderr(errlog_file);

        let mut virgl_server = virgl_cmd
            .spawn()
            .map_err(|x| NodeError::VirglError(x.to_string()))?;

        if let Ok(_) = timeout(Duration::from_millis(200), virgl_server.wait()).await {
            return Err(NodeError::VirglError(
                String::from_utf8_lossy(&errlog.read().await?).to_string(),
            ));
        }

        let display_env = match env::var("DISPLAY") {
            Ok(v) => v,
            Err(err) => return Err(NodeError::NoX11Display(err.to_string())),
        };
        let display_id = display_env
            .strip_prefix(":")
            .ok_or(NodeError::NoX11Display(String::from(
                "Only local displays (strating with :) are supported",
            )))?;

        let xauth_info = simple_cmd!("<extract xauth cookie>"; xauth list (display_env)).await?;

        let mut x11_path = PathBuf::from("/tmp/.X11-unix/");
        x11_path.push(String::from("X") + display_id);

        let binds = vec![
            BaseBind::RO(x11_path, String::from("/tmp/.X11-unix/X0")),
            BaseBind::RO(virgl_entry.path().clone(), String::from("/tmp/.virgl_test")),
        ];

        Ok(Self {
            xauth_info,
            virgl_server,
            binds,
        })
    }

    pub fn get_binds(&self) -> Vec<BaseBind> {
        self.binds.clone()
    }

    pub async fn create_container_xauth(
        &self,
        hostname: &str,
        entry: &WorkEntry,
    ) -> NodeResult<WorkEntry> {
        let xauthority_entry = entry.get("Xauthority");
        let xauth_cmd_entry = entry.get("xauth_src");

        let xauth_commands: Vec<u8> = self
            .xauth_info
            .split(|x| *x == 0xa)
            .filter_map(|entry| {
                if entry.is_empty() {
                    return None;
                }
                let split_pos = entry.iter().position(|x| *x == 0x20);
                let split_pos = match split_pos {
                    None => {
                        println!("Failed loading x11 auth entry {:#?}", entry);
                        return None;
                    }
                    Some(pos) => pos,
                };

                let entry = entry.split_at(split_pos).1;
                let mut res_cmd: Vec<u8> = Vec::new();
                res_cmd.extend_from_slice("add ".as_bytes());
                res_cmd.extend_from_slice(hostname.as_bytes());
                res_cmd.extend_from_slice("/unix:0".as_bytes());
                res_cmd.extend_from_slice(entry);
                res_cmd.push(0xa);

                Some(res_cmd)
            })
            .flatten()
            .collect();

        xauthority_entry.create().await?;
        xauth_cmd_entry.write(xauth_commands).await?;

        simple_cmd!( "<create xauthority>"; xauth -f (xauthority_entry.path()) source (xauth_cmd_entry.path())).await?;
        xauthority_entry
            .set_permissions(Permissions::from_mode(0o644))
            .await?;

        Ok(xauthority_entry)
    }
}

impl Drop for X11 {
    fn drop(&mut self) {
        self.virgl_server.start_kill();
    }
}
