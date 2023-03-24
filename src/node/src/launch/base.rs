use std::{
    path::{PathBuf, Path},
    process::{ExitStatus, Stdio},
    sync::atomic::{self, AtomicBool},
    time::Duration,
};

use cloversim_lib::err::{NodeError, NodeResult};
use oci_spec::{
    runtime::{self, LinuxNamespaceType},
    OciSpecError,
};
use sys_mount::{Mount, UnmountDrop, UnmountFlags};
use tokio::{
    process::{Child, Command},
    time::timeout,
};

use super::fs::{OverlayEntry, UpperOverlayEntry};
use crate::{
    cmd::simple_cmd,
    errhelper::{IoNodeError, JoinNodeError},
    workroot::WorkEntry,
};

#[derive(Clone)]
pub enum BaseBind {
    RW(PathBuf, String),
    RO(PathBuf, String),
}

pub struct BaseContainerOptions<'a> {
    pub layers: Vec<&'a dyn OverlayEntry>,
    pub upper: &'a dyn UpperOverlayEntry,
    pub name: String,
    pub root: WorkEntry,
    pub net: Option<PathBuf>,
    pub target: Vec<String>,
    pub binds: Vec<BaseBind>,
    pub id: u16,
}

pub struct BaseContainer {
    name: String,
    main_child: Child,
    _mount: UnmountDrop<Mount>,
    running: AtomicBool,
}

const BASE_RUNC_SERIALIZED: &str = include_str!("./base_runc.json");

fn match_oci_spec_error(e: oci_spec::OciSpecError, path: &Path) -> NodeError {
    match e {
        oci_spec::OciSpecError::Other(err) => NodeError::InternalError(err),
        oci_spec::OciSpecError::Io(err) => IoNodeError::create_fs_error(path, err),
        oci_spec::OciSpecError::SerDe(err) => {
            NodeError::IoError(String::from("<oci serialization>"), err.to_string())
        }
        oci_spec::OciSpecError::Builder(err) => NodeError::InternalError(err.to_string()),
    }
}

fn create_oci_spec_error_mapper(
    path: &Path,
) -> impl Fn(oci_spec::OciSpecError) -> NodeError + '_ {
    |err| match_oci_spec_error(err, path)
}

impl BaseContainer {
    pub async fn start(options: BaseContainerOptions<'_>) -> NodeResult<BaseContainer> {
        let name = format!("{}_{:04x}", &options.name, options.id);

        let fs_entry = options.root.get("fs");
        fs_entry.create_dir_uniq().await?;

        let layers_combined = options
            .layers
            .iter()
            .map(|x| String::from(x.get_path().to_str().expect("Non utf-8 path")))
            .collect::<Vec<String>>()
            .join(":");

        let mount_data = format!(
            "lowerdir={},upperdir={},workdir={},index=off,metacopy=off",
            layers_combined,
            options.upper.get_path().to_str().expect("Non utf-8 path"),
            options.upper.get_work().to_str().expect("Non utf-8 path")
        );

        let mount = Mount::builder()
            .fstype("overlay")
            .data(mount_data.as_str())
            .mount_autodrop("overlay", fs_entry.path(), UnmountFlags::DETACH)
            .map_err(IoNodeError::create_errmap("mount<container overlay>"))?;

        let spec_entry = options.root.get("config.json");
        let spec_path = spec_entry.path().clone();

        let oci_err_map = create_oci_spec_error_mapper(spec_entry.path());
        let spec = match serde_json::from_str::<runtime::Spec>(BASE_RUNC_SERIALIZED) {
            Ok(spec) => spec,
            Err(e) => {
                return Err(NodeError::FailedToParseFile(
                    String::from("<default config>"),
                    e.to_string(),
                ))
            }
        };

        let spec = Self::modify_spec(spec, &options, fs_entry.path()).map_err(&oci_err_map)?;

        tokio::task::spawn_blocking(move || spec.save(spec_path))
            .await
            .map_err(JoinNodeError::from)?
            .map_err(oci_err_map)?;

        let errlog_entry = fs_entry.get("runc_errlog");
        let errlog_file = errlog_entry.create().await?;
        let errlog_file = errlog_file.into_std().await;

        let mut cmd = Command::new("runc");
        cmd.arg("run").arg(&name);
        cmd.current_dir(options.root.path());

        cmd.stderr(errlog_file);
        cmd.stdout(Stdio::null());
        cmd.stdin(Stdio::null());

        let mut child = cmd
            .spawn()
            .map_err(IoNodeError::create_errmap("<container launch>"))?;

        if timeout(Duration::from_millis(500), child.wait()).await.is_ok() {
            return Err(NodeError::ContainerCreateFailed(
                String::from_utf8_lossy(&errlog_entry.read().await?).to_string(),
            ));
        }

        Ok(BaseContainer {
            name,
            main_child: child,
            _mount: mount,
            running: AtomicBool::new(true),
        })
    }

    pub fn modify_spec(
        mut spec: runtime::Spec,
        options: &BaseContainerOptions<'_>,
        fs_path: &PathBuf,
    ) -> Result<runtime::Spec, OciSpecError> {
        spec.set_root(Some(
            runtime::RootBuilder::default()
                .path(fs_path)
                .readonly(false)
                .build()?,
        ));

        let mut linux = (spec.linux().to_owned()).unwrap_or(runtime::Linux::default());
        if let Some(netns) = &options.net {
            let netns = runtime::LinuxNamespaceBuilder::default()
                .typ(LinuxNamespaceType::Network)
                .path(netns)
                .build()?;
            let mut namespaces = linux.namespaces().to_owned().unwrap_or(vec![]);
            namespaces.push(netns);
            linux.set_namespaces(Some(namespaces));
        }

        spec.set_linux(Some(linux));

        let mut proc = spec
            .process()
            .to_owned()
            .unwrap_or(runtime::Process::default());

        proc.set_args(Some(options.target.clone()));
        spec.set_process(Some(proc));

        let mut mounts = spec.mounts().to_owned().unwrap_or(vec![]);
        let mut binds = options
            .binds
            .iter()
            .map(|m| {
                let (from, to, flags) = match m {
                    BaseBind::RW(from, to) => (from, to, "rw"),
                    BaseBind::RO(from, to) => (from, to, "ro"),
                };
                runtime::MountBuilder::default()
                    .source(from)
                    .destination(to)
                    .typ("bind")
                    .options(vec![String::from("bind"), String::from(flags)])
                    .build()
            })
            .collect::<Result<Vec<_>, OciSpecError>>()?;
        mounts.append(&mut binds);
        spec.set_mounts(Some(mounts));

        spec.set_hostname(Some(options.name.clone()));

        Ok(spec)
    }

    pub async fn wait(&mut self) -> NodeResult<ExitStatus> {
        let res = self
            .main_child
            .wait()
            .await
            .map_err(IoNodeError::create_errmap("<container wait>"));
        self.running.store(false, atomic::Ordering::Relaxed);
        res
    }

    pub fn tokio_exec(&self) -> (Command, String) {
        let mut cmd = Command::new("runc");
        cmd.arg("exec");
        cmd.stdin(Stdio::null());
        (cmd, self.name.to_owned())
    }

    pub async fn poweroff(&self) -> NodeResult<()> {
        if !self.running.load(atomic::Ordering::Relaxed) {
            return Ok(());
        }

        simple_cmd!("<Send poweroff to container>"; runc kill (self.name) SIGKILL).await?;
        Ok(())
    }
}
