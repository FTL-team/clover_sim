use std::{
    path::{Path, PathBuf},
    sync::{
        atomic::{AtomicU8, Ordering},
        Arc, Mutex,
    },
};
use tokio::process::Command;

use crate::node::{
    clean::Cleaner,
    cmd::simple_cmd,
    err::{NodeError, NodeResult},
    workroot::WorkEntry,
};

pub struct Network {
    id: u16,
    cleaner: Arc<Cleaner>,
    netname: String,
    cur_ip: AtomicU8,
    hosts_entry: WorkEntry,
    hosts: Mutex<String>,
}

pub struct ContainerNetwork {
    nsname: String,
    veth: String,
    cleaner: Arc<Cleaner>,
    _ip: u8,
}

const HOST_BASE: &str = "
# This file is managed by cloversim
# Default
127.0.0.1  localhost
::1        localhost ip6-localhost ip6-loopback
ff02::1    ip6-allnodes
ff02::2    ip6-allrouters

# Containers
192.168.77.1 host
";

impl Network {
    #[rustfmt::skip]
    async fn setup_network(netname: &String) -> NodeResult<()> {
        simple_cmd!("<bridge: create>"; ip link add name (netname) type bridge).await?;
        simple_cmd!("<bridge: set up>"; ip link set (netname) up).await?;
        simple_cmd!("<bridge: add ip>"; ip addr add ("192.168.77.1/24") dev (netname)).await?;

        simple_cmd!("<sysctl: enable ipv4 routing>"; sysctl -w ("net.ipv4.ip_forward=1")).await?;
        simple_cmd!("<sysctl: enable ipv6 routing>"; sysctl -w ("net.ipv6.conf.all.forwarding=1")).await?;

        simple_cmd!("<route : create nat route"; iptables -t nat -A POSTROUTING -s ("192.168.77.1/24") ("!") -o (netname) -j MASQUERADE).await?;

        Ok(())
    }

    async fn write_hosts(&self) -> NodeResult<()> {
        let host_s = self.hosts.lock().expect("Hosts lock failed").clone();
        tokio::fs::write(self.hosts_entry.path(), host_s.as_bytes())
            .await
            .map_err(NodeError::create_fs_errmap(self.hosts_entry.path()))?;
        Ok(())
    }

    pub async fn new(
        hosts_entry: WorkEntry,
        id: u16,
        cleaner: Arc<Cleaner>,
    ) -> NodeResult<Network> {
        let netname = format!("cloversim_{:04x}", id);
        tokio::fs::File::create(hosts_entry.path())
            .await
            .map_err(NodeError::create_fs_errmap(hosts_entry.path()))?;

        let hosts = String::from(HOST_BASE);
        let n = Network {
            id,
            cleaner,
            netname,
            hosts_entry,
            cur_ip: AtomicU8::new(10),
            hosts: Mutex::new(hosts),
        };
        Self::setup_network(&n.netname).await?;

        n.write_hosts().await?;
        Ok(n)
    }

    pub fn get_hosts_path(&self) -> PathBuf {
        self.hosts_entry.path().clone()
    }
}

impl Drop for Network {
    fn drop(&mut self) {
        let netname = self.netname.clone();
        self.cleaner.clone().add_task(tokio::spawn(async move {
            simple_cmd!("<remove bridge>"; ip link del name (netname)).await?;

            Ok(())
        }));
    }
}

impl ContainerNetwork {
    pub async fn new(
        net: &Network,
        name: &str,
        prefered_ip: Option<u8>,
    ) -> NodeResult<ContainerNetwork> {
        let ip = match prefered_ip {
            Some(ip) => ip,
            None => net.cur_ip.fetch_add(1, Ordering::Relaxed),
        };

        let nsname = format!("{}_{}", net.netname, name);
        let veth = format!("veth-{}-{:04x}", ip, net.id);
        let vpeer = format!("vpeer-{}-{:04x}", ip, net.id);

        let n = ContainerNetwork {
            cleaner: net.cleaner.clone(),
            _ip: ip,
            nsname,
            veth,
        };

        simple_cmd!("<netns: create>"; ip netns add (n.nsname)).await?;
        simple_cmd!("<netns: set lo>"; ip netns exec (n.nsname) ip link set lo up).await?;

        simple_cmd!("<veth : create>"; ip link add (n.veth) type veth peer name (vpeer)).await?;
        simple_cmd!("<veth : set up>"; ip link set (n.veth) up master (net.netname)).await?;

        simple_cmd!("<vpair: set up>"; ip link set (vpeer) netns (n.nsname) name host up).await?;
        simple_cmd!("<vpair: add ip>"; ip netns exec (n.nsname) ip addr add (format!("192.168.77.{}/24", ip)) dev host).await?;
        simple_cmd!("<vpair: route >"; ip netns exec (n.nsname) ip route add default via ("192.168.77.1")).await?;

        net.hosts
            .lock()
            .expect("Hosts lock failed")
            .push_str(&format!("192.168.77.{} {}\n", ip, name));
        net.write_hosts().await?;

        Ok(n)
    }

    pub fn get_ns_path(&self) -> std::path::PathBuf {
        return Path::new("/run/netns").join(&self.nsname);
    }
}

impl Drop for ContainerNetwork {
    fn drop(&mut self) {
        let nsname = self.nsname.clone();
        let veth = self.veth.clone();
        self.cleaner.clone().add_task(tokio::spawn(async move {
            let err1 = simple_cmd!("<remove veth>"; ip link del name (veth)).await;
            let err2 = simple_cmd!("<remove netns>"; ip netns del (nsname)).await;

            err1?;
            err2?;
            Ok(())
        }));
    }
}
