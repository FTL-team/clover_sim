use std::fs::File;

use async_broadcast::Sender;
use async_trait::async_trait;
use cloversim_lib::{
    err::{NodeError, NodeResult},
    launch_event::LaunchEvent,
    rpc::{NodeRpc, NodeStatus, TaskInfo},
};
use futures::{stream::StreamExt, SinkExt};
use reqwest::Url;
use tokio::select;
use tokio_tungstenite::tungstenite::Message::Text;

pub struct RemoteRpc {
    addr: Url,
    client: reqwest::Client,
}

impl RemoteRpc {
    pub async fn connect(addr: String) -> NodeResult<RemoteRpc> {
        let url = match Url::parse(&addr) {
            Ok(url) => url,
            Err(e) => return Err(NodeError::HttpError(e.to_string())),
        };
        let url = url.join("cloversim_api/").unwrap();
        let rpc = RemoteRpc {
            addr: url,
            client: reqwest::Client::new(),
        };
        let status = rpc.status().await?;
        if status.is_ok {
            Ok(rpc)
        } else {
            Err(NodeError::HttpError(format!(
                "Invalid response {:#?}",
                status
            )))
        }
    }
}

fn req_errmap(e: reqwest::Error) -> NodeError {
    NodeError::HttpError(e.to_string())
}

#[async_trait]
impl NodeRpc for RemoteRpc {
    async fn status(&self) -> NodeResult<NodeStatus> {
        reqwest::get(self.addr.join("status").unwrap())
            .await
            .map_err(|e| NodeError::HttpError(e.to_string()))?
            .json()
            .await
            .map_err(|e| NodeError::HttpError(e.to_string()))?
    }

    async fn create_workspace(&self, name: String) -> NodeResult<()> {
        self.client
            .post(self.addr.join("create_workspace").unwrap())
            .json(&name)
            .send()
            .await
            .map_err(req_errmap)?
            .json()
            .await
            .map_err(req_errmap)?
    }
    async fn remove_workspace(&self, name: String) -> NodeResult<()> {
        self.client
            .post(self.addr.join("remove_workspace").unwrap())
            .json(&name)
            .send()
            .await
            .map_err(req_errmap)?
            .json()
            .await
            .map_err(req_errmap)?
    }
    async fn list_workspaces(&self) -> NodeResult<Vec<String>> {
        self.client
            .get(self.addr.join("list_workspaces").unwrap())
            .send()
            .await
            .map_err(req_errmap)?
            .json()
            .await
            .map_err(req_errmap)?
    }
    async fn export_workspace(&self, name: String, to: File) -> NodeResult<()> {
        todo!();
    }
    async fn import_workspace(&self, from: File) -> NodeResult<String> {
        todo!();
    }
    async fn duplicate_workspace(&self, name: String, newname: String) -> NodeResult<()> {
        self.client
            .post(self.addr.join("duplicate_workspace").unwrap())
            .json(&(name, newname))
            .send()
            .await
            .map_err(req_errmap)?
            .json()
            .await
            .map_err(req_errmap)?
    }
    async fn clean_workspace(&self, name: String) -> NodeResult<()> {
        self.client
            .post(self.addr.join("clean_workspace").unwrap())
            .json(&name)
            .send()
            .await
            .map_err(req_errmap)?
            .json()
            .await
            .map_err(req_errmap)?
    }
    async fn list_tasks(&self) -> NodeResult<Vec<TaskInfo>> {
        self.client
            .get(self.addr.join("list_tasks").unwrap())
            .send()
            .await
            .map_err(req_errmap)?
            .json()
            .await
            .map_err(req_errmap)?
    }

    async fn launch(&self, ws_name: String, task_path: String) -> NodeResult<()> {
        self.client
            .post(self.addr.join("launch").unwrap())
            .json(&(ws_name, task_path))
            .send()
            .await
            .map_err(req_errmap)?
            .json()
            .await
            .map_err(req_errmap)?
    }

    async fn create_launch_channel(&self) -> NodeResult<Sender<LaunchEvent>> {
        let (chan_tx, mut chan_rx) = async_broadcast::broadcast(100);
        let mut wsurl = self.addr.join("./launch_channel").unwrap();
        wsurl
            .set_scheme(if wsurl.scheme() == "https" {
                "wss"
            } else {
                "ws"
            })
            .unwrap();
        let conn = tokio_tungstenite::connect_async(wsurl).await;
        let mut conn = match conn {
            Ok(conn) => conn.0,
            Err(e) => {
                return Err(NodeError::HttpError(format!("Websocket error: {}", e)));
            }
        };
        // let (ws_rx, ws_tx) = conn.split();
        let res_tx = chan_tx.clone();
        tokio::spawn(async move {
            loop {
                select! {
                    msg = conn.next() => {
                        let Some(msg) = msg else {
                            break;
                        };
                        let Ok(msg) = msg else {
                            break;
                        };
                        let Text(msg) = msg else {
                            continue;
                        };
                        let Ok(msg) = serde_json::from_str::<LaunchEvent>(&msg) else {
                            continue;
                        };

                        let chan_rx_disabled = chan_rx.deactivate();
                        chan_tx.try_broadcast(msg).unwrap();
                        chan_rx = chan_rx_disabled.activate();

                    }

                    msg = chan_rx.recv() => {
                        let Ok(msg) = msg else {
                            break;
                        };
                        let Ok(msg) = serde_json::to_string(&msg) else {
                            continue;
                        };
                        conn.send(Text(msg)).await.unwrap();
                    }
                }
            }
        });
        Ok(res_tx)
    }

    async fn path_task_file(&self, file: String) -> NodeResult<String> {
        Ok(self
            .addr
            .join("task/")
            .unwrap()
            .join(&file)
            .unwrap()
            .to_string())
    }

    async fn read_task_file(&self, file: String) -> NodeResult<Vec<u8>> {
        Ok(self.client
            .get(self.addr.join("task/").unwrap().join(&file).unwrap())
            .send()
            .await
            .map_err(req_errmap)?
            .bytes()
            .await
            .map_err(req_errmap)?
            .to_vec())
    }
}
