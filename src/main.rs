use std::sync::Arc;

use cloversim_lib::{err::NodeError, rpc::NodeRpc};
use cloversim_node::LocalRpc;

fn get_local_rpc() -> Result<Arc<dyn NodeRpc>, NodeError> {
    Ok(Arc::new(LocalRpc::new()))
}

#[tokio::main]
async fn main() {
    // let rpc =
    // cloversim_rrp::run(rpc.clone()).await;
    cloversim_cli::run(get_local_rpc).await;
}
