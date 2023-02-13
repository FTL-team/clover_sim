use node::{LocalRpc};

mod node;
mod filelock;
mod cli;

#[tokio::main]
async fn main() {
    let rpc = LocalRpc::new();
    cli::run(&rpc).await;
}
