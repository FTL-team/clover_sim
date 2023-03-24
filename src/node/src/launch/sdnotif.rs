use std::path::PathBuf;

use cloversim_lib::err::NodeResult;
use tokio::net::UnixDatagram;

use crate::{errhelper::IoNodeError, workroot::WorkEntry};

pub enum SdNotifyMessage {
    Ready,
}

pub struct SdNotifyListener {
    targ: WorkEntry,
    sock: UnixDatagram,
    recv_buffer: [u8; 1024],
}

impl SdNotifyListener {
    pub fn new(targ: WorkEntry) -> NodeResult<Self> {
        let sock =
            UnixDatagram::bind(targ.path()).map_err(IoNodeError::create_fs_errmap(targ.path()))?;

        Ok(Self {
            targ,
            sock,
            recv_buffer: [0u8; 1024],
        })
    }

    pub async fn recv(&mut self) -> NodeResult<SdNotifyMessage> {
        loop {
            let recieved_n = self
                .sock
                .recv(&mut self.recv_buffer)
                .await
                .map_err(IoNodeError::create_errmap("<sd_notify sock>"))?;

            for notification in self.recv_buffer[..recieved_n].split(|x| *x == 0xa) {
                if notification.starts_with("READY=".as_bytes()) {
                    return Ok(SdNotifyMessage::Ready);
                }
            }
        }
    }

    pub fn get_path(&self) -> &'_ PathBuf {
        self.targ.path()
    }
}
