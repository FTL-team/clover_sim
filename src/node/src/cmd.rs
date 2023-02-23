use cloversim_lib::err::{NodeError, NodeResult};
use tokio::process::Command;

pub async fn execute_cmd(mut cmd: Command, name: &str) -> NodeResult<Vec<u8>> {
    let res = cmd
        .output()
        .await
        .map_err(IoNodeError::create_errmap(name))?;

    if !res.status.success() {
        return Err(NodeError::CmdError(
            String::from(name),
            String::from_utf8(res.stderr).expect("Non-utf8 command output"),
        ));
    }
    Ok(res.stdout)
}

macro_rules! simple_cmd {
    ($cmd:ident, ) => {};

    ($cmd:ident, ($a:expr) $($tail:tt)*) =>
    {
        $cmd.arg(&$a);
        simple_cmd!($cmd, $($tail)*);
    };

    ($cmd:ident, $a:ident $($tail:tt)*) => (simple_cmd!($cmd, (stringify!($a)) $($tail)* ));
    ($cmd:ident, -$a:ident $($tail:tt)*) => (simple_cmd!($cmd, ("-".to_owned() + stringify!($a)) $($tail)* ));

    ($d:expr; $c:ident $($tail:tt)*) => {
        {
            use crate::cmd::execute_cmd;
            use std::process::Stdio;

            let mut cmd = Command::new(stringify!($c));
            cmd.stdin(Stdio::null());
            simple_cmd!(cmd, $($tail)*);
            execute_cmd(cmd, $d)
        }
    };
}

pub(crate) use simple_cmd;

use crate::errhelper::IoNodeError;

// #[macro_export]
// macro_rules! simple_cmd {
//     ( {$name:expr} $test:expr ) => {
//         {
//             let mut cmd = Command::new("test");
//             // $(
//             //     cmd.arg($x);
//             // )*
//             execute_cmd(cmd, $name)
//         }
//     };
// }
