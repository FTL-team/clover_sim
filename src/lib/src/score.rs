use serde::{Deserialize, Serialize};
use ts_rs::TS;

#[derive(Serialize, Deserialize, Clone, Debug, TS)]
#[ts(export)]
pub struct TaskScore {
    pub name: String,
    pub max_score: f64,
    pub score: f64,
    pub failed: bool,
    pub children: Vec<TaskScore>,
}

impl TaskScore {
    pub fn create_flat_repr(&self, level: usize) -> Vec<(usize, &TaskScore)> {
        let mut res = Vec::new();
        res.push((level, self));
        for child in self.children.iter() {
            let mut child_flatten = child.create_flat_repr(level + 1);
            res.append(&mut child_flatten);
        }

        res
    }
}
