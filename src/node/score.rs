use serde::Deserialize;

#[derive(Deserialize, Clone, Debug)]
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
