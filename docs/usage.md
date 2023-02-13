# Usage

## Workspaces

Workspace is a concept for managing and transfering simulator files: projects, packages, etc.


You can manage your workspaces using **workspaces** subcommands:
- `workspace create test` - Create a new workspace, name as param
- `workspace remove test` - Remove a workspace, name as param
- `workspace list` - get list of all workspaces
- `workspace export test` - Export a workspace as tar.gz archive, name as param
- `workspace import ./test.tar.gz` - Import a workspace from tar.gz archive, path to archieve as param
- `workspace duplicate old new` - Duplicate a workspace
- `workspace clean test` - Delete a workspace cache and logs

> Instead of writing workspace every time you can use shortcut **ws**

Some examples:

```bash
sudo ./cloversim ws create test # Create workspace "test"
sudo ./cloversim workspace list # List all workspaces
sudo ./cloversim workspace remove old_code # Remove workspace "old_code"
sudo ./cloversim ws export test # Export workspace "test", file with name test.tar.gz will apear
```

## Launch and task

To launch a simulation `launch` subcommand is used. `launch` subcommand accepts one required argument that contains workspace used for simulation. There is also optional argument called `--task` that contains path of task(relative to `tasks`), default value is `base_task`. Example usage:
```bash
launch --task=example_task workspace_name
```

To find currently available tasks you can use `task list`. This command will show list of available tasks with their description.

# Launch commands

After starting container via `launch` subcommand you can use these commands inside it:

-  `help`    - show this help
-  `exit`    - shutdowns cloversim
-  `start`   - start simulator (gazebo, clover, etc.)
-  `stop`    - stop simulator (gazebo, clover, etc.)
-  `restart` - restart the simulator (gazebo, clover, etc.)
-  `rget`    - get current randomzation
-  `rset`    - set randomzation to value from second argument
-  `rnew`    - generate new random randomzation
-  `score`   - show current task scoring
-  `tpath`   - get path of file in task (README.md by default)
-  `tread`   - read file in task (README.md by default)