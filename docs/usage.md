# Usage

## Workspaces

Workspace is a concept for managing and transfering simulator files: projects, packages, etc.


You can manage your workspaces using **workspaces** subcommands:
- `workspace create test` - Create a new workspace, name as param
- `workspace list` - get list of all workspaces
- `workspace remove test` - Remove a workspace, name as param
- `workspace export test` - Export a workspace as tar.gz archive, name as param
- `workspace import ./test.tar.gz` - Import a workspace from tar.gz archive, path to archieve as param
- `workspace duplicate old new` - Duplicate a workspace
- `workspace clean test` - Delete a workspace cache and logs

> Instead of writing workspace every time you can use shortcut **ws**

Some examples:

```bash
sudo ./clover_sim ws create test # Create workspace "test"
sudo ./clover_sim workspace list # List all workspaces
sudo ./clover_sim workspace remove old_code # Remove workspace "old_code"
sudo ./clover_sim ws export test # Export workspace "test", file with name test.tar.gz will apear
```

## Internal comments

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
-  `run`     - run user program (runs /home/clover/run.sh inside clover0 container)