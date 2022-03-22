# Getting started
## Workspaces

To work with workspaces you should use this command pattern:
```bash
sudo ./clover_sim workspace command
```
Instead of `workspace` you can use `ws`

Here is a list of commands:
- `create test` - Create a new workspace, name as param
- `list` - get list of all workspaces
- `remove test` - Remove a workspace, name as param
- `export test` - Export a workspace as tar.gz archive, name as param
- `import ./test.tar.gz` - Import a workspace from tar.gz archive, path to archieve as param
- `duplicate old new` - Duplicate a workspace
- `clean test` - Delete a workspace cache and logs



## Starting simulator

1. Go to `clover_sim` directory
2. Create workspace(workspace is concept for managing and transfering simulator files: projects, packages, etc.)
    ```bash
    sudo ./clover_sim workspace create test
    ```
3. Launch simulator container
    ```bash
    sudo  ./clover_sim launch test
    ```
4. fds   
5. Connect via ssh using `ssh clover@192.168.77.2`
6. Login into container  using login `clover` and password `clover`
7. Launch simulator using
    ```bash
    roslaunch clover_simulation simulator.launch 
    ```
8. Enjoy simulator

9. To exit container use `sudo poweroff` (inside of conatainer)

10. For security reasons you should write code inside another container
`ssh clover@192.168.77.10`


For more information check `sudo ../clover_sim help`