# Workspaces

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

