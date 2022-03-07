package main

import (
	"fmt"
	"io/ioutil"
	"os"
	"os/exec"
	"path"
	
	"gopkg.in/yaml.v2"
)

type Workspace struct {
	Name string
	Path string
}

type WorkspaceSerialized struct {
	Name string `yaml:"name"`
}

var CLEAN_DIRS = []string{
	"fs/var/log",
	"fs/var/cache",
	"fs/home/clover/.ros/log",
	"fs/home/clover/.cache",
}

func validateWorkspaceName(name string) error {
	for _, c := range name {

		ok := false
		ok = ok || (c >= 'a' && c <= 'z')
		ok = ok || (c >= 'A' && c <= 'Z')
		ok = ok || (c >= '0' && c <= '9')
		ok = ok || (c == '_')

		if !ok {
			return fmt.Errorf("invalid workspace name: %s, namespace names can contain only latin letters, numbers, and underscores", name)
		}
	}

	return nil
}

func (workspace *Workspace) Serialize() ([]byte, error) {
	return yaml.Marshal(&WorkspaceSerialized{
		Name: workspace.Name,
	})
}

func (workspace *Workspace) Save() error {

	serializedWorkspace, err := workspace.Serialize()
	if err != nil {
		return err
	}

	return ioutil.WriteFile(path.Join(workspace.Path, "workspace.yml"), serializedWorkspace, 0777)	
}

func DeserializeWorkspace(data []byte) (*Workspace, error) {
	var workspaceSerialized WorkspaceSerialized
	err := yaml.Unmarshal(data, &workspaceSerialized)
	if err != nil {
		return nil, err
	}

	if err := validateWorkspaceName(workspaceSerialized.Name); err != nil {
		return nil, err
	}

	return &Workspace{
		Name: workspaceSerialized.Name,
	}, nil
}


func CreateWorkspace(name string) (*Workspace, error) {

	if err := validateWorkspaceName(name); err != nil {
		return nil, err
	}

	Workspace := &Workspace{
		Name: name,
		Path: path.Join(LocateSetup(), "workspaces", name),
	}

	if _, err := os.Stat(Workspace.Path); err == nil {
		return nil, fmt.Errorf("Workspace %s already exists", Workspace.Name)
	}

	os.MkdirAll(Workspace.Path, os.ModePerm)
	if err := Workspace.Save(); err != nil {
		return nil, err
	}	

	os.MkdirAll(path.Join(Workspace.Path, "fs"), os.ModePerm)

	return Workspace, nil
}

func LoadWorkspace(name string) (*Workspace, error) {
	workspacePath := path.Join(LocateSetup(), "workspaces", name) 
	if _, err := os.Stat(workspacePath); err != nil {
		return nil, fmt.Errorf("Workspace %s does not exist", name)
	}
	workspaceInfo, err := ioutil.ReadFile(workspacePath + "/workspace.yml")
	if err != nil {
		return nil, err
	}

	workspace, err := DeserializeWorkspace(workspaceInfo)
	if err != nil {
		return nil, err
	}

	workspace.Path = workspacePath;

	return workspace, nil
}

func (workspace *Workspace) Remove() error {
	return os.RemoveAll(workspace.Path)
}

func ListWorkspaceNames() ([]string, error) {
	entries, err := os.ReadDir(path.Join(LocateSetup(), "workspaces"))
	if err != nil {
		return nil,	err
	}
	
	names := []string{}
	for _, entry := range entries {
		names = append(names, entry.Name())
	}
	return names, nil
}

func (workspace *Workspace) Export() error {
	wd, err := os.Getwd()
	if err != nil {
		return err
	}

	filesI, err := os.ReadDir(workspace.Path)
	if err != nil {
		return err
	}

	to_exclude := []string{}
	for _, dir := range CLEAN_DIRS {
		to_exclude = append(to_exclude, "--exclude=" + dir)
	}

	export_path := path.Join(wd, workspace.Name + ".tar.gz")
	args := []string{"-zcvf", export_path}
	args = append(to_exclude, args...)

	for _, fileI := range filesI {
		args = append(args, fileI.Name())
	}

	cmd := exec.Command("tar", args...)
	cmd.Dir = workspace.Path
	cmd.Stdout = os.Stdout
	cmd.Stderr = os.Stderr

	err = cmd.Run()
	if err != nil { 
		return err
	}

	uid, gid, err := GetUserID()
	if err != nil {
		return err
	}
	return os.Chown(export_path, int(uid), int(gid))
}

func ImportWorkspace(file string) error {
	cwd, err := os.Getwd()
	if err != nil {
		return err
	}

	file = path.Join(cwd, file)

	if _, err := os.Stat(file); err != nil {
		return err
	}

	unpackingPath := path.Join(LocateSetup(), "workspaces", "__unpackingWS")
	os.MkdirAll(unpackingPath, os.ModePerm)
	defer os.Remove(unpackingPath)

	cmd := exec.Command("tar", "-xvf", file)
	cmd.Dir = unpackingPath
	cmd.Stdout = os.Stdout
	cmd.Stderr = os.Stderr

	err = cmd.Run()
	if err != nil { 
		return err
	}

	workspace, err := LoadWorkspace("__unpackingWS")
	if err != nil {
		return err
	}
	return os.Rename(unpackingPath, path.Join(LocateSetup(), "workspaces", workspace.Name))
}

func (sourceWorkspace *Workspace) Duplicate(targetName string) error {
	if err := validateWorkspaceName(targetName); err != nil {
		return err
	}

	targetPath := path.Join(LocateSetup(), "workspaces", targetName)
	if _, err := os.Stat(targetPath); err == nil {
		return fmt.Errorf("Workspace %s already exists", targetName)
	}

	targetWorkspace := &Workspace{
		Name: targetName,
		Path: targetPath,
	}


	cmd := exec.Command("cp", "-rp", sourceWorkspace.Path, targetWorkspace.Path)
	if err := cmd.Run(); err != nil {
		return err
	}


	return targetWorkspace.Save()
}

func (workspace *Workspace) CreateOverlayEntry() (*OverlayEntry) {
	overlayEntry := &OverlayEntry{
		Path: path.Join(workspace.Path, "fs"),
		OverlayWork: path.Join(workspace.Path, ".overlay_work"),
	}
	return overlayEntry
}

func (workspace *Workspace) Clean() error {
	for _, entry := range CLEAN_DIRS {
		os.RemoveAll(path.Join(workspace.Path, entry))
	}
	return nil
}