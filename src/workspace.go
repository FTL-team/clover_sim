package main

import (
	"fmt"
	"io/ioutil"
	"os"
	"path"
	"os/exec"


	"gopkg.in/yaml.v2"

)

type Workspace struct {
	Name string
	Path string
}

type WorkspaceSerialized struct {
	Name string `yaml:"name"`
}

func SerializeWorkspace(workspace *Workspace) ([]byte, error) {
	return yaml.Marshal(&WorkspaceSerialized{
		Name: workspace.Name,
	})
}

func DeserializeWorkspace(data []byte) (*Workspace, error) {
	var workspaceSerialized WorkspaceSerialized
	err := yaml.Unmarshal(data, &workspaceSerialized)
	if err != nil {
		return nil, err
	}

	return &Workspace{
		Name: workspaceSerialized.Name,
	}, nil
}


func CreateWorkspace(name string) (*Workspace, error) {
	Workspace := &Workspace{
		Name: name,
		Path: path.Join(LocateSetup(), "workspaces", name),
	}

	if _, err := os.Stat(Workspace.Path); err == nil {
		return nil, fmt.Errorf("Workspace %s already exists", Workspace.Name)
	}

	os.MkdirAll(Workspace.Path, os.ModePerm)
	
	serializedWorkspace, err := SerializeWorkspace(Workspace)
	if err != nil {
		return nil, err
	}

	err = ioutil.WriteFile(path.Join(Workspace.Path, "workspace.yml"), serializedWorkspace, 0777)
	if err != nil {
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

func RemoveWorkspace(Workspace *Workspace) error {
	return os.RemoveAll(Workspace.Path)
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

func ExportWorkspace(workspace *Workspace) error {
	wd, err := os.Getwd()
	if err != nil {
		return err
	}

	filesI, err := os.ReadDir(workspace.Path)
	if err != nil {
		return err
	}

	export_path := path.Join(wd, workspace.Name + ".tar.gz")
	args := []string{"-zcvf", export_path}

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