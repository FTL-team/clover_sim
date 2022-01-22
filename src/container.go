package main

import (
	"bytes"
	"fmt"
	"os"
	"os/exec"
	"path"
	"path/filepath"
	"syscall"
	"time"
)

type Container struct {
	Name string
	Path string

	Bases  []string
	UpperLayer string
	Mounts []string
}

func mountOverlayRootfs(c *Container) error {
	lowerDir := ""
	for i, base := range c.Bases {
		if i != 0 {
			lowerDir += ":"
		}
		base_real, err := filepath.EvalSymlinks(path.Join(LocateSetup(), "base_fs", base))
		if err != nil {
			return err
		}
		lowerDir += base_real
	}

	upperDir := path.Join(c.UpperLayer, "fs")
	workDir := path.Join(c.UpperLayer, ".overlay_work")
	os.Mkdir(workDir, os.ModePerm)

	targetDir := path.Join(c.Path, "rootfs")

	os.MkdirAll(targetDir, os.ModePerm)
	os.MkdirAll(workDir, os.ModePerm)

	c.Mounts = append(c.Mounts, targetDir)

	fmt.Println(fmt.Sprintf("lowerdir=%s,upperdir=%s,workdir=%s", lowerDir, upperDir, workDir))

	return syscall.Mount("overlay", targetDir, "overlay", 0, fmt.Sprintf("lowerdir=%s,upperdir=%s,workdir=%s", lowerDir, upperDir, workDir))
}

func CreateContainer(name string, workspace *Workspace) (*Container, error) {
	container := &Container{
		Name: name,
		Path: path.Join(LocateSetup(), "containers", name),
		Bases: []string{"base"},
		UpperLayer: path.Join(workspace.Path),
	}

	os.MkdirAll(container.Path, os.ModePerm)

	err := mountOverlayRootfs(container)
	if err != nil {
		// destroyContainer(container)
		return nil, err
	}

	return container, nil
}

func (container *Container) Destroy() {
	fmt.Println("Destroying container: " + container.Path)

	for i := 0; i < 3; i++ {
		for _, mount := range container.Mounts {
			syscall.Unmount(mount, 0)
		}

		time.Sleep(time.Second)
	}

	os.RemoveAll(container.Path)
	os.Remove(path.Join(container.UpperLayer, ".overlay_work"))
}

func (container *Container) GetLauncher(net *NetworkConfig) (*exec.Cmd, error) {
	nspawnArgs := []string{}

	nspawnArgs = append(nspawnArgs, "--machine", container.Name)
	nspawnArgs = append(nspawnArgs, "--directory", ".")

	readonlyMounts := []string{"/tmp/.X11-unix", "/tmp/.virgl_test"}

	for _, mount := range readonlyMounts {
		nspawnArgs = append(nspawnArgs, fmt.Sprintf("--bind-ro=%s", mount))
	}

	nspawnArgs = append(nspawnArgs, "--network-bridge="+net.BridgeName)

	nspawnArgs = append(nspawnArgs, "--boot")
	nspawnArgs = append(nspawnArgs, "clover_sim")

	cmd := exec.Command("systemd-nspawn", nspawnArgs...)
	cmd.Dir = path.Join(container.Path, "rootfs")

	return cmd, nil
}

type ExecContainerOptions struct {
	Command        string
	ServiceOptions map[string]string
	Description    string
	Uid int
}

func (container *Container) Exec(options ExecContainerOptions) *exec.Cmd {
	runOptions := []string{}
	for k, v := range options.ServiceOptions {
		runOptions = append(runOptions, fmt.Sprintf("--property=%s=%s", k, v))
	}

	if options.Uid != 0 {
		runOptions = append(runOptions, fmt.Sprintf("--uid=%d", options.Uid))
	}

	runOptions = append(runOptions, "--description="+options.Description)
	runOptions = append(runOptions, "--machine="+container.Name)
	runOptions = append(runOptions, "-P")
	runOptions = append(runOptions, "/bin/bash")
	runOptions = append(runOptions, "-c")
	runOptions = append(runOptions, options.Command)

	return exec.Command("systemd-run", runOptions...)
}

func (container *Container) SetupNetwork(net *NetworkConfig) error {
	cmd := container.Exec(ExecContainerOptions{
		Command: net.GenerateContainerSetup(),
		ServiceOptions: map[string]string{
			"After": "network-online.target",
			"Wants": "network-online.target",
		},
		Description: "Network setup",
	})
	return cmd.Run()
}


func (container *Container) SendXauth() error {
	xauthCmd := exec.Command("xauth", "nextract", "-", ":0")
	out, err := xauthCmd.Output()
	if err != nil {
		return err
	}

	out[0] = 'f'
	out[1] = 'f'
	out[2] = 'f'
	out[3] = 'f'

	containerCmd := container.Exec(ExecContainerOptions{
		Command: "xauth nmerge - ",
		Description: "Setup xauth",
		Uid: 1000,
		ServiceOptions: map[string]string{
			"After": "multi-user.target",
			"Wants": "multi-user.target",
		},
	})
	containerCmd.Stdin = bytes.NewReader(out)
	containerCmd.Stdout = os.Stdout
	return containerCmd.Run()
}