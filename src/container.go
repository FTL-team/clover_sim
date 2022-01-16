package main

import (
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
	Mounts []string
}

func mountOverlayRootfs(c *Container, workspace *Workspace) error {
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
	upperDir := path.Join(workspace.Path, "fs")
	workDir := path.Join(c.Path, "overlay_work")

	targetDir := path.Join(c.Path, "rootfs")

	os.MkdirAll(targetDir, os.ModePerm)
	os.MkdirAll(workDir, os.ModePerm)

	c.Mounts = append(c.Mounts, targetDir)

	fmt.Println(fmt.Sprintf("lowerdir=%s,upperdir=%s,workdir=%s", lowerDir, upperDir, workDir))

	return syscall.Mount("overlay", targetDir, "overlay", 0, fmt.Sprintf("lowerdir=%s,upperdir=%s,workdir=%s", lowerDir, upperDir, workDir))
}

func CreateContainer(name string, workspace *Workspace) (*Container, error) {
	container := &Container{Name: name}
	container.Path = path.Join(LocateSetup(), "containers", container.Name)
	container.Bases = []string{"base"}

	os.MkdirAll(container.Path, os.ModePerm)

	err := mountOverlayRootfs(container, workspace)
	if err != nil {
		// destroyContainer(container)
		return nil, err
	}

	return container, nil
}

func DestroyContainer(container *Container) {
	fmt.Println("Destroying container: " + container.Path)

	for i := 0; i < 3; i++ {
		for _, mount := range container.Mounts {
			syscall.Unmount(mount, 0)
		}

		time.Sleep(time.Second)
	}

	os.RemoveAll(container.Path)
}

func GetContainerLauncher(container *Container, net *NetworkConfig) (*exec.Cmd, error) {
	nspawnArgs := []string{}

	nspawnArgs = append(nspawnArgs, "--machine", container.Name)
	nspawnArgs = append(nspawnArgs, "--directory", ".")

	readonlyMounts := []string{"/tmp/.X11-unix", "/tmp/.virgl_test"}
	readonlyMounts = append(readonlyMounts, path.Join(LocateUserHome(), ".Xauthority")+":/home/clover/.Xauthority")

	for _, mount := range readonlyMounts {
		nspawnArgs = append(nspawnArgs, fmt.Sprintf("--bind=%s", mount))
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
}

func ExecInsideContainer(container *Container, options ExecContainerOptions) *exec.Cmd {
	runOptions := []string{}
	for k, v := range options.ServiceOptions {
		runOptions = append(runOptions, fmt.Sprintf("--property=%s=%s", k, v))
	}
	runOptions = append(runOptions, "--description="+options.Description)
	runOptions = append(runOptions, "--machine="+container.Name)
	runOptions = append(runOptions, "-P")
	runOptions = append(runOptions, "/bin/bash")
	runOptions = append(runOptions, "-c")
	runOptions = append(runOptions, options.Command)

	return exec.Command("systemd-run", runOptions...)
}

func SetupContainerNetwork(container *Container, net *NetworkConfig) error {
	cmd := ExecInsideContainer(container, ExecContainerOptions{
		Command: GenerateContainerNetworkSetup(net),
		ServiceOptions: map[string]string{
			"After": "network-online.target",
			"Wants": "network-online.target",
		},
		Description: "Network setup",
	})
	return cmd.Run()
}