package main

import (
	"bytes"
	"fmt"
	"io/ioutil"
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

	IPs []string

	Logger *Logger
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

	options := fmt.Sprintf("lowerdir=%s,upperdir=%s,workdir=%s", lowerDir, upperDir, workDir)

	c.Logger.Verbose("Mounting overlay rootfs: mount -t overlay overlay %s %s", targetDir, options)

	return syscall.Mount("overlay", targetDir, "overlay", 0, options)
}

func mountTmpfs(c *Container) (string, error) {
	targetDir := path.Join(c.Path, "tmpfs")
	os.MkdirAll(targetDir, os.ModePerm)
	err := syscall.Mount("tmpfs", targetDir, "tmpfs", 0, "")
	if err != nil {
		return targetDir, err
	}
	c.Mounts = append(c.Mounts, targetDir)

	err = os.MkdirAll(path.Join(targetDir, "fs"), os.ModePerm)
	return targetDir, err
}

func CreateContainer(name string, workspace *Workspace) (*Container, error) {
	container := &Container{
		Name: name,
		Path: path.Join(LocateSetup(), "containers", name),
		Bases: []string{"base"},
		Logger: NewLogger(name),
	}

	if workspace != nil {
		container.UpperLayer = path.Join(workspace.Path)
	} else {
		upl, err := mountTmpfs(container)
		if err != nil {
			return container, err
		}
		container.UpperLayer = upl
	}

	os.MkdirAll(container.Path, os.ModePerm)

	err := mountOverlayRootfs(container)
	if err != nil {
		// destroyContainer(container)
		return nil, err
	}

	ioutil.WriteFile(path.Join(container.Path, "hostname"), []byte(container.Name + "\n"), 0644)

	return container, nil
}

func (container *Container) Destroy() {
	container.Logger.Info("Destroying container: %s", container.Name)

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
	readonlyMounts = append(readonlyMounts,  path.Join(LocateSetup(), "containers", "hosts") + ":/etc/hosts")
	readonlyMounts = append(readonlyMounts, path.Join(container.Path, "hostname") + ":/etc/hostname")

	for _, mount := range readonlyMounts {
		nspawnArgs = append(nspawnArgs, fmt.Sprintf("--bind-ro=%s", mount))
	}

	nspawnArgs = append(nspawnArgs, "--network-bridge="+net.BridgeName)

	nspawnArgs = append(nspawnArgs, "--boot")
	nspawnArgs = append(nspawnArgs, "clover_sim")

	container.Logger.Verbose("Systemd-nspawn options: %s", nspawnArgs)

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

func (container *Container) SendXauth() error {
	xauthCmd := exec.Command("xauth", "nextract", "-", os.Getenv("DISPLAY"))
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