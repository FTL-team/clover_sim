package main

import (
	"fmt"
	"io/ioutil"
	"os"
	"os/exec"
	"path"
	"sync"
	"time"
)

type Container struct {
	Name string
	Path string

	Overlay *Overlay

	IPs []string

	Logger *Logger

	Plugins []*ContainerPlugin
}

type ContainerPlugin struct {
	Name string

	MountsRO []string
	MountsRW []string
	LauncherArguments []string
	
	RunOnBoot func(*Container) error
}

func CreateContainer(name string, workspace *Workspace) (*Container, error) {
	container := &Container{
		Name: name,
		Path: path.Join(LocateSetup(), "containers", name),
		Logger: NewLogger(name),
	}

	container.Overlay = &Overlay{
		Layers: []*OverlayEntry{
			CreateBaseFsEntry("base"),
			CreateBaseFsEntry("cloversim"),
		},
		Path: path.Join(container.Path, "rootfs"),
		Logger: container.Logger,
	}

	if workspace != nil {
		container.Overlay.OverlayLayer = workspace.CreateOverlayEntry()
	} else {
		overlayLayer, err := CreateTmpFsEntry(path.Join(container.Path, "tmpfs")) 
		container.Overlay.OverlayLayer = overlayLayer
		if err != nil {
			return container, err	
		}
	}

	os.MkdirAll(container.Path, os.ModePerm)
	os.MkdirAll(path.Join(container.Path, "shared"), os.ModePerm)

	err := container.Overlay.Mount()
	if err != nil {
		// destroyContainer(container)
		return container, err
	}

	ioutil.WriteFile(container.ContainerFile("hostname"), []byte(container.Name + "\n"), 0644)

	return container, nil
}

func (container *Container) AddPlugin(plugin *ContainerPlugin) {
	container.Plugins = append(container.Plugins, plugin)
}

func (container *Container) AddPluginCheckError(plugin *ContainerPlugin, err error) error {
	if err != nil {
		if plugin != nil {
			container.Logger.Error("Plugin %s failed: %s", plugin.Name, err)
		}else{
			container.Logger.Error("Plugin error: %s", err)
		}
		return err
	}else{
		container.Plugins = append(container.Plugins, plugin)
		return nil
	}
}



func (container *Container) Destroy() {
	container.Logger.Info("Destroying container: %s", container.Name)

	container.Overlay.Destroy()
	os.RemoveAll(container.Path)
}

func (container *Container) GetLauncher() (*exec.Cmd, error) {
	nspawnArgs := []string{}

	nspawnArgs = append(nspawnArgs, "--machine", container.Name)
	nspawnArgs = append(nspawnArgs, "--directory", ".")

	readonlyMounts := []string{}
	readwriteMounts := []string{}

	for _, plugin := range container.Plugins {
		readonlyMounts = append(readonlyMounts, plugin.MountsRO...)
		readwriteMounts = append(readwriteMounts, plugin.MountsRW...)
		nspawnArgs = append(nspawnArgs, plugin.LauncherArguments...)
	}

	for _, mount := range readonlyMounts {
		nspawnArgs = append(nspawnArgs, fmt.Sprintf("--bind-ro=%s", mount))
	}

	for _, mount := range readwriteMounts {
		nspawnArgs = append(nspawnArgs, fmt.Sprintf("--bind=%s", mount))
	}

	nspawnArgs = append(nspawnArgs, "--boot")
	nspawnArgs = append(nspawnArgs, "clover_sim")

	container.Logger.Verbose("Systemd-nspawn options: %s", nspawnArgs)

	cmd := exec.Command("systemd-nspawn", nspawnArgs...)
	cmd.Dir = path.Join(container.Path, "rootfs")

	return cmd, nil
}

func (container *Container) Run(stopSignal *sync.Cond,) error {

	container.Logger.Info("Launching container: %s ", container.Name)
 
	cmd, err := container.GetLauncher()
	if err != nil {
		container.Logger.Error("Failed to launch container: %s", err)
		return err
	}

	Running := true

	go func() {
		stopSignal.L.Lock()
		defer stopSignal.L.Unlock()
		stopSignal.Wait()
		if Running {
			container.Logger.Info("Stopping container %s", container.Name)
			container.Poweroff()
		}
	}()

	go func() {
		time.Sleep(800 * time.Millisecond)
		for _, plugin := range container.Plugins {
			go plugin.RunOnBoot(container)
		}
	}()

	err = cmd.Run()
	Running = false
	if err != nil {
		container.Logger.Error("Container failed: %s", err)
	} else {
		container.Logger.Info("Container %s exited", container.Name)
	}

	return err
}

type ExecContainerOptions struct {
	Command        string
	ServiceOptions map[string]string
	Description    string
	Uid int
	Gid int
	Unit string
}

func (container *Container) Exec(options ExecContainerOptions) *exec.Cmd {
	runOptions := []string{}
	for k, v := range options.ServiceOptions {
		runOptions = append(runOptions, fmt.Sprintf("--property=%s=%s", k, v))
	}

	if options.Uid != 0 {
		runOptions = append(runOptions, fmt.Sprintf("--uid=%d", options.Uid))
	}

	if options.Gid != 0 {
		runOptions = append(runOptions, fmt.Sprintf("--gid=%d", options.Gid))
	}

	if options.Unit != "" {
		runOptions = append(runOptions, fmt.Sprintf("--unit=%s", options.Unit))
		runOptions = append(runOptions, "--remain-after-exit")
	}

	runOptions = append(runOptions, "--description="+options.Description)
	runOptions = append(runOptions, "--machine="+container.Name)
	runOptions = append(runOptions, "-P")
	runOptions = append(runOptions, "/bin/bash")
	runOptions = append(runOptions, "-ic")
	runOptions = append(runOptions, options.Command)

	return exec.Command("systemd-run", runOptions...)
}

func (container *Container) Systemctl(options ...string) error {
	return exec.Command("systemctl", append([]string{"--machine="+container.Name}, options...)...).Run()
}

func (container *Container) Poweroff() error {
	return container.Systemctl("poweroff")
}

func (container *Container) ContainerFile(name string) string {
	return path.Join(container.Path, name)
}

var createdShared = false
func SharedContainerFile(name string) string {
	if !createdShared {
		os.MkdirAll(path.Join(LocateSetup(), "containers", "_shared"), os.ModePerm)
		createdShared = true
	}
	return 	path.Join(LocateSetup(), "containers", "_shared", name)
}