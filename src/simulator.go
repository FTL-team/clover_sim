package main

import (
	"sync"
	"time"

)


type MachineOptions struct {
	Name string
	Workspace *Workspace
	Network *NetworkConfig
	DesiredIP int
}


func LaunchMachine(options MachineOptions, net *NetworkConfig) (error) {
	container, err := CreateContainer(options.Name, options.Workspace)
	if err != nil {
		if container != nil {
			container.Logger.Error("Failed to create container: %s", err)
		}else{
			HostLogger.Error("Failed to create container: %s", err)
		}

		return err
	}
	defer container.Destroy()


	container.Logger.Info("Launching container: %s ", container.Name)

	cmd, err := container.GetLauncher(net)
	if err != nil {
		container.Logger.Error("Failed to launch container: %s", err)
		return err
	}

	Running := true

	go func() {
		time.Sleep(time.Millisecond * 800) // Wait 1.5 second for systemd to start
		if !Running {
			return
		}

		net.SetupContainer(container, options.DesiredIP)
		
		container.SendXauth()

		container.Logger.Info("Container %s is ready", container.Name)

	}()

	err = cmd.Run()
	Running = false
	if err != nil {
		container.Logger.Error("Container failed: %s", err)
	}else{
		container.Logger.Info("Container %s exited", container.Name)
	}

	return err
}


func LaunchSimulator(workspace *Workspace) error {
	go StartVirgl()
	time.Sleep(time.Second)

	net, err := SetupNetwork()
	defer net.Destroy()
	if err != nil {
		HostLogger.Error("Failed to setup network")
		return err
	}

	machines := []MachineOptions{
		{
			Name: "cloversim",
			Workspace: nil,
			DesiredIP: 2,
		}, {
			Name: "clover0",
			Workspace: workspace,
			DesiredIP: 0,
		},
	}
		

	wg := sync.WaitGroup{}

	for _, machine := range machines {
		go func() {
			wg.Add(1)
			err = LaunchMachine(machine, net)
			wg.Done()
		}()
		time.Sleep(time.Second)
	}


	wg.Wait()

	return nil
}