package main

import (
	"os"
	"os/signal"
	"sync"
	"syscall"
	"time"
)


type MachineOptions struct {
	Name string
	Workspace *Workspace
	Network *NetworkConfig
	DesiredIP int
	Mode string
}

type Simulator struct {
	net *NetworkConfig
	stopSignal *sync.Cond
}

func LaunchContainerSim(container *Container, mode string) {
	container.Logger.Info("Launching container simulator node: %s", mode)

	cmd := container.Exec(ExecContainerOptions{
		Command: ". /etc/profile; . ~/.bashrc; roslaunch cloversim " + mode + ".launch",
		Description: "Start simulator",
		Uid: 1000,
		Gid: 1000,
		ServiceOptions: map[string]string{
			"After": "multi-user.target",
			"Wants": "multi-user.target",
			"EnvironmentFile": "/etc/environment",
		},
		Unit: "cloversim.service",
	})
	cmd.StdinPipe()
	// cmd.Stderr = os.Stderr
	// cmd.Stdout = os.Stdout
	go func() {
		cmd.Run()
		container.Logger.Info("Container simulator node exited")
	}()
}


func LaunchMachine(options MachineOptions, sim *Simulator) (error) {
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

	cmd, err := container.GetLauncher(sim.net)
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

		sim.net.SetupContainer(container, options.DesiredIP)
		
		container.SendXauth()

		container.Logger.Info("Container %s is ready", container.Name)

		LaunchContainerSim(container, options.Mode)
	}()

	go func() {
		sim.stopSignal.L.Lock()
		defer sim.stopSignal.L.Unlock()
		sim.stopSignal.Wait()
		if Running {
			Running = false
			container.Logger.Info("Stopping container %s", container.Name)
			container.Poweroff()
		}
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

	simulator := &Simulator{
		net: net,
		stopSignal: sync.NewCond(&sync.Mutex{}),
	}

	machines := []MachineOptions{
		{
			Name: "cloversim",
			Workspace: nil,
			DesiredIP: 2,
			Mode: "simulator",
		}, {
			Name: "clover0",
			Workspace: workspace,
			DesiredIP: 0,
			Mode: "copter",
		},
	}
		
	wg := sync.WaitGroup{}

	for _, machine := range machines {
		wg.Add(1)
		go func() {
			err = LaunchMachine(machine, simulator)
			wg.Done()
		}()
		time.Sleep(time.Second)
	}

	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt, syscall.SIGTERM)
	go func() {
		for range c {
			HostLogger.Info("Stopping simulator")
			simulator.stopSignal.Broadcast()
		}
	}()


	wg.Wait()
	HostLogger.Info("All containers stopped")

	return nil
}