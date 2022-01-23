package main

import (
	"sync"
	"time"
)


type MachineStatus struct {
	Name string
	IP string
	Status string
}

type MachineOptions struct {
	Name string
	Workspace *Workspace
	Network *NetworkConfig
	DesiredIP int
}

type SimulatorStatus struct {
	Machines map[string]MachineStatus
	Locked sync.RWMutex
}

func (s *SimulatorStatus) WLock() {
	s.Locked.Lock()
}

func (s *SimulatorStatus) RLock() {
	s.Locked.RLock()
}

func (s *SimulatorStatus) WUnlock() {
	s.Locked.Unlock()
}

func (s *SimulatorStatus) RUnlock() {
	s.Locked.RUnlock()
}


func LaunchMachine(options MachineOptions, net *NetworkConfig, status *SimulatorStatus) (error) {
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

	status.WLock()
	status.Machines[options.Name] = MachineStatus{
		Name: options.Name,
		Status: "launching",
	}
	status.WUnlock()

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

		status.WLock()
		status.Machines[options.Name] = MachineStatus{
			Name: options.Name,
			Status: "running",
			IP: container.IPs[0],
		}
		status.WUnlock()
	}()

	err = cmd.Run()
	Running = false
	if err != nil {
		container.Logger.Error("Container failed: %s", err)
	}else{
		container.Logger.Info("Container %s exited", container.Name)
	}

	status.WLock()
	status.Machines[options.Name] = MachineStatus{
		Name: options.Name,
		Status: "stopped",
		IP: "",
	}
	status.WUnlock()

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

	status := &SimulatorStatus{
		Machines: make(map[string]MachineStatus),
	}

	wg := sync.WaitGroup{}

	for _, machine := range machines {
		go func() {
			wg.Add(1)
			err = LaunchMachine(machine, net, status)
			wg.Done()
		}()
		time.Sleep(time.Second)
	}


	wg.Wait()

	return nil
}