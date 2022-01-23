package main

import (
	"fmt"
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
		return err
	}
	defer container.Destroy()

	status.WLock()
	status.Machines[options.Name] = MachineStatus{
		Name: options.Name,
		Status: "launching",
	}
	status.WUnlock()

	cmd, err := container.GetLauncher(net)
	if err != nil {
		return err
	}

	go func() {
		time.Sleep(time.Millisecond * 800) // Wait 1.5 second for systemd to start
		
		net.SetupContainer(container, options.DesiredIP)
		
		container.SendXauth()

		fmt.Printf("Container %s is ready\n", container.Name)

		status.WLock()
		status.Machines[options.Name] = MachineStatus{
			Name: options.Name,
			Status: "running",
			IP: container.IPs[0],
		}
		status.WUnlock()
	}()

	err = cmd.Run()

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
		fmt.Println("Failed to setup network")
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
			if err != nil {
				fmt.Printf("Error in container:")
				fmt.Println(err)
			}
			wg.Done()
		}()
		time.Sleep(time.Second)
	}

	wg.Wait()

	return nil
}