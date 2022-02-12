package main

import (
	"os"
	"os/signal"
	"sync"
	"syscall"
	"time"
)

type MachineCommand struct {
	Command string
}

type SimulatorCommand struct {
	Command string
	MachineCommand MachineCommand
}

type MachineOptions struct {
	Name      string
	Workspace *Workspace
	Network   *NetworkConfig
	DesiredIP int
	Mode      string

	Commander chan MachineCommand
}

type Simulator struct {
	net                   *NetworkConfig
	stopSignal            *sync.Cond
	startSimulatorAtStart bool
}

type SimulatorOptions struct {
	Workspace *Workspace
	NoStart   bool
}

func LaunchContainerSim(container *Container, mode string) {
	container.Logger.Info("Launching container simulator node: %s", mode)

	cmd := container.Exec(ExecContainerOptions{
		Command:     ". /etc/profile; . ~/.bashrc; roslaunch --wait cloversim " + mode + ".launch",
		Description: "Start simulator",
		Uid:         1000,
		Gid:         1000,
		ServiceOptions: map[string]string{
			"After":           "roscore.target",
			"Wants":           "roscore.target",
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

func LaunchMachine(options MachineOptions, sim *Simulator) error {
	container, err := CreateContainer(options.Name, options.Workspace)
	if err != nil {
		if container != nil {
			container.Logger.Error("Failed to create container: %s", err)
		} else {
			HostLogger.Error("Failed to create container: %s", err)
		}

		return err
	}
	defer container.Destroy()

	container.AddPluginCheckError(sim.net.GetNetworkPlugin(container, options.DesiredIP))
	container.AddPluginCheckError(NewX11Plugin())
	container.AddPluginCheckError(NewSimulatorServicePlugin(container, options.Mode, sim.startSimulatorAtStart))

	go func() {
		for {
			command := <-options.Commander
			switch command.Command {
			case "simulator_start":
				container.Logger.Info("Starting simulator")
				container.Systemctl("start", "cloversim.service")
			case "simulator_stop":
				container.Logger.Info("Stopping simulator")
				container.Systemctl("stop", "cloversim.service")
			}
		}
	}()

	return container.Run(sim.stopSignal)
}

func LaunchSimulator(options SimulatorOptions) error {
	if ShouldRebuildCloversimLayer() {
		if err := BuildCloversimLayer(false); err != nil {
			HostLogger.Error("Failed to build cloversim layer: %s", err)
			return err
		}
	}
	
	go StartVirgl()
	time.Sleep(time.Second)

	net, err := SetupNetwork()
	defer net.Destroy()
	if err != nil {
		HostLogger.Error("Failed to setup network")
		return err
	}

	simulator := &Simulator{
		net:                   net,
		stopSignal:            sync.NewCond(&sync.Mutex{}),
		startSimulatorAtStart: !options.NoStart,
	}

	machines := []MachineOptions{
		{
			Name:      "cloversim",
			Workspace: nil,
			DesiredIP: 2,
			Mode:      "simulator",
			Commander: make(chan MachineCommand),
		}, {
			Name:      "clover0",
			Workspace: options.Workspace,
			DesiredIP: 0,
			Mode:      "copter",
			Commander: make(chan MachineCommand),
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
	promptExit := make(chan bool, 1)
	simulatorCommands := make(chan SimulatorCommand, 1)

	go func() {
		for {
			cmd := <-simulatorCommands
			if cmd.Command == "machine" {
				machineCommand := cmd.MachineCommand
				for _, machine := range machines {
					machine.Commander <- machineCommand
				}
			}
		}
	}()

	signal.Notify(c, os.Interrupt, syscall.SIGTERM)
	go func() {
		for range c {
			HostLogger.Info("Stopping simulator")
			promptExit <- true
			simulator.stopSignal.Broadcast()
		}
	}()

	go SimulatorController(c, promptExit, simulatorCommands)

	wg.Wait()
	promptExit <- true
	HostLogger.Info("All containers stopped")

	return nil
}
