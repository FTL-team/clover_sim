package main

import (
	"context"
	"os"
	"os/signal"
	"path"
	"sync"
	"syscall"
	"time"
)

type SimulatorContainerOptions struct {
	Name      string
	Workspace *Workspace
	DesiredIP int
	Mode      string
}

type SimulatorOptions struct {
	Workspace    *Workspace
	StartAtReady bool
	TaskPath 		 string
}

type SimulatorContainers struct {
	Containers map[string]*Container
	ExitWait   sync.WaitGroup
	ReadyWait  sync.WaitGroup
	mutex      sync.RWMutex
}

type Simulator struct {
	Network       *NetworkConfig
	Context       context.Context
	ContextCancel context.CancelFunc
	Containers    SimulatorContainers
	TaskPkgName   string
	TaskSeed      SeedControl
}

func (sc *SimulatorContainers) Get(name string) *Container {
	sc.mutex.RLock()
	defer sc.mutex.RUnlock()
	return sc.Containers[name]
}

func (sc *SimulatorContainers) Add(container *Container) {
	sc.mutex.Lock()
	defer sc.mutex.Unlock()
	sc.Containers[container.Name] = container
}

func (sc *SimulatorContainers) GetAllContainers() []*Container {
	sc.mutex.RLock()
	defer sc.mutex.RUnlock()

	containers := make([]*Container, 0, len(sc.Containers))
	for _, container := range sc.Containers {
		containers = append(containers, container)
	}
	return containers
}

func (sim *Simulator) LaunchContainer(options SimulatorContainerOptions) error {
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

	container.AddPluginCheckError(sim.Network.GetNetworkPlugin(container, options.DesiredIP))
	container.AddPluginCheckError(NewX11Plugin())
	container.AddPluginCheckError(NewSimulatorServicePlugin(container, options.Mode))
	container.AddPluginCheckError(NewReadyPlugin(&sim.Containers.ReadyWait))

	if options.Mode == "simulator" {
		container.AddPluginCheckError(NewTaskSeedPlugin(&sim.TaskSeed))
	}

	sim.Containers.Add(container)
	if container == nil {
		HostLogger.Error("Failed to add container: %s", err)
		return err
	}
	HostLogger.Info("Container created: %s", container.Name)
	return container.Run(sim.Context)
}

func LaunchSimulator(options SimulatorOptions) error {
	context, cancel := context.WithCancel(context.Background())

	sim := &Simulator{
		Context:       context,
		ContextCancel: cancel,
		Containers: SimulatorContainers{
			Containers: make(map[string]*Container),
			ExitWait:   sync.WaitGroup{},
		},
	}

	defer cancel()

	{
		cloversimLayer := GetCloversimLayer()

		if err := cloversimLayer.RebuildIfNeeded(); err != nil {
			HostLogger.Error("Failed to build cloversim layer: %s", err)
			return err
		}

		taskLayer, err := GetTaskRosLayer(path.Join(LocateSetup(), "tasks", options.TaskPath))
		if err != nil {
			return err
		}

		if err := taskLayer.RebuildIfNeeded(); err != nil {
			HostLogger.Error("Failed to build task layer: %s", err)
			return err
		}
		sim.TaskPkgName = taskLayer.RosPackageName
	}

	go StartVirgl()
	time.Sleep(time.Second)

	net, err := SetupNetwork()
	defer net.Destroy()
	if err != nil {
		HostLogger.Error("Failed to setup network")
		return err
	}

	sim.Network = net

	cotainersToLaunch := []SimulatorContainerOptions{
		{
			Name:      "cloversim",
			Workspace: nil,
			DesiredIP: 2,
			Mode:      "simulator",
		}, {
			Name:      "clover0",
			Workspace: options.Workspace,
			DesiredIP: 0,
			Mode:      "copter",
		},
	}

	for _, containerOptions := range cotainersToLaunch {
		sim.Containers.ExitWait.Add(1)
		go func() {
			err = sim.LaunchContainer(containerOptions)
			sim.Containers.ExitWait.Done()
		}()
		time.Sleep(time.Second / 2)
	}

	{
		c := make(chan os.Signal, 1)
		signal.Notify(c, os.Interrupt, syscall.SIGTERM)
		go func() {
			for range c {
				HostLogger.Info("Stopping simulator")
				sim.ContextCancel()
			}
		}()
	}

	if options.StartAtReady {
		go sim.StartSimulator()
	}

	go SimulatorController(sim)

	sim.Containers.ExitWait.Wait()
	HostLogger.Info("All containers stopped")

	return nil
}

func (sim *Simulator) AllSystemctl(options ...string) {
	for _, container := range sim.Containers.GetAllContainers() {
		container.Systemctl(options...)
	}
}

func (sim *Simulator) StartSimulator() {
	sim.Containers.ReadyWait.Wait()
	sim.AllSystemctl("start", "cloversim")
}

func (sim *Simulator) StopSimulator() {
	sim.Containers.ReadyWait.Wait()
	sim.AllSystemctl("stop", "cloversim")
}

func (sim *Simulator) RestartSimulator() {
	sim.Containers.ReadyWait.Wait()
	sim.AllSystemctl("stop", "cloversim")
	time.Sleep(time.Second)
	sim.AllSystemctl("start", "cloversim")
}
