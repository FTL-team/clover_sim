package main

import (
	"fmt"
	"os"
	"time"
)



func LaunchSimulator(workspace *Workspace) error {
	go StartVirgl()
	time.Sleep(time.Second)

	net, err := SetupNetwork()
	defer net.Destroy()
	if err != nil {
		fmt.Println("Failed to setup network")
		return err
	}

	container, err := CreateContainer("cloversim", workspace)
	if err != nil {
		return err
	}
	defer container.Destroy()

	cmd, err := container.GetLauncher(net)
	if err != nil {
		return err
	}

	cmd.Stderr = os.Stderr
	cmd.Stdout = os.Stdout
	cmd.Stdin = os.Stdin

	go func() {
		time.Sleep(time.Millisecond * 800) // Wait 1.5 second for systemd to start
		
		container.SetupNetwork(net)
		container.SendXauth()
	}()

	return cmd.Run()
}