package main

import (
	"os"
	"time"
)



func LaunchSimulator(workspace *Workspace) error {
	go StartVirgl()
	time.Sleep(time.Second)
	container, err := CreateContainer("base", workspace)
	if err != nil {
		return err
	}
	defer DestroyContainer(container)

	cmd, err := GetContainerLauncher(container)
	if err != nil {
		return err
	}

	cmd.Stderr = os.Stderr
	cmd.Stdout = os.Stdout
	cmd.Stdin = os.Stdin

	return cmd.Run()
}