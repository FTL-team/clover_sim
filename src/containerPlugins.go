package main

import (
	"bytes"
	"fmt"
	"io/ioutil"
	"os"
	"os/exec"
	"strconv"
	"strings"
	"sync"
	"math/rand"
)

func NewX11Plugin() (*ContainerPlugin, error) {

	plugin := &ContainerPlugin{
		Name: "X11",
	}

	if len(os.Getenv("DISPLAY")) < 2 {
		return plugin, fmt.Errorf("x11 display not found, DISPLAY environment variable is not set")
	}

	plugin.MountsRO = []string{
		fmt.Sprintf("/tmp/.X11-unix/X%s:/tmp/.X11-unix/X0", os.Getenv("DISPLAY")[1:]),
		"/tmp/.virgl_test",
	}

	plugin.RunOnBoot = func(container *Container) error {
		xauthCmd := exec.Command("xauth", "nextract", "-", os.Getenv("DISPLAY"))
		out, err := xauthCmd.Output()
		if err != nil {
			return err
		}
		l := strings.Split(string(out), "\n")
		for i := range l {

			parts := strings.Fields(string(l[i]))
			if len(parts) > 0 {
				parts[0] = "ffff"

				dd, err := strconv.Atoi(parts[len(parts)-5])
				if err != nil {
					fmt.Printf("ERROR, xauth, report this to cloversim developers: %s\n", out)
					panic(err)
				}

				di, err := strconv.Atoi(os.Getenv("DISPLAY")[1:])
				if err != nil {
					fmt.Printf("ERROR, xauth, report this to cloversim developers: %s\n", out)
					panic(err)
				}

				parts[len(parts)-5] = strconv.Itoa(dd - di)
			}
			l[i] = strings.Join(parts, " ")
		}

		out = []byte(strings.Join(l, "\n"))

		containerCmd := container.Exec(ExecContainerOptions{
			Command:     "xauth nmerge - ",
			Description: "Setup xauth",
			Uid:         1000,
			ServiceOptions: map[string]string{
				"After": "multi-user.target",
				"Wants": "multi-user.target",
			},
		})
		containerCmd.Stdin = bytes.NewReader(out)
		return containerCmd.Run()
	}

	return plugin, nil
}

func NewSimulatorServicePlugin(container *Container, mode string) (*ContainerPlugin, error) {

	plugin := &ContainerPlugin{
		Name: "cloversimService",
	}

	service := fmt.Sprintf(`
[Unit]
Description=Start simulator
After=roscore.target
Wants=roscore.target

[Service]
EnvironmentFile=/etc/environment
RemainAfterExit=yes
User=1000
Group=1000
ExecStart="/bin/bash" "-ic" ". /etc/profile; . ~/.bashrc; mkfifo /tmp/cloversim_inp; cat /tmp/cloversim_inp | roslaunch --wait cloversim %s.launch"
`, mode)
	servicePath := container.ContainerFile("cloversim.service")
	err := ioutil.WriteFile(servicePath, []byte(service), 0644)
	if err != nil {
		return plugin, err
	}

	plugin.MountsRO = []string{
		servicePath + ":/etc/systemd/system/cloversim.service",
	}

	ioutil.WriteFile(SharedContainerFile("aruco_map.txt"), []byte{}, 0666)
	arucoMap := SharedContainerFile("aruco_map.txt") + ":/home/clover/catkin_ws/src/clover/aruco_pose/map/map.txt";
	if mode == "simulator" {
		plugin.MountsRW = []string{
			arucoMap,
		}
	}else{
		plugin.MountsRO = append(plugin.MountsRO, arucoMap)
	}

	return plugin, nil
}


func NewReadyPlugin(readyWait *sync.WaitGroup) (*ContainerPlugin, error) {
	readyWait.Add(1)

	plugin := &ContainerPlugin{
		Name: "ready",
	}

	plugin.RunOnBoot = func(container *Container) error {
		readyWait.Done()
		return nil
	}

	return plugin, nil
}


type SeedControl struct {
	setSeed func(seed string)
	Seed string
}

func (seed *SeedControl) NewSeed() {
	seed.Seed = strconv.Itoa(rand.Int() & 0xFFFFFFFF)
	seed.setSeed(seed.Seed)
}

func (seed *SeedControl) SetSeed(seedS string) {
	seed.Seed = seedS
	seed.setSeed(seedS)
}

func NewTaskSeedPlugin(seed *SeedControl) (*ContainerPlugin, error) {
	plugin := &ContainerPlugin{
		Name: "taskSeed",
	}

	seedFilePath := SharedContainerFile("task_seed")
	seed.setSeed = func(seed string) {
		ioutil.WriteFile(seedFilePath, []byte(seed), 0666)
	}

	seed.NewSeed()

	plugin.MountsRO = []string{
		seedFilePath + ":/home/clover/task_seed",
	}

	return plugin, nil
}