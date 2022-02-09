package main

import (
	"bytes"
	"fmt"
	"io/ioutil"
	"os"
	"os/exec"
	"path"
	"strconv"
	"strings"
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

func NewSimulatorServicePlugin(container *Container, mode string, launch bool) (*ContainerPlugin, error) {

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
	servicePath := path.Join(container.Path, "cloversim.service")
	err := ioutil.WriteFile(servicePath, []byte(service), 0644)
	if err != nil {
		return plugin, err
	}

	plugin.MountsRO = []string{
		servicePath + ":/etc/systemd/system/cloversim.service",
	}

	plugin.RunOnBoot = func(container *Container) error {
		if launch {
			container.Logger.Info("Starting simulator")
			return container.Systemctl("start", "cloversim.service")
		}
		return nil
	}

	return plugin, nil
}
