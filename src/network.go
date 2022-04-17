package main

import (
	"fmt"
	"os"
	"strconv"
)

const hostsDefault = `# This file is managed by cloversim
# Default
127.0.0.1  localhost
::1        localhost ip6-localhost ip6-loopback
ff02::1    ip6-allnodes
ff02::2    ip6-allrouters

# Containers
`

type NetworkConfig struct {
	BridgeName   string
	AllocatedIPs map[int]bool
	HostsPath    string
	HostsFile    *os.File
}

func SetupNetwork() (*NetworkConfig, error) {
	HostLogger.Info("Setting up network")

	net := &NetworkConfig{
		BridgeName:   "cloversim",
		AllocatedIPs: map[int]bool{1: true},
		HostsPath:    SharedContainerFile("hosts"),
	}

	setupCommands := [][]string{
		{"ip", "link", "add", "name", net.BridgeName, "type", "bridge"},
		{"ip", "link", "set", net.BridgeName, "up"},
		{"ip", "addr", "add", "192.168.77.1/24", "dev", net.BridgeName},
		{"sysctl", "-w", "net.ipv4.ip_forward=1"},
		{"sysctl", "-w", "net.ipv6.conf.all.forwarding=1"},
		{"iptables", "-t", "nat", "-A", "POSTROUTING", "-s", "192.168.77.1/24", "!", "-o", net.BridgeName, "-j", "MASQUERADE"},
	}

	err := ExecCommands(setupCommands)
	if err != nil {
		return net, err
	}

	os.Remove(net.HostsPath)
	f, err := os.OpenFile(net.HostsPath, os.O_RDWR|os.O_CREATE, 0644)

	net.HostsFile = f
	if err != nil {
		return net, err
	}

	_, err = net.HostsFile.WriteString(hostsDefault)
	if err != nil {
		return net, err
	}

	net.AddHosts("192.168.77.1", "host")

	return net, err
}

func (net *NetworkConfig) AddHosts(ip string, name string) error {
	_, err := net.HostsFile.WriteString(fmt.Sprintf("%s\t%s\n", ip, name))
	if err != nil {
		return err
	}
	net.HostsFile.Sync()
	return nil
}

func (net *NetworkConfig) Destroy() error {
	HostLogger.Info("Destroying network")
	return ExecCommands([][]string{
		{"ip", "link", "del", "name", net.BridgeName},
	})
}

func (net *NetworkConfig) GetNextIP(desired int) string {

	if desired <= 0 {
		desired = 10
	}

	for net.AllocatedIPs[desired] {
		desired++
	}
	net.AllocatedIPs[desired] = true

	return "192.168.77." + strconv.Itoa(desired)
}

func (net *NetworkConfig) GetNetworkPlugin(container *Container, desiredIP int) (*ContainerPlugin, error) {
	plugin := &ContainerPlugin{
		Name: "network",
	}

	ip := net.GetNextIP(desiredIP)
	container.IPs = append(container.IPs, ip)
	net.AddHosts(ip, container.Name)

	plugin.MountsRO = []string{
		SharedContainerFile("hosts") + ":/etc/hosts",
		container.ContainerFile("hostname") + ":/etc/hostname",
	}

	plugin.LauncherArguments = []string{
		"--network-bridge="+net.BridgeName,
	}

	plugin.RunOnBoot = func(container *Container) error {

		setIpCommand := fmt.Sprintf("ip addr add %s/24 dev host0 && ip link set host0 up && ip route add default via 192.168.77.1", ip)
		cmd := container.Exec(ExecContainerOptions{
			Command: setIpCommand,
			ServiceOptions: map[string]string{
				"After": "network-online.target",
				"Wants": "network-online.target",
			},
			Description: "Network setup",
		})

		out, err := cmd.CombinedOutput()
		if err == nil {
			container.Logger.Verbose("Container network ready: %s", ip)
		}else{
			container.Logger.Error("Network setup failed: %s, output: \n%s", err, out)
		}

		return err
	}

	return plugin, nil
}
