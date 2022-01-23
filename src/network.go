package main

import (
	"fmt"
	"strconv"
	"path"
	"os"
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
	BridgeName      string
	AllocatedIPs map[int]bool
	HostsPath string
	HostsFile *os.File
}

func SetupNetwork() (*NetworkConfig, error) {
	HostLogger.Info("Setting up network")

	net := &NetworkConfig{
		BridgeName:      "cloversim",
		AllocatedIPs: map[int]bool{1: true},
		HostsPath: path.Join(LocateSetup(), "containers", "hosts"),
	}

	setupCommands := [][]string {
		{"ip", "link", "add", "name", net.BridgeName, "type", "bridge"},
		{"ip", "link", "set", net.BridgeName, "up"},
		{"ip", "addr", "add", "192.168.77.1/24", "dev", net.BridgeName},
		{"sysctl", "-w", "net.ipv4.ip_forward=1"},
		{"sysctl", "-w", "net.ipv6.conf.all.forwarding=1"},
		{"nft", "add", "table", "cloversim"},
		{"nft", "add", "chain", "cloversim", "nat", "{type nat hook postrouting priority srcnat; policy accept; }"},
		{"nft", "add", "rule", "cloversim", "nat", "ip", "saddr", "192.168.77.1/24", "oif", "!=", net.BridgeName, "masquerade"},
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
		{"nft", "flush", "table", "cloversim_nat"},
	});
}

func (net *NetworkConfig) GetNextIP(desired int) string {

	if desired <= 0 {
		desired = 10			
	}

	for net.AllocatedIPs[desired] == true {
		desired++
	}
	net.AllocatedIPs[desired] = true

	return "192.168.77." + strconv.Itoa(desired)
}


func (net *NetworkConfig) SetupContainer(container *Container, desiredIP int) error {
	ip := net.GetNextIP(desiredIP)

	setIpCommand := fmt.Sprintf("ip addr add %s/24 dev host0 && ip link set host0 up && ip route add default via 192.168.77.1", ip)
	cmd := container.Exec(ExecContainerOptions{
		Command: setIpCommand,
		ServiceOptions: map[string]string{
			"After": "network-online.target",
			"Wants": "network-online.target",
		},
		Description: "Network setup",
	})

	net.AddHosts(ip, container.Name)
	container.IPs = append(container.IPs, ip)
	
	container.Logger.Verbose("Container network ready: %s", ip)

	return cmd.Run()
}


