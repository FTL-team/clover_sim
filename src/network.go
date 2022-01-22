package main

import (
	"fmt"
	"strconv"
)

type NetworkConfig struct {
	BridgeName      string
	LastAllocatedIP int
}

func SetupNetwork() (*NetworkConfig, error) {
	fmt.Println("Setting up network")

	net := &NetworkConfig{
		BridgeName:      "cloversim",
		LastAllocatedIP: 1,
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

	return net, err
}

func (net *NetworkConfig) Destroy() error {
	fmt.Println("Destroying network")
	return ExecCommands([][]string{
		{"ip", "link", "del", "name", net.BridgeName},
		{"nft", "flush", "table", "cloversim_nat"},
	});
}

func (net *NetworkConfig) GetNextId() string {
	net.LastAllocatedIP++
	return "192.168.77." + strconv.Itoa(net.LastAllocatedIP)
}

func (net *NetworkConfig) GenerateContainerSetup() string {
	ip := net.GetNextId()

	return "ip addr add " + ip + "/24 dev host0 && ip link set host0 up && ip route add default via 192.168.77.1"
}
