package main

import (
	"os"
	"os/exec"
	"path"
)

func UnpackFS(name string) error {
	
	basefs := path.Join(LocateSetup(), "base_fs")
	os.RemoveAll(path.Join(basefs, name))
	
	file := path.Join(basefs, name + ".tar.gz")

	if _, err := os.Stat(file); err != nil {
		return err
	}

	cmd := exec.Command("tar", "-xvf", file)
	cmd.Dir = basefs
	cmd.Stdout = os.Stdout
	cmd.Stderr = os.Stderr

	err := cmd.Run()
	if err != nil { 
		return err
	}

	return nil
}

func Prepare() error {

	if err := UnpackFS("base"); err != nil {
		return err
	}
	HostLogger.Info("Base FS unpacked")

	return nil
}

func GetCloversimLayer() (*RosLayer) {
	return &RosLayer{
		LayerEntry: *CreateBaseFsEntry("cloversim"),
		RosPackageName: "cloversim",
		RosPackagePath: path.Join(LocateSetup(), "sim", "cloversim"),
		ParentLayers: []*OverlayEntry{
			CreateBaseFsEntry("base"),
		},

		ReportName: "cloversim",
	}
}