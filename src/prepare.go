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

func BuildCloversimLayer() error {
	HostLogger.Info("Building cloversim layer, this may take a while...")

	layerPath := path.Join(LocateSetup(), "base_fs", "cloversim")
	buildContainerPath := path.Join(LocateSetup(), "containers", "__build_cloversim")
	os.RemoveAll(layerPath)
	os.MkdirAll(layerPath, os.ModePerm)

	overlay := &Overlay{
		Layers: []*OverlayEntry{
			CreateBaseFsEntry("base"),
		},

		OverlayLayer: CreateBaseFsEntry("cloversim"),
		
		Path: buildContainerPath,
		Logger: HostLogger,
	}
	defer overlay.Destroy()

	if err:= overlay.Mount(); err != nil {
		return err
	}

	binder := path.Join(LocateSetup(), "sim") + ":/sim"

	cmd := exec.Command("systemd-nspawn", "-u", "clover", "--bind-ro", binder, "-D", overlay.Path, "/bin/bash", "-i", "/sim/build.sh")

	err := cmd.Run()
	if err != nil {
		HostLogger.Error("Build failed, %s", err)
	}
	return err
}