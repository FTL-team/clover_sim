package main

import (
	"os"
	"os/exec"
	"fmt"
	"path"
	"hash/crc32"
	"io/ioutil"
)

type RosLayer struct {
	LayerEntry OverlayEntry
	RosPackageName string
	RosPackagePath string
	ParentLayers []*OverlayEntry

	ReportName string
} 



func (rl *RosLayer) ShouldRebuild() (bool) {
	crc32q := crc32.MakeTable(0x82f63b78)

	packageContent, err := ioutil.ReadFile(path.Join(rl.RosPackagePath, "package.xml"))
	if err != nil {
		return true
	}
	packageCrc := crc32.Checksum(packageContent, crc32q)
	
	
	layerContent, err := ioutil.ReadFile(path.Join(rl.LayerEntry.Path, "home/clover/catkin_ws/src", rl.RosPackageName, "package.xml"))
	if err != nil {
		return true
	}
	layerCrc := crc32.Checksum(layerContent, crc32q)

	HostLogger.Verbose("%s package crc %x, layer crc %x", rl.ReportName, packageCrc, layerCrc)
	
	return layerCrc != packageCrc
} 

func (rl *RosLayer) RebuildLayer(fastBuild bool) error {
	HostLogger.Info("Building %s layer, this may take a while...", rl.ReportName)

	buildContainerPath := path.Join(LocateSetup(), "containers", "__build_layer")
	if !fastBuild {
		os.RemoveAll(rl.LayerEntry.Path)
	}
	os.MkdirAll(rl.LayerEntry.Path, os.ModePerm)

	overlay := &Overlay{
		Layers: rl.ParentLayers,
		OverlayLayer: &rl.LayerEntry,
		
		Path: buildContainerPath,
		Logger: HostLogger,
	}
	defer overlay.Destroy()

	if err:= overlay.Mount(); err != nil {
		return err
	}

	pkgLayerPath := path.Join("/home/clover/catkin_ws/src", rl.RosPackageName)

	binder := rl.RosPackagePath + ":/build_rospkg"
	buildCommand := fmt.Sprintf("rm %[1]s; cp -r /build_rospkg %[1]s && chown -R clover %[1]s", pkgLayerPath)
	buildCommand += "&& source /etc/profile && source ~/.bashrc"
	buildCommand += fmt.Sprintf("&& cd /home/clover/catkin_ws/ && catkin_make %s", rl.RosPackageName)
	buildCommand += "&& history -c"
	HostLogger.Info(buildCommand)
	cmd := exec.Command("systemd-nspawn", "-u", "clover", "--bind-ro", binder, "-D", overlay.Path, "/bin/bash", "-i", "-c", buildCommand)

	err := cmd.Run()
	if err != nil {
		HostLogger.Error("Build failed, %s", err)
	}
	return err
}

func (rl *RosLayer) RebuildIfNeeded() error {
	if rl.ShouldRebuild() {
		return rl.RebuildLayer(false)
	}
	return nil
}