package main

import (
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

func (rl *RosLayer) RebuildLayer(fastBuild bool) error {
	pkgLayerPath := path.Join("/home/clover/catkin_ws/src", rl.RosPackageName)
	
	buildCommand := fmt.Sprintf("rm %[1]s; cp -r /build_rospkg %[1]s && chown -R clover %[1]s", pkgLayerPath)
	buildCommand += "&& source /etc/profile && source ~/.bashrc"
	buildCommand += fmt.Sprintf("&& cd /home/clover/catkin_ws/ && catkin_make %s", rl.RosPackageName)
	buildCommand += "&& history -c"
	
	buildLayer := &BuildLayer{
		LayerEntry: &rl.LayerEntry,
		ParentLayers: rl.ParentLayers,
		ReportName: rl.ReportName,
		BuildCommand: buildCommand,

		Binds: []string{
			rl.RosPackagePath + ":/build_rospkg",
		},
	}

	return buildLayer.RebuildLayer(fastBuild, false)
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


func (rl *RosLayer) RebuildIfNeeded() error {
	if rl.ShouldRebuild() {
		return rl.RebuildLayer(false)
	}
	return nil
}