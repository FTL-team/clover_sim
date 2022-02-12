package main

import (
	"os"
	"os/exec"
	"io/ioutil"
	"encoding/xml"
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

func BuildCloversimLayer(buildFast bool) error {
	HostLogger.Info("Building cloversim layer, this may take a while...")

	layerPath := path.Join(LocateSetup(), "base_fs", "cloversim")
	buildContainerPath := path.Join(LocateSetup(), "containers", "__build_cloversim")
	if !buildFast {
		os.RemoveAll(layerPath)
	}
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

type CatkinPackageVersion struct {
	Version string `xml:"version"`
}

func getXMLCatkinPackageVersion(path string) (string, error) {
	f, err := os.Open(path)
	if err != nil {
		return "", err
	}
	defer f.Close()

	byteValue, err := ioutil.ReadAll(f)
	if err != nil {
		return "", err
	}
	var simVersion CatkinPackageVersion
	xml.Unmarshal(byteValue, &simVersion)
	return simVersion.Version, nil
}

func ShouldRebuildCloversimLayer() bool {
	simVersion, err := getXMLCatkinPackageVersion(path.Join(LocateSetup(), "sim", "cloversim",  "package.xml"))
	if err != nil {
		return true
	}

	layerVersion, err := getXMLCatkinPackageVersion(path.Join(LocateSetup(), "base_fs", "cloversim", "home/clover/catkin_ws/src", "cloversim", "package.xml"))
	if err != nil {
		return true
	}
	HostLogger.Verbose("Cloversim version: %s, layer version: %s", simVersion, layerVersion)
	return simVersion != layerVersion
}