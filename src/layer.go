package main

import (
	"os"
	"os/exec"
	"path"
)

type BuildLayer struct {
	LayerEntry *OverlayEntry
	ParentLayers []*OverlayEntry

	ReportName string

	BuildCommand string
	Binds []string
} 

func (layer *BuildLayer) RebuildLayer(useAlreadyBuilt bool, reportAll bool) error {
	HostLogger.Info("Building %s layer, this may take a while...", layer.ReportName)

	buildContainerPath := path.Join(LocateSetup(), "containers", "__build_layer")
	if !useAlreadyBuilt {
		os.RemoveAll(layer.LayerEntry.Path)
	}
	os.MkdirAll(layer.LayerEntry.Path, os.ModePerm)

	overlay := &Overlay{
		Layers: layer.ParentLayers,
		OverlayLayer: layer.LayerEntry,
		
		Path: buildContainerPath,
		Logger: HostLogger,
	}
	defer overlay.Destroy()

	if err:= overlay.Mount(); err != nil {
		return err
	}

	nspawnArgs := []string{"-u", "clover"}
	for _, bindPath := range layer.Binds {
		nspawnArgs = append(nspawnArgs, "--bind-ro", bindPath)
	}

	nspawnArgs = append(nspawnArgs, "-D", overlay.Path, "/bin/bash", "-i", "-c", layer.BuildCommand)


	cmd := exec.Command("systemd-nspawn", nspawnArgs...)
	var err error
	var out []byte
	if reportAll {
		cmd.Stdout = os.Stdout
		cmd.Stderr = os.Stderr
		err = cmd.Run()
	}else{
		out, err = cmd.CombinedOutput()
	}
	
	if err != nil {
		HostLogger.Error("Build failed, %s", err)
		
		if !reportAll{
			HostLogger.Info("Build output: %s", string(out))
		}
	}
	return err
}