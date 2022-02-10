package main

import (
	"os"
	"fmt"
	"syscall"
	"path"
	"path/filepath"
	"time"
)

type OverlayEntry struct {
	Path string
	OverlayWork string
	Destroy func() 
}

type Overlay struct {
	Layers []*OverlayEntry
	OverlayLayer *OverlayEntry

	Path string
	Logger *Logger
}

func (ov *Overlay) Mount() error {
	lowerDir := ""
	for i, base := range ov.Layers {
		if i != 0 {
			lowerDir += ":"
		}
		base_real, err := filepath.EvalSymlinks(base.Path)
		if err != nil {
			return err
		}
		lowerDir += base_real
	}

	upperDir := ov.OverlayLayer.Path
	workDir := ov.OverlayLayer.OverlayWork
	targetDir := ov.Path
	
	os.Mkdir(workDir, os.ModePerm)
	os.MkdirAll(targetDir, os.ModePerm)

	options := fmt.Sprintf("lowerdir=%s,upperdir=%s,workdir=%s", lowerDir, upperDir, workDir)

	ov.Logger.Verbose("Mounting overlay rootfs: mount -t overlay overlay %s %s", targetDir, options)

	return syscall.Mount("overlay", targetDir, "overlay", 0, options)
}

func (ov *Overlay) Destroy() {
	for i := 0; i < 3; i++ {
		syscall.Unmount(ov.Path, 0)
		time.Sleep(time.Second / 2)
	}
	os.Remove(ov.OverlayLayer.OverlayWork)

	for _, entry := range ov.Layers {
		if entry.Destroy != nil {
			entry.Destroy()
		}
	}
	if ov.OverlayLayer.Destroy != nil {
		ov.OverlayLayer.Destroy()
	}
}

func CreateTmpFsEntry(targetDir string) (*OverlayEntry, error) {
	entry := &OverlayEntry{
		Path: path.Join(targetDir, "fs"),
		OverlayWork: path.Join(targetDir, ".overlay_work"),
		Destroy: func() {
			for i := 0; i < 3; i++ {
				syscall.Unmount(targetDir, 0)
				time.Sleep(time.Second / 3)
			}
		},
	}

	os.MkdirAll(targetDir, os.ModePerm)
	err := syscall.Mount("tmpfs", targetDir, "tmpfs", 0, "")
	if err != nil {
		return entry, err
	}
	
	err = os.MkdirAll(path.Join(targetDir, "fs"), os.ModePerm)
	return entry, err
}

func CreateBaseFsEntry(name string) (*OverlayEntry) {
	entry := &OverlayEntry{
		Path: path.Join(LocateSetup(), "base_fs", name),
		OverlayWork: path.Join(LocateSetup(), "base_fs", ".overlay_work_" + name),
	}
	return entry
}