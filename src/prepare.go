package main

import (
	"fmt"
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
	fmt.Println("Base FS unpacked")

	return nil
}