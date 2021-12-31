package main

import (
	"os"
	"os/exec"
	"path"
	"syscall"
	"fmt"
)

func StartVirgl() {
	fmt.Println("Staring virgl")
	cmd := exec.Command(path.Join(LocateSetup(), "./virgl/virgl_test_server"))
	cmd.Env = append(os.Environ(), fmt.Sprintf("LD_LIBRARY_PATH=%s", path.Join(LocateSetup(), "virgl")))
	cmd.SysProcAttr = &syscall.SysProcAttr{}
	uid, gid, err := GetUserID()
	if err != nil {
		fmt.Println("Failed to get user id, before sudo")
		panic(err)
	}

	cmd.SysProcAttr.Credential = &syscall.Credential{Uid: uid, Gid: gid}

	// cmd.Stderr = os.Stderr
	// cmd.Stdout = os.Stdout
	err = cmd.Run()
	if err != nil {
		fmt.Println("Failed to start virgl")
		panic(err)
	}
}

func setup_virgl() {

}