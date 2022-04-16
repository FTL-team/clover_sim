package main

import (
	"os"
	"os/exec"
	"path"
	"syscall"
	"fmt"
)

func StartVirgl() {
	HostLogger.Info("Staring virgl")
	cmd := exec.Command(path.Join(LocateSetup(), "./virgl/virgl_test_server"), "--use-egl-surfaceless")
	cmd.Env = append(os.Environ(), fmt.Sprintf("LD_LIBRARY_PATH=%s", path.Join(LocateSetup(), "virgl")))
	cmd.SysProcAttr = &syscall.SysProcAttr{
		Setpgid: true,
	}
	uid, gid, err := GetUserID()
	if err != nil {
		HostLogger.Error("Failed to get sudo user id")
		panic(err)
	}

	cmd.SysProcAttr.Credential = &syscall.Credential{Uid: uid, Gid: gid}

	// cmd.Stderr = os.Stderr
	// cmd.Stdout = os.Stdout
	err = cmd.Run()
	if err != nil {
		HostLogger.Error("Failed to start virgl: %s", err)
		panic(err)
	}
}
