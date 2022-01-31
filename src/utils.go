package main

import (
	"os" 
	"os/exec"
	"fmt"
	"strconv"
)

func LocateSetup() string {
	ex, err := os.Getwd()
	if err != nil {
		HostLogger.Error("Could not determine setup location")
		panic(err)
	}
	return ex
}


func GetUserID() (uint32, uint32, error) {
	uid, err := strconv.ParseUint(os.Getenv("SUDO_UID"), 10, 32)
	if err != nil {
		return 0, 0, err
	}
	gid, err := strconv.ParseUint(os.Getenv("SUDO_GID"), 10, 32)
	if err != nil {
		return 0, 0, err
	}
	return uint32(uid), uint32(gid), nil
}

func LocateUserHome() string {
	user := os.Getenv("SUDO_USER")
	homePath := "/home/" + user

	if _, err := os.Stat(homePath); err != nil {
		HostLogger.Error("Could not find home directory of user, report this bug")
		panic(err)
	}

	return homePath
}


func ExecCommands(commands [][]string) (error) {
	for _, cmd := range commands {
		out, err := exec.Command(cmd[0], cmd[1:]...).CombinedOutput()
		if err != nil {
			return fmt.Errorf("Failed to run command:\nOutput: \n%s\nError: %s", out, err)
		}
	}
	return nil
}