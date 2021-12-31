package main

import (
	"os" 
	"fmt"
	"strconv"
)

func LocateSetup() string {
	ex, err := os.Getwd()
	if err != nil {
		fmt.Println("Could not determine setup location")
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
		panic(fmt.Errorf("Could not find home directory for user %s, report this bug", user))
	}

	return homePath
}