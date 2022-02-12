package main

import (
	"encoding/xml"
	"fmt"
	"io/ioutil"
	"os"
	"path"
)

type packageXml struct {
	Name string `xml:"name"`
}

func readPackageName(taskPath string) (string, error) {
	content, err := ioutil.ReadFile(path.Join(taskPath, "package.xml"))
	if err != nil {
		return "", err
	}
	packag := packageXml{} 
	err = xml.Unmarshal([]byte(content), &packag)
	return packag.Name, err
}

func GetTaskRosLayer(taskPath string) (*RosLayer, error) {
	taskName, err := readPackageName(taskPath)
	if os.IsNotExist(err) {
		return nil, fmt.Errorf("task %s is not exist, or is not task", taskPath)
	} else if err != nil {
		return nil, err
	}
	


	return &RosLayer{
		LayerEntry: *CreateBaseFsEntry("task"),
		RosPackageName: taskName,
		RosPackagePath: taskPath,

		ParentLayers: []*OverlayEntry{
			CreateBaseFsEntry("base"),
			CreateBaseFsEntry("cloversim"),
		},
		ReportName: "task",
	}, nil
}