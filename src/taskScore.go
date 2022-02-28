package main

import (
	"io/ioutil"
	"encoding/json"
	"fmt"
	"os"
	"github.com/FTL-team/clover_sim/src/color"
)

type TaskScorer struct {
	Path string	
}

func NewTaskScorePlugin(score *TaskScorer) (*ContainerPlugin, error) {
	plugin := &ContainerPlugin{
		Name: "taskScore",
	}

	score.Path = SharedContainerFile("task_score.json")
	ioutil.WriteFile(score.Path, []byte("{}"), 0666)
	os.Chmod(score.Path , 0666)
	

	plugin.MountsRW = []string{
		score.Path + ":/home/clover/task_score.json",
	}

	return plugin, nil
}

type ScoreEntry struct {
	Name string `json:"name"`
	MaxScore float64 `json:"max_score"`
	Score float64 `json:"score"`
	Failed bool `json:"failed"`
	Children []ScoreEntry `json:"children"`
}

type ScoreTableRow struct {
	Name string
	Score string
	MaxScore string
	Failed bool
}


func PrintScore(score *ScoreEntry, space string) []ScoreTableRow {
	arr := []ScoreTableRow{
		{
			Name: space + score.Name,
			Score: fmt.Sprintf("%.2f", score.Score),
			MaxScore: fmt.Sprintf("%.2f", score.MaxScore),
			Failed: score.Failed,
		},
	}
	for i := 0; i < len(score.Children); i++ {
		arr = append(arr, PrintScore(&score.Children[i], space + "  ")...)
	}
	return arr
}

func PrintTable(rows []ScoreTableRow) {
	nameMaxLen := 0
	scoreMaxLen := 0
	maxScoreMaxLen := 0
	for _, row := range rows {
		if nameMaxLen <  len(row.Name){
			nameMaxLen = len(row.Name)
		}

		if scoreMaxLen < len(row.Score) {
			scoreMaxLen = len(row.Score)
		}

		if maxScoreMaxLen < len(row.MaxScore) {
			maxScoreMaxLen = len(row.MaxScore)
		}
	}
	// fmt.Println(nameMaxLen, scoreMaxLen, maxScoreMaxLen)
	
	for _, row := range rows {
		// fmt.Println(nameMaxLen - len(row.Name) - len(row.Score))
		fmt.Printf("\r%*s  ", -nameMaxLen - 4, row.Name)
		if row.Failed {
			fmt.Printf(color.RED_ESC + "Failed" + color.RESET_ESC)
		}else{
			if row.Score == row.MaxScore {
				fmt.Print(color.GREEN_ESC)
			}else if row.Score == "0.00" {
				fmt.Print(color.GRAY_ESC)
			}
			fmt.Printf("%*s / ", scoreMaxLen, row.Score)
			fmt.Printf("%*s", maxScoreMaxLen, row.MaxScore)
			if row.Score == row.MaxScore || row.Score == "0.00" {
				fmt.Print(color.RESET_ESC)
			}
		}
		fmt.Println()
	}
}

func (sc TaskScorer) Display() {
	byteValue, _ := ioutil.ReadFile(sc.Path)
	var score ScoreEntry
	err := json.Unmarshal(byteValue, &score)
	if err != nil {
		HostLogger.Error("No score information found")
		return
	}

	PrintTable(PrintScore(&score, ""))
}