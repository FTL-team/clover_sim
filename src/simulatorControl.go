package main

import (
	"fmt"
	"os"
	"strings"

	"golang.org/x/term"
)

func ProcessCommand(command string, sim *Simulator) {
  if len(command) == 0 {
    return
  } 

  parts := strings.Split(command, " ")
  if len(parts) == 0 {
    return
  }

  switch parts[0] {
  case "help":
    fmt.Println("\r")
    fmt.Println("\rAvailable commands:")
    fmt.Println("\r  help     - show this help")
    fmt.Println("\r  exit     - shutdowns cloversim")
    fmt.Println("\r  start    - start simulator (gazebo, clover, etc.)")
    fmt.Println("\r  stop     - stop simulator (gazebo, clover, etc.)")
    fmt.Println("\r  restart  - restart the simulator (gazebo, clover, etc.)")
    fmt.Println("\r  rget     - get current randomzation")
    fmt.Println("\r  rset     - set randomzation to value from second argument")
    fmt.Println("\r  rnew     - generate new random randomzation")
    fmt.Println("\r  score    - show current task scoring")
    fmt.Println("\r  run      - run user program (runs /home/clover/run.sh inside clover0 container)")

  case "exit":
    sim.ContextCancel()
  case "start":
    sim.StartSimulator()

  case "stop":
    sim.StopSimulator()

  case "restart":
    sim.RestartSimulator()

  case "rget":
    fmt.Println("\rRandomization: " + sim.TaskSeed.Seed)
  
  case "rset":
    if len(parts) < 2 {
      HostLogger.Error("Not enough arguments for randomization command, check help")
      return
    }
    sim.TaskSeed.SetSeed(parts[1])
    HostLogger.Info("\rNew randomization: " + sim.TaskSeed.Seed)
  
  case "rnew":
    sim.TaskSeed.NewSeed()
    fmt.Println("\rNew randomization: " + sim.TaskSeed.Seed)

  case "score":
    sim.TaskScore.Display()

  case "run":
    sim.RunUser()

  default:
    HostLogger.Error("Unknown command: %s, check help", parts[0])
  }

}

var oldState *term.State

func make_raw() {
	if IsTTY() {
    oldState, _ = term.MakeRaw(int(os.Stdin.Fd()))
  }
}

func make_normal() {
  if IsTTY() {
    if oldState != nil {
      term.Restore(int(os.Stdin.Fd()), oldState)
    }
  }
}

func SimulatorController(sim *Simulator) {
  make_raw()
  defer make_normal()

	inp := make(chan byte)
  read := make(chan bool)
  hist := []string{}
  
	go func() {
		b := make([]byte, 1)
		for {
      <-read
			_, err := os.Stdin.Read(b)
			if err != nil {
				HostLogger.Error("%s", err)
				return
			}
			inp <- b[0]
		}
	}()
  
  input := ""
  pos := 0
  
  hist_save := ""
  hist_pos := 0

  mode := 0
  SetPrompt(">>> ")

mainLoop:
	for {
    read <- true
		select {
		case <-sim.Context.Done():
			break mainLoop

		case ch := <-inp:
      if mode == 0 {
        if ch >= 32 && ch <= 126 {
          input = input[:pos] + string(ch) + input[pos:]
          pos++
        } else if ch == 10 || ch == 13 {
          fmt.Println("\r>>> " + input)
          SetPrompt("")
          if len(hist) == 0 || hist[len(hist)-1] != input {
            hist = append(hist, input)
          }
          make_normal()
          ProcessCommand(input, sim)
          make_raw()
          pos = 0
          input = ""
          hist_pos = 0
          hist_save = ""
        } else if ch == 3 || ch == 4 {
          sim.Interrupt()
        } else if ch == 27 {
          mode = 1
        } else if ch == 127 {
          if pos > 0 {
            input = input[:pos-1] + input[pos:]
            pos--
          }
        }
      } else if mode == 1 {
        if ch == 91 {
          mode = 2
        } else {
          mode = 0
        }
      } else if mode == 2 {
        
        if ch == 68 {
          if pos > 0 {
            pos--
          }
        }else if ch == 67 {
          if pos < len(input) {
            pos ++
          }
        }else if ch == 66 {
          if hist_pos > 0 {
            hist_pos--
          }
          if hist_pos == 0 {
            input = hist_save
          }else{
            input = hist[len(hist)-hist_pos]
          }
          pos = len(input)
        } else if ch == 65 {
          if hist_pos == 0 {
            hist_save = input
          }

          if hist_pos < len(hist) {
            hist_pos++
          }
          if hist_pos == 0 {
            input = hist_save
          }else{
            input = hist[len(hist)-hist_pos]
          }
          pos = len(input)
        }

        mode = 0
      }

      prompt := ">>> " + input + "\x1b[1000D"
      prompt += fmt.Sprintf("\x1b[%dC", pos + 4)
      SetPrompt(prompt)
		}
	}
  SetPrompt("")

}
