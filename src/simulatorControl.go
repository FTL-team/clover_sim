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
    fmt.Println("\r  help               - show this help")
    fmt.Println("\r  exit               - shutdowns cloversim")
    fmt.Println("\r  simulator start    - start simulator (gazebo, clover, etc.)")
    fmt.Println("\r  simulator stop     - stop simulator (gazebo, clover, etc.)")
    fmt.Println("\r  simulator restart  - restart the simulator (gazebo, clover, etc.)")
  case "exit":
    sim.ContextCancel()
  case "simulator":
    if len(parts) < 2 {
      HostLogger.Error("Not enough arguments for simulator command, check help")
      return
    }
    switch parts[1] {
    case "start":
      sim.StartSimulator()

    case "stop":
      sim.StopSimulator()

    case "restart":
      sim.RestartSimulator()
    
    default:
      HostLogger.Error("Unknown simulator command: %s, check help", parts[1])
    }
  case "randomization":
    if len(parts) < 2 {
      HostLogger.Error("Not enough arguments for randomization command, check help")
      return
    }
    switch parts[1] {
    case "get":
      fmt.Println("\rRandomization: " + sim.TaskSeed.Seed)
    
    case "set":
      if len(parts) < 3 {
        HostLogger.Error("Not enough arguments for randomization command, check help")
      }
      sim.TaskSeed.SetSeed(parts[2])
    
    case "new":
      sim.TaskSeed.NewSeed()
      fmt.Println("\rNew randomization: " + sim.TaskSeed.Seed)

    default:
      HostLogger.Error("Unknown randomization command: %s, check help", parts[1])
    }
  case "score":
    sim.TaskScore.Display()

  default:
    HostLogger.Error("Unknown command: %s, check help", parts[0])
  }

}

func SimulatorController(sim *Simulator) {
	if IsTTY() {

		oldState, err := term.MakeRaw(int(os.Stdin.Fd()))
		if err != nil {
			fmt.Println(err)
			return
		}
		defer func() {
      term.Restore(int(os.Stdin.Fd()), oldState)
    }()
	}

	inp := make(chan byte)
  hist := []string{}
  
	go func() {
		b := make([]byte, 1)
		for {
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

          ProcessCommand(input, sim)
          pos = 0
          input = ""
          hist_pos = 0
          hist_save = ""
        } else if ch == 3 || ch == 4 {
          sim.ContextCancel()
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
