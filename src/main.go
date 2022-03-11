package main

import (
	"fmt"
	"os"

	"github.com/urfave/cli/v2"
	"math/rand"
	"time"
)

var HostLogger *Logger

func main() {
	rand.Seed(time.Now().UnixNano())

	HostLogger = NewLogger("host")
	
	if os.Geteuid() != 0 {
		HostLogger.Error("You should run this as root")
		os.Exit(1)
	}


	app := &cli.App{
		Flags: []cli.Flag{
		  &cli.BoolFlag{
				Name: "verbose",
				Usage: "enable verbose logging",

			},
		},
		Before: func(c *cli.Context) error {
			if c.Bool("verbose") {
				SetLogLevel(VERBOSE_LOGLEVEL)
			}else{
				SetLogLevel(INFO_LOGLEVEL)
			}
			
			return nil
		},
		Commands: []*cli.Command{
			{
				Name:    "workspace",
				Aliases: []string{"ws"},
				Usage:   "Manage workspaces",
				Subcommands: []*cli.Command{
					{
						Name:      "create",
						Usage:     "Create workspace",
						ArgsUsage: "NAME",
						Action: func(c *cli.Context) error {
							if c.Args().Len() != 1 {
								cli.ShowCommandHelp(c, "create")
								return fmt.Errorf("no name specified")
							}
							_, err := CreateWorkspace(c.Args().First())
							return err
						},
					}, {
						Name:      "remove",
						Usage:     "Remove workspace",
						ArgsUsage: "NAME",
						Action: func(c *cli.Context) error {
							if c.Args().Len() != 1 {
								cli.ShowCommandHelp(c, "remove")
								return fmt.Errorf("no name specified")
							}
							workspace, err := LoadWorkspace(c.Args().First())
							
							if err != nil {
								return err
							}
							return workspace.Remove()
						},
					},
					{
						Name:  "list",
						Usage: "List workspaces",
						Action: func(c *cli.Context) error {
							names, err := ListWorkspaceNames()
							if err != nil {
								return err
							}
							if len(names) == 0 {
								fmt.Println("No workspaces found")
							} else {
								fmt.Println("Workspaces:")
								for _, name := range names {
									fmt.Println("  ", name)
								}
							}
							return nil
						},
					},
          {
						Name:      "export",
						Usage:     "Export workspace",
						ArgsUsage: "NAME",
						Action: func(c *cli.Context) error {
							if c.Args().Len() != 1 {
								cli.ShowCommandHelp(c, "remove")
								return fmt.Errorf("no name specified")
							}
							workspace, err := LoadWorkspace(c.Args().First())
							if err != nil {
								return err
							}
							return workspace.Export()
						},
					},
					{
						Name:      "import",
						Usage:     "Import workspace",
						ArgsUsage: "PATH_TO_WORKSPACE",
						Action: func(c *cli.Context) error {
							if c.Args().Len() != 1 {
								cli.ShowCommandHelp(c, "import")
								return fmt.Errorf("no path to file specified")
							}
							return ImportWorkspace(c.Args().First())
						},
					},

					{
						Name:      "duplicate",
						Usage:     "Create copy of workspace",
						ArgsUsage: "SOURCE_WORKSPACE DUPLICATED_WORKSPACE_NAME",
						Action: func(c *cli.Context) error {
							if c.Args().Len() != 2 {
								cli.ShowCommandHelp(c, "duplicate")

								return fmt.Errorf("no source workspace or duplicate workspace name not specified")
							}
							workspace, err := LoadWorkspace(c.Args().First())
							if err != nil {
								return err
							}
							return workspace.Duplicate(c.Args().Get(1))
						},
					},

					{
						Name:      "clean",
						Usage:     "Clean workspace from logs and cache, can dramatically reduce size",
						ArgsUsage: "WORKSPACE_NAME",
						Action: func(c *cli.Context) error {
							if c.Args().Len() != 1 {
								cli.ShowCommandHelp(c, "cleanup")

								return fmt.Errorf("no workspace name specified")
							}
							workspace, err := LoadWorkspace(c.Args().First())
							if err != nil {
								return err
							}
							return workspace.Clean()
						},
					},
					{
						Name:      "build",
						Usage:     "Build workspace using user supplied script",
						ArgsUsage: "WORKSPACE_NAME BUILD_COMMAND...",
							Flags: []cli.Flag{
							&cli.StringFlag {
								Name: "task",
								Usage: "path to task in tasks dir, example: base_task, example_task, task_collections/task_task",
								Value: "base_task",
							},
							&cli.StringSliceFlag{
								Name: "bind",
								Usage: "bind dirs from host to build container, exmaple: /home/user/buil_proj:/build_proj, this will allow acces to /home/user/buil_proj to /build_proj in build container",
							},
						},
						Action: func(c *cli.Context) error {
							if c.Args().Len() < 2 {
								cli.ShowCommandHelp(c, "build")

								return fmt.Errorf("not enough argumnetc")
							}
							workspace, err := LoadWorkspace(c.Args().First())
							command := ""
							for _, a := range c.Args().Tail() {
								command += a + " "
							}
							fmt.Println(command)

							if err != nil {
								return err
							}
							return workspace.BuildWorkspace(c.String("task"), command, c.StringSlice("bind"))
						},
					},
				},
			}, {
				Name: "prepare",
				Usage: "Prepare simulator",
				Action: func(c *cli.Context) error {
					return Prepare()
				},
			}, {
				Name: "launch",
				Usage: "Launch simulator",
				ArgsUsage: "WORKSPACE_NAME",
				Flags: []cli.Flag{
					&cli.BoolFlag{
						Name: "no-start",
						Usage: "disable automatic simulator(gazebo, px4, clover, etc.) start",
					},
					&cli.StringFlag {
						Name: "task",
						Usage: "path to task in tasks dir, example: base_task, example_task, task_collections/task_task",
						Value: "base_task",
					},
				},
				
				Action: func(c *cli.Context) error {
					if c.Args().Len() != 1 {
						cli.ShowCommandHelp(c, "remove")
						return fmt.Errorf("no workspace name specified")
					}
					workspace, err := LoadWorkspace(c.Args().First())
					if err != nil {
						HostLogger.Error("Failed to load workspace, check if it exists")
						return err
					}

					return LaunchSimulator(SimulatorOptions{
						Workspace: workspace,
						StartAtReady: !c.Bool("no-start"),
						TaskPath: c.String("task"),
					})
				},
			}, {
				Name: "rebuild_cloversim",
				Hidden: true,
				HideHelp: true,
				Flags: []cli.Flag{
					&cli.BoolFlag{
						Name: "fast",
					},
				},
				
				Action: func(c *cli.Context) error {
					cloversimLayer := GetCloversimLayer()
		
					return cloversimLayer.RebuildLayer(c.Bool("fast"))
				},
			},
		},
	}

	cli.AppHelpTemplate = `clover_sim is a tool for running clover simulators.

USAGE: {{.HelpName}} command [subcommand] [command options] [arguments...]
{{if .Commands}}
COMMANDS:
{{range .Commands}}{{if not .HideHelp}}  {{join .Names ", "}}{{ "\t"}}{{.Usage}}{{range .Subcommands}}
{{ "   "}} {{join .Names ", "}}{{ "\t"}}{{.Usage}} {{end}}{{ "\n" }}{{end}}{{end}}{{end}}
`
	err := app.Run(os.Args)
	if err != nil {
		HostLogger.Error("%s", err)
	}
}
