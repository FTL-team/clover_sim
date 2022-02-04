package main

import (
	"fmt"
	"os"

	"github.com/urfave/cli/v2"
	// "time"
)

var HostLogger *Logger

func main() {
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

					return LaunchSimulator(workspace)
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
