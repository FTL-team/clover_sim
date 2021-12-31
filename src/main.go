package main

import (
	"fmt"
	"log"
	"os"

	"github.com/urfave/cli/v2"
	// "time"
)

func main() {
	if os.Geteuid() != 0 {
		fmt.Println("You should run this as root")
		os.Exit(1)
	}

	app := &cli.App{
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
								return fmt.Errorf("No name specified")
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
								return fmt.Errorf("No name specified")
							}
							workspace, err := LoadWorkspace(c.Args().First())
							
							if err != nil {
								return err
							}
							return RemoveWorkspace(workspace)
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
								return fmt.Errorf("No name specified")
							}
							workspace, err := LoadWorkspace(c.Args().First())
							if err != nil {
								return err
							}
							return ExportWorkspace(workspace)
						},
					},
					{
						Name:      "import",
						Usage:     "Import workspace",
						ArgsUsage: "PATH_TO_WORKSPACE",
						Action: func(c *cli.Context) error {
							if c.Args().Len() != 1 {
								cli.ShowCommandHelp(c, "import")
								return fmt.Errorf("No path to file specified")
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
						return fmt.Errorf("No workspace name specified")
					}
					workspace, err := LoadWorkspace(c.Args().First())
					if err != nil {
						fmt.Println("Failed to load workspace, check if it exists")
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
	// cli.SubcommandHelpTemplate = `

	err := app.Run(os.Args)
	if err != nil {
		log.Fatal(err)
	}

	// wait_virgl := make(chan bool)
	// go start_virgl(wait_virgl)
	// if <-wait_virgl {
	// 	fmt.Println("Virgl server ready")
	// }else {
	// 	fmt.Println("Virgl server failed to start")
	// 	os.Exit(1)
	// }

	// c, err := createContainer("clover_sim")
	// time.Sleep(5 * time.Second)
	// defer destroyContainer(c)
	// if err != nil {
	// 	fmt.Println("Could not create container")
	// 	panic(err)
	// }

	// fmt.Scanln()
}
