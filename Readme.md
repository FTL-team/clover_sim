![Cloversim banner](./docs/banner.png)

## Features
Simple and powerful tool for Clover simulation with workspaces and tasks.

Key features:

- Simple and fast way to start a simulator
- Workspaces concept that allows you to easily manage and share your projects
- Powerful tasks framework with ability to generate worlds and automatically check user solutions
- User code for clover doesn't have access to gazebo and can't cheat

## Installation

To install this tool just run these commands:
```bash
mkdir clover_sim && cd clover_sim
curl https://raw.githubusercontent.com/FTL-team/clover_sim/main/setup.sh | bash
```

Dependencies:
- curl
- systemd-nspawn
- systemd-run
- nftables
- iproute2

## Starting simulator

1. Go to `clover_sim` directory
2. Create workspace
    ```bash
    sudo ./clover_sim workspace create test
    ```
3. Launch simulator container
    - Simple start:
    ```bash
    sudo  ./clover_sim launch test
    ```
    - Start with some task:
    ```bash
    sudo ./clover_sim launch --task example_task test
    ```
    
> You can connect to your simulated clover drone using `ssh clover@192.168.77.10` password: **clover**

4. Enjoy simulator



> For more information check `sudo ../clover_sim help`