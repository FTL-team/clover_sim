![Cloversim banner](./banner.png)

## Features
Simple and powerful tool for Clover simulation with workspaces and tasks.

Key features:

- Simple and fast way to start a simulator
- Workspaces concept that allows you to easily manage and share your projects
- Powerful tasks framework with ability to generate worlds and automatically check user solutions
- User code for clover doesn't have access to gazebo and can't cheat

## Installation

### Dependencies

> Currently installation is supported only on x86 systems

> For windows users: you can install cloversim in [WSL 2](https://learn.microsoft.com/en-us/windows/wsl/install), to run gazebo gui client [WSLg](https://github.com/microsoft/wslg) is required.
>
> In case you can't use linux or WSL you can install cloversim in Virtual Machine, don't forget to enable 3d acceleration and allocate enough resources

Dependencies:
- runc
- xauth
- iproute2
- iptables
- OpenGL drivers
- virglrenderer
- tar

Install dependency on Ubuntu:
```bash
sudo apt update && sudo apt install runc libvirglrenderer1 iptables mesa-utils procps xauth tar
```

### Installation process
1. Download and unpack archive from [cloversim releases](https://github.com/FTL-team/clover_sim/releases)
2. Download `base.sqsh` from [cloversim_basefs releases](https://github.com/FTL-team/clover_sim_basefs/releases) and place it in same folder as contents of archive of step 1
3. Done!!! You can now run simulator


## Starting simulator

1. Go to `cloversim` directory
2. Create workspace
    ```bash
    sudo ./cloversim workspace create test
    ```
3. Launch simulator container
    - Simple start:
    ```bash
    sudo  ./cloversim launch test
    ```
    - Start with some task:
    ```bash
    sudo ./cloversim launch --task example_task test
    ```
    
> You can connect to your simulated clover drone using `ssh clover@192.168.77.10` password: **clover**

4. Enjoy simulator

> For more information check `sudo ../cloversim help`