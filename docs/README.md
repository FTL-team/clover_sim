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
2. Download [base.sqsh](https://drive.google.com/file/d/1g2m74UiFM8sQkwyFyCn8fJp3ZVH7-PXl/view?usp=sharing) and place it in same folder as contents of archive of step 1
3. Done!!! You can now run simulator
4. Optional. Install cloversim course:
    ```bash
    cd <path_to_cloversim>/tasks && git clone https://github.com/FTL-team/cloversim_course
    ```

## Option 1: Use clover_sim with WebUI

1. Go to `cloversim` directory
2. Launch cloversim webnode
    ```bash
    sudo ./cloversim node
    ```
3. Open `localhost:7777` in browser
4. Enjoy the cloversim

We also have video demonstration/tutorial: [link](https://youtu.be/aPOPHD3M3ZM) 

## Option 2: Use clover_sim with CLI

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