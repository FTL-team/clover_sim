# Clover sim
![https://i.imgur.com/AUTIJBg.png](https://i.imgur.com/AUTIJBg.png)

Simple and powerful tool for Clover simulation with ability to create workspaces

You will get:

- Simple and fast way to start a simulator
- Tasks creation
- Tasks checker
- Full simulation of Raspberry pi functions(GPIO, camera, USB, etc.)
- Gazebo and task checker are isolated from user workspace and clovered
- Ability to start several copters in different containers

# Installation

To install this tool just run these commands:
```bash
mkdir clover_sim && cd clover_sim
curl https://raw.githubusercontent.com/FTL-team/clover_sim/main/setup.sh | bash
```

For successful installation you need:
- curl
- systemd-nspawn
- systemd-run
- nftables
- iproute2