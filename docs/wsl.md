# Cloversim inside wsl

![wsl](./assets/wsl.png)

## Installation
1. Install wsl and ubuntu inside it:
   ```powershell
   wsl --set-default-version 2
   wsl --install -d ubuntu
   ```
2. Install systemd inside ubuntu container and reboot your pc:
   ```bash
   curl -L -O "https://raw.githubusercontent.com/nullpo-head/wsl-distrod/main/install.sh"
   chmod +x install.sh
   sudo ./install.sh install
   sudo /opt/distrod/bin/distrod enable
   ```
   Reboot your pc

3. Install [VcXsrv](https://sourceforge.net/projects/vcxsrv/)

4. Install cloversim dependencies:
   ```bash
   sudo apt update && sudo apt install systemd-container libvirglrenderer1 iptables mesa-utils socat wget unzip libegl1-mesa
   ```

5. Install cloversim tool:
   ```bash
   mkdir clover_sim && cd clover_sim
   curl https://raw.githubusercontent.com/FTL-team/clover_sim/main/setup.sh | bash
   ```

## Running cloversim

### Option 1: WSLg
In case you are using windows 11 and wsl preview you can just run cloversim. But in other case check option 2

### Option2: VcXsrv
To launch cloversim using vcxsrv follow this instructions:
1. Launch VcXsrv:
   ![Options](./assets/vcxsrv.png)

2. Run this code in one wsl terminal:
   ```bash
   export HOSTIP=$(cat /etc/resolv.conf|grep nameserver|head -n 1|cut -d ' ' -f2)&&socat UNIX-LISTEN:/tmp/.X11-unix/X2,fork TCP-CONNECT:$HOSTIP:6000
   ```

3. Open new wsl terminal and run `export DISPLAY=:2`, now you can use cloversim.

# Troubleshooting

## Failed to determine whether the unified cgroups hierarchy is used: No such file or directory

Your current distro is WSL1 cloversim requires WSL2.
Change your distro to WSL2 using:
`wsl --set-version Ubuntu 2`
