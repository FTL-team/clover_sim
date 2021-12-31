# Clover sim
![https://i.imgur.com/AUTIJBg.png](https://i.imgur.com/AUTIJBg.png)
Simple and powerful tool for Clover simulation


# How to setup
```
mkdir clover_sim && cd clover_sim
curl https://raw.githubusercontent.com/FTL-team/clover_sim/main/setup.sh | bash
```

# How to use
1. Go to `clover_sim` directory
2. Create workspace(workspace is concept for managing and transfering simulator files: projects, packages, etc.)
    ```bash
    sudo ./clover_sim workspace create test
    ```
3. Launch simulator container
    ```bash
    sudo  ./clover_sim launch test
    ```
4. Login into container  using login `clover` and password `clover`
5. Launch simulator using
    ```bash
    roslaunch clover_simulation simulator.launch 
    ```
6. Enjoy simulator
7. You can connect via ssh using `ssh clover@localhost -p 2222`
8. To exit container use `sudo poweroff` (inside of conatainer)

Check `sudo ../clover_sim help` for more information
