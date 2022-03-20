# Start container

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

For more information check`sudo ../clover_sim help`