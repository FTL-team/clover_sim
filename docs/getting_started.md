# Starting simulator

1. Go to `clover_sim` directory
2. Create workspace(workspace is concept for managing and transfering simulator files: projects, packages, etc.)
    ```bash
    sudo ./clover_sim workspace create test
    ```
3. Launch simulator container
    ```bash
    sudo  ./clover_sim launch test
    ```
4. fds   
5. Connect via ssh using `ssh clover@192.168.77.10`
6. Login into container  using login `clover` and password `clover`
7. Launch simulator using
    ```bash
    roslaunch clover_simulation simulator.launch 
    ```
8. Enjoy simulator

9. To exit container use `sudo poweroff` (inside of conatainer)

10. For security reasons you should write code inside another container
`ssh clover@192.168.77.2`


For more information check `sudo ../clover_sim help`