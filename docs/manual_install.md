# Manuall installation

In case you want to install clover_sim manually, follow this instructions:

> Currently installation is supported only on linux x86 systems
> You can also install cloversim on windows x86 system with wsl, [check this instruction](./wsl.md)
> 
1. Download and unpack clover_sim

   ```bash
   wget https://nightly.link/FTL-team/clover_sim/workflows/build/main/clover_sim.zip
   unzip clover_sim.zip
   ```

2. Give clover_sim execute permission
   ```bash
   chmod +x ./clover_sim ./virgl/virgl_test_server
   ```
3. Create base_fs directory
   ```bash
   mkdir base_fs
   cd base_fs
   ```
4. Download and unpack base_fs
   ```bash
   wget https://nightly.link/FTL-team/clover_sim_basefs/workflows/build/main/base.tar.gz.zip
   unzip base.tar.gz.zip
   rm base.tar.gz.zip
   ```
5. Go back to clover_sim binary and finish installation
   ```bash
   cd ..
   sudo ./clover_sim prepare
   ```
