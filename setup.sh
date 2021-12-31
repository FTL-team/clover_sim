#!/bin/bash

wget https://nightly.link/FTL-team/clover_sim/workflows/build/main/clover_sim.zip
unzip clover_sim.zip

chmod +x ./clover_sim ./virgl/virgl_test_server

mkdir base_fs
cd base_fs

wget https://nightly.link/FTL-team/clover_sim_basefs/workflows/build/main/base.tar.gz.zip
unzip base.tar.gz.zip
rm base.tar.gz.zip

cd ..
sudo ./clover_sim prepare