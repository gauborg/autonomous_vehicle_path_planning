#!/bin/bash

mkdir build;
cd build;
cmake ..;
make;

./path_planning;