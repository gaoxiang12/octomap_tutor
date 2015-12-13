#!/bin/bash
builddir="./build/"
if [ ! -x "$builddir" ]; then
    mkdir "$builddir"
fi
cd build;
cmake ..
make -j8 2>error.log;
head -n 20 error.log;
cd ..
