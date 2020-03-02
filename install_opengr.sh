#!/bin/bash

WORKDIR=/tmp/opengr_work
mkdir -p ${WORKDIR}
git clone --quiet https://github.com/STORM-IRIT/OpenGR.git ${WORKDIR}
git -C ${WORKDIR} reset --hard 0967cd880950b35786b8fd098837c9eb1fe2aca4
mkdir ${WORKDIR}/build && cd ${WORKDIR}/build
cmake -DCMAKE_BUILD_TYPE=Release ..
make install
