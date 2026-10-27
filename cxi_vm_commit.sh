#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only or BSD-2-Clause
# Copyright 2020 Hewlett Packard Enterprise Development LP

./autogen.sh
mkdir -p install
./configure --with-criterion=$(realpath ../Criterion/build/install/) --prefix=$(realpath ./install) --with-systemdsystemunitdir=$(realpath ./install) --with-udevrulesdir=$(realpath ./install)

make dist
make -j 8
make install # libfabric uses the libcxi installation
ws=~/workspace/workspace/os-networking-team/cassini-vm/libcxi
ssh -tt localhost "cd $ws &&
	rm -f tests/libcxi_test.tap tests/libcxi_vf_test.tap &&
	exec timeout -k 30 15m make check"
