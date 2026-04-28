#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only or BSD-2-Clause
# Copyright 2020 Hewlett Packard Enterprise Development LP

cd $(dirname $0)
. ./virtualize.sh

echo 1 > /sys/class/cxi/cxi0/device/sriov_numvfs
sleep 1

CXIL_TEST_DEV=1 ./libcxi_test --verbose --tap=libcxi_vf_test.tap --tap=- -j1
