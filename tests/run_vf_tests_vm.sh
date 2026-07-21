#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only or BSD-2-Clause
# Copyright 2020 Hewlett Packard Enterprise Development LP

cd $(dirname $0)
. ./virtualize.sh

cxi_service="../utils/cxi_service"
parent_yaml=$(mktemp)
vf_yaml=$(mktemp)

create_service()
{
  local cmd_out

  cmd_out=$($cxi_service create "$@")
  if [ $? -ne 0 ]; then
    echo "$cmd_out"
    echo "Failed to create service" >&2
    exit 1
  fi

  SERVICE_ID=$(printf '%s\n' "$cmd_out" | \
    sed -n 's/^Successfully created service:[[:space:]]*\([0-9]\+\)[[:space:]]*$/\1/p' | \
    tail -n 1)

  if [ -z "$SERVICE_ID" ]; then
    echo "$cmd_out"
    echo "Failed to parse service ID from cxi_service create output" >&2
    exit 1
  fi

  echo "$cmd_out"
}

cleanup()
{
  rm -f "$parent_yaml"
  rm -f "$vf_yaml"
  echo 0 > /sys/class/cxi/cxi0/device/sriov_numvfs
  if [ -n "$parent_svc_id" ]; then
    echo 0 > /sys/class/cxi/cxi0/vf/0/svc_id
    $cxi_service -d cxi0 delete -s "$parent_svc_id"
  fi
}
trap cleanup EXIT

cat <<EOF > "$parent_yaml"
resource_limits: 0
restricted_vnis: 0
restricted_members: 0
restricted_tcs: 0
exclusive_cp: 0

vnis:
  vni_min: 32
  vni_max: 63

EOF

cat <<EOF > "$vf_yaml"
resource_limits: 0
restricted_vnis: 1
restricted_members: 0
restricted_tcs: 0
exclusive_cp: 0

vnis:
  vni: 63
EOF

create_service -d cxi0 -y "$parent_yaml"
parent_svc_id="$SERVICE_ID"

echo "$parent_svc_id" > /sys/class/cxi/cxi0/vf/0/svc_id

echo 1 > /sys/class/cxi/cxi0/device/sriov_numvfs
sleep 1

create_service -d cxi1 -y "$vf_yaml"
vf_svc_id="$SERVICE_ID"
$cxi_service list -d cxi1

CXIL_TEST_DEV=1 CXIL_TEST_SVC_ID="$vf_svc_id" ./libcxi_test --verbose \
  --tap=libcxi_vf_test.tap --tap=- -j1
