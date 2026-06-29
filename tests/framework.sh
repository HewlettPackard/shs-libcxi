# SPDX-License-Identifier: GPL-2.0-only or BSD-2-Clause
# Copyright 2020 Hewlett Packard Enterprise Development LP

# Framework for testing in a VM using virtme.
#
# Thin shim over the shared launcher in devbootstrap/vm-tools. This must be
# sourced by a test script.

TOP_DIR=$(realpath $(pwd)/../..)
source "$TOP_DIR/vm-tools/vm-lib.sh"
source "$TOP_DIR/libcxi/vm.conf"

# Arguments are '$noexit [command [args...]]'
# If $noexit != 0, run the command as a boot (init) script and stay in the VM.
# Otherwise run the command, then exit the VM.
# The tests directory is exposed read-write so .tap output lands on the host.
function startvm {
	local noexit=$1
	shift 1

	if [[ $noexit -ne 0 ]]; then
		vm_startvm --interactive --rwdir "$(pwd)" "$@"
	else
		vm_startvm --rwdir "$(pwd)" "$@"
	fi
}
