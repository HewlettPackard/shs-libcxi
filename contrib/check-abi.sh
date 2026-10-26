#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only or BSD-2-Clause
# Copyright 2026 Hewlett Packard Enterprise Development LP
#
# check-abi.sh - check the built libcxi.so.1 against the ABI baseline
#
# abi/libcxi.abi (the "baseline") is a generated description of the ABI we
# last agreed to ship. This script makes two comparisons using abidiff
# (libabigail):
#
#   1. The built library against the baseline in your working tree.
#      Any difference fails. For a new function, run `make update-abi`
#      and commit the regenerated baseline so the addition is reviewed.
#
#   2. The built library against the baseline as it was at the PR base,
#      ignoring added functions. Anything removed or changed fails.
#      Without this, an incompatible change could be hidden by simply
#      regenerating the baseline in the same PR.
#
# Usage: contrib/check-abi.sh -s SRCDIR -l LIBRARY [-p BASE_REF]
#   -s  top of the source tree (contains abi/ and include/)
#   -l  the built libcxi.so.1
#   -p  git ref for comparison 2 (default: merge-base of HEAD and
#       $UPSTREAM_BRANCH, which defaults to origin/main)

set -o errexit
set -o pipefail

eprintf() {
	# shellcheck disable=SC2059
	printf "$@" >&2
}

main() {
	local src="" lib="" base_ref="" opt rc base_baseline
	local upstream="${UPSTREAM_BRANCH:-origin/main}"

	while getopts "s:l:p:" opt; do
		case $opt in
		s) src="$OPTARG" ;;
		l) lib="$OPTARG" ;;
		p) base_ref="$OPTARG" ;;
		*) eprintf "usage: %s -s SRCDIR -l LIBRARY [-p BASE_REF]\n" "$0"
		   exit 1 ;;
		esac
	done
	if [ -z "$src" ] || [ -z "$lib" ]; then
		eprintf "usage: %s -s SRCDIR -l LIBRARY [-p BASE_REF]\n" "$0"
		exit 1
	fi

	if ! command -v abidiff >/dev/null; then
		eprintf "abidiff not found; install libabigail "
		eprintf "(libabigail-tools on SLES/openSUSE, "
		eprintf "libabigail on RHEL/Fedora)\n"
		exit 1
	fi

	local scope=(--headers-dir2 "$src/include" --drop-private-types
		     --suppressions "$src/abi/libcxi.suppr")

	# 1. Built library against the baseline in the working tree
	rc=0
	abidiff "${scope[@]}" "$src/abi/libcxi.abi" "$lib" || rc=$?
	if [ "$rc" -ne 0 ]; then
		eprintf "\nERROR: libcxi ABI differs from abi/libcxi.abi "
		eprintf "(abidiff exit %d).\n" "$rc"
		eprintf "  Removed or changed functions/types are never "
		eprintf "acceptable.\n"
		eprintf "  If you only added a function, run 'make update-abi'"
		eprintf " and\n  commit abi/libcxi.abi, together with the new "
		eprintf "LIBCXI_x.y\n  version node in src/libcxi.sym and the "
		eprintf "version bump.\n"
		exit 1
	fi

	# 2. Built library against the baseline at the PR base
	if ! git -C "$src" rev-parse --git-dir >/dev/null 2>&1; then
		echo "ABI OK (not a git tree: PR base comparison skipped)"
		return 0
	fi
	if [ -z "$base_ref" ] &&
	   ! base_ref="$(git -C "$src" merge-base "$upstream" HEAD 2>/dev/null)"
	then
		eprintf "error - no merge-base of HEAD and %s\n" "$upstream"
		eprintf "Set UPSTREAM_BRANCH or pass ABI_BASE=<ref>\n"
		exit 1
	fi
	base_baseline="$(mktemp)"
	trap 'rm -f "$base_baseline"' EXIT
	if ! git -C "$src" show "${base_ref}:abi/libcxi.abi" \
	     >"$base_baseline" 2>/dev/null; then
		echo "ABI OK (no baseline at PR base: comparison skipped)"
		return 0
	fi

	rc=0
	abidiff --no-added-syms "${scope[@]}" "$base_baseline" "$lib" || rc=$?
	if [ "$rc" -ne 0 ]; then
		eprintf "\nERROR: libcxi ABI changed incompatibly since the "
		eprintf "PR base (%s; abidiff exit %d).\n" \
			"$(git -C "$src" rev-parse --short "$base_ref")" "$rc"
		eprintf "  A function or type that existed at the PR base was "
		eprintf "removed or changed.\n  Regenerating abi/libcxi.abi "
		eprintf "does not make that acceptable; add a new function "
		eprintf "instead.\n"
		exit 1
	fi

	echo "ABI OK: $lib matches abi/libcxi.abi; nothing removed or changed" \
	     "since the PR base"
}

main "$@"
