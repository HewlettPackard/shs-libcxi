#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only or BSD-2-Clause
# Copyright 2026 Hewlett Packard Enterprise Development LP
#
# Check that src/libcxi_stub.c (the --enable-stub-only link stub) defines
# exactly the set of CXIL_API functions declared in include/libcxi.h, and
# that src/libcxi.sym exports each of them (the version script ends in
# "local: *", so a function missing from it is silently not exported).
#
# The stub is compiled against the header with -Werror in every regular build
# (src/libcxi_stub_check.la), which catches a changed signature; this script
# catches a function that was added to the header but not to the stub or the
# version script, or left in the stub after being removed from the header.
# Runs as part of `make check` (TAP output, two tests) and needs no device,
# VM, or criterion. A completed comparison exits 0 and reports through TAP;
# only an execution failure exits non-zero.
#
# Usage: contrib/check-stub-symbols.sh [top_srcdir]

set -e

top_srcdir=${1:-$(cd "$(dirname "$0")/.." && pwd)}
header="$top_srcdir/include/libcxi.h"
stub="$top_srcdir/src/libcxi_stub.c"
symfile="$top_srcdir/src/libcxi.sym"

echo "1..2"
for f in "$header" "$stub" "$symfile"; do
	if [ ! -r "$f" ]; then
		echo "Bail out! check-stub-symbols: cannot read $f"
		exit 1
	fi
done

# Print one "<file-tag> <function>" line per CXIL_API declaration or
# definition. A declaration may wrap before its opening parenthesis.
list_api_functions() {
	awk -v tag="$1" '
		index($0, "CXIL_API ") == 1 {
			line = $0
			while (index(line, "(") == 0 && (getline nextline) > 0)
				line = line " " nextline
			sub(/\(.*/, "", line)
			gsub(/\*/, " ", line)
			$0 = line
			print tag, $NF
		}
	' "$2"
}

header_fns=$(list_api_functions header "$header" | awk '{print $2}' | sort -u)
stub_fns=$(list_api_functions stub "$stub" | awk '{print $2}' | sort -u)

missing=$(comm -23 <(echo "$header_fns") <(echo "$stub_fns"))
extra=$(comm -13 <(echo "$header_fns") <(echo "$stub_fns"))

status=0
if [ -n "$missing" ]; then
	echo "# declared in include/libcxi.h but not defined in src/libcxi_stub.c:"
	# shellcheck disable=SC2086
	printf '#   %s\n' $missing
	status=1
fi
if [ -n "$extra" ]; then
	echo "# defined in src/libcxi_stub.c but not declared in include/libcxi.h:"
	# shellcheck disable=SC2086
	printf '#   %s\n' $extra
	status=1
fi

count=$(echo "$header_fns" | wc -l | tr -d ' ')
if [ $status -eq 0 ]; then
	echo "ok 1 - src/libcxi_stub.c defines all $count CXIL_API functions in include/libcxi.h"
else
	echo "not ok 1 - src/libcxi_stub.c does not match the $count CXIL_API functions in include/libcxi.h"
fi

# Every header function must appear exactly once in the version script.
sym_fns=$(sed -n 's/^[[:space:]]*\(cxil_[A-Za-z0-9_]*\);.*/\1/p' "$symfile" | sort)
unexported=$(comm -23 <(echo "$header_fns") <(echo "$sym_fns" | sort -u))
duplicated=$(echo "$sym_fns" | uniq -d)
if [ -z "$unexported" ] && [ -z "$duplicated" ]; then
	echo "ok 2 - src/libcxi.sym exports every CXIL_API function exactly once"
else
	if [ -n "$unexported" ]; then
		echo "# declared in include/libcxi.h but not listed in src/libcxi.sym:"
		# shellcheck disable=SC2086
		printf '#   %s\n' $unexported
	fi
	if [ -n "$duplicated" ]; then
		echo "# listed more than once in src/libcxi.sym:"
		# shellcheck disable=SC2086
		printf '#   %s\n' $duplicated
	fi
	echo "not ok 2 - src/libcxi.sym does not export every CXIL_API function exactly once"
fi
exit 0
