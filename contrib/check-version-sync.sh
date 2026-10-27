#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-only or BSD-2-Clause
# Copyright 2026 Hewlett Packard Enterprise Development LP
#
# Check that the API version recorded in configure.ac (api_major, api_minor,
# api_patch) is also what the other version sources say:
#   include/libcxi.h   LIBCXI_API_MAJOR / LIBCXI_API_MINOR / LIBCXI_API_PATCH
#   cray-libcxi.spec   Version: (skipped when the spec does not exist)
# (src/libcxi.sym is checked separately, together with the version-node
# rules, by the symbol-version check.) `make check` runs this as a TAP test
# (two tests) so the three cannot drift apart; CI runs it for every commit. It is a source-tree lint, not a
# configure check: configure runs in every downstream build, where nothing
# can be fixed. Needs no device, VM or criterion. POSIX sh.
#
# Usage: contrib/check-version-sync.sh [top_srcdir [expected_version]]
#   expected_version  MAJOR.MINOR.PATCH to check against; by default the
#                     values are read from configure.ac.
# Like contrib/check-stub-symbols.sh, a completed comparison exits 0 and
# reports through TAP (`not ok` lines); only an execution failure (a missing
# or ambiguous value, an unreadable file) exits non-zero.

top_srcdir=${1:-$(cd "$(dirname "$0")/.." && pwd)}
expected=$2
configure_ac="$top_srcdir/configure.ac"
header="$top_srcdir/include/libcxi.h"
spec="$top_srcdir/cray-libcxi.spec"

bail() {
	echo "Bail out! check-version-sync: $1"
	exit 1
}

echo "1..2"
for f in "$configure_ac" "$header"; do
	[ -r "$f" ] || bail "cannot read $f"
done

# one_value FILE SED-EXPR: the single value the substitution extracts from
# FILE (CR stripped); nothing when it is absent, AMBIGUOUS when it occurs
# more than once.
one_value() {
	v=$(tr -d '\r' < "$1" | sed -n "$2")
	case $(printf '%s' "$v" | grep -c '') in
		0) ;;
		1) printf '%s' "$v" ;;
		*) printf 'AMBIGUOUS' ;;
	esac
}
# m4_value NAME: the N in configure.ac's m4_define([NAME],[N]).
m4_value() {
	one_value "$configure_ac" \
		"s/^m4_define(\[$1\],[[:space:]]*\[\([0-9][0-9]*\)\])[[:space:]]*$/\1/p"
}
# header_value NAME: the N in include/libcxi.h's "#define NAME N"; a trailing
# comment is allowed, a trailing expression is not.
header_value() {
	one_value "$header" \
		"s/^#define[[:space:]][[:space:]]*$1[[:space:]][[:space:]]*\([0-9][0-9]*\)[[:space:]]*\(\/\*.*\)\{0,1\}$/\1/p"
}
# version_problem VALUE: why VALUE is not a usable MAJOR.MINOR.PATCH, if so.
version_problem() {
	case $1 in
		*AMBIGUOUS*) echo "defined more than once" ;;
		*..*|.*|*.|*[!0-9.]*|*.*.*.*) echo "missing or not numeric" ;;
		*) ;;
	esac
}

if [ -z "$expected" ]; then
	expected="$(m4_value api_major).$(m4_value api_minor).$(m4_value api_patch)"
fi
problem=$(version_problem "$expected")
[ -z "$problem" ] || bail "configure.ac api_major/api_minor/api_patch $problem ($expected)"

# 1. The committed header must carry the same three numbers.
hdr="$(header_value LIBCXI_API_MAJOR).$(header_value LIBCXI_API_MINOR).$(header_value LIBCXI_API_PATCH)"
problem=$(version_problem "$hdr")
if [ -n "$problem" ]; then
	echo "not ok 1 - include/libcxi.h LIBCXI_API_* $problem ($hdr)"
elif [ "$hdr" = "$expected" ]; then
	echo "ok 1 - include/libcxi.h LIBCXI_API_* is $expected"
else
	echo "not ok 1 - include/libcxi.h LIBCXI_API_* ($hdr) does not match configure.ac ($expected)"
fi

# 2. The committed RPM spec, when it exists, must carry the same version.
if [ ! -e "$spec" ]; then
	echo "ok 2 # SKIP cray-libcxi.spec not present"
else
	[ -r "$spec" ] || bail "cannot read $spec"
	specv=$(one_value "$spec" 's/^Version:[[:space:]]*\([^[:space:]]*\)[[:space:]]*$/\1/p')
	if [ "$specv" = "$expected" ]; then
		echo "ok 2 - cray-libcxi.spec Version is $expected"
	else
		echo "not ok 2 - cray-libcxi.spec Version (${specv:-none}) does not match configure.ac ($expected)"
	fi
fi

exit 0
