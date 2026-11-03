#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only or BSD-2-Clause
# Copyright 2026 Hewlett Packard Enterprise Development LP
#
# check-glibc-baseline.sh - keep libcxi.so.1 loadable on older glibc
#
# The installed libcxi.so.1 is bind-mounted into containers (the libfabric
# stub-link model), so the glibc symbol versions it requires are the oldest
# glibc a container can have. Two checks on the built library:
#
#   1. The highest GLIBC_x.y version node it references must not exceed
#      the baseline (default 2.28, the RHEL 8 glibc). A build on a newer host
#      only exceeds it when the code starts using a newer glibc symbol, a
#      feature macro selects redirected variants (for example _GNU_SOURCE on
#      glibc 2.38 binding the scanf family to __isoc23_*), or the link uses
#      DT_RELR relocations (-z pack-relative-relocs, the default LDFLAGS on
#      Fedora 39+ builders), whose GLIBC_ABI_DT_RELR node counts as 2.36.
#   2. Every NEEDED entry must be on the allowed list (default: libc, libm,
#      the libraries glibc 2.34 merged into libc, and libnl-3) or be glibc's
#      own dynamic linker (aarch64 builds with a global stack-protector
#      canary reference __stack_chk_guard from ld-linux-aarch64.so.1). A new
#      dependency has to be added here deliberately, because the container
#      has to provide it as well.
#
# Runs as part of `make check` (TAP output, two tests) in the regular and the
# --enable-stub-only build; needs readelf. A completed comparison exits 0 and
# reports through TAP; only an execution failure exits non-zero.
#
# Usage: contrib/check-glibc-baseline.sh [library]
#   library   the built libcxi.so.1 (default: src/.libs/libcxi.so.1 in the
#             current directory, i.e. the build tree under `make check`)
# Environment:
#   LIBCXI_GLIBC_BASELINE   highest allowed GLIBC_ version, numeric x.y
#                           (default 2.28)
#   LIBCXI_NEEDED_ALLOW     space-separated allowed NEEDED names (default:
#                           libc.so.6 libm.so.6 libpthread.so.0 libdl.so.2
#                           librt.so.1 libnl-3.so.200)
#   READELF                 readelf binary (default: readelf)
set -e

bail() {
	echo "Bail out! check-glibc-baseline: $1"
	exit 1
}

lib=${1:-src/.libs/libcxi.so.1}
baseline=${LIBCXI_GLIBC_BASELINE:-2.28}
default_allowed="libc.so.6 libm.so.6 libpthread.so.0 libdl.so.2 librt.so.1"
default_allowed="$default_allowed libnl-3.so.200"
allowed=${LIBCXI_NEEDED_ALLOW:-$default_allowed}
readelf=${READELF:-readelf}

echo "1..2"

command -v "$readelf" >/dev/null 2>&1 || bail "$readelf not found"
[ -f "$lib" ] || bail "cannot read $lib"
# The baseline must be a glibc version x.y or x.y.z; anything else ("28",
# "GLIBC_2.28") would sort after every real version and pass test 1
# unconditionally.
numeric='^[0-9]+\.[0-9]+(\.[0-9]+)?$'
echo "$baseline" | grep -Eq "$numeric" ||
	bail "LIBCXI_GLIBC_BASELINE must be x.y or x.y.z, got '$baseline'"

# readelf output is captured before it is filtered, so a readelf failure
# bails instead of passing as an empty table.
versions_out=$("$readelf" -W -V "$lib") || bail "$readelf -V failed on $lib"
dynamic_out=$("$readelf" -W -d "$lib") || bail "$readelf -d failed on $lib"

# The GLIBC_ nodes among the versions the library needs (the .gnu.version_r
# "Version needs" entries, one per glibc symbol version). GLIBC_ABI_DT_RELR,
# present when the library was linked with -z pack-relative-relocs, is the
# one non-numeric node; only glibc 2.36 and newer understand the DT_RELR
# relocations it stands for, so it counts as a 2.36 requirement (it names
# no symbol, so the listing below cannot show it). A DT_RELR table without
# the marker (a linker driven with --pack-dyn-relocs=relr) has the same
# requirement and no load-time check at all, so it counts the same way.
version_needs=$(echo "$versions_out" |
	sed -n 's/.*Name: \(GLIBC_[A-Za-z0-9_.]*\).*/\1/p' | sort -u)
needed_versions=$(echo "$version_needs" |
	grep -E "^GLIBC_[0-9]+\.[0-9]+(\.[0-9]+)?$" || true)
other=$(echo "$version_needs" |
	grep -Ev "^GLIBC_[0-9]+\.[0-9]+(\.[0-9]+)?$" |
	grep -vx 'GLIBC_ABI_DT_RELR' || true)
[ -z "$other" ] || bail "unrecognised GLIBC_ version need(s) in $lib: $other"
relr=""
if echo "$version_needs" | grep -qx 'GLIBC_ABI_DT_RELR'; then
	relr="GLIBC_ABI_DT_RELR version need"
elif echo "$dynamic_out" | grep -q '(RELR)'; then
	relr="DT_RELR table without the GLIBC_ABI_DT_RELR version need"
fi
if [ -n "$relr" ]; then
	needed_versions=$(printf '%s\n%s\n' "$needed_versions" "GLIBC_2.36")
fi
highest=$(echo "$needed_versions" | sed 's/^GLIBC_//' | sort -V | tail -1)
[ -n "$highest" ] || bail "no GLIBC_ version needs found in $lib"

# "highest <= baseline" when the baseline sorts last (or equal) in version
# order.
last=$(printf '%s\n%s\n' "$highest" "$baseline" | sort -V | tail -1)
if [ "$last" = "$baseline" ]; then
	echo "ok 1 - $lib needs GLIBC_$highest at most" \
		"(baseline GLIBC_$baseline)"
else
	echo "# $lib requires GLIBC_$highest," \
		"above the baseline GLIBC_$baseline"
	echo "# symbols bound to versions above the baseline:"
	if [ -n "$relr" ]; then
		echo "#   $relr (packed relative relocations;" \
			"glibc 2.36 and newer)"
	fi
	"$readelf" -W --dyn-syms "$lib" | awk -v base="$baseline" '
		{
			# the "name@GLIBC_x.y" field; readelf may append " (n)"
			# after it
			sym = ""
			for (i = 1; i <= NF; i++)
				if ($i ~ /@GLIBC_[0-9.]+$/) sym = $i
			if (sym == "") next
			ver = sym; sub(/.*@GLIBC_/, "", ver)
			# version compare: split on dots, numeric
			n = split(ver, v, "."); m = split(base, b, ".")
			above = 0
			for (i = 1; i <= n || i <= m; i++) {
				x = (i <= n) ? v[i] + 0 : 0
				y = (i <= m) ? b[i] + 0 : 0
				if (x > y) { above = 1; break }
				if (x < y) { break }
			}
			if (above) print "#   " sym
		}' | sort -u
	echo "not ok 1 - $lib requires GLIBC_$highest," \
		"above the baseline GLIBC_$baseline"
fi

# NEEDED entries must all be on the allowed list, or be the dynamic linker
# itself (part of every glibc; RHEL and Ubuntu aarch64 toolchains make the
# library reference __stack_chk_guard from ld-linux-aarch64.so.1).
unexpected=""
needed=$(echo "$dynamic_out" | sed -n 's/.*(NEEDED).*\[\(.*\)\].*/\1/p')
[ -n "$needed" ] || bail "no NEEDED entries found in $lib"
for dep in $needed; do
	case " $allowed " in
	*" $dep "*) continue ;;
	esac
	case $dep in
	ld-linux-*.so.*|ld64.so.*|ld.so.*) continue ;;
	esac
	unexpected="$unexpected $dep"
done
if [ -z "$unexpected" ]; then
	echo "ok 2 - $lib NEEDED entries are all allowed ($allowed)"
else
	echo "# NEEDED entries not on the allowed list:"
	# shellcheck disable=SC2086
	printf '#   %s\n' $unexpected
	echo "not ok 2 - $lib has NEEDED entries outside the allowed list"
fi
exit 0
