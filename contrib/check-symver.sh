#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only or BSD-2-Clause
# Copyright 2026 Hewlett Packard Enterprise Development LP
#
# check-symver.sh - keep libcxi's two ABI version promises in agreement
#
# Why this exists
# ---------------
# A shared library's version is a promise to programs that were already
# compiled against it: "everything you used is still here and still works
# the same". libcxi makes that promise in two files, and they must agree:
#
#   src/libcxi.sym   Every public function, listed in "version nodes"
#                    (the GNU linker manual's term): one block per release
#                    that added functions, e.g. LIBCXI_1.9 { ... }. A
#                    program compiled against 13.1 records "I need
#                    LIBCXI_1.9"; the dynamic loader refuses to start it
#                    against a library that lacks that version node.
#
#   configure.ac     The libtool version (vers_current:0:vers_age) that
#                    becomes the file name, e.g. libcxi.so.1.12.0, and the
#                    API version (1.api_minor.0) that packaging reports.
#
# It is easy to add a version node to libcxi.sym and forget configure.ac,
# leaving two different libraries with the same version. Review alone does
# not reliably catch this, so this script checks it automatically in CI.
#
# What a developer does
# ---------------------
#   Adding a public function     Put it in a NEW version node at the end of
#                                libcxi.sym, LIBCXI_1.N where N is one more
#                                than the last node. In configure.ac add 1
#                                to each of vers_current, vers_age and
#                                api_minor, and match LIBCXI_API_MINOR in
#                                include/libcxi.h and Version: in
#                                cray-libcxi.spec. Add the function to
#                                src/libcxi_stub.c and a line to NEWS.
#
#   Changing what an existing    Don't. Add a new function (as above) and
#   function does                leave the old one working as before.
#
#   Removing a public function   Don't. Programs out there depend on it.
#
#   Editing a version node that  Don't. A version node that is already on
#   is already on the main       the main branch is frozen, whether or not
#   branch                       it has shipped. A new function always gets
#                                its own new version node, so each PR that
#                                adds functions bumps the version. If
#                                another PR claims the same N first, rebase
#                                and renumber.
#
# Why "already on the main branch" and not "already shipped": every merged
# commit can become a build (nightly packages, CI images, anyone building
# from main). If a version node could gain functions after landing, two
# different libraries would carry the same node name, and a program built
# against the later one would start on the earlier one and then fail with
# an undefined symbol. That is exactly what version nodes exist to prevent.
#
# What this script checks, against the PR base (merge-base with origin/main)
# ---------------------------------------------------------------------------
#   - No existing version node gained or lost a function, or changed which
#     version node it inherits from.
#   - Every new version node has at least one function.
#   - If a version node was added, configure.ac was bumped.
#   - If configure.ac was bumped: vers_current and api_minor equal N of the
#     newest LIBCXI_1.N version node (so the two files cannot drift apart
#     again),
#     vers_current did not go backwards, and vers_current - vers_age is
#     unchanged (that difference is the "1" in libcxi.so.1; changing it
#     would make every existing program fail to load).
#
# These are GNU libtool's standard versioning rules plus the convention,
# adopted with the 1.13.0 bump, that vers_current tracks the newest node.
# See README.md, "Backwards compatibility checks".
#
# Usage: contrib/check-symver.sh [-p BASE_REF] [-b HEAD_REF] [-q]
#        make check-symver [SYMVER_BASE=<ref>]

set -o errexit
set -o pipefail

SYM=src/libcxi.sym
CONF=configure.ac

eprintf() {
	# shellcheck disable=SC2059
	printf "$@" >&2
}

usage() {
	cat <<EOF
Usage: $(basename "$0") [-p BASE_REF] [-b HEAD_REF] [-q]

Check that src/libcxi.sym and the library version in configure.ac
agree, and that no existing version node was changed. The rules and
what to do about each error are explained at the top of this script.

  -p BASE_REF  ref to compare against (default: merge-base of HEAD and
               \$UPSTREAM_BRANCH, which defaults to origin/main)
  -b HEAD_REF  ref to check (default: working tree)
  -q           quiet on success
  -h           this help
EOF
}

# Print file contents at a ref, or from the working tree if ref is empty
at_ref() {
	local ref="$1" path="$2"

	if [ -z "$ref" ]; then
		cat "$path"
	else
		git show "${ref}:${path}"
	fi
}

# Emit "node function" and "node >previous-node" lines from libcxi.sym
parse_sym() {
	awk '
		/^LIBCXI_[0-9]+\.[0-9]+ \{/ { node = $1; next }
		/^\}/ {
			pred = $2; sub(/;/, "", pred)
			print node, ">" (pred == "" ? "-" : pred)
			node = ""
			next
		}
		node != "" && /^\t[a-z_][a-zA-Z0-9_]*;/ {
			sym = $1; sub(/;/, "", sym)
			print node, sym
		}
	'
}

# Print the numeric value of an m4_define([name],[N]) from configure.ac text
m4_val() {
	sed -n "s/^m4_define(\[$1\],\[\([0-9]*\)\])/\1/p"
}

# Highest N among the LIBCXI_1.N nodes in libcxi.sym
highest_node() {
	grep -oE '^LIBCXI_1\.[0-9]+' | sed 's/LIBCXI_1\.//' | sort -n | tail -1
}

errors=0
err() {
	# shellcheck disable=SC2059
	printf "ERROR: $1\n" "${@:2}" >&2
	errors=$((errors + 1))
}

note() {
	# shellcheck disable=SC2059
	printf "  $1\n" "${@:2}" >&2
}

# Functions (and ">previous-node") of one node from the parsed output
node_members() {
	echo "$1" | awk -v n="$2" '$1 == n {print $2}'
}

node_names() {
	echo "$1" | awk '{print $1}' | sort -u
}

main() {
	local base_ref="" head_ref="" quiet=false upstream
	local base_sym head_sym base_conf head_conf
	local node b h new_nodes=0 conf_changed=false
	local b_cur b_age b_minor h_cur h_age h_minor high

	while getopts "hp:b:q" opt; do
		case $opt in
		h) usage; exit 0 ;;
		p) base_ref="$OPTARG" ;;
		b) head_ref="$OPTARG" ;;
		q) quiet=true ;;
		*) usage; exit 1 ;;
		esac
	done

	cd "$(git rev-parse --show-toplevel)"

	upstream="${UPSTREAM_BRANCH:-origin/main}"
	if [ -z "$base_ref" ] &&
	   ! base_ref="$(git merge-base "$upstream" HEAD 2>/dev/null)"; then
		eprintf "error - no merge-base of HEAD and %s\n" "$upstream"
		eprintf "Set UPSTREAM_BRANCH or pass -p BASE_REF\n"
		exit 1
	fi
	if ! git rev-parse --verify -q "${base_ref}^{commit}" >/dev/null; then
		eprintf "error - unknown git ref '%s'\n" "$base_ref"
		exit 1
	fi

	base_sym="$(at_ref "$base_ref" "$SYM" | parse_sym | sort)"
	head_sym="$(at_ref "$head_ref" "$SYM" | parse_sym | sort)"
	base_conf="$(at_ref "$base_ref" "$CONF")"
	head_conf="$(at_ref "$head_ref" "$CONF")"

	# Every node that existed at the base must be unchanged
	for node in $(node_names "$base_sym"); do
		b="$(node_members "$base_sym" "$node")"
		h="$(node_members "$head_sym" "$node")"
		if [ -z "$h" ]; then
			err "version node %s was removed" "$node"
			note "Existing version nodes must stay."
		elif [ "$b" != "$h" ]; then
			err "existing version node %s changed:" "$node"
			diff <(echo "$b") <(echo "$h") |
				sed -n 's/^</  removed: /p
					s/^>/  added:   /p' >&2 || true
			note "Existing version nodes are frozen. Put new"
			note "functions in a new LIBCXI_1.N version node."
		fi
	done

	# New nodes must list functions, and need a configure.ac bump
	for node in $(comm -13 <(node_names "$base_sym") \
			       <(node_names "$head_sym")); do
		new_nodes=$((new_nodes + 1))
		h="$(node_members "$head_sym" "$node" | grep -v '^>' || true)"
		[ -n "$h" ] ||
			err "new version node %s lists no functions" "$node"
	done

	b_cur=$(echo "$base_conf" | m4_val vers_current)
	b_age=$(echo "$base_conf" | m4_val vers_age)
	b_minor=$(echo "$base_conf" | m4_val api_minor)
	h_cur=$(echo "$head_conf" | m4_val vers_current)
	h_age=$(echo "$head_conf" | m4_val vers_age)
	h_minor=$(echo "$head_conf" | m4_val api_minor)
	high=$(at_ref "$head_ref" "$SYM" | highest_node)

	if [ "$b_cur$b_age$b_minor" != "$h_cur$h_age$h_minor" ]; then
		conf_changed=true
	fi

	if [ "$new_nodes" -gt 0 ] && [ "$conf_changed" = false ]; then
		err "%d new version node(s), configure.ac unchanged" \
			"$new_nodes"
		note "Add 1 to each of vers_current, vers_age and api_minor,"
		note "match include/libcxi.h and cray-libcxi.spec, and list"
		note "the new functions in NEWS."
	fi

	# When the version fields change they must be internally consistent
	if [ "$conf_changed" = true ]; then
		if [ "$h_cur" -lt "$b_cur" ]; then
			err "vers_current went backwards (%s -> %s)" \
				"$b_cur" "$h_cur"
			note "Versions only go up."
		fi
		if [ $((h_cur - h_age)) -ne $((b_cur - b_age)) ]; then
			err "vers_current - vers_age changed (%d -> %d)" \
				$((b_cur - b_age)) $((h_cur - h_age))
			note "That difference is the 1 in libcxi.so.1; changing"
			note "it breaks every existing program. Bump vers_age"
			note "by the same amount as vers_current."
		fi
		if [ "$h_cur" != "$high" ]; then
			err "vers_current is %s, newest node is LIBCXI_1.%s" \
				"$h_cur" "$high"
			note "vers_current must equal N of the newest node."
		fi
		if [ "$h_minor" != "$high" ]; then
			err "api_minor is %s, newest node is LIBCXI_1.%s" \
				"$h_minor" "$high"
			note "api_minor must equal N of the newest node."
		fi
	fi

	if [ "$errors" -gt 0 ]; then
		eprintf "%d problem(s); see the rules at the top of %s\n" \
			"$errors" "$0"
		exit 1
	fi

	[ "$quiet" = true ] && return 0
	if [ "$new_nodes" -gt 0 ] || [ "$conf_changed" = true ]; then
		printf "Symbol versions OK: %d new node(s), %s:0:%s (1.%s)\n" \
			"$new_nodes" "$h_cur" "$h_age" "$h_minor"
	else
		printf "Symbol versions OK: %s and version unchanged\n" "$SYM"
	fi
}

main "$@"
