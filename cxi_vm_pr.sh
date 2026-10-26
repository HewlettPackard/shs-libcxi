#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only or BSD-2-Clause
# Copyright 2020 Hewlett Packard Enterprise Development LP

set -e
git checkout -b rebase-test-branch
./autogen.sh
./configure --with-criterion=$(realpath ../Criterion/build/install)

# Once per PR: a version bump may land in a later commit than the symbols.
make check-symver || exit 1

# Always leave a TAP file for Jenkins and abort any unfinished rebase.
finish() {
	git rebase --abort 2>/dev/null || true
	rm -f "$gate"
	if [ -s commit-results.tap ]; then
		n=$(wc -l < commit-results.tap)
		sed -i "1s/^/TAP version 13\n1..$n\n/" commit-results.tap
		mv -f commit-results.tap tests/
	fi
}
trap finish EXIT
rm -f commit-results.tap
# Per-commit VM results; clear any left by an earlier build.
rm -rf tests/commit-taps
mkdir -p tests/commit-taps

# Per-commit gate, run via `git rebase --exec` (which rejects multi-line
# commands, so it goes in a temp file).
gate=$(mktemp)
cat > "$gate" <<'EOF'
#!/bin/bash
set -e
SHORT=$(git rev-parse --short HEAD)
trap 'echo "*** FAILED at commit $SHORT: $BASH_COMMAND" >&2' ERR
echo "=== commit $SHORT: $(git log -1 --format=%s) ==="
CWD=$PWD

if git log -1 | grep -q "Signed-off-by: "; then
	echo "ok - Commit ${SHORT} Signed-off" >> commit-results.tap
else
	echo "not ok - Commit ${SHORT} Signed-off" >> commit-results.tap
	echo "Commit ${SHORT} has no Signed-off-by line" >&2
	exit 1
fi

# Don't let the previous commit's results decide this one.
rm -f tests/libcxi_test.tap
make clean
make dist
make -j 8

# Judged by the results file, not ssh's exit status. The timeout runs on
# the ssh side so it can kill the VM; Jenkins can't reach it.
ssh -tt localhost "cd $CWD && exec timeout -k 30 15m make check" || true
# Keep a copy; cxi_vm_post.sh still needs the original.
if [ -f tests/libcxi_test.tap ]; then
	cp tests/libcxi_test.tap "tests/commit-taps/libcxi_test.${SHORT}.tap"
fi
if [ ! -f tests/libcxi_test.tap ]; then
	echo "not ok - Commit ${SHORT} Tests Status" >> commit-results.tap
	echo "VM tests produced no tests/libcxi_test.tap" >&2
	exit 1
fi
if grep -q "not ok" tests/libcxi_test.tap; then
	echo "not ok - Commit ${SHORT} Tests Status" >> commit-results.tap
	echo "VM tests failed; see tests/libcxi_test.tap" >&2
	exit 1
fi
# make check also runs the host-side tests; each result is in its .trs.
# This gate comes from the PR head and runs against every commit, so skip
# a test the commit doesn't have yet.
for t in contrib/check-stub-symbols.sh contrib/check-version-sync.sh \
	 tests/legacy_opcode_test; do
	[ -f "$t" ] || [ -f "$t.c" ] || continue
	if ! grep -q "^:global-test-result: PASS" "$t.trs"; then
		echo "not ok - Commit ${SHORT} Tests Status" \
			>> commit-results.tap
		echo "$t failed; see $t.log" >&2
		exit 1
	fi
done
echo "ok - Commit ${SHORT} Tests Status" >> commit-results.tap
EOF
chmod +x "$gate"
git rebase $(git merge-base origin/main HEAD) --exec "$gate"

# Once per PR: the baseline may be regenerated in a later commit.
make check-abi || exit 1
