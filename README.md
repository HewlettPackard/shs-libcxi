libcxi - The CXI library
=============================

The CXI library provides interfaces which interact directly with CXI drivers.
CXI drivers support communication over the Cray Cassini NIC.

How to build
------------

The libcxi project uses autotools.  To build, perform the following procedure:

```
./autogen.sh
./configure
make
```

How to test
-----------

The libcxi project contains unit tests written for the Criterion testing
framework (https://github.com/Snaipe/Criterion).  In order to build tests along
with the libcxi library, enable Criterion during configure:

```
./autogen.sh
./configure --with-criterion=<criterion_path>
make
```

To run the unit tests:

```
./tests/run_tests_vm.sh
```

To run the unit tests and remain in the VM:

```
./tests/run_tests_vm.sh -n
```

The Address Sanitizer (supported by gcc and clang) can also be enabled
by adding "--enable-asan" to the configure command line.

Building a link-only stub (--enable-stub-only)
----------------------------------------------

Container images that bind-mount the host's `libcxi.so.1` at runtime need
to *link* against libcxi at build time without the full build dependencies
(libconfig, libuv, fuse, yaml, libnl, numa, lm_sensors) and without a Cassini
device. `--enable-stub-only` builds and installs only `include/libcxi/libcxi.h`,
`libcxi.pc`, and a `libcxi.so.1` that exports every `CXIL_API` function with
the same `LIBCXI_x.y` version nodes as the real library:

```
./autogen.sh
./configure --enable-stub-only --prefix=/opt/libcxi
make
make install
```

The stub is link-only. Every function in it prints a diagnostic and aborts,
so a program that reaches the stub at runtime was started without the host
`libcxi.so.1` bind-mounted in front of it. The stub still needs the Cassini
and CXI driver UAPI headers to compile `libcxi.h` (pass them through
`CPPFLAGS` if they are not installed system-wide).

**Symbol versions decide which hosts a stub-linked binary can run on.** A
binary linked against the stub records a version-node requirement for every
libcxi function it references, for example `cxil_alloc_trig_cp@LIBCXI_1.9`.
`ld.so` refuses to load that binary on a host whose `libcxi.so.1` does not
define the node:

```
error while loading shared libraries: libcxi.so.1: version `LIBCXI_1.9' not found
```

That is the intended fail-fast behaviour, and it means a container may only
reference the symbols that exist on the *oldest* host it targets. The stub
exports the full current symbol set; it does not hide newer symbols. Audit a
binary with:

```
readelf -W --dyn-syms <binary> | grep -o '@LIBCXI_[0-9.]*' | sort -uV
```

Version nodes by release: `LIBCXI_1.0` through `LIBCXI_1.8` are present on
every SHS release from 11.x through 13.0; `LIBCXI_1.9` first shipped in SHS
13.1, `LIBCXI_1.10` in SHS 14.0, and `LIBCXI_1.11` through `LIBCXI_1.13` in
SHS 15.0. libfabric's CXI provider resolves its post-`LIBCXI_1.8` calls at
runtime with `dlsym()` (upstream libfabric and shs-libfabric with the
runtime-symbol change), so a stub-linked `libfabric.so` requires nothing above
`LIBCXI_1.5`; other consumers must check their own requirements.

Maintaining the stub: `src/libcxi_stub.c` must define exactly the `CXIL_API`
functions declared in `include/libcxi.h`. Every regular build compiles the stub
against the header with `-Werror` (`src/libcxi_stub_check.la`, never linked
into anything), which catches a changed signature, and `make check` runs
`contrib/check-stub-symbols.sh`, which reports a function present in the
header but not in the stub (or vice versa) and a header function that
`src/libcxi.sym` does not export. When adding a `CXIL_API` function, add it
to the header, to `src/libcxi.sym`, and to the stub in the same commit.

API Versioning
----------
The "API" / Overall Project revision will utilize Semantic versioning.

Given a version number MAJOR.MINOR.PATCH, increment the:

- MAJOR version when you make incompatible API changes,
- MINOR version when you add functionality in a backwards compatible manner, and
- PATCH version when you make backwards compatible bug fixes.

Every version bump changes:
- `configure.ac` (`api_major`, `api_minor`, `api_patch`),
- `include/libcxi.h` (`LIBCXI_API_MAJOR`, `LIBCXI_API_MINOR`, `LIBCXI_API_PATCH`), and
- `cray-libcxi.spec` (`Version:`);

a MAJOR or MINOR bump also adds a `LIBCXI_<MAJOR>.<MINOR>` version node
to `src/libcxi.sym` for the new symbols (see "Backwards compatibility
checks" below).

`make check` runs `contrib/check-version-sync.sh`, a TAP test that fails when
`include/libcxi.h` or `cray-libcxi.spec` disagree with `configure.ac`; CI runs
it for every commit (the symbol-version script has its own check). It is
deliberately not part of `configure`, which every downstream build runs. The
script can also be run by hand from the source tree; mismatches are reported
as `not ok` lines and the exit status is non-zero only when the script could
not run.

Further details can be found here:
https://semver.org/

Ensure NEWS is updated when the libcxi API version is updated.

ABI Versioning
--------------
For the "ABI" libtool versioning is used.

Libtool versions follow the format of Current:Revision:Age. In general they do not map to the API version; libcxi keeps them aligned (see below).
'Current' and 'Age' together define a range of interface numbers that are compatible (i.e. does not require recompilation of any consumer libraries or executables.).

Official Rules from GNU
- If the library source code has changed at all since the last update, then increment revision (‘c:r:a’ becomes ‘c:r+1:a’).
- If any interfaces have been added, removed, or changed since the last update, increment current, and set revision to 0.
- If any interfaces have been added since the last public release, then increment age.
- If any interfaces have been removed or changed since the last public release, then set age to 0.

As a rule of thumb, Age should only be incremented if the binary interface is backwards compatible.

libcxi follows these rules with one deliberate difference. Instead of
updating the version once per public release, every PR that adds functions
adds a new version node and bumps the version (see "Why each PR gets its
own version node" below). In `configure.ac`:

- `vers_current` and `api_minor` equal N of the newest `LIBCXI_1.N` version
  node (for example `13:0:12` is API 1.13).
- `vers_age` is bumped together with `vers_current`, so
  `vers_current - vers_age` stays 1 and the SONAME stays `libcxi.so.1`.
- `vers_revision` stays 0.

These changes should be made in configure.ac.
See: ABI Versioning - Libtool

For more details on libtool versioning see:
https://www.gnu.org/software/libtool/manual/html_node/Libtool-versioning.html#Libtool-versioning
https://www.gnu.org/software/libtool/manual/html_node/Updating-version-info.html

Still confusing? This has further helpful clarifications:
https://autotools.io/libtool/version.html

Backwards compatibility checks
------------------------------

### Why these exist

Users run libcxi inside containers. The libcxi in the container and the
kernel driver on the host come from different SHS releases, and both
directions have to work. Some changes are easy to get wrong without
noticing: an existing function that starts using a newer kernel command
fails on older drivers, and a new function added without updating the
library version makes two different libraries claim the same version.
Review alone does not reliably catch these, so these checks detect them
automatically in CI, before a release.

### Terms used below

| Term | Meaning |
|---|---|
| **Public function** | A function declared in `include/libcxi.h` for programs to call. |
| **ABI** | Everything an already-compiled program relies on when it calls libcxi: function names and signatures, the layout of the structs it passes, and the numbered commands libcxi sends to the kernel driver. A program built against an older libcxi keeps working only while the ABI it used is unchanged. |
| **Driver command (opcode)** | libcxi talks to the kernel driver by writing a small request that starts with a command number, the opcode. A driver only understands the opcodes that existed when it shipped. |
| **Version node** | The GNU linker manual's term for a block in `src/libcxi.sym` such as `LIBCXI_1.9 { ... }`. Each one lists the functions added by one change and names the node it follows. A program records the newest version node it uses and refuses to start against a library that lacks it. A version node that is already on the main branch is frozen, whether or not it has shipped: new functions go in a new version node. |
| **Library version numbers** | `vers_current`, `vers_age` and `api_minor` in `configure.ac`. They set the library's file name (for example `libcxi.so.1.12.0`) and the API version reported to packaging. By convention `vers_current` and `api_minor` equal N of the newest `LIBCXI_1.N` node. |
| **SONAME** | The `libcxi.so.1` part of the file name. Programs are linked against it, so it must not change for a compatible update. |
| **Baseline** | `abi/libcxi.abi`, a generated description of the ABI we last agreed to ship. |
| **PR base** | The commit your branch started from. The checks compare your change against it. |

### What a developer has to do differently

Nothing, unless the change adds or alters a public function or type.
An internal change that touches no public function or type passes all
the checks unchanged.

**Adding a new public function**

1. Declare it in `include/libcxi.h`, and add a stub for it to
   `src/libcxi_stub.c` (copy a neighbour: it reports the call and aborts).
2. Add it to a **new** version node at the end of `src/libcxi.sym`. Its number
   is the newest existing node's number plus one, it lists only the
   new function(s), and its closing line names the previous node:

   ```
   LIBCXI_1.14 {
   global:
   	cxil_my_new_function;
   } LIBCXI_1.13;
   ```

   Never add a function to a version node that is already on `main`, even
   if it has not shipped yet. Each PR that adds functions gets its own
   new version node; if another PR claims the same N first, rebase and
   renumber.
3. In `configure.ac` add 1 to each of `vers_current`, `vers_age` and
   `api_minor` (for example `13` / `12` / `13` becomes `14` / `13` /
   `14`), and make `LIBCXI_API_MINOR` in `include/libcxi.h` and
   `Version:` in `cray-libcxi.spec` match.
4. Add a line for it to `NEWS`.
5. Run `make update-abi` and commit the regenerated `abi/libcxi.abi`, so
   the new function shows up in the PR diff for review.

`check-stub-symbols.sh` reports a missing stub or `libcxi.sym` entry,
`check-symver` reports which of steps 2 to 4 are missing,
`check-version-sync.sh` reports a header or spec version that does not
match, and `check-abi` fails until step 5 is done. A new function is not
covered by `legacy_opcode_test` (see below); the other checks do cover
it.

**Other kinds of change**

| Change you are making | What to do | Enforced by |
|---|---|---|
| Make an existing function use a newer kernel command | Don't. Add a new function (above) and leave the old one sending what it always did. | `legacy_opcode_test` |
| Fix a bug inside an existing function | Fine, as long as its signature and the command it sends do not change. | |
| Change or add a field in an existing public struct | Don't change existing fields. Add a new struct, or a new function that takes one. | `check-abi` |
| Remove or rename a public function | Don't. Programs already built against libcxi depend on it. | `check-symver`, `check-abi` |
| Edit a version node that is already on `main` | Don't. Existing version nodes are frozen, shipped or not. | `check-symver` |

If a check fails and you believe the change is intentional, raise it
with the maintainers instead of editing the baseline or the check. A
change that breaks compatibility on purpose needs a new SONAME, not a
quiet update.

### Why each PR gets its own version node

A version node is a promise: "a library that has this node contains every
function in it". A program built against it records "I need LIBCXI_1.N",
and the dynamic loader checks only that the node exists.

If a node could gain functions after it landed on `main`, two different
libraries could carry the same node name. A program built against the later
one (using function B) would start on the earlier one (without B) and then
fail mid-run with an undefined symbol, the failure version nodes exist to
prevent. This is not hypothetical: every merged commit can become a build
(nightly packages, CI images, anyone building from `main`), not only
releases.

So the rule can be checked from git alone: a version node that already
exists on the PR base is frozen, and new functions go in a new node with its
own version bump. It needs no knowledge of what has shipped, a clash between
two PRs claiming the same N shows up as a merge conflict (the second one
renumbers) instead of a silent mistake, and it is cheap: node numbers are
free and the SONAME does not change.

The cost is more churn in `configure.ac`, `NEWS` and `libcxi.sym`, and a
release can span several `api_minor` values. libtool's own advice is to
update the version once just before a public release; that assumes
intermediate builds never leave the building, which is not true here.

### The checks

CI runs all of them on every pull request. `make check` runs the
host-side tests, which CI judges for every commit; `check-symver` and
`check-abi` compare against the PR base, so CI runs them once per pull
request (a version bump or baseline update may land in a later commit
than the change that needs it). Each can be run locally:

```
make check          # legacy_opcode_test, check-stub-symbols.sh,
                    # check-version-sync.sh (and the VM tests if available)
make check-symver   # libcxi.sym and configure.ac agree; existing version nodes unchanged
make check-abi      # built libcxi.so.1 matches abi/libcxi.abi
```

The `make check` tests report through TAP (`not ok` lines) and exit 0
when they ran; `check-symver` and `check-abi` print errors and exit
non-zero.

**`check-symver`** (`contrib/check-symver.sh`) reads `src/libcxi.sym` and
`configure.ac` at the PR base and in your working tree and reports any
existing version node that gained or lost a function, any new version
node without a version bump, and any version bump that does not match the
newest version node or would change the SONAME from `libcxi.so.1`. Every
error message says what to change; the rules are explained at the top of
the script.

**`check-abi`** (`contrib/check-abi.sh`) compares the freshly built
`libcxi.so.1` with the baseline, `abi/libcxi.abi` (every exported
function, its signature, and the layout of every public type). The
baseline is generated by `make update-abi` and never edited by hand. It
makes two comparisons:

- Against the baseline in your working tree. Any difference fails,
  including additions, so that extending the ABI always shows up in a PR
  diff for review.
- Against the baseline as it was at the PR base, ignoring additions.
  Anything removed or changed fails, so regenerating the baseline cannot
  hide an incompatible change.

Private structures from `include/libcxi_priv.h` are excluded via
`abi/libcxi.suppr`.

**`legacy_opcode_test`** (`tests/legacy_opcode_test.c`) calls each of the
45 functions in version nodes `LIBCXI_1.0` through `LIBCXI_1.8` (the API as of
SHS 12.0) and checks that it still sends the same driver command it sent
in SHS 12.0. It needs no device or VM. The list of functions is fixed:
anything added later goes in a newer version node and is checked by
`check-symver` and `check-abi`.

**`check-stub-symbols.sh`** (`contrib/`) checks that `src/libcxi_stub.c`
defines exactly the public functions in `include/libcxi.h` and that
`src/libcxi.sym` exports each of them once. See "Building a link-only stub"
above.

**`check-version-sync.sh`** (`contrib/`) checks that `LIBCXI_API_*` in
`include/libcxi.h` and `Version:` in `cray-libcxi.spec` match
`configure.ac`. See "API Versioning" above.

`make check-abi` and `make update-abi` need libabigail 2.3 or later
(`libabigail-tools` on SLES/openSUSE via PackageHub, `libabigail` on
RHEL/Fedora, `abigail-tools` on Debian/Ubuntu). The others have no
extra dependencies.

Submitting a patch
------------------

This project follows the linux coding style. A git pre-commit hook can
be installed by running

    ./contrib/install-git-hook.sh

It will run checkpatch for every commit. If the commit has warnings or
errors, the commit can't happen. That check can be disabled by adding
the --no-verify option to git commit, or by setting environment
variable GIT\_HOOKS\_PRE\_COMMIT\_HPE\_CRAY\_CHECKPATCH\_SKIP to a
non-zero numeric value.
