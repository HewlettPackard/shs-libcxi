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

a MAJOR or MINOR bump also adds a `LIBCXI_<MAJOR>.<MINOR>` symbol-version
stanza to `src/libcxi.sym` for the new symbols.

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

Libtool versions follow the format of Current:Revision:Age and they *do not* map to the project revision.
'Current' and 'Age' together define a range of interface numbers that are compatible (i.e. does not require recompilation of any consumer libraries or executables.).

Official Rules from GNU
- If the library source code has changed at all since the last update, then increment revision (‘c:r:a’ becomes ‘c:r+1:a’).
- If any interfaces have been added, removed, or changed since the last update, increment current, and set revision to 0.
- If any interfaces have been added since the last public release, then increment age.
- If any interfaces have been removed or changed since the last public release, then set age to 0.

As a rule of thumb, Age should only be incremented if the binary interface is backwards compatible.

These changes should be made in configure.ac.
See: ABI Versioning - Libtool

For more details on libtool versioning see:
https://www.gnu.org/software/libtool/manual/html_node/Libtool-versioning.html#Libtool-versioning
https://www.gnu.org/software/libtool/manual/html_node/Updating-version-info.html

Still confusing? This has further helpful clarifications:
https://autotools.io/libtool/version.html

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
