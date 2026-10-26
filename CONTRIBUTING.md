# Contributing to libcxi

This project follows the [Slingshot Host Software Governance Model](https://github.com/HewlettPackard/shs-governance-model).
Please review the governance model for general contribution guidelines, code of
conduct, security policies, and commit requirements (including DCO sign-off).
The sections below cover additional requirements specific to libcxi. See the
governance model's [review process](https://github.com/HewlettPackard/shs-governance-model/blob/main/CONTRIBUTING.md#pull-requests)
for how PRs are reviewed and integrated.

## Coding Style

libcxi follows the Linux kernel coding style. A git pre-commit hook is
available to validate commits automatically:

```
./contrib/install-git-hook.sh
```

The hook runs `checkpatch` on every commit. Commits with warnings or errors will
be rejected by the hook.

## Chained Sign-Offs

When the team merges an external contribution, reviewers add their own
`Signed-off-by` to the commit, following the Linux kernel convention of chained
sign-offs to document the patch's path from contributor to maintainer.

## Requirements Checklist

- [ ] Commits are signed off (`--signoff`).
- [ ] Change description and motivation are included in the PR.
- [ ] Testing approach or results are documented in the PR.
- [ ] Code follows the Linux kernel coding style (passes `checkpatch`).
- [ ] `make check`, `make check-symver` and `make check-abi` pass. If the
      change adds a public function, it is in a new version node
      (`LIBCXI_x.y`) in `src/libcxi.sym` and in `src/libcxi_stub.c`; the
      version in `configure.ac`, `include/libcxi.h` and `cray-libcxi.spec`
      and `NEWS` are updated; and `abi/libcxi.abi` is regenerated with
      `make update-abi` (see "Backwards compatibility checks" in
      README.md).
