#!/bin/bash
#
# Apply a patch to a third-party dependency checked out by vcstool
# (see the top-level dep.repos). Run from within the dependency's directory.
#
# The apply is guarded so it is a no-op once the patch is in place, which keeps
# repeated `colcon build` invocations idempotent.
set -euo pipefail

patch_file="${1:?usage: apply_patch.bash <patch-file>}"

# Avoid Git's "dubious ownership" error when the dependency was cloned by a
# different user than the one building (common with container / CI bind mounts).
git config --global --add safe.directory "$(pwd)"

if [[ -z $(git status --porcelain) ]]; then
  git apply "${patch_file}"
fi
