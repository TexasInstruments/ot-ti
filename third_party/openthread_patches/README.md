# OpenThread patches

Patches applied to the `openthread` submodule by `script/apply-openthread-patches`, which
`script/build` runs before configuring. A patch that is already applied is skipped.

| Patch | Purpose |
|-------|---------|
| `0001-ncp-reset-the-NCP-state-in-place-on-Spinel-RESET.patch` | Handle Spinel `RESET` in place instead of resetting the device, for multi-stack DMM builds |

## Updating the openthread submodule

The patches are made against the revision pinned by the submodule. After changing it, check
with `script/apply-openthread-patches`. If a patch no longer applies, port the change to the new
revision and regenerate the patch with `git format-patch -1`.
