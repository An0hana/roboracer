# mppi_generic_vendor

This package pins MPPI-Generic `v0.9.0` to commit
`b5c8daab6a9157ff9342d5a74838535c1b2431f7`. It never follows the upstream
default branch.

On a non-CUDA development machine the package installs only its lock and
license metadata so the CPU reference controller and unit tests remain
buildable. On Jetson Orin, build the CUDA dependency with:

```bash
colcon build --packages-select mppi_generic_vendor \
  --cmake-args -DMPPI_GENERIC_VENDOR_ENABLE_CUDA=ON \
               -DMPPI_GENERIC_CUDA_ARCH=87
```

The first CUDA build needs network access to clone the exact locked commit and
its `cnpy` submodule. Preserve the resulting source/build cache for offline
competition deployment.
