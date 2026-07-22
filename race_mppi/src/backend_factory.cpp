#include "race_mppi/mppi_core.hpp"

#include <stdexcept>
#include <string>

#ifndef RACE_MPPI_HAS_CUDA_BACKEND
#define RACE_MPPI_HAS_CUDA_BACKEND 0
#endif

namespace race_mppi
{

#if RACE_MPPI_HAS_CUDA_BACKEND
// Defined in mppi_cuda_backend.cu.  Keeping the CUDA implementation out of a
// public header lets every non-CUDA consumer include mppi_core.hpp normally.
std::unique_ptr<MppiBackend> makeMppiGenericCudaBackend(
  const MppiConfig & config, const VehicleConfig & vehicle);
#endif

bool cudaBackendCompiled() noexcept
{
#if RACE_MPPI_HAS_CUDA_BACKEND
  return true;
#else
  return false;
#endif
}

std::unique_ptr<MppiBackend> makeBackend(
  const std::string & backend, const MppiConfig & config,
  const VehicleConfig & vehicle)
{
  if (backend == "cpu" || backend == "cpu_reference") {
    return makeCpuBackend(config, vehicle);
  }

  if (backend == "auto") {
#if RACE_MPPI_HAS_CUDA_BACKEND
    return makeMppiGenericCudaBackend(config, vehicle);
#else
    return makeCpuBackend(config, vehicle);
#endif
  }

  if (backend == "cuda") {
#if RACE_MPPI_HAS_CUDA_BACKEND
    return makeMppiGenericCudaBackend(config, vehicle);
#else
    throw std::runtime_error(
            "backend='cuda' requested, but race_mppi was built without the "
            "MPPI-Generic CUDA backend");
#endif
  }

  throw std::invalid_argument(
          "unknown MPPI backend '" + backend +
          "' (expected auto, cuda, cpu, or cpu_reference)");
}

}  // namespace race_mppi
