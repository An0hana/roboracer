#include "mppi_controller/mppi_core.hpp"

#include <stdexcept>
#include <string>

#ifndef MPPI_CONTROLLER_HAS_CUDA_BACKEND
#define MPPI_CONTROLLER_HAS_CUDA_BACKEND 0
#endif

namespace mppi_controller
{

#if MPPI_CONTROLLER_HAS_CUDA_BACKEND
// Defined in mppi_cuda_backend.cu.  Keeping the CUDA implementation out of a
// public header lets every non-CUDA consumer include mppi_core.hpp normally.
std::unique_ptr<MppiBackend> makeMppiGenericCudaBackend(
  const MppiConfig & config, const VehicleConfig & vehicle);
#endif

bool cudaBackendCompiled() noexcept
{
#if MPPI_CONTROLLER_HAS_CUDA_BACKEND
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
#if MPPI_CONTROLLER_HAS_CUDA_BACKEND
    return makeMppiGenericCudaBackend(config, vehicle);
#else
    return makeCpuBackend(config, vehicle);
#endif
  }

  if (backend == "cuda") {
#if MPPI_CONTROLLER_HAS_CUDA_BACKEND
    return makeMppiGenericCudaBackend(config, vehicle);
#else
    throw std::runtime_error(
            "backend='cuda' requested, but mppi_controller was built without the "
            "MPPI-Generic CUDA backend");
#endif
  }

  throw std::invalid_argument(
          "unknown MPPI backend '" + backend +
          "' (expected auto, cuda, cpu, or cpu_reference)");
}

}  // namespace mppi_controller
