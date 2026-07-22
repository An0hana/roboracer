#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "race_mppi/mppi_core.hpp"

namespace race_mppi
{
namespace
{

TEST(BackendFactory, ExplicitCpuAlwaysSelectsReferenceBackend)
{
  auto backend = makeBackend("cpu", MppiConfig{}, VehicleConfig{});
  ASSERT_NE(backend, nullptr);
  EXPECT_EQ(backend->name(), "cpu_reference");
}

TEST(BackendFactory, AutoReportsTheBackendActuallyCompiled)
{
  auto backend = makeBackend("auto", MppiConfig{}, VehicleConfig{});
  ASSERT_NE(backend, nullptr);
  if (cudaBackendCompiled()) {
    EXPECT_EQ(backend->name(), "cuda_mppi_generic_v0.9.0");
  } else {
    EXPECT_EQ(backend->name(), "cpu_reference");
  }
}

TEST(BackendFactory, ExplicitCudaNeverSilentlyFallsBack)
{
  if (cudaBackendCompiled()) {
    auto backend = makeBackend("cuda", MppiConfig{}, VehicleConfig{});
    ASSERT_NE(backend, nullptr);
    EXPECT_EQ(backend->name(), "cuda_mppi_generic_v0.9.0");
  } else {
    EXPECT_THROW(
      (void)makeBackend("cuda", MppiConfig{}, VehicleConfig{}),
      std::runtime_error);
  }
}

TEST(BackendFactory, RejectsUnknownBackend)
{
  EXPECT_THROW(
    (void)makeBackend("definitely_not_a_backend", MppiConfig{}, VehicleConfig{}),
    std::invalid_argument);
}

}  // namespace
}  // namespace race_mppi

