#include <glim/localization/localization_cpu.hpp>

extern "C" glim::LocalizationBase* create_localization_cpu() {
  glim::LocalizationCPUParams params;
  return new glim::LocalizationCPU(params);
}
