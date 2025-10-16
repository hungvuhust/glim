#include <glim/localization/localization_cpu.hpp>

extern "C" glim::LocalizationBase* create_localization_module() {
  glim::LocalizationCPUParams params;
  return new glim::LocalizationCPU(params);
}
