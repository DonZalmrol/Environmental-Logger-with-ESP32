#ifndef LOGGER_USER_CONFIG_H
#define LOGGER_USER_CONFIG_H

struct TubePresetDefinition {
  const char *id;
  const char *label;
  float deadTimeSeconds;
  float conversionFactorUsvPerCpm;
  float operatingVoltageMin;
  float operatingVoltageMax;
  const char *note;
};

// Tube presets are practical starting points for end users.
// Verify dose conversion and HV operating range against the exact tube batch,
// manufacturer datasheet, and your own calibration before using the station
// for anything beyond comparative background monitoring.
static constexpr TubePresetDefinition TUBE_PRESET_DEFINITIONS[] = {
  {"sbm19", "SBM-19", 0.000250f, 0.001500f, 350.0f, 475.0f, "Low-sensitivity tube preset used as the project default."},
  {"sbm20", "SBM-20", 0.000190f, 0.006315f, 350.0f, 475.0f, "Common general-purpose GM tube preset."},
  {"sts5", "STS-5", 0.000190f, 0.006315f, 350.0f, 475.0f, "Preset aligned with SBM-20 style behavior; verify against your tube batch."},
  {"sbt10", "SBT-10", 0.000190f, 0.013500f, 350.0f, 450.0f, "Higher-sensitivity starting preset; confirm calibration before relying on dose output."},
  {"sts6", "STS-6", 0.000190f, 0.006315f, 380.0f, 475.0f, "Starting preset for STS-6 tubes; verify final conversion against measured response."},
  {"si22g", "SI-22G", 0.000190f, 0.001714f, 380.0f, 475.0f, "SI-22G preset using the existing project conversion factor note."},
  {"lnd712", "LND-712", 0.000090f, 0.005940f, 500.0f, 600.0f, "LND-712 gamma tube starting preset; confirm dose conversion with your own calibration source."},
  {"lnd7317", "LND-7317", 0.000090f, 0.002100f, 450.0f, 550.0f, "LND-7317 thin-wall tube starting preset; verify final conversion and plateau on your hardware."},
  {"si3bg", "SI-3BG", 0.000190f, 0.006315f, 380.0f, 475.0f, "SI-3BG starting preset aligned to the existing Soviet-tube defaults; verify against datasheet and measured response."},
};

static constexpr size_t TUBE_PRESET_DEFINITIONS_COUNT = sizeof(TUBE_PRESET_DEFINITIONS) / sizeof(TUBE_PRESET_DEFINITIONS[0]);

static constexpr const char *LOGGER_DEFAULT_TUBE_PRESET_ID = "sbm19";
static constexpr float LOGGER_DEFAULT_TUBE_DEAD_TIME_SECONDS = 0.000250f;
static constexpr float LOGGER_DEFAULT_TUBE_CONVERSION_FACTOR_USV_PER_CPM = 0.001500f;

#endif