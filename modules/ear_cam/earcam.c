/*
 * Copyright 2026 OpenUAS
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define _POSIX_C_SOURCE 200809L
#include "version.h"

/**
 * @file earcam.c
 * @brief ALSA acoustic detector for motionSCOUT-style narrow-band alarm tones.
 * @details EARcam captures mono PCM, isolates the configured frequency band with
 * cascaded Butterworth biquads, measures a bank of Goertzel tone bins over short Hann
 * windows, and emits either an operator display or machine-readable samples for USHER.
 * The process remains independently useful for bench work and deliberately keeps the
 * USHER server protocol simple: newline-delimited output plus STOP/QUIT input.
 */

#include <alsa/asoundlib.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "ear_inference.h"
#include "../../src/inference_settings.h"

// "auto" = first USB audio card that offers a capture PCM (any vendor/device id).
#define DEFAULT_DEVICE "auto"
#define AUTO_DEVICE_NAME_MAX 64U
#define DEFAULT_RATE 48000U
#define DEFAULT_INTERVAL_MS 50U
#define MIN_RATE_HZ 8000U
#define MIN_INTERVAL_MS 20U
#define MAX_RATE 384000U
#define MAX_INTERVAL_MS 10000U
#define MILLISECONDS_PER_SECOND 1000U
#define MICROSECONDS_PER_SECOND 1000000U
#define NANOSECONDS_PER_SECOND 1000000000L
#define NANOSECONDS_PER_MILLISECOND 1000000U
#define NANOSECONDS_PER_MICROSECOND 1000U
#define MILLISECONDS_PER_SECOND_F 1000.0
/** ALSA asks for 5 ms periods and a 20 ms buffer; accepted periods remain bounded. */
#define CAPTURE_PERIODS_PER_SECOND 200U
#define CAPTURE_BUFFERS_PER_SECOND 50U
#define CAPTURE_MONO_CHANNELS 1U
#define CAPTURE_WAIT_MS 100
#define CAPTURE_STALL_TIMEOUT_MS 2000U
#define MAX_INTERVAL_FRAMES ((MAX_RATE * MAX_INTERVAL_MS) / MILLISECONDS_PER_SECOND)
#define MAX_PERIOD_FRAMES (MAX_RATE / CAPTURE_BUFFERS_PER_SECOND)
/** Guard opaque ALSA ABI sizes at runtime rather than using direct EARcam heap storage. */
#define ALSA_OBJECT_CAPACITY 4096U
#define EARCAM_MAX_ARGUMENTS 64
#define EARCAM_MAX_PARSED_ARGUMENTS (EARCAM_MAX_ARGUMENTS * 2 + 1)
#define DEFAULT_LOW_CUT_HZ 1800.0
#define DEFAULT_HIGH_CUT_HZ 3200.0
/** Level and contrast gates operate before calibration; clipping is reported separately. */
#define DEFAULT_MIN_LEVEL_DBFS (-90.0)
#define DB_FLOOR (-120.0)
#define METER_WIDTH 50U
#define TONE_BIN_COUNT 57U
#define TONE_CONTRAST_THRESHOLD_DB (-3.0)
#define TREND_THRESHOLD_DB 0.8
#define ENVELOPE_RELEASE_DB_PER_SECOND 6.0
/** Bench distance display assumes this reference source and free-field attenuation,
 * not a measured range. Use only with an independently calibrated microphone chain. */
#define SOURCE_LEVEL_DB_SPL 95.0
#define SOURCE_REFERENCE_METERS 3.0
#define CLIP_THRESHOLD 32700
#define PI 3.14159265358979323846

/** Parsing bounds retain room below the maximum supported sample-rate Nyquist limit. */
#define MAX_CUTOFF_HZ 190000.0
#define MIN_CALIBRATION_DB -300.0
#define MAX_CALIBRATION_DB 300.0
#define CONTROL_DEVICE_NAME_CAPACITY 32
#define SHORT_OPTION_STORAGE 3
#define SERVER_SAMPLE_LINE_CAPACITY 384
#define SERVER_COMMAND_CAPACITY 64
#define SERVER_READ_CHUNK_CAPACITY 64
/** q = 1/sqrt(2) for each second-order Butterworth section; two sections per edge. */
#define BUTTERWORTH_Q 0.7071067811865476
#define FILTER_SECTIONS_PER_EDGE 2
#define PCM_S16_FULL_SCALE 32768.0
#define AMPLITUDE_FLOOR 0.000001
#define AMPLITUDE_DB_PER_DECADE 20.0
#define DB_LOG_BASE 10.0
/** Hann coherent gain and sinusoid amplitude recovery combine to this factor. */
#define HANN_TONE_AMPLITUDE_SCALE 4.0
/** Fast/slow peak-envelope smoothing in seconds; minimum alarm windows avoids early labels. */
#define HOMING_FAST_TIME_CONSTANT_S 0.20
#define HOMING_SLOW_TIME_CONSTANT_S 1.20
#define HOMING_ACQUISITION_WINDOWS 4U
/** Deterministic self-test fixtures, not live detector thresholds or measured calibration. */
#define TEST_TIMESTAMP_US 1000000
#define TEST_UNDERFLOW_TIMESTAMP_US 100
#define TEST_UNREAD_FRAMES 480
#define TEST_RATE_HZ 48000
#define TEST_EXPECTED_READ_END_US 990000
#define TEST_TARGET_HZ 2700.0
#define TEST_INTERFERENCE_HZ 800.0
#define TEST_TARGET_AMPLITUDE 0.02
#define TEST_INTERFERENCE_AMPLITUDE 0.20
#define TEST_TONE_MIN_DB -36.0
#define TEST_TONE_MAX_DB -32.0
#define TEST_CONTRAST_MAX_DB -15.0
#define TEST_TREND_WINDOWS 40U
#define TEST_APPROACH_START_DB -65.0
#define TEST_RECEDE_START_DB -45.0
#define TEST_TREND_STEP_DB 0.3

/**
 * @brief Validated capture and reporting policy, fixed before the sample loop.
 * @details Calibration is a caller-supplied offset, not automatic microphone
 * calibration. It changes reported levels, not the dBFS alarm decision.
 */
struct options {
  const char *device;          /**< Borrowed ALSA PCM name; auto selects a USB capture card. */
  unsigned int rate;           /**< Exact requested sample rate, hertz. */
  unsigned int interval_ms;    /**< Analysis/report window duration, milliseconds. */
  double low_cut_hz;           /**< High-pass edge and first tone-bank frequency, hertz. */
  double high_cut_hz;          /**< Low-pass edge and last tone-bank frequency, hertz. */
  double min_level_dbfs;       /**< Absolute uncalibrated tone gate before contrast testing. */
  double calibration_db;       /**< Additive report offset from dBFS to caller-calibrated dB SPL. */
  bool calibrated;            /**< Caller explicitly supplied the calibration offset. */
  bool self_test;             /**< Run deterministic software checks without opening ALSA. */
  bool server;                /**< Use nonblocking line protocol instead of a terminal meter. */
  const char *settings, *audio_file;
  bool onnx, check_model;
};

/** @brief Normalized biquad coefficients and transposed-direct-form-II delay state. */
struct biquad {
  double b0;
  double b1;
  double b2;
  double a1;
  double a2;
  double z1;
  double z2;
  bool enabled;
};

/** @brief One Goertzel recurrence; state resets at each analysis-window boundary. */
struct tone_bin {
  double coefficient;
  double first;
  double second;
  double frequency_hz;
};

/** @brief Peak hold and fast/slow averages; direction is guidance, not a bearing measurement. */
struct homing_state {
  double envelope_db;
  double fast_db;
  double slow_db;
  unsigned int alarm_windows;
  bool initialized;
};

static volatile sig_atomic_t keep_running = 1;

/** @brief Aligned bounded storage for an opaque ALSA object after its sizeof guard succeeds. */
union alsa_object_storage {
  max_align_t alignment;
  unsigned char bytes[ALSA_OBJECT_CAPACITY];
};

static union alsa_object_storage card_info_storage;
static union alsa_object_storage pcm_info_storage;
static union alsa_object_storage hardware_params_storage;
static union alsa_object_storage software_params_storage;
/* The largest supported configuration sets workspace bounds. Only the active
 * period/window is touched; changing the CLI maxima also changes static RAM cost. */
static int16_t capture_samples[MAX_PERIOD_FRAMES];
static double analysis_window[MAX_INTERVAL_FRAMES];
static bool capture_timestamps;

/**
 * @brief Subtract unread PCM duration from a monotonic ALSA timestamp.
 * @return End of consumed audio in microseconds, or zero if the calculation is unusable.
 * @details Arrival time alone includes buffered audio delay. This correction lets
 * USHER associate a window with flight poses closer to its actual capture time.
 */
static uint64_t audio_read_end_us(uint64_t hardware_us, uint64_t unread_frames, unsigned int rate)
{
  if (!rate || unread_frames > UINT64_MAX / MICROSECONDS_PER_SECOND) return 0;
  const uint64_t delay_us = unread_frames * MICROSECONDS_PER_SECOND / rate;
  return hardware_us > delay_us ? hardware_us - delay_us : 0;
}

/** @brief Long-to-short option mapping; borrowed values are passed to standard getopt. */
struct option_alias {
  const char *name;
  char option[SHORT_OPTION_STORAGE];
  bool requires_argument;
};

/**
 * @brief Normalize --name value and --name=value into bounded getopt arguments.
 * @return Argument count, or -1 for unknown options, missing values or capacity exhaustion.
 * @details Pointers borrow argv/static alias strings; no copied argument payload or heap is needed.
 */
static int normalize_long_options(int argc, char *const argv[], char *normalized[],
                                  size_t capacity, const struct option_alias aliases[],
                                  size_t alias_count)
{
  if (argc < 1 || argc > EARCAM_MAX_ARGUMENTS || capacity == 0) {
    return -1;
  }
  size_t output_count = 0;
  normalized[output_count++] = argv[0];
  for (int index = 1; index < argc; ++index) {
    const char *argument = argv[index];
    if (strcmp(argument, "--") == 0) {
      if (output_count >= capacity) {
        return -1;
      }
      normalized[output_count++] = argv[index];
      for (++index; index < argc; ++index) {
        if (output_count >= capacity) {
          return -1;
        }
        normalized[output_count++] = argv[index];
      }
      break;
    }
    if (strncmp(argument, "--", 2) != 0 || argument[2] == '\0') {
      if (output_count >= capacity) {
        return -1;
      }
      normalized[output_count++] = argv[index];
      continue;
    }
    const char *name = argument + 2;
    size_t name_length = strcspn(name, "=");
    const char *inline_value = name[name_length] == '=' ? name + name_length + 1 : NULL;
    const struct option_alias *alias = NULL;
    for (size_t alias_index = 0; alias_index < alias_count; ++alias_index) {
      if (strlen(aliases[alias_index].name) == name_length
          && strncmp(name, aliases[alias_index].name, name_length) == 0) {
        alias = &aliases[alias_index];
        break;
      }
    }
    if (alias == NULL || (!alias->requires_argument && inline_value != NULL)
        || output_count >= capacity) {
      return -1;
    }
    normalized[output_count++] = (char *)alias->option;
    if (alias->requires_argument) {
      if (inline_value == NULL) {
        if (++index >= argc) {
          return -1;
        }
        inline_value = argv[index];
      }
      if (output_count >= capacity) {
        return -1;
      }
      normalized[output_count++] = (char *)inline_value;
    }
  }
  if (output_count >= capacity) {
    return -1;
  }
  normalized[output_count] = NULL;
  return (int)output_count;
}

/** @brief Request orderly capture-loop termination from a signal handler.
 * @param signal_number Delivered signal, intentionally unused.
 * @details sig_atomic_t is used because complex cleanup is unsafe in signal context. */
static void handle_signal(int signal_number)
{
  (void)signal_number;
  keep_running = 0;
}

/** @brief Return monotonic milliseconds for correlating samples without wall-clock jumps.
 * @return Monotonic timestamp in milliseconds. */
static uint64_t monotonic_ms(void)
{
  struct timespec now;
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0 || now.tv_sec < 0 || now.tv_nsec < 0 ||
      now.tv_nsec >= NANOSECONDS_PER_SECOND) return 0;
  return (uint64_t)now.tv_sec * MILLISECONDS_PER_SECOND
    + (uint64_t)now.tv_nsec / NANOSECONDS_PER_MILLISECOND;
}

/** @brief Print command-line help or version-led usage text.
 * @param stream Destination stream.
 * @param program Executable name shown in examples. */
static void print_usage(FILE *stream, const char *program)
{
  fprintf(stream, "%s\n", EARCAM_BUILD_VERSION);
  fprintf(stream, "Model options: --settings FILE, --check-model, --analyze-audio MONO_S16LE.pcm\n");
  fprintf(stream,
          "Usage: %s [options]\n"
          "Measure microphone level and display it in the terminal.\n\n"
          "Options:\n"
          "  -d, --device NAME       ALSA capture device (default: %s = first USB capture card)\n"
          "  -r, --rate HZ           Sample rate (default: %u)\n"
          "  -i, --interval MS       Display interval (default: %u)\n"
          "  -l, --low-cut HZ        Search band lower edge (default: %.0f)\n"
          "  -u, --high-cut HZ       Search band upper edge (default: %.0f)\n"
          "  -n, --min-level DBFS    Minimum alarm tone level (default: %.0f)\n"
          "  -c, --calibration DB    Add DB to dBFS readings to report dB SPL\n"
          "      --server            Machine-readable SAMPLE lines for USHER; STOP/QUIT on stdin\n"
          "      --self-test         Test detector without a microphone\n"
          "      --version           Show application version and build Git revision\n"
          "  -h, --help              Show this help\n\n"
          "Examples:\n"
          "  %s --device plughw:1,0\n"
          "  %s -d default           motionSCOUT rescue profile\n"
          "  arecord -L              List ALSA capture device names\n",
          program, DEFAULT_DEVICE, DEFAULT_RATE, DEFAULT_INTERVAL_MS,
          DEFAULT_LOW_CUT_HZ, DEFAULT_HIGH_CUT_HZ, DEFAULT_MIN_LEVEL_DBFS,
          program, program);
}

/** @brief Parse a bounded unsigned command-line value.
 * @return 0 on success, otherwise -1 without modifying the output value. */
static int parse_unsigned(const char *text, unsigned int minimum,
                          unsigned int maximum, unsigned int *value)
{
  char *end = NULL;
  unsigned long parsed;

  errno = 0;
  parsed = strtoul(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0' || parsed < minimum ||
      parsed > maximum) {
    return -1;
  }
  *value = (unsigned int)parsed;
  return 0;
}

// Find the first USB audio card with a capture PCM. Writes "hw:CARD=<id>,DEV=<n>"
// into name; returns 0 on success, -1 when no USB capture device is present.
/** @brief Discover the first USB ALSA card offering a capture PCM.
 * @param name Destination ALSA device name.
 * @param name_size Capacity of @p name.
 * @return 0 when a usable device is found, otherwise -1.
 * @details USB-Audio matching prevents an arbitrary onboard sound device from silently
 * becoming the payload sensor when `--device auto` is used. */
static int find_usb_capture_device(char *name, size_t name_size)
{
  if (snd_ctl_card_info_sizeof() > sizeof(card_info_storage.bytes)
      || snd_pcm_info_sizeof() > sizeof(pcm_info_storage.bytes)) {
    errno = EOVERFLOW;
    return -1;
  }
  snd_ctl_card_info_t *info = (snd_ctl_card_info_t *)card_info_storage.bytes;
  snd_pcm_info_t *pcm_info = (snd_pcm_info_t *)pcm_info_storage.bytes;
  int found = -1;
  int card = -1;
  while (found != 0 && snd_card_next(&card) == 0 && card >= 0) {
    char control_name[CONTROL_DEVICE_NAME_CAPACITY];
    if (snprintf(control_name, sizeof(control_name), "hw:%d", card) <= 0) {
      continue;
    }
    snd_ctl_t *control = NULL;
    if (snd_ctl_open(&control, control_name, 0) < 0) {
      continue;
    }
    if (snd_ctl_card_info(control, info) < 0) {
      snd_ctl_close(control);
      continue;
    }
    const char *driver = snd_ctl_card_info_get_driver(info);
    const char *card_id = snd_ctl_card_info_get_id(info);
    if (driver == NULL || card_id == NULL || strcmp(driver, "USB-Audio") != 0) {
      snd_ctl_close(control);
      continue;
    }
    int device = -1;
    while (snd_ctl_pcm_next_device(control, &device) == 0 && device >= 0) {
      snd_pcm_info_set_device(pcm_info, (unsigned int)device);
      snd_pcm_info_set_subdevice(pcm_info, 0);
      snd_pcm_info_set_stream(pcm_info, SND_PCM_STREAM_CAPTURE);
      if (snd_ctl_pcm_info(control, pcm_info) < 0) {
        continue;
      }
      int length = snprintf(name, name_size, "hw:CARD=%s,DEV=%d", card_id, device);
      found = (length > 0 && (size_t)length < name_size) ? 0 : -1;
      break;
    }
    snd_ctl_close(control);
  }
  return found;
}

/** @brief Parse a finite bounded floating-point command-line value.
 * @return 0 on success or -1 for malformed, non-finite, or out-of-range input. */
static int parse_double_value(const char *text, double minimum, double maximum,
                              double *value)
{
  char *end = NULL;
  double parsed;

  errno = 0;
  parsed = strtod(text, &end);
  if (errno != 0 || end == text || *end != '\0' || !isfinite(parsed) ||
      parsed < minimum || parsed > maximum) {
    return -1;
  }
  *value = parsed;
  return 0;
}

/** @brief Parse and cross-validate EARcam command-line configuration.
 * @return 0 for a runnable configuration, 1 after help/version output, or -1 on error.
 * @details Nyquist and band-order checks happen here rather than in the DSP loop so a
 * misconfigured filter cannot produce misleading field measurements. */
static int parse_options(int argc, char **argv, struct options *options)
{
  static const struct option_alias long_options[] = {
    {"device", "-d", true},
    {"rate", "-r", true},
    {"interval", "-i", true},
    {"low-cut", "-l", true},
    {"high-cut", "-u", true},
    {"min-level", "-n", true},
    {"calibration", "-c", true},
    {"self-test", "-S", false},
    {"server", "-M", false},
    {"settings", "-Z", true},
    {"analyze-audio", "-X", true},
    {"check-model", "-Q", false},
    {"version", "-V", false},
    {"help", "-h", false}
  };
  int option;
  char *normalized_argv[EARCAM_MAX_PARSED_ARGUMENTS];
  int normalized_argc = normalize_long_options(argc, argv, normalized_argv,
                                               EARCAM_MAX_PARSED_ARGUMENTS, long_options,
                                               sizeof(long_options) / sizeof(long_options[0]));
  if (normalized_argc < 0) {
    fprintf(stderr, "Invalid or excessive command-line arguments.\n");
    return -1;
  }
  optind = 1;
  while ((option = getopt(normalized_argc, normalized_argv,
                          "d:r:i:l:u:n:c:hSMVZ:X:Q")) != -1) {
    switch (option) {
      case 'Z': options->settings = optarg; break;
      case 'X': options->audio_file = optarg; break;
      case 'Q': options->check_model = true; break;
      case 'd':
        options->device = optarg;
        break;
      case 'r':
        if (parse_unsigned(optarg, MIN_RATE_HZ, MAX_RATE, &options->rate) != 0) {
          fprintf(stderr, "Invalid sample rate: %s\n", optarg);
          return -1;
        }
        break;
      case 'i':
        if (parse_unsigned(optarg, MIN_INTERVAL_MS, MAX_INTERVAL_MS, &options->interval_ms) != 0) {
          fprintf(stderr, "Invalid display interval: %s\n", optarg);
          return -1;
        }
        break;
      case 'l':
        if (parse_double_value(optarg, 0.0, MAX_CUTOFF_HZ, &options->low_cut_hz) != 0) {
          fprintf(stderr, "Invalid low-cut frequency: %s\n", optarg);
          return -1;
        }
        break;
      case 'u':
        if (parse_double_value(optarg, 0.0, MAX_CUTOFF_HZ, &options->high_cut_hz) != 0) {
          fprintf(stderr, "Invalid high-cut frequency: %s\n", optarg);
          return -1;
        }
        break;
      case 'n':
        if (parse_double_value(optarg, DB_FLOOR, 0.0,
                               &options->min_level_dbfs) != 0) {
          fprintf(stderr, "Invalid minimum level: %s\n", optarg);
          return -1;
        }
        break;
      case 'c':
        if (parse_double_value(optarg, MIN_CALIBRATION_DB, MAX_CALIBRATION_DB, &options->calibration_db) != 0) {
          fprintf(stderr, "Invalid calibration offset: %s\n", optarg);
          return -1;
        }
        options->calibrated = true;
        break;
      case 'S':
        options->self_test = true;
        break;
      case 'M':
        options->server = true;
        break;
      case 'h':
        print_usage(stdout, argv[0]);
        return 1;
      case 'V':
        puts(EARCAM_BUILD_VERSION);
        return 1;
      default:
        print_usage(stderr, argv[0]);
        return -1;
    }
  }

  if (optind != normalized_argc) {
    fprintf(stderr, "Unexpected argument: %s\n", normalized_argv[optind]);
    return -1;
  }
  if (options->low_cut_hz <= 0.0 || options->high_cut_hz <= 0.0) {
    fprintf(stderr, "Search-band frequencies must be greater than zero.\n");
    return -1;
  }
  if (options->low_cut_hz >= (double)options->rate / 2.0 ||
      options->high_cut_hz >= (double)options->rate / 2.0) {
    fprintf(stderr, "Filter cutoffs must be below the Nyquist frequency (%.0f Hz).\n",
            (double)options->rate / 2.0);
    return -1;
  }
  if (options->low_cut_hz >= options->high_cut_hz) {
    fprintf(stderr, "Low-cut frequency must be below high-cut frequency.\n");
    return -1;
  }
  return 0;
}

/** @brief Configure a second-order Butterworth high- or low-pass section.
 * @details Two identical sections are cascaded to give useful out-of-band rejection
 * before tone analysis without the latency and allocation cost of an FFT filter bank. */
static void configure_biquad(struct biquad *filter, double cutoff_hz,
                             unsigned int sample_rate, bool high_pass)
{
  const double q = BUTTERWORTH_Q;
  double omega;
  double cosine;
  double alpha;
  double a0;

  memset(filter, 0, sizeof(*filter));
  if (cutoff_hz <= 0.0) {
    return;
  }

  /* Factors of two and unity below belong to the biquad equations, not tuning:
   * omega is radians/sample; a0 normalizes both numerator and feedback terms. */
  omega = 2.0 * PI * cutoff_hz / (double)sample_rate;
  cosine = cos(omega);
  alpha = sin(omega) / (2.0 * q);
  a0 = 1.0 + alpha;

  if (high_pass) {
    filter->b0 = (1.0 + cosine) / (2.0 * a0);
    filter->b1 = -(1.0 + cosine) / a0;
    filter->b2 = filter->b0;
  } else {
    filter->b0 = (1.0 - cosine) / (2.0 * a0);
    filter->b1 = (1.0 - cosine) / a0;
    filter->b2 = filter->b0;
  }
  filter->a1 = (-2.0 * cosine) / a0;
  filter->a2 = (1.0 - alpha) / a0;
  filter->enabled = true;
}

/** @brief Filter one PCM sample while retaining the section delay state.
 * @return Filtered sample, or @p input when the section is disabled. */
static double apply_biquad(struct biquad *filter, double input)
{
  double output;

  if (!filter->enabled) {
    return input;
  }
  output = filter->b0 * input + filter->z1;
  filter->z1 = filter->b1 * input - filter->a1 * output + filter->z2;
  filter->z2 = filter->b2 * input - filter->a2 * output;
  return output;
}

/** @brief Convert a linear amplitude to bounded dBFS.
 * @details Flooring avoids negative infinity propagating into detector thresholds. */
static double amplitude_to_db(double amplitude)
{
  if (amplitude <= AMPLITUDE_FLOOR) {
    return DB_FLOOR;
  }
  return AMPLITUDE_DB_PER_DECADE * log10(amplitude);
}

/** @brief Initialize evenly spaced Goertzel bins across the selected search band.
 * @details The small fixed bank is cheaper and more targeted than a broad FFT because
 * mission alarms occupy a known band and only the strongest frequency is needed. */
static void configure_tone_bins(struct tone_bin *bins,
                                const struct options *options)
{
  double frequency_step = (options->high_cut_hz - options->low_cut_hz) /
                          (double)(TONE_BIN_COUNT - 1U);
  unsigned int index;

  for (index = 0; index < TONE_BIN_COUNT; ++index) {
    double frequency = options->low_cut_hz + frequency_step * (double)index;
    bins[index].coefficient =
      2.0 * cos(2.0 * PI * frequency / (double)options->rate);
    bins[index].first = 0.0;
    bins[index].second = 0.0;
    bins[index].frequency_hz = frequency;
  }
}

/** @brief Feed one windowed, band-passed sample into every Goertzel recurrence.
 * @param bins Persistent Goertzel states for the search frequencies.
 * @param sample Current band-passed PCM value normalized to [-1, 1].
 * @param window Precomputed Hann weight for this sample's interval position.
 * @details Windowing reduces spectral leakage; precomputation keeps trigonometry out of
 * the realtime sample loop. */
static void update_tone_bins(struct tone_bin *bins, double sample, double window)
{
  unsigned int index;

  for (index = 0; index < TONE_BIN_COUNT; ++index) {
    double value = sample * window + bins[index].coefficient * bins[index].first -
                   bins[index].second;
    bins[index].second = bins[index].first;
    bins[index].first = value;
  }
}

/** @brief Extract and reset the strongest Goertzel-bin power estimate.
 * @param bins Goertzel states to inspect and reset for the next interval.
 * @param sample_count Number of samples represented by the states.
 * @param frequency_hz Receives the winning bin frequency.
 * @return Window-normalized tone level in dBFS. */
static double strongest_tone_db(struct tone_bin *bins, uint64_t sample_count,
                                double *frequency_hz)
{
  double maximum_power = 0.0;
  unsigned int index;

  for (index = 0; index < TONE_BIN_COUNT; ++index) {
    double power = bins[index].first * bins[index].first +
                   bins[index].second * bins[index].second -
                   bins[index].coefficient * bins[index].first *
                   bins[index].second;
    if (power > maximum_power) {
      maximum_power = power;
      *frequency_hz = bins[index].frequency_hz;
    }
    bins[index].first = 0.0;
    bins[index].second = 0.0;
  }
  return amplitude_to_db(HANN_TONE_AMPLITUDE_SCALE * sqrt(maximum_power) / (double)sample_count);
}

/** @brief Update envelope and fast/slow trend estimates for operator guidance.
 * @return Stable direction label for the current acoustic evidence.
 * @details The peak-hold envelope falls gradually while two exponential averages make
 * direction less sensitive to individual noisy windows. */
static const char *update_homing_state(struct homing_state *state,
                                       double tone_db, bool alarm_present,
                                       const struct options *options,
                                       double *trend_db)
{
  double elapsed = (double)options->interval_ms / MILLISECONDS_PER_SECOND_F;
  double fast_alpha = 1.0 - exp(-elapsed / HOMING_FAST_TIME_CONSTANT_S);
  double slow_alpha = 1.0 - exp(-elapsed / HOMING_SLOW_TIME_CONSTANT_S);

  if (!state->initialized) {
    state->envelope_db = tone_db;
    state->fast_db = tone_db;
    state->slow_db = tone_db;
    state->initialized = true;
  }

  state->envelope_db -= ENVELOPE_RELEASE_DB_PER_SECOND * elapsed;
  if (alarm_present) {
    if (tone_db > state->envelope_db) {
      state->envelope_db = tone_db;
    }
    ++state->alarm_windows;
  }
  if (state->envelope_db < DB_FLOOR) {
    state->envelope_db = DB_FLOOR;
  }

  state->fast_db += fast_alpha * (state->envelope_db - state->fast_db);
  state->slow_db += slow_alpha * (state->envelope_db - state->slow_db);
  *trend_db = state->fast_db - state->slow_db;

  if (state->alarm_windows < HOMING_ACQUISITION_WINDOWS) {
    return "ACQUIRING ";
  }
  if (*trend_db > TREND_THRESHOLD_DB) {
    return "APPROACH  ";
  }
  if (*trend_db < -TREND_THRESHOLD_DB) {
    return "RECEDING  ";
  }
  return "STEADY    ";
}

/** @brief Analyze a deterministic synthetic tone mixture for self-test coverage. */
static void analyze_test_window(struct tone_bin *bins,
                                const struct options *options,
                                double target_amplitude,
                                double interference_amplitude,
                                double *tone_db, double *contrast_db)
{
  uint64_t sample_count =
    ((uint64_t)options->rate * options->interval_ms) / MILLISECONDS_PER_SECOND;
  uint64_t sample_index;
  double sum_squares = 0.0;
  double frequency_hz = 0.0;
  if (sample_count == 0U || sample_count > MAX_INTERVAL_FRAMES) {
    *tone_db = DB_FLOOR;
    *contrast_db = 0.0;
    return;
  }
  for (sample_index = 0; sample_index < sample_count; ++sample_index) {
    analysis_window[sample_index] = sample_count > 1U
      ? 0.5 - 0.5 * cos(2.0 * PI * (double)sample_index /
                        (double)(sample_count - 1U))
      : 1.0;
  }

  for (sample_index = 0; sample_index < sample_count; ++sample_index) {
    double time = (double)sample_index / (double)options->rate;
    double sample = target_amplitude * sin(2.0 * PI * TEST_TARGET_HZ * time) +
            interference_amplitude * sin(2.0 * PI * TEST_INTERFERENCE_HZ * time);
    update_tone_bins(bins, sample, analysis_window[sample_index]);
    sum_squares += sample * sample;
  }
  *tone_db = strongest_tone_db(bins, sample_count, &frequency_hz);
  *contrast_db = *tone_db -
                 amplitude_to_db(sqrt(sum_squares / (double)sample_count));
}

/** @brief Verify selectivity and trend classification without ALSA hardware.
 * @return 0 on expected detector behavior, otherwise -1. */
static int run_self_test(const struct options *options)
{
    /* A 10 ms unread tail must shift the timestamp back by exactly that duration;
     * underflow and missing sample rate must not manufacture a plausible time. */
    if (audio_read_end_us(TEST_TIMESTAMP_US, TEST_UNREAD_FRAMES, TEST_RATE_HZ) != TEST_EXPECTED_READ_END_US
      || audio_read_end_us(TEST_UNDERFLOW_TIMESTAMP_US, TEST_UNREAD_FRAMES, TEST_RATE_HZ) != 0
      || audio_read_end_us(TEST_TIMESTAMP_US, TEST_UNREAD_FRAMES, 0) != 0) return -1;
  struct tone_bin bins[TONE_BIN_COUNT];
  struct homing_state approaching = {0};
  struct homing_state receding = {0};
  double tone_db;
  double contrast_db;
  double trend_db = 0.0;
  const char *direction = NULL;
  unsigned int index;

  configure_tone_bins(bins, options);
  /* A weak in-band tone amid stronger out-of-band interference checks bin
   * selectivity. The tolerance is software-fixture evidence, not sensor accuracy. */
  analyze_test_window(bins, options, TEST_TARGET_AMPLITUDE, TEST_INTERFERENCE_AMPLITUDE, &tone_db, &contrast_db);
  if (tone_db < TEST_TONE_MIN_DB || tone_db > TEST_TONE_MAX_DB || contrast_db > TEST_CONTRAST_MAX_DB) {
    fprintf(stderr, "Self-test failed: tone selectivity (%.1f, %.1f dB)\n",
            tone_db, contrast_db);
    return -1;
  }

  for (index = 0; index < TEST_TREND_WINDOWS; ++index) {
    double level = TEST_APPROACH_START_DB + (double)index * TEST_TREND_STEP_DB;
    direction = update_homing_state(&approaching, level, true, options,
                                    &trend_db);
  }
  if (strcmp(direction, "APPROACH  ") != 0) {
    fprintf(stderr, "Self-test failed: approaching trend (%.1f dB)\n",
            trend_db);
    return -1;
  }

  for (index = 0; index < TEST_TREND_WINDOWS; ++index) {
    double level = TEST_RECEDE_START_DB - (double)index * TEST_TREND_STEP_DB;
    direction = update_homing_state(&receding, level, true, options,
                                    &trend_db);
  }
  if (strcmp(direction, "RECEDING  ") != 0) {
    fprintf(stderr, "Self-test failed: receding trend (%.1f dB)\n", trend_db);
    return -1;
  }

  printf("motionSCOUT detector self-test passed\n");
  return 0;
}

/** @brief Render one human-facing measurement line for bench use. */
static void display_homing(double level_db, double frequency_hz,
                           double contrast_db, bool alarm_present,
                           const char *direction, double trend_db,
                           const struct homing_state *state,
                           const struct options *options)
{
  double displayed_level = level_db + options->calibration_db;
  double meter_level = (state->envelope_db - DB_FLOOR) / -DB_FLOOR;
  unsigned int filled;
  unsigned int index;

  if (meter_level < 0.0) {
    meter_level = 0.0;
  } else if (meter_level > 1.0) {
    meter_level = 1.0;
  }
  filled = (unsigned int)(meter_level * (double)METER_WIDTH + 0.5);

  printf("\r[");
  for (index = 0; index < METER_WIDTH; ++index) {
    putchar(index < filled ? '#' : ' ');
  }
  printf("] %s %s %6.1f %s  %4.0f Hz  trend %+5.1f dB  conf %+4.1f dB",
         alarm_present ? "ALARM" : "listen", direction, displayed_level,
         options->calibrated ? "dB SPL" : "dBFS", frequency_hz, trend_db,
         contrast_db);
  if (options->calibrated && alarm_present) {
    double distance = SOURCE_REFERENCE_METERS *
                      pow(DB_LOG_BASE, (SOURCE_LEVEL_DB_SPL - displayed_level) / AMPLITUDE_DB_PER_DECADE);
    printf("  ~%5.1f m", distance);
  }
  fflush(stdout);
}

/** @brief Configure mono signed-16 ALSA capture with low-latency period sizes.
 * @return ALSA status code.
 * @details Periods target five milliseconds to keep USHER samples timely without
 * forcing an excessively interrupt-heavy capture configuration. */
static int configure_capture(snd_pcm_t *pcm, unsigned int sample_rate)
{
  if (snd_pcm_hw_params_sizeof() > sizeof(hardware_params_storage.bytes)
      || snd_pcm_sw_params_sizeof() > sizeof(software_params_storage.bytes)) {
    errno = EOVERFLOW;
    return -EOVERFLOW;
  }
  snd_pcm_hw_params_t *hardware = (snd_pcm_hw_params_t *)hardware_params_storage.bytes;
  snd_pcm_sw_params_t *software = (snd_pcm_sw_params_t *)software_params_storage.bytes;
  snd_pcm_uframes_t period_frames = sample_rate / CAPTURE_PERIODS_PER_SECOND;
  snd_pcm_uframes_t buffer_frames = sample_rate / CAPTURE_BUFFERS_PER_SECOND;
  int direction = 0;
  int result;

  result = snd_pcm_hw_params_any(pcm, hardware);
  if (result >= 0) {
    result = snd_pcm_hw_params_set_access(
      pcm, hardware, SND_PCM_ACCESS_RW_INTERLEAVED);
  }
  if (result >= 0) {
    result = snd_pcm_hw_params_set_format(pcm, hardware,
                                          SND_PCM_FORMAT_S16_LE);
  }
  if (result >= 0) {
    result = snd_pcm_hw_params_set_channels(pcm, hardware, CAPTURE_MONO_CHANNELS);
  }
  if (result >= 0) {
    result = snd_pcm_hw_params_set_rate(pcm, hardware, sample_rate, 0);
  }
  if (result >= 0) {
    result = snd_pcm_hw_params_set_period_size_near(
      pcm, hardware, &period_frames, &direction);
  }
  if (result >= 0) {
    result = snd_pcm_hw_params_set_buffer_size_near(
      pcm, hardware, &buffer_frames);
  }
  if (result >= 0) {
    result = snd_pcm_hw_params(pcm, hardware);
  }
  if (result < 0) {
    return result;
  }

  result = snd_pcm_sw_params_current(pcm, software);
  capture_timestamps = result >= 0
    && snd_pcm_sw_params_set_tstamp_mode(pcm, software, SND_PCM_TSTAMP_ENABLE) >= 0
    && snd_pcm_sw_params_set_tstamp_type(pcm, software, SND_PCM_TSTAMP_TYPE_MONOTONIC) >= 0;
  if (result >= 0) {
    result = snd_pcm_sw_params_set_avail_min(pcm, software, period_frames);
  }
  if (result >= 0) {
    result = snd_pcm_sw_params_set_start_threshold(pcm, software, 1U);
  }
  if (result >= 0) {
    result = snd_pcm_sw_params_set_stop_threshold(pcm, software,
                                                  buffer_frames);
  }
  if (result >= 0) {
    result = snd_pcm_sw_params(pcm, software);
  }
  if (result >= 0) {
    result = snd_pcm_prepare(pcm);
  }
  return result;
}

// Server output: one self-describing line per analysis window. A slow reader
// must never stall audio capture, so stdout is non-blocking and drops count.
static unsigned long server_dropped_lines = 0;

/** @brief Emit one classifier window to the server's nonblocking output.
 * @param result Classification, clipping and monotonic window evidence.
 * @param options Level calibration and its qualification flag.
 * @details Format and write the record as a unit. Count incomplete writes
 * instead of letting parent backpressure stall audio acquisition. */
static void emit_classifier_sample(const struct ear_inference_result *result, const struct options *options)
{
  char line[SERVER_SAMPLE_LINE_CAPACITY];
  const int length = snprintf(line, sizeof(line),
    "CLASS t=%llu level=%.2f score=%.6f class=%u alarm=%u clip=%u cal=%u "
    "begin_us=%llu end_us=%llu error_us=%llu sequence=%llu\n",
    (unsigned long long)monotonic_ms(), result->level_dbfs + options->calibration_db,
    result->score, result->category, result->alarm ? 1U : 0U, result->clipped ? 1U : 0U,
    options->calibrated ? 1U : 0U, (unsigned long long)result->begin_us,
    (unsigned long long)result->end_us, (unsigned long long)result->error_us,
    (unsigned long long)result->sequence);
  if (length <= 0 || (size_t)length >= sizeof(line)) return;
  const ssize_t written = write(STDOUT_FILENO, line, (size_t)length);
  if (written != length) ++server_dropped_lines;
}

/** @brief Emit one self-describing USHER server sample without blocking capture.
 * @details A slow parent loses a measurement rather than stalling ALSA; the drop count
 * remains observable for diagnostics. */
static void emit_server_sample(uint64_t timestamp_ms, double level_db,
                               double frequency_hz, double contrast_db,
                               double trend_db, double envelope_db,
                               bool alarm_present, bool clipped,
                               const struct options *options, uint64_t begin_us,
                               uint64_t end_us, uint64_t error_us, uint64_t sequence)
{
  char line[SERVER_SAMPLE_LINE_CAPACITY];
  int length = snprintf(line, sizeof(line),
                        "SAMPLE t=%llu level=%.2f freq=%.0f contrast=%.2f "
                        "trend=%.2f env=%.2f alarm=%d clip=%d cal=%d"
                        " begin_us=%llu end_us=%llu error_us=%llu sequence=%llu\n",
                        (unsigned long long)timestamp_ms,
                        level_db + options->calibration_db, frequency_hz,
                        contrast_db, trend_db,
                        envelope_db + options->calibration_db,
                        alarm_present ? 1 : 0, clipped ? 1 : 0,
                        options->calibrated ? 1 : 0,
                        (unsigned long long)begin_us, (unsigned long long)end_us,
                        (unsigned long long)error_us, (unsigned long long)sequence);
  if (length <= 0 || (size_t)length >= sizeof(line)) {
    return;
  }
  size_t sent = 0;
  while (sent < (size_t)length) {
    ssize_t written = write(STDOUT_FILENO, line + sent, (size_t)length - sent);
    if (written > 0) {
      sent += (size_t)written;
      continue;
    }
    if (written < 0 && errno == EINTR) {
      continue;
    }
    ++server_dropped_lines;
    return;
  }
}

// Returns 1 when the controller asked us to stop, 0 otherwise.
/** @brief Consume non-blocking STOP/QUIT commands from USHER.
 * @return 1 when capture should stop, otherwise 0.
 * @details Chunked input accepts fragmented commands while avoiding one syscall per
 * character and never delaying audio acquisition. */
static int poll_server_stdin(void)
{
  struct pollfd descriptor = {.fd = STDIN_FILENO, .events = POLLIN, .revents = 0};
  static char command[SERVER_COMMAND_CAPACITY];
  static size_t command_size = 0;

  if (poll(&descriptor, 1, 0) <= 0) {
    return 0;
  }
  if ((descriptor.revents & (POLLHUP | POLLERR | POLLNVAL)) != 0
      && (descriptor.revents & POLLIN) == 0) {
    return 1;
  }
  char chunk[SERVER_READ_CHUNK_CAPACITY];
  ssize_t count = read(STDIN_FILENO, chunk, sizeof(chunk));
  if (count == 0) {
    return 1;
  }
  if (count < 0) {
    return (errno == EAGAIN || errno == EINTR) ? 0 : 1;
  }
  for (ssize_t index = 0; index < count; ++index) {
    if (chunk[index] == '\n') {
      command[command_size] = '\0';
      command_size = 0;
      if (strcmp(command, "STOP") == 0 || strcmp(command, "QUIT") == 0) {
        return 1;
      }
    } else if (command_size + 1 < sizeof(command)) {
      command[command_size++] = chunk[index];
    }
  }
  return 0;
}

/** @brief Capture, filter, analyze, and report audio until stopped.
 * @return 0 for orderly stop or -1 for unrecoverable ALSA or bounds failure.
 * @details ALSA recovery handles transient overruns. Each reporting window produces
 * band RMS, strongest tone, contrast, and homing trend. */
static int run_meter(snd_pcm_t *pcm, const struct options *options)
{
  snd_pcm_uframes_t buffer_frames;
  snd_pcm_uframes_t period_frames;
  struct biquad high_pass[FILTER_SECTIONS_PER_EDGE];
  struct biquad low_pass[FILTER_SECTIONS_PER_EDGE];
  struct tone_bin tone_bins[TONE_BIN_COUNT];
  struct homing_state homing = {0};
  uint64_t interval_frames =
    ((uint64_t)options->rate * options->interval_ms) / MILLISECONDS_PER_SECOND;
  uint64_t accumulated_frames = 0U;
  double sum_squares = 0.0;
  bool window_clipped = false;
  bool window_timing_valid = true;
  uint64_t window_sequence = 0;

  if (snd_pcm_get_params(pcm, &buffer_frames, &period_frames) < 0 ||
      period_frames == 0U || period_frames > MAX_PERIOD_FRAMES) {
    fprintf(stderr, "Cannot determine ALSA capture period size.\n");
    return -1;
  }
  if (interval_frames == 0U || interval_frames > MAX_INTERVAL_FRAMES) {
    fprintf(stderr, "Analysis interval exceeds the fixed workspace.\n");
    return -1;
  }
  /* The Hann equation uses half-scale/half-offset and a full 2*pi cycle. These
   * mathematical coefficients stay literal; detector policy lives in named constants. */
  for (uint64_t index = 0; index < interval_frames; ++index) {
    analysis_window[index] = interval_frames > 1U
      ? 0.5 - 0.5 * cos(2.0 * PI * (double)index /
                        (double)(interval_frames - 1U))
      : 1.0;
  }

  configure_biquad(&high_pass[0], options->low_cut_hz, options->rate, true);
  configure_biquad(&high_pass[1], options->low_cut_hz, options->rate, true);
  configure_biquad(&low_pass[0], options->high_cut_hz, options->rate, false);
  configure_biquad(&low_pass[1], options->high_cut_hz, options->rate, false);
  configure_tone_bins(tone_bins, options);
  if (snd_pcm_nonblock(pcm, 1) < 0) { errno = EIO; return -1; }
  uint64_t last_progress_ms = monotonic_ms();
  if (!last_progress_ms) { errno = EIO; return -1; }
  while (keep_running) {
    if (options->server && poll_server_stdin()) {
      break;
    }
    const uint64_t now_ms = monotonic_ms();
    if (!now_ms || now_ms < last_progress_ms || now_ms - last_progress_ms >= CAPTURE_STALL_TIMEOUT_MS) {
      fprintf(stderr, "Capture made no progress within its deadline.\n");
      errno = ETIMEDOUT;
      return -1;
    }
    snd_pcm_sframes_t frames_read = snd_pcm_readi(pcm, capture_samples, period_frames);
    snd_pcm_sframes_t frame;

    if (!keep_running) break;
    const uint64_t completed_ms = monotonic_ms();
    if (!completed_ms || completed_ms < last_progress_ms || completed_ms - last_progress_ms >= CAPTURE_STALL_TIMEOUT_MS) {
      errno = ETIMEDOUT;
      return -1;
    }
    const int remaining_ms = (int)(CAPTURE_STALL_TIMEOUT_MS - (completed_ms - last_progress_ms));
    const int wait_ms = remaining_ms < CAPTURE_WAIT_MS ? remaining_ms : CAPTURE_WAIT_MS;
    if (frames_read == 0 || frames_read == -EAGAIN || frames_read == -EINTR) {
      const int ready = snd_pcm_wait(pcm, wait_ms);
      if (ready >= 0 || ready == -EINTR || ready == -EAGAIN) continue;
      frames_read = ready;
    }
    if (frames_read < 0) {
      int recovery;
      if (frames_read == -ESTRPIPE) {
        recovery = snd_pcm_resume(pcm);
        if (recovery == -EAGAIN) {
          if (poll(NULL, 0, wait_ms) < 0 && errno != EINTR) return -1;
          continue;
        }
        if (recovery < 0) recovery = snd_pcm_prepare(pcm);
      } else recovery = snd_pcm_recover(pcm, (int)frames_read, 1);
      if (recovery < 0) {
        fprintf(stderr, "\nCapture failed: %s\n", snd_strerror(recovery));
        if (options->server) {
          printf("EARCAM_ERROR capture %s\n", snd_strerror(recovery));
          fflush(stdout);
        }
        return -1;
      }
      /* An overrun breaks window continuity and pose timing. Discard accumulated
       * spectral/envelope evidence rather than joining audio across that gap. */
      accumulated_frames = 0;
      if (options->onnx) ear_inference_discontinuity();
      sum_squares = 0;
      window_clipped = false;
      window_timing_valid = false;
      homing = (struct homing_state){0};
      configure_tone_bins(tone_bins, options);
      continue;
    }

    if ((snd_pcm_uframes_t)frames_read > period_frames) { errno = EOVERFLOW; return -1; }
    last_progress_ms = completed_ms;
    snd_pcm_uframes_t available_frames = 0;
    snd_htimestamp_t timestamp = {0};
    uint64_t read_end_us = 0;
    if (capture_timestamps && snd_pcm_htimestamp(pcm, &available_frames, &timestamp) == 0
        && timestamp.tv_sec > 0 && timestamp.tv_nsec >= 0 && timestamp.tv_nsec < NANOSECONDS_PER_SECOND
        && available_frames <= buffer_frames) {
      const uint64_t hardware_us = (uint64_t)timestamp.tv_sec * MICROSECONDS_PER_SECOND
                   + (uint64_t)timestamp.tv_nsec / NANOSECONDS_PER_MICROSECOND;
      read_end_us = audio_read_end_us(hardware_us, available_frames, options->rate);
    }
    if (options->onnx) {
      const uint64_t error_us = ((uint64_t)period_frames * MICROSECONDS_PER_SECOND + options->rate - 1) / options->rate + 1;
      ear_inference_push(capture_samples, (size_t)frames_read, read_end_us, error_us);
      struct ear_inference_result classified;
      if (ear_inference_take(&classified)) {
        if (options->server) emit_classifier_sample(&classified, options);
        else printf("class=%u score=%.6f alarm=%u pcm_rms=%.2f dBFS\n", classified.category,
                    classified.score, classified.alarm ? 1U : 0U, classified.level_dbfs);
      }
      continue;
    }
    window_timing_valid = (accumulated_frames == 0 || window_timing_valid) && read_end_us != 0;

    for (frame = 0; frame < frames_read; ++frame) {
      int16_t raw = capture_samples[frame];
      double sample = (double)raw / PCM_S16_FULL_SCALE;
      if (raw >= CLIP_THRESHOLD || raw <= -CLIP_THRESHOLD) {
        window_clipped = true;
      }

      sample = apply_biquad(&high_pass[0], sample);
      sample = apply_biquad(&high_pass[1], sample);
      sample = apply_biquad(&low_pass[0], sample);
      sample = apply_biquad(&low_pass[1], sample);
      update_tone_bins(tone_bins, sample, analysis_window[accumulated_frames]);
      sum_squares += sample * sample;
      ++accumulated_frames;

      if (accumulated_frames == interval_frames) {
        double band_db = amplitude_to_db(
          sqrt(sum_squares / (double)accumulated_frames));
        double frequency_hz = 0.0;
        double tone_db = strongest_tone_db(tone_bins, accumulated_frames,
                                            &frequency_hz);
        double contrast_db = tone_db - band_db;
        double trend_db;
        /* A level gate alone accepts broadband aircraft noise. Requiring bin
         * contrast adds tone-like evidence, but remains a heuristic alarm flag. */
        bool alarm_present = tone_db >= options->min_level_dbfs &&
                             contrast_db >= TONE_CONTRAST_THRESHOLD_DB;
        const char *direction = update_homing_state(
          &homing, tone_db, alarm_present, options, &trend_db);

        if (options->server) {
          uint64_t end_us = window_timing_valid
            ? audio_read_end_us(read_end_us, (uint64_t)(frames_read - frame - 1), options->rate) : 0;
          /* A reporting boundary may lie inside this read. Subtract its remaining
           * tail too; otherwise all windows in a read would get the same end time.
           * Missing hardware timing stays zero for the parent's explicit fallback. */
          const uint64_t duration_us = interval_frames * MICROSECONDS_PER_SECOND / options->rate;
          uint64_t begin_us = end_us > duration_us ? end_us - duration_us : 0;
          if (!begin_us) end_us = 0;
          const uint64_t error_us = ((uint64_t)period_frames * MICROSECONDS_PER_SECOND + options->rate - 1) / options->rate + 1;
          emit_server_sample(monotonic_ms(), tone_db, frequency_hz, contrast_db,
                             trend_db, homing.envelope_db, alarm_present,
                             window_clipped, options, begin_us, end_us, error_us, ++window_sequence);
        } else {
          display_homing(tone_db, frequency_hz, contrast_db, alarm_present,
                         direction, trend_db, &homing, options);
        }
        accumulated_frames = 0U;
        sum_squares = 0.0;
        window_clipped = false;
        window_timing_valid = read_end_us != 0;
      }
    }
  }
  if (!options->server) {
    putchar('\n');
  }
  return 0;
}

/** @brief Configure EARcam, optionally self-test it, then run the ALSA meter.
 * @return EXIT_SUCCESS on normal termination, EXIT_FAILURE on configuration/capture errors. */
int main(int argc, char **argv)
{
  struct options options = {
    .device = DEFAULT_DEVICE,
    .rate = DEFAULT_RATE,
    .interval_ms = DEFAULT_INTERVAL_MS,
    .low_cut_hz = DEFAULT_LOW_CUT_HZ,
    .high_cut_hz = DEFAULT_HIGH_CUT_HZ,
    .min_level_dbfs = DEFAULT_MIN_LEVEL_DBFS,
    .calibration_db = 0.0,
    .calibrated = false,
    .self_test = false,
    .server = false
  };
  snd_pcm_t *pcm = NULL;
  struct sigaction action;
  int result;

  result = parse_options(argc, argv, &options);
  if (result != 0) {
    return result > 0 ? EXIT_SUCCESS : EXIT_FAILURE;
  }
  if (options.self_test) {
    return run_self_test(&options) == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
  }
  char settings_path[INFERENCE_PATH_CAPACITY], inference_error[2048] = {0};
  if (!options.settings && inference_settings_find("earcam", "ear_cam", settings_path, sizeof(settings_path)))
    options.settings = settings_path;
  if (options.settings && !ear_inference_configure(options.settings, options.rate, &options.onnx,
                                                    inference_error, sizeof(inference_error))) {
    fprintf(stderr, "earcam: %s\n", inference_error);
    return EXIT_FAILURE;
  }
  if (atexit(ear_inference_stop)) { ear_inference_stop(); return EXIT_FAILURE; }
  if (options.check_model) {
    puts(options.onnx ? "earcam model ready adapter=ear_pcm_classifier_v1" : "earcam native detector ready");
    return EXIT_SUCCESS;
  }
  if (options.audio_file) return options.onnx ? ear_inference_file(options.audio_file) : EXIT_FAILURE;
  if (options.onnx && !ear_inference_start(inference_error, sizeof(inference_error))) {
    fprintf(stderr, "earcam: %s\n", inference_error);
    return EXIT_FAILURE;
  }

  memset(&action, 0, sizeof(action));
  action.sa_handler = handle_signal;
  sigemptyset(&action.sa_mask);
  sigaction(SIGINT, &action, NULL);
  sigaction(SIGTERM, &action, NULL);
  if (options.server) {
    signal(SIGPIPE, SIG_IGN);
    int flags = fcntl(STDOUT_FILENO, F_GETFL);
    if (flags >= 0) {
      fcntl(STDOUT_FILENO, F_SETFL, flags | O_NONBLOCK);
    }
    flags = fcntl(STDIN_FILENO, F_GETFL);
    if (flags >= 0) {
      fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }
  }

  char auto_device[AUTO_DEVICE_NAME_MAX];
  if (strcmp(options.device, "auto") == 0) {
    if (find_usb_capture_device(auto_device, sizeof(auto_device)) != 0) {
      fprintf(stderr, "No USB audio capture device found (use --device to select one).\n");
      if (options.server) {
        printf("EARCAM_ERROR open no USB capture device\n");
        fflush(stdout);
      }
      return EXIT_FAILURE;
    }
    options.device = auto_device;
  }

  result = snd_pcm_open(&pcm, options.device, SND_PCM_STREAM_CAPTURE, 0);
  if (result < 0) {
    fprintf(stderr, "Cannot open ALSA capture device '%s': %s\n",
            options.device, snd_strerror(result));
    if (options.server) {
      printf("EARCAM_ERROR open %s\n", snd_strerror(result));
      fflush(stdout);
    }
    return EXIT_FAILURE;
  }

  result = configure_capture(pcm, options.rate);
  if (result < 0) {
    fprintf(stderr, "Cannot configure '%s' for %u Hz mono capture: %s\n",
            options.device, options.rate, snd_strerror(result));
    if (options.server) {
      printf("EARCAM_ERROR configure %s\n", snd_strerror(result));
      fflush(stdout);
    }
    snd_pcm_close(pcm);
    return EXIT_FAILURE;
  }

  if (options.server) {
    printf("EARCAM_READY device=%s rate=%u interval_ms=%u low=%.0f high=%.0f cal=%d\n",
           options.device, options.rate, options.interval_ms, options.low_cut_hz,
           options.high_cut_hz, options.calibrated ? 1 : 0);
    fflush(stdout);
  } else {
    printf("earcam motionSCOUT: device=%s, rate=%u Hz, search=%.0f..%.0f Hz, "
           "update=%u ms, output=%s\n",
           options.device, options.rate, options.low_cut_hz,
           options.high_cut_hz, options.interval_ms,
           options.calibrated ? "calibrated dB SPL" : "uncalibrated dBFS");
    printf("Press Ctrl+C to stop.\n");
  }

  result = run_meter(pcm, &options);
  snd_pcm_drop(pcm);
  snd_pcm_close(pcm);
  if (options.server) {
    printf("EARCAM_STOPPED dropped=%lu\n", server_dropped_lines);
    fflush(stdout);
  }
  return result == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
