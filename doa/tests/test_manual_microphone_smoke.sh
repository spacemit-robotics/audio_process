#!/usr/bin/env bash
set -euo pipefail

demo="${SROBOTIS_OUTPUT_STAGING:-output/staging}/bin/ssl_demo"
artifact_dir="${SROBOTIS_TEST_ARTIFACT_DIR:-/tmp/doa-manual-smoke}"
log_path="${artifact_dir}/doa-manual-microphone-smoke.log"
mkdir -p "${artifact_dir}"

if [[ "${DOA_MANUAL_RUN:-0}" != "1" ]]; then
  cat >&2 <<'EOF'
NOT RUN: DOA manual microphone smoke requires a real multi-mic device.
Set DOA_MANUAL_RUN=1 after confirming the microphone array, channel count,
sample rate, and mic geometry are available.
Optional: set DOA_MANUAL_DEVICE, DOA_MANUAL_CHANNELS, DOA_MANUAL_CAPTURE_CHANNELS,
DOA_MANUAL_PICK, DOA_MANUAL_POSITIONS, and DOA_MANUAL_AZIMUTH_OFFSET.
EOF
  exit 2
fi

if [[ ! -x "${demo}" ]]; then
  echo "ERROR: ssl_demo not found or not executable: ${demo}" >&2
  echo "Build components/multimedia/audio_process/doa before running manual smoke." >&2
  exit 1
fi

duration_s="${DOA_MANUAL_DURATION_S:-10}"
channels="${DOA_MANUAL_CHANNELS:-3}"
sample_rate="${DOA_MANUAL_SAMPLE_RATE:-16000}"
device="${DOA_MANUAL_DEVICE:--1}"
mic_distance="${DOA_MANUAL_MIC_DISTANCE:-0.063}"

cmd=(
  "${demo}"
  -c "${channels}"
  -l
  -d "${mic_distance}"
  -r "${sample_rate}"
  -i "${device}"
  --verbose
)

if [[ -n "${DOA_MANUAL_CAPTURE_CHANNELS:-}" ]]; then
  cmd+=(--capture-channels "${DOA_MANUAL_CAPTURE_CHANNELS}")
fi
if [[ -n "${DOA_MANUAL_PICK:-}" ]]; then
  cmd+=(--pick "${DOA_MANUAL_PICK}")
fi
if [[ -n "${DOA_MANUAL_POSITIONS:-}" ]]; then
  cmd+=(--positions "${DOA_MANUAL_POSITIONS}")
fi
if [[ -n "${DOA_MANUAL_AZIMUTH_OFFSET:-}" ]]; then
  cmd+=(--azimuth-offset "${DOA_MANUAL_AZIMUTH_OFFSET}")
fi

echo "Running bounded live DOA smoke for ${duration_s}s" | tee "${log_path}"
printf 'Command:' | tee -a "${log_path}"
printf ' %q' "${cmd[@]}" | tee -a "${log_path}"
printf '\n' | tee -a "${log_path}"

set +e
timeout -s INT "${duration_s}s" "${cmd[@]}" 2>&1 | tee -a "${log_path}"
status=${PIPESTATUS[0]}
set -e

if [[ "${status}" != "0" && "${status}" != "124" ]]; then
  echo "ERROR: live DOA smoke failed with status ${status}" >&2
  exit "${status}"
fi

grep -q "Live Capture" "${log_path}"
grep -q "Stopped. callbacks:" "${log_path}"

echo "PASS DOA manual microphone smoke: ${log_path}"
