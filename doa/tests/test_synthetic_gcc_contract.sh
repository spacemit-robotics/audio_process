#!/usr/bin/env bash
set -euo pipefail

module_dir="components/multimedia/audio_process/doa"
build_dir="$(mktemp -d "${TMPDIR:-/tmp}/doa-pr-test.XXXXXX")"
trap 'rm -rf "${build_dir}"' EXIT

pkg-config --exists fftw3f
read -r -a fftw_flags <<< "$(pkg-config --cflags --libs fftw3f)"

g++ -std=c++17 -Wall -Wextra -Werror \
  "${module_dir}/tests/doa_pr_contract_test.cpp" \
  "${module_dir}/src/sound_locator.cpp" \
  "${module_dir}/src/gcc_phat_pair.cpp" \
  "${module_dir}/src/multi_sound_locator.cpp" \
  -I"${module_dir}/include" \
  "${fftw_flags[@]}" \
  -lm \
  -o "${build_dir}/doa_pr_contract_test"

"${build_dir}/doa_pr_contract_test" --synthetic-gcc-contract
