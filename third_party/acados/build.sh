#!/usr/bin/env bash
set -e

export SOURCE_DATE_EPOCH=0
export ZERO_AR_DATE=1

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"

ARCHNAME="x86_64"
BLAS_TARGET="X64_AUTOMATIC"
if [ -f /TICI ]; then
  ARCHNAME="larch64"
  BLAS_TARGET="ARMV8A_ARM_CORTEX_A57"
fi

ACADOS_FLAGS=(
  "-DACADOS_WITH_QPOASES=ON"
  "-UBLASFEO_TARGET"
  "-DBLASFEO_TARGET=$BLAS_TARGET"
)

if [[ "$OSTYPE" != "darwin"* ]]; then
  ACADOS_FLAGS+=(
    "-DCMAKE_C_FLAGS=-D_POSIX_C_SOURCE=200112L -include stdlib.h -Wno-error=incompatible-pointer-types"
    "-DCMAKE_SHARED_LINKER_FLAGS=-Wl,-z,noexecstack"
    "-DCMAKE_MODULE_LINKER_FLAGS=-Wl,-z,noexecstack"
    "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
  )
fi

if [[ "$OSTYPE" == "darwin"* ]]; then
  ACADOS_FLAGS+=(
    "-DCMAKE_OSX_ARCHITECTURES=arm64"
    "-DCMAKE_MACOSX_RPATH=1"
  )
  ARCHNAME="Darwin"
fi

if [ ! -d "$DIR/acados_repo/" ]; then
  git clone https://github.com/acados/acados.git "$DIR/acados_repo"
  # git clone https://github.com/commaai/acados.git "$DIR/acados_repo"
fi
cd "$DIR/acados_repo"
git fetch --all
git checkout 8af9b0ad180940ef611884574a0b27a43504311d # v0.2.2
git submodule update --depth=1 --recursive --init

mkdir -p build
cd build
cmake "${ACADOS_FLAGS[@]}" ..
make -j20 install

INSTALL_DIR="$DIR/$ARCHNAME"
rm -rf "$INSTALL_DIR"
mkdir -p "$INSTALL_DIR"

rm "$DIR/acados_repo/lib/"*.json

rm -rf "$DIR/include" "$DIR/acados_template"
cp -r "$DIR/acados_repo/include" "$DIR"
cp -r "$DIR/acados_repo/lib" "$INSTALL_DIR"
cp -r "$DIR/acados_repo/interfaces/acados_template/acados_template" "$DIR"

if [[ "$OSTYPE" != "darwin"* ]]; then
  emitted_libraries_file="$(mktemp)"
  trap 'rm -f "$emitted_libraries_file"' EXIT
  if find "$INSTALL_DIR" -type f \( -name '*.so' -o -name '*.so.*' \) -print0 >"$emitted_libraries_file"; then
    :
  else
    echo "failed to discover acados shared libraries" >&2
    exit 1
  fi
  mapfile -d '' emitted_libraries < "$emitted_libraries_file"
  rm -f "$emitted_libraries_file"
  trap - EXIT
  if (( ${#emitted_libraries[@]} == 0 )); then
    echo "acados build emitted no shared libraries" >&2
    exit 1
  fi
  for library in "${emitted_libraries[@]}"; do
    if readelf_output="$(readelf -lW "$library")"; then
      :
    else
      echo "readelf failed for $library" >&2
      exit 1
    fi
    if stack_status="$(awk '$1 == "GNU_STACK" {
      found = 1
      for (i = 2; i <= NF; i++) {
        if ($i ~ /^[RWE]+$/) {
          if ($i ~ /E/) exit 2
          exit 0
        }
      }
      exit 3
    }
    END {
      if (!found) exit 3
    }' <<< "$readelf_output")"; then
      :
    else
      status=$?
      if (( status == 2 )); then
        echo "executable GNU_STACK in $library" >&2
      else
        echo "missing or malformed GNU_STACK in $library" >&2
      fi
      exit 1
    fi
  done
fi

# skip macOS - sed is different :/
if [[ "$OSTYPE" != "darwin"* ]]; then
  # strip future_fstrings to avoid having to install the compatibility package
  find "$DIR/acados_template/" -type f -exec sed -i '/future.fstrings/d' {} +
fi

# build tera
cd "$DIR/acados_repo/interfaces/acados_template/tera_renderer/"
if [[ "$OSTYPE" == "darwin"* ]]; then
  cargo build --verbose --release --target aarch64-apple-darwin
  cp target/aarch64-apple-darwin/release/t_renderer target/release/t_renderer
else
  cargo build --verbose --release
fi
cp target/release/t_renderer "$INSTALL_DIR/"
