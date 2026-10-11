"""On-drive MUTCD speed-sign logger. Stock modelV2 has no speedSign head.

These env vars are set before speedsignd.py imports numpy. Manager still
forks this process after numpy is already loaded, so main() also shrinks
the live OpenBLAS / OpenMP pool. An env var cannot resize a pool that
already started.
"""
import os

_THREAD_ENV = (
  "OMP_NUM_THREADS",
  "OPENBLAS_NUM_THREADS",
  "MKL_NUM_THREADS",
  "NUMEXPR_NUM_THREADS",
  "VECLIB_MAXIMUM_THREADS",
  "BLIS_NUM_THREADS",
  "OPENCV_FOR_THREADS_NUM",
  "GOTO_NUM_THREADS",
  "TINYGRAD_NUM_THREADS",
  "KMP_NUM_THREADS",
)
for _thread_key in _THREAD_ENV:
  os.environ[_thread_key] = "1"
os.environ["OMP_DYNAMIC"] = "FALSE"
os.environ["MKL_DYNAMIC"] = "FALSE"
os.environ["OMP_WAIT_POLICY"] = "PASSIVE"
os.environ["DEV"] = "CPU"
os.environ["CPU"] = "1"
os.environ["CLANG"] = "1"
os.environ["CPU_COUNT"] = "1"
os.environ["BEAM"] = "0"
os.environ["NOOPT"] = "0"
for _gpu_key in ("GPU", "QCOM", "CUDA", "METAL", "AMD", "NV", "CL", "WEBGPU", "DSP"):
  os.environ[_gpu_key] = "0"
os.environ["OPENCV_OPENCL_RUNTIME"] = "disabled"
os.environ["OPENCV_DNN_OPENCL"] = "0"
