import os
_repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INCLUDE_DIR = os.path.join(_repo, 'third_party', 'libyuv', 'include')
LIB_DIR = os.path.join(_repo, 'third_party', 'libyuv', 'larch64', 'lib')
