import os


_DEFAULT_BASEDIR = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "../"))
BASEDIR = os.path.abspath(os.environ["BASEDIR"]) if os.environ.get("BASEDIR") else _DEFAULT_BASEDIR
