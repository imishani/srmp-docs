.. _installation:

Installation
============

SRMP runs with Python 3.9 through 3.14 on:

- **Linux**: Ubuntu 22.04 or newer, x86-64. The CPU needs AVX2 (Intel Haswell from 2013 or
  later, AMD Zen+ or later).
- **macOS**: macOS 11 or newer on Apple Silicon (M1 or later). Intel Macs are not supported.
- **Windows**: through WSL2, see below.

.. code-block:: console

   $ python3 -m venv .venv && source .venv/bin/activate
   (.venv) $ pip install srmp

Check that it imports, and which version you got:

.. code-block:: console

   (.venv) $ python -c "import srmp" && pip show srmp | head -2

The package includes the planners, the robot registry client, the viser visualizer and the
agent mode. Robot models are downloaded on first use (see :doc:`../user_guide/robot_registry`).

.. note::

   :doc:`VAMP plugins <../user_guide/vamp_plugins>` need Linux x86-64. On macOS, only the
   robots built into srmp can use the VAMP backend.

Windows (WSL2)
--------------

SRMP has no native Windows build, but the Linux package runs under WSL2 (Windows Subsystem for
Linux) on Windows 10 and 11, VAMP backend included.

1. In PowerShell as administrator, install WSL2 with Ubuntu 24.04 (22.04 or newer is
   required), then restart if asked:

   .. code-block:: console

      PS> wsl --install -d Ubuntu-24.04

2. Open the Ubuntu terminal and install SRMP into a virtual environment:

   .. code-block:: console

      $ sudo apt update && sudo apt install -y python3-venv
      $ python3 -m venv ~/srmp-venv && source ~/srmp-venv/bin/activate
      (srmp-venv) $ pip install srmp

Notes:

- Keep your project files inside the Linux file system (e.g. ``~/projects``), not under
  ``/mnt/c``; file access across the boundary is much slower.
- The viser visualizer prints a ``http://localhost:<port>`` address; open it in your Windows
  browser (WSL2 forwards ``localhost`` by default).
- Building your own VAMP plugins (:doc:`../user_guide/vamp_plugins`) needs Docker inside
  WSL2: Docker Desktop with WSL integration enabled, or Docker Engine installed in the Ubuntu
  distribution.
- The CPU requirement (AVX2) applies inside WSL2 too.

Troubleshooting
---------------

**C++ library errors under conda.** If importing srmp in a conda environment fails with a
``GLIBCXX`` or other libstdc++ error, install a newer runtime:

.. code-block:: console

   $ conda install -c conda-forge libstdcxx-ng

**Older srmp installed.** ``pip install srmp`` picks the newest release with a wheel for your
platform and Python. ``pip show srmp`` tells you which version you got; on an unsupported
platform it is an old one, or none.
