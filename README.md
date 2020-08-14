# MCHA3500

## Getting started

1. Refer to the toolchain setup instructions to create a root directory to store this repo and auxilliary files.

    This directory can be populated with support tools that shouldn't be in the git repository, for example,

    * `mcha3500`
        * `doc`
        * `drivers`
        * `util`
        * `workspace` (this repo)
        * `toolchain`
        * `shell.bat`

    Follow the steps below to setup the `workspace` subdirectory.

1.  Clone this repo into a new `workspace` subdirectory of `mcha3500`,

        cd mcha3500
        git clone --recurse-submodules https://bitbucket.org/uonmcha3500_2020/mcha3500.git workspace

    The `--recurse-submodules` option will automatically setup the dependent submodules.

1.  Profit

        cd workspace/robot
        make

