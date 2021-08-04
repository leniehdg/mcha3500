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

2.  Clone this repo into a new `workspace` subdirectory of `mcha3500`,

        cd mcha3500
        git clone -b master --recurse-submodules https://bitbucket.org/uon-mcha3500/mcha3500_labs_2021.git workspace

    The `-b master` will only clone the master branch of the repository.
    The `--recurse-submodules` option will automatically setup the dependent submodules.

1.  Profit

        cd workspace/mcha3500labs
        make

