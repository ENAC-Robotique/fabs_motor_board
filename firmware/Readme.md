Testeur software
================

To build the firmware, there is few thing you need to have.

First, Fetch ChibiOs and some third party contributions with `git submodule update --init --recursive`.

__Install Perl dependencies (used to generate board.h file from board.cfg):__

```
sudo apt update
sudo apt install libmodern-perl-perl libxml-libxml-perl
sudo perl -MCPAN -e 'install String::LCSS'
```

__Install GCC for ARM:__

`sudo apt install gcc-arm-none-eabi gdb-multiarch`

__Install STM32CubeMX__

Download and install SMT32CubeMx with admin rights (need to signup on st.com): [http://www.st.com/en/development-tools/stm32cubemx.html](http://www.st.com/en/development-tools/stm32cubemx.html).

Make sure it is installed in `/usr/local` for linux users (_not_ in your $HOME) or `C:\Program Files` for windows users.


Protobuf
--------

This firmware uses EmbeddedProto. See https://github.com/Embedded-AMS/EmbeddedProto

`git submodule update --init --recursive`

You need to install `protobuf-compiler`: `sudo apt install protobuf-compiler`

Then you need to run: `cd EmbeddedProto && ./setup.sh`

Run `make generate` to generate source code from protobuf files.


