# OpenThread on Realtek RTL8777G Example

This repo contains example platform drivers for the [Realtek RTL8777G][RTL8777G].

[RTL8777G]: https://www.realtek.com/

## Downlod submoudle
In a Bash terminal, follow these instructions to clone all submoudle.

```bash
$ cd <path-to-ot-realtek>
$ git submodule update --init --recursive
```

## Building

In a Bash terminal, follow these instructions to build the RTL8777G examples.

```bash
$ cd <path-to-ot-realtek>
$ OT_CMAKE_NINJA_TARGET="ot-cli-ftd" ./Realtek/build bee4 sdk rtl8777g
```

## Flash Binaries

If the build completed successfully, the `bin` files may be found in `<path-to-realtek>/build/bin/`.

To flash the images with [MPCli tool][MPCli], we should copy image file to MPCli tool file.

```bash
$ cd <path-to-ot-realtek>
$ cp ./build/bin/<ot-cli-ftd_MP_dev_0.0.0.0_XXXX.bin> ./tool/mpcli/bin
```

Then we should modify mptool config file.

```bash
$ vim ./tool/mpcli/mptoolconfig.json
```

Replace <ot-cli-ftd.bin> with image file name.

Program file with MPCli.
```bash
$ sudo mpcli -f mptoolconfig.json -c <serial port> -a -r
```
Example: ```sudo mpcli -f mptoolconfig.json -c /dev/ttyUSB0 -a -r```

## Interact

1. Open terminal to `/dev/ttyACM0` (serial port settings: 115200 8-N-1).
2. Type `help` for list of commands.
3. See [OpenThread CLI Reference README.md][cli] to learn more.

[cli]: https://github.com/openthread/openthread/blob/main/src/cli/README.md

# Contributing

We would love for you to contribute to OpenThread and help make it even better than it is today! See our [Contributing Guidelines](https://github.com/openthread/openthread/blob/main/CONTRIBUTING.md) for more information.

Contributors are required to abide by our [Code of Conduct](https://github.com/openthread/openthread/blob/main/CODE_OF_CONDUCT.md) and [Coding Conventions and Style Guide](https://github.com/openthread/openthread/blob/main/STYLE_GUIDE.md).

# License

OpenThread is released under the [BSD 3-Clause license](https://github.com/openthread/ot-realtek/blob/main/LICENSE). See the [`LICENSE`](https://github.com/openthread/ot-realtek/blob/main/LICENSE) file for more information.

Please only use the OpenThread name and marks when accurately referencing this software distribution. Do not use the marks in a way that suggests you are endorsed by or otherwise affiliated with Nest, Google, or The Thread Group.

# Need help?

OpenThread support is available on GitHub:

- Bugs and feature requests pertaining to the OpenThread on Realtek Example — [submit to the openthread/ot-realtek Issue Tracker](https://github.com/openthread/ot-realtek/issues)
- OpenThread bugs and feature requests — [submit to the OpenThread Issue Tracker](https://github.com/openthread/openthread/issues)
- Community Discussion - [ask questions, share ideas, and engage with other community members](https://github.com/openthread/openthread/discussions)

