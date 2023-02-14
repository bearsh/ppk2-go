[![Go Reference](https://pkg.go.dev/badge/github.com/bearsh/ppk2-go.svg)](https://pkg.go.dev/github.com/bearsh/ppk2-go)
[![License](https://img.shields.io/github/license/bearsh/ppk2-go)](LICENSE.md)

# ppk2-go

This projects provides a golang API and a command line tool to access the Nordic
Power Profile Kit 2. The code is heavily based on the the
[ppk2-api-python](https://github.com/IRNAS/ppk2-api-python) project.

## CLI

```
Usage: ppk2-cli [OPTION]
  -l, --list                  List PPK2 devices
  -p, --port port             Serial port
  -s, --sample-time seconds   Sample time in seconds (default 0.1)
      --source                Source the target, needs voltage to be specified
      --voltage mV            Voltage (in mV) to source or expected voltage in ampere meter mod
```

## License

As this code is mostly based on
[ppk2-api-python](https://github.com/IRNAS/ppk2-api-python) it uses the same
licesne which is the **GPLv2**.
