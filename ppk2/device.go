// SPDX-License-Identifier: GPL-2.0

package ppk2

import (
	"bufio"
	"fmt"
	"os"
	"path/filepath"
	"runtime"
	"strings"

	"go.bug.st/serial/enumerator"
)

func readLine(filename string) (string, error) {
	file, err := os.Open(filename)
	if os.IsNotExist(err) {
		return "", nil
	}
	if err != nil {
		return "", err
	}
	defer file.Close()
	reader := bufio.NewReader(file)
	line, _, err := reader.ReadLine()
	return string(line), err
}

func ListDevices() []*enumerator.PortDetails {
	l := []*enumerator.PortDetails{}

	ports, err := enumerator.GetDetailedPortsList()
	if err != nil {
		return l
	}

	for _, p := range ports {
		if !p.IsUSB {
			continue
		}

		if strings.HasPrefix(p.Product, "nRF Connect USB CDC ACM") || strings.Contains(p.Product, "PPK2") {
			l = append(l, p)
			continue
		}
		if runtime.GOOS == "linux" {
			portName := filepath.Base(p.Name)
			devicePath := fmt.Sprintf("/sys/class/tty/%s/device", portName)
			realDevicePath, _ := filepath.EvalSymlinks(devicePath)
			usbDevPath := filepath.Dir(realDevicePath)

			if product, err := readLine(filepath.Join(usbDevPath, "product")); err == nil && product == "PPK2" {
				l = append(l, p)
				continue
			}
		}
	}

	return l
}
