// SPDX-License-Identifier: GPL-2.0

package main

import (
	"context"
	"errors"
	"fmt"
	"os"
	"os/signal"
	"path/filepath"
	"sync"
	"time"

	"github.com/bearsh/ppk2-go/ppk2"
	flag "github.com/spf13/pflag"
)

var (
	AppName string
	Version string
)

var (
	port       = flag.StringP("port", "p", "", "Serial `port` of device")
	sn         = flag.String("sn", "", "Serial number (`sn`) of device")
	list       = flag.BoolP("list", "l", false, "List PPK2 devices")
	voltage    = flag.Uint("voltage", 0, "Voltage (in `mV`) to source or expected voltage in ampere meter mode")
	source     = flag.Bool("source", false, "Source the target, needs voltage to be specified")
	sampleTime = flag.Float32P("sample-time", "s", 0.1, "Sample time in `seconds`")
	runTime    = flag.Float32("time", 0, "The application quits after the given number of `seconds`")
	version    = flag.Bool("version", false, "Display the version and exit")
)

func init() {
	_, AppName = filepath.Split(os.Args[0])
	if Version == "" {
		Version = "unknown"
	}

	flag.ErrHelp = errors.New("help requested")
	flag.Usage = func() {
		fmt.Fprintf(os.Stderr, "Usage: %s [OPTION]\n", AppName)
		flag.PrintDefaults()
	}
}

func main() {
	flag.Parse()

	if *version {
		fmt.Printf("%s\n", Version)
		os.Exit(0)
	}

	devs := ppk2.ListDevices()

	if *list {
		for _, i := range devs {
			fmt.Printf("- Port: %s, SN: %s\n", i.Name, i.SerialNumber)
		}
		os.Exit(0)
	}

	if *port == "" && *sn == "" {
		if len(devs) > 0 {
			*port = devs[0].Name
		}
	} else if *sn != "" {
		for _, i := range devs {
			if i.SerialNumber == *sn {
				*port = i.Name
				break
			}
		}
	}
	if *port == "" {
		fmt.Fprintf(os.Stderr, "no port or serial number give and no connected devices found\n")
		os.Exit(-1)
	}

	if *source && *voltage == 0 {
		fmt.Fprintf(os.Stderr, "source to the target requested but no voltage specified\n")
		os.Exit(-1)
	}

	p, err := ppk2.NewPPK2(*port)
	if err != nil {
		fmt.Fprintf(os.Stderr, "%v\n", err)
		os.Exit(1)
	}

	chunkSize := uint(1)
	if *sampleTime > 0 {
		chunkSize = uint(100000 * *sampleTime)
	}

	p.GetModifiers()

	if *source {
		p.UseSourceMeter()
	} else {
		p.UseAmpereMeter()
	}

	if *voltage != 0 {
		p.SetSourceVoltage(*voltage)
	}

	p.ToggleDUTPower(true)

	d := p.StartReader()
	ds := ppk2.NewChunker(chunkSize, d)

	wg := sync.WaitGroup{}

	f := func() {
		defer wg.Done()
		wg.Add(1)

		startTime := time.Now()
		for i := range ds.C {
			fmt.Printf("v:%v;%v;%v\n", time.Since(startTime), i.Average(), len(i))
		}
		fmt.Printf("\n")
	}
	go f()

	ctx, stop := context.WithCancel(context.Background())

	if *runTime > 0 {
		time.AfterFunc(time.Duration(*runTime*float32(time.Second)), func() {
			stop()
		})
	}

	ctx, _ = signal.NotifyContext(ctx, os.Interrupt)
	<-ctx.Done()
	stop()

	p.StopReader()
	p.ToggleDUTPower(false)

	wg.Wait()
}
