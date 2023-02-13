// SPDX-License-Identifier: GPL-2.0

package ppk2

type Chunk struct {
	i  <-chan Samples
	C  chan Samples
	nb uint
}

func NewChunker(nb uint, c <-chan Samples) *Chunk {
	d := &Chunk{
		i:  c,
		C:  make(chan Samples, 100000/nb),
		nb: nb,
	}

	go d.chunker()

	return d
}

func (d *Chunk) chunker() {
	defer close(d.C)

	buf := make(Samples, d.nb)
	cnt := 0

	for i := range d.i {
		off := 0
		for {
			n := copy(buf[cnt:], i[off:])
			cnt += n
			off += n
			//fmt.Printf("n: %v - %v\n", n, cnt)

			if cnt == len(buf) {
				d.C <- buf
				buf = make(Samples, d.nb)
				cnt = 0
			}
			if n == 0 {
				break
			}
		}
	}
}
