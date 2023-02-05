package ppk2

import "bytes"

type bytesReader struct {
	*bytes.Reader
}

// NewReader returns a new Reader reading from b.
func newBytesReader(b []byte) *bytesReader {
	return &bytesReader{bytes.NewReader(b)}
}

// ReadUint32 returns a uint32
func (r *bytesReader) ReadUint32() (u uint32, err error) {
	a := make([]byte, 4)
	if _, err = r.Read(a); err != nil {
		return
	}
	u = uint32(a[0]) | uint32(a[1])<<8 | uint32(a[2])<<16 | uint32(a[3])<<24
	return
}
