package main

import (
	"encoding/json"
	"os"
	"testing"
)

func TestRunDetectLive(t *testing.T) {
	if os.Getenv("LOCORIX_LIVE") == "" {
		t.Skip("set LOCORIX_LIVE=1 to hit the Jetson")
	}
	c := newClient()
	snap, err := c.runDetect("", "")
	if err != nil {
		t.Fatal(err)
	}
	b, _ := json.MarshalIndent(snap, "", "  ")
	t.Log(string(b))
	if !snap.OK || snap.CX == nil {
		t.Fatalf("expected hole CX, got: %+v", snap)
	}
}
