/*
 * Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package main

import (
	"bytes"
	"encoding/json"
	"fmt"
	"io"
	"net/http"
	"os"
	"strconv"
	"time"
)

const defaultBaseURL = "http://172.16.82.121:8080"
const defaultCameraID = "223400130"
const defaultPreset = "hole-locate"

type lrxClient struct {
	baseURL  string
	cameraID string
	preset   string
	http     *http.Client
}

type detectRequest struct {
	CameraID string `json:"cameraId"`
	Preset   string `json:"preset"`
	Persist  bool   `json:"persist"`
}

type gauge struct {
	Name  string  `json:"name"`
	Value float64 `json:"value"`
	Lo    float64 `json:"lo"`
	Hi    float64 `json:"hi"`
	Pass  bool    `json:"pass"`
}

type detectResult struct {
	OK      bool               `json:"ok"`
	Message string             `json:"message"`
	Symbols map[string]float64 `json:"symbols"`
	Gauges  []gauge            `json:"gauges"`
}

type detectResponse struct {
	Status int `json:"status"`
	Result *detectResult `json:"result"`
	Error  string        `json:"error"`
}

type holeSnapshot struct {
	OK           bool    `json:"ok"`
	CameraID     string  `json:"camera_id"`
	Preset       string  `json:"preset"`
	Message      string  `json:"message,omitempty"`
	HolesCount   int     `json:"holes_count"`
	CX           *float64 `json:"cx_mm,omitempty"`
	CY           *float64 `json:"cy_mm,omitempty"`
	CZ           *float64 `json:"cz_mm,omitempty"`
	DiameterMM   *float64 `json:"diameter_mm,omitempty"`
	Score        *float64 `json:"score,omitempty"`
	GaugesPass   *bool    `json:"gauges_pass,omitempty"`
	GaugesHolesPass *bool `json:"gauges_holes_pass,omitempty"`
	Frame        string   `json:"frame"`
	Reason       string   `json:"reason,omitempty"`
}

func newClient() *lrxClient {
	base := os.Getenv("LOCORIX_BASE_URL")
	if base == "" {
		base = defaultBaseURL
	}
	cam := os.Getenv("LOCORIX_CAMERA_ID")
	if cam == "" {
		cam = defaultCameraID
	}
	preset := os.Getenv("LOCORIX_PRESET")
	if preset == "" {
		preset = defaultPreset
	}
	timeout := 20 * time.Second
	if v := os.Getenv("LOCORIX_TIMEOUT_MS"); v != "" {
		if n, err := strconv.Atoi(v); err == nil && n > 0 {
			timeout = time.Duration(n) * time.Millisecond
		}
	}
	return &lrxClient{
		baseURL:  base,
		cameraID: cam,
		preset:   preset,
		http:     &http.Client{Timeout: timeout},
	}
}

func symbolFloat(symbols map[string]float64, key string) (float64, bool) {
	if symbols == nil {
		return 0, false
	}
	v, ok := symbols[key]
	return v, ok
}

func ptr(v float64) *float64 { return &v }

func (c *lrxClient) runDetect(cameraID, preset string) (*holeSnapshot, error) {
	if cameraID == "" {
		cameraID = c.cameraID
	}
	if preset == "" {
		preset = c.preset
	}
	body, err := json.Marshal(detectRequest{
		CameraID: cameraID,
		Preset:   preset,
		Persist:  false,
	})
	if err != nil {
		return nil, err
	}
	url := c.baseURL + "/api/lrx/run"
	req, err := http.NewRequest(http.MethodPost, url, bytes.NewReader(body))
	if err != nil {
		return nil, err
	}
	req.Header.Set("Content-Type", "application/json")
	resp, err := c.http.Do(req)
	if err != nil {
		return nil, fmt.Errorf("locorix POST %s: %w", url, err)
	}
	defer resp.Body.Close()
	raw, err := io.ReadAll(resp.Body)
	if err != nil {
		return nil, err
	}
	if resp.StatusCode < 200 || resp.StatusCode >= 300 {
		return nil, fmt.Errorf("locorix HTTP %d: %s", resp.StatusCode, truncate(string(raw), 200))
	}

	// Symbols may arrive as numbers; decode flexibly.
	var envelope struct {
		Status int             `json:"status"`
		Error  string          `json:"error"`
		Result json.RawMessage `json:"result"`
	}
	if err := json.Unmarshal(raw, &envelope); err != nil {
		return nil, fmt.Errorf("decode envelope: %w", err)
	}
	if envelope.Error != "" && envelope.Result == nil {
		return &holeSnapshot{
			OK:       false,
			CameraID: cameraID,
			Preset:   preset,
			Frame:    "camera",
			Reason:   envelope.Error,
			Message:  envelope.Error,
		}, nil
	}
	if len(envelope.Result) == 0 {
		return &holeSnapshot{
			OK:       false,
			CameraID: cameraID,
			Preset:   preset,
			Frame:    "camera",
			Reason:   "no result in Detect response",
		}, nil
	}

	var result struct {
		OK      bool                       `json:"ok"`
		Message string                     `json:"message"`
		Symbols map[string]json.RawMessage `json:"symbols"`
		Gauges  []gauge                    `json:"gauges"`
	}
	if err := json.Unmarshal(envelope.Result, &result); err != nil {
		return nil, fmt.Errorf("decode result: %w", err)
	}

	syms := map[string]float64{}
	for k, rawV := range result.Symbols {
		var f float64
		if err := json.Unmarshal(rawV, &f); err == nil {
			syms[k] = f
			continue
		}
		var b bool
		if err := json.Unmarshal(rawV, &b); err == nil {
			if b {
				syms[k] = 1
			} else {
				syms[k] = 0
			}
		}
	}

	out := &holeSnapshot{
		OK:       result.OK,
		CameraID: cameraID,
		Preset:   preset,
		Message:  result.Message,
		Frame:    "camera",
	}
	if count, ok := symbolFloat(syms, "holes.count"); ok {
		out.HolesCount = int(count)
	}
	if v, ok := symbolFloat(syms, "holes.0.cx"); ok {
		out.CX = ptr(v)
	}
	if v, ok := symbolFloat(syms, "holes.0.cy"); ok {
		out.CY = ptr(v)
	}
	if v, ok := symbolFloat(syms, "holes.0.cz"); ok {
		out.CZ = ptr(v)
	}
	if v, ok := symbolFloat(syms, "holes.0.diameter_mm"); ok {
		out.DiameterMM = ptr(v)
	}
	if v, ok := symbolFloat(syms, "holes.0.score"); ok {
		out.Score = ptr(v)
	}
	if v, ok := symbolFloat(syms, "gauges_all_pass"); ok {
		b := v != 0
		out.GaugesPass = &b
	}
	if v, ok := symbolFloat(syms, "gauges.holes.pass"); ok {
		b := v != 0
		out.GaugesHolesPass = &b
	}
	for _, g := range result.Gauges {
		if g.Name == "holes" {
			b := g.Pass
			out.GaugesHolesPass = &b
		}
	}

	if !result.OK {
		out.Reason = "Detect result.ok is false"
		if result.Message != "" {
			out.Reason = result.Message
		}
		return out, nil
	}
	if out.HolesCount <= 0 || out.CX == nil {
		out.OK = false
		out.Reason = "Detect succeeded but holes.count is 0 / no holes.0.cx"
		return out, nil
	}
	out.Reason = "ok"
	return out, nil
}

func truncate(s string, n int) string {
	if len(s) <= n {
		return s
	}
	return s[:n] + "..."
}
