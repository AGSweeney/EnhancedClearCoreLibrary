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
	"context"
	"encoding/json"
	"log"
	"os"
	"sync"

	"github.com/modelcontextprotocol/go-sdk/mcp"
)

type emptyArgs struct{}

type detectArgs struct {
	CameraID *string `json:"camera_id,omitempty" jsonschema:"Helios camera id (default 223400130)"`
	Preset   *string `json:"preset,omitempty" jsonschema:"Detect preset (default hole-locate)"`
}

type planMoveArgs struct {
	CameraID *string  `json:"camera_id,omitempty" jsonschema:"Helios camera id (default 223400130)"`
	Preset   *string  `json:"preset,omitempty" jsonschema:"Detect preset (default hole-locate)"`
	OffsetMM *float64 `json:"offset_mm,omitempty" jsonschema:"Calibration offset: move_x_mm = cx_mm * scale + offset_mm"`
	Scale    *float64 `json:"scale,omitempty" jsonschema:"Calibration scale on CX (default 1 when offset_mm set)"`
	RequireGaugesPass *bool `json:"require_gauges_pass,omitempty" jsonschema:"If true, go=false when gauges fail (default true)"`
}

type setCalArgs struct {
	OffsetMM *float64 `json:"offset_mm,omitempty" jsonschema:"Stored offset_mm for plan_move_x"`
	Scale    *float64 `json:"scale,omitempty" jsonschema:"Stored scale for plan_move_x (default 1)"`
	Clear    *bool    `json:"clear,omitempty" jsonschema:"If true, clear stored calibration"`
}

type movePlan struct {
	Go      bool     `json:"go"`
	CXMM    *float64 `json:"cx_mm"`
	MoveXMM *float64 `json:"move_x_mm"`
	Reason  string   `json:"reason"`
}

type calibration struct {
	mu       sync.Mutex
	has      bool
	offsetMM float64
	scale    float64
}

func (c *calibration) get() (offset, scale float64, ok bool) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if !c.has {
		return 0, 1, false
	}
	return c.offsetMM, c.scale, true
}

func (c *calibration) set(offset, scale float64) {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.has = true
	c.offsetMM = offset
	c.scale = scale
}

func (c *calibration) clear() {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.has = false
	c.offsetMM = 0
	c.scale = 1
}

func toolText(v any) (*mcp.CallToolResult, any, error) {
	b, err := json.Marshal(v)
	if err != nil {
		return &mcp.CallToolResult{
			Content: []mcp.Content{&mcp.TextContent{Text: err.Error()}},
			IsError: true,
		}, nil, nil
	}
	return &mcp.CallToolResult{
		Content: []mcp.Content{&mcp.TextContent{Text: string(b)}},
	}, v, nil
}

func toolErr(msg string) (*mcp.CallToolResult, any, error) {
	return &mcp.CallToolResult{
		Content: []mcp.Content{&mcp.TextContent{Text: msg}},
		IsError: true,
	}, nil, nil
}

func registerTools(server *mcp.Server, client *lrxClient, cal *calibration) {
	mcp.AddTool(server, &mcp.Tool{
		Name: "detect_hole",
		Description: "Run LocoRix Helios Detect (hole-locate). Returns camera-frame mm from result.symbols " +
			"(holes.0.cx/cy/cz, diameter_mm, score). Never invents CX. Does not estimate pose from images.",
	}, func(ctx context.Context, _ *mcp.CallToolRequest, args detectArgs) (*mcp.CallToolResult, any, error) {
		_ = ctx
		cam, preset := "", ""
		if args.CameraID != nil {
			cam = *args.CameraID
		}
		if args.Preset != nil {
			preset = *args.Preset
		}
		snap, err := client.runDetect(cam, preset)
		if err != nil {
			return toolErr(err.Error())
		}
		return toolText(snap)
	})

	mcp.AddTool(server, &mcp.Tool{
		Name: "get_cx",
		Description: "Run Detect and return holes.0.cx (camera-frame mm) only. " +
			"Fails clearly if Detect failed or holes.count is 0. Do not invent CX.",
	}, func(ctx context.Context, _ *mcp.CallToolRequest, args detectArgs) (*mcp.CallToolResult, any, error) {
		_ = ctx
		cam, preset := "", ""
		if args.CameraID != nil {
			cam = *args.CameraID
		}
		if args.Preset != nil {
			preset = *args.Preset
		}
		snap, err := client.runDetect(cam, preset)
		if err != nil {
			return toolErr(err.Error())
		}
		if !snap.OK || snap.CX == nil {
			reason := snap.Reason
			if reason == "" {
				reason = "no CX from Detect"
			}
			return toolText(map[string]any{
				"ok":       false,
				"cx_mm":    nil,
				"reason":   reason,
				"frame":    "camera",
				"camera_id": snap.CameraID,
			})
		}
		return toolText(map[string]any{
			"ok":         true,
			"cx_mm":      *snap.CX,
			"cy_mm":      snap.CY,
			"cz_mm":      snap.CZ,
			"diameter_mm": snap.DiameterMM,
			"score":      snap.Score,
			"gauges_pass": snap.GaugesPass,
			"frame":      "camera",
			"camera_id":  snap.CameraID,
			"reason":     "ok",
		})
	})

	mcp.AddTool(server, &mcp.Tool{
		Name: "set_calibration",
		Description: "Optional camera-to-motor X calibration for plan_move_x: " +
			"move_x_mm = cx_mm * scale + offset_mm. Without this, plan_move_x uses move_x_mm = cx_mm.",
	}, func(ctx context.Context, _ *mcp.CallToolRequest, args setCalArgs) (*mcp.CallToolResult, any, error) {
		_ = ctx
		if args.Clear != nil && *args.Clear {
			cal.clear()
			return toolText(map[string]any{"ok": true, "calibrated": false, "reason": "cleared"})
		}
		scale := 1.0
		if args.Scale != nil {
			scale = *args.Scale
		}
		offset := 0.0
		if args.OffsetMM != nil {
			offset = *args.OffsetMM
		}
		if args.OffsetMM == nil && args.Scale == nil {
			return toolErr("provide offset_mm and/or scale, or clear=true")
		}
		cal.set(offset, scale)
		return toolText(map[string]any{
			"ok":         true,
			"calibrated": true,
			"offset_mm":  offset,
			"scale":      scale,
			"formula":    "move_x_mm = cx_mm * scale + offset_mm",
		})
	})

	mcp.AddTool(server, &mcp.Tool{
		Name: "plan_move_x",
		Description: "Detect CX then return motor-move decision JSON only: " +
			`{"go":bool,"cx_mm":number|null,"move_x_mm":number|null,"reason":"..."}. ` +
			"Uses Detect symbols only. go=false if Detect fails, no hole, or gauges fail. " +
			"Default move_x_mm = cx_mm (camera frame). Optional offset_mm/scale or set_calibration: " +
			"move_x_mm = cx_mm * scale + offset_mm. Does not command ClearAI motors.",
	}, func(ctx context.Context, _ *mcp.CallToolRequest, args planMoveArgs) (*mcp.CallToolResult, any, error) {
		_ = ctx
		cam, preset := "", ""
		if args.CameraID != nil {
			cam = *args.CameraID
		}
		if args.Preset != nil {
			preset = *args.Preset
		}
		snap, err := client.runDetect(cam, preset)
		if err != nil {
			return toolText(movePlan{Go: false, Reason: err.Error()})
		}
		if !snap.OK || snap.CX == nil {
			reason := snap.Reason
			if reason == "" {
				reason = "Detect failed or no holes.0.cx"
			}
			return toolText(movePlan{Go: false, Reason: reason})
		}

		requireGauges := true
		if args.RequireGaugesPass != nil {
			requireGauges = *args.RequireGaugesPass
		}
		if requireGauges {
			if snap.GaugesHolesPass != nil && !*snap.GaugesHolesPass {
				return toolText(movePlan{Go: false, CXMM: snap.CX, Reason: "gauges.holes did not pass"})
			}
			if snap.GaugesPass != nil && !*snap.GaugesPass {
				return toolText(movePlan{Go: false, CXMM: snap.CX, Reason: "gauges_all_pass is false"})
			}
		}

		// Default: identity (camera-frame CX as move_x). Optional cal / args override.
		offset, scale := 0.0, 1.0
		reason := "ok; move_x_mm = cx_mm (camera frame)"
		if o, s, ok := cal.get(); ok {
			offset, scale = o, s
			reason = "ok; applied stored calibration"
		}
		if args.OffsetMM != nil || args.Scale != nil {
			scale = 1
			if args.Scale != nil {
				scale = *args.Scale
			}
			offset = 0
			if args.OffsetMM != nil {
				offset = *args.OffsetMM
			}
			reason = "ok; applied call offset/scale"
		}
		move := (*snap.CX)*scale + offset
		return toolText(movePlan{
			Go:      true,
			CXMM:    snap.CX,
			MoveXMM: &move,
			Reason:  reason,
		})
	})

	mcp.AddTool(server, &mcp.Tool{
		Name:        "get_config",
		Description: "Return LocoRix base URL, default camera id, preset, and stored calibration. No Detect call.",
	}, func(ctx context.Context, _ *mcp.CallToolRequest, _ emptyArgs) (*mcp.CallToolResult, any, error) {
		_ = ctx
		offset, scale, ok := cal.get()
		out := map[string]any{
			"base_url":   client.baseURL,
			"camera_id":  client.cameraID,
			"preset":     client.preset,
			"calibrated": ok,
			"frame":      "camera",
			"note":       "CX/CY/CZ from Detect are camera-frame mm until calibrated",
		}
		if ok {
			out["offset_mm"] = offset
			out["scale"] = scale
			out["formula"] = "move_x_mm = cx_mm * scale + offset_mm"
		}
		return toolText(out)
	})
}

func main() {
	log.SetFlags(0)
	client := newClient()
	cal := &calibration{}
	server := mcp.NewServer(&mcp.Implementation{
		Name:    "locorix",
		Title:   "LocoRix Helios Detect (structured CX only)",
		Version: "1.0.0",
	}, nil)
	registerTools(server, client, cal)
	if err := server.Run(context.Background(), &mcp.StdioTransport{}); err != nil {
		log.Printf("locorix-mcp: %v", err)
		os.Exit(1)
	}
}
