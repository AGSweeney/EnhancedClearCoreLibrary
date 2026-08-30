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
	"strconv"
	"time"

	"github.com/modelcontextprotocol/go-sdk/mcp"
)

type emptyArgs struct{}

type waitIdleArgs struct {
	TimeoutMS *int `json:"timeout_ms,omitempty" jsonschema:"milliseconds to wait (default 60000)"`
}

type testModeArgs struct {
	Enabled  *bool `json:"enabled,omitempty" jsonschema:"true bypasses hardware safety gates"`
	TestMode *bool `json:"test_mode,omitempty" jsonschema:"alias for enabled"`
}

type unitsArgs struct {
	Units string `json:"units" jsonschema:"mm or inch"`
}

type modeArgs struct {
	Mode string `json:"mode" jsonschema:"abs or rel"`
}

type poseArgs struct {
	X *float64 `json:"x,omitempty"`
	Y *float64 `json:"y,omitempty"`
	Z *float64 `json:"z,omitempty"`
	A *float64 `json:"a,omitempty"`
}

type moveLinearArgs struct {
	X     *float64 `json:"x,omitempty"`
	Y     *float64 `json:"y,omitempty"`
	Z     *float64 `json:"z,omitempty"`
	A     *float64 `json:"a,omitempty"`
	Feed  *float64 `json:"feed,omitempty" jsonschema:"units per minute"`
	Rapid *bool    `json:"rapid,omitempty"`
}

type moveArcArgs struct {
	X         *float64 `json:"x,omitempty"`
	Y         *float64 `json:"y,omitempty"`
	I         float64  `json:"i" jsonschema:"center offset X from start"`
	J         float64  `json:"j" jsonschema:"center offset Y from start"`
	Clockwise *bool    `json:"clockwise,omitempty"`
	Feed      *float64 `json:"feed,omitempty"`
}

type jogArgs struct {
	X    *float64 `json:"x,omitempty"`
	Y    *float64 `json:"y,omitempty"`
	Z    *float64 `json:"z,omitempty"`
	A    *float64 `json:"a,omitempty"`
	Feed *float64 `json:"feed,omitempty"`
}

type dwellArgs struct {
	Seconds float64 `json:"seconds"`
}

type configureArgs struct {
	StepsPerRevX *int     `json:"steps_per_rev_x,omitempty"`
	StepsPerRevY *int     `json:"steps_per_rev_y,omitempty"`
	StepsPerRevZ *int     `json:"steps_per_rev_z,omitempty"`
	StepsPerRevA *int     `json:"steps_per_rev_a,omitempty"`
	PitchX       *float64 `json:"pitch_x,omitempty"`
	PitchY       *float64 `json:"pitch_y,omitempty"`
	PitchZ       *float64 `json:"pitch_z,omitempty"`
	GearX        *float64 `json:"gear_x,omitempty"`
	GearY        *float64 `json:"gear_y,omitempty"`
	GearZ        *float64 `json:"gear_z,omitempty"`
	GearA        *float64 `json:"gear_a,omitempty"`
	Vel          *int     `json:"vel,omitempty" jsonschema:"max velocity steps/s"`
	Accel        *int     `json:"accel,omitempty"`
	Decel        *int     `json:"decel,omitempty"`
	AxisMask     *int     `json:"axis_mask,omitempty" jsonschema:"bit0 X, bit1 Y, bit2 Z, bit3 A"`
	EstopDI6     *int     `json:"estop_di6,omitempty"`
	TestMode     *bool    `json:"test_mode,omitempty"`
}

func resultJSON(raw json.RawMessage, err error) (*mcp.CallToolResult, any, error) {
	if err != nil {
		return &mcp.CallToolResult{
			Content: []mcp.Content{&mcp.TextContent{Text: err.Error()}},
			IsError: true,
		}, nil, nil
	}
	text := string(raw)
	if text == "" {
		text = "null"
	}
	var v any
	_ = json.Unmarshal(raw, &v)
	return &mcp.CallToolResult{
		Content: []mcp.Content{&mcp.TextContent{Text: text}},
	}, v, nil
}

func addRPC[T any](server *mcp.Server, board *rpcClient, name, desc, method string, timeout time.Duration) {
	mcp.AddTool(server, &mcp.Tool{Name: name, Description: desc},
		func(ctx context.Context, _ *mcp.CallToolRequest, args T) (*mcp.CallToolResult, any, error) {
			_ = ctx
			return resultJSON(board.call(method, args, timeout))
		})
}

func registerTools(server *mcp.Server, board *rpcClient) {
	addRPC[emptyArgs](server, board, "get_capabilities",
		"Return ClearAI protocol version, enabled axes, units, mode, and allowed methods. Call this first.",
		"get_capabilities", 5*time.Second)
	addRPC[emptyArgs](server, board, "get_status",
		"Return enable/moving/HLFB/estop/alert flags, queue depth, and work pose.",
		"get_status", 5*time.Second)
	addRPC[emptyArgs](server, board, "get_pose",
		"Return work coordinates x,y,z,a in the active units (A is always degrees).",
		"get_pose", 5*time.Second)
	addRPC[emptyArgs](server, board, "enable",
		"Enable ClearPath motors on the configured axis mask. Required before motion.",
		"enable", 5*time.Second)
	addRPC[emptyArgs](server, board, "disable",
		"Abruptly stop and disable all motors.",
		"disable", 5*time.Second)
	addRPC[emptyArgs](server, board, "clear_alerts",
		"Clear motor alert flags after a fault.",
		"clear_alerts", 5*time.Second)
	addRPC[emptyArgs](server, board, "stop",
		"Decelerate and stop motion. Motors stay enabled.",
		"stop", 5*time.Second)
	addRPC[emptyArgs](server, board, "estop",
		"Immediate stop and disable. Use for safety. Accepted during wait_idle and dwell.",
		"estop", 5*time.Second)
	mcp.AddTool(server, &mcp.Tool{
		Name:        "wait_idle",
		Description: "Block until queued motion finishes or timeout_ms elapses. Always call after move_linear, move_arc, or jog.",
	}, func(ctx context.Context, _ *mcp.CallToolRequest, args waitIdleArgs) (*mcp.CallToolResult, any, error) {
		_ = ctx
		ms := 60000
		if args.TimeoutMS != nil && *args.TimeoutMS > 0 {
			ms = *args.TimeoutMS
		}
		return resultJSON(board.call("wait_idle", args, time.Duration(ms+2000)*time.Millisecond))
	})
	addRPC[configureArgs](server, board, "configure",
		"Set mechanical parameters and axis mask. Motors must be disabled first, except test_mode.",
		"configure", 5*time.Second)
	mcp.AddTool(server, &mcp.Tool{
		Name:        "set_test_mode",
		Description: "Bench test mode: bypass hardware estop, HLFB, alert, and enable gates. Software stop/estop/disable still halt.",
	}, func(ctx context.Context, _ *mcp.CallToolRequest, args testModeArgs) (*mcp.CallToolResult, any, error) {
		_ = ctx
		params := map[string]any{}
		if args.Enabled != nil {
			params["enabled"] = *args.Enabled
		} else if args.TestMode != nil {
			params["enabled"] = *args.TestMode
		} else {
			params["enabled"] = true
		}
		return resultJSON(board.call("set_test_mode", params, 5*time.Second))
	})
	addRPC[unitsArgs](server, board, "set_units",
		"Set linear units to mm or inch. Axis A remains degrees.",
		"set_units", 5*time.Second)
	addRPC[modeArgs](server, board, "set_mode",
		"Set absolute or relative coordinate mode for move_linear and move_arc.",
		"set_mode", 5*time.Second)
	addRPC[poseArgs](server, board, "set_work_origin",
		"G92-style work offset. Empty object zeros all axes at the current machine pose.",
		"set_work_origin", 5*time.Second)
	addRPC[moveLinearArgs](server, board, "move_linear",
		"Queue a linear move (optional Z/A). Non-blocking; follow with wait_idle.",
		"move_linear", 5*time.Second)
	addRPC[moveArcArgs](server, board, "move_arc",
		"Queue an XY arc. i,j are center offsets from the start. No Z/A on the same call.",
		"move_arc", 5*time.Second)
	addRPC[jogArgs](server, board, "jog",
		"Relative jog; ignores abs/rel mode.",
		"jog", 5*time.Second)
	mcp.AddTool(server, &mcp.Tool{
		Name:        "dwell",
		Description: "Wait seconds (max 600). Interruptible by estop.",
	}, func(ctx context.Context, _ *mcp.CallToolRequest, args dwellArgs) (*mcp.CallToolResult, any, error) {
		_ = ctx
		sec := args.Seconds
		if sec < 0 {
			sec = 0
		}
		return resultJSON(board.call("dwell", args, time.Duration(sec+2)*time.Second))
	})
	mcp.AddTool(server, &mcp.Tool{
		Name:        "discover",
		Description: "UDP-broadcast CLEARAI_DISCOVER? and return the board IP and TCP ports. Does not move motors.",
	}, func(ctx context.Context, _ *mcp.CallToolRequest, _ emptyArgs) (*mcp.CallToolResult, any, error) {
		_ = ctx
		ip, err := discover(2500*time.Millisecond, 9102)
		out := map[string]any{"tcp": 9100, "tel": 9101, "discover": 9102}
		if err != nil {
			out["error"] = err.Error()
			b, _ := json.Marshal(out)
			return resultJSON(b, nil)
		}
		out["host"] = ip
		b, _ := json.Marshal(out)
		return resultJSON(b, nil)
	})
}

func main() {
	log.SetFlags(0)
	tcpPort := 9100
	if v := os.Getenv("CLEARAI_TCP_PORT"); v != "" {
		if n, err := strconv.Atoi(v); err == nil && n > 0 {
			tcpPort = n
		}
	}
	board := &rpcClient{port: tcpPort}
	server := mcp.NewServer(&mcp.Implementation{
		Name:    "clearai",
		Title:   "ClearAI gantry (UDP discover + TCP 9100)",
		Version: "1.0.0",
	}, nil)
	registerTools(server, board)
	if err := server.Run(context.Background(), &mcp.StdioTransport{}); err != nil {
		log.Printf("clearai-mcp: %v", err)
		os.Exit(1)
	}
}
