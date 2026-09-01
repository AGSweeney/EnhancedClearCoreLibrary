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

type readInputsArgs struct {
	Pin *int `json:"pin,omitempty" jsonschema:"ClearCore pin index 0-12; omit for all"`
}

type writeOutputArgs struct {
	Pin   int  `json:"pin" jsonschema:"IO pin index 0-5 only"`
	State bool `json:"state" jsonschema:"true=high false=low"`
}

type readAnalogArgs struct {
	Pin *int `json:"pin,omitempty" jsonschema:"A-9..A-12 pin index; omit for all analog inputs"`
}

type writeAnalogArgs struct {
	Pin       int  `json:"pin" jsonschema:"must be 0 (IO-0 analog current output)"`
	Value     *int `json:"value,omitempty" jsonschema:"raw 11-bit DAC 0-2047 (0-20mA)"`
	Microamps *int `json:"microamps,omitempty" jsonschema:"output current in microamps (0-20000); overrides value"`
}

type writePwmArgs struct {
	Pin  int `json:"pin" jsonschema:"IO pin index 0-5 only"`
	Duty int `json:"duty" jsonschema:"PWM duty 0-255"`
}

type subscribeInputsArgs struct {
	Pins       []int `json:"pins" jsonschema:"array of pin indexes 0-12 to monitor for edges"`
	DebounceMs *int  `json:"debounce_ms,omitempty" jsonschema:"min ms between edges per pin; default 5"`
}

type configureNetworkArgs struct {
	Mode     *string `json:"mode,omitempty" jsonschema:"dhcp or static"`
	IpAddress *string `json:"ip_address,omitempty" jsonschema:"static IP a.b.c.d; required for static"`
	Netmask  *string `json:"netmask,omitempty" jsonschema:"static netmask a.b.c.d"`
	Gateway  *string `json:"gateway,omitempty" jsonschema:"static gateway a.b.c.d; 0.0.0.0 = none"`
}

type jogVelocityArgs struct {
	X *float64 `json:"x,omitempty" jsonschema:"X velocity in user units/sec (signed)"`
	Y *float64 `json:"y,omitempty" jsonschema:"Y velocity in user units/sec (signed)"`
	Z *float64 `json:"z,omitempty" jsonschema:"Z velocity in user units/sec (signed)"`
	A *float64 `json:"a,omitempty" jsonschema:"A velocity in user units/sec (signed)"`
}

type moveBatchArgs struct {
	Moves []map[string]any `json:"moves" jsonschema:"array of move objects (linear/arc/dwell)"`
}

type homeArgs struct {
	Axis      string   `json:"axis" jsonschema:"x, y, z, or a"`
	Dir       string   `json:"dir" jsonschema:"pos or neg (which limit to seek)"`
	Feed      *float64 `json:"feed,omitempty" jsonschema:"homing feed in units/min"`
	Seek      *float64 `json:"seek,omitempty" jsonschema:"max travel in work units (default 1000)"`
	Backoff   *float64 `json:"backoff,omitempty" jsonschema:"retract after contact (work units)"`
	Zero     *bool    `json:"zero,omitempty" jsonschema:"set work origin = 0 at final position (default true)"`
	TimeoutMS *int    `json:"timeout_ms,omitempty" jsonschema:"overall timeout ms (default 30000)"`
}

type probeArgs struct {
	Axis      string   `json:"axis" jsonschema:"x, y, z, or a"`
	Dir       string   `json:"dir" jsonschema:"pos or neg"`
	Pin       int      `json:"pin" jsonschema:"probe input DI index 1-12"`
	Feed      *float64 `json:"feed,omitempty" jsonschema:"probe feed in units/min"`
	Seek      *float64 `json:"seek,omitempty" jsonschema:"max travel in work units (default 1000)"`
	Backoff   *float64 `json:"backoff,omitempty" jsonschema:"retract after touch (work units)"`
	Zero      *bool    `json:"zero,omitempty" jsonschema:"set work origin = 0 at touch point (default false)"`
	Active    string   `json:"active,omitempty" jsonschema:"probe polarity: high or low (default high)"`
	TimeoutMS *int    `json:"timeout_ms,omitempty" jsonschema:"overall timeout ms (default 30000)"`
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
	MinX         *float64 `json:"min_x,omitempty" jsonschema:"soft min in work units (mm/inch; A=deg)"`
	MaxX         *float64 `json:"max_x,omitempty"`
	MinY         *float64 `json:"min_y,omitempty"`
	MaxY         *float64 `json:"max_y,omitempty"`
	MinZ         *float64 `json:"min_z,omitempty"`
	MaxZ         *float64 `json:"max_z,omitempty"`
	MinA         *float64 `json:"min_a,omitempty"`
	MaxA         *float64 `json:"max_a,omitempty"`
	ClearLimits  *bool    `json:"clear_limits,omitempty"`
	ClearMinX    *bool    `json:"clear_min_x,omitempty"`
	ClearMaxX    *bool    `json:"clear_max_x,omitempty"`
	ClearMinY    *bool    `json:"clear_min_y,omitempty"`
	ClearMaxY    *bool    `json:"clear_max_y,omitempty"`
	ClearMinZ    *bool    `json:"clear_min_z,omitempty"`
	ClearMaxZ    *bool    `json:"clear_max_z,omitempty"`
	ClearMinA    *bool    `json:"clear_min_a,omitempty"`
	ClearMaxA    *bool    `json:"clear_max_a,omitempty"`
	PosLimX      *int     `json:"pos_lim_x,omitempty" jsonschema:"Pin index 0-5 IO in/out (forced input), 6-12 DI/A input-only; 0 or 255 disables"`
	NegLimX      *int     `json:"neg_lim_x,omitempty" jsonschema:"Pin index for X- limit; 0 or 255 disables"`
	PosLimY      *int     `json:"pos_lim_y,omitempty"`
	NegLimY      *int     `json:"neg_lim_y,omitempty"`
	PosLimZ      *int     `json:"pos_lim_z,omitempty"`
	NegLimZ      *int     `json:"neg_lim_z,omitempty"`
	PosLimA      *int     `json:"pos_lim_a,omitempty"`
	NegLimA      *int     `json:"neg_lim_a,omitempty"`
	WatchdogMs   *int     `json:"watchdog_ms,omitempty" jsonschema:"host keepalive timeout in ms; 0 disables"`
	VelX         *int     `json:"vel_x,omitempty" jsonschema:"per-axis velocity cap steps/s; 0 inherits global vel"`
	VelY         *int     `json:"vel_y,omitempty"`
	VelZ         *int     `json:"vel_z,omitempty"`
	VelA         *int     `json:"vel_a,omitempty"`
	AccelX       *int     `json:"accel_x,omitempty" jsonschema:"per-axis accel cap steps/s^2; 0 inherits global accel"`
	AccelY       *int     `json:"accel_y,omitempty"`
	AccelZ       *int     `json:"accel_z,omitempty"`
	AccelA       *int     `json:"accel_a,omitempty"`
	DecelX       *int     `json:"decel_x,omitempty" jsonschema:"per-axis decel cap steps/s^2; 0 inherits global decel"`
	DecelY       *int     `json:"decel_y,omitempty"`
	DecelZ       *int     `json:"decel_z,omitempty"`
	DecelA       *int     `json:"decel_a,omitempty"`
	OutPowerOn0  *int     `json:"out_power_on_0,omitempty" jsonschema:"power-on state for IO-0; 0/1 set, 255 don't care"`
	OutPowerOn1  *int     `json:"out_power_on_1,omitempty"`
	OutPowerOn2  *int     `json:"out_power_on_2,omitempty"`
	OutPowerOn3  *int     `json:"out_power_on_3,omitempty"`
	OutPowerOn4  *int     `json:"out_power_on_4,omitempty"`
	OutPowerOn5  *int     `json:"out_power_on_5,omitempty"`
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
	addRPC[emptyArgs](server, board, "get_config",
		"Return live ClearAI config and whether NVM has a valid persisted blob (axis_mask, test_mode, mechanics, vel).",
		"get_config", 5*time.Second)
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
		"Set mechanical parameters, axis mask, and optional soft travel limits (min_x/max_x, etc.). Mechanical fields require motors disabled; limits and test_mode may change while enabled. Persisted to NVM.",
		"configure", 5*time.Second)
	addRPC[emptyArgs](server, board, "reset_config",
		"Restore compile-time defaults, clear NVM config blob. Motors must be disabled first.",
		"reset_config", 5*time.Second)
	mcp.AddTool(server, &mcp.Tool{
		Name:        "set_test_mode",
		Description: "Bench test mode: bypass hardware estop, HLFB, alert, and enable gates. Persisted to NVM. Software stop/estop/disable still halt.",
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
		"Set linear units to mm or inch. Axis A is unaffected (use set_units_a).",
		"set_units", 5*time.Second)
	addRPC[unitsArgs](server, board, "set_units_a",
		"Set the rotary A-axis unit to degrees (deg) or revolutions (rev). Persisted to NVM.",
		"set_units_a", 5*time.Second)
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
	addRPC[readInputsArgs](server, board, "read_inputs",
		"Read raw digital state for ClearCore onboard pins 0-12 (IO-0..A-12). Optional pin for one connector.",
		"read_inputs", 5*time.Second)
	addRPC[writeOutputArgs](server, board, "write_output",
		"Drive IO-0..IO-5 as digital output (state true/false). Pins 6-12 are input-only.",
		"write_output", 5*time.Second)
	addRPC[emptyArgs](server, board, "queue_status",
		"Introspect the coordinated motion queue: pending segment count and active flag.",
		"queue_status", 5*time.Second)
	addRPC[emptyArgs](server, board, "queue_clear",
		"Flush the coordinated motion queue: decelerate the active move to a stop and drop pending segments. Motors stay enabled.",
		"queue_clear", 5*time.Second)
	addRPC[homeArgs](server, board, "home",
		"Home/zero one axis: seek the configured hardware limit (pos_lim_<axis>/neg_lim_<axis>) at a slow feed, stop on contact, optional backoff, optional zero. Blocking.",
		"home", 35*time.Second)
	addRPC[probeArgs](server, board, "probe",
		"Probe along one axis at a slow feed until the probe input DI triggers; reports touch position and optional zero. Blocking.",
		"probe", 35*time.Second)
	addRPC[emptyArgs](server, board, "keepalive",
		"Reset the host watchdog timer and clear any watchdog trip latch. Call periodically (faster than watchdog_ms) while a session is active.",
		"keepalive", 5*time.Second)
	addRPC[readAnalogArgs](server, board, "read_analog",
		"Read analog voltage (volts) and raw ADC for A-9..A-12. Optional pin for one connector.",
		"read_analog", 5*time.Second)
	addRPC[writeAnalogArgs](server, board, "write_analog",
		"Drive IO-0 analog current output: raw 11-bit value (0-2047, 0-20mA) or microamps.",
		"write_analog", 5*time.Second)
	addRPC[writePwmArgs](server, board, "write_pwm",
		"Drive IO-0..IO-5 as PWM output (duty 0-255). Frequency is fixed by the timer.",
		"write_pwm", 5*time.Second)
	addRPC[subscribeInputsArgs](server, board, "subscribe_inputs",
		"Opt-in to edge notifications for the given pins (0-12). input_changed events are pushed on the telemetry stream (port 9101).",
		"subscribe_inputs", 5*time.Second)
	addRPC[emptyArgs](server, board, "unsubscribe_inputs",
		"Stop input edge notifications previously enabled by subscribe_inputs.",
		"unsubscribe_inputs", 5*time.Second)
	addRPC[configureNetworkArgs](server, board, "configure_network",
		"Configure network IP mode (dhcp or static) and static IP/netmask/gateway. Persisted to NVM; applies on next restart. Allowed while motors are enabled.",
		"configure_network", 5*time.Second)
	addRPC[emptyArgs](server, board, "restart",
		"Reset the ClearCore board to apply pending network config (and re-run the application). The TCP connection will drop.",
		"restart", 5*time.Second)
	addRPC[jogVelocityArgs](server, board, "jog_velocity",
		"Start continuous per-axis velocity jog (user units/sec, signed). Hardware limits auto-stop; soft limits are not enforced. Stop with jog_stop or stop.",
		"jog_velocity", 5*time.Second)
	addRPC[emptyArgs](server, board, "jog_stop",
		"Decelerate-stop any active jog_velocity motion.",
		"jog_stop", 5*time.Second)
	addRPC[moveBatchArgs](server, board, "move_batch",
		"Queue a batch of moves (array of linear/arc/dwell objects) in one round-trip. Stops at the first rejected element.",
		"move_batch", 5*time.Second)
	addRPC[emptyArgs](server, board, "get_log",
		"Return the recent motion log (rejected moves and info events) as a JSON array, most-recent first.",
		"get_log", 5*time.Second)
	addRPC[emptyArgs](server, board, "clear_log",
		"Clear the motion log ring buffer.",
		"clear_log", 5*time.Second)
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
