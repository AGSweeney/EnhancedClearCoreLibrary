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
	"bufio"
	"encoding/json"
	"fmt"
	"net"
	"os"
	"regexp"
	"sync"
	"time"
)

var ipRe = regexp.MustCompile(`IP=(\d{1,3}(?:\.\d{1,3}){3})`)

func discover(timeout time.Duration, udpPort int) (string, error) {
	conn, err := net.ListenUDP("udp4", &net.UDPAddr{IP: net.IPv4zero, Port: 0})
	if err != nil {
		return "", err
	}
	defer conn.Close()
	_ = conn.SetDeadline(time.Now().Add(timeout))
	req := []byte("CLEARAI_DISCOVER?")
	dests := []string{
		"255.255.255.255",
		"192.168.0.255",
		"192.168.1.255",
		"172.16.255.255",
		"172.16.82.255",
	}
	for _, d := range dests {
		addr, err := net.ResolveUDPAddr("udp4", net.JoinHostPort(d, fmt.Sprintf("%d", udpPort)))
		if err != nil {
			continue
		}
		_, _ = conn.WriteToUDP(req, addr)
	}
	buf := make([]byte, 256)
	for {
		n, addr, err := conn.ReadFromUDP(buf)
		if err != nil {
			return "", err
		}
		text := string(buf[:n])
		if m := ipRe.FindStringSubmatch(text); len(m) == 2 {
			return m[1], nil
		}
		if len(text) > 0 {
			return addr.IP.String(), nil
		}
	}
}

type rpcClient struct {
	mu     sync.Mutex
	conn   net.Conn
	reader *bufio.Reader
	nextID int
	host   string
	port   int
}

func (c *rpcClient) ensure() error {
	if c.conn != nil {
		return nil
	}
	host := os.Getenv("CLEARAI_HOST")
	if host == "" {
		found, err := discover(2500*time.Millisecond, 9102)
		if err != nil || found == "" {
			host = os.Getenv("CLEARAI_FALLBACK_IP")
			if host == "" {
				host = "192.168.0.109"
			}
		} else {
			host = found
		}
	}
	port := c.port
	if port == 0 {
		port = 9100
	}
	d := net.Dialer{Timeout: 3 * time.Second}
	conn, err := d.Dial("tcp", net.JoinHostPort(host, fmt.Sprintf("%d", port)))
	if err != nil {
		return fmt.Errorf("tcp %s:%d: %w", host, port, err)
	}
	if tc, ok := conn.(*net.TCPConn); ok {
		_ = tc.SetNoDelay(true)
		_ = tc.SetKeepAlive(true)
	}
	c.conn = conn
	c.reader = bufio.NewReader(conn)
	c.host = host
	return nil
}

func (c *rpcClient) close() {
	if c.conn != nil {
		_ = c.conn.Close()
		c.conn = nil
		c.reader = nil
	}
}

func (c *rpcClient) call(method string, params any, timeout time.Duration) (json.RawMessage, error) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if err := c.ensure(); err != nil {
		return nil, err
	}
	c.nextID++
	id := c.nextID
	payload := map[string]any{
		"jsonrpc": "2.0",
		"id":      id,
		"method":  method,
		"params":  params,
	}
	if params == nil {
		payload["params"] = map[string]any{}
	}
	raw, err := json.Marshal(payload)
	if err != nil {
		return nil, err
	}
	raw = append(raw, '\n')
	_ = c.conn.SetDeadline(time.Now().Add(timeout))
	if _, err := c.conn.Write(raw); err != nil {
		c.close()
		return nil, err
	}
	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		_ = c.conn.SetDeadline(deadline)
		line, err := c.reader.ReadBytes('\n')
		if err != nil {
			c.close()
			return nil, err
		}
		var msg struct {
			ID     json.RawMessage `json:"id"`
			Result json.RawMessage `json:"result"`
			Error  *struct {
				Code    int    `json:"code"`
				Message string `json:"message"`
			} `json:"error"`
		}
		if err := json.Unmarshal(line, &msg); err != nil {
			continue
		}
		if len(msg.ID) == 0 || string(msg.ID) == "null" {
			continue
		}
		var got int
		if json.Unmarshal(msg.ID, &got) != nil || got != id {
			continue
		}
		if msg.Error != nil {
			return nil, fmt.Errorf("%d: %s", msg.Error.Code, msg.Error.Message)
		}
		return msg.Result, nil
	}
	return nil, fmt.Errorf("timeout waiting for %s", method)
}
