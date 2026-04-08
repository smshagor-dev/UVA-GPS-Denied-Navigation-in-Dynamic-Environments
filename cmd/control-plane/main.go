// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

package main

import (
	"context"
	"io"
	"log"
	"os"
	"os/signal"
	"path/filepath"
	"syscall"
	"time"

	"drone_swarm/controlplane/internal/controlplane"
)

func main() {
	if err := os.MkdirAll(filepath.Join("logs", "control-plane"), 0o755); err == nil {
		if file, ferr := os.OpenFile(filepath.Join("logs", "control-plane", "control-plane.log"), os.O_CREATE|os.O_WRONLY|os.O_APPEND, 0o644); ferr == nil {
			log.SetOutput(io.MultiWriter(os.Stdout, file))
		}
	}
	log.Printf("control-plane logging initialized")
	addr := ":8080"
	if env := os.Getenv("DRONE_SWARM_ADDR"); env != "" {
		addr = env
	}

	server := controlplane.NewServer(addr)
	errCh := make(chan error, 1)
	go func() {
		errCh <- server.Start()
	}()

	sigCh := make(chan os.Signal, 1)
	signal.Notify(sigCh, syscall.SIGINT, syscall.SIGTERM)

	select {
	case sig := <-sigCh:
		log.Printf("shutting down control-plane after %s", sig)
	case err := <-errCh:
		if err != nil {
			log.Printf("control-plane failed: %v", err)
			os.Exit(1)
		}
		log.Printf("control-plane exited cleanly")
		return
	}

	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	if err := server.Shutdown(ctx); err != nil {
		log.Printf("graceful shutdown failed: %v", err)
	}
}
