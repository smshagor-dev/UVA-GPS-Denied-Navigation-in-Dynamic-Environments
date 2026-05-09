// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

package main

import (
	"bufio"
	"context"
	"encoding/json"
	"errors"
	"io"
	"log"
	"os"
	"os/signal"
	"path/filepath"
	"strconv"
	"strings"
	"syscall"
	"time"

	"drone_swarm/controlplane/internal/controlplane"
)

func loadEnvFile(path string) {
	file, err := os.Open(path)
	if err != nil {
		return
	}
	defer file.Close()

	scanner := bufio.NewScanner(file)
	for scanner.Scan() {
		line := strings.TrimSpace(scanner.Text())
		if line == "" || strings.HasPrefix(line, "#") || !strings.Contains(line, "=") {
			continue
		}
		parts := strings.SplitN(line, "=", 2)
		key := strings.TrimSpace(parts[0])
		value := strings.Trim(strings.TrimSpace(parts[1]), `"'`)
		if key == "" {
			continue
		}
		if _, exists := os.LookupEnv(key); !exists {
			_ = os.Setenv(key, value)
		}
	}
}

func main() {
	loadEnvFile(".env")
	loadEnvFile(".env.local")
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

	securityCfg := controlplane.SecurityConfig{
		Profile:               os.Getenv("DRONE_SECURITY_PROFILE"),
		RequireSignedCommands: parseBoolEnv("DRONE_REQUIRE_SIGNED_COMMANDS", false),
		OperatorID:            os.Getenv("DRONE_OPERATOR_ID"),
		OperatorRole:          os.Getenv("DRONE_OPERATOR_ROLE"),
		OperatorSecret:        os.Getenv("DRONE_OPERATOR_SECRET"),
		Operators:             parseOperatorCredentialsEnv(os.Getenv("DRONE_OPERATOR_CREDENTIALS")),
		Devices:               parseDeviceRegistry(os.Getenv("DRONE_DEVICE_REGISTRY"), os.Getenv("DRONE_DEVICE_REGISTRY_FILE")),
		RevokedFingerprints:   parseStringSet(os.Getenv("DRONE_REVOKED_CERT_FINGERPRINTS")),
		RevokedIdentities:     parseStringSet(os.Getenv("DRONE_REVOKED_IDENTITIES")),
		MaxCommandSkew:        time.Duration(parseIntEnv("DRONE_MAX_COMMAND_SKEW_SEC", 30)) * time.Second,
		MaxCommandTTL:         time.Duration(parseIntEnv("DRONE_COMMAND_TTL_SEC", 90)) * time.Second,
		NonceRetention:        time.Duration(parseIntEnv("DRONE_COMMAND_NONCE_RETENTION_SEC", 300)) * time.Second,
		MinCertValidity:       time.Duration(parseIntEnv("DRONE_CERT_MIN_VALIDITY_HOURS", 24)) * time.Hour,
	}
	if err := securityCfg.Validate(); err != nil {
		log.Printf("invalid security configuration: %v", err)
		os.Exit(1)
	}
	tlsCfg := controlplane.TLSConfig{
		Enabled:           parseBoolEnv("DRONE_TLS_ENABLED", false),
		CertFile:          os.Getenv("DRONE_TLS_CERT_FILE"),
		KeyFile:           os.Getenv("DRONE_TLS_KEY_FILE"),
		ClientCAFile:      os.Getenv("DRONE_TLS_CA_FILE"),
		RequireClientCert: parseBoolEnv("DRONE_TLS_REQUIRE_CLIENT_CERT", false),
	}
	if err := tlsCfg.Validate(); err != nil {
		log.Printf("invalid tls configuration: %v", err)
		os.Exit(1)
	}
	if err := validateHardenedTransport(securityCfg, tlsCfg); err != nil {
		log.Printf("invalid hardened transport configuration: %v", err)
		os.Exit(1)
	}
	log.Printf("control-plane security profile=%s signed_commands_required=%t operator_id=%s operator_role=%s",
		controlplane.NormalizeSecurityProfile(securityCfg.Profile),
		securityCfg.RequireSignedCommands || controlplane.NormalizeSecurityProfile(securityCfg.Profile) != "lab",
		strings.TrimSpace(securityCfg.OperatorID),
		controlplane.NormalizeOperatorRole(securityCfg.OperatorRole))
	log.Printf("control-plane transport tls_enabled=%t client_cert_required=%t cert=%s ca=%s",
		tlsCfg.Enabled,
		tlsCfg.RequireClientCert,
		strings.TrimSpace(tlsCfg.CertFile),
		strings.TrimSpace(tlsCfg.ClientCAFile))

	server := controlplane.NewServer(addr, securityCfg, tlsCfg)
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

func parseOperatorCredentialsEnv(value string) map[string]controlplane.OperatorCredential {
	operators := map[string]controlplane.OperatorCredential{}
	for _, item := range strings.Split(value, ";") {
		entry := strings.TrimSpace(item)
		if entry == "" {
			continue
		}
		parts := strings.SplitN(entry, ":", 3)
		if len(parts) != 3 {
			continue
		}
		id := strings.TrimSpace(parts[0])
		role := strings.TrimSpace(parts[1])
		secret := strings.TrimSpace(parts[2])
		if id == "" || secret == "" {
			continue
		}
		operators[id] = controlplane.OperatorCredential{
			OperatorID:     id,
			OperatorRole:   role,
			OperatorSecret: secret,
		}
	}
	return operators
}

func parseDeviceRegistry(value, filePath string) map[string]controlplane.DeviceRecord {
	devices := map[string]controlplane.DeviceRecord{}
	if strings.TrimSpace(filePath) != "" {
		loaded, err := os.ReadFile(filePath)
		if err == nil {
			var records []controlplane.DeviceRecord
			if jsonErr := json.Unmarshal(loaded, &records); jsonErr == nil {
				for _, record := range records {
					identity := strings.TrimSpace(record.Identity)
					if identity == "" {
						continue
					}
					devices[identity] = record
				}
			}
		}
	}
	for _, item := range strings.Split(value, ";") {
		entry := strings.TrimSpace(item)
		if entry == "" {
			continue
		}
		parts := strings.SplitN(entry, ":", 4)
		if len(parts) < 2 {
			continue
		}
		identity := strings.TrimSpace(parts[0])
		deviceType := strings.TrimSpace(parts[1])
		scope := []string{}
		status := "active"
		if len(parts) >= 3 && strings.TrimSpace(parts[2]) != "" {
			scope = strings.Split(parts[2], ",")
		}
		if len(parts) >= 4 && strings.TrimSpace(parts[3]) != "" {
			status = strings.TrimSpace(parts[3])
		}
		if identity == "" || deviceType == "" {
			continue
		}
		devices[identity] = controlplane.DeviceRecord{
			Identity:     identity,
			DeviceType:   deviceType,
			ClusterScope: scope,
			Status:       status,
		}
	}
	return devices
}

func parseStringSet(value string) map[string]struct{} {
	out := map[string]struct{}{}
	for _, item := range strings.FieldsFunc(value, func(r rune) bool {
		return r == ';' || r == ',' || r == '\n'
	}) {
		item = strings.ToLower(strings.TrimSpace(item))
		if item == "" {
			continue
		}
		out[item] = struct{}{}
	}
	return out
}

func parseBoolEnv(key string, fallback bool) bool {
	value := strings.TrimSpace(strings.ToLower(os.Getenv(key)))
	switch value {
	case "1", "true", "yes", "on":
		return true
	case "0", "false", "no", "off":
		return false
	default:
		return fallback
	}
}

func parseIntEnv(key string, fallback int) int {
	value := strings.TrimSpace(os.Getenv(key))
	if value == "" {
		return fallback
	}
	parsed, err := strconv.Atoi(value)
	if err != nil {
		return fallback
	}
	return parsed
}

func validateHardenedTransport(securityCfg controlplane.SecurityConfig, tlsCfg controlplane.TLSConfig) error {
	profile := controlplane.NormalizeSecurityProfile(securityCfg.Profile)
	if profile == "lab" {
		return nil
	}
	if !tlsCfg.Enabled {
		return errors.New("tls must be enabled outside lab mode")
	}
	if !tlsCfg.RequireClientCert {
		return errors.New("mutual tls with client certificates is required outside lab mode")
	}
	return nil
}
