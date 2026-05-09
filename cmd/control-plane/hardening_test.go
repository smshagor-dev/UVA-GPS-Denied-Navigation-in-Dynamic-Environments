package main

import (
	"testing"

	"drone_swarm/controlplane/internal/controlplane"
)

func TestValidateHardenedTransportAllowsLabWithoutTLS(t *testing.T) {
	if err := validateHardenedTransport(controlplane.SecurityConfig{Profile: "lab"}, controlplane.TLSConfig{}); err != nil {
		t.Fatalf("expected lab mode to allow plaintext startup, got %v", err)
	}
}

func TestValidateHardenedTransportRequiresTLSOutsideLab(t *testing.T) {
	err := validateHardenedTransport(controlplane.SecurityConfig{Profile: "production"}, controlplane.TLSConfig{})
	if err == nil {
		t.Fatal("expected hardened transport validation to fail without TLS")
	}
}

func TestValidateHardenedTransportRequiresClientCertOutsideLab(t *testing.T) {
	err := validateHardenedTransport(
		controlplane.SecurityConfig{Profile: "field"},
		controlplane.TLSConfig{Enabled: true},
	)
	if err == nil {
		t.Fatal("expected hardened transport validation to fail without client cert enforcement")
	}
}
