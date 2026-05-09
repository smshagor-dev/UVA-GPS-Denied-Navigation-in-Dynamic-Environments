package controlplane

import (
	"crypto/rand"
	"crypto/rsa"
	"crypto/tls"
	"crypto/x509"
	"crypto/x509/pkix"
	"math/big"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"
)

func TestTelemetryRejectedWithoutVerifiedPeerInHardenedProfile(t *testing.T) {
	server := NewServer(":0", SecurityConfig{Profile: "production"}, TLSConfig{
		Enabled:           true,
		RequireClientCert: true,
	}, ServerConfig{})

	req := httptest.NewRequest(http.MethodPost, "/api/v1/telemetry", strings.NewReader(`{"drone_id":1}`))
	rec := httptest.NewRecorder()

	server.handleTelemetry(rec, req)

	if rec.Code != http.StatusUnauthorized {
		t.Fatalf("expected unauthorized telemetry rejection, got %d", rec.Code)
	}
}

func TestMissionPostForbiddenInHardenedProfile(t *testing.T) {
	server := NewServer(":0", SecurityConfig{Profile: "production"}, TLSConfig{
		Enabled:           true,
		RequireClientCert: true,
	}, ServerConfig{})

	req := httptest.NewRequest(http.MethodPost, "/api/v1/missions", strings.NewReader(`{"mission_id":"m1","name":"test"}`))
	rec := httptest.NewRecorder()

	server.handleMissions(rec, req)

	if rec.Code != http.StatusForbidden {
		t.Fatalf("expected mission POST to be forbidden, got %d", rec.Code)
	}
}

func TestApprovalsRequireVerifiedPeerOutsideLab(t *testing.T) {
	server := NewServer(":0", SecurityConfig{Profile: "field"}, TLSConfig{
		Enabled:           true,
		RequireClientCert: true,
	}, ServerConfig{})

	req := httptest.NewRequest(http.MethodGet, "/api/v1/approvals", nil)
	rec := httptest.NewRecorder()

	server.handleApprovals(rec, req)

	if rec.Code != http.StatusUnauthorized {
		t.Fatalf("expected unauthorized approvals access, got %d", rec.Code)
	}
}

func TestTelemetryRejectedWhenPeerNotInDeviceRegistry(t *testing.T) {
	server := NewServer(":0", SecurityConfig{
		Profile: "production",
		Devices: map[string]DeviceRecord{
			"drone-node-registered": {Identity: "drone-node-registered", DeviceType: "drone", Status: "active"},
		},
	}, TLSConfig{
		Enabled:           true,
		RequireClientCert: true,
	}, ServerConfig{})

	req := newVerifiedPeerRequest(t, http.MethodPost, "/api/v1/telemetry", `{"drone_id":1,"cluster_id":"cluster-01"}`, "drone-node-unknown", time.Now().Add(72*time.Hour))
	rec := httptest.NewRecorder()

	server.handleTelemetry(rec, req)

	if rec.Code != http.StatusUnauthorized {
		t.Fatalf("expected unknown device to be rejected, got %d", rec.Code)
	}
}

func TestTelemetryRejectedWhenClientCertRotationIsDue(t *testing.T) {
	server := NewServer(":0", SecurityConfig{
		Profile:         "production",
		MinCertValidity: 24 * time.Hour,
		Devices: map[string]DeviceRecord{
			"drone-node-1": {Identity: "drone-node-1", DeviceType: "drone", Status: "active"},
		},
	}, TLSConfig{
		Enabled:           true,
		RequireClientCert: true,
	}, ServerConfig{})

	req := newVerifiedPeerRequest(t, http.MethodPost, "/api/v1/telemetry", `{"drone_id":1,"cluster_id":"cluster-01"}`, "drone-node-1", time.Now().Add(2*time.Hour))
	rec := httptest.NewRecorder()

	server.handleTelemetry(rec, req)

	if rec.Code != http.StatusUnauthorized {
		t.Fatalf("expected expiring certificate to be rejected, got %d", rec.Code)
	}
}

func TestApprovalsAllowRegisteredCommanderPeer(t *testing.T) {
	server := NewServer(":0", SecurityConfig{
		Profile: "field",
		Devices: map[string]DeviceRecord{
			"operator-console-1": {Identity: "operator-console-1", DeviceType: "commander", Status: "active"},
		},
	}, TLSConfig{
		Enabled:           true,
		RequireClientCert: true,
	}, ServerConfig{})

	req := newVerifiedPeerRequest(t, http.MethodGet, "/api/v1/approvals", "", "operator-console-1", time.Now().Add(72*time.Hour))
	rec := httptest.NewRecorder()

	server.handleApprovals(rec, req)

	if rec.Code != http.StatusOK {
		t.Fatalf("expected registered commander to access approvals, got %d", rec.Code)
	}
}

func newVerifiedPeerRequest(t *testing.T, method, target, body, commonName string, notAfter time.Time) *http.Request {
	t.Helper()
	req := httptest.NewRequest(method, target, strings.NewReader(body))
	cert := mustIssueTestCertificate(t, commonName, notAfter)
	req.TLS = &tls.ConnectionState{
		PeerCertificates: []*x509.Certificate{cert},
		VerifiedChains:   [][]*x509.Certificate{{cert}},
	}
	return req
}

func mustIssueTestCertificate(t *testing.T, commonName string, notAfter time.Time) *x509.Certificate {
	t.Helper()
	key, err := rsa.GenerateKey(rand.Reader, 2048)
	if err != nil {
		t.Fatalf("generate key: %v", err)
	}
	now := time.Now().UTC()
	template := &x509.Certificate{
		SerialNumber: big.NewInt(now.UnixNano()),
		Subject: pkix.Name{
			CommonName: commonName,
		},
		NotBefore:             now.Add(-1 * time.Hour),
		NotAfter:              notAfter.UTC(),
		KeyUsage:              x509.KeyUsageDigitalSignature | x509.KeyUsageKeyEncipherment,
		ExtKeyUsage:           []x509.ExtKeyUsage{x509.ExtKeyUsageClientAuth},
		BasicConstraintsValid: true,
		DNSNames:              []string{commonName},
	}
	der, err := x509.CreateCertificate(rand.Reader, template, template, &key.PublicKey, key)
	if err != nil {
		t.Fatalf("create certificate: %v", err)
	}
	cert, err := x509.ParseCertificate(der)
	if err != nil {
		t.Fatalf("parse certificate: %v", err)
	}
	return cert
}
