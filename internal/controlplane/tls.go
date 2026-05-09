package controlplane

import (
	"crypto/sha256"
	"crypto/tls"
	"crypto/x509"
	"encoding/hex"
	"errors"
	"net/http"
	"os"
	"strings"
	"time"
)

type TLSConfig struct {
	Enabled           bool
	CertFile          string
	KeyFile           string
	ClientCAFile      string
	RequireClientCert bool
}

type PeerIdentity struct {
	CommonName        string
	Identities        []string
	FingerprintSHA256 string
	Verified          bool
	NotBefore         time.Time
	NotAfter          time.Time
}

func (cfg TLSConfig) Validate() error {
	if !cfg.Enabled {
		return nil
	}
	if strings.TrimSpace(cfg.CertFile) == "" {
		return errors.New("DRONE_TLS_CERT_FILE is required when TLS is enabled")
	}
	if strings.TrimSpace(cfg.KeyFile) == "" {
		return errors.New("DRONE_TLS_KEY_FILE is required when TLS is enabled")
	}
	if cfg.RequireClientCert && strings.TrimSpace(cfg.ClientCAFile) == "" {
		return errors.New("DRONE_TLS_CA_FILE is required when client certificates are enforced")
	}
	return nil
}

func (cfg TLSConfig) ServerTLSConfig() (*tls.Config, error) {
	if !cfg.Enabled {
		return nil, nil
	}

	tlsCfg := &tls.Config{
		MinVersion: tls.VersionTLS13,
	}

	if strings.TrimSpace(cfg.ClientCAFile) != "" {
		clientCABytes, err := os.ReadFile(cfg.ClientCAFile)
		if err != nil {
			return nil, err
		}
		clientCAPool := x509.NewCertPool()
		if !clientCAPool.AppendCertsFromPEM(clientCABytes) {
			return nil, errors.New("failed to parse DRONE_TLS_CA_FILE")
		}
		tlsCfg.ClientCAs = clientCAPool
		if cfg.RequireClientCert {
			tlsCfg.ClientAuth = tls.RequireAndVerifyClientCert
		} else {
			tlsCfg.ClientAuth = tls.VerifyClientCertIfGiven
		}
	}

	return tlsCfg, nil
}

func RequestPeerIdentity(r *http.Request) PeerIdentity {
	if r == nil || r.TLS == nil || len(r.TLS.PeerCertificates) == 0 {
		return PeerIdentity{}
	}

	cert := r.TLS.PeerCertificates[0]
	names := []string{}
	appendName := func(value string) {
		value = strings.TrimSpace(value)
		if value == "" {
			return
		}
		for _, existing := range names {
			if existing == value {
				return
			}
		}
		names = append(names, value)
	}

	appendName(cert.Subject.CommonName)
	for _, value := range cert.DNSNames {
		appendName(value)
	}
	for _, value := range cert.EmailAddresses {
		appendName(value)
	}
	for _, value := range cert.URIs {
		appendName(value.String())
	}

	fingerprint := sha256.Sum256(cert.Raw)
	return PeerIdentity{
		CommonName:        strings.TrimSpace(cert.Subject.CommonName),
		Identities:        names,
		FingerprintSHA256: hex.EncodeToString(fingerprint[:]),
		Verified:          len(r.TLS.VerifiedChains) > 0,
		NotBefore:         cert.NotBefore,
		NotAfter:          cert.NotAfter,
	}
}

func (p PeerIdentity) Matches(identity string) bool {
	identity = strings.TrimSpace(identity)
	if identity == "" {
		return false
	}
	for _, candidate := range p.Identities {
		if candidate == identity {
			return true
		}
	}
	return false
}
