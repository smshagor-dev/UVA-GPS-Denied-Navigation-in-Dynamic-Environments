package controlplane

import (
	"fmt"
	"sync"
	"testing"
	"time"
)

func testSecurityConfig() SecurityConfig {
	return SecurityConfig{
		Profile:               "production",
		RequireSignedCommands: true,
		OperatorID:            "operator-console-1",
		OperatorRole:          "operator",
		OperatorSecret:        "super-secret-test-key",
		Operators: map[string]OperatorCredential{
			"operator-console-1": {
				OperatorID:     "operator-console-1",
				OperatorRole:   "operator",
				OperatorSecret: "super-secret-test-key",
			},
			"operator-console-2": {
				OperatorID:     "operator-console-2",
				OperatorRole:   "commander",
				OperatorSecret: "super-secret-commander-key",
			},
		},
		MaxCommandSkew: 30 * time.Second,
		MaxCommandTTL:  90 * time.Second,
		NonceRetention: 5 * time.Minute,
	}
}

func TestCommandSecurityValidatorAcceptsSignedCommand(t *testing.T) {
	cfg := testSecurityConfig()
	validator := NewCommandSecurityValidator(cfg)
	now := time.Now().UTC()
	issuedAt := now.Format(time.RFC3339Nano)
	expiresAt := now.Add(45 * time.Second).Format(time.RFC3339Nano)
	payloadJSON := `{"cluster_id":"cluster-01","target_ids":[1,2]}`
	nonce := "nonce-001"

	req := commandRequest{
		Action:      "formation",
		PayloadJSON: payloadJSON,
		Auth: &CommandAuth{
			OperatorID:   cfg.OperatorID,
			OperatorRole: cfg.OperatorRole,
			IssuedAt:     issuedAt,
			ExpiresAt:    expiresAt,
			Nonce:        nonce,
			Signature:    SignCommand(cfg.OperatorSecret, "formation", payloadJSON, cfg.OperatorID, cfg.OperatorRole, issuedAt, expiresAt, nonce),
		},
	}

	validated, err := validator.Validate(req, now)
	if err != nil {
		t.Fatalf("expected signed command to validate, got error: %v", err)
	}
	if !validated.Authenticated {
		t.Fatal("expected validated command to be authenticated")
	}
	if validated.IssuedBy != "operator:"+cfg.OperatorID {
		t.Fatalf("unexpected issuer %q", validated.IssuedBy)
	}
	if validated.IssuerRole != cfg.OperatorRole {
		t.Fatalf("unexpected issuer role %q", validated.IssuerRole)
	}
}

func TestCommandSecurityValidatorRejectsReplayNonce(t *testing.T) {
	cfg := testSecurityConfig()
	validator := NewCommandSecurityValidator(cfg)
	now := time.Now().UTC()
	issuedAt := now.Format(time.RFC3339Nano)
	expiresAt := now.Add(30 * time.Second).Format(time.RFC3339Nano)
	payloadJSON := `{"cluster_id":"cluster-01"}`
	nonce := "replay-001"
	signature := SignCommand(cfg.OperatorSecret, "hold", payloadJSON, cfg.OperatorID, cfg.OperatorRole, issuedAt, expiresAt, nonce)

	req := commandRequest{
		Action:      "hold",
		PayloadJSON: payloadJSON,
		Auth: &CommandAuth{
			OperatorID:   cfg.OperatorID,
			OperatorRole: cfg.OperatorRole,
			IssuedAt:     issuedAt,
			ExpiresAt:    expiresAt,
			Nonce:        nonce,
			Signature:    signature,
		},
	}

	if _, err := validator.Validate(req, now); err != nil {
		t.Fatalf("first validation should succeed, got %v", err)
	}
	if _, err := validator.Validate(req, now.Add(1*time.Second)); err == nil {
		t.Fatal("expected nonce replay rejection")
	}
}

func TestCommandSecurityValidatorRejectsUnsignedWhenRequired(t *testing.T) {
	cfg := testSecurityConfig()
	validator := NewCommandSecurityValidator(cfg)

	_, err := validator.Validate(commandRequest{
		Action:  "land",
		Payload: map[string]any{"target_ids": []int{1}},
	}, time.Now().UTC())
	if err == nil {
		t.Fatal("expected unsigned command rejection")
	}
}

func TestCommandSecurityValidatorRejectsUnexpectedOperatorRole(t *testing.T) {
	cfg := testSecurityConfig()
	validator := NewCommandSecurityValidator(cfg)
	now := time.Now().UTC()
	issuedAt := now.Format(time.RFC3339Nano)
	expiresAt := now.Add(30 * time.Second).Format(time.RFC3339Nano)
	payloadJSON := `{"cluster_id":"cluster-01"}`
	role := "commander"
	nonce := "role-001"

	_, err := validator.Validate(commandRequest{
		Action:      "election",
		PayloadJSON: payloadJSON,
		Auth: &CommandAuth{
			OperatorID:   cfg.OperatorID,
			OperatorRole: role,
			IssuedAt:     issuedAt,
			ExpiresAt:    expiresAt,
			Nonce:        nonce,
			Signature:    SignCommand(cfg.OperatorSecret, "election", payloadJSON, cfg.OperatorID, role, issuedAt, expiresAt, nonce),
		},
	}, now)
	if err == nil {
		t.Fatal("expected operator role rejection")
	}
}

func TestCommandSecurityValidatorAcceptsRegisteredSecondaryOperator(t *testing.T) {
	cfg := testSecurityConfig()
	validator := NewCommandSecurityValidator(cfg)
	now := time.Now().UTC()
	issuedAt := now.Format(time.RFC3339Nano)
	expiresAt := now.Add(30 * time.Second).Format(time.RFC3339Nano)
	payloadJSON := `{"cluster_id":"cluster-01"}`
	operatorID := "operator-console-2"
	operatorRole := "commander"
	nonce := "secondary-001"

	validated, err := validator.Validate(commandRequest{
		Action:      "election",
		PayloadJSON: payloadJSON,
		Auth: &CommandAuth{
			OperatorID:   operatorID,
			OperatorRole: operatorRole,
			IssuedAt:     issuedAt,
			ExpiresAt:    expiresAt,
			Nonce:        nonce,
			Signature:    SignCommand(cfg.Operators[operatorID].OperatorSecret, "election", payloadJSON, operatorID, operatorRole, issuedAt, expiresAt, nonce),
		},
	}, now)
	if err != nil {
		t.Fatalf("expected secondary operator to validate, got %v", err)
	}
	if validated.IssuedBy != "operator:"+operatorID {
		t.Fatalf("unexpected issuer %q", validated.IssuedBy)
	}
}

func TestRoleAllowsActionEnforcesCommanderOnlyElection(t *testing.T) {
	if RoleAllowsAction("operator", "election") {
		t.Fatal("expected operator role to be denied election")
	}
	if !RoleAllowsAction("commander", "election") {
		t.Fatal("expected commander role to be allowed election")
	}
	if !RoleAllowsAction("maintenance", "add_drone") {
		t.Fatal("expected maintenance role to retain standard add_drone access")
	}
	if !RoleAllowsAction("maintenance", "firmware_update") {
		t.Fatal("expected maintenance role to be allowed firmware updates")
	}
}

func TestSecurityConfigNormalizedIsSafeUnderConcurrency(t *testing.T) {
	cfg := testSecurityConfig()
	cfg.Devices = map[string]DeviceRecord{
		"drone-node-1": {
			Identity:   "drone-node-1",
			DeviceType: "drone",
			Status:     "active",
		},
	}
	const goroutines = 32
	const iterations = 200
	var wg sync.WaitGroup
	errCh := make(chan string, goroutines*iterations)
	wg.Add(goroutines)
	for i := 0; i < goroutines; i++ {
		go func() {
			defer wg.Done()
			for j := 0; j < iterations; j++ {
				normalized := cfg.normalized()
				if normalized.Profile != "production" {
					errCh <- fmt.Sprintf("expected production profile, got %q", normalized.Profile)
					return
				}
				if _, ok := normalized.Devices["operator-console-1"]; !ok {
					errCh <- "expected normalized device registry to include operator-derived identity"
					return
				}
				if _, ok := normalized.Operators["operator-console-1"]; !ok {
					errCh <- "expected normalized operators to include primary operator"
					return
				}
			}
		}()
	}
	wg.Wait()
	close(errCh)
	for err := range errCh {
		t.Fatal(err)
	}
}
