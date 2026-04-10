package controlplane

import (
	"crypto/hmac"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"strings"
	"sync"
	"time"
)

type SecurityConfig struct {
	Profile               string
	RequireSignedCommands bool
	OperatorID            string
	OperatorRole          string
	OperatorSecret        string
	Operators             map[string]OperatorCredential
	MaxCommandSkew        time.Duration
	MaxCommandTTL         time.Duration
	NonceRetention        time.Duration
}

type OperatorCredential struct {
	OperatorID   string
	OperatorRole string
	OperatorSecret string
}

type CommandAuth struct {
	OperatorID   string `json:"operator_id"`
	OperatorRole string `json:"operator_role,omitempty"`
	IssuedAt     string `json:"issued_at"`
	ExpiresAt    string `json:"expires_at"`
	Nonce        string `json:"nonce"`
	Signature    string `json:"signature"`
}

type commandRequest struct {
	Action      string         `json:"action"`
	Payload     map[string]any `json:"payload"`
	PayloadJSON string         `json:"payload_json"`
	Auth        *CommandAuth   `json:"auth"`
}

type ValidatedCommand struct {
	Action        string
	Payload       map[string]any
	PayloadJSON   string
	IssuedBy      string
	IssuerRole    string
	IssuedAt      time.Time
	ExpiresAt     time.Time
	Nonce         string
	Authenticated bool
}

type CommandSecurityValidator struct {
	cfg        SecurityConfig
	mu         sync.Mutex
	usedNonces map[string]time.Time
}

func NormalizeSecurityProfile(profile string) string {
	switch strings.ToLower(strings.TrimSpace(profile)) {
	case "field":
		return "field"
	case "production", "prod":
		return "production"
	default:
		return "lab"
	}
}

func NormalizeOperatorRole(role string) string {
	switch strings.ToLower(strings.TrimSpace(role)) {
	case "commander", "mission-commander", "mission_commander":
		return "commander"
	case "maintenance", "maintainer":
		return "maintenance"
	default:
		return "operator"
	}
}

func (cfg SecurityConfig) normalized() SecurityConfig {
	cfg.Profile = NormalizeSecurityProfile(cfg.Profile)
	cfg.OperatorRole = NormalizeOperatorRole(cfg.OperatorRole)
	if cfg.Operators == nil {
		cfg.Operators = map[string]OperatorCredential{}
	}
	if id := strings.TrimSpace(cfg.OperatorID); id != "" {
		cfg.Operators[id] = OperatorCredential{
			OperatorID:     id,
			OperatorRole:   NormalizeOperatorRole(cfg.OperatorRole),
			OperatorSecret: strings.TrimSpace(cfg.OperatorSecret),
		}
	}
	normalizedOperators := make(map[string]OperatorCredential, len(cfg.Operators))
	for id, operator := range cfg.Operators {
		normalizedID := strings.TrimSpace(id)
		if normalizedID == "" {
			continue
		}
		if strings.TrimSpace(operator.OperatorID) == "" {
			operator.OperatorID = normalizedID
		}
		operator.OperatorID = strings.TrimSpace(operator.OperatorID)
		operator.OperatorRole = NormalizeOperatorRole(operator.OperatorRole)
		operator.OperatorSecret = strings.TrimSpace(operator.OperatorSecret)
		normalizedOperators[normalizedID] = operator
	}
	cfg.Operators = normalizedOperators
	if cfg.MaxCommandSkew <= 0 {
		cfg.MaxCommandSkew = 30 * time.Second
	}
	if cfg.MaxCommandTTL <= 0 {
		cfg.MaxCommandTTL = 90 * time.Second
	}
	if cfg.NonceRetention <= 0 {
		cfg.NonceRetention = 5 * time.Minute
	}
	return cfg
}

func (cfg SecurityConfig) signedCommandsRequired() bool {
	cfg = cfg.normalized()
	return cfg.RequireSignedCommands || cfg.Profile == "field" || cfg.Profile == "production"
}

func (cfg SecurityConfig) Validate() error {
	cfg = cfg.normalized()
	if cfg.MaxCommandTTL <= 0 {
		return errors.New("max command ttl must be positive")
	}
	if cfg.MaxCommandSkew < 0 {
		return errors.New("max command skew cannot be negative")
	}
	if cfg.NonceRetention <= 0 {
		return errors.New("nonce retention must be positive")
	}
	if NormalizeOperatorRole(cfg.OperatorRole) == "" {
		return errors.New("operator role is invalid")
	}
	if cfg.signedCommandsRequired() {
		if len(cfg.Operators) == 0 {
			return errors.New("at least one operator credential is required when signed commands are enabled")
		}
		for id, operator := range cfg.Operators {
			if strings.TrimSpace(id) == "" {
				return errors.New("operator credential id cannot be empty")
			}
			if strings.TrimSpace(operator.OperatorSecret) == "" {
				return fmt.Errorf("operator secret is required for operator %q", id)
			}
		}
	}
	return nil
}

func NewCommandSecurityValidator(cfg SecurityConfig) *CommandSecurityValidator {
	return &CommandSecurityValidator{
		cfg:        cfg.normalized(),
		usedNonces: make(map[string]time.Time),
	}
}

func (v *CommandSecurityValidator) Validate(req commandRequest, now time.Time) (ValidatedCommand, error) {
	action := strings.TrimSpace(strings.ToLower(req.Action))
	if action == "" {
		return ValidatedCommand{}, errors.New("action is required")
	}
	if now.IsZero() {
		now = time.Now().UTC()
	}

	payloadJSON, payload, err := canonicalizePayload(req.PayloadJSON, req.Payload)
	if err != nil {
		return ValidatedCommand{}, err
	}

	if req.Auth == nil {
		if v.cfg.signedCommandsRequired() {
			return ValidatedCommand{}, errors.New("signed command auth is required")
		}
		return ValidatedCommand{
			Action:        action,
			Payload:       payload,
			PayloadJSON:   payloadJSON,
			IssuedBy:      "dashboard-unsigned",
			IssuerRole:    "operator",
			IssuedAt:      now,
			Authenticated: false,
		}, nil
	}

	auth := *req.Auth
	if strings.TrimSpace(payloadJSON) == "" {
		return ValidatedCommand{}, errors.New("payload_json is required for signed commands")
	}
	if strings.TrimSpace(auth.OperatorID) == "" {
		return ValidatedCommand{}, errors.New("operator_id is required")
	}
	operatorRole := NormalizeOperatorRole(auth.OperatorRole)
	if strings.TrimSpace(auth.Nonce) == "" {
		return ValidatedCommand{}, errors.New("nonce is required")
	}
	if strings.TrimSpace(auth.Signature) == "" {
		return ValidatedCommand{}, errors.New("signature is required")
	}
	operatorCredential, ok := v.cfg.Operators[auth.OperatorID]
	if !ok {
		return ValidatedCommand{}, fmt.Errorf("unknown operator_id %q", auth.OperatorID)
	}
	if operatorRole != operatorCredential.OperatorRole {
		return ValidatedCommand{}, fmt.Errorf("operator role %q is not authorized for operator %q", operatorRole, auth.OperatorID)
	}

	issuedAt, err := time.Parse(time.RFC3339Nano, auth.IssuedAt)
	if err != nil {
		return ValidatedCommand{}, errors.New("issued_at must be RFC3339")
	}
	expiresAt, err := time.Parse(time.RFC3339Nano, auth.ExpiresAt)
	if err != nil {
		return ValidatedCommand{}, errors.New("expires_at must be RFC3339")
	}
	if !expiresAt.After(issuedAt) {
		return ValidatedCommand{}, errors.New("expires_at must be after issued_at")
	}
	if expiresAt.Sub(issuedAt) > v.cfg.MaxCommandTTL {
		return ValidatedCommand{}, errors.New("command ttl exceeds allowed maximum")
	}
	if issuedAt.After(now.Add(v.cfg.MaxCommandSkew)) {
		return ValidatedCommand{}, errors.New("issued_at is too far in the future")
	}
	if expiresAt.Before(now.Add(-v.cfg.MaxCommandSkew)) {
		return ValidatedCommand{}, errors.New("command has expired")
	}

	expectedSignature := SignCommand(operatorCredential.OperatorSecret, action, payloadJSON, auth.OperatorID, operatorRole, auth.IssuedAt, auth.ExpiresAt, auth.Nonce)
	if !hmac.Equal([]byte(strings.ToLower(auth.Signature)), []byte(expectedSignature)) {
		return ValidatedCommand{}, errors.New("command signature mismatch")
	}
	if err := v.claimNonce(auth.OperatorID, auth.Nonce, expiresAt, now); err != nil {
		return ValidatedCommand{}, err
	}

	return ValidatedCommand{
		Action:        action,
		Payload:       payload,
		PayloadJSON:   payloadJSON,
		IssuedBy:      "operator:" + auth.OperatorID,
		IssuerRole:    operatorRole,
		IssuedAt:      issuedAt,
		ExpiresAt:     expiresAt,
		Nonce:         auth.Nonce,
		Authenticated: true,
	}, nil
}

func SignCommand(secret, action, payloadJSON, operatorID, operatorRole, issuedAt, expiresAt, nonce string) string {
	mac := hmac.New(sha256.New, []byte(secret))
	_, _ = mac.Write([]byte(strings.Join([]string{
		strings.TrimSpace(strings.ToLower(action)),
		payloadJSON,
		operatorID,
		NormalizeOperatorRole(operatorRole),
		issuedAt,
		expiresAt,
		nonce,
	}, "\n")))
	return hex.EncodeToString(mac.Sum(nil))
}

func RoleAllowsAction(role, action string) bool {
	role = NormalizeOperatorRole(role)
	action = strings.TrimSpace(strings.ToLower(action))
	switch action {
	case "election":
		return role == "commander"
	case "firmware_update", "maintenance", "maintenance_mode":
		return role == "maintenance"
	default:
		return role == "operator" || role == "commander" || role == "maintenance"
	}
}

func canonicalizePayload(payloadJSON string, payload map[string]any) (string, map[string]any, error) {
	if strings.TrimSpace(payloadJSON) == "" {
		if payload == nil {
			payload = map[string]any{}
		}
		encoded, err := json.Marshal(payload)
		if err != nil {
			return "", nil, errors.New("payload is not serializable")
		}
		payloadJSON = string(encoded)
		return payloadJSON, payload, nil
	}

	var decoded map[string]any
	if err := json.Unmarshal([]byte(payloadJSON), &decoded); err != nil {
		return "", nil, errors.New("payload_json is not valid JSON")
	}
	if decoded == nil {
		decoded = map[string]any{}
	}
	encoded, err := json.Marshal(decoded)
	if err != nil {
		return "", nil, errors.New("payload_json normalization failed")
	}
	return string(encoded), decoded, nil
}

func (v *CommandSecurityValidator) claimNonce(operatorID, nonce string, expiresAt, now time.Time) error {
	key := operatorID + ":" + nonce
	retainUntil := expiresAt
	if retainUntil.Before(now) {
		retainUntil = now
	}
	retainUntil = retainUntil.Add(v.cfg.NonceRetention)

	v.mu.Lock()
	defer v.mu.Unlock()

	for existing, expiry := range v.usedNonces {
		if expiry.Before(now) {
			delete(v.usedNonces, existing)
		}
	}
	if _, exists := v.usedNonces[key]; exists {
		return errors.New("nonce replay rejected")
	}
	v.usedNonces[key] = retainUntil
	return nil
}
