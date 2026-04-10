#!/usr/bin/env python3

from __future__ import annotations

import argparse
import ipaddress
from datetime import UTC, datetime, timedelta
from pathlib import Path

from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.x509.oid import ExtendedKeyUsageOID, NameOID


ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CERT_DIR = ROOT / "certs"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate local CA/server/client certificates for drone_swarm")
    parser.add_argument("--output-dir", default=str(DEFAULT_CERT_DIR), help="directory to write generated certs")
    parser.add_argument("--server-name", default="drone-control-plane", help="server certificate common name")
    parser.add_argument("--operator-id", default="operator-console-1", help="operator client certificate common name")
    parser.add_argument("--drone-id", default="drone-node-1", help="drone client certificate common name")
    parser.add_argument("--days", type=int, default=365, help="certificate lifetime in days")
    parser.add_argument(
        "--server-san",
        action="append",
        default=["127.0.0.1", "localhost"],
        help="server subjectAltName entry; can be repeated",
    )
    parser.add_argument("--force", action="store_true", help="overwrite existing certificate material")
    return parser.parse_args()


def ensure_writable(path: Path, force: bool) -> None:
    if path.exists() and not force:
        raise FileExistsError(f"{path} already exists; rerun with --force to overwrite")


def write_private_key(path: Path, key: rsa.RSAPrivateKey, force: bool) -> None:
    ensure_writable(path, force)
    path.write_bytes(
        key.private_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PrivateFormat.TraditionalOpenSSL,
            encryption_algorithm=serialization.NoEncryption(),
        )
    )


def write_certificate(path: Path, cert: x509.Certificate, force: bool) -> None:
    ensure_writable(path, force)
    path.write_bytes(cert.public_bytes(serialization.Encoding.PEM))


def build_name(common_name: str, organization: str) -> x509.Name:
    return x509.Name(
        [
            x509.NameAttribute(NameOID.COUNTRY_NAME, "BD"),
            x509.NameAttribute(NameOID.ORGANIZATION_NAME, organization),
            x509.NameAttribute(NameOID.COMMON_NAME, common_name),
        ]
    )


def new_private_key() -> rsa.RSAPrivateKey:
    return rsa.generate_private_key(public_exponent=65537, key_size=3072)


def sign_certificate(
    *,
    subject: x509.Name,
    issuer: x509.Name,
    public_key,
    issuer_key,
    serial_number: int,
    not_before: datetime,
    not_after: datetime,
    is_ca: bool,
    san_entries: list[str] | None = None,
    client_auth: bool = False,
    server_auth: bool = False,
) -> x509.Certificate:
    builder = (
        x509.CertificateBuilder()
        .subject_name(subject)
        .issuer_name(issuer)
        .public_key(public_key)
        .serial_number(serial_number)
        .not_valid_before(not_before)
        .not_valid_after(not_after)
        .add_extension(x509.BasicConstraints(ca=is_ca, path_length=None), critical=True)
        .add_extension(
            x509.KeyUsage(
                digital_signature=True,
                key_encipherment=True,
                key_cert_sign=is_ca,
                crl_sign=is_ca,
                key_agreement=not is_ca,
                data_encipherment=False,
                content_commitment=False,
                encipher_only=False,
                decipher_only=False,
            ),
            critical=True,
        )
        .add_extension(x509.SubjectKeyIdentifier.from_public_key(public_key), critical=False)
    )

    if not is_ca:
        builder = builder.add_extension(
            x509.AuthorityKeyIdentifier.from_issuer_public_key(issuer_key.public_key()),
            critical=False,
        )
        eku = []
        if server_auth:
            eku.append(ExtendedKeyUsageOID.SERVER_AUTH)
        if client_auth:
            eku.append(ExtendedKeyUsageOID.CLIENT_AUTH)
        if eku:
            builder = builder.add_extension(x509.ExtendedKeyUsage(eku), critical=False)

    if san_entries:
        general_names: list[x509.GeneralName] = []
        for entry in san_entries:
            try:
                general_names.append(x509.IPAddress(ipaddress.ip_address(entry)))
            except ValueError:
                general_names.append(x509.DNSName(entry))
        builder = builder.add_extension(x509.SubjectAlternativeName(general_names), critical=False)

    return builder.sign(private_key=issuer_key, algorithm=hashes.SHA256())


def main() -> int:
    args = parse_args()
    out_dir = Path(args.output_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    now = datetime.now(UTC) - timedelta(minutes=5)
    not_after = now + timedelta(days=max(args.days, 1))

    ca_key = new_private_key()
    ca_subject = build_name("drone-swarm-local-ca", "Drone Swarm Local PKI")
    ca_cert = sign_certificate(
        subject=ca_subject,
        issuer=ca_subject,
        public_key=ca_key.public_key(),
        issuer_key=ca_key,
        serial_number=x509.random_serial_number(),
        not_before=now,
        not_after=not_after,
        is_ca=True,
    )

    server_key = new_private_key()
    server_subject = build_name(args.server_name, "Drone Swarm Control Plane")
    server_cert = sign_certificate(
        subject=server_subject,
        issuer=ca_subject,
        public_key=server_key.public_key(),
        issuer_key=ca_key,
        serial_number=x509.random_serial_number(),
        not_before=now,
        not_after=not_after,
        is_ca=False,
        san_entries=list(dict.fromkeys(args.server_san)),
        server_auth=True,
    )

    operator_key = new_private_key()
    operator_subject = build_name(args.operator_id, "Drone Swarm Operator")
    operator_cert = sign_certificate(
        subject=operator_subject,
        issuer=ca_subject,
        public_key=operator_key.public_key(),
        issuer_key=ca_key,
        serial_number=x509.random_serial_number(),
        not_before=now,
        not_after=not_after,
        is_ca=False,
        san_entries=[args.operator_id],
        client_auth=True,
    )

    drone_key = new_private_key()
    drone_subject = build_name(args.drone_id, "Drone Swarm Drone")
    drone_cert = sign_certificate(
        subject=drone_subject,
        issuer=ca_subject,
        public_key=drone_key.public_key(),
        issuer_key=ca_key,
        serial_number=x509.random_serial_number(),
        not_before=now,
        not_after=not_after,
        is_ca=False,
        san_entries=[args.drone_id],
        client_auth=True,
    )

    write_private_key(out_dir / "ca.key", ca_key, args.force)
    write_certificate(out_dir / "ca.crt", ca_cert, args.force)
    write_private_key(out_dir / "server.key", server_key, args.force)
    write_certificate(out_dir / "server.crt", server_cert, args.force)
    write_private_key(out_dir / "operator-client.key", operator_key, args.force)
    write_certificate(out_dir / "operator-client.crt", operator_cert, args.force)
    write_private_key(out_dir / "drone-client.key", drone_key, args.force)
    write_certificate(out_dir / "drone-client.crt", drone_cert, args.force)

    print(f"certificate bundle written to {out_dir}")
    print("generated files:")
    for name in [
        "ca.crt",
        "ca.key",
        "server.crt",
        "server.key",
        "operator-client.crt",
        "operator-client.key",
        "drone-client.crt",
        "drone-client.key",
    ]:
        print(f"  - {out_dir / name}")
    print("next step:")
    print("  set DRONE_TLS_* env vars to these files and use https://127.0.0.1:8080")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
