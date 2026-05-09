import argparse
from pathlib import Path
import hashlib


def sign_manifest(secret: str, version: str, measurement: str, signer: str,
                  secure_boot_attested: bool, bootloader_locked: bool,
                  rollback_counter: int) -> str:
    payload = "\n".join([
        secret.strip(),
        version.strip(),
        measurement.strip(),
        signer.strip(),
        "1" if secure_boot_attested else "0",
        "1" if bootloader_locked else "0",
        str(int(rollback_counter)),
    ]).encode("utf-8")
    return hashlib.sha3_256(payload).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate a signed firmware manifest for drone_swarm phase 4")
    parser.add_argument("--version", default="2.0.0")
    parser.add_argument("--measurement", required=True)
    parser.add_argument("--signer", default="release-ca")
    parser.add_argument("--secret", required=True, help="shared signing secret used by DRONE_FIRMWARE_SIGNING_SECRET")
    parser.add_argument("--rollback-counter", type=int, default=1)
    parser.add_argument("--secure-boot-attested", action="store_true")
    parser.add_argument("--bootloader-locked", action="store_true")
    parser.add_argument("--out", default="firmware/manifest.env")
    args = parser.parse_args()

    signature = sign_manifest(
        args.secret,
        args.version,
        args.measurement,
        args.signer,
        args.secure_boot_attested,
        args.bootloader_locked,
        args.rollback_counter,
    )

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(
        "\n".join([
            f"version={args.version}",
            f"measurement={args.measurement}",
            f"signer={args.signer}",
            f"signature={signature}",
            f"secure_boot_attested={'true' if args.secure_boot_attested else 'false'}",
            f"bootloader_locked={'true' if args.bootloader_locked else 'false'}",
            f"rollback_counter={args.rollback_counter}",
            "",
        ]),
        encoding="utf-8",
    )
    print(f"firmware manifest written to {out_path}")
    print("Set the following environment variables for hardened runtime:")
    print(f"  DRONE_FIRMWARE_MANIFEST_FILE={out_path}")
    print(f"  DRONE_FIRMWARE_SIGNING_SECRET={args.secret}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
