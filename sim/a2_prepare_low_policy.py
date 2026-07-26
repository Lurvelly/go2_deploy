#!/usr/bin/env python3
"""Create an a2_deploy config for a custom 45D -> 12D TorchScript actor."""

from __future__ import annotations

import argparse
import hashlib
import os
import tempfile
from copy import deepcopy
from pathlib import Path
from typing import Any, Dict, List

import yaml


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def probe_policy(path: Path) -> List[float]:
    try:
        import torch
    except ImportError as exc:
        raise RuntimeError("PyTorch is required to probe the low policy") from exc

    module = torch.jit.load(str(path), map_location="cpu")
    module.eval()
    observation = torch.zeros((1, 45), dtype=torch.float32)
    observation[0, 5] = -1.0
    with torch.inference_mode():
        output = module(observation)
    if not isinstance(output, torch.Tensor):
        raise RuntimeError("low policy must return one Tensor")
    if output.dtype != torch.float32 or tuple(output.shape) != (1, 12):
        raise RuntimeError(
            "low policy must map float32 [1,45] to float32 [1,12], got "
            f"dtype={output.dtype}, shape={tuple(output.shape)}"
        )
    if not bool(torch.isfinite(output).all()):
        raise RuntimeError("low policy probe returned NaN or Inf")
    return [float(value) for value in output[0].cpu().tolist()]


def load_template(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict) or not isinstance(document.get("policy"), dict):
        raise RuntimeError("template must contain a policy mapping")
    return document


def write_config(path: Path, document: Dict[str, Any], force: bool) -> None:
    if path.exists() and not force:
        raise RuntimeError(f"output already exists: {path}; pass --force to replace it")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_name = ""
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=path.name + ".",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary_name = stream.name
            yaml.safe_dump(document, stream, sort_keys=False)
        os.replace(temporary_name, path)
    finally:
        if temporary_name and os.path.exists(temporary_name):
            os.unlink(temporary_name)


def main() -> int:
    project_root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(
        description="Prepare an a2_deploy YAML contract for custom low-policy weights"
    )
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--template", type=Path, default=project_root / "params" / "a2.yaml"
    )
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    policy_path = args.policy.expanduser().resolve()
    template_path = args.template.expanduser().resolve()
    output_path = args.output.expanduser().resolve()
    if not policy_path.is_file():
        parser.error(f"policy does not exist: {policy_path}")
    if not template_path.is_file():
        parser.error(f"template does not exist: {template_path}")

    try:
        document = deepcopy(load_template(template_path))
        reference_output = probe_policy(policy_path)
        policy = document["policy"]
        policy["file"] = str(policy_path)
        policy["sha256"] = sha256_file(policy_path)
        policy["golden_output"] = reference_output
        sim = document.get("sim")
        if not isinstance(sim, dict) or not isinstance(sim.get("xml"), str):
            raise RuntimeError("template must contain sim.xml")
        xml_path = Path(sim["xml"]).expanduser()
        if not xml_path.is_absolute():
            xml_path = project_root / xml_path
        sim["xml"] = str(xml_path.resolve())
        write_config(output_path, document, args.force)
    except Exception as exc:
        parser.error(str(exc))

    print(f"Wrote custom low-policy config: {output_path}")
    print(f"Policy: {policy_path}")
    print(f"SHA-256: {document['policy']['sha256']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
