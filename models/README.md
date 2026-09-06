# A2 policy artifact

`a2_45d_policy_0804.jit` is the bundled deterministic actor for the A2 45D
deployment contract. Its SHA-256 identity is:

```text
bundled TorchScript: 886a653beb628ec09b3287b0e0db79279756e535037679f06d05c29d243b6cc8
```

The frozen ABI is one 45D observation in this order:

```text
command(3), projected_gravity(3), body_angular_velocity(3),
joint_position_minus_default(12), joint_velocity(12), previous_raw_action(12)
```

It returns one 12D raw action. Every 12D joint/action block uses
`FR, FL, RR, RL`, with `hip, thigh, calf` within each leg. The deployed joint
target is `default_position + 0.25 * raw_action`. Signal scales and the exact
robot contract are pinned in `params/a2.yaml`.

This is not a Unitree-distributed policy and must not be exchanged with a Go2
45D actor or an A2 47D actor merely because the output shape is 12. Runtime
loaders reject a hash mismatch and a model that does not map float32 45D input
to finite float32 12D output. The bundled config also pins a probe output for
this exact artifact. Custom weights with the same ABI use their own SHA-256
and probe output in a separate YAML file; they do not need to match the
bundled policy's bytes or actions.

Verify the artifact with:

```bash
sha256sum models/a2_45d_policy_0804.jit
```

Generate a config for another trained 45D policy with:

```bash
python3 sim/a2_prepare_low_policy.py \
  --policy /absolute/path/to/my_low_policy.jit \
  --output params/my_low_policy.yaml
./build/a2_deploy --check-contract --config params/my_low_policy.yaml
```
