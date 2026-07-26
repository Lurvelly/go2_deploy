# A2 policy artifact

`a2_45d_policy.jit` is the deterministic actor from the locally trained A2
iteration-20000 checkpoint. Its current local source copy is:

```text
/home/lurvelly/Workspace/Legged-Nexus/
  legged_gym/ctrl_model/a2/a2_45d_policy.jit
```

The training checkpoint and an alternate exported wrapper are stored at:

```text
/home/lurvelly/Workspace/Legged-Nexus/logs/a2/Jul22_00-16-45/model_20000.pt
/home/lurvelly/Workspace/Legged-Nexus/logs/a2/Jul22_00-16-45/
  exported/Jul22_00-16-45_ite20000.pt
```

Their identities are:

```text
training checkpoint: 2f0728bf629435d465fd9fb4392a235505b8eddfe4ea7ecc6fdc6ccfd7eb3eb9
exported wrapper:    f8555f63ee2fceae0f37286860345b83522d6088cb6e144c3c1c4cc398eb708b
bundled TorchScript: 324d851114f77bb848255026bd56d8d4ebe00a72a72aac54f4271cd644c6fb65
```

All eight actor tensors are bit-exact across the checkpoint, exported wrapper,
and bundled TorchScript (maximum absolute tensor difference 0). Whole-file
digests differ because the checkpoint also contains critic/optimizer state and
the two TorchScript files use different archive/module wrappers. The exported
wrapper additionally applies the training action clip `[-100, 100]`; this does
not change the validated deployment outputs.

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
sha256sum models/a2_45d_policy.jit
```

Generate a config for another trained 45D policy with:

```bash
python3 sim/a2_prepare_low_policy.py \
  --policy /absolute/path/to/my_low_policy.jit \
  --output params/my_low_policy.yaml
./build/a2_deploy --check-contract --config params/my_low_policy.yaml
```
