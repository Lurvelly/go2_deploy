# Unitree A2 asset provenance

The copied A2 description comes from:

- repository: `https://github.com/unitreerobotics/unitree_ros`
- path: `robots/a2_description`
- commit: `d96d8f63ae17a7108d4f7229c00ef875ba7129c9`
- license: BSD-3-Clause; see `LICENSE`

`a2.xml` and all 17 files under `meshes/` are copied byte-for-byte from the
local Legged-Nexus asset snapshot. Their relative layout is required by the
MuJoCo compiler's `meshdir="meshes/"` setting. This deployment package omits
the URDF because the standalone simulator consumes only MJCF.

`scene.xml` is the flat-ground MuJoCo wrapper from the matching local
`/home/lurvelly/Workspace/unitree_rl_gym/resources/robots/unitree_robotics/a2`
snapshot. It adds only the floor, light, and viewer materials around `a2.xml`.
Verify the copied runtime payload with:

```bash
cd assets/a2
sha256sum -c SHA256SUMS
```

The deployment pose, PD gains, and action scale are based on Unitree's official
`unitree_rl_mjlab` A2 configuration at commit
`1425b15f73bd4095f0df53709d7c389c3eb9e790`, specifically
`src/assets/robots/unitree_a2/a2_constants.py` and the A2 velocity deployment
configuration. Those values are authored in `params/a2.yaml` and are not part
of the copied description payload. The bundled policy uses the project-native
45D schema; it is not a Unitree-distributed policy.
