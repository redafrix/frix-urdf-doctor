# urdf-doctor starter

Read-only Python CLI for link/joint names, graph roots/cycles, mass/inertia, finite vectors, joint limits/axes, collision omissions and mesh-path review. It never opens mesh paths, fetches URLs, expands untrusted xacro, edits input files or certifies hardware.

```sh
python urdf_doctor.py public-examples/cartpole.urdf --output cartpole-report.json
python -m pytest test_urdf_doctor.py -q
```

Three public Bullet descriptions are pinned by commit and SHA256 in `public-examples/PROVENANCE.json`, with the upstream Zlib licence retained. Original files remain unchanged. The reports distinguish errors from review warnings: a static base may intentionally have zero mass; a continuous joint can omit URDF limits but still needs controller effort/velocity review. The installed ROS Humble `check_urdf` parser independently accepted the original cartpole.

`demo_description/` packages the unchanged licensed cartpole for ROS 2 Humble installation. This is a syntax/packaging example, not a repaired or physically validated client robot. Real RViz screenshots, physical correctness, frame/unit review and any hardware test remain separate pending checks. No Isaac installation or local simulator claim.

`OFFERS.md` contains bounded FR/EN offers and monthly support. `ORDER.md` requires Reda's physical review and recorded diff/tests before every delivery. A public GitHub release is allowed only with these limitations intact; no client data is included.

Own CLI code: MIT. Upstream example data: Zlib; see its retained licence. No licence is asserted over third-party content beyond those terms.

Verified locally on 25 Oct 2026: seven tests PASS, three pinned public descriptions linted, unchanged cartpole accepted by ROS Humble check_urdf, and its ament_cmake package built successfully. RViz and hardware checks are not claimed.
