# LW robot description provenance

This directory vendors the LW-only subset of `rl_sar_zoo` as ordinary files
in the parent `rl_sar` repository. It intentionally contains no nested Git
metadata and no descriptions for other robots.

## Source

- Project fork: <https://github.com/bhwxzx/rl_sar_zoo.git>
- Upstream project: <https://github.com/fan-ziqi/rl_sar_zoo.git>
- Imported baseline: `349d14a700ecf248b3cdbec5e7bac30882b66e62`
- Imported directory: `LW_description/`

The imported package metadata declares `Apache-2.0`. The parent repository's
Apache License is retained at `/LICENSE`; this provenance record does not
replace or broaden the rights stated by the source package and repository.

## Preserved working-tree changes

The vendored snapshot includes the exact local LW Sim2Sim work that existed at
the time of import:

- modified `LW_description/mjcf/LW.xml`;
- modified `LW_description/mjcf/scene.xml`;
- modified `LW_description/mjcf/scene_terrain.xml`;
- added terrain height map
  `LW_description/mjcf/assets/terrain/blind_rough_and_stairs_terrain.png`;
- added terrain metadata
  `LW_description/mjcf/assets/terrain/blind_rough_and_stairs_terrain_info.txt`.

`LW_DESCRIPTION_MANIFEST.sha256` records the exact imported package contents.
Intentional future asset changes must update that manifest in the same commit.
