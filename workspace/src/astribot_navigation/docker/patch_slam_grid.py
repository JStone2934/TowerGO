"""Pad upstream Karto bounds before ray tracing, not the completed map."""
from pathlib import Path
p=Path('/slam_overlay/src/slam_toolbox/lib/karto_sdk/include/karto_sdk/Karto.h')
s=p.read_text()
old='    rWidth = static_cast<kt_int32s>(math::Round(size.GetWidth() * scale));\n    rHeight = static_cast<kt_int32s>(math::Round(size.GetHeight() * scale));\n    rOffset = boundingBox.GetMinimum();'
new='    // TowerGO: WorldToGrid rounds coordinates; include maximum endpoints\n    // plus two unknown cells on either side before accumulating laser hits.\n    // Fixes clipped occupied cells on aligned outer walls (upstream #689).\n    const kt_int32s padding = 2;\n    rWidth = static_cast<kt_int32s>(std::ceil(size.GetWidth() * scale)) + 1 + 2 * padding;\n    rHeight = static_cast<kt_int32s>(std::ceil(size.GetHeight() * scale)) + 1 + 2 * padding;\n    rOffset = boundingBox.GetMinimum() - Vector2<kt_double>(padding * resolution, padding * resolution);'
if s.count(old)!=1:raise RuntimeError('Pinned upstream source differs from expected grid implementation')
p.write_text(s.replace(old,new))
