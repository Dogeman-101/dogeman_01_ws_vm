# Maps

## animation_field.{pgm,yaml}

**MVP 阶段的空白占位地图**。

### 由来

动捕场地为空（4m × 5m），没有能撑起 SLAM 的特征物，因此跳过 gmapping，
用一张 8m × 8m 的纯空白地图占位，让 `map_server` 有东西可加载。

### 参数

- 实际场地：4m × 5m（动捕原点在场地中心）
- 地图大小：8m × 8m（每边约 1.5~2m 缓冲）
- 分辨率：0.05 m/pixel（160 × 160 像素）
- 地图原点：`[-4.0, -4.0, 0.0]` → 地图中心对齐动捕原点 (0, 0)

### 已知限制

- `global_planner` 看不到真实场地边界
- 车若被规划到地图外（~4m 外），move_base 会报 unknown 区域错误，没有下层保护
- goal 设置请控制在 ±2m 以内以保证安全

### 生成方式

```python
import numpy as np
from PIL import Image
img = np.full((160, 160), 255, dtype=np.uint8)  # 255 = free
Image.fromarray(img).save('animation_field.pgm')
```

### 未来替换

正式演示前，建议在场地铺设黑色边带等视觉特征后，重新跑 gmapping 建真实地图。
