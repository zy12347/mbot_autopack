import numpy as np
from PIL import Image

# 地图参数
width = 500   # 像素宽度
height = 500  # 像素高度
obstacle_ratio = 0.02  # 障碍占比

# 创建空白地图（255=白色=可通行）
map_data = np.ones((height, width), dtype=np.uint8) * 100

# 随机生成障碍（0=黑色=障碍）
np.random.seed(42)  # 固定随机种子，确保结果可复现
obstacle_mask = np.random.rand(height, width) < obstacle_ratio
map_data[obstacle_mask] = 0

# 添加边界障碍（可选，确保机器人在地图内）
map_data[0, :] = 0       # 上边界
map_data[-1, :] = 0      # 下边界
map_data[:, 0] = 0       # 左边界
map_data[:, -1] = 0      # 右边界

# 保存为PGM格式
img = Image.fromarray(map_data)
img.save('img/map.pgm')
print(f"PGM地图已生成：my_map.pgm（尺寸：{width}x{height}像素）")
