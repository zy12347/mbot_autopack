uint32 width          # 栅格图宽度（像素数）
uint32 height         # 栅格图高度（像素数）
float64 resolution    # 每个栅格的实际尺寸（米/像素）
float64 origin_x      # 原点x坐标（米）
float64 origin_y      # 原点y坐标（米）
float64 origin_yaw    # 原点偏航角（弧度）
float32[] data        # 栅格数据，按行优先存储