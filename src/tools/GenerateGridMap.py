import sys
import numpy as np
import matplotlib.pyplot as plt


def generate_grid_map(width, height, cell_size):    
    grid_map = np.zeros((height, width))
    # 将grid_map保存为bmp图片
    # 随机生成障碍物，0为障碍物，255为空白
    obstacle_prob = 0.01  # 障碍物概率，可调整
    random_matrix = np.random.rand(height, width)
    grid_map[random_matrix < obstacle_prob] = 0      # 障碍物
    grid_map[random_matrix >= obstacle_prob] = 255   # 空白
    plt.imshow(grid_map, cmap='gray', vmin=0, vmax=255)
    plt.show()
    # 保存为bmp图片
    plt.imsave('grid_map.bmp', grid_map, cmap='gray', vmin=0, vmax=255)
    return grid_map

def main():
    if len(sys.argv) < 3:
        print("Usage: python GenerateGridMap.py <width> <height>")
        width = 100
        height = 100
    else:
        print(sys.argv[1], sys.argv[2])
        width = int(sys.argv[1])
        height = int(sys.argv[2])

    cell_size = 1
    grid_map = generate_grid_map(width, height, cell_size)  
if __name__ == "__main__":
    main()