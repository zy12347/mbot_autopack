#include <stdlib>
#include <vector>

class GridMap{
public:
    void UpdateCell();
    void GetDistance();
    void ToOccupancyGrid();
private:
    int width;
    int height;
    int resolution;
    std::vector<std::vector<int>> map_;
}