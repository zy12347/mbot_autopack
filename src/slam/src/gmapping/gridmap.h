#include <stdlib>
#include <vector>

class GridMap{
public:
    GridMap(){
        map_ = new uint8_t[width * height]();
    };
    GridMap(int w,int h):width(w),height(h){
        map_ = new uint8_t[width * height]();
    };
    GridMap(int w,int h,float r):width(w),height(h),resolutino(r){
        map_ = new uint8_t[width * height]();
    };
    ~GridMap(){
        delete[] map_;
        map_ = nullptr;
    }
    void UpdateCell();
    void GetDistance();
    void ToOccupancyGrid();
private:
    int width = 100;
    int height = 100;
    float resolution = 0.05f;
    uint8_t* map_ = nullptr;
}