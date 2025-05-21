#ifndef LidarStructs_h
#define LidarStructs_h


struct ScanData
{
    int distance = 0;
    int quality = 0;
};

struct ScanPoint
{
    int x = 0;
    int y = 0;
};

#endif