#pragma once

#include <iostream>
#include <Eigen/Eigen>
#include <vector>
#include <memory>
#include <random>
#include <cmath>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
using namespace std;
using namespace Eigen;

// 用于多机器人任务分配的2D地图类
class Map_2D
{
public:
    double resolution_x; // 地图分辨率
    double resolution_y;
    int n_x; // 地图尺寸
    int n_y;
    int n_starts; // 起点数量
    int n_tasks; // 任务数量
    int n_obstacles; // 障碍物数量
    int n_ob_points; // 障碍物顶点数

    vector<Vector2d> starts; // 起点
    vector<Vector2d> tasks; // 任务
    vector<uint8_t> task_status; // 任务状态, 1为已完成
    vector<vector<Vector2d>> obstacles; // 障碍物

    //栅格地图
    uint8_t* grid_map; // 0:可通行, 1:障碍物, [x * n_y + y]

    Map_2D(double resolution_x, double resolution_y, int n_x, int n_y, int n_starts, int n_tasks, int n_obstacles, int n_ob_points)
    {
        this->resolution_x = resolution_x;
        this->resolution_y = resolution_y;
        this->n_x = n_x;
        this->n_y = n_y;
        this->n_starts = n_starts;
        this->n_tasks = n_tasks;
        this->n_obstacles = n_obstacles;
        this->n_ob_points = n_ob_points;

        // 初始化地图
        grid_map = new uint8_t[n_x * n_y];
        for (int i = 0; i < n_x * n_y; i++)
        {
            grid_map[i] = 0;
        }

        // 初始化任务状态
        task_status.resize(n_tasks);
        for (int i = 0; i < n_tasks; i++)
        {
            task_status[i] = 0;
        }
    }

    ~Map_2D()
    {
        delete[] grid_map;
    }


    // 手动输入地图
    void input_map(const vector<Vector2d> &starts_, const vector<Vector2d> &tasks_, const vector<vector<Vector2d>> &obstacles_);

    // 显示地图
    void show_map();
    void show_grid_map();
};