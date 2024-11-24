#include "map.hpp"

int main()
{
    Map_2D map(0.1, 0.1, 100, 100, 1, 1, 1, 4);
    vector<Vector2d> starts;
    starts.push_back(Vector2d(0.5, 0.5));
    vector<Vector2d> tasks;
    tasks.push_back(Vector2d(9.5, 9.5));
    vector<vector<Vector2d>> obstacles;
    vector<Vector2d> obstacle;
    obstacle.push_back(Vector2d(3.0, 3.0));
    obstacle.push_back(Vector2d(3.0, 5.0));
    obstacle.push_back(Vector2d(6.0, 5.0));
    obstacle.push_back(Vector2d(6.0, 3.0));
    obstacles.push_back(obstacle);
    map.input_map(starts, tasks, obstacles);
    // map.show_map();
    // map.show_grid_map();
    return 0;
}