#include "map.hpp"
#include "hybrid_astar.hpp"

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
    
    Vector3d resolution(0.1, 0.1, M_PI/16);
    Vector3i grid_size(100, 100, 32);
    double max_v = 0.4;
    double max_w = 0.3;
    int step_v = 4;
    int step_w = 3;
    double dt = 0.5;
    double finish_radius = 0.4;
    HybridAStar hybrid_astar(resolution, grid_size, map.grid_map, max_v, max_w, step_v, step_w, dt, finish_radius);
    Vector3d start(0.5, 0.5, 0.0);
    Vector3d goal(9.5, 9.5, 0.0);
    PlanResult& plan_result = hybrid_astar.plan(start, goal);
    cout << "path length: " << plan_result.trace.size() << endl;

    //plot
    map.show_map();
    vector<double> trace_x, trace_y, trace_theta, trace_v, trace_w;
    for (int i = 0; i < plan_result.trace.size(); i++)
    {
        trace_x.push_back(plan_result.trace[i](0));
        trace_y.push_back(plan_result.trace[i](1));
        trace_theta.push_back(plan_result.trace[i](2));
        trace_v.push_back(plan_result.controls[i](0));
        trace_w.push_back(plan_result.controls[i](1));
    }
    plt::plot(trace_x, trace_y, "r-");
    plt::show();


    return 0;
}