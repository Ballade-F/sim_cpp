#include "map.hpp"
#include "hybrid_astar.hpp"
#include "nmpc.hpp"

void updateState(Vector3d& state, VectorXd& control, double dt)
{
    state(0) += control(0) * cos(state(2)) * dt;
    state(1) += control(0) * sin(state(2)) * dt;
    state(2) += control(1) * dt;
    if (state(2) > M_PI)
    {
        state(2) -= 2 * M_PI;
    }
    if (state(2) < -M_PI)
    {
        state(2) += 2 * M_PI;
    }
}


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
    
    Vector3d resolution(0.1, 0.1, M_PI/8);
    Vector3i grid_size(100, 100, 16);
    double max_v = 0.3;
    double max_w = M_PI/4;
    int step_v = 2;
    int step_w = 2;
    double dt = 0.5;
    double finish_radius = 0.3;
    bool path_flag = true;
    HybridAStar hybrid_astar(resolution, grid_size, map.grid_map, max_v, max_w, step_v, step_w, dt, finish_radius,path_flag);
    Vector3d start(0.5, 0.5, 0.0);
    Vector3d goal(9.5, 9.5, 0);
    PlanResult& plan_result = hybrid_astar.plan(start, goal);
    int trace_num = plan_result.trace.size();
    cout << "path length: " << plan_result.path.size() << endl;
    cout << "path success: " << plan_result.success << endl;
    cout << "path planTime: " << plan_result.planTime << endl;
    cout << "path iterations: " << plan_result.iterations << endl;
    cout << "path cost: " << plan_result.cost << endl;

    // if(!path_flag)
    // {
    //     return 0;
    // }

    //nmpc
    int N = 10;
    double dT = 0.1;
    Eigen::MatrixXd Q(3, 3);
    Q << 10, 0, 0,
         0, 10, 0,
         0, 0, 1;
    Eigen::MatrixXd R(2, 2);
    R << 1, 0,
         0, 0.1;
    Eigen::MatrixXd Qf(3, 3);
    Qf << 10, 0, 0,
          0, 10, 0,
          0, 0, 1;
    MPC mpc(N, dT, 0.4, 0.238, Q, R, Qf);
    VectorXd x_ref(trace_num), y_ref(trace_num), theta_ref(trace_num), v_ref(trace_num), w_ref(trace_num);
    for (int i = 0; i < trace_num; i++)
    {
        x_ref(i) = plan_result.trace[i](0);
        y_ref(i) = plan_result.trace[i](1);
        theta_ref(i) = plan_result.trace[i](2);
        v_ref(i) = plan_result.trace_controls[i](0);
        w_ref(i) = plan_result.trace_controls[i](1);
    }
    mpc.setTrackReference(x_ref, y_ref, theta_ref, v_ref, w_ref);
    Vector3d state_car(0.5, 0.5, 0.0);
    Vector2d u_car(0.0, 0.0);
    VectorXd control;
    //plot
    vector<double> mpc_x, mpc_y;
    double mpc_time = 0;
    while(1)
    {
        mpc_x.push_back(state_car(0));
        mpc_y.push_back(state_car(1));
        VectorXd mpc_state(5);
        mpc_state << state_car(0), state_car(1), state_car(2), u_car(0), u_car(1);
        auto start_time = std::chrono::high_resolution_clock::now();
        bool done = mpc.update(state_car, control);
        auto end_time = std::chrono::high_resolution_clock::now();
        mpc_time += chrono::duration<double, std::milli>(end_time - start_time).count();
        if (done)
        {
            break;
        }
        u_car(0) = control(0);
        u_car(1) = control(1);
        updateState(state_car, control, dT);
    }
    mpc_time /= mpc_x.size();
    cout << "mpc time: " << mpc_time << endl;

    // //plot
    map.show_map();
    vector<double> trace_x, trace_y, trace_theta, trace_v, trace_w,path_x,path_y;
    for (int i = 0; i < plan_result.path.size(); i++)
    {
        path_x.push_back(plan_result.path[i](0));
        path_y.push_back(plan_result.path[i](1));
        trace_theta.push_back(plan_result.path[i](2));
        trace_v.push_back(plan_result.controls[i](0));
        trace_w.push_back(plan_result.controls[i](1));
        // cout << plan_result.path[i] << endl;
    }
    for (int i = 0; i < plan_result.trace.size(); i++)
    {
        trace_x.push_back(plan_result.trace[i](0));
        trace_y.push_back(plan_result.trace[i](1));
    }
    plt::plot(trace_x, trace_y, "r-");
    plt::plot(mpc_x, mpc_y, "b-");
    // plt::plot(path_x, path_y, "g-");

    plt::show();


    return 0;
}