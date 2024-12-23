#include "sim_server.hpp"


SimServer::SimServer(string map_dir_, string robot_config_path_, string trace_save_path_)
{
    map_dir = map_dir_;
    robot_config_path = robot_config_path_;
    trace_save_dir = trace_save_path_;
    string map_csv_path = map_dir + "/info.csv";
    string map_json_path = map_dir + "/batch_info.json";
    // 创建 JSON 解析器
    Json::Reader reader;
    Json::Value map_root;
    Json::Value robot_root;
    // 读取 map.json 文件
    std::ifstream map_file(map_json_path, std::ifstream::binary);
    if (!map_file.is_open()) 
    {
        cout << "map.json open failed!" << endl;
        return ;
    }
    if (!reader.parse(map_file, map_root, false)) 
    {
        cout << "map.json parse failed!" << endl;
        return ;
    }
    map_file.close();
    // 读取 robot.json 文件
    std::ifstream robot_file(robot_config_path, std::ifstream::binary);
    if (!robot_file.is_open()) 
    {
        cout << "robot.json open failed!" << endl;
        return ;
    }
    if (!reader.parse(robot_file, robot_root, false)) 
    {
        cout << "robot.json parse failed!" << endl;
        return ;
    }
    robot_file.close();
    // 提取配置信息
    //map
    map_Nrobot = map_root["n_robot"].asInt();
    map_Ntask = map_root["n_task"].asInt();
    map_Nobstacle = map_root["n_obstacle"].asInt();
    map_ob_points = map_root["ob_points"].asInt();
    map_Nx = map_root["n_x"].asInt();
    map_Ny = map_root["n_y"].asInt();
    map_resolution_x = map_root["resolution_x"].asDouble();
    map_resolution_y = map_root["resolution_y"].asDouble();
    //planner
    planner_Ntheta = robot_root["planner_Ntheta"].asDouble();
    planner_Vmax = robot_root["planner_Vmax"].asDouble();
    planner_Wmax = robot_root["planner_Wmax"].asDouble();
    planner_Vstep = robot_root["planner_Vstep"].asInt();
    planner_Wstep = robot_root["planner_Wstep"].asInt();
    planner_TraceStep = robot_root["planner_TraceStep"].asInt();
    planner_dt = robot_root["planner_dt"].asDouble();
    planner_Rfinish = robot_root["planner_Rfinish"].asDouble();
    //mpc
    mpc_N = robot_root["mpc_N"].asInt();
    mpc_dt = robot_root["mpc_dt"].asDouble();
    mpc_Qxy = robot_root["mpc_Qxy"].asDouble();
    mpc_Qtheta = robot_root["mpc_Qtheta"].asDouble();
    mpc_Rv = robot_root["mpc_Rv"].asDouble();
    mpc_Rw = robot_root["mpc_Rw"].asDouble();
    mpc_wheel_width = robot_root["mpc_wheel_width"].asDouble();
    mpc_wheel_Vmax = robot_root["mpc_wheel_Vmax"].asDouble();
    //network
    string device_string = robot_root["device"].asString();
    string allocation_model_path = robot_root["allocation_model_path"].asString();
    string intention_model_path = robot_root["intention_model_path"].asString();

    //初始化各个vector
    vector<vector<Vector2d>> obstacles;
    vector<Vector3d> start_(map_Nrobot, Vector3d::Zero());
    vector<Vector3d> task_(map_Ntask, Vector3d::Zero());
    csv2vector(map_csv_path, start_,task_,obstacles, map_Nrobot, map_Ntask, map_Nobstacle, map_ob_points);
    robot_states.resize(map_Nrobot, Vector3d::Zero());
    robot_ctrls.resize(map_Nrobot, Vector2d::Zero());
    task_states.resize(map_Ntask, Vector3d::Zero());
    task_finished.resize(map_Ntask, 0);
    for(int i = 0; i < map_Nrobot; i++)
	{
		robot_states[i] = start_[i];
	}
	for(int i = 0; i < map_Ntask; i++)
	{
		task_states[i] = task_[i];
	}
    counter_max = 1000;//TODO: 与任务规模有关
    robot_trace.resize(counter_max, vector<Vector3d>(map_Nrobot, Vector3d::Zero()));
    distances.resize(map_Nrobot, 0.0);

    //初始化机器人
    for(int i = 0; i < map_Nrobot; i++)
    {
        //map
        auto map_p = std::make_shared<Map_2D>();
        map_p->init(map_resolution_x, map_resolution_y, map_Nx, map_Ny, map_Nrobot, map_Ntask, map_Nobstacle, map_ob_points);
        map_p->input_map(start_, task_, obstacles);
        //hybrid_astar
        auto hybrid_astar_p = std::make_shared<HybridAStar>();
        auto hybrid_dist_astar_p = std::make_shared<HybridAStar>();
        Vector3d resolution(map_resolution_x, map_resolution_y, 2*M_PI/planner_Ntheta);
        Vector3i grid_size(map_Nx, map_Ny, planner_Ntheta);
        hybrid_astar_p->init(resolution, grid_size, map_p->grid_map, planner_Vmax, planner_Wmax, 
                             planner_Vstep, planner_Wstep,planner_TraceStep, planner_dt, planner_Rfinish, true);
        hybrid_dist_astar_p->init(resolution, grid_size, map_p->grid_map, planner_Vmax, planner_Wmax,
                                  planner_Vstep, planner_Wstep,planner_TraceStep, planner_dt, planner_Rfinish, false);
        //mpc
        Eigen::MatrixXd Q(3, 3);
        Q << mpc_Qxy, 0, 0,
            0, mpc_Qxy, 0,
            0, 0, mpc_Qtheta;
        Eigen::MatrixXd R(2, 2);
        R << mpc_Rv, 0,
            0, mpc_Rw;
        Eigen::MatrixXd Qf(3, 3);
        Qf << mpc_Qxy, 0, 0,
            0, mpc_Qxy, 0,
            0, 0, mpc_Qtheta;
        auto mpc_p = std::make_shared<MPC>();
        mpc_p->init(mpc_N, mpc_dt, mpc_wheel_Vmax, mpc_wheel_width, Q, R, Qf);
        //network
        double x_max = map_Nx * map_resolution_x;
        double y_max = map_Ny * map_resolution_y;
        double v_max_network = 0.6*planner_Vmax * dt * keyframe_counter;
        auto network_p = std::make_shared<Network>(x_max, y_max, v_max_network, map_Nrobot, map_Ntask, map_Nobstacle, map_ob_points, ROBOT_BUFFER_SIZE,
                                          allocation_model_path, intention_model_path, device_string, map_csv_path);
        //robot
        auto robot_p = std::make_shared<Robot>(i, map_p, mpc_p, hybrid_astar_p, hybrid_dist_astar_p, network_p);
        robots.push_back(robot_p);
    }

    

}

//按dt仿真更新一次
bool SimServer::SimUpdate()
{
    perceptionUpdate();
    if(time_counter % keyframe_counter == 0)
    {
        keyframeUpdate();
    }
    if(time_counter % decision_counter == 0)
    {
        decisionUpdate();
    }
    controlUpdate();
    traceUpdate();
    bool finish_flag_ = judgeFinish();
    time_counter++;
    return finish_flag_;
}

void SimServer::perceptionUpdate()
{
    for (int i = 0; i < map_Nrobot; i++)
    {
        robots[i]->perceptionUpdate(robot_states, task_states, task_finished, robot_ctrls[i]);
    }
}

void SimServer::keyframeUpdate()
{
    for (int i = 0; i < map_Nrobot; i++)
    {
        robots[i]->keyframeUpdate();
    }
}

void SimServer::decisionUpdate()
{
    for (int i = 0; i < map_Nrobot; i++)
    {
        //测回调时间
        auto start_time = std::chrono::high_resolution_clock::now();
        robots[i]->decisionUpdate();
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> decision_time = end_time - start_time;
        cout << "robot_id: " << i << " decision_callback_time: " << decision_time.count() << endl;
    }
}

void SimServer::controlUpdate()
{
    for(int i = 0; i < map_Nrobot; i++)
    {
        robots[i]->pncUpdate();
        robot_ctrls[i] = robots[i]->ctrlOutput();
    }
}

void SimServer::traceUpdate()
{
    //更新机器人状态
    for(int i = 0; i < map_Nrobot; i++)
    {
        Vector3d state = robot_states[i];
        Vector2d ctrl = robot_ctrls[i];
        state(0) += ctrl(0) * cos(state(2)) * dt;
        state(1) += ctrl(0) * sin(state(2)) * dt;
        state(2) += ctrl(1) * dt;
        robot_states[i] = state;
        //更新距离和轨迹
        distances[i] += ctrl(0) * dt;
        robot_trace.at(time_counter).at(i) = state;
    }
    //更新任务状态
    for(int i = 0; i < map_Ntask; i++)
    {
        if(task_finished[i] == 1)
        {
            continue;
        }
        Vector3d state = task_states[i];
        for(int j = 0; j < map_Nrobot; j++)
        {
            if((robot_states[j].head(2) - state.head(2)).norm() < finish_radius)
            {
                task_finished[i] = 1;
                break;
            }
        }
    }
}

//所有任务都完成或时间超限,如果完成，存一个csv保存轨迹，存一个json保存时间、距离、完成情况
bool SimServer::judgeFinish()
{
    bool finish_flag = true;
    for(int i = 0; i < map_Ntask; i++)
    {
        if(task_finished[i] == 0)
        {
            finish_flag = false;
            break;
        }
    }
    if(time_counter >= counter_max)
    {
        finish_flag = true;
    }
    if(finish_flag)
    {
        //存csv
        ofstream tracefile(trace_save_dir + "/trace.csv");
        for(int i = 0; i < time_counter; i++)
        {
            for(int j = 0; j < map_Nrobot; j++)
            {
                tracefile << i << "," << j << "," << robot_trace[i][j][0] << "," << robot_trace[i][j][1] << "," << robot_trace[i][j][2] << endl;
            }
        }
        tracefile.close();
        //存json
        Json::Value root;
        root["time"] = time_counter;
        root["distances_sum"] = std::accumulate(distances.begin(), distances.end(), 0.0);
        root["unfinished_sum"] = map_Ntask - std::accumulate(task_finished.begin(), task_finished.end(), 0);
        root["distances"] = Json::Value(Json::arrayValue);
        root["finished"] = Json::Value(Json::arrayValue);
        for(int i = 0; i < map_Nrobot; i++)
        {
            root["distances"].append(distances[i]);
            root["finished"].append(task_finished[i]);
        }
        Json::StyledWriter writer;
        std::ofstream jsonfile(trace_save_dir + "/trace.json");
        jsonfile << writer.write(root);
        jsonfile.close();
    }
    return finish_flag;
}

void SimServer::csv2vector(const string& csv_path, vector<Vector3d>& starts_, vector<Vector3d>& tasks_, vector<vector<Vector2d>>& obstacles_, int n_robot, int n_task, int n_obstacle, int ob_point)
{
    obstacles_.clear();
    for(int i = 0; i < n_obstacle; i++)
    {
        vector<Vector2d> obstacle(ob_point, Vector2d::Zero());
        obstacles_.push_back(obstacle);
    }
    ifstream csvfile(csv_path);
    std::string line;
    int idx = 0;
    while (std::getline(csvfile, line)) 
    {
        std::istringstream ss(line);
        std::string token;
        std::vector<std::string> row;
        while (std::getline(ss, token, ',')) 
        {
            row.push_back(token);
        }
		// RCLCPP_INFO(this->get_logger(), "CONDISION:%s", (idx > n_robot+n_task && idx <= n_robot+n_task+n_obstacle*ob_point)?"TRUE":"FALSE");
        //表头
        if(idx == 0)
        {
            idx++;
            continue;
        }
        else if(idx <= n_robot)
        {
            starts_[idx-1][0] = std::stof(row[1]) * map_resolution_x * map_Nx;
            starts_[idx-1][1] = std::stof(row[2]) * map_resolution_y * map_Ny;
        }
        else if(idx <= n_robot+n_task)
        {
            tasks_[idx-n_robot-1][0] = std::stof(row[1]) * map_resolution_x * map_Nx;
            tasks_[idx-n_robot-1][1] = std::stof(row[2]) * map_resolution_y * map_Ny;
        }
        else if(idx <= n_robot+n_task+n_obstacle*ob_point)
        {
            int idx_ob = std::stoi(row[0]) - 1;
            int idx_point = (idx - n_robot - n_task - 1) - idx_ob * ob_point;
            obstacles_[idx_ob][idx_point][0] = std::stof(row[1]) * map_resolution_x * map_Nx;
            obstacles_[idx_ob][idx_point][1] = std::stof(row[2]) * map_resolution_y * map_Ny;
        }
        idx++;
    }
    csvfile.close();
}