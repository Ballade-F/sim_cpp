#include "sim_server.hpp"

SimServer::SimServer(string map_dir_, string robot_config_path_, string trace_save_path_)
{
    map_dir = map_dir_;
    robot_config_path = robot_config_path_;
    trace_save_path = trace_save_path_;
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