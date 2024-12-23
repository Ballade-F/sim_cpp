#include "sim_server.hpp"

int main()
{
    string save_dir = "/home/zj/Desktop/wzr/sim_cpp/result/test";
    string map_dir = "/home/zj/Desktop/wzr/sim_cpp/src/config/map/map_exp";
    string robot_config_path = "/home/zj/Desktop/wzr/sim_cpp/src/config/robot/robot_0.json";
    SimServer sim(map_dir, robot_config_path, save_dir);
    bool finished = false;
    while(1)
    {
        finished = sim.SimUpdate();
        if(finished)
        {
            break;
        }
    }
    return 0;
}