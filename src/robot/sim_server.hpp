#include "robot.hpp"
#include "json/json.h"
#include <numeric>
class SimServer
{
public:
    //输入参数
    string map_csv_path;
    string map_json_path;
    string robot_config_path;
    string trace_save_dir;

    //map
    int map_Nrobot;
    int map_Ntask;
    int map_Nobstacle;
    int map_ob_points;
    double map_resolution_x;
    double map_resolution_y;
    int map_Nx;
    int map_Ny;
    //planner
    int planner_Ntheta;
    double planner_Vmax;
    double planner_Wmax;
    int planner_Vstep;
    int planner_Wstep;
    int planner_TraceStep;
    double planner_dt;
    double planner_Rfinish;
    //mpc
    int mpc_N;
    double mpc_dt;
    double mpc_wheel_Vmax;
    double mpc_wheel_width;
    double mpc_Qxy;
    double mpc_Qtheta;
    double mpc_Rv;
    double mpc_Rw;

    vector<Vector3d> robot_states;//各个机器人当前的状态 x, y, theta
    vector<Vector2d> robot_ctrls;//各个机器人当前的控制 v, w
    vector<Vector3d> task_states;//各个任务的位置 x, y, theta 但是theta不用
    vector<uint8_t> task_finished;//各个任务是否完成

    double dt = 0.1;
    int keyframe_counter = 5;
    int decision_counter = 10;
    double finish_radius = 0.3;

    vector<std::shared_ptr<Robot>> robots;

    //记录轨迹，最外层是时间，中间是机器人，最里层是x,y,theta，最后存csv
    int time_counter = 0;
    int counter_max;//与任务规模有关
    vector<vector<Vector3d>> robot_trace;//初始化的时候分配好内存
    vector<double> distances;
    

    SimServer(string map_csv_path_, string map_json_path, string robot_config_path_, string trace_save_path_);
    
    bool SimUpdate();

    void perceptionUpdate();
    void keyframeUpdate();
    void decisionUpdate();
    void controlUpdate();
    void traceUpdate();
    bool judgeFinish();


    void csv2vector(const string& csv_path, vector<Vector3d>& starts_, vector<Vector3d>& tasks_, vector<vector<Vector2d>>& obstacles_, int n_robot, int n_task, int n_obstacle, int ob_point);

};