#ifndef CONFIG_READER_H_
#define CONFIG_READER_H_

#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <unordered_map>

namespace Planning
{

    struct VehicleStruct // 车辆
    {
        int id_{};            // 序列号
        std::string frame_{}; // 坐标名
        double length_{};     // 长度
        double width_{};      // 宽度
        double pose_x_{};     // x坐标
        double pose_y_{};     // y坐标
        double pose_theta_{}; // 航向角
        double speed_ori_{};  // 初速度
    };

    struct PNCMapStruct // PNC地图
    {
        std::string frame_{};      // 坐标名
        int type_{};               // 类型，0: straight, 1: sturn
        double road_length_{};     // 道路长度
        double road_half_width_{}; // 车道半宽
        double segment_len_{};     // 分段长度
        double speed_limit_{};     // 道路限速
    };

    /* 全局路径 */
    struct GlobalPathStruct
    {
        int type_{}; // 类型， 0: normal, 1: astar
    };

    /* 局部路径 */
    struct LocalPathStruct
    {
        int curve_type_{}; // 多项式类型， 0：一次，1：三次，2：五次
        int path_size_{};  // 路径点数
    };

    /* 速度 */
    struct LocalSpeedStruct
    {
        int speed_size_{}; // 速度点数
    };

    /* 参考线 */
    struct ReferLineStruct
    {
        int type_{};       // 类型， 0: normal, 1: stich (拼接)
        int front_size_{}; // 前方点数
        int back_size_{};  // 后方点数
    };

    /* 决策 */
    struct DecisionStruct
    {
        double safe_dis_l_{}; // 侧向安全距离
        double safe_dis_s_{}; // 横向安全距离
    };

    /* 总流程 */
    struct ProcessStruct
    {
        double obs_dis_{}; // 考虑障碍物的距离
    };

    class ConfigReader // 配置文件读取器
    {
    public:
        ConfigReader();

        /* vehicle */
        void read_vehicle_config(VehicleStruct &vehicle, const std::string &name);
        void read_vehicles_config();
        inline const VehicleStruct &get_main_car() const { return main_car_; }
        inline const VehicleStruct &get_obs_car1() const { return obs_car1_; }
        inline const VehicleStruct &get_obs_car2() const { return obs_car2_; }
        inline const VehicleStruct &get_obs_car3() const { return obs_car3_; }
        inline std::unordered_map<int, VehicleStruct> obs_pari() const
        {
            return obs_pair_;
        }

        /* pnc_map */
        void read_pnc_map_config();
        inline const PNCMapStruct &get_pnc_map() const { return pnc_map_; }

        /* global_path */
        void read_global_path_config();
        inline const GlobalPathStruct &get_global_path() const { return global_path_; }

        /* local_path */
        void read_local_path_config();
        inline const LocalPathStruct &get_local_path() const { return local_path_; }

        /* local_speed */
        void read_local_speeds_config();
        inline const LocalSpeedStruct &get_local_speeds() const { return local_speed_; }

        /* reference_line */
        void read_reference_line_config();
        inline const ReferLineStruct &get_refer_line() const { return refer_line_; }

        /* decision */
        void read_decision_config();
        inline const DecisionStruct &get_decision() const { return decision_; }

        /* planning_process */
        void read_planning_process_config();
        inline const ProcessStruct &get_process() const { return process_; }

        /* move_cmd */
        void read_move_cmd_config();

    private:
        YAML::Node planning_config;

        /* vehicle */
        VehicleStruct main_car_;
        VehicleStruct obs_car1_;
        VehicleStruct obs_car2_;
        VehicleStruct obs_car3_;
        std::unordered_map<int, VehicleStruct> obs_pair_;

        /* pnc_map */
        PNCMapStruct pnc_map_;

        /* Global_path */
        GlobalPathStruct global_path_;

        /* local_path */
        LocalPathStruct local_path_;

        /* local_speed */
        LocalSpeedStruct local_speed_;

        /* reference_line */
        ReferLineStruct refer_line_;

        /* decision */
        DecisionStruct decision_;

        /* planning_process */
        ProcessStruct process_;
    };

} // namespace Planning

#endif // CONFIG_READER_H_