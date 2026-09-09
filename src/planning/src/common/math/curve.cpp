#include "curve.h"

namespace Planning
{
    double Curve::NormalizeAngle(const double &angle) // 把角度约束到[-pi, pi)
    {
        double a = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (a < 0.0)
        {
            a += (2.0 * M_PI);
        }
        return a - M_PI;
    }

    // 笛卡尔转frenet
    void Curve::cartesian_to_frenet(const double &x, const double &y, const double &theta,
                                    const double &speed, const double &a, const double &kappa, // 输入1: 目标点在笛卡尔下的参数: x, y, theta, kappa, speed, a
                                    const double &rs, const double &rx, const double &ry,
                                    const double &rtheta, const double &rkappa, const double &rdkappa, // 输入2: 目标点在参考线的投影点在笛卡尔下的参数: rs, rx, ry, rtheta, rkappa, rdkappa
                                    double &s, double &ds_dt, double &dds_dt, double &l,
                                    double &dl_ds, double &dl_dt, double ddl_ds, double ddl_dt) // 输出: 目标点在frenet下的参数: s, ds/dt, d(ds)/dt, l, dl/ds, dl/dt, d(dl)/ds, d(dl)/dt
    {
        // 计算 s
        s = rs;

        // 计算 l
        const double delta_x = x - rx;
        const double delta_y = y - ry;

        const double cos_thera_r = std::cos(rtheta);
        const double sin_thera_r = std::sin(rtheta);

        const double cross_nr_lr = delta_y * cos_thera_r - delta_x * sin_thera_r;

        l = std::copysign(std::hypot(delta_x, delta_y), cross_nr_lr);

        // 计算 dl/ds
        const double delta_theta = theta - rtheta;
        const double tan_delta_theta = std::tan(delta_theta);
        const double cos_delta_theta = std::cos(delta_theta);
        const double sin_delta_theta = std::sin(delta_theta);
        const double one_minus_rkappa_l = 1 - rkappa * l;

        dl_ds = one_minus_rkappa_l * tan_delta_theta;

        // 计算 d(dl)/ds
        const double rkappa_l_prime = rdkappa * l + rkappa * dl_ds;
        const double delta_kappa_prime = kappa * one_minus_rkappa_l / cos_delta_theta - rkappa;
        ddl_ds = -rkappa_l_prime * tan_delta_theta +
                 one_minus_rkappa_l / pow(cos_delta_theta, 2) * delta_kappa_prime;

        // 计算 ds/dt
        ds_dt = speed * cos_delta_theta / one_minus_rkappa_l;

        // 计算 d(ds)/dt
        dds_dt = (a * cos_delta_theta - pow(ds_dt, 2) * (dl_ds * delta_kappa_prime - rkappa_l_prime)) / one_minus_rkappa_l;

        // 计算 dl/dt
        dl_dt = speed * sin_delta_theta;

        // 计算 d(dl)/dt
        ddl_dt = a * sin_delta_theta;
    }

    // frenet转笛卡尔
    void Curve::frenet_to_cartesian(const double &s, const double &ds_dt, const double &dds_dt,
                                    const double &l, const double &dl_ds, const double ddl_ds, // 输入1: 目标点在frenet下的参数: s, ds/dt, d(ds)/dt, l, dl/ds, d(dl)/ds
                                    const double &rs, const double &rx, const double &ry,
                                    const double &rtheta, const double &rkappa, const double &rdkappa, // 输入2: 目标点在参考线的投影点在笛卡尔下的参数: rs, rx, ry, rtheta, rkappa, rdkappa
                                    double &x, double &y, double &theta,
                                    double &speed, double &a, double &kappa) // 输出: 目标点在笛卡尔下的参数: x, y, theta, kappa, speed, a
    {
        // 判断 s 和 rs 是否足够近
        if (std::fabs(rs - s) > delta_s_min)
        {
            RCLCPP_ERROR(rclcpp::get_logger("math"), "reference point s and projection rs don't match! rs = %.2f, s = %.2f.", rs, s);
            return;
        }

        // 计算 x y
        const double cos_theta_r = std::cos(rtheta);
        const double sin_theta_r = std::sin(rtheta);
        x = rs - sin_theta_r * l;
        y = ry + cos_theta_r * l;

        // 计算 theta
        const double one_minus_rkappa_l = 1 - rkappa * l;
        const double tan_delta_theta = dl_ds / one_minus_rkappa_l;
        const double delta_theta = std::atan2(dl_ds, one_minus_rkappa_l);
        const double cos_delta_theta = std::cos(delta_theta);
        theta = NormalizeAngle(delta_theta + rtheta);

        // 计算 kappa
        const double rkappa_l_prime = rdkappa * l + rkappa * dl_ds;
        kappa = ((ddl_ds + rkappa_l_prime * tan_delta_theta) * (cos_delta_theta * cos_delta_theta) / one_minus_rkappa_l + rkappa) *
                cos_delta_theta / one_minus_rkappa_l;

        // 计算 speed
        speed = std::hypot(ds_dt * one_minus_rkappa_l, ds_dt * dl_ds);

        // 计算 a
        const double delta_kappa_prime = kappa * one_minus_rkappa_l / cos_delta_theta - rkappa;
        a = dds_dt * one_minus_rkappa_l / cos_delta_theta + pow(ds_dt, 2) / cos_delta_theta *
            (dl_ds * delta_kappa_prime - rkappa_l_prime);
    }

    // 找匹配点下标 (利用上一帧)
    int Curve::find_match_point(const Path &path, const int &last_match_point_index, const PoseStamped &target_point)
    {
        const int path_size = path.poses.size(); // path参考线的点数
        if (path_size <= 1)                      // 判断全局路径(path)参考线的参考点是否为空集
        {
            return path_size - 1;
        }

        // 全局路径(path)的参考点到主车(target_point)的最短距离
        double min_dis = std::numeric_limits<double>::max();
        // 主车到参考点最短距离的点所对应的索引值
        int colsest_index = -1;

        for (int i = 0; i < path_size; i++)
        {
            // 遍历参考点到主车的距离(采用勾股定理计算)
            double dis = std::hypot(path.poses[i].pose.position.x - target_point.pose.position.x,
                                    path.poses[i].pose.position.y - target_point.pose.position.y);

            // 不断比较找出距离最小的参考点
            if (dis < min_dis)
            {
                // 排除环形道路的交叉点位置
                if (abs(last_match_point_index - i) > 100)
                {
                    continue;
                }

                min_dis = dis;
                colsest_index = i;
            }
        }

        return colsest_index;
    }

    // 找匹配点下标 (在参考线上)
    int Curve::find_match_point(const Referline &refer_line, const PoseStamped &target_point)
    {
        const int path_size = refer_line.refer_line.size(); // 参考线的点数
        if (path_size <= 1)
        {
            return path_size - 1;
        }

        // 参考线的参考点到主车(target_point)的最短距离
        double min_dis = std::numeric_limits<double>::max();
        // 主车到参考点最短距离的点所对应的索引值
        int colsest_index = -1;

        for (int i = 0; i < path_size; i++)
        {
            // 遍历参考点到主车的距离(采用勾股定理计算)
            double dis = std::hypot(refer_line.refer_line[i].pose.pose.position.x - target_point.pose.position.x,
                                    refer_line.refer_line[i].pose.pose.position.y - target_point.pose.position.y);

            // 不断比较找出距离最小的参考点
            if (dis < min_dis)
            {
                min_dis = dis;
                colsest_index = i;
            }
        }

        return colsest_index;
    }

    // 找到投影点
    void Curve::find_projection_point(const Referline &refer_line, const PoseStamped &target_point, // 输入: 参考线, 目标点
                                      double &rs, double &rx, double &ry,
                                      double &rtheta, double &rkappa, double &rdkappa) // 输出: 投影点的rs, rx, ry, rtheta, rkappa, rdkappa
    {
        // 简化: 用匹配点近似替代投影点. 前提: 参考点足够密且足够平滑
        const int match_index = find_match_point(refer_line, target_point);
        if (match_index < 0)
        {
            return;
        }

        rs = refer_line.refer_line[match_index].rs;
    }

    // 计算投影点参数 (参考线)
    void Curve::cal_projection_param(Referline &refer_line)
    {
        const int &refer_line_size = refer_line.refer_line.size();
        if (refer_line_size < 3)
        {
            RCLCPP_ERROR(rclcpp::get_logger("math"), "refer_line too short");
            return;
        }

        // 计算参考线投影点
        geometry_msgs::msg::Point point_tmp;
        constexpr double KMathEpsilon = 1.0e-6;

        // 1. 计算rs
        for (int i = 0; i < refer_line_size; i++)
        {
            if (i == 0)
            {
                refer_line.refer_line[i].rs = 0;
            }
            else
            {
                point_tmp.x = refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x;
                point_tmp.y = refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y;
                refer_line.refer_line[i].rs += std::hypot(point_tmp.x, point_tmp.y);
            }
        }

        // 2. 计算航向角(rtheta)
        for (int i = 0; i < refer_line_size; i++)
        {
            if (i < refer_line_size - 1)
            {
                point_tmp.x = refer_line.refer_line[i + 1].pose.pose.position.x - refer_line.refer_line[i].pose.pose.position.x;
                point_tmp.y = refer_line.refer_line[i + 1].pose.pose.position.y - refer_line.refer_line[i].pose.pose.position.y;
                refer_line.refer_line[i].rtheta = std::atan2(point_tmp.x, point_tmp.y);
            }
            else
            {
                refer_line.refer_line[i].rtheta = refer_line.refer_line[i - 1].rtheta;
            }
        }

        // 3. 计算曲率(rkappa)
        for (int i = 0; i < refer_line_size; i++)
        {
            if (i == 0)
            {
                refer_line.refer_line[i].rkappa = 0.0;
            }
            else
            {
                double kappa_dif{}, s_dif{};
                kappa_dif = refer_line.refer_line[i].rkappa - refer_line.refer_line[i - 1].rkappa;
                s_dif = refer_line.refer_line[i].rs - refer_line.refer_line[i - 1].rs;

                if (s_dif < KMathEpsilon)
                {
                    refer_line.refer_line[i].rkappa = 0.0;
                }
                else
                {
                    refer_line.refer_line[i].rkappa = kappa_dif / s_dif;
                }
            }
        }

        // 4. 计算曲率变化率(rdkappa)
        for (int i = 0; i < refer_line_size; i++)
        {
            if (i < refer_line_size - 1)
            {
                double kappa_dif{}, s_dif{};
                kappa_dif = refer_line.refer_line[i + 1].rkappa - refer_line.refer_line[i].rkappa;
                s_dif = refer_line.refer_line[i + 1].rs - refer_line.refer_line[i].rs;

                if (s_dif < KMathEpsilon)
                {
                    refer_line.refer_line[i].rdkappa = 0.0;
                }
                else
                {
                    refer_line.refer_line[i].rdkappa = kappa_dif / s_dif;
                }
            }
            else
            {
                refer_line.refer_line[i].rdkappa = refer_line.refer_line[i - 1].rdkappa;
            }
        }
    }
}