#ifndef SWARM_FORMATION_H
#define SWARM_FORMATION_H

#include "common/types.h"
#include <unordered_map>
#include <vector>

struct FollowerTarget {
    int    sysid   = 0;
    double lon     = 0;    // rad
    double lat     = 0;    // rad
    double alt     = 0;    // m (relative)
    double vel     = 0;    // m/s
    double hdg     = 0;    // rad
};

class FormationCalculator {
public:
    FormationCalculator() = default;

    void set_params(const FormationParams &p) { params_ = p; current_dis_ = p.delta_dis; }
    const FormationParams &params() const { return params_; }

    // 根据 leader 状态和航点，计算所有 follower 目标位置
    // follower_ids: 左右僚机 sysid
    // follower_states: 所有 follower 当前状态 (用于速度调节)
    std::vector<FollowerTarget> compute(
        const UnitState &leader_state,
        const WayPoint  &leader_wp,
        const std::vector<int> &follower_ids,
        const std::unordered_map<int, UnitState> &follower_states);

    // 自动编队距离渐变更新（每帧调用）
    void update_auto_formation(const UnitState &leader_state,
                               const WayPoint wps[], int wp_num);

private:
    FormationParams params_;
    double current_dis_ = 100;
    bool   auto_inited_ = false;
};

#endif
