#ifndef SWARM_MAVLINK_MANAGER_H
#define SWARM_MAVLINK_MANAGER_H

#include "mavlink_adapter/mavlink_vehicle.h"
#include "common/types.h"
#include <unordered_map>
#include <memory>
#include <vector>
#include <mutex>

class MavlinkManager {
public:
    MavlinkManager() = default;
    ~MavlinkManager() = default;

    bool add_vehicle(const VehicleConfig &cfg);
    void remove_vehicle(int sysid);

    MavlinkVehicle *get_vehicle(int sysid);
    std::vector<int> connected_sysids() const;
    size_t vehicle_count() const;

    // 全部车辆批量收包
    void read_all();
    // 全部车辆发送心跳
    void send_heartbeats();

    // 获取全局状态快照
    std::unordered_map<int, UnitState> get_all_states() const;

    // 检查连接健康（超时判断）
    bool is_vehicle_alive(int sysid, uint64_t timeout_ms = 3000) const;

private:
    std::unordered_map<int, std::unique_ptr<MavlinkVehicle>> vehicles_;
    mutable std::mutex mtx_;
};

#endif
