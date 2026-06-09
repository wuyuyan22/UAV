#include "mavlink_adapter/mavlink_manager.h"
#include <cstdio>
#include <chrono>

bool MavlinkManager::add_vehicle(const VehicleConfig &cfg) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (vehicles_.count(cfg.sysid)) {
        fprintf(stderr, "[manager] sysid %d already exists\n", cfg.sysid);
        return false;
    }
    auto v = std::make_unique<MavlinkVehicle>(cfg);
    if (!v->connect()) {
        fprintf(stderr, "[manager] connect vehicle %d failed\n", cfg.sysid);
        return false;
    }
    printf("[manager] vehicle %d added\n", cfg.sysid);
    vehicles_[cfg.sysid] = std::move(v);
    return true;
}

void MavlinkManager::remove_vehicle(int sysid) {
    std::lock_guard<std::mutex> lk(mtx_);
    vehicles_.erase(sysid);
}

MavlinkVehicle *MavlinkManager::get_vehicle(int sysid) {
    std::lock_guard<std::mutex> lk(mtx_);
    auto it = vehicles_.find(sysid);
    return (it != vehicles_.end()) ? it->second.get() : nullptr;
}

std::vector<int> MavlinkManager::connected_sysids() const {
    std::lock_guard<std::mutex> lk(mtx_);
    std::vector<int> ids;
    for (auto &kv : vehicles_) {
        if (kv.second->is_connected()) ids.push_back(kv.first);
    }
    return ids;
}

size_t MavlinkManager::vehicle_count() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return vehicles_.size();
}

void MavlinkManager::read_all() {
    std::lock_guard<std::mutex> lk(mtx_);
    for (auto &kv : vehicles_) {
        kv.second->read_messages();
    }
}

void MavlinkManager::send_heartbeats() {
    std::lock_guard<std::mutex> lk(mtx_);
    for (auto &kv : vehicles_) {
        kv.second->send_heartbeat();
    }
}

std::unordered_map<int, UnitState> MavlinkManager::get_all_states() const {
    std::lock_guard<std::mutex> lk(mtx_);
    std::unordered_map<int, UnitState> states;
    for (auto &kv : vehicles_) {
        states[kv.first] = kv.second->state();
    }
    return states;
}

bool MavlinkManager::is_vehicle_alive(int sysid, uint64_t timeout_ms) const {
    std::lock_guard<std::mutex> lk(mtx_);
    auto it = vehicles_.find(sysid);
    if (it == vehicles_.end()) return false;

    using namespace std::chrono;
    uint64_t now = duration_cast<milliseconds>(
        steady_clock::now().time_since_epoch()).count();
    return (now - it->second->last_heartbeat_ms()) < timeout_ms;
}
