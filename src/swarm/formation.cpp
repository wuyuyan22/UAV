#include "swarm/formation.h"
#include "common/math_utils.h"
#include <cmath>
#include <cstdio>

std::vector<FollowerTarget> FormationCalculator::compute(
    const UnitState &leader,
    const WayPoint  &leader_wp,
    const std::vector<int> &follower_ids,
    const std::unordered_map<int, UnitState> &follower_states)
{
    std::vector<FollowerTarget> targets;
    if (follower_ids.empty()) return targets;

    // 渐变编队间距
    double target_dis = params_.delta_dis;
    if (fabs(target_dis - current_dis_) <= 1.5) {
        current_dis_ = target_dis;
    } else if (target_dis > current_dis_ + 1.5) {
        current_dis_ += 1.5;
    } else if (target_dis < current_dis_ - 1.5) {
        current_dis_ -= 1.5;
    }

    double heading = leader.heading;
    double vel_ahead_km = 2.0 * leader_wp.vel * 1e-3;

    // 第一个 follower：左翼
    if (follower_ids.size() >= 1) {
        FollowerTarget ft;
        ft.sysid = follower_ids[0];

        double tmplon, tmplat;
        offset_position(current_dis_ * 1e-3,
                        heading - params_.angle,
                        leader.lon, leader.lat,
                        &tmplon, &tmplat);
        offset_position(vel_ahead_km, heading,
                        tmplon, tmplat, &tmplon, &tmplat);

        ft.lon = tmplon;
        ft.lat = tmplat;
        ft.alt = leader_wp.alt - params_.delta_H;
        ft.alt = clamp(ft.alt, params_.minH, params_.maxH);
        ft.hdg = heading;

        auto it = follower_states.find(ft.sysid);
        if (it != follower_states.end()) {
            double dis = distance_between(it->second.lat, it->second.lon, tmplat, tmplon);
            ft.vel = clamp(0.5 * dis, params_.minV, params_.maxV);
        } else {
            ft.vel = leader_wp.vel;
        }
        targets.push_back(ft);
    }

    // 第二个 follower：右翼
    if (follower_ids.size() >= 2) {
        FollowerTarget ft;
        ft.sysid = follower_ids[1];

        double tmplon, tmplat;
        offset_position(current_dis_ * 1e-3,
                        heading + params_.angle,
                        leader.lon, leader.lat,
                        &tmplon, &tmplat);
        offset_position(vel_ahead_km, heading,
                        tmplon, tmplat, &tmplon, &tmplat);

        ft.lon = tmplon;
        ft.lat = tmplat;
        ft.alt = leader_wp.alt + params_.delta_H;
        ft.alt = clamp(ft.alt, params_.minH, params_.maxH);
        ft.hdg = heading;

        auto it = follower_states.find(ft.sysid);
        if (it != follower_states.end()) {
            double dis = distance_between(it->second.lat, it->second.lon, tmplat, tmplon);
            ft.vel = clamp(0.5 * dis, params_.minV, params_.maxV);
        } else {
            ft.vel = leader_wp.vel;
        }
        targets.push_back(ft);
    }

    // 更多 follower（环形编队扩展）
    for (size_t i = 2; i < follower_ids.size(); i++) {
        FollowerTarget ft;
        ft.sysid = follower_ids[i];
        double angle_offset = params_.angle * ((i % 2 == 0) ? -(double)(i/2) : (double)((i+1)/2));
        double tmplon, tmplat;
        offset_position(current_dis_ * 1e-3 * (1.0 + i * 0.2),
                        heading + angle_offset,
                        leader.lon, leader.lat,
                        &tmplon, &tmplat);
        offset_position(vel_ahead_km, heading, tmplon, tmplat, &tmplon, &tmplat);
        ft.lon = tmplon;
        ft.lat = tmplat;
        ft.alt = clamp(leader_wp.alt + params_.delta_H * ((i % 2 == 0) ? -1.0 : 1.0) * (i / 2 + 1),
                       params_.minH, params_.maxH);
        ft.hdg = heading;
        ft.vel = leader_wp.vel;
        targets.push_back(ft);
    }

    return targets;
}

void FormationCalculator::update_auto_formation(
    const UnitState &leader, const WayPoint wps[], int wp_num)
{
    if (!params_.auto_formation || wp_num < 2) return;
    if (auto_inited_) return;

    double mlon, mlat;
    double azimuth = bearing_between(wps[1].lat, wps[1].lon, wps[0].lat, wps[0].lon);
    double dis1 = distance_between(wps[0].lat, wps[0].lon, wps[1].lat, wps[1].lon);

    if (dis1 > 2.0 * params_.dis_to_fly) {
        offset_position(params_.dis_to_fly * 1e-3, azimuth,
                        wps[1].lon, wps[1].lat, &mlon, &mlat);
    } else {
        mlon = (wps[0].lon + wps[1].lon) / 2.0;
        mlat = (wps[0].lat + wps[1].lat) / 2.0;
    }

    double dis = distance_between(leader.lat, leader.lon, mlat, mlon);
    if (dis < 100.0) {
        params_.auto_formation = false;
        auto_inited_ = true;
        params_.delta_dis += params_.auto_formation_dis;
        if (params_.delta_dis < 0) params_.delta_dis = 0;
        printf("[formation] auto-adjust complete, new delta_dis=%.1f\n", params_.delta_dis);
    }
}
