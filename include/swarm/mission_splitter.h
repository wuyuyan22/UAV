#ifndef SWARM_MISSION_SPLITTER_H
#define SWARM_MISSION_SPLITTER_H

#include "common/types.h"
#include <vector>

enum class SplitStrategy : uint8_t {
    EVEN_WP = 1,
};

struct SubMission {
    int sysid = 0;  // 目标机 MAVLink sysid（长机或僚机）
    std::vector<WayPoint> waypoints;
};

class MissionSplitter {
public:
    /**
     * 将主任务在 agent_sysids 上按航点个数均分（顺序 [长机, 僚机...]）。
     * 除长机外每段起点与上一段最后一个航点重合，保证衔接处被重复覆盖、巡检不断档。
     */
    std::vector<SubMission> split_evenly(const std::vector<WayPoint> &master,
                                         const std::vector<int> &agent_sysids) const;
};

#endif
