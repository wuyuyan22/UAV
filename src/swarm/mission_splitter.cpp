#include "swarm/mission_splitter.h"

std::vector<SubMission> MissionSplitter::split_evenly(
    const std::vector<WayPoint> &master,
    const std::vector<int> &agent_sysids) const
{
    std::vector<SubMission> out;
    if (master.empty() || agent_sysids.empty()) return out;

    const size_t n = agent_sysids.size();
    out.resize(n);
    for (size_t i = 0; i < n; i++) out[i].sysid = agent_sysids[i];

    const size_t total = master.size();
    const size_t base = total / n;
    const size_t rem = total % n;

    // 先按个数均分得到无重叠区间 [old_start, old_end)，再让僚机段起点前移 1
    // 个航点，与上一段共点衔接（例：10 点 3 机 -> [0,4), [3,7), [6,10)）。
    std::vector<size_t> old_start(n);
    std::vector<size_t> old_end(n);
    size_t pos = 0;
    for (size_t i = 0; i < n; i++) {
        const size_t cnt = base + (i < rem ? 1 : 0);
        old_start[i] = pos;
        old_end[i] = pos + cnt;
        pos = old_end[i];
    }

    for (size_t i = 0; i < n; i++) {
        const size_t cnt = old_end[i] - old_start[i];
        if (cnt == 0)
            continue;

        size_t seg_begin = old_start[i];
        if (i > 0 && seg_begin > 0)
            seg_begin -= 1;

        const size_t seg_end = old_end[i];
        out[i].waypoints.insert(out[i].waypoints.end(),
                                master.begin() + static_cast<std::ptrdiff_t>(seg_begin),
                                master.begin() + static_cast<std::ptrdiff_t>(seg_end));
    }

    return out;
}
