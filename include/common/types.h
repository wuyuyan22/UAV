#ifndef SWARM_TYPES_H
#define SWARM_TYPES_H

#include <cstdint>
#include <cmath>

#define SWARM_PI       3.14159265358979323846
#define SWARM_RE       6371393.0
#define SWARM_DEG2RAD  (SWARM_PI / 180.0)
#define SWARM_RAD2DEG  (180.0 / SWARM_PI)
#define SWARM_GRAVITY  9.81

static constexpr int MAX_VEHICLES   = 10;
static constexpr int MAX_WAYPOINTS  = 128;

enum class NodeRole : uint8_t {
    LEADER    = 1,
    FOLLOWER  = 2,
    UNDEFINED = 3
};

struct UnitState {
    int      id        = 0;
    uint8_t  role      = 0;
    float    pitch     = 0;      // rad
    float    roll      = 0;      // rad
    float    heading   = 0;      // rad
    float    mag_hdg   = 0;      // rad
    float    vn        = 0;      // m/s 北向
    float    ve        = 0;      // m/s 东向
    float    vd        = 0;      // m/s 天向
    double   lon       = 0;      // rad
    double   lat       = 0;      // rad
    float    alt_bar   = 0;      // m 海拔
    float    alt_rel   = 0;      // m 相对高度
    uint8_t  gps_fix   = 0;
    float    vel_cruise = 0;     // m/s 巡航
    float    throttle  = 0;

    float    vcmd      = 0;      // 速度指令
    float    hcmd      = 0;      // 高度指令
    float    heading_cmd = 0;    // 航向指令
    double   lon_cmd   = 0;      // 目标经度 rad
    double   lat_cmd   = 0;      // 目标纬度 rad

    uint16_t health    = 0xFFFF;
    uint8_t  armed     = 0;
    uint32_t custom_mode = 0;
    uint8_t  custom_mode_valid = 0;
    int      battery_pct = -1;   // 电池百分比 0-100, -1=未知

    // 本地 NED 坐标（以 Home 点为原点）
    float    x_ned = 0;  // m 北向
    float    y_ned = 0;  // m 东向
    float    z_ned = 0;  // m 下方为正（即 -alt_rel）
};

struct WayPoint {
    double lon    = 0;   // rad
    double lat    = 0;   // rad
    double alt    = 0;   // m (relative)
    double vel    = 0;   // m/s
    double hdg    = 0;   // rad
    int    mode   = 0;   // 1=直飞 2=航线 3=盘旋
    double radius = 0;   // 盘旋半径 m
};

struct UavCmd {
    uint8_t mode     = 0;
    double  lon_cmd  = 0;   // rad
    double  lat_cmd  = 0;   // rad
    double  alt_cmd  = 0;   // m
    double  hdg_cmd  = 0;   // rad
    double  vel_cmd  = 0;   // m/s
    double  minV     = 0;
};

struct FormationParams {
    int     type           = 0;
    double  delta_dis      = 100;    // m 编队间距
    double  delta_H        = 30;     // m 高度差
    double  angle          = 2.0 * SWARM_PI / 3.0; // 编队角度
    double  maxH           = 1000;
    double  minH           = 50;
    double  maxV           = 36;
    double  minV           = 30;
    bool    auto_formation = false;
    double  auto_formation_dis = 0;
    double  dis_to_fly     = 1000;
};

enum class SwarmCommand : uint8_t {
    NONE             = 0x00,
    UPLOAD_WP        = 0x02,
    START_FORMATION  = 0x03,
    PAUSE_FORMATION  = 0x04,
    STOP_ALL         = 0x05,
    LEADER_WP_CMD    = 0x06,
    SET_PARAMS       = 0x07,
    START_SIMULATION = 0x0A,
    END_SIMULATION   = 0x0B,
    SET_CTRL_HZ      = 0x0C,
    START_DLH        = 0x0D,
    END_DLH          = 0x0E,
};

// ArduCopter 飞行模式 (custom_mode 值)
enum class CopterMode : uint32_t {
    STABILIZE    = 0,
    ACRO         = 1,
    ALT_HOLD     = 2,
    AUTO         = 3,
    GUIDED       = 4,
    LOITER       = 5,
    RTL          = 6,
    CIRCLE       = 7,
    LAND         = 9,
    DRIFT        = 11,
    SPORT        = 13,
    POSHOLD      = 16,
    BRAKE        = 17,
    GUIDED_NOGPS = 20,
    SMART_RTL    = 21,
};

// ArduPlane 飞行模式
enum class PlaneMode : uint32_t {
    MANUAL    = 0,
    CIRCLE    = 1,
    STABILIZE = 2,
    FLY_BY_WIRE_A = 5,
    FLY_BY_WIRE_B = 6,
    AUTO      = 10,
    RTL       = 11,
    LOITER    = 12,
    GUIDED    = 15,
    QLOITER   = 19,
    QLAND     = 20,
    QRTL      = 21,
    QHOVER    = 18,
};

enum class VehicleType : uint8_t {
    COPTER = 0,
    PLANE  = 1,
};

// SET_POSITION_TARGET 的 type_mask 常量
// 参考: https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
namespace TypeMask {
    // 使用位置 (忽略速度/加速度/偏航率)
    static constexpr uint16_t USE_POSITION      = 0b0000111111111000;  // 0x0FF8
    // 使用速度 (忽略位置/加速度)
    static constexpr uint16_t USE_VELOCITY       = 0b0000111111000111;  // 0x0FC7
    // 使用位置+速度
    static constexpr uint16_t USE_POS_AND_VEL    = 0b0000111111000000;  // 0x0FC0
    // 使用位置+偏航角
    static constexpr uint16_t USE_POS_YAW        = 0b0000101111111000;  // 0x0BF8
    // 使用速度+偏航角
    static constexpr uint16_t USE_VEL_YAW        = 0b0000101111000111;  // 0x0BC7
}

// GCS -> 群控节点 (伴机) 的标准 MAVLink 命令映射
// 使用 COMMAND_LONG，command 字段与 MAV_CMD_USER_1..5 一致（见 common.xml，勿直接包含 mavlink 头以免破坏包含顺序）。
namespace SwarmMavCmd {
    static constexpr uint16_t START_FORMATION = 31010;  // MAV_CMD_USER_1
    static constexpr uint16_t STOP_FORMATION  = 31011;  // MAV_CMD_USER_2
    static constexpr uint16_t RTL             = 31012;  // MAV_CMD_USER_3
    static constexpr uint16_t LAND            = 31013;  // MAV_CMD_USER_4
    static constexpr uint16_t ARM_TAKEOFF     = 31014;  // MAV_CMD_USER_5, param1=alt(m)
    static constexpr uint16_t SET_MODE        = 31015;  // param1: 1=FORMATION, 2=INDEPENDENT
    static constexpr uint16_t START_EXEC      = 31016;  // 按当前模式启动任务
    static constexpr uint16_t STOP_EXEC       = 31017;  // 停止当前模式任务

    // 机间任务下发协议（Leader <-> Follower）
    static constexpr uint16_t TASK_BEGIN      = 31030;  // p1=version, p2=count
    static constexpr uint16_t TASK_COMMIT     = 31031;  // p1=version
    static constexpr uint16_t TASK_ACK        = 31032;  // p1=version, p2=result(0=ok)
    static constexpr uint16_t TASK_START_AUTO = 31033;  // p1=version

    /** 机间 CTRL_CMD：GCS 标准指令经长机代理到僚机（COMMAND_LONG wire，param1=CtrlProxyOp） */
    static constexpr uint16_t CTRL_PROXY = 31034;
}

#endif
