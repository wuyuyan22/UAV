#ifndef SWARM_MAVLINK_VEHICLE_H
#define SWARM_MAVLINK_VEHICLE_H

#include "common/types.h"
#include "hal/serial_port.h"
#include "hal/udp_socket.h"

/* 单入口包含 ardupilotmega 方言（内含 common 消息），勿再直接包含 common/mavlink.h */
#include <ardupilotmega/mavlink.h>

#include <mutex>
#include <atomic>
#include <functional>
#include <cstdint>
#include <string>
#include <vector>
#include <chrono>

enum class LinkType { SERIAL, UDP };

struct VehicleConfig {
    int      sysid   = 1;
    int      compid  = 1;
    LinkType link    = LinkType::SERIAL;
    std::string serial_dev  = "/dev/ttyS8";      // 鲁班猫0 UART8 (40pin: TX=8, RX=10)
    int         serial_baud = 57600;              // 与飞控 SERIAL1_BAUD 一致
    std::string udp_ip      = "127.0.0.1";       // SITL 地址
    int         udp_port    = 14550;
    VehicleType vehicle_type = VehicleType::COPTER;

    // 飞控链路解析出完整 MAVLink 帧后可选转发到 GCS（Mission Planner）：重打包后写出，便于节流/过滤/合并僚机遥测
    std::function<void(const mavlink_message_t &msg)> gcs_parsed_forward;

    // 连接成功后 REQUEST_DATA_STREAM 请求速率 (Hz)。与 MP 共用串口桥接时应 <=4，避免占满带宽导致参数下载失败
    uint16_t data_stream_rate_hz = 10;
};

class MavlinkVehicle {
public:
    static constexpr uint8_t GCS_SYSID  = 255;
    static constexpr uint8_t GCS_COMPID = 190;

    explicit MavlinkVehicle(const VehicleConfig &cfg);
    ~MavlinkVehicle();

    bool connect();
    void disconnect();
    bool is_connected() const { return connected_; }

    // ---- 收包（主循环调用） ----
    void read_messages();

    // ---- 状态查询 ----
    const UnitState &state() const { return state_; }
    uint64_t last_heartbeat_ms() const { return last_hb_ms_; }

    // ---- 模式控制 ----
    int send_heartbeat();
    int set_copter_mode(CopterMode mode);
    int set_plane_mode(PlaneMode mode);
    int arm(bool do_arm);
    int takeoff(float alt_m);
    int land();
    int rtl();

    // ---- Guided 模式目标（编队核心指令） ----

    // 全球坐标位置目标 (lat/lon 度, alt 相对高度 m)
    int set_position_target_global(double lat_deg, double lon_deg,
                                   float alt_rel_m,
                                   uint16_t type_mask = TypeMask::USE_POSITION);

    // 全球坐标位置+速度 (用于更平滑的编队跟踪)
    int set_position_velocity_global(double lat_deg, double lon_deg,
                                     float alt_rel_m,
                                     float vx, float vy, float vz);

    // 本地 NED 位置目标 (m, 相对 Home 点)
    int set_position_target_local_ned(float x, float y, float z,
                                      uint16_t type_mask = TypeMask::USE_POSITION);

    // 本地 NED 速度目标 (m/s, >=2Hz 持续发送!)
    int set_velocity_target_local_ned(float vx, float vy, float vz);

    // 姿态目标 (Guided/Guided_NoGPS, 四元数+推力)
    int set_attitude_target(float roll_rad, float pitch_rad, float yaw_rad,
                            float thrust_normalized);

    // ---- 航点任务上传 (AUTO 模式) ----
    /** @return 0 成功发出 MISSION_COUNT，-1 失败 */
    int upload_mission(const std::vector<WayPoint> &wps);

    // ---- 通用 COMMAND_LONG ----
    int send_command_long(uint16_t cmd_id, float p1 = 0, float p2 = 0,
                          float p3 = 0, float p4 = 0, float p5 = 0,
                          float p6 = 0, float p7 = 0);

    // 请求指定消息流 (确保飞控持续发送遥测)
    int request_data_stream(uint8_t stream_id, uint16_t rate_hz);

    // ---- Swarm (GCS -> companion) 命令接收回调 ----
    // 通过 COMMAND_LONG 承载（建议用 MAV_CMD_USER_1..N）。
    void set_swarm_command_handler(const std::function<void(const mavlink_command_long_t &)> &cb);

    int sysid() const { return cfg_.sysid; }
    const VehicleConfig &config() const { return cfg_; }
    bool mission_uploading() const { return mission_uploading_; }
    int last_mission_ack_type() const { return last_mission_ack_type_; }

    /** 透明转发（地面站串口 -> 飞控），原始字节 */
    int write_raw_bytes(const uint8_t *buf, size_t len);

private:
    VehicleConfig cfg_;
    SerialPort    serial_;
    UdpSocket     udp_;
    std::mutex    tx_mtx_;  // 串口/UDP 发送互斥（控制线程与 GCS 转发并发写飞控链路）

    UnitState     state_;
    std::mutex    state_mtx_;

    mavlink_status_t mav_status_{};

    std::atomic<bool> connected_{false};
    uint64_t last_hb_ms_ = 0;

    int send_msg(mavlink_message_t *msg);
    int read_raw(uint8_t *buf, size_t len);

    void handle_message(const mavlink_message_t &msg);
    void handle_heartbeat(const mavlink_message_t &msg);
    void handle_global_position(const mavlink_message_t &msg);
    void handle_local_position(const mavlink_message_t &msg);
    void handle_attitude(const mavlink_message_t &msg);
    void handle_sys_status(const mavlink_message_t &msg);
    void handle_gps_raw(const mavlink_message_t &msg);
    void handle_command_ack(const mavlink_message_t &msg);
    void handle_mission_ack(const mavlink_message_t &msg);
    void handle_mission_request(const mavlink_message_t &msg);

    std::vector<WayPoint> pending_mission_;
    bool mission_uploading_ = false;
    int  last_mission_ack_type_ = -1;

    static uint64_t now_ms();

    std::function<void(const mavlink_command_long_t &)> swarm_cmd_handler_;

    void handle_command_long(const mavlink_message_t &msg);
    void send_command_ack(const mavlink_message_t &cmd_msg,
                           uint16_t cmd_id,
                           uint8_t result = MAV_RESULT_ACCEPTED);
};

#endif
