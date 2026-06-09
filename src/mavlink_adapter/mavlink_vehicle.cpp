#include "mavlink_adapter/mavlink_vehicle.h"
#include <cstdio>
#include <cstring>
#include <cmath>

MavlinkVehicle::MavlinkVehicle(const VehicleConfig &cfg) : cfg_(cfg) {}

MavlinkVehicle::~MavlinkVehicle() { disconnect(); }

uint64_t MavlinkVehicle::now_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

bool MavlinkVehicle::connect() {
    bool ok = false;
    if (cfg_.link == LinkType::SERIAL) {
        ok = serial_.open(cfg_.serial_dev, cfg_.serial_baud);
    } else {
        ok = udp_.set_target(cfg_.udp_ip, cfg_.udp_port);
        if (ok) ok = udp_.bind_local("0.0.0.0", cfg_.udp_port);
    }
    if (ok) {
        connected_ = true;
        request_data_stream(MAV_DATA_STREAM_ALL, cfg_.data_stream_rate_hz);
    }
    return ok;
}

void MavlinkVehicle::disconnect() {
    connected_ = false;
    serial_.close();
    udp_.close();
}

int MavlinkVehicle::read_raw(uint8_t *buf, size_t len) {
    if (cfg_.link == LinkType::SERIAL)
        return serial_.read_bytes(buf, len);
    return udp_.recv_bytes(buf, len, 10);
}

int MavlinkVehicle::send_msg(mavlink_message_t *msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    std::lock_guard<std::mutex> lk(tx_mtx_);
    if (cfg_.link == LinkType::SERIAL)
        return serial_.write_bytes(buf, len);
    return udp_.send_bytes(buf, len);
}

int MavlinkVehicle::write_raw_bytes(const uint8_t *buf, size_t len) {
    if (!connected_ || len == 0) return 0;
    std::lock_guard<std::mutex> lk(tx_mtx_);
    if (cfg_.link == LinkType::SERIAL)
        return serial_.write_bytes(buf, len);
    return udp_.send_bytes(buf, len);
}

// ==================== 收包 ====================

void MavlinkVehicle::read_messages() {
    uint8_t buf[512];
    int n = read_raw(buf, sizeof(buf));
    if (n <= 0) return;

    for (int i = 0; i < n; i++) {
        mavlink_message_t msg;
        if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &mav_status_)) {
            handle_message(msg);
            if (cfg_.gcs_parsed_forward)
                cfg_.gcs_parsed_forward(msg);
        }
    }
}

void MavlinkVehicle::handle_message(const mavlink_message_t &msg) {
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:            handle_heartbeat(msg);        break;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:  handle_global_position(msg);  break;
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:   handle_local_position(msg);   break;
    case MAVLINK_MSG_ID_ATTITUDE:             handle_attitude(msg);         break;
    case MAVLINK_MSG_ID_SYS_STATUS:           handle_sys_status(msg);       break;
    case MAVLINK_MSG_ID_GPS_RAW_INT:          handle_gps_raw(msg);          break;
    case MAVLINK_MSG_ID_COMMAND_ACK:          handle_command_ack(msg);      break;
    case MAVLINK_MSG_ID_COMMAND_LONG:        handle_command_long(msg);     break;
    case MAVLINK_MSG_ID_MISSION_ACK:          handle_mission_ack(msg);      break;
    case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
    case MAVLINK_MSG_ID_MISSION_REQUEST:      handle_mission_request(msg);  break;
    default: break;
    }
}

void MavlinkVehicle::set_swarm_command_handler(
    const std::function<void(const mavlink_command_long_t &)> &cb) {
    swarm_cmd_handler_ = cb;
}

void MavlinkVehicle::handle_command_long(const mavlink_message_t &msg) {
    if (!swarm_cmd_handler_) return;
    if (msg.sysid == 0) return;

    mavlink_command_long_t cl{};
    mavlink_msg_command_long_decode(&msg, &cl);

    // 仅处理发给本机 sysid 的伴机命令，避免误处理其他链路命令。
    if (cl.target_system != static_cast<uint8_t>(cfg_.sysid)) return;
    if (cl.target_component != MAV_COMP_ID_ALL &&
        cl.target_component != MAV_COMP_ID_ONBOARD_COMPUTER) return;

    swarm_cmd_handler_(cl);
    send_command_ack(msg, cl.command, MAV_RESULT_ACCEPTED);
}

void MavlinkVehicle::send_command_ack(const mavlink_message_t &cmd_msg,
                                        uint16_t cmd_id,
                                        uint8_t result) {
    mavlink_message_t ack_msg;
    mavlink_command_ack_t ack{};
    ack.command = cmd_id;
    ack.result  = result;
    ack.progress = 0;
    ack.result_param2 = 0;
    ack.target_system = cmd_msg.sysid;
    ack.target_component = cmd_msg.compid;

    mavlink_msg_command_ack_encode(static_cast<uint8_t>(cfg_.sysid),
                                   MAV_COMP_ID_ONBOARD_COMPUTER,
                                   &ack_msg, &ack);
    send_msg(&ack_msg);
}

void MavlinkVehicle::handle_heartbeat(const mavlink_message_t &msg) {
    if (msg.sysid != cfg_.sysid) return;
    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(&msg, &hb);

    std::lock_guard<std::mutex> lk(state_mtx_);
    state_.armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) ? 1 : 0;
    state_.custom_mode = hb.custom_mode;
    last_hb_ms_ = now_ms();
}

void MavlinkVehicle::handle_global_position(const mavlink_message_t &msg) {
    if (msg.sysid != cfg_.sysid) return;
    mavlink_global_position_int_t gp;
    mavlink_msg_global_position_int_decode(&msg, &gp);

    std::lock_guard<std::mutex> lk(state_mtx_);
    state_.lat     = gp.lat * 1e-7 * SWARM_DEG2RAD;
    state_.lon     = gp.lon * 1e-7 * SWARM_DEG2RAD;
    state_.alt_rel = gp.relative_alt * 0.001f;
    state_.alt_bar = gp.alt * 0.001f;
    state_.vn      = gp.vx * 0.01f;
    state_.ve      = gp.vy * 0.01f;
    state_.vd      = gp.vz * 0.01f;
    state_.heading = gp.hdg * 0.01f * SWARM_DEG2RAD;
    state_.vel_cruise = sqrtf(state_.vn * state_.vn + state_.ve * state_.ve);
}

void MavlinkVehicle::handle_local_position(const mavlink_message_t &msg) {
    if (msg.sysid != cfg_.sysid) return;
    mavlink_local_position_ned_t lp;
    mavlink_msg_local_position_ned_decode(&msg, &lp);

    std::lock_guard<std::mutex> lk(state_mtx_);
    state_.x_ned = lp.x;
    state_.y_ned = lp.y;
    state_.z_ned = lp.z;
}

void MavlinkVehicle::handle_attitude(const mavlink_message_t &msg) {
    if (msg.sysid != cfg_.sysid) return;
    mavlink_attitude_t att;
    mavlink_msg_attitude_decode(&msg, &att);

    std::lock_guard<std::mutex> lk(state_mtx_);
    state_.roll  = att.roll;
    state_.pitch = att.pitch;
    if (att.yaw >= 0)
        state_.heading = att.yaw;
}

void MavlinkVehicle::handle_sys_status(const mavlink_message_t &msg) {
    if (msg.sysid != cfg_.sysid) return;
    mavlink_sys_status_t ss;
    mavlink_msg_sys_status_decode(&msg, &ss);

    std::lock_guard<std::mutex> lk(state_mtx_);
    state_.battery_pct = ss.battery_remaining;
}

void MavlinkVehicle::handle_gps_raw(const mavlink_message_t &msg) {
    if (msg.sysid != cfg_.sysid) return;
    mavlink_gps_raw_int_t gps;
    mavlink_msg_gps_raw_int_decode(&msg, &gps);

    std::lock_guard<std::mutex> lk(state_mtx_);
    state_.gps_fix = gps.fix_type;
}

void MavlinkVehicle::handle_command_ack(const mavlink_message_t &msg) {
    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(&msg, &ack);
    const char *result_str = (ack.result == MAV_RESULT_ACCEPTED) ? "OK" : "FAIL";
    printf("[vehicle %d] CMD_ACK cmd=%d result=%s(%d)\n",
           cfg_.sysid, ack.command, result_str, ack.result);
}

void MavlinkVehicle::handle_mission_ack(const mavlink_message_t &msg) {
    mavlink_mission_ack_t ack;
    mavlink_msg_mission_ack_decode(&msg, &ack);
    printf("[vehicle %d] MISSION_ACK type=%d\n", cfg_.sysid, ack.type);
    last_mission_ack_type_ = ack.type;
    mission_uploading_ = false;
    // 避免内部上传结束后 pending 仍保留，误在 MP 上传时向飞控注入 MISSION_ITEM
    pending_mission_.clear();
}

void MavlinkVehicle::handle_mission_request(const mavlink_message_t &msg) {
    // 仅响应本机 upload_mission() 触发的协议；MP 经伴机透传上传时勿注入，否则与 GCS 抢答导致
    // MISSION_OPERATION_CANCELLED / MissionPlanner setWPTotal 超时。
    if (!mission_uploading_) return;

    uint16_t seq = 0;
    if (msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST_INT) {
        mavlink_mission_request_int_t req;
        mavlink_msg_mission_request_int_decode(&msg, &req);
        seq = req.seq;
    } else {
        mavlink_mission_request_t req;
        mavlink_msg_mission_request_decode(&msg, &req);
        seq = req.seq;
    }

    if (seq >= pending_mission_.size()) return;

    const WayPoint &wp = pending_mission_[seq];
    mavlink_message_t out;
    mavlink_mission_item_int_t item{};
    item.target_system    = cfg_.sysid;
    item.target_component = cfg_.compid;
    item.seq              = seq;
    item.frame            = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    item.command           = MAV_CMD_NAV_WAYPOINT;
    item.current          = (seq == 0) ? 1 : 0;
    item.autocontinue     = 1;
    item.x                = static_cast<int32_t>(wp.lat * SWARM_RAD2DEG * 1e7);
    item.y                = static_cast<int32_t>(wp.lon * SWARM_RAD2DEG * 1e7);
    item.z                = static_cast<float>(wp.alt);
    item.mission_type     = MAV_MISSION_TYPE_MISSION;

    mavlink_msg_mission_item_int_encode(GCS_SYSID, GCS_COMPID, &out, &item);
    send_msg(&out);
    printf("[vehicle %d] sent MISSION_ITEM seq=%d\n", cfg_.sysid, seq);
}

// ==================== 命令接口 ====================

int MavlinkVehicle::send_heartbeat() {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(GCS_SYSID, GCS_COMPID, &msg,
        MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 0);
    return send_msg(&msg);
}

int MavlinkVehicle::set_copter_mode(CopterMode mode) {
    return send_command_long(MAV_CMD_DO_SET_MODE,
                             MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                             static_cast<float>(static_cast<uint32_t>(mode)));
}

int MavlinkVehicle::set_plane_mode(PlaneMode mode) {
    return send_command_long(MAV_CMD_DO_SET_MODE,
                             MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                             static_cast<float>(static_cast<uint32_t>(mode)));
}

int MavlinkVehicle::arm(bool do_arm) {
    return send_command_long(MAV_CMD_COMPONENT_ARM_DISARM,
                             do_arm ? 1.0f : 0.0f);
}

int MavlinkVehicle::takeoff(float alt_m) {
    return send_command_long(MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt_m);
}

int MavlinkVehicle::land() {
    return send_command_long(MAV_CMD_NAV_LAND);
}

int MavlinkVehicle::rtl() {
    if (cfg_.vehicle_type == VehicleType::COPTER)
        return set_copter_mode(CopterMode::RTL);
    else
        return set_plane_mode(PlaneMode::RTL);
}

// ==================== Guided 位置/速度/姿态目标 ====================

int MavlinkVehicle::set_position_target_global(
    double lat_deg, double lon_deg, float alt_rel_m, uint16_t type_mask)
{
    mavlink_message_t msg;
    mavlink_set_position_target_global_int_t sp{};
    sp.time_boot_ms     = 0;
    sp.target_system    = cfg_.sysid;
    sp.target_component = cfg_.compid;
    sp.coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    sp.type_mask        = type_mask;
    sp.lat_int          = static_cast<int32_t>(lat_deg * 1e7);
    sp.lon_int          = static_cast<int32_t>(lon_deg * 1e7);
    sp.alt              = alt_rel_m;

    mavlink_msg_set_position_target_global_int_encode(
        GCS_SYSID, GCS_COMPID, &msg, &sp);
    return send_msg(&msg);
}

int MavlinkVehicle::set_position_velocity_global(
    double lat_deg, double lon_deg, float alt_rel_m,
    float vx, float vy, float vz)
{
    mavlink_message_t msg;
    mavlink_set_position_target_global_int_t sp{};
    sp.time_boot_ms     = 0;
    sp.target_system    = cfg_.sysid;
    sp.target_component = cfg_.compid;
    sp.coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    sp.type_mask        = TypeMask::USE_POS_AND_VEL;
    sp.lat_int          = static_cast<int32_t>(lat_deg * 1e7);
    sp.lon_int          = static_cast<int32_t>(lon_deg * 1e7);
    sp.alt              = alt_rel_m;
    sp.vx               = vx;
    sp.vy               = vy;
    sp.vz               = vz;

    mavlink_msg_set_position_target_global_int_encode(
        GCS_SYSID, GCS_COMPID, &msg, &sp);
    return send_msg(&msg);
}

int MavlinkVehicle::set_position_target_local_ned(
    float x, float y, float z, uint16_t type_mask)
{
    mavlink_message_t msg;
    mavlink_set_position_target_local_ned_t sp{};
    sp.time_boot_ms     = 0;
    sp.target_system    = cfg_.sysid;
    sp.target_component = cfg_.compid;
    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
    sp.type_mask        = type_mask;
    sp.x                = x;
    sp.y                = y;
    sp.z                = z;   // NED: 下方为正，所以 -alt

    mavlink_msg_set_position_target_local_ned_encode(
        GCS_SYSID, GCS_COMPID, &msg, &sp);
    return send_msg(&msg);
}

int MavlinkVehicle::set_velocity_target_local_ned(float vx, float vy, float vz) {
    mavlink_message_t msg;
    mavlink_set_position_target_local_ned_t sp{};
    sp.time_boot_ms     = 0;
    sp.target_system    = cfg_.sysid;
    sp.target_component = cfg_.compid;
    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
    sp.type_mask        = TypeMask::USE_VELOCITY;
    sp.vx               = vx;
    sp.vy               = vy;
    sp.vz               = vz;

    mavlink_msg_set_position_target_local_ned_encode(
        GCS_SYSID, GCS_COMPID, &msg, &sp);
    return send_msg(&msg);
}

int MavlinkVehicle::set_attitude_target(
    float roll_rad, float pitch_rad, float yaw_rad, float thrust_normalized)
{
    mavlink_message_t msg;
    mavlink_set_attitude_target_t at{};
    at.time_boot_ms     = 0;
    at.target_system    = cfg_.sysid;
    at.target_component = cfg_.compid;
    at.type_mask        = 0b00000111;  // 忽略 body roll/pitch/yaw rate

    // 欧拉角转四元数 (ZYX 约定)
    float cy = cosf(yaw_rad * 0.5f);
    float sy = sinf(yaw_rad * 0.5f);
    float cp = cosf(pitch_rad * 0.5f);
    float sp = sinf(pitch_rad * 0.5f);
    float cr = cosf(roll_rad * 0.5f);
    float sr = sinf(roll_rad * 0.5f);

    at.q[0] = cr * cp * cy + sr * sp * sy;
    at.q[1] = sr * cp * cy - cr * sp * sy;
    at.q[2] = cr * sp * cy + sr * cp * sy;
    at.q[3] = cr * cp * sy - sr * sp * cy;

    at.thrust = thrust_normalized;  // 0.0~1.0 (需设 GUID_OPTIONS bit3=8)

    mavlink_msg_set_attitude_target_encode(
        GCS_SYSID, GCS_COMPID, &msg, &at);
    return send_msg(&msg);
}

// ==================== 任务上传 ====================

int MavlinkVehicle::upload_mission(const std::vector<WayPoint> &wps) {
    if (wps.empty()) return -1;
    pending_mission_ = wps;
    mission_uploading_ = true;
    last_mission_ack_type_ = -1;

    mavlink_message_t msg;
    mavlink_mission_count_t mc{};
    mc.target_system    = cfg_.sysid;
    mc.target_component = cfg_.compid;
    mc.count            = static_cast<uint16_t>(wps.size());
    mavlink_msg_mission_count_encode(GCS_SYSID, GCS_COMPID, &msg, &mc);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t need = mavlink_msg_to_send_buffer(buf, &msg);
    const int sent = send_msg(&msg);
    if (sent < 0 || static_cast<uint16_t>(sent) != need) {
        mission_uploading_ = false;
        pending_mission_.clear();
        return -1;
    }
    return 0;
}

// ==================== 通用命令 ====================

int MavlinkVehicle::send_command_long(uint16_t cmd_id,
    float p1, float p2, float p3, float p4,
    float p5, float p6, float p7)
{
    mavlink_message_t msg;
    mavlink_command_long_t cmd{};
    cmd.target_system    = cfg_.sysid;
    cmd.target_component = cfg_.compid;
    cmd.command          = cmd_id;
    cmd.confirmation     = 0;
    cmd.param1 = p1; cmd.param2 = p2; cmd.param3 = p3; cmd.param4 = p4;
    cmd.param5 = p5; cmd.param6 = p6; cmd.param7 = p7;
    mavlink_msg_command_long_encode(GCS_SYSID, GCS_COMPID, &msg, &cmd);
    return send_msg(&msg);
}

int MavlinkVehicle::request_data_stream(uint8_t stream_id, uint16_t rate_hz) {
    mavlink_message_t msg;
    mavlink_request_data_stream_t rds{};
    rds.target_system    = cfg_.sysid;
    rds.target_component = cfg_.compid;
    rds.req_stream_id    = stream_id;
    rds.req_message_rate = rate_hz;
    rds.start_stop       = 1;
    mavlink_msg_request_data_stream_encode(GCS_SYSID, GCS_COMPID, &msg, &rds);
    return send_msg(&msg);
}
