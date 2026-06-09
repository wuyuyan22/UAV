#include "swarm/swarm_controller.h"
#include "comm/gcs_ip_link.h"
#include "comm/gcs_serial_link.h"
#include "comm/gcs_p900_link.h"
#include "common/math_utils.h"
#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <ctime>
#include <utility>

namespace {

WayPoint mission_item_int_to_waypoint(const mavlink_mission_item_int_t &it) {
    WayPoint wp;
    if (it.x == INT32_MAX || it.y == INT32_MAX) {
        wp.lat = 0;
        wp.lon = 0;
    } else {
        wp.lat = static_cast<double>(it.x) * 1e-7 * SWARM_DEG2RAD;
        wp.lon = static_cast<double>(it.y) * 1e-7 * SWARM_DEG2RAD;
    }
    wp.alt  = it.z;
    wp.vel  = 0;
    wp.hdg  = 0;
    wp.mode = 2;
    wp.radius = 0;
    return wp;
}

}  // namespace

// GcsSerialLink / GcsP900Link 已抽到 include/comm/，此处不再内联实现。

/** 机间 CTRL_CMD 语义（COMMAND_LONG.param1 / LinkPacket.seq） */
enum CtrlProxyOp : uint32_t {
    PROXY_ARM_DISARM = 1,
    PROXY_NAV_TAKEOFF = 2,
    PROXY_NAV_LAND = 3,
    PROXY_NAV_RTL = 4,
    PROXY_DO_SET_MODE = 5,
};

static bool gcs_ctrl_target_component_ok(uint8_t comp) {
    return comp == MAV_COMP_ID_ALL || comp == MAV_COMP_ID_AUTOPILOT1 ||
           comp == MAV_COMP_ID_ONBOARD_COMPUTER;
}

static uint64_t steady_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

static void format_wallclock_ms(char *buf, size_t len) {
    using namespace std::chrono;
    const auto now = system_clock::now();
    const auto sec_tp = time_point_cast<seconds>(now);
    const auto ms = duration_cast<milliseconds>(now - sec_tp).count();
    const std::time_t tt = system_clock::to_time_t(now);
    std::tm tm_now{};
    localtime_r(&tt, &tm_now);
    std::snprintf(buf, len, "%04d-%02d-%02d %02d:%02d:%02d.%03lld",
                  tm_now.tm_year + 1900, tm_now.tm_mon + 1, tm_now.tm_mday,
                  tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec,
                  static_cast<long long>(ms));
}

static void log_cmd_info(const char *fmt, ...) {
    char ts[32];
    format_wallclock_ms(ts, sizeof(ts));
    std::printf("[%s] ", ts);
    va_list args;
    va_start(args, fmt);
    std::vprintf(fmt, args);
    va_end(args);
}

static void log_cmd_error(const char *fmt, ...) {
    char ts[32];
    format_wallclock_ms(ts, sizeof(ts));
    std::fprintf(stderr, "[%s] ", ts);
    va_list args;
    va_start(args, fmt);
    std::vfprintf(stderr, fmt, args);
    va_end(args);
}

static bool should_forward_gcs_msg_to_local_fc(const mavlink_message_t &msg, int local_sysid) {
    const uint8_t local = static_cast<uint8_t>(local_sysid);
    const auto match_or_broadcast = [local](uint8_t target_sys) -> bool {
        return target_sys == 0 || target_sys == local;
    };

    switch (msg.msgid) {
    case MAVLINK_MSG_ID_COMMAND_LONG: {
        mavlink_command_long_t cl{};
        mavlink_msg_command_long_decode(&msg, &cl);
        // 群控命令由伴机本地处理并应答，不透传给飞控，避免双通道 ACK 混淆。
        const bool is_swarm_cmd =
            (cl.command >= SwarmMavCmd::START_FORMATION &&
             cl.command <= SwarmMavCmd::STOP_EXEC);
        if (is_swarm_cmd &&
            (cl.target_component == MAV_COMP_ID_ALL ||
             cl.target_component == MAV_COMP_ID_ONBOARD_COMPUTER)) {
            return false;
        }
        return match_or_broadcast(cl.target_system);
    }
    case MAVLINK_MSG_ID_COMMAND_INT: {
        mavlink_command_int_t ci{};
        mavlink_msg_command_int_decode(&msg, &ci);
        return match_or_broadcast(ci.target_system);
    }
    case MAVLINK_MSG_ID_SET_MODE: {
        mavlink_set_mode_t sm{};
        mavlink_msg_set_mode_decode(&msg, &sm);
        return match_or_broadcast(sm.target_system);
    }
    case MAVLINK_MSG_ID_MANUAL_CONTROL: {
        mavlink_manual_control_t mc{};
        mavlink_msg_manual_control_decode(&msg, &mc);
        return match_or_broadcast(mc.target);
    }
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
        mavlink_param_request_list_t pr{};
        mavlink_msg_param_request_list_decode(&msg, &pr);
        return match_or_broadcast(pr.target_system);
    }
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
        mavlink_param_request_read_t pr{};
        mavlink_msg_param_request_read_decode(&msg, &pr);
        return match_or_broadcast(pr.target_system);
    }
    case MAVLINK_MSG_ID_PARAM_SET: {
        mavlink_param_set_t ps{};
        mavlink_msg_param_set_decode(&msg, &ps);
        return match_or_broadcast(ps.target_system);
    }
    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST: {
        mavlink_param_ext_request_list_t pr{};
        mavlink_msg_param_ext_request_list_decode(&msg, &pr);
        return match_or_broadcast(pr.target_system);
    }
    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ: {
        mavlink_param_ext_request_read_t pr{};
        mavlink_msg_param_ext_request_read_decode(&msg, &pr);
        return match_or_broadcast(pr.target_system);
    }
    case MAVLINK_MSG_ID_PARAM_EXT_SET: {
        mavlink_param_ext_set_t ps{};
        mavlink_msg_param_ext_set_decode(&msg, &ps);
        return match_or_broadcast(ps.target_system);
    }
    case MAVLINK_MSG_ID_MISSION_COUNT: {
        mavlink_mission_count_t mc{};
        mavlink_msg_mission_count_decode(&msg, &mc);
        return match_or_broadcast(mc.target_system);
    }
    case MAVLINK_MSG_ID_MISSION_ITEM_INT: {
        mavlink_mission_item_int_t mi{};
        mavlink_msg_mission_item_int_decode(&msg, &mi);
        return match_or_broadcast(mi.target_system);
    }
    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL: {
        mavlink_mission_clear_all_t mca{};
        mavlink_msg_mission_clear_all_decode(&msg, &mca);
        return match_or_broadcast(mca.target_system);
    }
    case MAVLINK_MSG_ID_MISSION_SET_CURRENT: {
        mavlink_mission_set_current_t msc{};
        mavlink_msg_mission_set_current_decode(&msg, &msc);
        return match_or_broadcast(msc.target_system);
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
        mavlink_mission_request_list_t mrl{};
        mavlink_msg_mission_request_list_decode(&msg, &mrl);
        return match_or_broadcast(mrl.target_system);
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST: {
        mavlink_mission_request_t mr{};
        mavlink_msg_mission_request_decode(&msg, &mr);
        return match_or_broadcast(mr.target_system);
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST_INT: {
        mavlink_mission_request_int_t mr{};
        mavlink_msg_mission_request_int_decode(&msg, &mr);
        return match_or_broadcast(mr.target_system);
    }
    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST: {
        mavlink_mission_write_partial_list_t mwp{};
        mavlink_msg_mission_write_partial_list_decode(&msg, &mwp);
        return match_or_broadcast(mwp.target_system);
    }
    default:
        // 无 target_system 字段的消息（如 HEARTBEAT/TIMESYNC）默认保留透传，兼容 GCS 常规链路维护。
        return true;
    }
}

static bool should_drop_fc_msg_to_gcs(const mavlink_message_t &msg) {
    switch (msg.msgid) {
    // 与 ATTITUDE 重叠，先剔除以降低带宽占用
    case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
    // 常规 MP 监看非必需
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
    // 传感器/调试高频流
    case MAVLINK_MSG_ID_RAW_IMU:
    case MAVLINK_MSG_ID_SCALED_IMU:
    case MAVLINK_MSG_ID_SCALED_IMU2:
    case MAVLINK_MSG_ID_SCALED_IMU3:
    case MAVLINK_MSG_ID_HIGHRES_IMU:
    case MAVLINK_MSG_ID_VIBRATION:
    case MAVLINK_MSG_ID_AHRS2:
    case MAVLINK_MSG_ID_AHRS3:
    // ESC 遥测（高频）；BATTERY_STATUS 需转发至 GCS 以显示单节电压等
    case MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4:
    case MAVLINK_MSG_ID_ESC_TELEMETRY_5_TO_8:
    case MAVLINK_MSG_ID_ESC_TELEMETRY_9_TO_12:
    case MAVLINK_MSG_ID_ESC_TELEMETRY_13_TO_16:
    case MAVLINK_MSG_ID_ESC_STATUS:
    // 调试与命名值
    case MAVLINK_MSG_ID_NAMED_VALUE_INT:
    case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
    case MAVLINK_MSG_ID_DEBUG:
    case MAVLINK_MSG_ID_DEBUG_VECT:
    case MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY:
    case MAVLINK_MSG_ID_MEMINFO:
        return true;
    default:
        return false;
    }
}

SwarmController::SwarmController(const SwarmConfig &cfg)
    : cfg_(cfg) {}

SwarmController::~SwarmController() { stop(); }

void SwarmController::timing_record(TimingCounter &c, uint64_t us, bool ok, uint64_t bytes) {
    c.count++;
    if (!ok) c.fail_count++;
    c.total_us += us;
    c.bytes += bytes;
    if (c.min_us == 0 || us < c.min_us) c.min_us = us;
    if (us > c.max_us) c.max_us = us;
}

void SwarmController::maybe_print_leader_timing_stats() {
    // Keep collecting timing counters for future diagnostics,
    // but suppress periodic leader timing logs in normal operation.
    (void)cfg_;
}

bool SwarmController::init() {
    // 控制频率检查：Guided 模式要求 >=2Hz，否则 GUID_TIMEOUT 触发悬停
    if (cfg_.ctrl_hz < 2) {
        fprintf(stderr, "[swarm] WARNING: ctrl_hz=%d < 2Hz, Guided mode may timeout! "
                "Clamping to 2Hz.\n", cfg_.ctrl_hz);
        cfg_.ctrl_hz = 2;
    }

    // GCS：在 add_vehicle 之前注册 FC->GCS 转发回调
    if (cfg_.gcs_serial_enable) {
        if (cfg_.local_fc.data_stream_rate_hz > 4)
            cfg_.local_fc.data_stream_rate_hz = 4;
        cfg_.local_fc.gcs_parsed_forward = [this](const mavlink_message_t &msg) {
            forward_fc_parsed_to_gcs(msg);
        };
        build_gcs_link_();

        if (gcs_link_ && !gcs_link_->open()) {
            fprintf(stderr, "[swarm] GCS link open failed\n");
            return false;
        }
        if (gcs_link_) {
            printf("[swarm] GCS link enabled: type=%d, data_stream=%uHz\n",
                   static_cast<int>(cfg_.gcs_link_type), cfg_.local_fc.data_stream_rate_hz);
        }
    }

    if (cfg_.gcs_serial_enable) {
        // 令牌桶：IP 主链按 gcs_tx_max_kbps 直接配置；串口/P900 兜底沿用波特率×70% 估算。
        if (cfg_.gcs_link_type == SwarmConfig::GcsLinkType::IP) {
            gcs_tx_token_rate_bps_ = std::max<uint32_t>(128, cfg_.gcs_tx_max_kbps * 1000u);
        } else {
            const uint32_t baud = static_cast<uint32_t>(std::max(1200, cfg_.gcs_serial_baud));
            gcs_tx_token_rate_bps_ = std::max<uint32_t>(128, (baud / 10) * 7 / 10);
        }
        gcs_tx_token_capacity_ = std::max<uint32_t>(gcs_tx_token_rate_bps_ / 2, 512);
        gcs_tx_tokens_ = gcs_tx_token_capacity_;
        gcs_tx_tokens_last_refill_ms_ = steady_ms();
    }

    // 5G 主链路健康监视：仅 IP 模式启用
    if (cfg_.health_enable && cfg_.gcs_link_type == SwarmConfig::GcsLinkType::IP
        && gcs_ip_link_ptr_ != nullptr) {
        HealthMonitor::Config hc{};
        hc.recv_warn_ms = cfg_.health_warn_ms;
        hc.recv_lost_ms = cfg_.health_lost_ms;
        health_monitor_.set_config(hc);
        health_monitor_.set_link(gcs_ip_link_ptr_);
        health_monitor_.set_status_emitter(
            [this](const std::string &text, uint8_t sev) {
                send_health_statustext_(text, sev);
            });
        health_monitor_.set_state_callback(
            [this](HealthMonitor::State to, HealthMonitor::State from,
                   const std::string &reason) {
                on_health_state_changed_(static_cast<uint8_t>(to),
                                         static_cast<uint8_t>(from), reason);
            });
        health_monitor_.start(steady_ms());
        printf("[swarm] 5G health monitor enabled: warn=%llums lost=%llums fallback=%s\n",
               static_cast<unsigned long long>(cfg_.health_warn_ms),
               static_cast<unsigned long long>(cfg_.health_lost_ms),
               cfg_.p900_uart9_fallback ? "yes" : "no");
    }

    if (!mav_mgr_.add_vehicle(cfg_.local_fc)) {
        fprintf(stderr, "[swarm] local FC connect failed\n");
        if (gcs_link_) gcs_link_->close();
        link_p900_.close();
        return false;
    }

    // 订阅来自 GCS 的 Swarm 控制命令（COMMAND_LONG + MAV_CMD_USER_xx）
    if (auto *v = mav_mgr_.get_vehicle(cfg_.local_fc.sysid)) {
        v->set_swarm_command_handler([this](const mavlink_command_long_t &cl) {
            switch (cl.command) {
            case SwarmMavCmd::START_FORMATION:
                this->cmd_start_formation();
                break;
            case SwarmMavCmd::STOP_FORMATION:
                this->cmd_stop_formation();
                break;
            case SwarmMavCmd::RTL:
                this->cmd_rtl();
                break;
            case SwarmMavCmd::LAND:
                this->cmd_land();
                break;
            case SwarmMavCmd::ARM_TAKEOFF: {
                // param1: takeoff altitude (m)
                float alt = cl.param1;
                this->cmd_arm_and_takeoff(alt);
                break;
            }
            case SwarmMavCmd::SET_MODE:
                this->set_exec_mode(cl.param1 == 2.0f ? ExecMode::INDEPENDENT
                                                       : ExecMode::FORMATION);
                break;
            case SwarmMavCmd::START_EXEC:
                this->start_exec();
                break;
            case SwarmMavCmd::STOP_EXEC:
                this->stop_exec();
                break;
            default:
                break;
            }
        });
    }

    // P900 启动期开启条件：
    //   1) --p900-uart9 显式启用复用模式（air + leader-GCS 都走 P900）；
    //   2) --gcs-link p900 单独把 GCS 通道挂到 P900 串口（air 仍可走 UDP）。
    const bool p900_open_needed = cfg_.p900_uart9_mode
        || cfg_.gcs_link_type == SwarmConfig::GcsLinkType::P900
        || cfg_.gcs_link_type == SwarmConfig::GcsLinkType::LEGACY_P900_LEADER;
    if (p900_open_needed) {
        if (!link_p900_.open(cfg_.p900_uart9_dev, cfg_.p900_uart9_baud)) {
            fprintf(stderr, "[swarm] P900 UART open failed: %s@%d\n",
                    cfg_.p900_uart9_dev.c_str(), cfg_.p900_uart9_baud);
            if (gcs_link_) gcs_link_->close();
            return false;
        }
        link_p900_.set_local_identity(static_cast<uint8_t>(cfg_.local_sysid));
    }

    if (cfg_.p900_uart9_mode) {
        air_link_ = &link_p900_;
        printf("[swarm] air link: P900 framed %s@%d\n",
               cfg_.p900_uart9_dev.c_str(), cfg_.p900_uart9_baud);
    } else {
        if (cfg_.role == NodeRole::LEADER) {
            if (!link_udp_.bind(cfg_.link_listen_ip, cfg_.link_listen_port))
                fprintf(stderr, "[swarm] UDP link bind failed\n");
        } else {
            link_udp_.set_target(cfg_.link_target_ip, cfg_.link_target_port);
            if (!link_udp_.bind(cfg_.link_listen_ip, cfg_.link_listen_port))
                fprintf(stderr, "[swarm] UDP link bind failed\n");
        }
        link_udp_.set_local_identity(static_cast<uint8_t>(cfg_.local_sysid));
        air_link_ = &link_udp_;
        printf("[swarm] air link: UDP port %d\n", cfg_.link_listen_port);
    }

    printf("[swarm] init ok, sysid=%d role=%s type=%s FC=%s@%d\n",
           cfg_.local_sysid,
           cfg_.role == NodeRole::LEADER ? "LEADER" : "FOLLOWER",
           cfg_.vehicle_type == VehicleType::COPTER ? "COPTER" : "PLANE",
           cfg_.local_fc.serial_dev.c_str(), cfg_.local_fc.serial_baud);
    return true;
}

void SwarmController::start() {
    running_ = true;
    ctrl_thread_ = std::thread(&SwarmController::control_loop, this);
    recv_thread_ = std::thread(&SwarmController::receive_loop, this);
}

void SwarmController::stop() {
    running_ = false;
    formation_active_ = false;
    if (ctrl_thread_.joinable()) ctrl_thread_.join();
    if (recv_thread_.joinable()) recv_thread_.join();
    if (gcs_link_) gcs_link_->close();
    link_p900_.close();
    std::lock_guard<std::mutex> lk(gcs_tx_queue_mtx_);
    gcs_tx_queue_high_.clear();
    gcs_tx_queue_normal_.clear();
}

const UnitState &SwarmController::local_state() const {
    auto *v = const_cast<MavlinkManager&>(mav_mgr_).get_vehicle(cfg_.local_fc.sysid);
    return v->state();
}

// ==================== 收包线程 ====================

void SwarmController::receive_loop() {
    while (running_) {
        if (cfg_.p900_uart9_mode || p900_fallback_engaged_.load(std::memory_order_relaxed))
            link_p900_.rx_pump();

        // 先处理 MP 下行，避免遥测读占满线程导致 PARAM_REQUEST 等无法及时转发到飞控
        if (cfg_.gcs_serial_enable)
            poll_gcs_link();

        // GCS 链路心跳/保活与健康监视
        const uint64_t now_loop_ms = steady_ms();
        if (gcs_link_) gcs_link_->on_tick(now_loop_ms);
        if (cfg_.health_enable && gcs_ip_link_ptr_ != nullptr)
            health_monitor_.tick(now_loop_ms);

        for (int k = 0; k < 6; k++)
            mav_mgr_.read_all();

        if (cfg_.role == NodeRole::FOLLOWER) {
            LinkPacket pkt;
            while (air_link_->recv_packet(pkt)) {
                if (pkt.src_id == static_cast<uint8_t>(cfg_.leader_sysid))
                    follower_last_rx_from_leader_ms_.store(steady_ms(),
                                                            std::memory_order_relaxed);
                if (pkt.msg_type == LinkMsgType::CTRL_CMD) {
                    if (pkt.dst_id == static_cast<uint8_t>(cfg_.local_fc.sysid))
                        execute_follower_ctrl_cmd(pkt);
                    continue;
                }
                if (pkt.msg_type == LinkMsgType::FOLLOWER_CMD) {
                    // 只响应发给本机的目标点
                    if (pkt.dst_id != cfg_.local_fc.sysid) continue;
                    auto *fc = mav_mgr_.get_vehicle(cfg_.local_fc.sysid);
                    if (fc && formation_active_) {
                        double lat_deg = pkt.lat * SWARM_RAD2DEG;
                        double lon_deg = pkt.lon * SWARM_RAD2DEG;
                        fc->set_position_target_global(lat_deg, lon_deg, pkt.alt);
                        last_target_sent_ms_.store(steady_ms(), std::memory_order_relaxed);
                    }
                    continue;
                }
                handle_follower_task_packets(pkt);
            }
        }

        if (cfg_.role == NodeRole::LEADER) {
            LinkPacket pkt;
            while (air_link_->recv_packet(pkt)) {
                const auto t_pkt0 = std::chrono::steady_clock::now();
                if (pkt.msg_type == LinkMsgType::STATE_REPORT) {
                    // 记录 follower 的位置，用于编队距离/速度自适应
                    std::lock_guard<std::mutex> lk(follower_state_mtx_);
                    uint64_t &cnt = follower_link_rx_total_[pkt.src_id];
                    const bool first_sr = (cnt == 0);
                    cnt++;
                    follower_link_last_rx_ms_[pkt.src_id] = steady_ms();
                    auto &st = follower_state_cache_[pkt.src_id];
                    st.id = pkt.src_id;
                    st.lat = pkt.lat;
                    st.lon = pkt.lon;
                    st.alt_rel = pkt.alt;
                    st.vel_cruise = pkt.vel;
                    st.heading = pkt.hdg;
                    st.health = pkt.health;
                    st.armed = pkt.armed;
                    st.custom_mode = pkt.custom_mode;
                    st.custom_mode_valid = pkt.mode_valid;
                    (void)first_sr;
                }
                handle_follower_task_packets(pkt);
                const uint64_t pkt_us = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - t_pkt0).count());
                std::lock_guard<std::mutex> lk(timing_mtx_);
                timing_record(leader_timing_.air_recv_handle, pkt_us, true, sizeof(LinkPacket));
            }
        }

        maybe_print_leader_timing_stats();
        flush_gcs_tx_queue(16, 2048);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void SwarmController::mark_gcs_param_fetch_from_gcs() {
    gcs_param_fetch_last_ms_.store(steady_ms(), std::memory_order_relaxed);
}

bool SwarmController::gcs_param_fetch_active() const {
    if (!cfg_.gcs_serial_enable) return false;
    const uint64_t last = gcs_param_fetch_last_ms_.load(std::memory_order_relaxed);
    if (last == 0) return false;
    constexpr uint64_t kGraceMs = 4000;
    return (steady_ms() - last) < kGraceMs;
}

bool SwarmController::follower_sysid_contains(int sysid) const {
    const uint8_t want = static_cast<uint8_t>(sysid);
    for (int f : cfg_.follower_sysids) {
        if (static_cast<uint8_t>(f) == want)
            return true;
    }
    return false;
}

static void fill_ctrl_packet(LinkPacket &pkt, const SwarmConfig &cfg, int dst_sysid,
                             uint32_t op, float p2, float p3, float p4,
                             float p5, float p6, float p7) {
    pkt.msg_type = LinkMsgType::CTRL_CMD;
    pkt.src_id = static_cast<uint8_t>(cfg.local_sysid);
    pkt.dst_id = static_cast<uint8_t>(dst_sysid);
    pkt.seq = op;
    pkt.ctrl_pf[0] = p2;
    pkt.ctrl_pf[1] = p3;
    pkt.ctrl_pf[2] = p4;
    pkt.ctrl_pf[3] = p5;
    pkt.ctrl_pf[4] = p6;
    pkt.ctrl_pf[5] = p7;
}

bool SwarmController::try_proxy_gcs_command_to_follower(const mavlink_message_t &msg) {
    if (cfg_.role != NodeRole::LEADER || !cfg_.gcs_serial_enable || !air_link_)
        return false;

    mavlink_command_long_t cl{};
    mavlink_msg_command_long_decode(&msg, &cl);

    const int tgt = static_cast<int>(cl.target_system);
    if (tgt == cfg_.local_fc.sysid || !follower_sysid_contains(tgt))
        return false;
    if (!gcs_ctrl_target_component_ok(cl.target_component))
        return false;

    if ((cl.command >= SwarmMavCmd::START_FORMATION &&
          cl.command <= SwarmMavCmd::STOP_EXEC) ||
        (cl.command >= SwarmMavCmd::TASK_BEGIN &&
         cl.command <= SwarmMavCmd::TASK_START_AUTO) ||
        cl.command == SwarmMavCmd::CTRL_PROXY) {
        send_gcs_command_ack(msg, cl.command, MAV_RESULT_DENIED);
        return true;
    }

    LinkPacket pkt{};
    switch (cl.command) {
    case MAV_CMD_COMPONENT_ARM_DISARM:
        fill_ctrl_packet(pkt, cfg_, tgt, PROXY_ARM_DISARM,
                         cl.param1, cl.param2, 0, 0, 0, 0);
        break;
    case MAV_CMD_NAV_TAKEOFF:
        fill_ctrl_packet(pkt, cfg_, tgt, PROXY_NAV_TAKEOFF,
                         cl.param2, cl.param3, cl.param4, cl.param5, cl.param6,
                         cl.param7);
        break;
    case MAV_CMD_NAV_LAND:
        fill_ctrl_packet(pkt, cfg_, tgt, PROXY_NAV_LAND,
                         cl.param1, cl.param2, cl.param3, cl.param4, cl.param5,
                         cl.param6);
        break;
    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        fill_ctrl_packet(pkt, cfg_, tgt, PROXY_NAV_RTL, 0, 0, 0, 0, 0, 0);
        break;
    case MAV_CMD_DO_SET_MODE:
        fill_ctrl_packet(pkt, cfg_, tgt, PROXY_DO_SET_MODE,
                         cl.param1, cl.param2, 0, 0, 0, 0);
        break;
    default:
        send_gcs_command_ack(msg, cl.command, MAV_RESULT_UNSUPPORTED);
        return true;
    }

    const auto t0 = std::chrono::steady_clock::now();
    const int send_ret = air_link_->send_packet(pkt);
    const uint64_t dt_us = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - t0).count());
    {
        std::lock_guard<std::mutex> lk(timing_mtx_);
        timing_record(leader_timing_.gcs_to_air_send, dt_us, send_ret >= 0, sizeof(LinkPacket));
    }

    if (send_ret < 0) {
        log_cmd_error("[swarm] GCS proxy: CTRL_CMD send failed (dst=%d cmd=%u)\n",
                      tgt, static_cast<unsigned>(cl.command));
        send_gcs_command_ack(msg, cl.command, MAV_RESULT_FAILED);
    } else {
        log_cmd_info("[swarm] GCS proxy: COMMAND_LONG cmd=%u -> follower %d\n",
                     static_cast<unsigned>(cl.command), tgt);
        send_gcs_command_ack(msg, cl.command, MAV_RESULT_ACCEPTED);
    }
    return true;
}

bool SwarmController::try_proxy_gcs_set_mode_to_follower(const mavlink_message_t &msg) {
    if (cfg_.role != NodeRole::LEADER || !cfg_.gcs_serial_enable || !air_link_)
        return false;

    mavlink_set_mode_t sm{};
    mavlink_msg_set_mode_decode(&msg, &sm);

    const int tgt = static_cast<int>(sm.target_system);
    if (tgt == cfg_.local_fc.sysid || !follower_sysid_contains(tgt))
        return false;

    LinkPacket pkt{};
    fill_ctrl_packet(pkt, cfg_, tgt, PROXY_DO_SET_MODE,
                     static_cast<float>(sm.base_mode),
                     static_cast<float>(sm.custom_mode), 0, 0, 0, 0);

    const auto t0 = std::chrono::steady_clock::now();
    const int send_ret = air_link_->send_packet(pkt);
    const uint64_t dt_us = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - t0).count());
    {
        std::lock_guard<std::mutex> lk(timing_mtx_);
        timing_record(leader_timing_.gcs_to_air_send, dt_us, send_ret >= 0, sizeof(LinkPacket));
    }

    if (send_ret < 0) {
        log_cmd_error("[swarm] GCS proxy: SET_MODE send failed (dst=%d)\n", tgt);
    } else {
        log_cmd_info("[swarm] GCS proxy: SET_MODE -> follower %d custom=%u\n", tgt,
                     static_cast<unsigned>(sm.custom_mode));
    }
    return true;
}

bool SwarmController::try_proxy_gcs_command_int_to_follower(const mavlink_message_t &msg) {
    if (cfg_.role != NodeRole::LEADER || !cfg_.gcs_serial_enable || !air_link_)
        return false;

    mavlink_command_int_t ci{};
    mavlink_msg_command_int_decode(&msg, &ci);

    const int tgt = static_cast<int>(ci.target_system);
    if (tgt == cfg_.local_fc.sysid || !follower_sysid_contains(tgt))
        return false;
    if (!gcs_ctrl_target_component_ok(ci.target_component))
        return false;

    const uint16_t cmd = ci.command;
    if ((cmd >= SwarmMavCmd::START_FORMATION &&
         cmd <= SwarmMavCmd::STOP_EXEC) ||
        (cmd >= SwarmMavCmd::TASK_BEGIN &&
         cmd <= SwarmMavCmd::TASK_START_AUTO) ||
        cmd == SwarmMavCmd::CTRL_PROXY) {
        send_gcs_command_ack(msg, cmd, MAV_RESULT_DENIED);
        return true;
    }

    LinkPacket pkt{};
    switch (cmd) {
    case MAV_CMD_COMPONENT_ARM_DISARM:
        fill_ctrl_packet(pkt, cfg_, tgt, PROXY_ARM_DISARM,
                         ci.param1, ci.param2, 0, 0, 0, 0);
        break;
    case MAV_CMD_NAV_TAKEOFF:
        fill_ctrl_packet(pkt, cfg_, tgt, PROXY_NAV_TAKEOFF,
                         ci.param2, ci.param3, ci.param4, ci.param1, 0, ci.z);
        break;
    case MAV_CMD_NAV_LAND:
        fill_ctrl_packet(pkt, cfg_, tgt, PROXY_NAV_LAND,
                         ci.param1, ci.param2, ci.param3, ci.param4, 0, 0);
        break;
    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        fill_ctrl_packet(pkt, cfg_, tgt, PROXY_NAV_RTL, 0, 0, 0, 0, 0, 0);
        break;
    case MAV_CMD_DO_SET_MODE:
        fill_ctrl_packet(pkt, cfg_, tgt, PROXY_DO_SET_MODE,
                         ci.param1, ci.param2, 0, 0, 0, 0);
        break;
    default:
        send_gcs_command_ack(msg, cmd, MAV_RESULT_UNSUPPORTED);
        return true;
    }

    const auto t0 = std::chrono::steady_clock::now();
    const int send_ret = air_link_->send_packet(pkt);
    const uint64_t dt_us = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - t0).count());
    {
        std::lock_guard<std::mutex> lk(timing_mtx_);
        timing_record(leader_timing_.gcs_to_air_send, dt_us, send_ret >= 0, sizeof(LinkPacket));
    }

    if (send_ret < 0) {
        log_cmd_error("[swarm] GCS proxy: COMMAND_INT send failed (dst=%d cmd=%u)\n",
                      tgt, static_cast<unsigned>(cmd));
        send_gcs_command_ack(msg, cmd, MAV_RESULT_FAILED);
    } else {
        log_cmd_info("[swarm] GCS proxy: COMMAND_INT cmd=%u -> follower %d\n",
                     static_cast<unsigned>(cmd), tgt);
        send_gcs_command_ack(msg, cmd, MAV_RESULT_ACCEPTED);
    }
    return true;
}

void SwarmController::execute_follower_ctrl_cmd(const LinkPacket &pkt) {
    auto *fc = mav_mgr_.get_vehicle(cfg_.local_fc.sysid);
    if (!fc) return;

    const uint32_t op = pkt.seq;
    switch (op) {
    case PROXY_ARM_DISARM:
        fc->arm(pkt.ctrl_pf[0] > 0.5f);
        log_cmd_info("[swarm] follower CTRL: ARM_DISARM arm=%d\n",
                     pkt.ctrl_pf[0] > 0.5f ? 1 : 0);
        break;
    case PROXY_NAV_TAKEOFF:
        if (cfg_.vehicle_type == VehicleType::COPTER)
            fc->set_copter_mode(CopterMode::GUIDED);
        else
            fc->set_plane_mode(PlaneMode::GUIDED);
        fc->takeoff(pkt.ctrl_pf[5]);
        last_target_sent_ms_.store(steady_ms(), std::memory_order_relaxed);
        log_cmd_info("[swarm] follower CTRL: TAKEOFF alt=%.1f m\n", pkt.ctrl_pf[5]);
        break;
    case PROXY_NAV_LAND:
        fc->land();
        log_cmd_info("[swarm] follower CTRL: LAND\n");
        break;
    case PROXY_NAV_RTL:
        fc->rtl();
        log_cmd_info("[swarm] follower CTRL: RTL\n");
        break;
    case PROXY_DO_SET_MODE: {
        const uint32_t custom =
            static_cast<uint32_t>(pkt.ctrl_pf[1] + 0.5f);
        if (cfg_.vehicle_type == VehicleType::COPTER)
            fc->set_copter_mode(static_cast<CopterMode>(custom));
        else
            fc->set_plane_mode(static_cast<PlaneMode>(custom));
        log_cmd_info("[swarm] follower CTRL: DO_SET_MODE custom_mode=%u\n",
                     static_cast<unsigned>(custom));
        break;
    }
    default:
        log_cmd_error("[swarm] follower CTRL: unknown op=%u\n",
                      static_cast<unsigned>(op));
        break;
    }
}

void SwarmController::poll_gcs_link() {
    auto dispatch_gcs_msg = [this](const mavlink_message_t &msg) {
        const auto t_dispatch0 = std::chrono::steady_clock::now();
        bool consumed_locally = false;

        // 先处理需要由伴机本地消费的消息，避免已本地执行的群控命令再透传飞控造成双 ACK。
        switch (msg.msgid) {
        case MAVLINK_MSG_ID_COMMAND_LONG: {
            mavlink_command_long_t cl{};
            mavlink_msg_command_long_decode(&msg, &cl);
            if (cfg_.role == NodeRole::LEADER) {
                log_cmd_info("[swarm] leader recv GCS COMMAND_LONG: cmd=%u src=%u/%u target=%u/%u\n",
                             static_cast<unsigned>(cl.command),
                             static_cast<unsigned>(msg.sysid),
                             static_cast<unsigned>(msg.compid),
                             static_cast<unsigned>(cl.target_system),
                             static_cast<unsigned>(cl.target_component));
            }
            if (try_proxy_gcs_command_to_follower(msg)) {
                consumed_locally = true;
                break;
            }
            const bool is_swarm_cmd =
                (cl.command >= SwarmMavCmd::START_FORMATION &&
                 cl.command <= SwarmMavCmd::STOP_EXEC);
            const bool to_local_swarm_handler =
                (cl.target_system == static_cast<uint8_t>(cfg_.local_sysid)) &&
                (cl.target_component == MAV_COMP_ID_ALL ||
                 cl.target_component == MAV_COMP_ID_ONBOARD_COMPUTER);
            if (is_swarm_cmd && to_local_swarm_handler) {
                handle_gcs_command_long(msg);
                consumed_locally = true;
            }
            break;
        }
        case MAVLINK_MSG_ID_COMMAND_INT:
            if (cfg_.role == NodeRole::LEADER) {
                mavlink_command_int_t ci{};
                mavlink_msg_command_int_decode(&msg, &ci);
                log_cmd_info("[swarm] leader recv GCS COMMAND_INT: cmd=%u src=%u/%u target=%u/%u\n",
                             static_cast<unsigned>(ci.command),
                             static_cast<unsigned>(msg.sysid),
                             static_cast<unsigned>(msg.compid),
                             static_cast<unsigned>(ci.target_system),
                             static_cast<unsigned>(ci.target_component));
            }
            if (try_proxy_gcs_command_int_to_follower(msg))
                consumed_locally = true;
            break;
        case MAVLINK_MSG_ID_SET_MODE:
            if (cfg_.role == NodeRole::LEADER) {
                mavlink_set_mode_t sm{};
                mavlink_msg_set_mode_decode(&msg, &sm);
                log_cmd_info("[swarm] leader recv GCS SET_MODE: custom=%u src=%u/%u target=%u\n",
                             static_cast<unsigned>(sm.custom_mode),
                             static_cast<unsigned>(msg.sysid),
                             static_cast<unsigned>(msg.compid),
                             static_cast<unsigned>(sm.target_system));
                log_leader_fc_mode_state("recv GCS SET_MODE");
            }
            if (try_proxy_gcs_set_mode_to_follower(msg))
                consumed_locally = true;
            break;
        default:
            break;
        }

        if (!consumed_locally) {
            if (auto *v = mav_mgr_.get_vehicle(cfg_.local_fc.sysid)) {
                if (should_forward_gcs_msg_to_local_fc(msg, cfg_.local_fc.sysid)) {
                    uint8_t wire[MAVLINK_MAX_PACKET_LEN];
                    uint16_t wire_len = mavlink_msg_to_send_buffer(wire, &msg);
                    const auto t0 = std::chrono::steady_clock::now();
                    const int n = v->write_raw_bytes(wire, wire_len);
                    const uint64_t dt_us = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - t0).count());
                    std::lock_guard<std::mutex> lk(timing_mtx_);
                    timing_record(leader_timing_.gcs_to_fc_write, dt_us, n >= 0, wire_len);
                }
            }
        }

        // 再处理需要伴机侧维护状态/镜像的逻辑。
        switch (msg.msgid) {
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ:
        case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST:
            mark_gcs_param_fetch_from_gcs();
            break;
        case MAVLINK_MSG_ID_MISSION_COUNT:
            handle_gcs_mission_count(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_ITEM_INT:
            handle_gcs_mission_item_int(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
            handle_gcs_mission_clear_all(msg);
            break;
        default:
            break;
        }

        const uint64_t dispatch_us = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t_dispatch0).count());
        std::lock_guard<std::mutex> lk(timing_mtx_);
        timing_record(leader_timing_.gcs_msg_dispatch, dispatch_us, true, 0);
    };

    if (!gcs_link_) return;
    uint8_t b = 0;
    while (gcs_link_->read_byte(&b)) {
        mavlink_message_t msg;
        if (mavlink_parse_char(MAVLINK_COMM_1, b, &msg, &gcs_mav_status_))
            dispatch_gcs_msg(msg);
    }
}

void SwarmController::handle_gcs_command_long(const mavlink_message_t &msg) {
    mavlink_command_long_t cl{};
    mavlink_msg_command_long_decode(&msg, &cl);

    if (cl.target_system != static_cast<uint8_t>(cfg_.local_sysid)) return;
    if (cl.target_component != MAV_COMP_ID_ALL &&
        cl.target_component != MAV_COMP_ID_ONBOARD_COMPUTER) return;

    switch (cl.command) {
    case SwarmMavCmd::START_FORMATION:
        log_cmd_info("[swarm] handle GCS command: START_FORMATION\n");
        cmd_start_formation();
        break;
    case SwarmMavCmd::STOP_FORMATION:
        log_cmd_info("[swarm] handle GCS command: STOP_FORMATION\n");
        cmd_stop_formation();
        break;
    case SwarmMavCmd::RTL:
        log_cmd_info("[swarm] handle GCS command: RTL\n");
        cmd_rtl();
        break;
    case SwarmMavCmd::LAND:
        log_cmd_info("[swarm] handle GCS command: LAND\n");
        cmd_land();
        break;
    case SwarmMavCmd::ARM_TAKEOFF:
        log_cmd_info("[swarm] handle GCS command: ARM_TAKEOFF alt=%.1f\n", cl.param1);
        cmd_arm_and_takeoff(cl.param1);
        break;
    case SwarmMavCmd::SET_MODE:
        log_cmd_info("[swarm] handle GCS command: SET_MODE mode=%.1f\n", cl.param1);
        set_exec_mode(cl.param1 == 2.0f ? ExecMode::INDEPENDENT : ExecMode::FORMATION);
        log_leader_fc_mode_state("handle GCS CMD SET_MODE");
        break;
    case SwarmMavCmd::START_EXEC:
        log_cmd_info("[swarm] handle GCS command: START_EXEC\n");
        start_exec();
        break;
    case SwarmMavCmd::STOP_EXEC:
        log_cmd_info("[swarm] handle GCS command: STOP_EXEC\n");
        stop_exec();
        break;
    default:
        return;
    }

    send_gcs_command_ack(msg, cl.command, MAV_RESULT_ACCEPTED);
}

void SwarmController::send_gcs_command_ack(const mavlink_message_t &cmd_msg,
                                           uint16_t cmd_id,
                                           uint8_t result) {
    mavlink_message_t ack_msg;
    mavlink_command_ack_t ack{};
    ack.command = cmd_id;
    ack.result = result;
    ack.progress = 0;
    ack.result_param2 = 0;
    ack.target_system = cmd_msg.sysid;
    ack.target_component = cmd_msg.compid;

    mavlink_msg_command_ack_encode(static_cast<uint8_t>(cfg_.local_sysid),
                                   MAV_COMP_ID_ONBOARD_COMPUTER,
                                   &ack_msg, &ack);

    enqueue_gcs_mavlink(ack_msg, GcsTxPriority::HIGH);
}

void SwarmController::enqueue_gcs_mavlink(const mavlink_message_t &msg,
                                          GcsTxPriority prio) {
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
    case MAVLINK_MSG_ID_COMMAND_ACK:
    case MAVLINK_MSG_ID_STATUSTEXT:
    case MAVLINK_MSG_ID_SYS_STATUS:
    case MAVLINK_MSG_ID_HIGH_LATENCY2:
        prio = GcsTxPriority::HIGH;
        break;
    default:
        break;
    }
    uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t tx_len = mavlink_msg_to_send_buffer(tx_buf, &msg);
    enqueue_gcs_bytes(tx_buf, tx_len, prio);
}

void SwarmController::enqueue_gcs_bytes(const uint8_t *buf, uint16_t len,
                                        GcsTxPriority prio) {
    if (!cfg_.gcs_serial_enable || len == 0 || len > MAVLINK_MAX_PACKET_LEN)
        return;

    QueuedGcsTxFrame frame{};
    frame.len = len;
    std::memcpy(frame.bytes.data(), buf, len);

    std::lock_guard<std::mutex> lk(gcs_tx_queue_mtx_);
    std::deque<QueuedGcsTxFrame> &queue =
        (prio == GcsTxPriority::HIGH) ? gcs_tx_queue_high_ : gcs_tx_queue_normal_;
    if (queue.size() >= gcs_tx_queue_max_frames_) {
        queue.pop_front();
        if (prio == GcsTxPriority::HIGH)
            gcs_tx_dropped_high_++;
        else
            gcs_tx_dropped_normal_++;
    }
    queue.push_back(frame);
    if (prio == GcsTxPriority::HIGH)
        gcs_tx_enqueued_high_++;
    else
        gcs_tx_enqueued_normal_++;
}

void SwarmController::flush_gcs_tx_queue(size_t max_packets, size_t max_bytes) {
    if (!cfg_.gcs_serial_enable || max_packets == 0 || max_bytes == 0)
        return;

    const uint64_t now = steady_ms();
    size_t sent_packets = 0;
    size_t sent_bytes = 0;
    {
        std::lock_guard<std::mutex> lk(gcs_tx_queue_mtx_);
        if (gcs_tx_tokens_last_refill_ms_ == 0)
            gcs_tx_tokens_last_refill_ms_ = now;
        const uint64_t elapsed_ms = now - gcs_tx_tokens_last_refill_ms_;
        if (elapsed_ms > 0 && gcs_tx_token_rate_bps_ > 0) {
            const uint64_t add =
                (static_cast<uint64_t>(gcs_tx_token_rate_bps_) * elapsed_ms) / 1000ULL;
            const uint64_t capped = std::min<uint64_t>(
                static_cast<uint64_t>(gcs_tx_tokens_) + add, gcs_tx_token_capacity_);
            gcs_tx_tokens_ = static_cast<uint32_t>(capped);
            gcs_tx_tokens_last_refill_ms_ = now;
        }
    }

    while (sent_packets < max_packets && sent_bytes < max_bytes) {
        QueuedGcsTxFrame frame{};
        bool has_frame = false;
        {
            std::lock_guard<std::mutex> lk(gcs_tx_queue_mtx_);
            if (!gcs_tx_queue_high_.empty()) {
                if (gcs_tx_tokens_ < gcs_tx_queue_high_.front().len)
                    break;
                frame = gcs_tx_queue_high_.front();
                gcs_tx_queue_high_.pop_front();
                has_frame = true;
            } else if (!gcs_tx_queue_normal_.empty()) {
                if (gcs_tx_tokens_ < gcs_tx_queue_normal_.front().len)
                    break;
                frame = gcs_tx_queue_normal_.front();
                gcs_tx_queue_normal_.pop_front();
                has_frame = true;
            }
            if (!has_frame)
                break;
            gcs_tx_tokens_ -= frame.len;
        }

        int ret = -1;
        if (gcs_link_) {
            std::lock_guard<std::mutex> lk(gcs_tx_mtx_);
            ret = gcs_link_->write_mavlink(frame.bytes.data(), frame.len);
        }
        if (ret < 0)
            break;

        sent_packets++;
        sent_bytes += frame.len;
        {
            std::lock_guard<std::mutex> lk(gcs_tx_queue_mtx_);
            gcs_tx_sent_packets_++;
            gcs_tx_sent_bytes_ += frame.len;
        }
    }
    maybe_print_gcs_tx_queue_stats();
}

void SwarmController::maybe_print_gcs_tx_queue_stats() {
    const uint64_t now = steady_ms();
    std::lock_guard<std::mutex> lk(gcs_tx_queue_mtx_);
    constexpr uint64_t kPrintMs = 5000;
    if (gcs_tx_diag_last_ms_ != 0 && (now - gcs_tx_diag_last_ms_) < kPrintMs)
        return;
    gcs_tx_diag_last_ms_ = now;
    fprintf(stderr,
            "[gcs_tx] qh=%zu qn=%zu tok=%u/%u enq_h=%llu enq_n=%llu drop_h=%llu drop_n=%llu sent_pkt=%llu sent_bytes=%llu\n",
            gcs_tx_queue_high_.size(),
            gcs_tx_queue_normal_.size(),
            static_cast<unsigned>(gcs_tx_tokens_),
            static_cast<unsigned>(gcs_tx_token_capacity_),
            static_cast<unsigned long long>(gcs_tx_enqueued_high_),
            static_cast<unsigned long long>(gcs_tx_enqueued_normal_),
            static_cast<unsigned long long>(gcs_tx_dropped_high_),
            static_cast<unsigned long long>(gcs_tx_dropped_normal_),
            static_cast<unsigned long long>(gcs_tx_sent_packets_),
            static_cast<unsigned long long>(gcs_tx_sent_bytes_));
}

void SwarmController::forward_fc_parsed_to_gcs(const mavlink_message_t &msg) {
    if (!cfg_.gcs_serial_enable) return;
    if (should_drop_fc_msg_to_gcs(msg)) return;

    const uint32_t msgid = msg.msgid;
    switch (msgid) {
    case MAVLINK_MSG_ID_ATTITUDE:
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
    case MAVLINK_MSG_ID_VFR_HUD:
    case MAVLINK_MSG_ID_ATTITUDE_QUATERNION: {
        uint64_t now = steady_ms();
        uint64_t &last = gcs_fwd_throttle_ms_[msgid];
        constexpr uint64_t kMinIntervalMs = 200;
        if (now - last < kMinIntervalMs) return;
        last = now;
        break;
    }
    default:
        break;
    }

    const auto t0 = std::chrono::steady_clock::now();
    enqueue_gcs_mavlink(msg);
    const uint64_t dt_us = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - t0).count());
    std::lock_guard<std::mutex> lk(timing_mtx_);
    timing_record(leader_timing_.fc_to_gcs_send, dt_us, true, 0);
}

void SwarmController::inject_follower_telemetry_to_gcs() {
    if (!cfg_.gcs_serial_enable || cfg_.role != NodeRole::LEADER) return;
    if (!cfg_.follower_synthetic_telem_to_gcs) return;
    // 按方案 §8.1：仅在 P900 长机兜底模式下保留合成遥测注入。
    if (cfg_.gcs_link_type != SwarmConfig::GcsLinkType::LEGACY_P900_LEADER &&
        cfg_.gcs_link_type != SwarmConfig::GcsLinkType::P900) return;
    if (cfg_.follower_sysids.empty()) return;
    if (gcs_param_fetch_active()) return;

    uint64_t now = steady_ms();
    const uint64_t last = follower_telem_last_ms_.load(std::memory_order_relaxed);
    if (now - last < 500) return;
    follower_telem_last_ms_.store(now, std::memory_order_relaxed);

    constexpr uint64_t kFollowerTelemStaleMs = 5000;
    std::vector<std::pair<int, UnitState>> snap;
    {
        std::lock_guard<std::mutex> lk(follower_state_mtx_);
        for (int fid : cfg_.follower_sysids) {
            auto it = follower_state_cache_.find(fid);
            if (it == follower_state_cache_.end())
                continue;
            auto lr = follower_link_last_rx_ms_.find(fid);
            if (lr == follower_link_last_rx_ms_.end())
                continue;
            if (now - lr->second >= kFollowerTelemStaleMs)
                continue;
            snap.emplace_back(fid, it->second);
        }
    }

    if (snap.empty()) return;

    const uint8_t mav_type = (cfg_.vehicle_type == VehicleType::COPTER)
                                 ? MAV_TYPE_QUADROTOR
                                 : MAV_TYPE_FIXED_WING;

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    for (const auto &kv : snap) {
        const int fid = kv.first;
        const UnitState &st = kv.second;
        const uint8_t sys = static_cast<uint8_t>(fid);

        mavlink_message_t msg;
        mavlink_heartbeat_t hb{};
        hb.type = mav_type;
        hb.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
        hb.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                       (st.armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0);
        hb.custom_mode = st.custom_mode_valid
                               ? st.custom_mode
                               : ((cfg_.vehicle_type == VehicleType::COPTER)
                                      ? static_cast<uint32_t>(CopterMode::GUIDED)
                                      : static_cast<uint32_t>(PlaneMode::GUIDED));
        hb.system_status = MAV_STATE_ACTIVE;
        mavlink_msg_heartbeat_encode(sys, MAV_COMP_ID_AUTOPILOT1, &msg, &hb);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        enqueue_gcs_bytes(buf, len);

        mavlink_global_position_int_t gp{};
        gp.time_boot_ms = 0;
        gp.lat = static_cast<int32_t>(st.lat * SWARM_RAD2DEG * 1e7);
        gp.lon = static_cast<int32_t>(st.lon * SWARM_RAD2DEG * 1e7);
        gp.alt = static_cast<int32_t>((st.alt_bar > 0.0f ? st.alt_bar : st.alt_rel) * 1000.0f);
        gp.relative_alt = static_cast<int32_t>(st.alt_rel * 1000.0f);
        const float vn = st.vel_cruise * cosf(st.heading);
        const float ve = st.vel_cruise * sinf(st.heading);
        gp.vx = static_cast<int16_t>(vn * 100.0f);
        gp.vy = static_cast<int16_t>(ve * 100.0f);
        gp.vz = static_cast<int16_t>(st.vd * 100.0f);
        float hdg_deg = st.heading * SWARM_RAD2DEG;
        if (hdg_deg < 0) hdg_deg += 360.0f;
        if (hdg_deg >= 360.0f) hdg_deg -= 360.0f;
        gp.hdg = static_cast<uint16_t>(hdg_deg * 100.0f);
        mavlink_msg_global_position_int_encode(sys, MAV_COMP_ID_AUTOPILOT1, &msg, &gp);
        len = mavlink_msg_to_send_buffer(buf, &msg);
        enqueue_gcs_bytes(buf, len);

        mavlink_attitude_t att{};
        att.time_boot_ms = 0;
        att.roll = st.roll;
        att.pitch = st.pitch;
        att.yaw = st.heading;
        att.rollspeed = 0;
        att.pitchspeed = 0;
        att.yawspeed = 0;
        mavlink_msg_attitude_encode(sys, MAV_COMP_ID_AUTOPILOT1, &msg, &att);
        len = mavlink_msg_to_send_buffer(buf, &msg);
        enqueue_gcs_bytes(buf, len);
    }
}

void SwarmController::reset_gcs_mission_rx() {
    gcs_mission_expected_ = 0;
    gcs_mission_rx_buf_.clear();
    gcs_mission_rx_got_.clear();
}

void SwarmController::set_exec_mode(ExecMode mode) {
    const ExecMode prev = exec_mode_.load(std::memory_order_relaxed);
    if (mode == prev)
        return;
    exec_mode_.store(mode, std::memory_order_relaxed);
    const char *prev_cn =
        (prev == ExecMode::FORMATION) ? "编队飞行" : "独立任务";
    const char *now_cn =
        (mode == ExecMode::FORMATION) ? "编队飞行" : "独立任务";
    printf("[swarm] 编队模式切换: %s -> %s\n", prev_cn, now_cn);
}

void SwarmController::start_exec() {
    if (exec_mode_.load(std::memory_order_relaxed) == ExecMode::FORMATION) {
        cmd_start_formation();
        return;
    }
    distribute_independent_mission();
}

void SwarmController::stop_exec() {
    if (exec_mode_.load(std::memory_order_relaxed) == ExecMode::FORMATION) {
        cmd_stop_formation();
        return;
    }
    leader_pending_independent_auto_.store(false, std::memory_order_relaxed);
    independent_follower_start_pending_.store(false, std::memory_order_relaxed);
    printf("[swarm] independent execution stop requested\n");
}

void SwarmController::distribute_independent_mission() {
    if (cfg_.role != NodeRole::LEADER) return;

    std::vector<WayPoint> master;
    {
        std::lock_guard<std::mutex> lk(wp_mtx_);
        if (wp_num_ <= 0) {
            printf("[swarm] independent mode ignored: no master mission\n");
            return;
        }
        master.assign(waypoints_, waypoints_ + wp_num_);
    }

    std::vector<int> agents;
    agents.reserve(1 + cfg_.follower_sysids.size());
    agents.push_back(cfg_.local_sysid);
    for (int fid : cfg_.follower_sysids) agents.push_back(fid);

    auto subs = mission_splitter_.split_evenly(master, agents);
    const uint32_t version = static_cast<uint32_t>(steady_ms() & 0x7fffffff);
    independent_task_version_.store(version, std::memory_order_relaxed);
    leader_pending_independent_auto_.store(false, std::memory_order_relaxed);
    independent_follower_start_pending_.store(false, std::memory_order_relaxed);

    auto *fc = mav_mgr_.get_vehicle(cfg_.local_fc.sysid);
    if (!fc) return;

    for (const auto &s : subs) {
        if (s.sysid == cfg_.local_sysid) {
            if (s.waypoints.empty()) {
                printf("[swarm] independent: leader subtask empty, skip local upload\n");
                if (!cfg_.follower_sysids.empty())
                    independent_follower_start_pending_.store(true, std::memory_order_relaxed);
                continue;
            }
            if (fc->upload_mission(s.waypoints) == 0) {
                leader_pending_independent_auto_.store(true, std::memory_order_relaxed);
                printf("[swarm] independent: leader subtask upload started, n=%zu version=%u\n",
                       s.waypoints.size(), version);
            } else {
                fprintf(stderr, "[swarm] independent: leader upload_mission failed to start "
                        "(n=%zu, fc_connected=%d)\n",
                        s.waypoints.size(), fc->is_connected() ? 1 : 0);
            }
            continue;
        }

        LinkPacket meta{};
        meta.msg_type = LinkMsgType::TASK_META;
        meta.src_id = cfg_.local_sysid;
        meta.dst_id = s.sysid;
        meta.seq = version;
        meta.alt = static_cast<float>(s.waypoints.size());
        air_link_->send_packet(meta);

        for (size_t i = 0; i < s.waypoints.size(); i++) {
            LinkPacket item{};
            item.msg_type = LinkMsgType::TASK_ITEM;
            item.src_id = cfg_.local_sysid;
            item.dst_id = s.sysid;
            item.seq = static_cast<uint32_t>(i);
            item.lat = s.waypoints[i].lat;
            item.lon = s.waypoints[i].lon;
            item.alt = static_cast<float>(s.waypoints[i].alt);
            air_link_->send_packet(item);
        }

        LinkPacket commit{};
        commit.msg_type = LinkMsgType::TASK_COMMIT;
        commit.src_id = cfg_.local_sysid;
        commit.dst_id = s.sysid;
        commit.seq = version;
        air_link_->send_packet(commit);
    }

    if (!cfg_.follower_sysids.empty()) {
        printf("[swarm] independent: subtasks sent to followers (await leader then TASK_START), "
               "agents=%zu version=%u\n",
               subs.size(), version);
    } else {
        printf("[swarm] independent: leader-only, version=%u\n", version);
    }
}

void SwarmController::try_finish_independent_leader_auto() {
    if (cfg_.role != NodeRole::LEADER) return;
    if (exec_mode_.load(std::memory_order_relaxed) != ExecMode::INDEPENDENT) return;

    auto *fc = mav_mgr_.get_vehicle(cfg_.local_fc.sysid);
    if (!fc) return;

    auto broadcast_task_start_to_followers = [this]() {
        for (int fid : cfg_.follower_sysids) {
            LinkPacket start{};
            start.msg_type = LinkMsgType::TASK_START;
            start.src_id = cfg_.local_sysid;
            start.dst_id = fid;
            start.seq = independent_task_version_.load(std::memory_order_relaxed);
            air_link_->send_packet(start);
        }
        printf("[swarm] independent: TASK_START -> followers, version=%u\n",
               independent_task_version_.load(std::memory_order_relaxed));
    };

    if (leader_pending_independent_auto_.load(std::memory_order_relaxed)) {
        if (fc->mission_uploading()) return;

        if (fc->last_mission_ack_type() != MAV_MISSION_ACCEPTED) {
            fprintf(stderr, "[swarm] independent: leader MISSION_ACK not accepted (type=%d), "
                    "skip AUTO + TASK_START\n",
                    fc->last_mission_ack_type());
            leader_pending_independent_auto_.store(false, std::memory_order_relaxed);
            return;
        }

        if (cfg_.vehicle_type == VehicleType::COPTER)
            fc->set_copter_mode(CopterMode::AUTO);
        else
            fc->set_plane_mode(PlaneMode::AUTO);
        log_leader_fc_mode_state("leader switch to AUTO");

        leader_pending_independent_auto_.store(false, std::memory_order_relaxed);

        if (!cfg_.follower_sysids.empty())
            broadcast_task_start_to_followers();

        printf("[swarm] independent: leader AUTO, version=%u\n",
               independent_task_version_.load(std::memory_order_relaxed));
        return;
    }

    if (independent_follower_start_pending_.load(std::memory_order_relaxed)) {
        independent_follower_start_pending_.store(false, std::memory_order_relaxed);
        if (!cfg_.follower_sysids.empty())
            broadcast_task_start_to_followers();
    }
}

void SwarmController::reset_follower_task_rx() {
    follower_task_expected_ = 0;
    follower_task_buf_.clear();
    follower_task_got_.clear();
}

void SwarmController::handle_follower_task_packets(const LinkPacket &pkt) {
    if (cfg_.role == NodeRole::LEADER) {
        if (pkt.msg_type == LinkMsgType::TASK_ACK) {
            printf("[swarm] follower %u task ack: version=%u result=%s\n",
                   pkt.src_id, pkt.seq, (pkt.alt == 0.0f ? "OK" : "FAIL"));
        }
        return;
    }
    if (cfg_.role != NodeRole::FOLLOWER) return;
    if (pkt.dst_id != cfg_.local_sysid) return;

    auto *fc = mav_mgr_.get_vehicle(cfg_.local_fc.sysid);
    if (!fc) return;

    if (pkt.msg_type == LinkMsgType::TASK_META) {
        follower_task_version_ = pkt.seq;
        follower_task_expected_ = static_cast<uint16_t>(pkt.alt);
        follower_task_buf_.assign(follower_task_expected_, WayPoint{});
        follower_task_got_.assign(follower_task_expected_, 0);
        follower_task_pending_auto_start_ = false;
        return;
    }

    if (pkt.msg_type == LinkMsgType::TASK_ITEM) {
        if (follower_task_expected_ == 0) return;
        if (pkt.seq >= follower_task_buf_.size()) return;
        WayPoint wp{};
        wp.lat = pkt.lat;
        wp.lon = pkt.lon;
        wp.alt = pkt.alt;
        wp.mode = 2;
        follower_task_buf_[pkt.seq] = wp;
        follower_task_got_[pkt.seq] = 1;
        return;
    }

    if (pkt.msg_type == LinkMsgType::TASK_COMMIT) {
        if (pkt.seq != follower_task_version_) return;

        LinkPacket ack{};
        ack.msg_type = LinkMsgType::TASK_ACK;
        ack.src_id = cfg_.local_sysid;
        ack.dst_id = cfg_.leader_sysid;
        ack.seq = follower_task_version_;

        if (follower_task_expected_ == 0) {
            ack.alt = 0.0f;
            follower_task_pending_auto_start_ = true;
            printf("[swarm] follower empty subtask ack ok, version=%u\n", follower_task_version_);
            air_link_->send_packet(ack);
            return;
        }

        bool complete = !follower_task_got_.empty() &&
            std::all_of(follower_task_got_.begin(), follower_task_got_.end(),
                        [](uint8_t v) { return v != 0; });

        if (complete && fc->upload_mission(follower_task_buf_) == 0) {
            ack.alt = 0.0f;
            follower_task_pending_auto_start_ = true;
            printf("[swarm] follower task uploaded, version=%u, n=%u\n",
                   follower_task_version_, follower_task_expected_);
        } else {
            ack.alt = 1.0f;
            follower_task_pending_auto_start_ = false;
            printf("[swarm] follower task upload failed, version=%u\n", follower_task_version_);
        }
        air_link_->send_packet(ack);
        return;
    }

    if (pkt.msg_type == LinkMsgType::TASK_START) {
        if (pkt.seq != follower_task_version_ || !follower_task_pending_auto_start_) return;
        if (cfg_.vehicle_type == VehicleType::COPTER)
            fc->set_copter_mode(CopterMode::AUTO);
        else
            fc->set_plane_mode(PlaneMode::AUTO);
        follower_task_pending_auto_start_ = false;
        printf("[swarm] follower AUTO started, version=%u\n", follower_task_version_);
        return;
    }
}

void SwarmController::handle_gcs_mission_count(const mavlink_message_t &msg) {
    if (cfg_.role != NodeRole::LEADER || !cfg_.gcs_serial_enable)
        return;

    mavlink_mission_count_t mc{};
    mavlink_msg_mission_count_decode(&msg, &mc);

    if (mc.target_system != static_cast<uint8_t>(cfg_.local_fc.sysid))
        return;
    if (mc.mission_type != MAV_MISSION_TYPE_MISSION)
        return;

    if (mc.count == 0) {
        reset_gcs_mission_rx();
        cmd_upload_waypoints({});
        printf("[swarm] GCS mission: count=0, waypoints cleared\n");
        return;
    }

    gcs_mission_expected_ = mc.count;
    gcs_mission_rx_buf_.assign(mc.count, WayPoint{});
    gcs_mission_rx_got_.assign(mc.count, 0);
    printf("[swarm] GCS mission: expecting %u items (MP -> FC, mirrored to waypoints_)\n",
           static_cast<unsigned>(mc.count));
}

void SwarmController::handle_gcs_mission_item_int(const mavlink_message_t &msg) {
    if (cfg_.role != NodeRole::LEADER || !cfg_.gcs_serial_enable)
        return;
    if (gcs_mission_expected_ == 0 || gcs_mission_rx_buf_.empty())
        return;

    mavlink_mission_item_int_t it{};
    mavlink_msg_mission_item_int_decode(&msg, &it);

    if (it.target_system != static_cast<uint8_t>(cfg_.local_fc.sysid))
        return;
    if (it.mission_type != MAV_MISSION_TYPE_MISSION)
        return;

    if (it.seq >= gcs_mission_rx_buf_.size())
        return;

    gcs_mission_rx_buf_[it.seq] = mission_item_int_to_waypoint(it);
    if (it.seq < gcs_mission_rx_got_.size())
        gcs_mission_rx_got_[it.seq] = 1;

    bool complete = !gcs_mission_rx_got_.empty() &&
        std::all_of(gcs_mission_rx_got_.begin(), gcs_mission_rx_got_.end(),
                    [](uint8_t v) { return v != 0; });

    if (complete) {
        cmd_upload_waypoints(gcs_mission_rx_buf_);
        reset_gcs_mission_rx();
    }
}

void SwarmController::handle_gcs_mission_clear_all(const mavlink_message_t &msg) {
    if (cfg_.role != NodeRole::LEADER || !cfg_.gcs_serial_enable)
        return;

    mavlink_mission_clear_all_t mca{};
    mavlink_msg_mission_clear_all_decode(&msg, &mca);

    if (mca.target_system != static_cast<uint8_t>(cfg_.local_fc.sysid))
        return;
    if (mca.mission_type != MAV_MISSION_TYPE_MISSION)
        return;

    reset_gcs_mission_rx();
    cmd_upload_waypoints({});
    printf("[swarm] GCS MISSION_CLEAR_ALL: waypoints cleared\n");
}

void SwarmController::log_leader_fc_mode_state(const char *scene) {
    auto *leader_fc = mav_mgr_.get_vehicle(cfg_.local_fc.sysid);
    if (!leader_fc) {
        log_cmd_error("[swarm] %s: 长机飞控模式状态读取失败: local fc unavailable\n", scene);
        return;
    }
    const UnitState &leader_st = leader_fc->state();
    log_cmd_info("[swarm] %s: 长机飞控模式状态: custom_mode=%u armed=%d health=%u\n",
                 scene,
                 static_cast<unsigned>(leader_st.custom_mode),
                 leader_st.armed ? 1 : 0,
                 static_cast<unsigned>(leader_st.health));
}

// ==================== 控制线程 ====================

void SwarmController::control_loop() {
    auto interval = std::chrono::milliseconds(1000 / cfg_.ctrl_hz);
    uint64_t tick = 0;

    while (running_) {
        auto t0 = std::chrono::steady_clock::now();

        // 1Hz：向飞控发伴机 GCS 心跳（编队控制）；飞控遥测经串口8->串口9 转发给 MP，无需单独发心跳
        if (tick % cfg_.ctrl_hz == 0) {
            mav_mgr_.send_heartbeats();
        }

        // 安全检查：飞控心跳丢失 -> 自动 RTL
        check_heartbeat_health();

        if (cfg_.role == NodeRole::LEADER) {
            run_leader_tick();
            try_finish_independent_leader_auto();
        } else {
            run_follower_tick();
        }

        process_arm_takeoff_state_machine();

        // 仅在 P900 长机兜底链路上启用「合成僚机遥测注入」；IP 主链下每机直连 MP，无需注入。
        if (cfg_.gcs_serial_enable && cfg_.role == NodeRole::LEADER &&
            !cfg_.follower_sysids.empty() &&
            (cfg_.gcs_link_type == SwarmConfig::GcsLinkType::LEGACY_P900_LEADER ||
             cfg_.gcs_link_type == SwarmConfig::GcsLinkType::P900))
            inject_follower_telemetry_to_gcs();

        // 安全保活：即使编队未启动，也要在 Guided 模式下周期发送当前位置
        // 防止 GUID_TIMEOUT 触发意外悬停/着陆
        send_guided_keepalive();
        flush_gcs_tx_queue(64, 8192);

        tick++;
        auto elapsed = std::chrono::steady_clock::now() - t0;
        if (elapsed < interval)
            std::this_thread::sleep_for(interval - elapsed);

        maybe_print_leader_timing_stats();
    }
}

void SwarmController::run_leader_tick() {
    if (gcs_param_fetch_active()) return;
    if (exec_mode_.load(std::memory_order_relaxed) != ExecMode::FORMATION) return;

    std::lock_guard<std::mutex> wp_lk(wp_mtx_);
    if (!formation_active_ || wp_num_ == 0) return;

    auto *fc = mav_mgr_.get_vehicle(cfg_.local_fc.sysid);
    if (!fc) return;

    const UnitState &leader = fc->state();

    navigate_leader();

    formation_.update_auto_formation(leader, waypoints_, wp_num_);

    auto all_states = mav_mgr_.get_all_states();
    std::unordered_map<int, UnitState> follower_states;
    {
        std::lock_guard<std::mutex> lk(follower_state_mtx_);
        for (int fid : cfg_.follower_sysids) {
            auto it = follower_state_cache_.find(fid);
            if (it != follower_state_cache_.end())
                follower_states[fid] = it->second;
        }
    }

    auto targets = formation_.compute(
        leader, waypoints_[cur_wp_],
        cfg_.follower_sysids, follower_states);

    for (auto &ft : targets) {
        LinkPacket pkt;
        pkt.msg_type = LinkMsgType::FOLLOWER_CMD;
        pkt.src_id   = cfg_.local_sysid;
        pkt.dst_id   = ft.sysid;
        pkt.lat      = ft.lat;
        pkt.lon      = ft.lon;
        pkt.alt      = ft.alt;
        pkt.vel      = ft.vel;
        pkt.hdg      = ft.hdg;
        air_link_->send_packet(pkt);
    }

    last_target_sent_ms_.store(steady_ms(), std::memory_order_relaxed);
}

void SwarmController::run_follower_tick() {
    auto *fc = mav_mgr_.get_vehicle(cfg_.local_fc.sysid);
    if (!fc) return;

    const UnitState &st = fc->state();
    LinkPacket pkt;
    pkt.msg_type = LinkMsgType::STATE_REPORT;
    pkt.src_id   = cfg_.local_sysid;
    pkt.dst_id   = cfg_.leader_sysid;
    pkt.lat      = st.lat;
    pkt.lon      = st.lon;
    pkt.alt      = st.alt_rel;
    pkt.vel      = st.vel_cruise;
    pkt.hdg      = st.heading;
    pkt.pitch    = st.pitch;
    pkt.roll     = st.roll;
    pkt.health   = st.health;
    pkt.armed    = st.armed;
    pkt.mode_valid = 1;
    pkt.custom_mode = st.custom_mode;
    if (air_link_->send_packet(pkt) < 0)
        follower_state_report_tx_fail_.fetch_add(1, std::memory_order_relaxed);
}

void SwarmController::navigate_leader() {
    auto *fc = mav_mgr_.get_vehicle(cfg_.local_fc.sysid);
    if (!fc || wp_num_ == 0) return;

    const UnitState &st = fc->state();
    double dis = distance_between(st.lat, st.lon,
                                  waypoints_[cur_wp_].lat,
                                  waypoints_[cur_wp_].lon);

    // 多旋翼到点半径较小
    double reach_radius = (cfg_.vehicle_type == VehicleType::COPTER) ? 3.0 : 30.0;
    if (dis < reach_radius) {
        int prev = cur_wp_;
        cur_wp_++;
        if (cur_wp_ >= wp_num_) cur_wp_ = 0;
        printf("[swarm] leader reached WP %d (d=%.1fm), next=%d\n", prev, dis, cur_wp_);
    }

    double lat_deg = waypoints_[cur_wp_].lat * SWARM_RAD2DEG;
    double lon_deg = waypoints_[cur_wp_].lon * SWARM_RAD2DEG;
    fc->set_position_target_global(lat_deg, lon_deg,
                                   static_cast<float>(waypoints_[cur_wp_].alt));
}

// ==================== 安全机制 ====================

void SwarmController::send_guided_keepalive() {
    auto *fc = mav_mgr_.get_vehicle(cfg_.local_fc.sysid);
    if (!fc) return;

    uint64_t now = steady_ms();
    // GUID_TIMEOUT 的一半时间内必须发一次，确保不超时
    uint64_t max_gap_ms = static_cast<uint64_t>(cfg_.guid_timeout * 500);
    if (max_gap_ms < 200) max_gap_ms = 200;

    if (now - last_target_sent_ms_.load(std::memory_order_relaxed) > max_gap_ms) {
        const UnitState &st = fc->state();
        if (st.armed && st.custom_mode == static_cast<uint32_t>(CopterMode::GUIDED)) {
            double lat_deg = st.lat * SWARM_RAD2DEG;
            double lon_deg = st.lon * SWARM_RAD2DEG;
            fc->set_position_target_global(lat_deg, lon_deg, st.alt_rel);
            last_target_sent_ms_.store(now, std::memory_order_relaxed);
        }
    }
}

void SwarmController::check_heartbeat_health() {
    auto *fc = mav_mgr_.get_vehicle(cfg_.local_fc.sysid);
    if (!fc) return;

    uint64_t now = steady_ms();
    uint64_t hb_age = now - fc->last_heartbeat_ms();

    // 飞控心跳丢失超过 5 秒 -> 触发安全 RTL
    if (fc->last_heartbeat_ms() > 0 && hb_age > 5000) {
        if (safety_rtl_triggered_ms_.load(std::memory_order_relaxed) == 0) {
            fprintf(stderr, "[SAFETY] FC heartbeat lost for %.1fs, triggering RTL!\n",
                    hb_age * 0.001);
            fc->rtl();
            formation_active_ = false;
            safety_rtl_triggered_ms_.store(now, std::memory_order_relaxed);
        }
    } else {
        safety_rtl_triggered_ms_.store(0, std::memory_order_relaxed);
    }
}

// ==================== 外部指令 ====================

void SwarmController::cmd_upload_waypoints(const std::vector<WayPoint> &wps) {
    std::lock_guard<std::mutex> lk(wp_mtx_);
    wp_num_ = std::min(static_cast<int>(wps.size()), MAX_WAYPOINTS);
    for (int i = 0; i < wp_num_; i++)
        waypoints_[i] = wps[i];
    cur_wp_ = 0;
    printf("[swarm] %d waypoints loaded\n", wp_num_);
}

void SwarmController::cmd_start_formation() {
    auto *fc = mav_mgr_.get_vehicle(cfg_.local_fc.sysid);
    if (!fc) return;

    if (cfg_.vehicle_type == VehicleType::COPTER)
        fc->set_copter_mode(CopterMode::GUIDED);
    else
        fc->set_plane_mode(PlaneMode::GUIDED);
    log_leader_fc_mode_state("leader switch to GUIDED");

    formation_active_ = true;
    last_target_sent_ms_.store(steady_ms(), std::memory_order_relaxed);
    log_cmd_info("[swarm] formation STARTED (Guided mode)\n");
}

void SwarmController::cmd_stop_formation() {
    formation_active_ = false;
    auto *fc = mav_mgr_.get_vehicle(cfg_.local_fc.sysid);
    if (fc) {
        if (cfg_.vehicle_type == VehicleType::COPTER)
            fc->set_copter_mode(CopterMode::LOITER);
        else
            fc->set_plane_mode(PlaneMode::LOITER);
        log_leader_fc_mode_state("leader switch to LOITER");
    }
    log_cmd_info("[swarm] formation STOPPED (-> Loiter)\n");
}

void SwarmController::cmd_set_formation_params(const FormationParams &p) {
    formation_.set_params(p);
    printf("[swarm] formation params updated: dis=%.0f angle=%.0fdeg dH=%.0f\n",
           p.delta_dis, p.angle * SWARM_RAD2DEG, p.delta_H);
}

void SwarmController::cmd_arm_and_takeoff(float alt) {
    std::lock_guard<std::mutex> lk(arm_takeoff_mtx_);
    arm_takeoff_active_ = true;
    arm_takeoff_alt_m_ = alt;
    arm_takeoff_stage_ = ArmTakeoffStage::WAIT_MODE;
    arm_takeoff_deadline_ms_ = steady_ms() + 1000;
    log_cmd_info("[swarm] arm & takeoff scheduled (non-blocking): %.1f m\n", alt);
}

void SwarmController::process_arm_takeoff_state_machine() {
    auto *fc = mav_mgr_.get_vehicle(cfg_.local_fc.sysid);
    if (!fc) return;

    std::lock_guard<std::mutex> lk(arm_takeoff_mtx_);
    if (!arm_takeoff_active_)
        return;

    const uint64_t now = steady_ms();
    switch (arm_takeoff_stage_) {
    case ArmTakeoffStage::WAIT_MODE:
        if (cfg_.vehicle_type == VehicleType::COPTER)
            fc->set_copter_mode(CopterMode::GUIDED);
        else
            fc->set_plane_mode(PlaneMode::GUIDED);
        log_leader_fc_mode_state("leader switch to GUIDED");
        if (now < arm_takeoff_deadline_ms_)
            return;
        fc->arm(true);
        arm_takeoff_stage_ = ArmTakeoffStage::WAIT_ARM;
        arm_takeoff_deadline_ms_ = now + 1000;
        return;
    case ArmTakeoffStage::WAIT_ARM:
        if (now < arm_takeoff_deadline_ms_)
            return;
        fc->takeoff(arm_takeoff_alt_m_);
        last_target_sent_ms_.store(now, std::memory_order_relaxed);
        arm_takeoff_active_ = false;
        arm_takeoff_stage_ = ArmTakeoffStage::IDLE;
        log_cmd_info("[swarm] arm & takeoff executed: %.1f m\n", arm_takeoff_alt_m_);
        return;
    case ArmTakeoffStage::IDLE:
    default:
        arm_takeoff_active_ = false;
        return;
    }
}

void SwarmController::cmd_land() {
    auto *fc = mav_mgr_.get_vehicle(cfg_.local_fc.sysid);
    if (fc) {
        formation_active_ = false;
        fc->land();
        log_cmd_info("[swarm] LAND commanded\n");
    }
}

void SwarmController::cmd_rtl() {
    auto *fc = mav_mgr_.get_vehicle(cfg_.local_fc.sysid);
    if (fc) {
        formation_active_ = false;
        fc->rtl();
        log_cmd_info("[swarm] RTL commanded\n");
    }
}

// ==================== GCS 链路构造 / NAT 心跳 / 健康监视 ====================

void SwarmController::build_gcs_link_() {
    gcs_ip_link_ptr_ = nullptr;
    switch (cfg_.gcs_link_type) {
    case SwarmConfig::GcsLinkType::IP: {
        auto ip_link = std::make_unique<GcsIpLink>(
            cfg_.gcs_udp_bind_ip, cfg_.gcs_udp_bind_port,
            cfg_.gcs_udp_target_ip, cfg_.gcs_udp_target_port);
        // 配置 NAT 保活心跳，1Hz 起步
        const int hz = std::max(0, cfg_.gcs_heartbeat_hz);
        if (hz > 0) {
            ip_link->set_heartbeat(hz,
                [this](uint8_t *wire, uint16_t max_len) -> uint16_t {
                    return build_nat_heartbeat_(wire, max_len);
                });
        }
        gcs_ip_link_ptr_ = ip_link.get();
        gcs_link_ = std::move(ip_link);
        break;
    }
    case SwarmConfig::GcsLinkType::SERIAL:
        gcs_link_ = std::make_unique<GcsSerialLink>(
            gcs_serial_, cfg_.gcs_serial_dev, cfg_.gcs_serial_baud);
        break;
    case SwarmConfig::GcsLinkType::P900:
    case SwarmConfig::GcsLinkType::LEGACY_P900_LEADER:
        gcs_link_ = std::make_unique<GcsP900Link>(link_p900_);
        break;
    case SwarmConfig::GcsLinkType::NONE:
    default:
        gcs_link_.reset();
        break;
    }
}

uint16_t SwarmController::build_nat_heartbeat_(uint8_t *wire, uint16_t max_len) {
    if (wire == nullptr || max_len < MAVLINK_MAX_PACKET_LEN) return 0;
    // 用伴飞 sysid 上报一帧 HEARTBEAT，向地面 MP 与运营商 NAT 表注入活性。
    // type 选 ONBOARD_CONTROLLER 以区分飞控自身的 HEARTBEAT。
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        static_cast<uint8_t>(cfg_.local_sysid),
        MAV_COMP_ID_ONBOARD_COMPUTER,
        &msg,
        MAV_TYPE_ONBOARD_CONTROLLER,
        MAV_AUTOPILOT_INVALID,
        MAV_MODE_FLAG_SAFETY_ARMED,
        0,
        MAV_STATE_ACTIVE);
    return mavlink_msg_to_send_buffer(wire, &msg);
}

void SwarmController::send_health_statustext_(const std::string &text,
                                               uint8_t severity) {
    if (!cfg_.gcs_serial_enable) return;
    mavlink_message_t msg;
    char buf[50] = {0};
    std::snprintf(buf, sizeof(buf), "%s", text.c_str());
    mavlink_msg_statustext_pack(
        static_cast<uint8_t>(cfg_.local_sysid),
        MAV_COMP_ID_ONBOARD_COMPUTER,
        &msg,
        severity,
        buf,
        0, 0);
    enqueue_gcs_mavlink(msg, GcsTxPriority::HIGH);
}

bool SwarmController::start_p900_fallback_() {
    if (!cfg_.p900_uart9_fallback) return false;
    if (p900_fallback_engaged_.load(std::memory_order_relaxed)) return true;
    if (cfg_.p900_uart9_dev.empty()) {
        fprintf(stderr, "[swarm] p900 fallback requested but p900-dev empty\n");
        return false;
    }

    if (!link_p900_.is_open()) {
        if (!link_p900_.open(cfg_.p900_uart9_dev, cfg_.p900_uart9_baud)) {
            fprintf(stderr, "[swarm] p900 fallback: open %s@%d failed\n",
                    cfg_.p900_uart9_dev.c_str(), cfg_.p900_uart9_baud);
            return false;
        }
        link_p900_.set_local_identity(static_cast<uint8_t>(cfg_.local_sysid));
    }

    // 切机间 air_link_ 与 GCS gcs_link_ 走 P900；
    // 注意：原 5G UDP socket 保留（不关闭），健康恢复后可立刻切回。
    air_link_ = &link_p900_;
    gcs_link_ = std::make_unique<GcsP900Link>(link_p900_);
    gcs_ip_link_ptr_ = nullptr;
    cfg_.gcs_link_type = SwarmConfig::GcsLinkType::LEGACY_P900_LEADER;
    cfg_.follower_synthetic_telem_to_gcs = (cfg_.role == NodeRole::LEADER);
    p900_fallback_engaged_.store(true, std::memory_order_relaxed);
    log_cmd_info("[swarm] p900 fallback engaged (dev=%s baud=%d)\n",
                 cfg_.p900_uart9_dev.c_str(), cfg_.p900_uart9_baud);
    return true;
}

void SwarmController::on_health_state_changed_(uint8_t new_state, uint8_t old_state,
                                               const std::string &reason) {
    const auto to   = static_cast<HealthMonitor::State>(new_state);
    const auto from = static_cast<HealthMonitor::State>(old_state);
    (void)from;
    (void)reason;

    auto apply = [this](SwarmConfig::DegradeAction action) {
        switch (action) {
        case SwarmConfig::DegradeAction::LOITER: {
            if (degrade_loiter_armed_.exchange(true)) return;
            auto *fc = mav_mgr_.get_vehicle(cfg_.local_fc.sysid);
            if (fc) {
                formation_active_ = false;
                if (cfg_.vehicle_type == VehicleType::COPTER)
                    fc->set_copter_mode(CopterMode::LOITER);
                else
                    fc->set_plane_mode(PlaneMode::LOITER);
                log_cmd_info("[5GHealth] DEGRADE -> LOITER\n");
            }
            break;
        }
        case SwarmConfig::DegradeAction::RTL: {
            if (degrade_rtl_armed_.exchange(true)) return;
            cmd_rtl();
            log_cmd_info("[5GHealth] DEGRADE -> RTL\n");
            break;
        }
        case SwarmConfig::DegradeAction::STATUSTEXT:
        case SwarmConfig::DegradeAction::NONE:
        default:
            break;
        }
    };

    if (to == HealthMonitor::State::HEALTHY) {
        degrade_loiter_armed_.store(false);
        degrade_rtl_armed_.store(false);
        log_cmd_info("[5GHealth] recovered to HEALTHY\n");
        return;
    }

    if (to == HealthMonitor::State::DEGRADED) {
        apply(cfg_.degrade_on_degraded);
    } else if (to == HealthMonitor::State::LOST) {
        apply(cfg_.degrade_on_lost);
        if (cfg_.p900_uart9_fallback) start_p900_fallback_();
    }
}
