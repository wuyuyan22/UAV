#ifndef SWARM_CONTROLLER_H
#define SWARM_CONTROLLER_H

#include "common/types.h"
#include "mavlink_adapter/mavlink_manager.h"
#include "swarm/formation.h"
#include "swarm/mission_splitter.h"
#include "comm/inter_uav_link.h"
#include "comm/gcs_link.h"
#include "comm/gcs_ip_link.h"
#include "comm/health_monitor.h"
#include "comm/p900_framed_link.h"
#include "hal/serial_port.h"

#include <atomic>
#include <thread>
#include <vector>
#include <functional>
#include <unordered_map>
#include <mutex>
#include <atomic>
#include <cstdint>
#include <deque>
#include <array>
#include <memory>
#include <string>

#include <ardupilotmega/mavlink.h>

struct SwarmConfig {
    enum class GcsLinkType : uint8_t {
        IP = 0,
        SERIAL = 1,
        P900 = 2,          ///< 普通串口直挂 P900 透传（一期 legacy）
        LEGACY_P900_LEADER = 3,  ///< P900 UART9 复用模式下长机合成遥测注入
        NONE = 4,
    };

    /** 主链 5G 异常时的自动应对策略。 */
    enum class DegradeAction : uint8_t {
        NONE          = 0,
        STATUSTEXT    = 1,  ///< 仅上报告警
        LOITER        = 2,  ///< 切 LOITER 保位悬停
        RTL           = 3,  ///< 切 RTL 自动返航
    };

    int         local_sysid    = 1;
    NodeRole    role           = NodeRole::LEADER;
    VehicleType vehicle_type   = VehicleType::COPTER;
    int         leader_sysid   = 1;
    std::vector<int> follower_sysids;

    int         ctrl_hz        = 10;     // 控制频率 Hz (Guided 模式要求 >=2Hz)
    bool        simulation     = false;
    float       guid_timeout   = 3.0f;   // 匹配飞控 GUID_TIMEOUT 参数
    float       takeoff_alt    = 5.0f;   // 默认起飞高度 m

    VehicleConfig local_fc;
    bool         gcs_serial_enable = false;
    std::string  gcs_serial_dev    = "/dev/ttyS9";
    int          gcs_serial_baud   = 57600;
    GcsLinkType  gcs_link_type     = GcsLinkType::IP;
    std::string  gcs_udp_target_ip = "10.8.0.100";
    int          gcs_udp_target_port = 14550;
    std::string  gcs_udp_bind_ip   = "0.0.0.0";
    int          gcs_udp_bind_port = 0;
    uint32_t     gcs_tx_max_kbps   = 384;
    int          gcs_heartbeat_hz  = 1;
    bool         follower_synthetic_telem_to_gcs = false;

    // 5G 主链健康监视与自动降级
    bool          health_enable          = true;
    uint64_t      health_warn_ms         = 3000;   ///< DEGRADED 阈值（无回包毫秒）
    uint64_t      health_lost_ms         = 8000;   ///< LOST 阈值
    DegradeAction degrade_on_degraded    = DegradeAction::STATUSTEXT;
    DegradeAction degrade_on_lost        = DegradeAction::LOITER;
    bool          p900_uart9_fallback    = false;  ///< LOST 时切 P900 兜底（需提前配置 p900-dev/baud）

    std::string  link_listen_ip   = "0.0.0.0";
    int          link_listen_port = 19870;
    std::string  link_target_ip   = "192.168.1.1";
    int          link_target_port = 19870;

    /** true：机间与（长机）地面站共用 P900 成帧透传，串口见 p900_uart9_dev（默认 UART9） */
    bool        p900_uart9_mode = false;
    std::string p900_uart9_dev  = "/dev/ttyS9";
    int         p900_uart9_baud = 57600;
};

class SwarmController {
public:
    explicit SwarmController(const SwarmConfig &cfg);
    ~SwarmController();

    bool init();
    void start();
    void stop();

    // 外部指令接口
    void cmd_upload_waypoints(const std::vector<WayPoint> &wps);
    void cmd_start_formation();
    void cmd_stop_formation();
    void cmd_set_formation_params(const FormationParams &p);
    void cmd_arm_and_takeoff(float alt);
    void cmd_land();
    void cmd_rtl();

    const UnitState &local_state() const;
    bool is_running() const { return running_; }

private:
    enum class ArmTakeoffStage : uint8_t {
        IDLE = 0,
        WAIT_MODE = 1,
        WAIT_ARM = 2,
    };
    enum class GcsTxPriority : uint8_t {
        HIGH = 0,
        NORMAL = 1,
    };

    struct TimingCounter {
        uint64_t count = 0;
        uint64_t fail_count = 0;
        uint64_t total_us = 0;
        uint64_t min_us = 0;
        uint64_t max_us = 0;
        uint64_t bytes = 0;
    };

    struct LeaderTimingStats {
        TimingCounter gcs_msg_dispatch;
        TimingCounter gcs_to_fc_write;
        TimingCounter gcs_to_air_send;
        TimingCounter air_recv_handle;
        TimingCounter fc_to_gcs_send;
        uint64_t window_start_ms = 0;
        uint64_t last_report_ms = 0;
    };

    enum class ExecMode : uint8_t {
        FORMATION = 1,
        INDEPENDENT = 2,
    };

    SwarmConfig         cfg_;
    MavlinkManager      mav_mgr_;
    FormationCalculator formation_;
    MissionSplitter     mission_splitter_;
    InterUavLink        link_udp_;
    P900FramedLink      link_p900_;
    ISwarmAirLink      *air_link_ = nullptr;

    WayPoint  waypoints_[MAX_WAYPOINTS];
    int       wp_num_ = 0;
    int       cur_wp_ = 0;
    std::mutex wp_mtx_;  // 保护 waypoints_ / wp_num_ / cur_wp_（GCS 任务解析线程 vs 控制线程）

    // MP 任务上传缓冲（仅 Leader 使用）：MISSION_COUNT 后按 seq 收齐 MISSION_ITEM_INT
    uint16_t              gcs_mission_expected_ = 0;
    std::vector<WayPoint> gcs_mission_rx_buf_;
    std::vector<uint8_t>  gcs_mission_rx_got_;

    std::atomic<bool>   running_{false};
    std::atomic<bool>   formation_active_{false};

    std::thread         ctrl_thread_;
    std::thread         recv_thread_;

    void control_loop();
    void receive_loop();
    void process_arm_takeoff_state_machine();

    void run_leader_tick();
    void run_follower_tick();

    void navigate_leader();
    void poll_gcs_link();
    void handle_gcs_command_long(const mavlink_message_t &msg);
    /** 解析 Mission Planner 下发的任务航点（MISSION_COUNT / MISSION_ITEM_INT），写入 waypoints_ */
    void handle_gcs_mission_count(const mavlink_message_t &msg);
    void handle_gcs_mission_item_int(const mavlink_message_t &msg);
    void handle_gcs_mission_clear_all(const mavlink_message_t &msg);
    void reset_gcs_mission_rx();
    void log_leader_fc_mode_state(const char *scene);
    void set_exec_mode(ExecMode mode);
    void start_exec();
    void stop_exec();
    void distribute_independent_mission();
    void try_finish_independent_leader_auto();
    void handle_follower_task_packets(const LinkPacket &pkt);
    void reset_follower_task_rx();

    bool follower_sysid_contains(int sysid) const;
    bool try_proxy_gcs_command_to_follower(const mavlink_message_t &msg);
    bool try_proxy_gcs_command_int_to_follower(const mavlink_message_t &msg);
    bool try_proxy_gcs_set_mode_to_follower(const mavlink_message_t &msg);
    void execute_follower_ctrl_cmd(const LinkPacket &pkt);

    void send_gcs_command_ack(const mavlink_message_t &cmd_msg,
                              uint16_t cmd_id,
                              uint8_t result = MAV_RESULT_ACCEPTED);
    void enqueue_gcs_mavlink(const mavlink_message_t &msg,
                             GcsTxPriority prio = GcsTxPriority::NORMAL);
    void enqueue_gcs_bytes(const uint8_t *buf, uint16_t len,
                           GcsTxPriority prio = GcsTxPriority::NORMAL);
    void flush_gcs_tx_queue(size_t max_packets = 32, size_t max_bytes = 4096);
    void maybe_print_gcs_tx_queue_stats();

    // 安全：Guided 保活（确保 >=2Hz 发送目标，防止 GUID_TIMEOUT 触发悬停）
    void send_guided_keepalive();
    // 安全：心跳丢失检测 -> 自动 RTL
    void check_heartbeat_health();

    std::atomic<uint64_t> last_target_sent_ms_{0};  // 上次下发目标的时间
    std::atomic<uint64_t> safety_rtl_triggered_ms_{0};
    std::atomic<ExecMode> exec_mode_{ExecMode::FORMATION};

    // follower 侧：接收 leader 下发的子任务并上传给本机飞控
    uint32_t follower_task_version_ = 0;
    uint16_t follower_task_expected_ = 0;
    std::vector<WayPoint> follower_task_buf_;
    std::vector<uint8_t>  follower_task_got_;
    bool follower_task_pending_auto_start_ = false;

    /** 非编队：长机子任务已 upload_mission，等待 MISSION_ACK 后切 AUTO 并向僚机发 TASK_START */
    std::atomic<bool> leader_pending_independent_auto_{false};
    /** 长机子任务为空时仅向僚机广播 TASK_START */
    std::atomic<bool> independent_follower_start_pending_{false};
    std::atomic<uint32_t> independent_task_version_{0};

    // leader 需要实时掌握 follower 的位置，用于距离/速度自适应。
    std::unordered_map<int, UnitState> follower_state_cache_;
    /** 长机：最近一次收到各僚机 STATE_REPORT 的时间与累计次数（与 follower_state_mtx_ 同锁） */
    std::unordered_map<int, uint64_t> follower_link_last_rx_ms_;
    std::unordered_map<int, uint64_t> follower_link_rx_total_;
    mutable std::mutex follower_state_mtx_;

    /** 僚机：最近一次收到长机机间包的时间；STATE_REPORT 连续发送失败次数 */
    std::atomic<uint64_t> follower_last_rx_from_leader_ms_{0};
    std::atomic<uint64_t> follower_state_report_tx_fail_{0};

    /** 飞控解析帧 -> MP：重打包写出，并对高频遥测节流，避免占满串口 */
    void forward_fc_parsed_to_gcs(const mavlink_message_t &msg);
    /** 仅 Leader：将僚机缓存状态打成 MAVLink 发给 MP（显示/地图；改参仍须直连各机） */
    void inject_follower_telemetry_to_gcs();

    /** MP 经串口发起读参时置位，超时后自动恢复 */
    bool gcs_param_fetch_active() const;
    void mark_gcs_param_fetch_from_gcs();
    std::atomic<uint64_t> gcs_param_fetch_last_ms_{0};

    std::mutex gcs_tx_mtx_;
    std::unordered_map<uint32_t, uint64_t> gcs_fwd_throttle_ms_;
    std::atomic<uint64_t> follower_telem_last_ms_{0};

    struct QueuedGcsTxFrame {
        std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> bytes{};
        uint16_t len = 0;
    };
    std::mutex gcs_tx_queue_mtx_;
    std::deque<QueuedGcsTxFrame> gcs_tx_queue_high_;
    std::deque<QueuedGcsTxFrame> gcs_tx_queue_normal_;
    size_t gcs_tx_queue_max_frames_ = 512;
    uint32_t gcs_tx_token_rate_bps_ = 0;
    uint32_t gcs_tx_token_capacity_ = 0;
    uint32_t gcs_tx_tokens_ = 0;
    uint64_t gcs_tx_tokens_last_refill_ms_ = 0;
    uint64_t gcs_tx_diag_last_ms_ = 0;
    uint64_t gcs_tx_enqueued_high_ = 0;
    uint64_t gcs_tx_enqueued_normal_ = 0;
    uint64_t gcs_tx_dropped_high_ = 0;
    uint64_t gcs_tx_dropped_normal_ = 0;
    uint64_t gcs_tx_sent_packets_ = 0;
    uint64_t gcs_tx_sent_bytes_ = 0;

    std::mutex arm_takeoff_mtx_;
    bool arm_takeoff_active_ = false;
    float arm_takeoff_alt_m_ = 0.0f;
    ArmTakeoffStage arm_takeoff_stage_ = ArmTakeoffStage::IDLE;
    uint64_t arm_takeoff_deadline_ms_ = 0;

    std::unique_ptr<IGcsLink> gcs_link_;
    GcsIpLink *gcs_ip_link_ptr_ = nullptr;  ///< 仅 IP 模式下指向 gcs_link_，便于 HealthMonitor 取统计
    SerialPort gcs_serial_;
    mavlink_status_t gcs_mav_status_{};

    HealthMonitor health_monitor_;
    std::atomic<bool> p900_fallback_engaged_{false};
    std::atomic<bool> degrade_loiter_armed_{false};
    std::atomic<bool> degrade_rtl_armed_{false};

    void build_gcs_link_();
    bool start_p900_fallback_();
    void send_health_statustext_(const std::string &text, uint8_t severity);
    void on_health_state_changed_(uint8_t new_state, uint8_t old_state,
                                  const std::string &reason);
    uint16_t build_nat_heartbeat_(uint8_t *wire, uint16_t max_len);

    mutable std::mutex timing_mtx_;
    LeaderTimingStats leader_timing_;
    void timing_record(TimingCounter &c, uint64_t us, bool ok, uint64_t bytes = 0);
    void maybe_print_leader_timing_stats();
};

#endif
