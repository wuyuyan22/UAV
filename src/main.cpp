#include "swarm/swarm_controller.h"
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <thread>
#include <chrono>

static SwarmController *g_ctrl = nullptr;

static void signal_handler(int) {
    printf("\n[main] signal received, stopping...\n");
    if (g_ctrl) g_ctrl->stop();
}

static void print_usage(const char *prog) {
    printf("=== UAV Swarm Controller for LubanCat0 + ArduPilot ===\n");
    printf("Usage: %s <sysid> <role> [options]\n\n", prog);
    printf("  sysid   : MAVLink system ID (1-255)\n");
    printf("  role    : 1=LEADER  2=FOLLOWER\n\n");
    printf("Options:\n");
    printf("  --serial <dev>     FC serial device    (default: /dev/ttyS0, LuBanCat4 pin 8/10 = UART0_M2)\n");
    printf("  --baud <rate>      serial baud rate     (default: 57600)\n");
    printf("  --udp <ip:port>    FC via UDP/SITL      (overrides serial)\n");
    printf("  --type <copter|plane>  vehicle type     (default: copter)\n");
    printf("  --gcs-link <ip|serial|p900|none> (default: ip)\n");
    printf("  --gcs-udp-target <ip:port>       (default: 10.8.0.100:14550)\n");
    printf("  --gcs-udp-bind <ip:port>         (default: 0.0.0.0:0)\n");
    printf("  --gcs-udp-tx-kbps <n>            (default: 384)\n");
    printf("  --gcs-heartbeat-hz <n>           NAT keep-alive HEARTBEAT rate (default: 1)\n");
    printf("  --gcs-serial <dev>  GCS serial path      (default: /dev/ttyS9)\n");
    printf("  --gcs-baud <rate>   GCS serial baud      (default: 57600)\n");
    printf("  --no-follower-synthetic-telem-gcs  disable leader synthetic follower telem injection\n");
    printf("  --follower-synthetic-telem-gcs     enable  leader synthetic follower telem injection (P900 fallback)\n");
    printf("  --link-port <p>    inter-UAV listen     (default: 19870)\n");
    printf("  --leader-ip <ip>   leader address       (follower only)\n");
    printf("  --hz <n>           control frequency    (default: 10, min 2)\n");
    printf("  --followers <a,b>  follower sysids      (leader only, e.g. 2,3)\n");
    printf("  --p900-uart9       P900 framed mux on UART9: air link + (leader) GCS\n");
    printf("  --p900-dev <dev>   P900 serial device    (default: /dev/ttyS9)\n");
    printf("  --p900-baud <n>    P900 baud             (default: same as gcs-baud)\n");
    printf("  --p900-uart9-fallback  arm runtime P900 fallback when 5G link lost\n");
    printf("  --health-disable      disable 5G link health monitor (default: enabled)\n");
    printf("  --health-warn-ms <n>  recv silence -> DEGRADED threshold ms (default: 3000)\n");
    printf("  --health-lost-ms <n>  recv silence -> LOST threshold ms     (default: 8000)\n");
    printf("  --degrade-on-degraded <none|status|loiter|rtl>  action on DEGRADED (default: status)\n");
    printf("  --degrade-on-lost     <none|status|loiter|rtl>  action on LOST     (default: loiter)\n");
    printf("  --sim              simulation mode\n");
}

static SwarmConfig::DegradeAction parse_degrade_action(const std::string &s) {
    if (s == "none")   return SwarmConfig::DegradeAction::NONE;
    if (s == "status") return SwarmConfig::DegradeAction::STATUSTEXT;
    if (s == "loiter") return SwarmConfig::DegradeAction::LOITER;
    if (s == "rtl")    return SwarmConfig::DegradeAction::RTL;
    return SwarmConfig::DegradeAction::STATUSTEXT;
}

static bool parse_ip_port(const std::string &addr, std::string &ip, int &port) {
    const auto colon = addr.find(':');
    if (colon == std::string::npos) return false;
    ip = addr.substr(0, colon);
    port = atoi(addr.substr(colon + 1).c_str());
    return !ip.empty() && port >= 0;
}

int main(int argc, char *argv[]) {
    if (argc < 3) { print_usage(argv[0]); return 1; }

    SwarmConfig cfg;
    cfg.local_sysid = atoi(argv[1]);
    int role_int    = atoi(argv[2]);
    cfg.role = (role_int == 1) ? NodeRole::LEADER : NodeRole::FOLLOWER;

    // 默认配置（匹配鲁班猫0硬件方案文档）
    cfg.local_fc.sysid       = cfg.local_sysid;
    cfg.local_fc.compid      = 1;
    cfg.local_fc.link        = LinkType::SERIAL;
    // LuBanCat4：40-pin pin 8/10 → UART0_M2 → /dev/ttyS0（TX=GPIO4_A3, RX=GPIO4_A4）
    // 其他板型请用 --serial 覆盖
    cfg.local_fc.serial_dev  = "/dev/ttyS0";
    cfg.local_fc.serial_baud = 57600;           // 与飞控 SERIAL1_BAUD 一致
    cfg.local_fc.vehicle_type = VehicleType::COPTER;
    cfg.vehicle_type         = VehicleType::COPTER;
    cfg.gcs_serial_enable    = true;
    cfg.gcs_serial_dev       = "/dev/ttyS9";
    cfg.gcs_serial_baud      = 57600;
    cfg.gcs_link_type        = SwarmConfig::GcsLinkType::IP;
    cfg.gcs_udp_target_ip    = "10.8.0.100";
    cfg.gcs_udp_target_port  = 14550;
    cfg.gcs_udp_bind_ip      = "0.0.0.0";
    cfg.gcs_udp_bind_port    = 0;
    cfg.gcs_tx_max_kbps      = 384;
    cfg.follower_synthetic_telem_to_gcs = false;

    cfg.leader_sysid = 1;
    if (cfg.role == NodeRole::LEADER) {
        cfg.follower_sysids = {2, 3};
    }

    for (int i = 3; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--serial" && i + 1 < argc) {
            cfg.local_fc.serial_dev = argv[++i];
        } else if (arg == "--baud" && i + 1 < argc) {
            cfg.local_fc.serial_baud = atoi(argv[++i]);
        } else if (arg == "--udp" && i + 1 < argc) {
            cfg.local_fc.link = LinkType::UDP;
            std::string addr = argv[++i];
            auto colon = addr.find(':');
            if (colon != std::string::npos) {
                cfg.local_fc.udp_ip   = addr.substr(0, colon);
                cfg.local_fc.udp_port = atoi(addr.substr(colon + 1).c_str());
            } else {
                cfg.local_fc.udp_ip   = addr;
                cfg.local_fc.udp_port = 14550;
            }
        } else if (arg == "--type" && i + 1 < argc) {
            std::string t = argv[++i];
            if (t == "plane") {
                cfg.vehicle_type = VehicleType::PLANE;
                cfg.local_fc.vehicle_type = VehicleType::PLANE;
            }
        } else if (arg == "--gcs-serial" && i + 1 < argc) {
            cfg.gcs_serial_enable = true;
            cfg.gcs_link_type = SwarmConfig::GcsLinkType::SERIAL;
            cfg.gcs_serial_dev = argv[++i];
        } else if (arg == "--gcs-link" && i + 1 < argc) {
            const std::string v = argv[++i];
            if (v == "ip") {
                cfg.gcs_link_type = SwarmConfig::GcsLinkType::IP;
                cfg.gcs_serial_enable = true;
            } else if (v == "serial") {
                cfg.gcs_link_type = SwarmConfig::GcsLinkType::SERIAL;
                cfg.gcs_serial_enable = true;
            } else if (v == "p900") {
                cfg.gcs_link_type = SwarmConfig::GcsLinkType::P900;
                cfg.gcs_serial_enable = true;
            } else if (v == "none") {
                cfg.gcs_link_type = SwarmConfig::GcsLinkType::NONE;
                cfg.gcs_serial_enable = false;
            }
        } else if (arg == "--gcs-udp-target" && i + 1 < argc) {
            std::string ip;
            int port = 0;
            if (parse_ip_port(argv[++i], ip, port)) {
                cfg.gcs_udp_target_ip = ip;
                cfg.gcs_udp_target_port = port;
            }
        } else if (arg == "--gcs-udp-bind" && i + 1 < argc) {
            std::string ip;
            int port = 0;
            if (parse_ip_port(argv[++i], ip, port)) {
                cfg.gcs_udp_bind_ip = ip;
                cfg.gcs_udp_bind_port = port;
            }
        } else if (arg == "--gcs-udp-tx-kbps" && i + 1 < argc) {
            cfg.gcs_tx_max_kbps = static_cast<uint32_t>(atoi(argv[++i]));
        } else if (arg == "--gcs-heartbeat-hz" && i + 1 < argc) {
            cfg.gcs_heartbeat_hz = atoi(argv[++i]);
        } else if (arg == "--p900-uart9-fallback") {
            cfg.p900_uart9_fallback = true;
        } else if (arg == "--health-disable") {
            cfg.health_enable = false;
        } else if (arg == "--health-warn-ms" && i + 1 < argc) {
            cfg.health_warn_ms = static_cast<uint64_t>(atoll(argv[++i]));
        } else if (arg == "--health-lost-ms" && i + 1 < argc) {
            cfg.health_lost_ms = static_cast<uint64_t>(atoll(argv[++i]));
        } else if (arg == "--degrade-on-degraded" && i + 1 < argc) {
            cfg.degrade_on_degraded = parse_degrade_action(argv[++i]);
        } else if (arg == "--degrade-on-lost" && i + 1 < argc) {
            cfg.degrade_on_lost = parse_degrade_action(argv[++i]);
        } else if (arg == "--no-follower-synthetic-telem-gcs") {
            cfg.follower_synthetic_telem_to_gcs = false;
        } else if (arg == "--follower-synthetic-telem-gcs") {
            cfg.follower_synthetic_telem_to_gcs = true;
        } else if (arg == "--gcs-baud" && i + 1 < argc) {
            cfg.gcs_serial_baud = atoi(argv[++i]);
        } else if (arg == "--no-gcs-serial") {
            cfg.gcs_serial_enable = false;
            cfg.gcs_link_type = SwarmConfig::GcsLinkType::NONE;
        } else if (arg == "--link-port" && i + 1 < argc) {
            cfg.link_listen_port = atoi(argv[++i]);
        } else if (arg == "--leader-ip" && i + 1 < argc) {
            cfg.link_target_ip = argv[++i];
        } else if (arg == "--hz" && i + 1 < argc) {
            cfg.ctrl_hz = atoi(argv[++i]);
        } else if (arg == "--followers" && i + 1 < argc) {
            cfg.follower_sysids.clear();
            std::string fstr = argv[++i];
            size_t pos = 0;
            while (pos < fstr.size()) {
                auto comma = fstr.find(',', pos);
                std::string tok = (comma != std::string::npos)
                    ? fstr.substr(pos, comma - pos)
                    : fstr.substr(pos);
                if (!tok.empty()) cfg.follower_sysids.push_back(atoi(tok.c_str()));
                pos = (comma != std::string::npos) ? comma + 1 : fstr.size();
            }
        } else if (arg == "--p900-uart9") {
            cfg.p900_uart9_mode = true;
            cfg.gcs_serial_enable = true;
            cfg.gcs_link_type = (cfg.role == NodeRole::LEADER)
                ? SwarmConfig::GcsLinkType::LEGACY_P900_LEADER
                : SwarmConfig::GcsLinkType::P900;
            // P900 长机兜底链路保持 v1.1 行为：合成僚机遥测注入到 GCS
            if (cfg.role == NodeRole::LEADER)
                cfg.follower_synthetic_telem_to_gcs = true;
            // P900 模式带宽紧张，关闭 5G 健康监视避免误判
            cfg.health_enable = false;
        } else if (arg == "--p900-dev" && i + 1 < argc) {
            cfg.p900_uart9_dev = argv[++i];
            cfg.gcs_serial_dev = cfg.p900_uart9_dev;
        } else if (arg == "--p900-baud" && i + 1 < argc) {
            cfg.p900_uart9_baud = atoi(argv[++i]);
            cfg.gcs_serial_baud = cfg.p900_uart9_baud;
        } else if (arg == "--sim") {
            cfg.simulation = true;
        }
    }

    printf("╔══════════════════════════════════════╗\n");
    printf("║  UAV Swarm Controller - LubanCat0+APM ║\n");
    printf("╠══════════════════════════════════════╣\n");
    printf("║  sysid      : %-23d  ║\n", cfg.local_sysid);
    printf("║  role        : %-23s  ║\n",
           cfg.role == NodeRole::LEADER ? "LEADER" : "FOLLOWER");
    printf("║  type        : %-23s  ║\n",
           cfg.vehicle_type == VehicleType::COPTER ? "COPTER" : "PLANE");
    if (cfg.local_fc.link == LinkType::SERIAL)
        printf("║  FC link     : %-15s@%-6d  ║\n",
               cfg.local_fc.serial_dev.c_str(), cfg.local_fc.serial_baud);
    else
        printf("║  FC link     : UDP %-15s:%-5d ║\n",
               cfg.local_fc.udp_ip.c_str(), cfg.local_fc.udp_port);
    if (cfg.gcs_link_type == SwarmConfig::GcsLinkType::IP) {
        printf("║  GCS link    : UDP %-15s:%-5d ║\n",
               cfg.gcs_udp_target_ip.c_str(), cfg.gcs_udp_target_port);
        printf("║  GCS hb/kbps : %2dHz / %4ukbps             ║\n",
               cfg.gcs_heartbeat_hz, cfg.gcs_tx_max_kbps);
        printf("║  5G health   : %-23s  ║\n",
               cfg.health_enable ? "enabled" : "disabled");
        if (cfg.p900_uart9_fallback)
            printf("║  P900 fallbk : %-15s@%-6d  ║\n",
                   cfg.p900_uart9_dev.c_str(), cfg.p900_uart9_baud);
    } else if (cfg.p900_uart9_mode) {
        printf("║  P900 UART9  : %-15s@%-6d  ║\n",
               cfg.p900_uart9_dev.c_str(), cfg.p900_uart9_baud);
    } else if (cfg.gcs_link_type == SwarmConfig::GcsLinkType::P900 ||
               cfg.gcs_link_type == SwarmConfig::GcsLinkType::LEGACY_P900_LEADER) {
        printf("║  GCS link    : P900 %-14s@%-6d ║\n",
               cfg.p900_uart9_dev.c_str(), cfg.p900_uart9_baud);
    } else if (cfg.gcs_serial_enable) {
        printf("║  GCS link    : %-15s@%-6d  ║\n",
               cfg.gcs_serial_dev.c_str(), cfg.gcs_serial_baud);
    } else {
        printf("║  GCS link    : %-23s  ║\n", "DISABLED");
    }
    printf("║  ctrl Hz     : %-23d  ║\n", cfg.ctrl_hz);
    if (cfg.role == NodeRole::LEADER && !cfg.follower_sysids.empty()) {
        printf("║  followers   : ");
        for (size_t i = 0; i < cfg.follower_sysids.size(); i++)
            printf("%d%s", cfg.follower_sysids[i],
                   i + 1 < cfg.follower_sysids.size() ? "," : "");
        printf("%*s║\n", static_cast<int>(23 - cfg.follower_sysids.size() * 2), "");
    }
    printf("╚══════════════════════════════════════╝\n");

    SwarmController ctrl(cfg);
    g_ctrl = &ctrl;

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    if (!ctrl.init()) {
        fprintf(stderr, "[main] init failed\n");
        return 1;
    }

    ctrl.start();

    while (ctrl.is_running()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    printf("[main] exited\n");
    return 0;
}
