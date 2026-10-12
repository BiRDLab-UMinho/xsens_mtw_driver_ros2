// =============================================================================
//  latency_eval — Xsens MTw driver ROS2 latency evaluation node
//
//  Auto-discovers all topics matching <topic_prefix>_* (IMUDataSingle) and
//  subscribes to each. No manual topic configuration needed.
//
//  Metrics per IMU:
//  [1] PUBLISH RATE JITTER  — inter-message arrival per IMU topic
//  [2] ROS2 TRANSPORT       — msg.timestamp → subscriber callback
//  [3] BUFFER HOLD TIME     — receive_stamp → publish_stamp (fresh only)
//  [4] STALE SAMPLE RATE    — is_fresh == false count
//  [5] PACKET COUNTER GAPS  — hardware sequence drops
//
//  Usage:
//    ros2 run xsens_mtw_driver latency_eval
//    ros2 run xsens_mtw_driver latency_eval --ros-args \
//        -p duration_s:=60.0 -p expected_imu_count:=3 -p save_dir:=/tmp
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <map>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "imu_msgs/msg/imu_data_single.hpp"

using Msg   = imu_msgs::msg::IMUDataSingle;
using Clock = std::chrono::steady_clock;
using TP    = Clock::time_point;

// -----------------------------------------------------------------------------
// Dual-output helpers
// -----------------------------------------------------------------------------

static void tprintf(FILE* fout, const char* fmt, ...) {
    va_list a1, a2;
    va_start(a1, fmt);
    va_copy(a2, a1);
    vprintf(fmt, a1);
    va_end(a1);
    if (fout) { vfprintf(fout, fmt, a2); }
    va_end(a2);
}

// -----------------------------------------------------------------------------
// Statistics helpers
// -----------------------------------------------------------------------------

struct Stats {
    double mean{0}, stddev{0}, min{0}, max{0};
    double p05{0}, p50{0}, p95{0}, p99{0};
    size_t n{0};
};

static Stats compute(std::vector<double>& v) {
    if (v.empty()) return {};
    std::sort(v.begin(), v.end());
    Stats s;
    s.n   = v.size();
    s.min = v.front();
    s.max = v.back();
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    s.mean = sum / static_cast<double>(s.n);
    double sq = 0.0;
    for (double x : v) sq += (x - s.mean) * (x - s.mean);
    s.stddev = std::sqrt(sq / static_cast<double>(s.n));
    auto pct = [&](double p) -> double {
        size_t i = std::min(static_cast<size_t>(p / 100.0 * s.n), s.n - 1);
        return v[i];
    };
    s.p05 = pct(5);
    s.p50 = pct(50);
    s.p95 = pct(95);
    s.p99 = pct(99);
    return s;
}

static void print_row(FILE* fout, const char* label, const Stats& s, const char* unit) {
    tprintf(fout,
            "  %-28s  n=%6zu  mean=%8.3f  std=%7.3f  "
            "min=%8.3f  p50=%8.3f  p95=%8.3f  p99=%8.3f  max=%8.3f  [%s]\n",
            label, s.n, s.mean, s.stddev,
            s.min, s.p50, s.p95, s.p99, s.max, unit);
}

static void print_histogram(FILE* fout, const char* label, std::vector<double>& v,
                             double lo, double hi, int bins = 24) {
    if (v.empty() || hi <= lo) return;
    const double w = (hi - lo) / bins;
    std::vector<int> cnt(bins, 0);
    int clipped = 0;
    for (double x : v) {
        int b = static_cast<int>((x - lo) / w);
        if (b < 0 || b >= bins) { ++clipped; continue; }
        ++cnt[b];
    }
    int peak = *std::max_element(cnt.begin(), cnt.end());
    tprintf(fout, "\n  Histogram: %s  (%.3f … %.3f%s)\n",
            label, lo, hi,
            clipped ? std::string(", " + std::to_string(clipped) + " clipped").c_str() : "");
    for (int i = 0; i < bins; ++i) {
        double a = lo + i * w, b = a + w;
        int bar = peak > 0 ? (cnt[i] * 50 / peak) : 0;
        tprintf(fout, "  [%8.3f,%8.3f)  %5d  |%s\n", a, b, cnt[i],
                std::string(bar, '#').c_str());
    }
}

static std::string timestamp_str() {
    std::time_t t = std::time(nullptr);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&t));
    return std::string(buf);
}

// -----------------------------------------------------------------------------
// Per-IMU state
// -----------------------------------------------------------------------------

struct ImuState {
    std::vector<double> arrival_intervals_ms;  // [1]
    std::vector<double> buffer_hold_ms;        // [3]
    size_t total_callbacks{0};
    size_t stale_count{0};
    bool     counter_init{false};
    uint16_t prev_counter{0};
    size_t   gap_events{0};
    size_t   dropped_packets{0};
    int64_t  prev_arrival_ns{0};
};

// -----------------------------------------------------------------------------
// Node
// -----------------------------------------------------------------------------

class LatencyEval : public rclcpp::Node {
public:
    LatencyEval() : Node("latency_eval") {
        declare_parameter("duration_s",         30.0);
        declare_parameter("topic_prefix",       std::string("xsens_imu_data"));
        declare_parameter("expected_rate_hz",   60.0);
        declare_parameter("expected_imu_count", 1);
        declare_parameter("histogram_bins",     24);

        const char* home = std::getenv("HOME");
        declare_parameter("save_dir", std::string(home ? home : "/tmp") + "/xsens_latency");

        declare_parameter("imu_prefix",  std::string(""));
        declare_parameter("imu_mapping", std::vector<std::string>{});

        _duration_s     = get_parameter("duration_s").as_double();
        _expected_hz    = get_parameter("expected_rate_hz").as_double();
        _expected_n_imu = static_cast<size_t>(get_parameter("expected_imu_count").as_int());
        _bins           = static_cast<int>(get_parameter("histogram_bins").as_int());
        _save_dir       = get_parameter("save_dir").as_string();
        _prefix         = get_parameter("topic_prefix").as_string();
        _imu_prefix     = get_parameter("imu_prefix").as_string();
        auto mapping    = get_parameter("imu_mapping").as_string_array();
        for (size_t i = 0; i + 1 < mapping.size(); i += 2)
            _imu_names[mapping[i]] = mapping[i + 1];

        printf("\n  latency_eval: prefix='%s'  duration=%.0f s"
               "  expected_rate=%.0f Hz  expected_imus=%zu  save_dir='%s'\n",
               _prefix.c_str(), _duration_s, _expected_hz, _expected_n_imu,
               _save_dir.c_str());

        // Discover per-IMU topics matching prefix (wait up to 5 s)
        printf("  Discovering topics matching '%s_*'...\n", _prefix.c_str());
        auto deadline = Clock::now() + std::chrono::seconds(5);
        std::vector<std::string> found_topics;
        while (found_topics.empty() && Clock::now() < deadline) {
            for (auto& [name, types] : get_topic_names_and_types()) {
                for (auto& t : types) {
                    if (t == "imu_msgs/msg/IMUDataSingle" &&
                        name.find("/" + _prefix + "_") == 0) {
                        found_topics.push_back(name);
                    }
                }
            }
            if (found_topics.empty())
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        if (found_topics.empty()) {
            printf("  ERROR: no topics found after 5 s. Is the driver running?\n\n");
        } else {
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
            printf("  Found %zu topic(s):\n", found_topics.size());
            for (auto& topic : found_topics) {
                // Extract device ID from topic name (everything after last '_')
                std::string dev_id = topic.substr(topic.rfind('_') + 1);
                printf("    %s  →  %s\n", topic.c_str(), resolve_name(dev_id).c_str());
                _subs.push_back(create_subscription<Msg>(
                    topic, qos, [this](const Msg::SharedPtr m) { callback(m); }));
            }
        }

        printf("  Waiting for messages…  (Ctrl-C also triggers report)\n\n");

        _report_timer = create_wall_timer(
            std::chrono::duration<double>(_duration_s),
            [this]() { if (!_report_done) report(); rclcpp::shutdown(); });
    }

    void report() {
        _report_done = true;
        const std::string ts          = timestamp_str();
        const std::string report_path = _save_dir + "/xsens_latency_report_" + ts + ".txt";
        const std::string csv_path    = _save_dir + "/xsens_latency_data_"   + ts + ".csv";

        std::filesystem::create_directories(_save_dir);
        FILE* fout = fopen(report_path.c_str(), "w");
        if (!fout)
            printf("  WARNING: could not open '%s' (%s)\n", report_path.c_str(), strerror(errno));

        const double elapsed =
            (_last_t.time_since_epoch().count() > 0 && _first_t.time_since_epoch().count() > 0)
            ? std::chrono::duration<double>(_last_t - _first_t).count()
            : _duration_s;

        tprintf(fout, "\n");
        tprintf(fout, "══════════════════════════════════════════════════════════════════════════════\n");
        tprintf(fout, "  XSENS MTW LATENCY EVALUATION REPORT\n");
        tprintf(fout, "  Messages received : %zu\n", _msg_count);
        tprintf(fout, "  Elapsed           : %.2f s\n", elapsed);
        tprintf(fout, "  Effective rate    : %.2f Hz  (expected %.1f Hz per IMU)\n",
                elapsed > 0 && !_imu_state.empty()
                    ? static_cast<double>(_msg_count) / elapsed / _imu_state.size()
                    : 0.0,
                _expected_hz);
        tprintf(fout, "  IMUs seen         : %zu  (expected %zu)\n",
                _imu_state.size(), _expected_n_imu);
        tprintf(fout, "══════════════════════════════════════════════════════════════════════════════\n");

        if (_msg_count < 2) {
            tprintf(fout, "  Not enough data — run longer or check that the driver is publishing.\n\n");
            if (fout) fclose(fout);
            return;
        }

        // ── Section 1: Publish rate jitter (per IMU) ─────────────────────────
        tprintf(fout, "\n[1] PUBLISH RATE JITTER  (inter-message arrival per IMU topic)\n");
        tprintf(fout, "    Expected interval = %.4f ms  (%.1f Hz)\n",
                1000.0 / _expected_hz, _expected_hz);
        for (auto& [id, st] : _imu_state) {
            if (st.arrival_intervals_ms.empty()) continue;
            auto s = compute(st.arrival_intervals_ms);
            auto name = resolve_name(id);
            print_row(fout, name.c_str(), s, "ms");
        }
        {
            std::vector<double> all_arr;
            for (auto& [id, st] : _imu_state)
                all_arr.insert(all_arr.end(),
                    st.arrival_intervals_ms.begin(), st.arrival_intervals_ms.end());
            print_histogram(fout, "arrival_interval [ms]", all_arr,
                            1000.0 / _expected_hz * 0.5,
                            1000.0 / _expected_hz * 1.5, _bins);
        }

        // ── Section 2: ROS2 transport latency ────────────────────────────────
        tprintf(fout, "\n[2] ROS2 TRANSPORT  (publish_stamp → subscriber callback)\n");
        if (_ros2_latencies_ms.empty()) {
            tprintf(fout, "    No data.\n");
        } else {
            auto s = compute(_ros2_latencies_ms);
            print_row(fout, "pub→sub latency", s, "ms");
            print_histogram(fout, "pub→sub latency [ms]", _ros2_latencies_ms,
                            0.0, std::max(s.p99 * 2.0, 2.0), _bins);
        }

        // ── Section 3: Buffer hold time ───────────────────────────────────────
        tprintf(fout, "\n[3] BUFFER HOLD TIME  (USB receive_stamp → publish_stamp, fresh only)\n");
        tprintf(fout, "    Upper bound ≈ 1/imu_rate = %.4f ms.\n", 1000.0 / _expected_hz);
        for (auto& [id, st] : _imu_state) {
            if (st.buffer_hold_ms.empty()) continue;
            auto v = st.buffer_hold_ms;
            auto s = compute(v);
            auto name = resolve_name(id);
            print_row(fout, name.c_str(), s, "ms");
        }
        if (!_buffer_hold_all_ms.empty()) {
            auto v = _buffer_hold_all_ms;
            auto s = compute(v);
            print_histogram(fout, "buffer_hold [ms]", _buffer_hold_all_ms,
                            0.0, std::max(s.p99 * 2.0, 1000.0 / _expected_hz * 2.0), _bins);
        }

        // ── Section 4: Stale sample rate ─────────────────────────────────────
        tprintf(fout, "\n[4] STALE SAMPLE RATE  (is_fresh==false per IMU)\n");
        for (auto& [id, st] : _imu_state) {
            double rate = st.total_callbacks > 0
                ? 100.0 * static_cast<double>(st.stale_count) / static_cast<double>(st.total_callbacks)
                : 0.0;
            tprintf(fout, "    %-40s  stale=%6zu / %6zu  (%.2f%%)\n",
                    resolve_name(id).c_str(), st.stale_count, st.total_callbacks, rate);
        }

        // ── Section 5: Packet counter gaps ───────────────────────────────────
        tprintf(fout, "\n[5] PACKET COUNTER GAPS  (uint16 hardware sequence)\n");
        bool any = false;
        for (auto& [id, st] : _imu_state) {
            if (!st.counter_init) continue;
            any = true;
            tprintf(fout, "    %-40s  gap_events=%4zu  estimated_dropped=%6zu\n",
                    resolve_name(id).c_str(), st.gap_events, st.dropped_packets);
        }
        if (!any)
            tprintf(fout, "    No packet counter data.\n");

        tprintf(fout, "\n══════════════════════════════════════════════════════════════════════════════\n");
        if (fout) {
            fclose(fout);
            printf("\n  Report saved → %s\n", report_path.c_str());
        }

        save_csv(csv_path);
    }

private:
    void save_csv(const std::string& path) {
        FILE* f = fopen(path.c_str(), "w");
        if (!f) {
            printf("  WARNING: could not open '%s' (%s)\n", path.c_str(), strerror(errno));
            return;
        }
        fprintf(f, "imu_id,arrival_interval_ms,ros2_latency_ms,buffer_hold_ms\n");
        // ros2 latency and buffer hold are not per-IMU indexed so we write them once
        const size_t n_ros2 = _ros2_latencies_ms.size();
        const size_t n_hold = _buffer_hold_all_ms.size();
        size_t ros2_i = 0, hold_i = 0;
        for (auto& [id, st] : _imu_state) {
            for (size_t i = 0; i < st.arrival_intervals_ms.size(); ++i) {
                fprintf(f, "%s,%.6f,", id.c_str(), st.arrival_intervals_ms[i]);
                if (ros2_i < n_ros2) fprintf(f, "%.6f", _ros2_latencies_ms[ros2_i++]);
                fprintf(f, ",");
                if (hold_i < n_hold) fprintf(f, "%.6f", _buffer_hold_all_ms[hold_i++]);
                fprintf(f, "\n");
            }
        }
        fclose(f);
        printf("  Raw data saved → %s\n\n", path.c_str());
    }

    void callback(const Msg::SharedPtr msg) {
        const auto t_now_ros = now();
        const std::string& id = msg->imu_data.id;
        ImuState& st = _imu_state[id];

        if (_msg_count == 0) _first_t = Clock::now();
        _last_t = Clock::now();

        // [1] Per-IMU arrival intervals
        if (st.prev_arrival_ns > 0) {
            double dt_ms = static_cast<double>(
                t_now_ros.nanoseconds() - st.prev_arrival_ns) / 1e6;
            if (dt_ms > 0.0 && dt_ms < 2000.0)
                st.arrival_intervals_ms.push_back(dt_ms);
        }
        st.prev_arrival_ns = t_now_ros.nanoseconds();

        // [2] ROS2 transport
        if (msg->timestamp > 0) {
            double lat_ms = static_cast<double>(
                t_now_ros.nanoseconds() - static_cast<int64_t>(msg->timestamp)) / 1e6;
            if (lat_ms >= 0.0 && lat_ms < 5000.0)
                _ros2_latencies_ms.push_back(lat_ms);
        }

        st.total_callbacks++;

        if (msg->imu_data.is_fresh) {
            // [3] Buffer hold time
            const int64_t recv_ns =
                static_cast<int64_t>(msg->imu_data.receive_stamp.sec) * 1000000000LL
                + static_cast<int64_t>(msg->imu_data.receive_stamp.nanosec);
            if (recv_ns > 0 && msg->timestamp > 0) {
                double hold_ms = static_cast<double>(
                    static_cast<int64_t>(msg->timestamp) - recv_ns) / 1e6;
                if (hold_ms >= 0.0 && hold_ms < 1000.0) {
                    st.buffer_hold_ms.push_back(hold_ms);
                    _buffer_hold_all_ms.push_back(hold_ms);
                }
            }

            // [5] Packet counter gaps
            if (msg->imu_data.packet_counter > 0 || st.counter_init) {
                if (st.counter_init) {
                    uint16_t gap = static_cast<uint16_t>(
                        msg->imu_data.packet_counter - st.prev_counter);
                    if (gap > 1) {
                        st.gap_events++;
                        st.dropped_packets += (gap - 1);
                    }
                }
                st.prev_counter = msg->imu_data.packet_counter;
                st.counter_init = true;
            }
        } else {
            st.stale_count++;
        }

        ++_msg_count;
    }

    // ── ROS handles ──────────────────────────────────────────────────────────
    std::vector<rclcpp::Subscription<Msg>::SharedPtr> _subs;
    rclcpp::TimerBase::SharedPtr                      _report_timer;

    // ── Parameters ───────────────────────────────────────────────────────────
    std::string resolve_name(const std::string& dev_id) const {
        if (!_imu_prefix.empty() && dev_id.size() > _imu_prefix.size()) {
            auto it = _imu_names.find(dev_id.substr(_imu_prefix.size()));
            if (it != _imu_names.end())
                return dev_id + " (" + it->second + ")";
        }
        return dev_id;
    }

    double      _duration_s{30.0};
    double      _expected_hz{60.0};
    size_t      _expected_n_imu{1};
    int         _bins{24};
    std::string _save_dir;
    std::string _prefix{"xsens_imu_data"};
    std::string _imu_prefix;
    std::map<std::string, std::string> _imu_names;

    // ── State ─────────────────────────────────────────────────────────────────
    bool    _report_done{false};
    size_t  _msg_count{0};
    TP      _first_t{};
    TP      _last_t{};

    // ── Buckets ───────────────────────────────────────────────────────────────
    std::vector<double> _ros2_latencies_ms;
    std::vector<double> _buffer_hold_all_ms;

    std::map<std::string, ImuState> _imu_state;
};

// -----------------------------------------------------------------------------
// main
// -----------------------------------------------------------------------------

static std::shared_ptr<LatencyEval> g_node;

static void sigint_handler(int) {
    printf("\n  [Ctrl-C] generating report...\n");
    if (g_node) g_node->report();
    rclcpp::shutdown();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    signal(SIGINT, sigint_handler);
    g_node = std::make_shared<LatencyEval>();
    rclcpp::spin(g_node);
    g_node.reset();
    rclcpp::shutdown();
    return 0;
}
