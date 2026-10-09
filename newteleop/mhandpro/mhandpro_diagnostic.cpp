#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <dlfcn.h>
#include <arpa/inet.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <sstream>
#include <thread>
#include <vector>
#include <sys/socket.h>
#include <unistd.h>

#include "sdk/include/VDMocapSDK_mHandPro_DataType.h"

using namespace mHandProDevice;

namespace {

constexpr double kPi = 3.14159265358979323846;

// 程序内部统一使用 w,x,y,z 顺序的双精度单位四元数。
struct Quaternion {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

Quaternion normalize(Quaternion q) {
    const double n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (!std::isfinite(n) || n < 1e-8) {
        throw std::runtime_error("invalid quaternion");
    }
    q.w /= n; q.x /= n; q.y /= n; q.z /= n;
    return q;
}

Quaternion conjugate(const Quaternion& q) {
    return {q.w, -q.x, -q.y, -q.z};
}

Quaternion multiply(const Quaternion& a, const Quaternion& b) {
    return {
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}

Quaternion relative(const Quaternion& parent, const Quaternion& child) {
    // 单位四元数的逆等于共轭。先消除父节点的世界旋转，再得到子节点局部旋转。
    return normalize(multiply(conjugate(normalize(parent)), normalize(child)));
}

double rotation_distance_deg(const Quaternion& a, const Quaternion& b) {
    // q 和 -q 表示同一个旋转，因此使用 abs(w) 避免伪 360 度跳变。
    const Quaternion delta = relative(a, b);
    const double w = std::clamp(std::abs(delta.w), 0.0, 1.0);
    return 2.0 * std::acos(w) * 180.0 / kPi;
}

using Vec3 = std::array<double, 3>;

// 把“从张手基准到当前姿态”的旋转转换为有方向的旋转向量，单位为度。
// 向量方向是旋转轴，向量长度是旋转角，因此可用来分离弯曲和侧摆。
Vec3 rotation_vector_deg(const Quaternion& reference, const Quaternion& current) {
    Quaternion q = relative(reference, current);
    if (q.w < 0.0) { q.w = -q.w; q.x = -q.x; q.y = -q.y; q.z = -q.z; }
    const double w = std::clamp(q.w, -1.0, 1.0);
    const double angle = 2.0 * std::acos(w);
    const double s = std::sqrt(std::max(0.0, 1.0 - w*w));
    if (s < 1e-8 || angle < 1e-8) return {0.0, 0.0, 0.0};
    const double scale = angle * 180.0 / kPi / s;
    return {q.x * scale, q.y * scale, q.z * scale};
}

Quaternion from_sdk(const float q[4]) {
    return normalize({q[0], q[1], q[2], q[3]}); // 官方 SDK 顺序：w,x,y,z
}

struct Frame {
    bool valid{false};
    _GloveMode_ side{GM_NONE};
    int frame_index{-1};
    int frequency{-1};
    float power{0.0f};
    std::chrono::steady_clock::time_point received{};
    std::array<Quaternion, NODES_HAND> node{};
    std::array<_SensorState_, NODES_HAND> sensor{};
};

std::mutex g_frame_mutex;
Frame g_left;
Frame g_right;
std::atomic<bool> g_disconnected{false};

void copy_frame(const _GloveMocapData_& src, Frame& dst) {
    if (!src.isUpdate) return;
    Frame f;
    f.valid = true;
    f.side = src.glove;
    f.frame_index = src.frameIndex;
    f.frequency = src.frequency;
    f.power = src.devicePower;
    f.received = std::chrono::steady_clock::now();
    try {
        for (int i = 0; i < NODES_HAND; ++i) {
            f.node[i] = from_sdk(src.quaternion[i]);
            f.sensor[i] = src.sensorState[i];
        }
    } catch (...) {
        return;
    }
    dst = f;
}

void on_data(_GloveMocapData_ right, _GloveMocapData_ left) {
    // SDK 回调中只复制数据，不做文件、网络或复杂解算。
    std::lock_guard<std::mutex> lock(g_frame_mutex);
    copy_frame(right, g_right);
    copy_frame(left, g_left);
}

void on_break(_GloveMode_) {
    g_disconnected.store(true);
}

Frame latest_frame(_GloveMode_ preferred) {
    std::lock_guard<std::mutex> lock(g_frame_mutex);
    if (preferred == GM_LeftGlove && g_left.valid) return g_left;
    if (preferred == GM_RightGlove && g_right.valid) return g_right;
    if (g_left.valid && !g_right.valid) return g_left;
    if (g_right.valid && !g_left.valid) return g_right;
    if (g_left.valid && g_right.valid) {
        return g_left.received >= g_right.received ? g_left : g_right;
    }
    return {};
}

using RelativeSet = std::array<std::array<Quaternion, 3>, 5>;

// 骨架链：拇指 0-1-2-3，食指 4-5-6-7，依次到小指 16-17-18-19。
RelativeSet compute_relatives(const Frame& f) {
    RelativeSet out{};
    const int bases[5] = {0, 4, 8, 12, 16};
    for (int finger = 0; finger < 5; ++finger) {
        const int base = bases[finger];
        for (int joint = 0; joint < 3; ++joint) {
            out[finger][joint] = relative(f.node[base + joint],
                                                   f.node[base + joint + 1]);
        }
    }
    return out;
}

double quaternion_dot(const Quaternion& a, const Quaternion& b) {
    return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
}

// 采集约 1.5 秒并对四元数做同半球平均，比单帧标定更抗抖动。
RelativeSet capture_average_relatives(_GloveMode_ side, double seconds = 1.5) {
    RelativeSet sum{};
    RelativeSet anchor{};
    for (auto& finger : sum) for (auto& q : finger) q = {0.0, 0.0, 0.0, 0.0};
    bool initialized = false;
    int samples = 0;
    const auto end = std::chrono::steady_clock::now()
                   + std::chrono::milliseconds(static_cast<int>(seconds * 1000.0));
    std::cout << "开始采集 " << std::fixed << std::setprecision(1) << seconds
              << " 秒，请保持当前姿势不动..." << std::flush;
    while (std::chrono::steady_clock::now() < end) {
        const Frame frame = latest_frame(side);
        if (frame.valid) {
            const RelativeSet current = compute_relatives(frame);
            if (!initialized) { anchor = current; initialized = true; }
            for (int f = 0; f < 5; ++f) {
                for (int j = 0; j < 3; ++j) {
                    Quaternion q = current[f][j];
                    if (quaternion_dot(anchor[f][j], q) < 0.0) {
                        q.w=-q.w; q.x=-q.x; q.y=-q.y; q.z=-q.z;
                    }
                    sum[f][j].w += q.w; sum[f][j].x += q.x;
                    sum[f][j].y += q.y; sum[f][j].z += q.z;
                }
            }
            ++samples;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (samples < 10) throw std::runtime_error("标定采样不足");
    for (auto& finger : sum) for (auto& q : finger) q = normalize(q);
    std::cout << " 完成，采集 " << samples << " 帧。\n";
    return sum;
}

using FeatureSet = std::array<std::array<double, 3>, 5>;
using DirectionFeature = std::array<double, 9>;

FeatureSet difference_from(const RelativeSet& reference, const RelativeSet& current) {
    // 计算当前局部关节姿态相对于“张手零点”的旋转角变化。
    FeatureSet result{};
    for (int f = 0; f < 5; ++f) {
        for (int j = 0; j < 3; ++j) {
            result[f][j] = rotation_distance_deg(reference[f][j], current[f][j]);
        }
    }
    return result;
}

DirectionFeature directional_feature(const RelativeSet& reference,
                                      const RelativeSet& current, int finger) {
    DirectionFeature out{};
    for (int j = 0; j < 3; ++j) {
        const Vec3 v = rotation_vector_deg(reference[finger][j], current[finger][j]);
        for (int axis = 0; axis < 3; ++axis) out[j*3 + axis] = v[axis];
    }
    return out;
}

double feature_dot(const DirectionFeature& a, const DirectionFeature& b) {
    double result = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) result += a[i] * b[i];
    return result;
}

double feature_norm(const DirectionFeature& value) {
    return std::sqrt(std::max(0.0, feature_dot(value, value)));
}

double axis_correlation(const DirectionFeature& a, const DirectionFeature& b) {
    const double denominator = feature_norm(a) * feature_norm(b);
    if (denominator < 1e-8) return 1.0;
    return std::clamp(std::abs(feature_dot(a, b)) / denominator, 0.0, 1.0);
}

double axis_angle_deg(const DirectionFeature& a, const DirectionFeature& b) {
    return std::acos(axis_correlation(a, b)) * 180.0 / kPi;
}

DirectionFeature subtract_scaled(const DirectionFeature& value,
                                 const DirectionFeature& axis, double scale) {
    DirectionFeature result{};
    for (std::size_t i = 0; i < result.size(); ++i) result[i] = value[i] - scale*axis[i];
    return result;
}

// Gram-Schmidt：从侧摆标定方向中去掉与弯曲方向平行的成分。
DirectionFeature orthogonal_component(const DirectionFeature& secondary,
                                      const DirectionFeature& primary) {
    const double aa = feature_dot(primary, primary);
    if (aa < 1e-8) return {};
    return subtract_scaled(secondary, primary, feature_dot(secondary, primary) / aa);
}

// 将当前动作投影到一个标定方向：0 表示张手，1 表示标定姿势。
double project_one_axis(const DirectionFeature& value, const DirectionFeature& axis) {
    const double denominator = feature_dot(axis, axis);
    if (denominator < 25.0) return 0.0;
    return std::clamp(feature_dot(value, axis) / denominator, 0.0, 1.0);
}

// 四指屈曲只关心沿标定轴的幅度。接近180度时，四元数的等价
// 表示可使旋转向量整体反号；若使用有符号投影，真实屈曲会被夹到0。
double project_flex_magnitude(const DirectionFeature& value,
                              const DirectionFeature& axis) {
    const double denominator = feature_dot(axis, axis);
    if (denominator < 25.0) return 0.0;
    return std::clamp(std::abs(feature_dot(value, axis)) / denominator, 0.0, 1.0);
}

// 同时在“弯曲”和“侧摆/对掌”两个可能不正交的标定方向上做最小二乘分解。
std::array<double, 2> project_two_axes(const DirectionFeature& value,
                                      const DirectionFeature& first,
                                      const DirectionFeature& second) {
    const double aa = feature_dot(first, first);
    const DirectionFeature second_orthogonal = orthogonal_component(second, first);
    const double oo = feature_dot(second_orthogonal, second_orthogonal);
    const double correlation = axis_correlation(first, second);

    // 两轴过于平行时，二维求解会放大小误差。此时只保留稳定的弯曲投影。
    if (aa < 25.0 || oo < 25.0 || correlation > 0.90) {
        return {project_one_axis(value, first), project_one_axis(value, second)};
    }

    // 先求只能由正交侧摆分量解释的部分，再从原动作中扣除侧摆后求弯曲。
    const double second_coefficient = feature_dot(value, second_orthogonal) / oo;
    const DirectionFeature without_second = subtract_scaled(value, second, second_coefficient);
    const double first_coefficient = feature_dot(without_second, first) / aa;
    return {std::clamp(first_coefficient, 0.0, 1.0),
            std::clamp(second_coefficient, 0.0, 1.0)};
}

bool axes_are_separable(const DirectionFeature& primary, const DirectionFeature& secondary) {
    return feature_norm(primary) >= 5.0
        && feature_norm(orthogonal_component(secondary, primary)) >= 5.0
        && axis_correlation(primary, secondary) <= 0.90;
}

void print_axis_quality(const char* label, const DirectionFeature& primary,
                        const DirectionFeature& secondary) {
    const double correlation = axis_correlation(primary, secondary);
    const double angle = axis_angle_deg(primary, secondary);
    const double orthogonal_norm = feature_norm(orthogonal_component(secondary, primary));
    std::cout << "  [" << label << "] 两标定方向相关系数="
              << std::fixed << std::setprecision(3) << correlation
              << "，夹角=" << std::setprecision(1) << angle
              << "度，正交有效幅度=" << orthogonal_norm << "度\n";
    if (axes_are_separable(primary, secondary)) {
        std::cout << "    结果：两个动作方向可分离，已启用稳定二维解耦。\n";
    } else {
        std::cout << "    警告：两个动作方向过于接近或侧摆幅度太小，"
                     "已回退为单弯曲轴，避免数值放大。\n";
    }
}

struct CalibrationModel {
    bool have_open{false};
    RelativeSet open{};
    std::array<bool, 5> have_flex{};
    std::array<DirectionFeature, 5> flex_axis{};
    std::array<bool, 5> have_spread{};
    std::array<DirectionFeature, 5> spread_axis{};
    bool have_thumb_opp{false};
    DirectionFeature thumb_opp_axis{};
};

constexpr const char* kCalibrationMagic = "MHANDPRO_CALIBRATION_V2";

void write_quaternion(std::ostream& out, const Quaternion& q) {
    out << q.w << ' ' << q.x << ' ' << q.y << ' ' << q.z << '\n';
}

Quaternion read_quaternion(std::istream& in) {
    Quaternion q;
    if (!(in >> q.w >> q.x >> q.y >> q.z)) throw std::runtime_error("标定文件中的四元数不完整");
    return normalize(q);
}

void write_feature(std::ostream& out, const DirectionFeature& feature) {
    for (double value : feature) out << value << ' ';
    out << '\n';
}

DirectionFeature read_feature(std::istream& in) {
    DirectionFeature feature{};
    for (double& value : feature) {
        if (!(in >> value) || !std::isfinite(value)) throw std::runtime_error("标定特征数据无效");
    }
    return feature;
}

void save_calibration(const CalibrationModel& model, const std::string& path) {
    if (!model.have_open) throw std::runtime_error("没有张手零点，不能保存");
    const std::filesystem::path file(path);
    if (file.has_parent_path()) std::filesystem::create_directories(file.parent_path());
    std::ofstream out(file);
    if (!out) throw std::runtime_error("无法写入标定文件：" + path);
    out << kCalibrationMagic << '\n' << std::setprecision(17);
    for (const auto& finger : model.open) for (const auto& q : finger) write_quaternion(out, q);
    for (bool value : model.have_flex) out << static_cast<int>(value) << ' ';
    out << '\n';
    for (const auto& feature : model.flex_axis) write_feature(out, feature);
    for (bool value : model.have_spread) out << static_cast<int>(value) << ' ';
    out << '\n';
    for (const auto& feature : model.spread_axis) write_feature(out, feature);
    out << static_cast<int>(model.have_thumb_opp) << '\n';
    write_feature(out, model.thumb_opp_axis);
    if (!out) throw std::runtime_error("保存标定文件时发生写入错误");
}

CalibrationModel load_calibration(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("无法打开标定文件：" + path);
    std::string magic;
    std::getline(in, magic);
    if (magic != kCalibrationMagic) throw std::runtime_error("标定文件版本不兼容");
    CalibrationModel model;
    for (auto& finger : model.open) for (auto& q : finger) q = read_quaternion(in);
    model.have_open = true;
    for (std::size_t i = 0; i < model.have_flex.size(); ++i) {
        int value = 0; if (!(in >> value)) throw std::runtime_error("标定文件缺少弯曲标志");
        model.have_flex[i] = value != 0;
    }
    for (auto& feature : model.flex_axis) feature = read_feature(in);
    for (std::size_t i = 0; i < model.have_spread.size(); ++i) {
        int value = 0; if (!(in >> value)) throw std::runtime_error("标定文件缺少侧摆标志");
        model.have_spread[i] = value != 0;
    }
    for (auto& feature : model.spread_axis) feature = read_feature(in);
    int thumb = 0; if (!(in >> thumb)) throw std::runtime_error("标定文件缺少拇指标志");
    model.have_thumb_opp = thumb != 0;
    model.thumb_opp_axis = read_feature(in);
    return model;
}

double weighted_flex(const FeatureSet& f, int finger) {
    return 0.50*f[finger][0] + 0.35*f[finger][1] + 0.15*f[finger][2];
}

const char* side_name(_GloveMode_ side) {
    return side == GM_LeftGlove ? "LEFT" : side == GM_RightGlove ? "RIGHT" : "UNKNOWN";
}

const char* sensor_name(_SensorState_ state) {
    switch (state) {
        case SS_Well: return "OK";
        case SS_NoData: return "NO_DATA";
        case SS_UnReady: return "UNREADY";
        case SS_BadMag: return "BAD_MAG";
        default: return "NONE";
    }
}

struct Sdk {
    // 按官方头文件/演示程序声明各导出函数的类型。
    using InitialFn = void(*)(_WorldSpace_, float[NODES_HAND][3], float[NODES_HAND][3]);
    using ConnectFn = _ConnectState_(*)();
    using DisconnectFn = void(*)();
    using SetCallbackFn = void(*)(GLOVEMOCAPDATA_CALLBACK);
    using SetBreakFn = void(*)(GLOVEBREAK_CALLBACK);
    using SetDimensionFn = void(*)(bool);
    using StartCalibrationFn = void(*)(_CalibrationMode_, float[4]);
    using GetCalibrationProgressFn = _CalibrationProgress_(*)();

    void* handle{nullptr};
    InitialFn initial{nullptr};
    ConnectFn connect{nullptr};
    DisconnectFn disconnect{nullptr};
    SetCallbackFn set_callback{nullptr};
    SetBreakFn set_break{nullptr};
    SetDimensionFn set_dimension{nullptr};
    StartCalibrationFn start_calibration{nullptr};
    GetCalibrationProgressFn calibration_progress{nullptr};

    template <typename T>
    T symbol(const char* name) {
        dlerror();
        auto value = reinterpret_cast<T>(dlsym(handle, name));
        if (const char* error = dlerror()) throw std::runtime_error(error);
        return value;
    }

    explicit Sdk(const char* path) {
        // 运行时加载官方 .so，再通过 dlsym 获取官方 API 入口。
        handle = dlopen(path, RTLD_NOW);
        if (!handle) throw std::runtime_error(dlerror());
        initial = symbol<InitialFn>("Initial");
        connect = symbol<ConnectFn>("Connect");
        disconnect = symbol<DisconnectFn>("DisConnect");
        set_callback = symbol<SetCallbackFn>("SetGloveDataCallBackFunc");
        set_break = symbol<SetBreakFn>("SetGloveBreakCallBackFunc");
        set_dimension = symbol<SetDimensionFn>("SetHandDimension");
        start_calibration = symbol<StartCalibrationFn>("StartCalibration");
        calibration_progress = symbol<GetCalibrationProgressFn>("GetCalibrationProgress");
    }

    ~Sdk() {
        if (disconnect) disconnect();
        if (handle) dlclose(handle);
    }
};

std::string find_accessible_glove_serial() {
    // mHandPro接收器在当前Ubuntu环境中枚举为ttyUSB设备。
    // 官方SDK在设备连接过程中突然消失时可能直接终止，因此调用SDK前先做基本检查。
    for (int i = 0; i < 16; ++i) {
        const std::string path = "/dev/ttyUSB" + std::to_string(i);
        if (std::filesystem::exists(path) && ::access(path.c_str(), R_OK | W_OK) == 0) {
            return path;
        }
    }
    return {};
}

void print_help() {
    std::cout
        << "\n========== mHandPro 六维映射诊断工具 ==========\n"
        << "  pose                SDK P-pose 姿态标定\n"
        << "  status              查看连接、帧率和传感节点状态\n"
        << "  open                采集自然张手零点（1.5秒平均）\n"
        << "  zero                只更新张手零点，保留已加载的动作标定\n"
        << "  calib index         标定食指单独完全弯曲\n"
        << "  calib middle        标定中指单独完全弯曲\n"
        << "  calib ring          标定无名指单独完全弯曲\n"
        << "  calib pinky         标定小指单独完全弯曲\n"
        << "  calib spread        标定四指保持伸直时的最大侧摆/展开\n"
        << "  calib thumb-flex    标定拇指弯曲\n"
        << "  calib thumb-opp     标定拇指对掌\n"
        << "  save [文件]         保存标定，默认 config/left_hand.calib\n"
        << "  load [文件]         加载标定，默认 config/left_hand.calib\n"
        << "  show                显示当前关节角和解耦后的六维命令\n"
        << "  vectors             显示每个关节的有向旋转向量 [rx ry rz]\n"
        << "  monitor             以 5 Hz 连续显示 10 秒\n"
        << "  teleop [配置]       按配置的频率和行程限制控制Inspire\n"
        << "  help                显示本帮助\n"
        << "  quit                断开手套并退出\n"
        << "=====================================================\n\n";
}

void wait_for_frame(_GloveMode_ side) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
        if (latest_frame(side).valid) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    throw std::runtime_error("5 秒内未收到手套姿态数据");
}

void print_status(const Frame& f) {
    if (!f.valid) {
        std::cout << "尚未收到有效数据帧。\n";
        return;
    }
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - f.received).count();
    std::cout << "手别=" << side_name(f.side)
              << " 帧号=" << f.frame_index
              << " 频率=" << f.frequency << "Hz"
              // 头文件声明为 0~1，但实机可能返回类似 4.27 的电压值，因此保留原值。
              << " 电量原始值=" << std::fixed << std::setprecision(3) << f.power
              << " 数据年龄=" << age_ms << "ms"
              << " 已断线=" << (g_disconnected.load() ? "是" : "否") << "\n";
    bool all_ok = true;
    for (int i = 0; i < NODES_HAND; ++i) {
        if (f.sensor[i] != SS_Well) {
            all_ok = false;
            std::cout << "  节点 " << i << ": " << sensor_name(f.sensor[i]) << "\n";
        }
    }
    if (all_ok) std::cout << "20 个节点状态全部正常。\n";
}

void print_vectors(const Frame& frame, const RelativeSet& open_ref) {
    const RelativeSet current = compute_relatives(frame);
    static const char* names[5] = {"拇指", "食指", "中指", "无名指", "小指"};
    std::cout << "有向旋转向量（度）：每行为 [rx ry rz]\n";
    for (int f = 0; f < 5; ++f) {
        std::cout << "  " << names[f] << "\n";
        for (int j = 0; j < 3; ++j) {
            const Vec3 v = rotation_vector_deg(open_ref[f][j], current[f][j]);
            std::cout << "    关节" << (j+1) << ": ["
                      << std::fixed << std::setprecision(1)
                      << std::setw(6) << v[0] << " " << std::setw(6) << v[1]
                      << " " << std::setw(6) << v[2] << "]\n";
        }
    }
}

std::array<double, 6> compute_command(const RelativeSet& current,
                                     const CalibrationModel& model) {
    std::array<double, 6> command{};
    // Inspire 顺序：小指、无名指、中指、食指、拇指弯曲、拇指对掌。
    const int fingers[4] = {4, 3, 2, 1};
    for (int out = 0; out < 4; ++out) {
        const int f = fingers[out];
        if (!model.have_flex[f]) continue;
        const DirectionFeature value = directional_feature(model.open, current, f);
        if (model.have_spread[f]) {
            if (axes_are_separable(model.flex_axis[f], model.spread_axis[f])) {
                command[out] = project_two_axes(value, model.flex_axis[f],
                                                 model.spread_axis[f])[0];
            } else {
                command[out] = project_one_axis(value, model.flex_axis[f]);
            }
        } else {
            command[out] = project_flex_magnitude(value, model.flex_axis[f]);
        }
    }
    const DirectionFeature thumb = directional_feature(model.open, current, 0);
    if (model.have_flex[0] && model.have_thumb_opp) {
        if (axes_are_separable(model.flex_axis[0], model.thumb_opp_axis)) {
            const auto result = project_two_axes(thumb, model.flex_axis[0], model.thumb_opp_axis);
            command[4] = result[0]; command[5] = result[1];
        } else {
            command[4] = project_one_axis(thumb, model.flex_axis[0]);
            command[5] = project_one_axis(thumb, model.thumb_opp_axis);
        }
    } else {
        if (model.have_flex[0]) command[4] = project_one_axis(thumb, model.flex_axis[0]);
        if (model.have_thumb_opp) command[5] = project_one_axis(thumb, model.thumb_opp_axis);
    }
    return command;
}

void print_measurement(const Frame& frame, const CalibrationModel& model) {
    if (!frame.valid) { std::cout << "当前没有有效数据帧。\n"; return; }
    const auto current = compute_relatives(frame);
    const auto delta = difference_from(model.open, current);
    static const char* names[5] = {"拇指", "食指", "中指", "无名指", "小指"};
    std::cout << "帧 " << frame.frame_index << " " << side_name(frame.side) << "\n";
    for (int f = 0; f < 5; ++f) {
        std::cout << "  " << std::setw(6) << names[f] << " 三段总转角(度): "
                  << std::fixed << std::setprecision(1)
                  << std::setw(6) << delta[f][0] << " "
                  << std::setw(6) << delta[f][1] << " "
                  << std::setw(6) << delta[f][2];
        if (f > 0) std::cout << "  旧加权值=" << std::setw(6) << weighted_flex(delta, f);
        std::cout << "\n";
    }
    const auto command = compute_command(current, model);
    std::cout << "  新六维 [小指 无名指 中指 食指 拇指弯曲 拇指对掌]: [";
    for (int i = 0; i < 6; ++i) {
        if (i) std::cout << " ";
        std::cout << std::fixed << std::setprecision(2) << command[i];
    }
    std::cout << "]\n";
}

struct InspireConfig {
    std::string host{"192.168.3.85"};
    int port{9102};
    std::array<int, 6> open_tick{500,500,500,500,500,500};
    std::array<int, 6> closed_tick{500,500,500,500,500,500};
    double range_scale{0.30};
    double ema_alpha{0.25};
    double deadzone{0.03};
    double max_step{0.04};
    int control_hz{30};
    int stale_hold_ms{200};
    int stale_abort_ms{500};
    int sensor_fault_warn_ms{150};
    int sensor_fault_abort_ms{800};
    bool pinch_assist{false};
    double pinch_index_on{0.30};
    double pinch_index_full{0.80};
    double pinch_opp_on{0.08};
    double pinch_opp_full{0.27};
    double pinch_index_target{0.88};
    double pinch_thumb_flex_target{0.78};
    double pinch_thumb_opp_target{0.85};
};

std::string trim(std::string value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    return value.substr(first, value.find_last_not_of(" \t\r\n") - first + 1);
}

std::array<int, 6> parse_six_ints(const std::string& text, const char* key) {
    std::array<int, 6> result{};
    std::stringstream stream(text);
    std::string item;
    for (int i = 0; i < 6; ++i) {
        if (!std::getline(stream, item, ',')) throw std::runtime_error(std::string(key) + " 必须有6个整数");
        result[i] = std::stoi(trim(item));
    }
    if (std::getline(stream, item, ',')) throw std::runtime_error(std::string(key) + " 只能有6个整数");
    return result;
}

InspireConfig load_inspire_config(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("无法打开 Inspire 配置：" + path);
    InspireConfig cfg;
    std::string line;
    while (std::getline(in, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        const auto pos = line.find('=');
        if (pos == std::string::npos) throw std::runtime_error("配置行缺少 '='：" + line);
        const std::string key = trim(line.substr(0, pos));
        const std::string value = trim(line.substr(pos + 1));
        if (key == "host") cfg.host = value;
        else if (key == "port") cfg.port = std::stoi(value);
        else if (key == "open") cfg.open_tick = parse_six_ints(value, "open");
        else if (key == "closed") cfg.closed_tick = parse_six_ints(value, "closed");
        else if (key == "range_scale") cfg.range_scale = std::stod(value);
        else if (key == "ema_alpha") cfg.ema_alpha = std::stod(value);
        else if (key == "deadzone") cfg.deadzone = std::stod(value);
        else if (key == "max_step") cfg.max_step = std::stod(value);
        else if (key == "control_hz") cfg.control_hz = std::stoi(value);
        else if (key == "stale_hold_ms") cfg.stale_hold_ms = std::stoi(value);
        else if (key == "stale_abort_ms") cfg.stale_abort_ms = std::stoi(value);
        else if (key == "sensor_fault_warn_ms") cfg.sensor_fault_warn_ms = std::stoi(value);
        else if (key == "sensor_fault_abort_ms") cfg.sensor_fault_abort_ms = std::stoi(value);
        else if (key == "pinch_assist") cfg.pinch_assist = std::stoi(value) != 0;
        else if (key == "pinch_index_on") cfg.pinch_index_on = std::stod(value);
        else if (key == "pinch_index_full") cfg.pinch_index_full = std::stod(value);
        else if (key == "pinch_opp_on") cfg.pinch_opp_on = std::stod(value);
        else if (key == "pinch_opp_full") cfg.pinch_opp_full = std::stod(value);
        else if (key == "pinch_index_target") cfg.pinch_index_target = std::stod(value);
        else if (key == "pinch_thumb_flex_target") cfg.pinch_thumb_flex_target = std::stod(value);
        else if (key == "pinch_thumb_opp_target") cfg.pinch_thumb_opp_target = std::stod(value);
        else throw std::runtime_error("未知 Inspire 配置项：" + key);
    }
    if (cfg.range_scale <= 0.0 || cfg.range_scale > 1.0)
        throw std::runtime_error("range_scale 必须在 (0, 1.0] 内");
    if (cfg.ema_alpha <= 0.0 || cfg.ema_alpha > 1.0) throw std::runtime_error("ema_alpha 必须在 (0,1] 内");
    if (cfg.control_hz < 5 || cfg.control_hz > 50) throw std::runtime_error("control_hz 必须在5~50Hz");
    if (cfg.sensor_fault_abort_ms < 100 || cfg.sensor_fault_abort_ms > 3000)
        throw std::runtime_error("sensor_fault_abort_ms 必须在100~3000ms");
    if (cfg.sensor_fault_warn_ms < 0 || cfg.sensor_fault_warn_ms >= cfg.sensor_fault_abort_ms)
        throw std::runtime_error("sensor_fault_warn_ms 必须大于等于0且小于sensor_fault_abort_ms");
    const auto unit = [](double value) { return value >= 0.0 && value <= 1.0; };
    if (!unit(cfg.pinch_index_on) || !unit(cfg.pinch_index_full) ||
        !unit(cfg.pinch_opp_on) || !unit(cfg.pinch_opp_full) ||
        !unit(cfg.pinch_index_target) || !unit(cfg.pinch_thumb_flex_target) ||
        !unit(cfg.pinch_thumb_opp_target) || cfg.pinch_index_on >= cfg.pinch_index_full ||
        cfg.pinch_opp_on >= cfg.pinch_opp_full)
        throw std::runtime_error("pinch参数必须在0~1内，且on必须小于full");
    for (int i = 0; i < 6; ++i) {
        if (cfg.open_tick[i] == cfg.closed_tick[i])
            throw std::runtime_error("第" + std::to_string(i) + "路 open/closed 相同：必须先实测端点");
    }
    return cfg;
}

bool frame_is_healthy(const Frame& frame, int max_age_ms, std::string& reason) {
    if (!frame.valid) { reason = "没有有效数据帧"; return false; }
    if (g_disconnected.load()) { reason = "SDK报告手套断线"; return false; }
    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - frame.received).count();
    if (age > max_age_ms) { reason = "数据超时 " + std::to_string(age) + "ms"; return false; }
    for (int i = 0; i < NODES_HAND; ++i) {
        if (frame.sensor[i] == SS_BadMag || frame.sensor[i] == SS_NoData || frame.sensor[i] == SS_UnReady) {
            reason = "节点" + std::to_string(i) + "状态=" + sensor_name(frame.sensor[i]);
            return false;
        }
    }
    return true;
}

double apply_deadzone(double value, double deadzone) {
    value = std::clamp(value, 0.0, 1.0);
    return value <= deadzone ? 0.0 : (value - deadzone) / (1.0 - deadzone);
}

double smooth_activation(double value, double onset, double full) {
    const double t = std::clamp((value-onset)/(full-onset), 0.0, 1.0);
    return t*t*(3.0-2.0*t);
}

std::array<double, 6> apply_pinch_assist(std::array<double, 6> command,
                                         const InspireConfig& cfg) {
    if (!cfg.pinch_assist) return command;
    // 只有食指弯曲与拇指对掌同时出现才判定为捏取意图，避免单独动作互相牵连。
    const double index_intent = smooth_activation(command[3], cfg.pinch_index_on,
                                                   cfg.pinch_index_full);
    const double thumb_intent = smooth_activation(command[5], cfg.pinch_opp_on,
                                                   cfg.pinch_opp_full);
    const double pinch = std::min(index_intent, thumb_intent);
    command[3] = std::max(command[3], pinch*cfg.pinch_index_target);
    command[4] = std::max(command[4], pinch*cfg.pinch_thumb_flex_target);
    command[5] = std::max(command[5], pinch*cfg.pinch_thumb_opp_target);
    return command;
}

struct CommandFilter {
    bool initialized{false};
    std::array<double, 6> value{};

    std::array<double, 6> update(const std::array<double, 6>& input, const InspireConfig& cfg) {
        // 每次遥操都从完全张手命令开始，再按max_step渐进跟随当前手势。
        // 不能让首帧直接等于target，否则100%模式下少量回零残差也可能超过桥接器50 tick限制。
        if (!initialized) {
            value.fill(0.0);
            initialized = true;
        }
        for (int i = 0; i < 6; ++i) {
            const double target = apply_deadzone(input[i], cfg.deadzone);
            const double ema = cfg.ema_alpha*target + (1.0-cfg.ema_alpha)*value[i];
            value[i] += std::clamp(ema - value[i], -cfg.max_step, cfg.max_step);
            value[i] = std::clamp(value[i], 0.0, 1.0);
        }
        return value;
    }
};

class TcpSocket {
public:
    int fd{-1};
    ~TcpSocket() { if (fd >= 0) ::close(fd); }
    void connect_to(const std::string& host, int port) {
        fd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0) throw std::runtime_error("创建TCP套接字失败");
        sockaddr_in address{}; address.sin_family = AF_INET; address.sin_port = htons(port);
        if (::inet_pton(AF_INET, host.c_str(), &address.sin_addr) != 1)
            throw std::runtime_error("只支持IPv4地址：" + host);
        if (::connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0)
            throw std::runtime_error("无法连接 " + host + ":" + std::to_string(port));
    }
    void send_line(const std::string& line) {
        std::size_t sent = 0;
        while (sent < line.size()) {
            const ssize_t n = ::send(fd, line.data()+sent, line.size()-sent, MSG_NOSIGNAL);
            if (n <= 0) throw std::runtime_error("TCP发送失败");
            sent += static_cast<std::size_t>(n);
        }
    }
};

std::array<int, 6> to_inspire_ticks(const std::array<double, 6>& command,
                                    const InspireConfig& cfg) {
    std::array<int, 6> ticks{};
    for (int i = 0; i < 6; ++i) {
        const double limited_closed = cfg.open_tick[i]
            + cfg.range_scale*(cfg.closed_tick[i]-cfg.open_tick[i]);
        ticks[i] = static_cast<int>(std::lround(cfg.open_tick[i]
            + command[i]*(limited_closed-cfg.open_tick[i])));
    }
    return ticks;
}

std::string control_json(const std::array<int, 6>& ticks) {
    std::ostringstream out;
    out << "{\"type\":\"ctrl\",\"angle_set\":[";
    for (int i = 0; i < 6; ++i) { if (i) out << ','; out << ticks[i]; }
    out << "],\"force_set\":[100,100,100,100,100,100],"
           "\"speed_set\":[100,100,100,100,100,100],\"mode\":1}\n";
    return out.str();
}

void run_teleop(_GloveMode_ side, const CalibrationModel& model,
                const std::string& config_path) {
    if (!model.have_open) throw std::runtime_error("请先 load 并执行 zero");
    const InspireConfig cfg = load_inspire_config(config_path);
    std::cout << "准备连接 Inspire 桥接 " << cfg.host << ':' << cfg.port
              << "，行程限制=" << cfg.range_scale*100.0 << "%\n"
              << "捏取协同=" << (cfg.pinch_assist ? "启用" : "关闭") << "\n"
              << "请先保持张手。输入 ARM 后才建立连接并开始发送：" << std::flush;
    std::string confirmation; std::getline(std::cin, confirmation);
    if (confirmation != "ARM") { std::cout << "未确认ARM，已取消。\n"; return; }
    // 必须在人工确认后再连接。若提前连接，等待用户输入期间会触发桥接器500ms看门狗，
    // 导致首次控制包发送到已经关闭的TCP连接。
    TcpSocket socket;
    socket.connect_to(cfg.host, cfg.port);
    // 首包始终是已确认的安全张手位，建立桥接器与遥操滤波器的共同起点。
    socket.send_line(control_json(cfg.open_tick));
    std::cout << "已连接 Inspire 桥接，已发送安全张手首包，开始渐进控制。\n";

    std::atomic<bool> stop{false};
    std::thread worker([&] {
        try {
            CommandFilter filter;
            std::array<double, 6> held{};
            bool have_held = false;
            bool sensor_fault_active = false;
            bool sensor_fault_warned = false;
            std::string sensor_fault_reason;
            std::chrono::steady_clock::time_point sensor_fault_started{};
            const auto period = std::chrono::microseconds(1000000 / cfg.control_hz);
            while (!stop.load()) {
                const auto started = std::chrono::steady_clock::now();
                const Frame frame = latest_frame(side);
                std::string reason;
                if (frame_is_healthy(frame, cfg.stale_hold_ms, reason)) {
                    if (sensor_fault_active) {
                        if (sensor_fault_warned) {
                            const auto fault_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now()-sensor_fault_started).count();
                            std::cerr << "\n[安全看门狗]" << sensor_fault_reason
                                      << "已恢复，持续" << fault_ms << "ms，继续遥操。\n";
                        }
                        sensor_fault_active = false;
                        sensor_fault_warned = false;
                    }
                    const auto raw = compute_command(compute_relatives(frame), model);
                    held = filter.update(apply_pinch_assist(raw, cfg), cfg);
                    have_held = true;
                    socket.send_line(control_json(to_inspire_ticks(held, cfg)));
                } else {
                    const auto age = frame.valid ? std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now()-frame.received).count() : cfg.stale_abort_ms+1;
                    const bool short_timeout = reason.rfind("数据超时", 0) == 0
                                            && age <= cfg.stale_abort_ms;
                    const bool sensor_fault = reason.rfind("节点", 0) == 0;
                    if (sensor_fault && have_held) {
                        const auto now = std::chrono::steady_clock::now();
                        if (!sensor_fault_active) {
                            sensor_fault_active = true;
                            sensor_fault_warned = false;
                            sensor_fault_reason = reason;
                            sensor_fault_started = now;
                        }
                        const auto fault_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now-sensor_fault_started).count();
                        if (!sensor_fault_warned && fault_ms >= cfg.sensor_fault_warn_ms) {
                            sensor_fault_warned = true;
                            std::cerr << "\n[安全看门狗]检测到" << sensor_fault_reason
                                      << "，已持续" << fault_ms << "ms；暂时保持上一命令，超过"
                                      << cfg.sensor_fault_abort_ms << "ms将张手停止。\n";
                        }
                        if (fault_ms <= cfg.sensor_fault_abort_ms) {
                            socket.send_line(control_json(to_inspire_ticks(held, cfg)));
                        } else {
                            socket.send_line(control_json(cfg.open_tick));
                            std::cerr << "\n[安全看门狗]传感器异常持续" << fault_ms
                                      << "ms，已发送张手命令并中止：" << reason << "\n";
                            stop.store(true);
                            break;
                        }
                    } else if (short_timeout && have_held) {
                        socket.send_line(control_json(to_inspire_ticks(held, cfg))); // 短暂丢帧：保持
                    } else {
                        socket.send_line(control_json(cfg.open_tick)); // 严重超时/断线：张手
                        std::cerr << "\n[安全看门狗]已中止控制并发送张手命令：" << reason << "\n";
                        stop.store(true);
                        break;
                    }
                }
                std::this_thread::sleep_until(started + period);
            }
        } catch (const std::exception& e) {
            std::cerr << "\n[遥操线程错误] " << e.what() << "\n";
            stop.store(true);
        }
    });
    std::cout << "遥操已启动，输入 STOP 后回车停止。\n";
    std::string stop_text;
    while (!stop.load() && std::getline(std::cin, stop_text)) if (stop_text == "STOP") break;
    stop.store(true);
    worker.join();
    try { socket.send_line(control_json(cfg.open_tick)); } catch (...) {}
    std::cout << "遥操已停止，已发送张手命令。\n";
}

} // namespace

int main(int argc, char** argv) {
    // 根据编译平台选择官方库；仍可用第一个参数覆盖库路径。
#if defined(__aarch64__)
    const char* default_library = "sdk/lib/arm64/libVDMocapSDK_mHandProArm64.so";
#elif defined(__x86_64__)
    const char* default_library =
        "../computer_debug/mhandpro_x64_sdk/lib/x64/libVDMocapSDK_mHandPro.so";
#else
#error "newteleop 目前只支持 aarch64 和 x86_64"
#endif
    const char* library_path = argc > 1 ? argv[1] : default_library;
    try {
        const std::string serial = find_accessible_glove_serial();
        if (serial.empty()) {
            std::cerr << "启动已取消：没有找到可读写的 /dev/ttyUSB*。\n"
                      << "请重新插拔接收器、确认手套电源和dialout权限，再启动程序。\n";
            return 2;
        }
        std::cout << "检测到手套串口：" << serial << "\n";
        Sdk sdk(library_path);
        float initial_right[NODES_HAND][3]{};
        float initial_left[NODES_HAND][3]{};
        sdk.initial(WS_Geo, initial_right, initial_left);
        sdk.set_break(on_break);

        const _ConnectState_ connected = sdk.connect();
        if (connected == CONNECTED_NONE) {
            std::cerr << "连接失败：请检查 /dev/ttyUSB*、dialout 权限和手套电源。\n";
            return 2;
        }
        sdk.set_dimension(true);
        sdk.set_callback(on_data);

        _GloveMode_ preferred = connected == CONNECTED_RightGlove ? GM_RightGlove : GM_LeftGlove;
        std::cout << "手套已连接："
                  << (connected == CONNECTED_BothGloves ? "BOTH" : side_name(preferred)) << "\n";
        wait_for_frame(preferred);
        print_help();

        CalibrationModel model;
        const std::string default_calibration = preferred == GM_RightGlove
            ? "config/right_hand.calib" : "config/left_hand.calib";

        std::string command;
        while (std::cout << "诊断> " && std::getline(std::cin, command)) {
            if (command == "quit" || command == "q") break;
            if (command == "help" || command == "h") {
                print_help();
            } else if (command == "status") {
                print_status(latest_frame(preferred));
            } else if (command == "pose") {
                float root[4]{};
                sdk.start_calibration(CM_Ppose, root);
                std::cout << "请保持 P-pose 姿势不动...\n";
                while (true) {
                    const auto p = sdk.calibration_progress();
                    std::cout << "\r状态=" << static_cast<int>(p.state)
                              << " 进度=" << std::fixed << std::setprecision(2)
                              << p.progress << std::flush;
                    if (p.state == CS_Successed || p.state == CS_Failed) {
                        std::cout << "\n";
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            } else if (command == "open") {
                model = CalibrationModel{};
                model.open = capture_average_relatives(preferred);
                model.have_open = true;
                std::cout << "已记录张手零点；之前的动作标定已清空。\n";
            } else if (command == "zero") {
                if (!model.have_open) {
                    std::cout << "尚未加载动作标定；请先 load，首次使用则执行 open 和完整标定。\n";
                    continue;
                }
                model.open = capture_average_relatives(preferred);
                std::cout << "已更新本次佩戴的张手零点，动作方向和满量程保持不变。\n";
            } else if (command == "save" || command.rfind("save ", 0) == 0) {
                const std::string path = command.size() > 5 ? command.substr(5) : default_calibration;
                save_calibration(model, path);
                std::cout << "标定已保存到：" << path << "\n";
            } else if (command == "load" || command.rfind("load ", 0) == 0) {
                const std::string path = command.size() > 5 ? command.substr(5) : default_calibration;
                model = load_calibration(path);
                std::cout << "标定已加载：" << path
                          << "\n请自然张手并执行 zero，再用 show 检查回零。\n";
            } else if (command.rfind("calib ", 0) == 0) {
                if (!model.have_open) {
                    std::cout << "请先保持自然张手并输入 open。\n";
                    continue;
                }
                const std::string target = command.substr(6);
                const RelativeSet pose = capture_average_relatives(preferred);
                if (target == "index" || target == "middle" ||
                    target == "ring" || target == "pinky") {
                    int f = target == "index" ? 1 : target == "middle" ? 2
                          : target == "ring" ? 3 : 4;
                    model.flex_axis[f] = directional_feature(model.open, pose, f);
                    const double amplitude = feature_norm(model.flex_axis[f]);
                    model.have_flex[f] = amplitude >= 5.0;
                    std::cout << (model.have_flex[f] ? "已完成" : "标定失败：")
                              << (f==1?"食指":f==2?"中指":f==3?"无名指":"小指")
                              << "独立弯曲有效幅度=" << std::fixed
                              << std::setprecision(1) << amplitude << "度"
                              << (model.have_flex[f] ? "。\n" : "，小于5度，请重新标定。\n");
                } else if (target == "spread") {
                    static const char* finger_names[5] = {"拇指", "食指", "中指", "无名指", "小指"};
                    for (int f = 1; f < 5; ++f) {
                        model.spread_axis[f] = directional_feature(model.open, pose, f);
                        model.have_spread[f] = feature_dot(model.spread_axis[f],
                                                           model.spread_axis[f]) >= 25.0;
                        if (model.have_flex[f] && model.have_spread[f]) {
                            print_axis_quality(finger_names[f], model.flex_axis[f],
                                               model.spread_axis[f]);
                        } else if (!model.have_flex[f]) {
                            std::cout << "  [" << finger_names[f]
                                      << "] 还没有弯曲标定，暂时无法评估解耦质量。\n";
                        } else {
                            std::cout << "  [" << finger_names[f]
                                      << "] 侧摆有效幅度小于5度，将使用单弯曲轴。\n";
                        }
                    }
                    std::cout << "已完成四指侧摆/展开标定。\n";
                } else if (target == "thumb-flex") {
                    model.flex_axis[0] = directional_feature(model.open, pose, 0);
                    model.have_flex[0] = true;
                    std::cout << "已完成拇指弯曲标定。\n";
                } else if (target == "thumb-opp") {
                    model.thumb_opp_axis = directional_feature(model.open, pose, 0);
                    model.have_thumb_opp = true;
                    std::cout << "已完成拇指对掌标定。\n";
                    if (model.have_flex[0]) {
                        print_axis_quality("拇指弯曲/对掌", model.flex_axis[0],
                                           model.thumb_opp_axis);
                    } else {
                        std::cout << "  警告：还没有拇指弯曲标定，请先执行 calib thumb-flex。\n";
                    }
                } else {
                    std::cout << "未知标定项目，请输入 help 查看可用命令。\n";
                }
            } else if (command == "show") {
                if (!model.have_open) { std::cout << "请先输入 open。\n"; continue; }
                print_measurement(latest_frame(preferred), model);
            } else if (command == "vectors") {
                if (!model.have_open) { std::cout << "请先输入 open。\n"; continue; }
                print_vectors(latest_frame(preferred), model.open);
            } else if (command == "monitor") {
                if (!model.have_open) { std::cout << "请先输入 open。\n"; continue; }
                for (int i = 0; i < 50; ++i) {
                    print_measurement(latest_frame(preferred), model);
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
            } else if (command == "teleop" || command.rfind("teleop ", 0) == 0) {
                const std::string default_inspire = preferred == GM_RightGlove
                    ? "config/inspire_right.cfg" : "config/inspire_left.cfg";
                const std::string path = command.size() > 7 ? command.substr(7) : default_inspire;
                run_teleop(preferred, model, path);
            } else if (!command.empty()) {
                std::cout << "未知命令，请输入 help 查看帮助。\n";
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "致命错误：" << e.what() << "\n";
        return 1;
    }
    return 0;
}
