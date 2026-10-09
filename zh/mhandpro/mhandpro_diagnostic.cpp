#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <arpa/inet.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <sstream>
#include <thread>
#include <vector>
#include <poll.h>
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
    // SDK 直接给各节点坐标(单位 m)。拇指指腹重定向要用到, 四指用不上 ——
    // 以前这里整块丢掉了。
    std::array<std::array<double, 3>, NODES_HAND> position{};
    std::array<std::array<double, 3>, NODES_HAND> gyr{};
    std::array<std::array<double, 3>, NODES_HAND> acc{};
    bool virtual_valid{false};
    // SDK 虚拟点顺序：拇指、食指、中指、无名指、小指；保存世界坐标，
    // 输出时再统一转换到掌心局部系。
    std::array<std::array<double, 3>, PC_FINGERS_VIRTUAL> fingertip{};
};

using Vec3 = std::array<double, 3>;

// 用四元数把向量转到该四元数的**局部**系: v_local = R(q)^T · v。
Vec3 rotate_into_local(const Quaternion& q, const Vec3& v) {
    const Quaternion c = conjugate(normalize(q));
    // v' = c * (0,v) * c^-1, 展开成实数运算避免再建一个四元数类型。
    const double tx = 2.0 * (c.y * v[2] - c.z * v[1]);
    const double ty = 2.0 * (c.z * v[0] - c.x * v[2]);
    const double tz = 2.0 * (c.x * v[1] - c.y * v[0]);
    return {v[0] + c.w * tx + (c.y * tz - c.z * ty),
            v[1] + c.w * ty + (c.z * tx - c.x * tz),
            v[2] + c.w * tz + (c.x * ty - c.y * tx)};
}

// 节点 0 = 手背, 节点 3 = 拇指末。指腹位置取拇指末端节点相对手背, 并转进
// 手背的局部系 —— 这样整只手怎么挥动都不影响这个量, 只有手指动作影响它。
Vec3 thumb_pad_in_palm(const Frame& f, bool prefer_virtual = false) {
    const Vec3 source = (prefer_virtual && f.virtual_valid)
        ? Vec3{f.fingertip[0][0], f.fingertip[0][1], f.fingertip[0][2]}
        : Vec3{f.position[3][0], f.position[3][1], f.position[3][2]};
    const Vec3 delta = {source[0] - f.position[0][0],
                        source[1] - f.position[0][1],
                        source[2] - f.position[0][2]};
    return rotate_into_local(f.node[0], delta);
}

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
            for (int k = 0; k < 3; ++k)
                f.position[i][k] = static_cast<double>(src.position[i][k]);
            for (int k = 0; k < 3; ++k) {
                f.gyr[i][k] = static_cast<double>(src.gyr[i][k]);
                f.acc[i][k] = static_cast<double>(src.acc[i][k]);
            }
        }
    } catch (...) {
        return;
    }
    dst = f;
}

void copy_virtual_frame(const _GloveMocapDataWithVirtual_& src, Frame& dst) {
    if (!src.isUpdate) return;
    Frame f;
    f.valid = true;
    f.side = src.glove;
    f.frame_index = src.frameIndex;
    f.frequency = src.frequency;
    f.power = src.devicePower;
    f.received = std::chrono::steady_clock::now();
    f.virtual_valid = true;
    try {
        for (int i = 0; i < NODES_HAND; ++i) {
            f.node[i] = from_sdk(src.quaternion[i]);
            f.sensor[i] = src.sensorState[i];
            for (int k = 0; k < 3; ++k) {
                f.position[i][k] = static_cast<double>(src.position[i][k]);
                f.gyr[i][k] = static_cast<double>(src.gyr[i][k]);
                f.acc[i][k] = static_cast<double>(src.acc[i][k]);
            }
        }
        for (int finger = 0; finger < PC_FINGERS_VIRTUAL; ++finger)
            for (int k = 0; k < 3; ++k)
                f.fingertip[finger][k] =
                    static_cast<double>(src.positionVirtual[finger][k]);
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

void on_virtual_data(_GloveMocapDataWithVirtual_ right,
                     _GloveMocapDataWithVirtual_ left) {
    std::lock_guard<std::mutex> lock(g_frame_mutex);
    copy_virtual_frame(right, g_right);
    copy_virtual_frame(left, g_left);
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

// 上位机需要同时查看左右手，不能使用 latest_frame() 的“另一只兜底”语义，
// 否则右手离线时可能把左手帧误标成右手。
Frame frame_for_side(_GloveMode_ side) {
    std::lock_guard<std::mutex> lock(g_frame_mutex);
    return side == GM_LeftGlove ? g_left : g_right;
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
    // 拇指指腹位置重定向用的三个参考点(掌心系, 单位 m), 由 mapcal 采集。
    // 没有它们时拇指退回原来的线性投影 —— 旧 V2 标定文件因此仍然能用。
    bool have_thumb_pad{false};
    bool thumb_pad_virtual{false};
    Vec3 thumb_pad_ref{};      // 比赞: 拇指零位
    Vec3 thumb_pad_palm{};     // 拇指横贴掌心
    Vec3 thumb_pad_pinch{};    // OK 手势
};

// 三个锚点手势对应的因时拇指目标 (对掌 u, 弯曲 v)。
//
// 用**语义手势**而不是"纯对掌/纯弯曲"这类抽象动作, 是因为后者人做不干净:
// "从根部旋向小指掌根"没有任何反馈, 实测右手采出来的轴方向和抓握时的实际
// 移动方向近乎相反(抓握位在该轴上的分量是 -0.88)。这三个手势各有明确形状或
// 触觉终点(拇指贴掌心、拇指碰食指), 人做得准也可重复。
//
// OK 手势的目标不是拍脑袋定的: 由 URDF 正运动学扫出来 —— 食指闭合 0.58 时,
// 拇指(弯曲 0.50, 对掌 1.00) 让两个指腹只差 3.6mm, 是真能捏上的解。
constexpr double kThumbPalmTarget[2] = {1.0, 0.0};   // 拇指横贴掌心
constexpr double kThumbPinchTarget[2] = {1.0, 0.5};  // OK 手势

constexpr const char* kCalibrationMagic = "MHANDPRO_CALIBRATION_V5";
// V2 没有拇指指腹三点, V3 有但**语义不同**(那时三点是 张开/纯弯曲/纯对掌,
// 现在是 比赞/横贴掌心/OK)。两者都仍然接受, 但一律不启用指腹重定向, 拇指
// 走老的线性投影 —— 按新语义去读旧数据不会报错, 只会悄悄标错, 而这份数据
// 是要驱动真手的。V4 继续按节点3位置运行；重标一次 mapcal 升到 V5，
// 新标定才使用 SDK 虚拟指尖，避免用新坐标解释旧锚点。
constexpr const char* kCalibrationMagicV2 = "MHANDPRO_CALIBRATION_V2";
constexpr const char* kCalibrationMagicV3 = "MHANDPRO_CALIBRATION_V3";
constexpr const char* kCalibrationMagicV4 = "MHANDPRO_CALIBRATION_V4";

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
    out << static_cast<int>(model.have_thumb_pad) << '\n';
    out << static_cast<int>(model.thumb_pad_virtual) << '\n';
    for (const Vec3* pad : {&model.thumb_pad_ref, &model.thumb_pad_palm,
                            &model.thumb_pad_pinch}) {
        out << (*pad)[0] << ' ' << (*pad)[1] << ' ' << (*pad)[2] << '\n';
    }
    if (!out) throw std::runtime_error("保存标定文件时发生写入错误");
}

CalibrationModel load_calibration(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("无法打开标定文件：" + path);
    std::string magic;
    std::getline(in, magic);
    const bool v5 = magic == kCalibrationMagic;
    const bool v4 = magic == kCalibrationMagicV4;
    if (!v5 && !v4 && magic != kCalibrationMagicV2 && magic != kCalibrationMagicV3)
        throw std::runtime_error("标定文件版本不兼容");
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
    if (v5 || v4) {
        int pad = 0;
        if (!(in >> pad)) throw std::runtime_error("标定文件缺少拇指指腹标志");
        model.have_thumb_pad = pad != 0;
        if (v5) {
            int virtual_pad = 0;
            if (!(in >> virtual_pad))
                throw std::runtime_error("标定文件缺少虚拟指尖标志");
            model.thumb_pad_virtual = virtual_pad != 0;
        }
        for (Vec3* target : {&model.thumb_pad_ref, &model.thumb_pad_palm,
                             &model.thumb_pad_pinch}) {
            for (int k = 0; k < 3; ++k) {
                if (!(in >> (*target)[k]))
                    throw std::runtime_error("标定文件缺少拇指指腹坐标");
            }
        }
    }
    return model;
}

// 与 capture_average_relatives 同一时长, 同一采集窗口。位置直接算术平均 ——
// 它是坐标不是旋转, 不需要同半球处理。
Vec3 capture_average_thumb_pad(_GloveMode_ side, double seconds = 1.5,
                               bool prefer_virtual = false) {
    Vec3 sum{0.0, 0.0, 0.0};
    int samples = 0;
    const auto end = std::chrono::steady_clock::now()
                   + std::chrono::milliseconds(static_cast<int>(seconds * 1000.0));
    while (std::chrono::steady_clock::now() < end) {
        const Frame frame = latest_frame(side);
        if (frame.valid && (!prefer_virtual || frame.virtual_valid)) {
            const Vec3 pad = thumb_pad_in_palm(frame, prefer_virtual);
            for (int k = 0; k < 3; ++k) sum[k] += pad[k];
            ++samples;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (samples == 0) return {0.0, 0.0, 0.0};
    for (int k = 0; k < 3; ++k) sum[k] /= samples;
    return sum;
}

// 当前指腹位置 → 归一化的 (对掌 u, 弯曲 v)。
//
// 以"比赞"为原点, 在"横贴掌心"和"OK"两个手势张成的平面上做最小二乘分解, 得到
// 两个系数, 再按各自**已知的因时目标**线性组合。垂直于该平面的分量被丢掉,
// 因为因时拇指只有两个自由度, 平面外的动作它做不出来。
//
// 关键在于不再假设"这个姿势就落在某根轴的端点上" —— 那个假设正是之前失败的
// 地方: 人做不出纯对掌, 采出来的轴甚至和抓握方向相反。现在每个手势对应哪个
// 因时姿态是**独立已知的**, 人只要把手势做像就行。
//
// 返回 false = 标定不可用(基底退化), 调用方必须退回线性投影, 不能送 0。
bool thumb_normalized_uv(const CalibrationModel& model, const Vec3& pad,
                         double& u, double& v) {
    if (!model.have_thumb_pad) return false;
    Vec3 a{}, b{}, d{};
    for (int k = 0; k < 3; ++k) {
        a[k] = model.thumb_pad_palm[k] - model.thumb_pad_ref[k];
        b[k] = model.thumb_pad_pinch[k] - model.thumb_pad_ref[k];
        d[k] = pad[k] - model.thumb_pad_ref[k];
    }
    auto dot = [](const Vec3& x, const Vec3& y) {
        return x[0]*y[0] + x[1]*y[1] + x[2]*y[2];
    };
    const double aa = dot(a, a), bb = dot(b, b), ab = dot(a, b);
    const double determinant = aa * bb - ab * ab;
    // 两条轴太短或几乎共线时二维解会放大噪声 —— 这正是原来在旋转空间里出的
    // 问题, 不能在这里重犯。阈值按"两条轴各至少 5mm 且不共线"取。
    if (aa < 2.5e-5 || bb < 2.5e-5 || determinant < 1e-12) return false;
    const double ad = dot(a, d), bd = dot(b, d);
    const double alpha = (bb * ad - ab * bd) / determinant;
    const double beta = (aa * bd - ab * ad) / determinant;
    u = alpha * kThumbPalmTarget[0] + beta * kThumbPinchTarget[0];
    v = alpha * kThumbPalmTarget[1] + beta * kThumbPinchTarget[1];
    return std::isfinite(u) && std::isfinite(v);
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

using GetGestureGlobalFn = void(*)(_Gesture_&, _Gesture_&);
GetGestureGlobalFn g_get_gesture{nullptr};

int gesture_for_side(_GloveMode_ side) {
    if (!g_get_gesture) return 0;
    _Gesture_ right = GESTURE_NONE, left = GESTURE_NONE;
    try { g_get_gesture(right, left); }
    catch (...) { return 0; }
    return static_cast<int>(side == GM_LeftGlove ? left : right);
}

struct Sdk {
    // 按官方头文件/演示程序声明各导出函数的类型。
    using InitialFn = void(*)(_WorldSpace_, float[NODES_HAND][3], float[NODES_HAND][3]);
    using ConnectFn = _ConnectState_(*)();
    using DisconnectFn = void(*)();
    using SetCallbackFn = void(*)(GLOVEMOCAPDATA_CALLBACK);
    using SetVirtualCallbackFn = void(*)(GLOVEMOCAPDATA_VIRTUAL_CALLBACK);
    using SetBreakFn = void(*)(GLOVEBREAK_CALLBACK);
    using SetDimensionFn = void(*)(bool);
    using FastCalibrationFn = void(*)(_GloveMode_);
    using StartCalibrationFn = void(*)(_CalibrationMode_, float[4]);
    using CancelCalibrationFn = void(*)();
    using GetCalibrationProgressFn = _CalibrationProgress_(*)();
    using StartMagCorrectFn = bool(*)();
    using CancelMagCorrectFn = void(*)();
    using EndMagCorrectFn = void(*)();
    using GetDGMagCorrectResultFn = bool(*)(_DGMagCorrectResult_*);
    using GetGestureFn = void(*)(_Gesture_&, _Gesture_&);
    using SetTremorFn = void(*)(_Tremor_, _Tremor_);
    using SetFrequencyFn = void(*)(_Frequency_);

    void* handle{nullptr};
    InitialFn initial{nullptr};
    ConnectFn connect{nullptr};
    DisconnectFn disconnect{nullptr};
    SetCallbackFn set_callback{nullptr};
    SetVirtualCallbackFn set_virtual_callback{nullptr};
    SetBreakFn set_break{nullptr};
    SetDimensionFn set_dimension{nullptr};
    FastCalibrationFn fast_calibration{nullptr};
    StartCalibrationFn start_calibration{nullptr};
    CancelCalibrationFn cancel_calibration{nullptr};
    GetCalibrationProgressFn calibration_progress{nullptr};
    // 磁校准。BAD_MAG 会被 teleop 的安全看门狗判为致命故障并中止控制，
    // 而磁场是环境属性、换个位置往往还在，所以必须能在本工具里现场校准。
    StartMagCorrectFn start_mag_correct{nullptr};
    CancelMagCorrectFn cancel_mag_correct{nullptr};
    EndMagCorrectFn end_mag_correct{nullptr};
    GetDGMagCorrectResultFn dg_mag_result{nullptr};
    GetGestureFn get_gesture{nullptr};
    SetTremorFn set_tremor{nullptr};
    SetFrequencyFn set_frequency{nullptr};

    template <typename T>
    T symbol(const char* name) {
        dlerror();
        auto value = reinterpret_cast<T>(dlsym(handle, name));
        if (const char* error = dlerror()) throw std::runtime_error(error);
        return value;
    }

    template <typename T>
    T optional_symbol(const char* name) {
        dlerror();
        auto value = reinterpret_cast<T>(dlsym(handle, name));
        dlerror();
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
        set_virtual_callback = optional_symbol<SetVirtualCallbackFn>(
            "SetGloveDataWithVirtualCallBackFunc");
        set_break = symbol<SetBreakFn>("SetGloveBreakCallBackFunc");
        set_dimension = symbol<SetDimensionFn>("SetHandDimension");
        fast_calibration = symbol<FastCalibrationFn>("FastCalibration");
        start_calibration = symbol<StartCalibrationFn>("StartCalibration");
        cancel_calibration = symbol<CancelCalibrationFn>("CancelCalibration");
        calibration_progress = symbol<GetCalibrationProgressFn>("GetCalibrationProgress");
        start_mag_correct = symbol<StartMagCorrectFn>("StartMagCorrect");
        cancel_mag_correct = symbol<CancelMagCorrectFn>("CancelMagCorrect");
        end_mag_correct = symbol<EndMagCorrectFn>("EndMagCorrect");
        dg_mag_result = symbol<GetDGMagCorrectResultFn>("GetDGMagCorrectResult");
        get_gesture = optional_symbol<GetGestureFn>("GetGesture");
        set_tremor = optional_symbol<SetTremorFn>("SetTremor");
        set_frequency = optional_symbol<SetFrequencyFn>("SetFrequency");
        // 上位机不再提供数据手套震动反馈；启动时主动清掉
        // SDK/旧进程可能遗留的震动状态，之后不再下发非零等级。
        if (set_tremor) set_tremor(TREMOR_NONE, TREMOR_NONE);
        g_get_gesture = get_gesture;
    }

    ~Sdk() {
        if (set_tremor) set_tremor(TREMOR_NONE, TREMOR_NONE);
        g_get_gesture = nullptr;
        if (disconnect) disconnect();
        if (handle) dlclose(handle);
    }
};

std::string sides_text(const std::vector<_GloveMode_>& sides) {
    std::string text;
    for (const auto side : sides) {
        if (!text.empty()) text += " + ";
        text += side_name(side);
    }
    return text;
}

void print_official_ppose(const std::vector<_GloveMode_>& sides) {
    std::cout
        << "\n========== 官方 P-pose 手掌标定 ==========\n"
        << "本次标定: " << sides_text(sides) << "\n"
        << "1. 身体站直, 手臂向正前方伸直并平举到胸口高度\n"
        << "2. 手掌朝下, 手掌、小臂、大臂尽量保持在同一直线上\n"
        << "3. 食指、中指、无名指、小指并拢伸直, 不要用力反翘\n"
        << "4. 拇指与食指张开约 45~60 度\n"
        << "5. 标定期间身体、手臂、手腕和手指全部保持静止\n"
        << "官方手册提示: 卡在 33%/66% 通常是没有静止绷直或穿戴位置错误。\n";
}

void calibration_countdown() {
    for (int seconds = 3; seconds > 0; --seconds) {
        std::cout << "\r" << seconds << " 秒后开始, 保持标准姿势... "
                  << std::flush;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "\r开始标定, 继续保持不动...       \n";
}

double calibration_percent(float progress) {
    const double value = static_cast<double>(progress);
    // 不同版本的官方 SDK 分别出现过 0~1 和 0~100 两种进度表示。
    return std::clamp(value <= 1.0 ? value * 100.0 : value, 0.0, 100.0);
}

bool run_official_ppose(Sdk& sdk, const std::vector<_GloveMode_>& sides,
                        bool fast) {
    print_official_ppose(sides);
    // 官方标准 P-pose (start_calibration/calibration_progress) 不带手别参数,
    // 本来就是把在线的手套一起标 —— 而 P-pose 的姿势是"双臂前平举"，两只手
    // 同时摆好本来就是自然的。以前只是把另一只的结果丢掉不显示而已。
    // 快速标定 (fast_calibration) 是按手别调用的, 所以逐只调一遍。
    if (sides.size() > 1)
        std::cout << "两只手套一起标定：请**双手**同时摆出 P-pose。\n";
    std::cout << (fast
        ? "快速模式不检查姿势是否合格, 只建议日常重复佩戴且姿势熟练后使用。\n"
        : "标准模式会由官方 SDK 检查姿势并报告成功或失败。\n")
        << "摆好姿势后按回车, 输入 n 回车取消: ";
    std::string reply;
    std::getline(std::cin, reply);
    if (reply == "n" || reply == "N") {
        std::cout << "已取消。\n";
        return false;
    }

    calibration_countdown();
    if (fast) {
        // 官方“快速标定”API没有进度或成功返回值，调用前必须先摆好P-pose。
        for (const auto side : sides) sdk.fast_calibration(side);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "官方快速 P-pose 已执行（该接口不返回姿势质量）: "
                  << sides_text(sides) << "\n";
        return true;
    }

    float root[4]{};
    sdk.start_calibration(CM_Ppose, root);
    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::seconds(30);
    while (std::chrono::steady_clock::now() < deadline) {
        const auto progress = sdk.calibration_progress();
        std::cout << "\r状态=" << static_cast<int>(progress.state)
                  << " 进度=" << std::fixed << std::setprecision(0)
                  << calibration_percent(progress.progress) << "%   "
                  << std::flush;
        if (progress.state == CS_Successed) {
            std::cout << "\n官方 P-pose 标定成功。\n";
            return true;
        }
        if (progress.state == CS_Failed) {
            std::cout << "\n官方 P-pose 标定失败。\n"
                      << "请检查手指是否并拢伸直、手掌是否朝下、手腕是否弯曲，"
                         "以及标定期间是否移动。\n";
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    sdk.cancel_calibration();
    std::cout << "\n官方 P-pose 等待 30 秒仍未结束, 已取消。\n"
              << "先运行 status；如果有 BAD_MAG，请完成 magcal 并重启手套后再试。\n";
    return false;
}

bool prepare_mapping_pose(int step, const char* title, const char* instruction) {
    std::cout << "\n========== mapcal " << step << "/5: " << title
              << " ==========\n"
              << instruction << "\n"
              << "摆好后按回车开始采集；输入 n 回车取消整次 mapcal: ";
    std::string reply;
    std::getline(std::cin, reply);
    if (reply == "n" || reply == "N") {
        std::cout << "mapcal 已取消，原动作映射没有修改。\n";
        return false;
    }
    for (int seconds = 3; seconds > 0; --seconds) {
        std::cout << "\r" << seconds << " 秒后采集, 保持当前姿势... "
                  << std::flush;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "\r开始采集, 继续保持不动...       \n";
    return true;
}

int side_slot(_GloveMode_ side) { return side == GM_RightGlove ? 0 : 1; }

// 双手一起做四姿势映射标定。
//
// 四个姿势本来就是两只手可以同时摆的(握拳/展开/拇指弯/拇指对掌), 一起做有两
// 个好处: 省一半时间, 而且两只手的标定项一定对称 —— 分两次做很容易一只标了
// 侧摆另一只没标, 结果就是"同样的动作左右手反应不一样"。
//
// 每一步要求**所有**在标的手都合格才前进; 有一只不合格就整步重做, 因为姿势
// 是双手一起摆的, 单独重做一只反而更乱。
bool run_mapping_calibration_multi(const std::vector<_GloveMode_>& sides,
                                   CalibrationModel* models,
                                   const std::string* paths) {
    static const char* names[5] = {"拇指", "食指", "中指", "无名指", "小指"};
    const bool dual = sides.size() > 1;
    auto tag = [&](_GloveMode_ side) {
        return dual ? std::string("[") + side_name(side) + "] " : std::string();
    };

    for (const auto side : sides) {
        if (!models[side_slot(side)].have_open) {
            std::cout << tag(side)
                      << "请先执行 calibrate，完成官方 P-pose 和张手回零。\n";
            return false;
        }
    }

    std::cout
        << "\n========== Inspire 六维映射五姿势标定 ==========\n"
        << "本次标定: " << sides_text(sides) << "\n"
        << "这不是厂商传感器标定；它只把官方标定后的手部动作映射成 Inspire 六通道。\n"
        << (dual ? "四个姿势【双手同时】摆, 每个姿势依次采两只手, "
                   "采完之前都不要动。\n" : "")
        << "五个姿势会【依次】采集，不是同时做：\n"
        << "  1. 比赞（四指弯到抓握深度，拇指竖起）\n"
        << "  2. 四指伸直并展开\n"
        << "  3. 只做拇指弯曲\n"
        << "  4. 拇指横贴掌心\n"
        << "  5. OK 手势（拇指指腹捏住食指指腹）\n"
        << "每步质量不足只重做当前步；全部合格后才覆盖保存文件。\n"
        << "\n【重要】第 1 步握拳的深度就是闭合度 1.0 的位置, 第 4 步之后所有\n"
        << "通道都是这两个端点之间的线性插值。握到你**平时遥操会握到**的深度,\n"
        << "不要刻意握得更死 —— 标得比日常深, 用起来就会觉得手指弯得不够。\n";

    std::vector<CalibrationModel> candidates(sides.size());
    for (std::size_t i = 0; i < sides.size(); ++i) {
        candidates[i].have_open = true;
        candidates[i].open = models[side_slot(sides[i])].open;
    }

    constexpr double kMinFingerFlexDeg = 45.0;
    while (true) {
        if (!prepare_mapping_pose(
                1, dual ? "比赞(双手)" : "比赞",
                "四指弯到你平时抓握会用到的深度(比日常再深一点)；"
                "拇指笔直朝上竖起来，像比赞那样。手腕和手掌不要转动。")) {
            return false;
        }
        bool good = true;
        for (std::size_t i = 0; i < sides.size(); ++i) {
            const RelativeSet pose = capture_average_relatives(sides[i]);
            // 比赞时拇指是竖直零位, 正好作指腹重定向的原点 —— 而且它是个有
            // 明确形状的手势, 比"向外张开到某个程度"可重复得多。
            candidates[i].thumb_pad_ref =
                capture_average_thumb_pad(
                    sides[i], 0.5, frame_for_side(sides[i]).virtual_valid);
            candidates[i].thumb_pad_virtual =
                frame_for_side(sides[i]).virtual_valid;
            for (int finger = 1; finger < 5; ++finger) {
                candidates[i].flex_axis[finger] =
                    directional_feature(candidates[i].open, pose, finger);
                const double amplitude =
                    feature_norm(candidates[i].flex_axis[finger]);
                candidates[i].have_flex[finger] = amplitude >= kMinFingerFlexDeg;
                std::cout << "  " << tag(sides[i]) << names[finger]
                          << "有效弯曲幅度=" << std::fixed
                          << std::setprecision(1) << amplitude << "度"
                          << (candidates[i].have_flex[finger]
                              ? "，合格\n" : "，不足45度\n");
                good = good && candidates[i].have_flex[finger];
            }
        }
        if (good) break;
        std::cout << "有通道弯曲幅度不足，请放松张手后重新做第1步。\n";
    }

    while (true) {
        if (!prepare_mapping_pose(
                2, dual ? "四指伸直并展开(双手)" : "四指伸直并展开",
                "四根手指全部保持伸直，再向左右尽量自然展开；"
                "不要弯指，手腕和手掌不要转动。")) {
            return false;
        }
        bool good = true;
        for (std::size_t i = 0; i < sides.size(); ++i) {
            const RelativeSet pose = capture_average_relatives(sides[i]);
            int effective = 0;
            for (int finger = 1; finger < 5; ++finger) {
                candidates[i].spread_axis[finger] =
                    directional_feature(candidates[i].open, pose, finger);
                const double amplitude =
                    feature_norm(candidates[i].spread_axis[finger]);
                candidates[i].have_spread[finger] = amplitude >= 5.0;
                if (candidates[i].have_spread[finger]) ++effective;
                std::cout << "  " << tag(sides[i]) << names[finger]
                          << "侧摆幅度=" << std::fixed << std::setprecision(1)
                          << amplitude << "度"
                          << (candidates[i].have_spread[finger]
                              ? "，有效\n" : "，忽略\n");
            }
            if (effective >= 2) {
                for (int finger = 1; finger < 5; ++finger) {
                    if (candidates[i].have_spread[finger]) {
                        print_axis_quality(names[finger],
                                           candidates[i].flex_axis[finger],
                                           candidates[i].spread_axis[finger]);
                    }
                }
            } else {
                std::cout << "  " << tag(sides[i]) << "只有 " << effective
                          << " 根手指检测到有效展开，至少需要2根。\n";
                good = false;
            }
        }
        if (good) break;
        std::cout << "请回到自然张手后重新做第2步。\n";
    }

    constexpr double kMinThumbFlexDeg = 25.0;
    while (true) {
        if (!prepare_mapping_pose(
                3, dual ? "拇指弯曲(双手)" : "拇指弯曲",
                "其余四指自然伸直；拇指主要弯曲自身关节，"
                "不要横向扫过掌心，手腕和手掌不要转动。")) {
            return false;
        }
        bool good = true;
        for (std::size_t i = 0; i < sides.size(); ++i) {
            // 这一步只喂旧的旋转空间线性投影(重定向不可用时的退路), 指腹
            // 重定向的三个锚点是第1/4/5步的手势, 不含这里。
            const RelativeSet pose = capture_average_relatives(sides[i]);
            candidates[i].flex_axis[0] =
                directional_feature(candidates[i].open, pose, 0);
            const double amplitude = feature_norm(candidates[i].flex_axis[0]);
            candidates[i].have_flex[0] = amplitude >= kMinThumbFlexDeg;
            std::cout << "  " << tag(sides[i]) << "拇指弯曲有效幅度="
                      << std::fixed << std::setprecision(1) << amplitude
                      << "度"
                      << (candidates[i].have_flex[0] ? "，合格\n"
                                                     : "，不足25度\n");
            good = good && candidates[i].have_flex[0];
        }
        if (good) break;
        std::cout << "拇指弯曲幅度不足，请放松拇指后重新做第3步。\n";
    }

    while (true) {
        if (!prepare_mapping_pose(
                4, dual ? "拇指横贴掌心(双手)" : "拇指横贴掌心",
                "四指伸直并拢；拇指横扫过掌心并**贴到掌心上**，"
                "贴住不动。贴到掌心这个触感就是终点，不用凭感觉找最大幅度。")) {
            return false;
        }
        bool good = true;
        for (std::size_t i = 0; i < sides.size(); ++i) {
            const RelativeSet pose = capture_average_relatives(sides[i]);
            candidates[i].thumb_pad_palm =
                capture_average_thumb_pad(
                    sides[i], 0.5, candidates[i].thumb_pad_virtual);
            candidates[i].thumb_opp_axis =
                directional_feature(candidates[i].open, pose, 0);
            const double amplitude = feature_norm(candidates[i].thumb_opp_axis);
            candidates[i].have_thumb_opp =
                amplitude >= 20.0
                && axes_are_separable(candidates[i].flex_axis[0],
                                      candidates[i].thumb_opp_axis);
            std::cout << "  " << tag(sides[i]);
            print_axis_quality("拇指弯曲/对掌",
                               candidates[i].flex_axis[0],
                               candidates[i].thumb_opp_axis);
            // 指腹基底要等第5步的 OK 手势齐了才能判, 这里只负责采点。
            good = good && candidates[i].have_thumb_opp;
        }
        if (good) break;
        std::cout << "拇指对掌幅度不足，或与弯曲动作过于相似；"
                     "请放松拇指后重新做第4步。\n";
    }

    // 第 5 步补齐第三个锚点。三个手势(比赞/横贴掌心/OK)各自对应一个**已知的**
    // 因时拇指姿态, 所以不需要人做出"纯"动作, 只要把手势做像。
    while (true) {
        if (!prepare_mapping_pose(
                5, dual ? "OK 手势(双手)" : "OK 手势",
                "拇指指腹和食指指腹**捏在一起**成一个圈, 其余三指自然伸开。"
                "捏到指腹相碰这个触感就是终点, 保持住。")) {
            return false;
        }
        bool good = true;
        for (std::size_t i = 0; i < sides.size(); ++i) {
            candidates[i].thumb_pad_pinch =
                capture_average_thumb_pad(
                    sides[i], 0.5, candidates[i].thumb_pad_virtual);
            // 三点齐了, 现在才能判基底。
            candidates[i].have_thumb_pad = true;
            Vec3 a{}, b{};
            for (int k = 0; k < 3; ++k) {
                a[k] = candidates[i].thumb_pad_palm[k]
                     - candidates[i].thumb_pad_ref[k];
                b[k] = candidates[i].thumb_pad_pinch[k]
                     - candidates[i].thumb_pad_ref[k];
            }
            const double na = std::sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
            const double nb = std::sqrt(b[0]*b[0]+b[1]*b[1]+b[2]*b[2]);
            double u = 0.0, v = 0.0;
            if (na < 0.005 || nb < 0.005
                || !thumb_normalized_uv(candidates[i],
                                        candidates[i].thumb_pad_pinch, u, v)) {
                candidates[i].have_thumb_pad = false;
                std::cout << "  " << tag(sides[i]) << "三个手势的指腹位置分不开"
                          << "(横贴掌心位移 " << std::fixed
                          << std::setprecision(1) << na*1000 << "mm, OK 位移 "
                          << nb*1000 << "mm), 本次不启用指腹重定向, "
                          << "拇指仍走线性投影。\n";
                good = false;
                continue;
            }
            const double cosine = std::clamp(
                std::abs(a[0]*b[0]+a[1]*b[1]+a[2]*b[2]) / (na*nb), 0.0, 1.0);
            const double angle = std::acos(cosine)*180.0/kPi;
            std::cout << "  " << tag(sides[i]) << "[指腹重定向] 横贴掌心位移="
                      << std::fixed << std::setprecision(1) << na*1000
                      << "mm  OK位移=" << nb*1000 << "mm  两手势夹角="
                      << angle << "度\n";
            if (angle < 20.0) {
                // 两个手势的指腹落点太接近时, 二维分解会放大噪声 —— 这正是
                // 之前在旋转空间里踩的坑, 不能在这里重犯。
                std::cout << "    夹角偏小: 横贴掌心那一步拇指要真的贴到掌心, "
                             "OK 那一步不要连着掌心一起压。请重做第5步。\n";
                candidates[i].have_thumb_pad = false;
                good = false;
            }
        }
        if (good) break;
        std::cout << "请重新做第5步(必要时回头重做第4步)。\n";
    }

    for (std::size_t i = 0; i < sides.size(); ++i) {
        const int slot = side_slot(sides[i]);
        save_calibration(candidates[i], paths[slot]);
        models[slot] = candidates[i];
        std::cout << tag(sides[i]) << "六维动作映射已保存：" << paths[slot]
                  << "\n";
    }
    std::cout << "\nmapcal 五个姿势全部合格。\n"
              << "请依次单独活动每根手指并执行 show 检查。\n";
    return true;
}

bool run_mapping_calibration(_GloveMode_ side, CalibrationModel& model,
                             const std::string& save_path) {
    // 单手就是只标一只的特例, 阈值和判定全部走同一份实现 —— 两份实现一定会
    // 漂, 而这里漂掉的是合格的判据。
    CalibrationModel models[2];
    std::string paths[2];
    models[side_slot(side)] = model;
    paths[side_slot(side)] = save_path;
    const bool ok = run_mapping_calibration_multi({side}, models, paths);
    if (ok) model = models[side_slot(side)];
    return ok;
}

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

const char* node_name(int index) {
    // 0=手背, 之后每根手指 3~4 个节点, 与 _HandNodes_ 的顺序一致。
    static const char* names[NODES_HAND] = {
        "手背", "拇指根", "拇指中", "拇指末",
        "食指根", "食指1", "食指2", "食指末",
        "中指根", "中指1", "中指2", "中指末",
        "无名根", "无名1", "无名2", "无名末",
        "小指根", "小指1", "小指2", "小指末",
    };
    return (index >= 0 && index < NODES_HAND) ? names[index] : "未知";
}

// 磁校准。严格按官方API生命周期:
// StartMagCorrect -> 手套覆盖各轴向采集 -> EndMagCorrect ->
// 轮询 GetDGMagCorrectResult 直到返回 true -> 检查 failedNodes。
// 简易校准约 30s, 深度校准约 75s (厂商 demo 注释)。
void run_mag_correct(Sdk& sdk, const std::vector<_GloveMode_>& sides,
                     int seconds) {
    const bool deep = seconds >= 75;
    std::cout
        << "\n========== 官方" << (deep ? "深度" : "普通")
        << "磁力校准 ==========\n"
        << "本次校准: " << sides_text(sides) << "\n"
        // StartMagCorrect/EndMagCorrect/DGMagResult 都不带手别参数, 结果结构
        // 里同时装着左右手 —— 采集本来就是两只一起做的。所以两只手套都在线
        // 时同时晃两只, 一遍就够, 不用左右各来一次 75 秒。
        << (sides.size() > 1
            ? "两只手套一起校准：请**双手各拿一只**同时按下面的方式转动。\n"
            : "")
        << "官方流程: StartMagCorrect → 采集 → EndMagCorrect → 读取结果\n"
        << "1. 摘下手套, 握住不带传感节点的腕口/接收部分\n"
        << "2. 远离显示器、机箱、音箱、无线充电板和磁性工具\n"
        << "3. 开始后让整只手套缓慢覆盖前后、左右、上下各个朝向\n"
        << "   同时绕三个轴连续翻转, 不要只在一个平面画圈\n"
        << "4. 全程 " << seconds << " 秒, 中途别停\n"
        << "校准完成后【必须重启手套】才生效。\n"
        << "准备好按回车开始, 输入 n 回车取消: ";
    std::string reply;
    std::getline(std::cin, reply);
    if (reply == "n" || reply == "N") {
        std::cout << "已取消。\n";
        return;
    }

    if (!sdk.start_mag_correct()) {
        std::cerr << "StartMagCorrect 返回失败, 手套可能未连接。\n";
        return;
    }

    std::cout << "官方磁校准采集已开始 ——\n";
    const auto collect_until = std::chrono::steady_clock::now()
        + std::chrono::seconds(seconds);
    while (std::chrono::steady_clock::now() < collect_until) {
        const auto left = std::chrono::duration_cast<std::chrono::seconds>(
            collect_until - std::chrono::steady_clock::now()).count();
        std::cout << "\r  剩余 " << left
                  << "s, 继续覆盖三个轴的全部朝向   " << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // 官方 EndMagCorrect 是采集与计算的明确分界；结束前不提前轮询结果。
    std::cout << "\r  采集结束, 正在调用 EndMagCorrect...               \n";
    sdk.end_mag_correct();

    _DGMagCorrectResult_ result{};
    auto side_result = [&](_GloveMode_ side) -> const _MagCorrectResult_& {
        return (side == GM_LeftGlove)
            ? result.LmagCorrectResult : result.RmagCorrectResult;
    };
    bool ready = false;
    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::seconds(30);
    while (std::chrono::steady_clock::now() < deadline) {
        if (sdk.dg_mag_result(&result)) {
            ready = true;
            break;
        }
        std::cout << "\r  等待官方校准结果... 进度 "
                  << std::fixed << std::setprecision(0)
                  << calibration_percent(side_result(sides.front()).progress)
                  << "%   " << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    std::cout << "\n";

    if (!ready) {
        std::cerr << "等待磁校准结果超时, 已取消。\n"
                  << "如果进度一直是 0, 说明 SDK 没采到足够的有效朝向:\n"
                  << "  - 必须绕三个轴翻转并覆盖全部朝向\n"
                  << "  - 附近仍有磁性物体, 换个房间再试\n";
        sdk.cancel_mag_correct();
        return;
    }

    // 逐只报告。一只成功一只失败是常见情况（两只手转动幅度往往不一样），
    // 只看合并结论会把失败的那只放过去。
    for (const auto side : sides) {
        const _MagCorrectResult_& one = side_result(side);
        std::cout << "[" << side_name(side) << "] 进度="
                  << std::fixed << std::setprecision(0)
                  << calibration_percent(one.progress) << "%"
                  << " 结束=" << (one.isFinished ? "是" : "否")
                  << " 有成功节点=" << (one.isHaveSucceed ? "是" : "否") << "\n";
        if (one.failedNodesLength > 0) {
            std::cout << "  失败节点 " << one.failedNodesLength << " 个:";
            for (int i = 0; i < one.failedNodesLength && i < NODES_HAND; ++i) {
                const int index = static_cast<int>(one.failedNodes[i]);
                std::cout << "  " << index << "(" << node_name(index) << ")";
            }
            std::cout << "\n  失败的节点通常是转动幅度不够, 或者附近仍有磁性物体。\n";
        } else if (one.isHaveSucceed && one.isFinished) {
            std::cout << "  全部节点校准成功。\n";
        } else {
            std::cout << "  官方 SDK 没有报告成功节点，这一只本次校准无效；"
                         "请换到低磁干扰位置重试。\n";
        }
    }
    std::cout << "\n【重要】现在把手套关机再开机, 校准才会生效。\n"
              << "重启后回到本工具重新连接, 用 status 确认没有 BAD_MAG。\n\n";
}

// 链路质量实测。teleop 的看门狗只会告诉你"超时 522ms"这一个瞬间, 看不出
// 是偶发一次还是一直在丢。这里按帧号变化统计真实到达间隔, 把"感觉断了"
// 变成可比较的数字。纯本地, 不连任何网络。
void run_link_test(_GloveMode_ side, int seconds) {
    std::cout << "\n链路实测 " << seconds << " 秒, 期间正常戴着手套活动 ——\n";
    std::vector<double> gaps_ms;
    int last_index = -1;
    auto last_arrival = std::chrono::steady_clock::now();
    const auto start = last_arrival;
    const auto until = start + std::chrono::seconds(seconds);
    float power_first = -1.0f, power_last = -1.0f;
    int freq_reported = -1;
    std::array<int, 5> state_counts{};   // 按 _SensorState_ 取值计数
    int sampled_frames = 0;

    while (std::chrono::steady_clock::now() < until) {
        const Frame f = latest_frame(side);
        if (f.valid && f.frame_index != last_index) {
            const auto now = std::chrono::steady_clock::now();
            if (last_index >= 0) {
                gaps_ms.push_back(
                    std::chrono::duration<double, std::milli>(
                        now - last_arrival).count());
            }
            last_index = f.frame_index;
            last_arrival = now;
            if (power_first < 0.0f) power_first = f.power;
            power_last = f.power;
            freq_reported = f.frequency;
            ++sampled_frames;
            for (int i = 0; i < NODES_HAND; ++i) {
                const int s = static_cast<int>(f.sensor[i]);
                if (s >= 0 && s < 5) ++state_counts[s];
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    if (gaps_ms.size() < 2) {
        std::cout << "几乎没收到帧 —— 手套没连上, 或者链路完全断了。\n";
        return;
    }
    std::sort(gaps_ms.begin(), gaps_ms.end());
    const auto at = [&](double q) {
        return gaps_ms[std::min(gaps_ms.size() - 1,
                                static_cast<size_t>(q * gaps_ms.size()))];
    };
    const int over_hold = static_cast<int>(std::count_if(
        gaps_ms.begin(), gaps_ms.end(), [](double v) { return v > 200.0; }));
    const int over_abort = static_cast<int>(std::count_if(
        gaps_ms.begin(), gaps_ms.end(), [](double v) { return v > 500.0; }));

    std::cout << std::fixed << std::setprecision(1)
              << "收到 " << sampled_frames << " 帧, 实测 "
              << sampled_frames / double(seconds) << " Hz"
              << " (SDK 自报 " << freq_reported << " Hz)\n"
              << "帧间隔 ms: 中位 " << at(0.5)
              << "  p95 " << at(0.95)
              << "  最大 " << gaps_ms.back() << "\n"
              << "超过 200ms(暂停阈值) " << over_hold << " 次, "
              << "超过 500ms(中止阈值) " << over_abort << " 次\n"
              << "电量原始值 " << std::setprecision(3)
              << power_first << " -> " << power_last << "\n";

    const char* labels[5] = {"未装", "正常", "无数据", "初始化中", "磁干扰"};
    std::cout << "传感节点状态累计:";
    for (int s = 0; s < 5; ++s) {
        if (state_counts[s]) std::cout << "  " << labels[s] << "="
                                       << state_counts[s];
    }
    std::cout << "\n";

    if (over_abort > 0) {
        std::cout << "\n有超过 500ms 的中断 —— teleop 必然会被看门狗中止。\n"
                  << "按嫌疑排查: 电量 / 接收器被机箱遮挡(换前面板或USB延长线)"
                  << " / 换USB口。\n";
    } else if (over_hold > 0) {
        std::cout << "\n有超过 200ms 的卡顿, teleop 会短暂保持不动但不中止。\n";
    } else {
        std::cout << "\n链路稳定, 这段时间内不会触发看门狗。\n";
    }
    std::cout << "\n";
}

void print_help() {
    std::cout
        << "\n========== mHandPro 官方标定与六维遥操 ==========\n"
        << "  calibrate           官方标准 P-pose 手掌标定（推荐，双手一起）\n"
        << "  quickpose           官方快速 P-pose（不检查姿势质量，双手一起）\n"
        << "  pose                calibrate 的兼容别名\n"
        << "  magcal              官方深度磁力校准, 75秒（双手一起，完成后须重启手套）\n"
        << "  magcal normal       官方普通磁力校准, 30秒\n"
        << "  magcal [秒]         指定采集时长, 限制为10~120秒\n"
        << "  linktest [秒]       链路质量实测, 默认20秒 (掉线/超时时先跑它)\n"
        << "  status              查看连接、帧率和传感节点状态\n"
        << "  show                显示当前关节角和解耦后的六维命令\n"
        << "  snapshot            输出供上位机读取的双手 JSON 快照（只读）\n"
        << "  monitor             以 5 Hz 连续显示 10 秒\n"
        << "  use right|left      切换标定/显示命令作用在哪只手上\n"
        << "  teleop [配置]       以30Hz控制Inspire/仿真（当前这只手）\n"
        << "  teleop both <右> <左>  双手同时跟随，各走各的端口和看门狗\n"
        << "  mapcal              五姿势引导标定六维映射并自动保存（当前这只手）\n"
        << "  mapcal both         五姿势双手一起标，两只手的标定项保证对称\n"
        << "  mapping-help        显示六维动作映射的高级维护命令\n"
        << "  help                显示本帮助\n"
        << "  quit                断开手套并退出\n"
        << "=====================================================\n\n";
}

void print_mapping_help() {
    std::cout
        << "\n========== 六维动作映射（高级维护） ==========\n"
        << "这些命令不是官方 P-pose/磁校准；仅在更换操作者、手套尺寸或映射明显串扰时使用。\n"
        << "  mapcal              推荐：四个姿势依次采集、质量重试、自动保存\n"
        << "  open                采集自然张手零点并清空旧动作映射\n"
        << "  zero                只更新张手零点，保留已加载的动作映射\n"
        << "  calib index         标定食指单独完全弯曲\n"
        << "  calib middle        标定中指单独完全弯曲\n"
        << "  calib ring          标定无名指单独完全弯曲\n"
        << "  calib pinky         标定小指单独完全弯曲\n"
        << "  calib spread        标定四指保持伸直时的最大侧摆/展开\n"
        << "  calib thumb-flex    标定拇指弯曲\n"
        << "  calib thumb-opp     标定拇指对掌\n"
        << "  save [文件]         保存当前手的动作映射\n"
        << "  load [文件]         加载当前手的动作映射\n"
        << "  vectors             显示每个关节的有向旋转向量 [rx ry rz]\n"
        << "===============================================\n\n";
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
        // SS_NONE 表示这个骨架节点本来就没有物理传感器，不是故障。把它算作
        // 异常会出现“60Hz 正常收帧但界面显示离线”的假警报。
        if (f.sensor[i] == SS_NoData || f.sensor[i] == SS_UnReady
            || f.sensor[i] == SS_BadMag) {
            all_ok = false;
            std::cout << "  节点 " << i << ": " << sensor_name(f.sensor[i]) << "\n";
        }
    }
    if (all_ok) std::cout << "所有已安装的传感器节点状态正常。\n";
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
            command[out] = project_one_axis(value, model.flex_axis[f]);
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

bool mapping_complete(const CalibrationModel& model) {
    return model.have_open
        && std::all_of(model.have_flex.begin(), model.have_flex.end(),
                       [](bool value) { return value; })
        && model.have_thumb_opp;
}

void print_snapshot_hand(std::ostream& out, const Frame& frame,
                         const CalibrationModel& model) {
    const bool valid = frame.valid;
    const auto age_ms = valid
        ? std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now() - frame.received).count()
        : -1;
    const bool sensors_ok = valid && std::none_of(
        frame.sensor.begin(), frame.sensor.end(),
        [](_SensorState_ state) {
            return state == SS_NoData || state == SS_UnReady
                || state == SS_BadMag;
        });
    std::array<int, 5> sensor_counts{};
    if (valid) {
        for (const auto state : frame.sensor) {
            const int index = static_cast<int>(state);
            if (index >= 0 && index < static_cast<int>(sensor_counts.size())) {
                ++sensor_counts[index];
            }
        }
    }
    std::array<double, 6> command{};
    double thumb_u = 0.0, thumb_v = 0.0;
    bool have_thumb_uv = false;
    if (valid && model.have_open) {
        const RelativeSet current = compute_relatives(frame);
        command = compute_command(current, model);
        have_thumb_uv = thumb_normalized_uv(
            model, thumb_pad_in_palm(frame, model.thumb_pad_virtual),
            thumb_u, thumb_v);
    }
    std::array<Vec3, PC_FINGERS_VIRTUAL> fingertips_local{};
    double pinch_mm = -1.0;
    if (valid && frame.virtual_valid) {
        for (int finger = 0; finger < PC_FINGERS_VIRTUAL; ++finger) {
            const Vec3 delta = {
                frame.fingertip[finger][0] - frame.position[0][0],
                frame.fingertip[finger][1] - frame.position[0][1],
                frame.fingertip[finger][2] - frame.position[0][2],
            };
            fingertips_local[finger] = rotate_into_local(frame.node[0], delta);
        }
        double squared = 0.0;
        for (int k = 0; k < 3; ++k) {
            const double d = fingertips_local[0][k] - fingertips_local[1][k];
            squared += d*d;
        }
        pinch_mm = std::sqrt(squared) * 1000.0;
    }
    out << "{\"valid\":" << (valid ? "true" : "false")
        << ",\"frame\":" << (valid ? frame.frame_index : -1)
        << ",\"frequency\":" << (valid ? frame.frequency : -1)
        << ",\"power\":" << std::fixed << std::setprecision(3)
        << (valid && std::isfinite(frame.power) ? frame.power : 0.0)
        << ",\"age_ms\":" << age_ms
        << ",\"sensors_ok\":" << (sensors_ok ? "true" : "false")
        << ",\"sensor_counts\":[";
    for (std::size_t i = 0; i < sensor_counts.size(); ++i) {
        if (i) out << ',';
        out << sensor_counts[i];
    }
    out << "]"
        << ",\"calibrated\":" << (mapping_complete(model) ? "true" : "false")
        << ",\"thumb_retarget\":" << (model.have_thumb_pad ? "true" : "false")
        << ",\"thumb_virtual\":" << (model.thumb_pad_virtual ? "true" : "false")
        << ",\"gesture\":" << (valid ? gesture_for_side(frame.side) : 0)
        << ",\"virtual_valid\":"
        << (valid && frame.virtual_valid ? "true" : "false")
        << ",\"pinch_mm\":" << std::fixed << std::setprecision(2) << pinch_mm
        << ",\"fingertips\":";
    if (valid && frame.virtual_valid) {
        out << '[';
        for (int finger = 0; finger < PC_FINGERS_VIRTUAL; ++finger) {
            if (finger) out << ',';
            out << '[' << std::fixed << std::setprecision(6)
                << fingertips_local[finger][0] << ','
                << fingertips_local[finger][1] << ','
                << fingertips_local[finger][2] << ']';
        }
        out << ']';
    } else {
        out << "null";
    }
    out
        << ",\"thumb_uv\":";
    if (have_thumb_uv) {
        out << '[' << std::fixed << std::setprecision(5)
            << std::clamp(thumb_u, 0.0, 1.0) << ','
            << std::clamp(thumb_v, 0.0, 1.0) << ']';
    } else {
        out << "null";
    }
    out
        << ",\"closure\":[";
    for (std::size_t i = 0; i < command.size(); ++i) {
        if (i) out << ',';
        const double value = std::isfinite(command[i])
            ? std::clamp(command[i], 0.0, 1.0) : 0.0;
        out << std::fixed << std::setprecision(5) << value;
    }
    out << "]}";
}

void print_machine_snapshot(const CalibrationModel* models,
                            _GloveMode_ selected) {
    // 固定前缀便于图形界面从人类可读日志中无歧义地抽出这一行。它是只读命令，
    // 不采集、不保存，也不触碰遥操链路。
    // 先拼成一整行再写 stdout，避免遥操线程的看门狗日志插进 JSON 中间。
    std::ostringstream out;
    out << "MHAND_SNAPSHOT {\"selected\":\""
        << (selected == GM_LeftGlove ? "left" : "right")
        << "\",\"disconnected\":"
        << (g_disconnected.load() ? "true" : "false")
        << ",\"hands\":{\"right\":";
    print_snapshot_hand(out, frame_for_side(GM_RightGlove), models[0]);
    out << ",\"left\":";
    print_snapshot_hand(out, frame_for_side(GM_LeftGlove), models[1]);
    out << "}}\n";
    std::cout << out.str();
}

struct InspireConfig {
    std::string host{"192.168.3.85"};
    int port{9102};
    std::array<int, 6> open_tick{500,500,500,500,500,500};
    std::array<int, 6> closed_tick{500,500,500,500,500,500};
    double range_scale{0.30};
    std::string filter{"one_euro"};
    double ema_alpha{0.25};
    double one_euro_min_cutoff{1.5};
    double one_euro_beta{0.10};
    double one_euro_d_cutoff{1.0};
    double deadzone{0.03};
    double max_step{0.04};
    int control_hz{30};
    int stale_hold_ms{200};
    int stale_abort_ms{500};
    int sensor_fault_warn_ms{150};
    int sensor_fault_abort_ms{800};
    // BAD_MAG 是否算致命故障。默认算 —— 驱动真手时磁场异常会让抓握位姿
    // 不可信。只有纯仿真的配置才应该关掉它, 见 inspire_right_sim.cfg。
    bool mag_fault_fatal{true};
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
        else if (key == "filter") cfg.filter = value;
        else if (key == "ema_alpha") cfg.ema_alpha = std::stod(value);
        else if (key == "one_euro_min_cutoff") cfg.one_euro_min_cutoff = std::stod(value);
        else if (key == "one_euro_beta") cfg.one_euro_beta = std::stod(value);
        else if (key == "one_euro_d_cutoff") cfg.one_euro_d_cutoff = std::stod(value);
        else if (key == "deadzone") cfg.deadzone = std::stod(value);
        else if (key == "max_step") cfg.max_step = std::stod(value);
        else if (key == "control_hz") cfg.control_hz = std::stoi(value);
        else if (key == "stale_hold_ms") cfg.stale_hold_ms = std::stoi(value);
        else if (key == "stale_abort_ms") cfg.stale_abort_ms = std::stoi(value);
        else if (key == "sensor_fault_warn_ms") cfg.sensor_fault_warn_ms = std::stoi(value);
        else if (key == "sensor_fault_abort_ms") cfg.sensor_fault_abort_ms = std::stoi(value);
        else if (key == "mag_fault_fatal") cfg.mag_fault_fatal = std::stoi(value) != 0;
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
    if (cfg.filter != "ema" && cfg.filter != "one_euro")
        throw std::runtime_error("filter 只能是 ema 或 one_euro");
    if (cfg.one_euro_min_cutoff <= 0.0 || cfg.one_euro_d_cutoff <= 0.0
        || cfg.one_euro_beta < 0.0)
        throw std::runtime_error("One Euro 参数必须为正数(beta可为0)");
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

bool frame_is_healthy(const Frame& frame, int max_age_ms, std::string& reason,
                      bool mag_fault_fatal = true) {
    if (!frame.valid) { reason = "没有有效数据帧"; return false; }
    if (g_disconnected.load()) { reason = "SDK报告手套断线"; return false; }
    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - frame.received).count();
    if (age > max_age_ms) { reason = "数据超时 " + std::to_string(age) + "ms"; return false; }
    for (int i = 0; i < NODES_HAND; ++i) {
        const auto state = frame.sensor[i];
        // BAD_MAG 只是"当前磁场与标定参考不符", 数据仍在流, 手指的相对
        // 弯曲还大致可用, 只是绝对朝向不可信。仿真里只用闭合度, 因此允许
        // 用 mag_fault_fatal=0 降级为非致命 —— 但真手抓握精度会受影响,
        // 驱动真手的配置绝不能关掉它。NO_DATA / UNREADY 永远是致命的。
        if (state == SS_BadMag && !mag_fault_fatal) continue;
        if (state == SS_BadMag || state == SS_NoData || state == SS_UnReady) {
            reason = "节点" + std::to_string(i) + "状态=" + sensor_name(state);
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
    std::array<double, 6> raw_previous{};
    std::array<double, 6> derivative{};

    static double alpha(double cutoff, double dt) {
        const double tau = 1.0 / (2.0 * kPi * std::max(1e-6, cutoff));
        return 1.0 / (1.0 + tau / std::max(1e-6, dt));
    }

    std::array<double, 6> update(const std::array<double, 6>& input, const InspireConfig& cfg) {
        // 每次遥操都从完全张手命令开始，再按max_step渐进跟随当前手势。
        // 不能让首帧直接等于target，否则100%模式下少量回零残差也可能超过桥接器50 tick限制。
        if (!initialized) {
            value.fill(0.0);
            raw_previous.fill(0.0);
            derivative.fill(0.0);
            initialized = true;
        }
        const double dt = 1.0 / std::max(1, cfg.control_hz);
        for (int i = 0; i < 6; ++i) {
            const double target = apply_deadzone(input[i], cfg.deadzone);
            double filtered = target;
            if (cfg.filter == "one_euro") {
                const double raw_derivative = (target - raw_previous[i]) / dt;
                const double derivative_alpha = alpha(cfg.one_euro_d_cutoff, dt);
                derivative[i] += derivative_alpha
                    * (raw_derivative - derivative[i]);
                const double cutoff = cfg.one_euro_min_cutoff
                    + cfg.one_euro_beta * std::abs(derivative[i]);
                const double signal_alpha = alpha(cutoff, dt);
                filtered = value[i] + signal_alpha * (target - value[i]);
            } else {
                filtered = cfg.ema_alpha*target
                    + (1.0-cfg.ema_alpha)*value[i];
            }
            raw_previous[i] = target;
            value[i] += std::clamp(filtered - value[i],
                                   -cfg.max_step, cfg.max_step);
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

std::string control_json(const std::array<int, 6>& ticks,
                         const double* thumb_uv = nullptr,
                         const Frame* frame = nullptr) {
    std::ostringstream out;
    out << "{\"type\":\"ctrl\",\"angle_set\":[";
    for (int i = 0; i < 6; ++i) { if (i) out << ','; out << ticks[i]; }
    out << "],\"force_set\":[100,100,100,100,100,100],"
           "\"speed_set\":[100,100,100,100,100,100],\"mode\":1";
    // 拇指指腹重定向用的归一化坐标。只在标定齐全时才带; 消费端拿不到就走
    // angle_set 里那两个通道的原值(线性投影), 所以老消费端不受影响。
    if (thumb_uv != nullptr) {
        out << ",\"thumb_uv\":[" << std::fixed << std::setprecision(5)
            << thumb_uv[0] << ',' << thumb_uv[1] << ']';
    }
    if (frame != nullptr && frame->valid) {
        const auto capture_us = std::chrono::duration_cast<std::chrono::microseconds>(
            frame->received.time_since_epoch()).count();
        out << ",\"glove_frame\":" << frame->frame_index
            << ",\"capture_monotonic_us\":" << capture_us
            << ",\"gesture\":" << gesture_for_side(frame->side);
        if (frame->virtual_valid) {
            out << ",\"fingertips\":[";
            for (int finger = 0; finger < PC_FINGERS_VIRTUAL; ++finger) {
                if (finger) out << ',';
                const Vec3 delta = {
                    frame->fingertip[finger][0] - frame->position[0][0],
                    frame->fingertip[finger][1] - frame->position[0][1],
                    frame->fingertip[finger][2] - frame->position[0][2],
                };
                const Vec3 local = rotate_into_local(frame->node[0], delta);
                out << '[' << std::fixed << std::setprecision(6)
                    << local[0] << ',' << local[1] << ',' << local[2] << ']';
            }
            out << ']';
        }
    }
    out << "}\n";
    return out.str();
}

// 一侧手套的遥操通道。
//
// 两只手套共用**一个**接收器和一个 /dev/ttyUSB*, 所以同时驱动双手只能在同一
// 个进程里做 —— 起第二个进程会卡在串口上连不进来。SDK 这一侧本来就同时在收
// 两只手套 (on_data 按 frame.side 分开存), 缺的只是下面这层。
struct TeleopChannel {
    _GloveMode_ side{GM_NONE};
    InspireConfig cfg{};
    CalibrationModel model{};
    TcpSocket socket{};
    // 每侧独立停机。合并成一个标志的话, 左手掉一帧会把正在抓握的右手一起停
    // 掉 —— 两只手是两条独立的降级链, 互不牵连。
    std::atomic<bool> stopped{false};
};

void teleop_loop(TeleopChannel& channel, const std::atomic<bool>& global_stop) {
    const _GloveMode_ side = channel.side;
    const InspireConfig& cfg = channel.cfg;
    const CalibrationModel& model = channel.model;
    TcpSocket& socket = channel.socket;
    std::atomic<bool>& stop = channel.stopped;
    try {
        CommandFilter filter;
        std::array<double, 6> held{};
        bool have_held = false;
        bool sensor_fault_active = false;
        bool sensor_fault_warned = false;
        std::string sensor_fault_reason;
        std::chrono::steady_clock::time_point sensor_fault_started{};
        const auto period = std::chrono::microseconds(1000000 / cfg.control_hz);
        while (!stop.load() && !global_stop.load()) {
            const auto started = std::chrono::steady_clock::now();
            const Frame frame = latest_frame(side);
            std::string reason;
            if (frame_is_healthy(frame, cfg.stale_hold_ms, reason,
                                 cfg.mag_fault_fatal)) {
                if (sensor_fault_active) {
                    if (sensor_fault_warned) {
                        const auto fault_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now()-sensor_fault_started).count();
                        std::cerr << "\n[安全看门狗 " << side_name(side) << "]" << sensor_fault_reason
                                  << "已恢复，持续" << fault_ms << "ms，继续遥操。\n";
                    }
                    sensor_fault_active = false;
                    sensor_fault_warned = false;
                }
                const auto raw = compute_command(compute_relatives(frame), model);
                held = filter.update(apply_pinch_assist(raw, cfg), cfg);
                have_held = true;
                // 指腹归一化坐标随帧带上。**不**在这里改 angle_set 的拇指两
                // 通道 —— 求解放在消费端(URDF 在那边, 运动学只留一份权威),
                // 带不出来时消费端自动退回 angle_set 里的线性投影值。
                double uv[2];
                const bool has_uv = thumb_normalized_uv(
                    model, thumb_pad_in_palm(frame, model.thumb_pad_virtual),
                    uv[0], uv[1]);
                socket.send_line(control_json(to_inspire_ticks(held, cfg),
                                              has_uv ? uv : nullptr, &frame));
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
                        std::cerr << "\n[安全看门狗 " << side_name(side) << "]检测到" << sensor_fault_reason
                                  << "，已持续" << fault_ms << "ms；暂时保持上一命令，超过"
                                  << cfg.sensor_fault_abort_ms << "ms将张手停止。\n";
                    }
                    if (fault_ms <= cfg.sensor_fault_abort_ms) {
                        socket.send_line(control_json(to_inspire_ticks(held, cfg)));
                    } else {
                        socket.send_line(control_json(cfg.open_tick));
                        std::cerr << "\n[安全看门狗 " << side_name(side) << "]传感器异常持续" << fault_ms
                                  << "ms，已发送张手命令并中止：" << reason << "\n";
                        stop.store(true);
                        break;
                    }
                } else if (short_timeout && have_held) {
                    socket.send_line(control_json(to_inspire_ticks(held, cfg))); // 短暂丢帧：保持
                } else {
                    socket.send_line(control_json(cfg.open_tick)); // 严重超时/断线：张手
                    std::cerr << "\n[安全看门狗 " << side_name(side) << "]已中止控制并发送张手命令：" << reason << "\n";
                    stop.store(true);
                    break;
                }
            }
            std::this_thread::sleep_until(started + period);
        }
    } catch (const std::exception& e) {
        std::cerr << "\n[" << side_name(side) << " 遥操线程错误] " << e.what() << "\n";
        stop.store(true);
    }
}

// 连接并驱动 1~2 条通道。单手就是只给一条的特例, 两条路径完全一致。
void run_teleop_channels(std::vector<std::unique_ptr<TeleopChannel>>& channels) {
    if (channels.empty()) throw std::runtime_error("没有可用的遥操通道");
    for (const auto& channel : channels) {
        if (!channel->model.have_open)
            throw std::runtime_error(std::string(side_name(channel->side))
                + " 侧没有张手零点: 先 use "
                + (channel->side == GM_RightGlove ? "right" : "left")
                + " 切过去, load 后执行 zero");
    }
    // 两侧写到同一个 host:port 时, 症状是"另一只手跟着这只动", 很难往配置上
    // 想。这里直接拦掉。
    for (std::size_t i = 0; i + 1 < channels.size(); ++i)
        for (std::size_t j = i + 1; j < channels.size(); ++j)
            if (channels[i]->cfg.host == channels[j]->cfg.host
                && channels[i]->cfg.port == channels[j]->cfg.port)
                throw std::runtime_error("两侧配置指向同一个 host:port, "
                                         "左右手必须各用一个端口");

    for (const auto& channel : channels) {
        std::cout << side_name(channel->side) << " → Inspire 桥接 "
                  << channel->cfg.host << ':' << channel->cfg.port
                  << "，行程限制=" << channel->cfg.range_scale*100.0 << "%"
                  << "，捏取协同=" << (channel->cfg.pinch_assist ? "启用" : "关闭")
                  << "\n";
        if (!channel->cfg.mag_fault_fatal) {
            // 安全门被显式关掉了, 每次都要说一遍, 不能让人忘了它开着。
            std::cout << "【注意】" << side_name(channel->side)
                      << " 配置把 BAD_MAG 降级为非致命 —— 仅限仿真。\n"
                      << "        磁场异常时手指绝对朝向不可信, 不要用它驱动真手。\n";
        }
    }

    std::cout << (channels.size() > 1 ? "请双手都保持张手。" : "请先保持张手。")
              << "输入 ARM 后才建立连接并开始发送：" << std::flush;
    std::string confirmation; std::getline(std::cin, confirmation);
    if (confirmation != "ARM") { std::cout << "未确认ARM，已取消。\n"; return; }

    // 必须在人工确认后再连接。若提前连接，等待用户输入期间会触发桥接器500ms
    // 看门狗，导致首次控制包发送到已经关闭的TCP连接。
    for (const auto& channel : channels) {
        channel->socket.connect_to(channel->cfg.host, channel->cfg.port);
        // 首包始终是已确认的安全张手位。
        channel->socket.send_line(control_json(channel->cfg.open_tick));
        std::cout << "已连接 " << side_name(channel->side)
                  << " 桥接，已发送安全张手首包。\n";
    }

    std::atomic<bool> global_stop{false};
    std::vector<std::thread> workers;
    for (const auto& channel : channels)
        workers.emplace_back([&channel, &global_stop] {
            teleop_loop(*channel, global_stop);
        });

    std::cout << "遥操已启动，输入 STOP 后回车停止。\n";
    CalibrationModel snapshot_models[2];
    for (const auto& channel : channels) {
        const int index = channel->side == GM_RightGlove ? 0 : 1;
        snapshot_models[index] = channel->model;
    }
    const char* snapshot_env = std::getenv("MHANDPRO_STUDIO_SNAPSHOTS");
    const bool stream_snapshots = snapshot_env != nullptr
        && std::string(snapshot_env) == "1";
    std::string stop_text;
    while (true) {
        if (stream_snapshots) {
            pollfd input{STDIN_FILENO, POLLIN, 0};
            const int result = ::poll(&input, 1, 100);
            if (result < 0) {
                if (errno == EINTR) continue;
                throw std::runtime_error("等待 STOP 输入失败");
            }
            if (result > 0 && (input.revents & (POLLIN | POLLHUP))) {
                if (!std::getline(std::cin, stop_text) || stop_text == "STOP") break;
            }
            print_machine_snapshot(snapshot_models, channels.front()->side);
        } else {
            if (!std::getline(std::cin, stop_text) || stop_text == "STOP") break;
        }
        const bool all_stopped = std::all_of(
            channels.begin(), channels.end(),
            [](const std::unique_ptr<TeleopChannel>& c) {
                return c->stopped.load();
            });
        if (all_stopped) {
            std::cout << "所有通道都已被看门狗停止。\n";
            break;
        }
    }
    global_stop.store(true);
    for (auto& worker : workers) worker.join();
    for (const auto& channel : channels) {
        try { channel->socket.send_line(control_json(channel->cfg.open_tick)); }
        catch (...) {}
    }
    std::cout << "遥操已停止，已发送张手命令。\n";
}

void run_teleop(_GloveMode_ side, const CalibrationModel& model,
                const std::string& config_path) {
    auto channel = std::make_unique<TeleopChannel>();
    channel->side = side;
    channel->model = model;
    channel->cfg = load_inspire_config(config_path);
    std::vector<std::unique_ptr<TeleopChannel>> channels;
    channels.push_back(std::move(channel));
    run_teleop_channels(channels);
}

} // namespace

int main(int argc, char** argv) {
    // 用户经常从 ~ 目录用绝对路径启动本程序。所有内置资源都相对可执行文件
    // 所在的 mhandpro 项目定位，不能依赖当前工作目录。
    const std::filesystem::path executable_path =
        std::filesystem::weakly_canonical(std::filesystem::absolute(argv[0]));
    const std::filesystem::path project_dir =
        executable_path.parent_path().parent_path();

    // 根据编译平台选择官方库；仍可用第一个参数覆盖库路径。
#if defined(__aarch64__)
    const char* default_library_relative =
        "sdk/lib/arm64/libVDMocapSDK_mHandProArm64.so";
#elif defined(__x86_64__)
    // 与 arm64 分支一样指向仓库内的 sdk/，不再依赖仓库外的 computer_debug/。
    // 库来自 mHandPro_LinuxSDK/so/ubuntu22.04_x64/（本机是 22.04）；换发行版
    // 要从那个目录换对应的一份，20.04 和 22.04 的构建不是同一个文件。
    const char* default_library_relative =
        "sdk/lib/x64/libVDMocapSDK_mHandPro.so";
#else
#error "newteleop 目前只支持 aarch64 和 x86_64"
#endif
    const std::string library_path = argc > 1
        ? std::string(argv[1])
        : (project_dir / default_library_relative).string();
    // 第二个参数: 两只手套都在线时用哪只。只开一只时不需要。
    _GloveMode_ requested_side = GM_NONE;
    if (argc > 2) {
        const std::string want = argv[2];
        if (want == "right" || want == "R" || want == "r") {
            requested_side = GM_RightGlove;
        } else if (want == "left" || want == "L" || want == "l") {
            requested_side = GM_LeftGlove;
        } else {
            std::cerr << "第二个参数只能是 right 或 left, 收到: " << want << "\n";
            return 2;
        }
    }
    try {
        const std::string serial = find_accessible_glove_serial();
        if (serial.empty()) {
            std::cerr << "启动已取消：没有找到可读写的 /dev/ttyUSB*。\n"
                      << "请重新插拔接收器、确认手套电源和dialout权限，再启动程序。\n";
            return 2;
        }
        std::cout << "检测到手套串口：" << serial << "\n";
        Sdk sdk(library_path.c_str());
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
        if (sdk.set_virtual_callback) {
            sdk.set_virtual_callback(on_virtual_data);
            std::cout << "已启用 SDK 五指虚拟指尖坐标。\n";
        } else {
            sdk.set_callback(on_data);
            std::cout << "当前 SDK 不提供虚拟指尖，使用节点位置兼容模式。\n";
        }
        // 只连了一只就用那一只; 两只都连上时必须显式选, 不能猜 ——
        // 猜错的症状是"另一只手套放在桌上睡着了 -> SDK 报断线 -> 看门狗
        // 中止", 而你手上那只明明好好的, 很难往这上面想。
        _GloveMode_ preferred;
        if (connected == CONNECTED_RightGlove) {
            preferred = GM_RightGlove;
        } else if (connected == CONNECTED_LeftGlove) {
            preferred = GM_LeftGlove;
        } else if (requested_side != GM_NONE) {
            preferred = requested_side;
        } else {
            std::cerr
                << "两只手套都连上了, 但没指定用哪只。\n"
                << "第二个参数给 right 或 left, 例如:\n"
                << "  mhandpro_diagnostic <so路径> right\n"
                << "(只开一只手套时不需要这个参数。)\n";
            return 2;
        }
        std::cout << "手套已连接："
                  << (connected == CONNECTED_BothGloves ? "BOTH" : side_name(preferred))
                  << "，本次使用：" << side_name(preferred) << "\n";
        if (connected == CONNECTED_BothGloves) {
            std::cout << "两只手套都在线：calibrate / magcal 会一起做，"
                         "teleop both 可以同时驱动两只手。\n"
                         "单只 teleop 只用上面这一只，另一只掉线不影响它。\n";
        }
        wait_for_frame(preferred);
        print_help();

        // 左右手各一份标定。双手遥操要同时用到两份, 而 P-pose 和张手零点都是
        // 逐手采的, 两只手不能共用一份 —— 所以这里一次把两侧都读进来, 由 use
        // 决定后面的标定命令作用在哪一侧。
        const std::string calibration_paths[2] = {
            (project_dir / "config" / "right_hand.calib").string(),
            (project_dir / "config" / "left_hand.calib").string(),
        };
        auto side_index = [](_GloveMode_ s) {
            return s == GM_RightGlove ? 0 : 1;
        };
        // 官方 P-pose 和磁校准在 SDK 层都不分手别, 两只在线时本来就是一起
        // 采的 —— 所以这两件事按"在线的所有手套"来做, 而不是按 use 选的那只。
        const std::vector<_GloveMode_> active_sides =
            connected == CONNECTED_BothGloves
                ? std::vector<_GloveMode_>{GM_RightGlove, GM_LeftGlove}
                : std::vector<_GloveMode_>{preferred};
        CalibrationModel models[2];
        for (int i = 0; i < 2; ++i) {
            if (!std::filesystem::exists(calibration_paths[i])) {
                std::cout << "未找到六维动作映射：" << calibration_paths[i]
                          << "\n官方标定仍可使用；首次遥操映射请查看 mapping-help。\n";
                continue;
            }
            try {
                models[i] = load_calibration(calibration_paths[i]);
                std::cout << "六维动作映射已自动加载：" << calibration_paths[i] << "\n";
            } catch (const std::exception& error) {
                std::cout << "六维动作映射自动加载失败：" << calibration_paths[i]
                          << " —— " << error.what()
                          << "\n仍可进行官方标定；遥操前需修复或重新建立动作映射。\n";
            }
        }
        std::cout << "当前标定侧：" << side_name(preferred)
                  << "（use right / use left 切换）\n";
        if (active_sides.size() > 1) {
            std::cout << "calibrate 和 magcal 会**两只手一起**做（"
                      << sides_text(active_sides) << "），各做一遍即可。\n"
                      << "只有 mapcal / calib / show 这类逐指命令才分左右，"
                         "用 use 切换。\n";
        } else {
            std::cout << "请先执行 calibrate；官方 P-pose 成功后会自动更新"
                         "本次张手零点。\n";
        }

        std::string command;
        while (std::cout << "[" << side_name(preferred) << "] 诊断> "
               && std::getline(std::cin, command)) {
            // 每轮重新绑定：use 切侧之后, 下面所有标定命令自动作用到新的一侧,
            // 不用再逐条判断。
            CalibrationModel& model = models[side_index(preferred)];
            const std::string& default_calibration =
                calibration_paths[side_index(preferred)];
            if (command == "quit" || command == "q") break;
            if (command == "help" || command == "h") {
                print_help();
            } else if (command == "mapping-help") {
                print_mapping_help();
            } else if (command == "mapcal both") {
                if (connected != CONNECTED_BothGloves) {
                    std::cout << "只有一只手套在线，无法双手标定。\n";
                    continue;
                }
                run_mapping_calibration_multi(active_sides, models,
                                              calibration_paths);
            } else if (command == "mapcal") {
                run_mapping_calibration(preferred, model, default_calibration);
            } else if (command == "magcal"
                       || command.rfind("magcal ", 0) == 0) {
                // 厂商示例定义普通校准30秒、深度校准75秒。无参数默认深度模式，
                // 因为它只在持续BAD_MAG/传感器磁化时使用，不是日常开机步骤。
                int seconds = 75;
                if (command.size() > 7) {
                    const std::string mode = trim(command.substr(7));
                    if (mode == "normal") {
                        seconds = 30;
                    } else if (mode == "deep") {
                        seconds = 75;
                    } else {
                        try {
                            seconds = std::stoi(mode);
                        } catch (const std::exception&) {
                            std::cout << "参数应为 normal、deep 或秒数；"
                                         "本次使用官方深度模式 75 秒。\n";
                            seconds = 75;
                        }
                    }
                }
                // 官方普通/深度时长为30/75秒；自定义范围仅保留给诊断。
                seconds = std::max(10, std::min(120, seconds));
                run_mag_correct(sdk, active_sides, seconds);
            } else if (command == "linktest"
                       || command.rfind("linktest ", 0) == 0) {
                int seconds = 20;
                if (command.size() > 9) {
                    try {
                        seconds = std::stoi(command.substr(9));
                    } catch (const std::exception&) { seconds = 20; }
                }
                run_link_test(preferred, std::max(5, std::min(120, seconds)));
            } else if (command == "status") {
                print_status(latest_frame(preferred));
            } else if (command == "calibrate" || command == "pose"
                       || command == "quickpose") {
                const bool fast = command == "quickpose";
                if (run_official_ppose(sdk, active_sides, fast)) {
                    // P-pose本身就是官方规定的并指伸直手型。成功后直接用同一
                    // 姿势更新六维映射的张手参考，日常无需再执行load+zero。
                    // 双手一起标时两只手都保持着同一个姿势, 所以两侧的张手
                    // 零点也在这一次里一起采掉, 不用再 use 过去补一遍。
                    std::cout << "继续保持四指并拢伸直，正在同步本次张手零点（"
                              << sides_text(active_sides) << "）。\n";
                    for (const auto side : active_sides) {
                        CalibrationModel& one = models[side_index(side)];
                        one.open = capture_average_relatives(side);
                        one.have_open = true;
                        const bool has_mapping =
                            std::any_of(one.have_flex.begin(),
                                        one.have_flex.end(),
                                        [](bool value) { return value; });
                        std::cout << "[" << side_name(side)
                                  << "] 官方手掌标定与张手回零完成，"
                                  << (has_mapping
                                      ? "已保留自动加载的六维动作映射。\n"
                                      : "但当前没有六维动作映射，"
                                        "遥操前请 use 到这一侧看 mapping-help。\n");
                    }
                    std::cout << "现在用 show 检查；正常即可 teleop。\n";
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
                    model.have_flex[f] = true;
                    std::cout << "已完成" << (f==1?"食指":f==2?"中指":f==3?"无名指":"小指")
                              << "独立弯曲标定。\n";
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
            } else if (command == "snapshot" || command == "snapshot both") {
                print_machine_snapshot(models, preferred);
            } else if (command == "vectors") {
                if (!model.have_open) { std::cout << "请先输入 open。\n"; continue; }
                print_vectors(latest_frame(preferred), model.open);
            } else if (command == "monitor") {
                if (!model.have_open) { std::cout << "请先输入 open。\n"; continue; }
                for (int i = 0; i < 50; ++i) {
                    print_measurement(latest_frame(preferred), model);
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
            } else if (command.rfind("use ", 0) == 0) {
                const std::string want = trim(command.substr(4));
                if (want == "right" || want == "R" || want == "r") {
                    preferred = GM_RightGlove;
                } else if (want == "left" || want == "L" || want == "l") {
                    preferred = GM_LeftGlove;
                } else {
                    std::cout << "use 只接受 right 或 left。\n";
                    continue;
                }
                std::cout << "已切到 " << side_name(preferred)
                          << "；标定命令现在作用于这一侧。\n";
                wait_for_frame(preferred);
            } else if (command.rfind("teleop both", 0) == 0) {
                // 双手同时跟随。两只手套共用一个接收器, 所以必须在这一个进程
                // 里同时驱动 —— 另起一个进程会卡在串口上。
                if (connected != CONNECTED_BothGloves) {
                    std::cout << "只有一只手套在线，无法双手遥操。\n";
                    continue;
                }
                std::istringstream args(command.substr(std::strlen("teleop both")));
                std::string right_cfg, left_cfg;
                args >> right_cfg >> left_cfg;
                if (right_cfg.empty() || left_cfg.empty()) {
                    std::cout << "用法: teleop both <右手配置> <左手配置>\n"
                                 "例如: teleop both config/inspire_right_sim.cfg "
                                 "config/inspire_left_sim.cfg\n";
                    continue;
                }
                // 配置写错不该打死整个会话 —— 刚采的两只手张手零点还在内存
                // 里, 退出就得重标一遍。回到提示符改路径重来即可。
                try {
                    std::vector<std::unique_ptr<TeleopChannel>> channels;
                    for (const auto& item :
                             {std::make_pair(GM_RightGlove, right_cfg),
                              std::make_pair(GM_LeftGlove, left_cfg)}) {
                        auto channel = std::make_unique<TeleopChannel>();
                        channel->side = item.first;
                        channel->model = models[side_index(item.first)];
                        channel->cfg = load_inspire_config(item.second);
                        channels.push_back(std::move(channel));
                    }
                    run_teleop_channels(channels);
                } catch (const std::exception& error) {
                    std::cerr << "双手遥操未启动：" << error.what() << "\n";
                }
            } else if (command == "teleop" || command.rfind("teleop ", 0) == 0) {
                const std::string default_inspire =
                    (project_dir / "config"
                     / (preferred == GM_RightGlove
                        ? "inspire_right.cfg" : "inspire_left.cfg")).string();
                const std::string path = command.size() > 7 ? command.substr(7) : default_inspire;
                try {
                    run_teleop(preferred, model, path);
                } catch (const std::exception& error) {
                    std::cerr << "遥操未启动：" << error.what() << "\n";
                }
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
