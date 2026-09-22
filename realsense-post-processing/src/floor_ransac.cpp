#include "floor_ransac.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include <Eigen/Eigenvalues>

namespace floor_ransac
{

namespace
{
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;
constexpr double kMinCrossNorm = 1e-9;
}  // namespace

std::uint64_t Rng::index(std::uint64_t n)
{
    if (n <= 1) {
        return 0;
    }
    // [0, n) 균일 분포. 나머지 편향을 없애기 위해 limit 이상은 버린다.
    const std::uint64_t bucket = std::numeric_limits<std::uint64_t>::max() / n;
    const std::uint64_t limit = bucket * n;
    std::uint64_t r;
    do {
        r = gen_();
    } while (r >= limit);
    return r / bucket;
}

double Rng::uniform01()
{
    // 상위 53비트 -> double 가수. 구현 의존적인 generate_canonical을 피한다.
    return static_cast<double>(gen_() >> 11) * (1.0 / 9007199254740992.0);
}

Angles plane_angles(const Eigen::Vector3d & n)
{
    Angles a;
    a.pitch_down_deg = std::asin(std::max(-1.0, std::min(1.0, -n[2]))) * kRad2Deg;
    a.roll_deg = std::atan2(n[0], -n[1]) * kRad2Deg;
    return a;
}

Eigen::VectorXd point_heights(const PointsRef & pts, const Plane & plane)
{
    return (pts * plane.n).array() + plane.d;
}

PointsRowMajor select_candidates(
    const PointsRef & organized, int height, int width, const Params & p)
{
    const Eigen::Index expected = static_cast<Eigen::Index>(height) * width;
    if (height <= 0 || width <= 0 || organized.rows() != expected) {
        return PointsRowMajor(0, 3);
    }
    // 화면 하단 roi_bottom 비율만 바닥 후보로 쓴다 (상단은 벽/천장이라 가설을 오염시킴)
    const double frac = std::max(0.0, std::min(1.0, p.roi_bottom));
    const int roi_top = static_cast<int>(static_cast<double>(height) * (1.0 - frac));

    PointsRowMajor out(expected, 3);
    Eigen::Index count = 0;
    for (Eigen::Index row = static_cast<Eigen::Index>(roi_top) * width; row < expected; ++row) {
        const double z = organized(row, 2);
        if (z > p.z_min && z < p.z_max) {
            out.row(count++) = organized.row(row);
        }
    }
    out.conservativeResize(count, 3);
    return out;
}

Result fit_plane(const PointsRef & pts, const Params & p)
{
    Result res;
    const Eigen::Index n_pts = pts.rows();
    res.debug.candidates = static_cast<int>(n_pts);

    if (n_pts < static_cast<Eigen::Index>(std::max(3, p.min_candidates))) {
        res.status = Status::kTooFewCandidates;
        return res;
    }

    Rng rng(p.seed);

    // ---- 가설 평가용 서브샘플 (비복원 추출) -------------------------------
    // 매 가설마다 전체 점을 검사하면 iters * N 이라 너무 비싸다.
    std::vector<Eigen::Index> eval_idx;
    const Eigen::Index eval_n =
        (p.eval_n <= 0) ? n_pts : std::min<Eigen::Index>(p.eval_n, n_pts);
    if (eval_n == n_pts) {
        eval_idx.resize(static_cast<std::size_t>(n_pts));
        std::iota(eval_idx.begin(), eval_idx.end(), Eigen::Index{0});
    } else {
        std::vector<Eigen::Index> pool(static_cast<std::size_t>(n_pts));
        std::iota(pool.begin(), pool.end(), Eigen::Index{0});
        for (Eigen::Index i = 0; i < eval_n; ++i) {  // 부분 Fisher-Yates
            const std::uint64_t j =
                i + rng.index(static_cast<std::uint64_t>(n_pts - i));
            std::swap(pool[static_cast<std::size_t>(i)], pool[static_cast<std::size_t>(j)]);
        }
        eval_idx.assign(pool.begin(), pool.begin() + eval_n);
    }
    res.debug.eval_points = static_cast<int>(eval_n);

    PointsRowMajor eval(eval_n, 3);
    for (Eigen::Index i = 0; i < eval_n; ++i) {
        eval.row(i) = pts.row(eval_idx[static_cast<std::size_t>(i)]);
    }

    // ---- 3점 가설 -> tilt 필터 -> 서브샘플 inlier 투표 ---------------------
    const double cos_tilt = std::cos(p.max_tilt_deg * kDeg2Rad);
    const int iters = std::max(0, p.iters);
    res.debug.hypotheses_tried = iters;
    res.debug.hypothesis_inliers.assign(static_cast<std::size_t>(iters), -1);

    Eigen::Vector3d best_n = Eigen::Vector3d::Zero();
    double best_d = 0.0;
    int best_cnt = -1;

    const std::uint64_t n_draw = static_cast<std::uint64_t>(n_pts);
    for (int it = 0; it < iters; ++it) {
        const Eigen::Vector3d p0 = pts.row(static_cast<Eigen::Index>(rng.index(n_draw)));
        const Eigen::Vector3d p1 = pts.row(static_cast<Eigen::Index>(rng.index(n_draw)));
        const Eigen::Vector3d p2 = pts.row(static_cast<Eigen::Index>(rng.index(n_draw)));

        Eigen::Vector3d nrm = (p1 - p0).cross(p2 - p0);
        const double nn = nrm.norm();
        if (nn < kMinCrossNorm) {  // 세 점이 거의 일직선
            ++res.debug.rejected_degenerate;
            continue;
        }
        nrm /= nn;
        if (std::abs(nrm[1]) < cos_tilt) {  // 바닥이 아니라 벽 -> 기각
            ++res.debug.rejected_tilt;
            continue;
        }
        const double d = -nrm.dot(p0);

        const Eigen::ArrayXd dist = ((eval * nrm).array() + d).abs();
        const int cnt = static_cast<int>((dist < p.thresh).count());

        res.debug.hypothesis_inliers[static_cast<std::size_t>(it)] = cnt;
        if (cnt > best_cnt) {
            best_cnt = cnt;
            best_n = nrm;
            best_d = d;
            res.debug.best_hypothesis = it;
            res.debug.best_eval_inliers = cnt;
        }
    }

    if (best_cnt < 0) {
        res.status = Status::kNoHypothesis;
        return res;
    }
    res.debug.hypothesis_plane.n = best_n[1] > 0.0 ? Eigen::Vector3d(-best_n) : best_n;
    res.debug.hypothesis_plane.d = best_n[1] > 0.0 ? -best_d : best_d;

    // ---- 전체 inlier로 최소제곱 재추정 ------------------------------------
    const Eigen::ArrayXd dist = ((pts * best_n).array() + best_d).abs();
    std::vector<Eigen::Index> inl;
    inl.reserve(static_cast<std::size_t>(n_pts));
    for (Eigen::Index i = 0; i < n_pts; ++i) {
        if (dist[i] < p.thresh) {
            inl.push_back(i);
        }
    }
    if (inl.size() < 3) {
        res.status = Status::kTooFewInliers;
        return res;
    }

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const Eigen::Index i : inl) {
        centroid += pts.row(i).transpose();
    }
    centroid /= static_cast<double>(inl.size());

    // 공분산의 최소 고유벡터 = (inl - centroid)의 최소 특이벡터 = 평면 법선.
    // M x 3 SVD 대신 3x3 고윳값 문제로 풀어 O(M)에 끝낸다 (임베디드 예산 고려).
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const Eigen::Index i : inl) {
        const Eigen::Vector3d q = pts.row(i).transpose() - centroid;
        cov.noalias() += q * q.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    Eigen::Vector3d n = es.eigenvectors().col(0);  // 고윳값 오름차순

    if (n[1] > 0.0) {
        n = -n;  // 법선 방향을 '위'(카메라 -y)로 통일. d 부호로 뒤집으면 안 된다.
    }
    if (std::abs(n[1]) < cos_tilt) {
        // 재추정 후 tilt 재검사. PCL setOptimizeCoefficients는 이걸 해주지 않는다.
        res.status = Status::kTiltRejectedAfterRefit;
        return res;
    }

    res.ok = true;
    res.status = Status::kOk;
    res.plane.n = n;
    res.plane.d = -n.dot(centroid);  // 바닥이 카메라 아래면 d > 0 = 카메라 높이
    res.inliers = static_cast<int>(inl.size());
    return res;
}

const char * status_string(Status s)
{
    switch (s) {
        case Status::kOk: return "ok";
        case Status::kTooFewCandidates: return "too_few_candidates";
        case Status::kNoHypothesis: return "no_hypothesis";
        case Status::kTooFewInliers: return "too_few_inliers";
        case Status::kTiltRejectedAfterRefit: return "tilt_rejected_after_refit";
    }
    return "unknown";
}

}  // namespace floor_ransac
