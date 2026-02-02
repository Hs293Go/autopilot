#include "autopilot/planning/minimum_snap_solver.hpp"

#include <Eigen/Cholesky>
#include <Eigen/LU>

namespace autopilot {

// Helper to compute t-vector derivatives: [0, 0, r!, (r+1)!*t, ...]
template <int N>
Eigen::RowVector<double, N> computeTVector(double t, int r) {
  Eigen::RowVector<double, N> v = Eigen::RowVector<double, N>::Zero();

  ForILoop<N>([&](int i) {
    if (i >= r) {
      double res = 1.0;
      for (int k = 0; k < r; ++k) {
        res *= (i - k);
      }
      v(i) = res * std::pow(t, i - r);
    }
  });
  return v;
}

// Port of _compute_Q from Python
template <int N>
Eigen::Matrix<double, N, N> computeQ(double duration) {
  Eigen::Matrix<double, N, N> qmat = Eigen::Matrix<double, N, N>::Zero();
  // Standard min-snap cost: integral of (derivative order 4)^2
  // You can generalize this with weights as in your Python config
  constexpr int kCostDerivative = kContinuousDegrees;
  using Arr = Eigen::Array<double, kCostDerivative, 1>;
  ForILoop<kCostDerivative, N>([&](int i) {
    const double fac_i = (i - Arr::NullaryExpr(std::identity())).prod();
    ForILoop<kCostDerivative, N>([&](int j) {
      const double fac_j = (j - Arr::NullaryExpr(std::identity())).prod();

      const auto r = i + j - 2 * kCostDerivative + 1;
      qmat(i, j) = fac_i * fac_j * std::pow(duration, r) / r;
    });
  });
  return qmat;
}

std::expected<PolynomialTrajectory, AutopilotErrc> MinimumSnapSolver::solve(
    std::span<const TrajectoryWaypoint> waypoints,
    const HeadingPolicy& policy) const {
  if (waypoints.size() < 2) {
    return std::unexpected(AutopilotErrc::kInvalidBufferSize);
  }

  constexpr auto kNCoeffs = kPolynomialDegree + 1;  // 8 for degree 7
  const auto n_poly_unsigned = waypoints.size() - 1;
  const auto n_poly = std::ssize(waypoints) - 1;
  const auto n_all_coeffs = n_poly * kNCoeffs;
  const auto n_d =
      (n_poly + 1) * kContinuousDegrees;  // Total derivative variables

  // 1. Durations
  std::vector<double> durs;
  for (std::size_t i = 0; i < n_poly_unsigned; ++i) {
    durs.push_back(waypoints[i + 1].time_from_start_secs -
                   waypoints[i].time_from_start_secs);
  }

  // 2. Build A (Coeffs to Node Derivatives) and Q (Cost)
  Eigen::MatrixXd qmat_all = Eigen::MatrixXd::Zero(n_all_coeffs, n_all_coeffs);
  Eigen::MatrixXd amat = Eigen::MatrixXd::Zero(
      2L * kContinuousDegrees * static_cast<int>(n_poly_unsigned),
      n_all_coeffs);

  for (std::size_t i = 0; i < n_poly_unsigned; ++i) {
    const auto ith_chunk =
        ix::seqN(static_cast<Eigen::Index>(i * kNCoeffs), ix::fix<kNCoeffs>);

    qmat_all(ith_chunk, ith_chunk) = computeQ<kNCoeffs>(durs[i]);

    const auto row_base = static_cast<Eigen::Index>(i) * 2 * kContinuousDegrees;
    const auto row_base_offset = row_base + kContinuousDegrees;
    ForILoop<kContinuousDegrees>([&](int r) {
      amat(row_base + r, ith_chunk) = computeTVector<kNCoeffs>(0.0, r);
      amat(row_base_offset + r, ith_chunk) =
          computeTVector<kNCoeffs>(durs[i], r);
    });
  }

  // 3. Selection Matrix M mapping unique node derivatives to A's output
  Eigen::MatrixXd mmat = Eigen::MatrixXd::Zero(amat.rows(), n_d);
  for (Eigen::Index i = 0; i < n_poly; ++i) {
    auto ith_chunk = [](auto row_start) {
      return ix::seqN(row_start * kContinuousDegrees,
                      ix::fix<kContinuousDegrees>());
    };
    mmat(ith_chunk(i * 2), ith_chunk(i)).setIdentity();
    mmat(ith_chunk(i * 2 + 1), ith_chunk((i + 1))).setIdentity();
  }

  // 4. K = A^-1 * M, and Cost Matrix R = K' * Q * K
  Eigen::MatrixXd kmat = amat.partialPivLu().solve(mmat);
  Eigen::MatrixXd rmat = kmat.transpose() * qmat_all * kmat;

  // 5. Solve for Free Derivatives (d_p)
  std::vector<TrajectorySegment> segments;
  segments.reserve(n_poly_unsigned);

  // We compute the partition Indices once
  std::vector<Eigen::Index> f_idx;
  std::vector<Eigen::Index> p_idx;
  for (int i = 0; i <= n_poly; ++i) {
    const bool is_endpoint = (i == 0 || i == n_poly);
    const auto& wp = waypoints[static_cast<std::size_t>(i)];
    ForILoop<kContinuousDegrees>([&](Eigen::Index r) {
      bool fixed = false;
      switch (r) {
        case 0:
          fixed = true;
          break;
        case 1:
          fixed = is_endpoint || wp.velocity.has_value();
          break;
        case 2:
          fixed = is_endpoint || wp.acceleration.has_value();
          break;
        case 3:
          fixed =
              is_endpoint;  // jerk fixed at endpoints to make solution unique
          break;
        default:
          fixed = false;
          break;
      }

      (fixed ? f_idx : p_idx).push_back(i * kContinuousDegrees + r);
    });
  }

  // Sub-matrices of R
  const Eigen::Ref<const Eigen::MatrixXd> rpp = rmat(p_idx, p_idx);
  const Eigen::Ref<const Eigen::MatrixXd> rpf = rmat(p_idx, f_idx);

  // 6. Loop per dimension (X, Y, Z)
  Eigen::MatrixXd final_coeffs(3, n_all_coeffs);
  const bool success = FilterForILoop<3>([&](auto dim) {
    Eigen::VectorXd d_f(f_idx.size());
    for (std::size_t k = 0; k < f_idx.size(); ++k) {
      const auto res = std::div(f_idx[k], kContinuousDegrees);
      const auto node = static_cast<std::size_t>(res.quot);
      const auto order = res.rem;
      // Fixed derivatives values
      double val;
      switch (order) {
        case 0:
          val = waypoints[node].position[dim];
          break;
        case 1:
          val = waypoints[node].velocity.has_value()
                    ? (*waypoints[node].velocity)[dim]
                    : 0.0;
          break;
        case 2:
          val = waypoints[node].acceleration.has_value()
                    ? (*waypoints[node].acceleration)[dim]
                    : 0.0;
          break;
        default:
          val = 0.0;
          break;
      }
      d_f(static_cast<Eigen::Index>(k)) = val;
    }

    Eigen::VectorXd d_p;
    if (p_idx.empty()) {
      d_p.resize(0);
    } else {
      auto llt = rpp.llt();
      if (llt.info() != Eigen::Success) {
        return false;
      }
      d_p = -llt.solve(rpf * d_f);
    }

    Eigen::VectorXd d_total = Eigen::VectorXd::Zero(n_d);
    d_total(f_idx) = d_f;
    d_total(p_idx) = d_p;

    final_coeffs.row(dim) = kmat * d_total;
    return true;
  });
  if (!success) {
    return std::unexpected(AutopilotErrc::kNumericalInstability);
  }

  // 7. Assemble Segments
  for (std::size_t i = 0; i < n_poly_unsigned; ++i) {
    const auto ith_chunk =
        ix::seqN(static_cast<Eigen::Index>(i * kNCoeffs), ix::fix<kNCoeffs>);
    auto axes = [&]<std::size_t... I>(std::index_sequence<I...>) {
      return std::array{
          PlanningPolynomial(final_coeffs(I, ith_chunk).reshaped())...};
    }(std::make_index_sequence<3>{});

    segments.emplace_back(durs[i], std::move(axes));
  }

  return PolynomialTrajectory(segments, waypoints.front().time_from_start_secs,
                              policy);
}

}  // namespace autopilot
