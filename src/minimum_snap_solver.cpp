#include "autopilot/planning/minimum_snap_solver.hpp"

#include <Eigen/Cholesky>
#include <Eigen/LU>

namespace autopilot {

// Helper to compute t-vector derivatives: [0, 0, r!, (r+1)!*t, ...]
Eigen::RowVectorXd MinimumSnapSolver::computeTVector(double t, int r,
                                                     int n_coeffs) const {
  Eigen::RowVectorXd v = Eigen::RowVectorXd::Zero(n_coeffs);
  for (int i = r; i < n_coeffs; ++i) {
    double res = 1.0;
    for (int k = 0; k < r; ++k) {
      res *= (i - k);
    }
    v(i) = res * std::pow(t, i - r);
  }
  return v;
}

// Port of _compute_Q from Python
Eigen::MatrixXd MinimumSnapSolver::computeQ(double duration,
                                            int n_coeffs) const {
  Eigen::MatrixXd qmat = Eigen::MatrixXd::Zero(n_coeffs, n_coeffs);
  // Standard min-snap cost: integral of (derivative order 4)^2
  // You can generalize this with weights as in your Python config
  constexpr int kCostDerivative = 4;
  for (int i = kCostDerivative; i < n_coeffs; ++i) {
    for (int j = kCostDerivative; j < n_coeffs; ++j) {
      double fac_i = 1.0;
      for (int k = 0; k < kCostDerivative; ++k) {
        fac_i *= (i - k);
      }
      double fac_j = 1.0;
      for (int k = 0; k < kCostDerivative; ++k) {
        fac_j *= (j - k);
      }
      qmat(i, j) = fac_i * fac_j *
                   std::pow(duration, i + j - 2 * kCostDerivative + 1) /
                   (i + j - 2 * kCostDerivative + 1);
    }
  }
  return qmat;
}

std::expected<PolynomialTrajectory, std::error_code> MinimumSnapSolver::solve(
    std::span<const TrajectoryWaypoint> waypoints) const {
  if (waypoints.size() < 2) {
    return std::unexpected(make_error_code(AutopilotErrc::kInvalidBufferSize));
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

    qmat_all(ith_chunk, ith_chunk) = computeQ(durs[i], kNCoeffs);

    const auto row_base = static_cast<Eigen::Index>(i) * 2 * kContinuousDegrees;
    const auto row_base_offset = row_base + kContinuousDegrees;
    for (int r = 0; r < kContinuousDegrees; ++r) {
      amat(row_base + r, ith_chunk) = computeTVector(0.0, r, kNCoeffs);
      amat(row_base_offset + r, ith_chunk) =
          computeTVector(durs[i], r, kNCoeffs);
    }
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
    for (int r = 0; r < kContinuousDegrees; ++r) {
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
    }
  }

  // Sub-matrices of R
  Eigen::MatrixXd rpp(p_idx.size(), p_idx.size());
  Eigen::MatrixXd rpf(p_idx.size(), f_idx.size());
  for (Eigen::Index r = 0; r < std::ssize(p_idx); ++r) {
    auto& rth_p = p_idx[static_cast<std::size_t>(r)];
    for (Eigen::Index c = 0; c < std::ssize(p_idx); ++c) {
      rpp(r, c) = rmat(rth_p, p_idx[static_cast<std::size_t>(c)]);
    }
    for (Eigen::Index c = 0; c < std::ssize(f_idx); ++c) {
      rpf(r, c) = rmat(rth_p, f_idx[static_cast<std::size_t>(c)]);
    }
  }

  // 6. Loop per dimension (X, Y, Z)
  Eigen::MatrixXd final_coeffs(3, n_all_coeffs);
  for (int dim = 0; dim < 3; ++dim) {
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
        return std::unexpected(
            make_error_code(AutopilotErrc::kNumericalInstability));
      }
      d_p = -llt.solve(rpf * d_f);
    }

    Eigen::VectorXd d_total = Eigen::VectorXd::Zero(n_d);
    for (std::size_t k = 0; k < f_idx.size(); ++k) {
      d_total(f_idx[k]) = d_f(static_cast<Eigen::Index>(k));
    }
    for (std::size_t k = 0; k < p_idx.size(); ++k) {
      d_total(p_idx[k]) = d_p(static_cast<Eigen::Index>(k));
    }

    final_coeffs.row(dim) = kmat * d_total;
  }

  // 7. Assemble Segments
  for (std::size_t i = 0; i < n_poly_unsigned; ++i) {
    const auto ith_chunk =
        ix::seqN(static_cast<Eigen::Index>(i * kNCoeffs), ix::fix<kNCoeffs>);
    Eigen::Vector<double, 8> cx = final_coeffs(0, ith_chunk).transpose();
    std::array<PlanningPolynomial, 3> axes = {
        PlanningPolynomial(cx),
        PlanningPolynomial(final_coeffs(1, ith_chunk).transpose()),
        PlanningPolynomial(final_coeffs(2, ith_chunk).transpose())};
    segments.emplace_back(durs[i], std::move(axes));
  }

  return PolynomialTrajectory(segments, waypoints.front().time_from_start_secs);
}

}  // namespace autopilot
