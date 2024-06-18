// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <matrix/math.hpp>

namespace sym {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: compute_flow_xy_innov_var_and_hx
 *
 * Args:
 *     state: Matrix24_1
 *     P: Matrix23_23
 *     distance: Scalar
 *     R: Scalar
 *     epsilon: Scalar
 *
 * Outputs:
 *     innov_var: Matrix21
 *     H: Matrix23_1
 */
template <typename Scalar>
void ComputeFlowXyInnovVarAndHx(const matrix::Matrix<Scalar, 24, 1>& state,
                                const matrix::Matrix<Scalar, 23, 23>& P, const Scalar distance,
                                const Scalar R, const Scalar epsilon,
                                matrix::Matrix<Scalar, 2, 1>* const innov_var = nullptr,
                                matrix::Matrix<Scalar, 23, 1>* const H = nullptr) {
  // Total ops: 275

  // Input arrays

  // Intermediate terms (42)
  const Scalar _tmp0 = 2 * state(4, 0);
  const Scalar _tmp1 = 2 * state(1, 0);
  const Scalar _tmp2 = _tmp1 * state(6, 0);
  const Scalar _tmp3 = -_tmp0 * state(3, 0) + _tmp2;
  const Scalar _tmp4 =
      Scalar(1.0) /
      (distance + epsilon * (2 * math::min<Scalar>(0, (((distance) > 0) - ((distance) < 0))) + 1));
  const Scalar _tmp5 = (Scalar(1) / Scalar(2)) * _tmp4;
  const Scalar _tmp6 = _tmp5 * state(2, 0);
  const Scalar _tmp7 = 2 * state(3, 0) * state(6, 0);
  const Scalar _tmp8 = _tmp5 * (_tmp0 * state(1, 0) + _tmp7);
  const Scalar _tmp9 = 2 * state(0, 0);
  const Scalar _tmp10 = state(3, 0) * state(5, 0);
  const Scalar _tmp11 = 2 * state(2, 0);
  const Scalar _tmp12 = _tmp11 * state(6, 0);
  const Scalar _tmp13 = -4 * _tmp10 + _tmp12 - _tmp9 * state(4, 0);
  const Scalar _tmp14 = _tmp5 * state(1, 0);
  const Scalar _tmp15 = _tmp9 * state(6, 0);
  const Scalar _tmp16 = _tmp0 * state(2, 0) + _tmp15 - 4 * state(1, 0) * state(5, 0);
  const Scalar _tmp17 = _tmp5 * state(3, 0);
  const Scalar _tmp18 = -_tmp13 * _tmp14 + _tmp16 * _tmp17 - _tmp3 * _tmp6 + _tmp8 * state(0, 0);
  const Scalar _tmp19 = 1 - 2 * std::pow(state(3, 0), Scalar(2));
  const Scalar _tmp20 = _tmp4 * (_tmp19 - 2 * std::pow(state(1, 0), Scalar(2)));
  const Scalar _tmp21 = _tmp3 * _tmp5;
  const Scalar _tmp22 = _tmp5 * state(0, 0);
  const Scalar _tmp23 =
      _tmp13 * _tmp6 + _tmp16 * _tmp22 - _tmp21 * state(1, 0) - _tmp8 * state(3, 0);
  const Scalar _tmp24 =
      _tmp13 * _tmp22 - _tmp16 * _tmp6 - _tmp21 * state(3, 0) + _tmp8 * state(1, 0);
  const Scalar _tmp25 = _tmp9 * state(3, 0);
  const Scalar _tmp26 = _tmp1 * state(2, 0);
  const Scalar _tmp27 = _tmp4 * (-_tmp25 + _tmp26);
  const Scalar _tmp28 = _tmp4 * (_tmp11 * state(3, 0) + _tmp9 * state(1, 0));
  const Scalar _tmp29 = _tmp4 * (_tmp19 - 2 * std::pow(state(2, 0), Scalar(2)));
  const Scalar _tmp30 = 4 * state(4, 0);
  const Scalar _tmp31 = _tmp2 - _tmp30 * state(3, 0) + _tmp9 * state(5, 0);
  const Scalar _tmp32 = 2 * state(5, 0);
  const Scalar _tmp33 = -_tmp15 - _tmp30 * state(2, 0) + _tmp32 * state(1, 0);
  const Scalar _tmp34 = _tmp32 * state(2, 0) + _tmp7;
  const Scalar _tmp35 = 2 * _tmp10 - _tmp12;
  const Scalar _tmp36 = _tmp35 * _tmp5;
  const Scalar _tmp37 = _tmp17 * _tmp33 - _tmp22 * _tmp34 - _tmp31 * _tmp6 + _tmp36 * state(1, 0);
  const Scalar _tmp38 = -_tmp14 * _tmp33 - _tmp22 * _tmp31 + _tmp34 * _tmp6 + _tmp36 * state(3, 0);
  const Scalar _tmp39 = _tmp14 * _tmp31 - _tmp17 * _tmp34 - _tmp22 * _tmp33 + _tmp35 * _tmp6;
  const Scalar _tmp40 = _tmp4 * (_tmp25 + _tmp26);
  const Scalar _tmp41 = _tmp4 * (_tmp1 * state(3, 0) - _tmp9 * state(2, 0));

  // Output terms (2)
  if (innov_var != nullptr) {
    matrix::Matrix<Scalar, 2, 1>& _innov_var = (*innov_var);

    _innov_var(0, 0) = R +
                       _tmp18 * (P(0, 1) * _tmp23 + P(1, 1) * _tmp18 + P(2, 1) * _tmp24 +
                                 P(3, 1) * _tmp27 + P(4, 1) * _tmp20 + P(5, 1) * _tmp28) +
                       _tmp20 * (P(0, 4) * _tmp23 + P(1, 4) * _tmp18 + P(2, 4) * _tmp24 +
                                 P(3, 4) * _tmp27 + P(4, 4) * _tmp20 + P(5, 4) * _tmp28) +
                       _tmp23 * (P(0, 0) * _tmp23 + P(1, 0) * _tmp18 + P(2, 0) * _tmp24 +
                                 P(3, 0) * _tmp27 + P(4, 0) * _tmp20 + P(5, 0) * _tmp28) +
                       _tmp24 * (P(0, 2) * _tmp23 + P(1, 2) * _tmp18 + P(2, 2) * _tmp24 +
                                 P(3, 2) * _tmp27 + P(4, 2) * _tmp20 + P(5, 2) * _tmp28) +
                       _tmp27 * (P(0, 3) * _tmp23 + P(1, 3) * _tmp18 + P(2, 3) * _tmp24 +
                                 P(3, 3) * _tmp27 + P(4, 3) * _tmp20 + P(5, 3) * _tmp28) +
                       _tmp28 * (P(0, 5) * _tmp23 + P(1, 5) * _tmp18 + P(2, 5) * _tmp24 +
                                 P(3, 5) * _tmp27 + P(4, 5) * _tmp20 + P(5, 5) * _tmp28);
    _innov_var(1, 0) = R -
                       _tmp29 * (P(0, 3) * _tmp37 + P(1, 3) * _tmp39 + P(2, 3) * _tmp38 -
                                 P(3, 3) * _tmp29 - P(4, 3) * _tmp40 - P(5, 3) * _tmp41) +
                       _tmp37 * (P(0, 0) * _tmp37 + P(1, 0) * _tmp39 + P(2, 0) * _tmp38 -
                                 P(3, 0) * _tmp29 - P(4, 0) * _tmp40 - P(5, 0) * _tmp41) +
                       _tmp38 * (P(0, 2) * _tmp37 + P(1, 2) * _tmp39 + P(2, 2) * _tmp38 -
                                 P(3, 2) * _tmp29 - P(4, 2) * _tmp40 - P(5, 2) * _tmp41) +
                       _tmp39 * (P(0, 1) * _tmp37 + P(1, 1) * _tmp39 + P(2, 1) * _tmp38 -
                                 P(3, 1) * _tmp29 - P(4, 1) * _tmp40 - P(5, 1) * _tmp41) -
                       _tmp40 * (P(0, 4) * _tmp37 + P(1, 4) * _tmp39 + P(2, 4) * _tmp38 -
                                 P(3, 4) * _tmp29 - P(4, 4) * _tmp40 - P(5, 4) * _tmp41) -
                       _tmp41 * (P(0, 5) * _tmp37 + P(1, 5) * _tmp39 + P(2, 5) * _tmp38 -
                                 P(3, 5) * _tmp29 - P(4, 5) * _tmp40 - P(5, 5) * _tmp41);
  }

  if (H != nullptr) {
    matrix::Matrix<Scalar, 23, 1>& _h = (*H);

    _h.setZero();

    _h(0, 0) = _tmp23;
    _h(1, 0) = _tmp18;
    _h(2, 0) = _tmp24;
    _h(3, 0) = _tmp27;
    _h(4, 0) = _tmp20;
    _h(5, 0) = _tmp28;
  }
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym