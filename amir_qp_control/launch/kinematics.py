#!/usr/bin/env python3

import numpy as np


class AmirKinematics:
    """
    AMIRアームの位置順運動学と位置ヤコビアン。

    この運動学では、link0_1またはJoint_1付近を基準として、
    Joint_1～Joint_4から手先位置[x, y, z]を計算する。
    """

    def __init__(self) -> None:
        # Joint_1からJoint_2付近までの高さ
        self.L1 = 0.0
        self.L2 = 0.0

        # アームリンク長
        self.L3 = 0.31
        self.L4 = 0.31

        # Joint_4から手先まで
        self.L56 = 0.07

    def forward_kinematics(
        self,
        q: np.ndarray,
    ) -> np.ndarray:
        """
        関節角度からエンドエフェクタ位置を計算する。

        Parameters
        ----------
        q : array_like, shape (4,)
            [q1, q2, q3, q4] [rad]

        Returns
        -------
        numpy.ndarray, shape (3,)
            エンドエフェクタ位置[x, y, z] [m]
        """

        q = np.asarray(q, dtype=float)

        if q.shape != (4,):
            raise ValueError(
                "qは4要素の配列"
                "[q1, q2, q3, q4]にしてください。"
            )

        q1, q2, q3, q4 = q

        theta2 = q2
        theta23 = q2 + q3
        theta234 = q2 + q3 + q4

        # Joint_1回転軸から手先までの水平距離
        radius = (
            self.L1 +
            self.L3 * np.cos(theta2)
            + self.L4 * np.cos(theta23)
            + self.L56 * np.cos(theta234)
        )

        # Joint_1の回転によってXY平面へ展開
        y = radius * np.cos(q1)
        x = -radius * np.sin(q1)

        # 高さ方向
        z = (
            self.L2
            + self.L3 * np.sin(theta2)
            + self.L4 * np.sin(theta23)
            + self.L56 * np.sin(theta234)
        )

        return np.array(
            [x, y, z],
            dtype=float,
        )

    def jacobian(
        self,
        q: np.ndarray,
    ) -> np.ndarray:
        """
        エンドエフェクタ位置の解析ヤコビアンを計算する。

        [x_dot, y_dot, z_dot]^T = J(q) q_dot

        Returns
        -------
        numpy.ndarray, shape (3, 4)
            位置ヤコビアン
        """

        q = np.asarray(q, dtype=float)

        if q.shape != (4,):
            raise ValueError(
                "qは4要素の配列"
                "[q1, q2, q3, q4]にしてください。"
            )

        q1, q2, q3, q4 = q

        theta2 = q2
        theta23 = q2 + q3
        theta234 = q2 + q3 + q4

        radius = (
            self.L1 +
            self.L3 * np.cos(theta2)
            + self.L4 * np.cos(theta23)
            + self.L56 * np.cos(theta234)
        )

        # radiusを各関節角度で偏微分
        # cos(theta)の微分は-sin(theta)
        dr_dq2 = -(
            self.L3 * np.sin(theta2)
            + self.L4 * np.sin(theta23)
            + self.L56 * np.sin(theta234)
        )

        dr_dq3 = -(
            self.L4 * np.sin(theta23)
            + self.L56 * np.sin(theta234)
        )

        dr_dq4 = -(
            self.L56 * np.sin(theta234)
        )

        # zを各関節角度で偏微分
        # sin(theta)の微分はcos(theta)
        dz_dq2 = (
            self.L3 * np.cos(theta2)
            + self.L4 * np.cos(theta23)
            + self.L56 * np.cos(theta234)
        )

        dz_dq3 = (
            self.L4 * np.cos(theta23)
            + self.L56 * np.cos(theta234)
        )

        dz_dq4 = (
            self.L56 * np.cos(theta234)
        )

        jacobian = np.array(
            [
                [
                    -radius * np.cos(q1),
                    -dr_dq2 * np.sin(q1),
                    -dr_dq3 * np.sin(q1),
                    -dr_dq4 * np.sin(q1),
                ],
                [
                    -radius * np.sin(q1),
                    dr_dq2 * np.cos(q1),
                    dr_dq3 * np.cos(q1),
                    dr_dq4 * np.cos(q1),
                ],
                [
                    0.0,
                    dz_dq2,
                    dz_dq3,
                    dz_dq4,
                ],
            ],
            dtype=float,
        )

        return jacobian

    def CBF_kinematics(
        self,
        q: np.ndarray,
    ) -> np.ndarray:
        """
        アーム上の8個のCBF近似点の位置を計算する。

        Parameters
        ----------
        q : array_like, shape (4,)
            [q1, q2, q3, q4] [rad]

        Returns
        -------
        numpy.ndarray, shape (8, 3)
            R1～R8に対応する位置[x, y, z] [m]。
            points[i]が第i+1近似点の位置を表す。
        """

        q = np.asarray(q, dtype=float)

        if q.shape != (4,):
            raise ValueError(
                "qは4要素の配列"
                "[q1, q2, q3, q4]にしてください。"
            )

        q1, q2, q3, q4 = q

        theta2 = q2
        theta23 = q2 + q3
        theta234 = q2 + q3 + q4

        R1 = self.L1 + 0.4 * self.L3 * np.cos(theta2)
        R2 = self.L1 + 0.7 * self.L3 * np.cos(theta2)
        R3 = self.L1 + self.L3 * np.cos(theta2)
        R4 = R3 + 0.3 * self.L4 * np.cos(theta23)
        R5 = R3 + 0.55 * self.L4 * np.cos(theta23)
        R6 = R3 + 0.8 * self.L4 * np.cos(theta23)
        R7 = R3 + self.L4 * np.cos(theta23)
        R8 = R7 + self.L56 * np.cos(theta234) * 1.3

        R1_Z = self.L2 + 0.4 * self.L3 * np.sin(theta2)
        R2_Z = self.L2 + 0.7 * self.L3 * np.sin(theta2)
        R3_Z = self.L2 + self.L3 * np.sin(theta2)
        R4_Z = R3_Z + 0.3 * self.L4 * np.sin(theta23)
        R5_Z = R3_Z + 0.55 * self.L4 * np.sin(theta23)
        R6_Z = R3_Z + 0.8 * self.L4 * np.sin(theta23)
        R7_Z = R3_Z + self.L4 * np.sin(theta23)
        R8_Z = R7_Z + self.L56 * np.sin(theta234)*1.3

        radii = np.array(
            [R1, R2, R3, R4, R5, R6, R7, R8],
            dtype=float,
        )
        heights = np.array(
            [R1_Z, R2_Z, R3_Z, R4_Z, R5_Z, R6_Z, R7_Z, R8_Z],
            dtype=float,
        )

        points = np.column_stack(
            (
                -radii * np.sin(q1),
                radii * np.cos(q1),
                heights,
            )
        )

        return points

    def CBF_jacobian(
        self,
        q: np.ndarray,
    ) -> np.ndarray:
        """
        アーム上の8個のCBF近似点の解析ヤコビアンを計算する。

        各近似点iについて、
        [x_i_dot, y_i_dot, z_i_dot]^T = J_i(q) q_dot

        Returns
        -------
        numpy.ndarray, shape (8, 3, 4)
            R1～R8に対応する位置ヤコビアン。
            jacobians[i]が第i+1近似点の3×4ヤコビアンを表す。
        """

        q = np.asarray(q, dtype=float)

        if q.shape != (4,):
            raise ValueError(
                "qは4要素の配列"
                "[q1, q2, q3, q4]にしてください。"
            )

        q1, q2, q3, q4 = q

        theta2 = q2
        theta23 = q2 + q3
        theta234 = q2 + q3 + q4

        R1 = self.L1 + 0.4 * self.L3 * np.cos(theta2)
        R2 = self.L1 + 0.7 * self.L3 * np.cos(theta2)
        R3 = self.L1 + self.L3 * np.cos(theta2)
        R4 = R3 + 0.4 * self.L4 * np.cos(theta23)
        R5 = R3 + 0.6 * self.L4 * np.cos(theta23)
        R6 = R3 + 0.8 * self.L4 * np.cos(theta23)
        R7 = R3 + self.L4 * np.cos(theta23)
        R8 = R7 + self.L56 * np.cos(theta234)*1.3
        radii = np.array(
            [R1, R2, R3, R4, R5, R6, R7, R8],
            dtype=float,
        )

        # dR_i / d[q1, q2, q3, q4]。R_iはq1には依存しない。
        dR_dq = np.zeros((8, 4), dtype=float)
        dR_dq[:, 1] = -np.array(
            [
                0.4 * self.L3 * np.sin(theta2),
                0.7 * self.L3 * np.sin(theta2),
                self.L3 * np.sin(theta2),
                self.L3 * np.sin(theta2)
                + 0.4 * self.L4 * np.sin(theta23),
                self.L3 * np.sin(theta2)
                + 0.6 * self.L4 * np.sin(theta23),
                self.L3 * np.sin(theta2)
                + 0.8 * self.L4 * np.sin(theta23),
                self.L3 * np.sin(theta2)
                + self.L4 * np.sin(theta23),
                self.L3 * np.sin(theta2)
                + self.L4 * np.sin(theta23)
                + self.L56 * np.sin(theta234),
            ],
            dtype=float,
        )
        dR_dq[:, 2] = -np.array(
            [
                0.0,
                0.0,
                0.0,
                0.4 * self.L4 * np.sin(theta23),
                0.6 * self.L4 * np.sin(theta23),
                0.8 * self.L4 * np.sin(theta23),
                self.L4 * np.sin(theta23),
                self.L4 * np.sin(theta23)
                + self.L56 * np.sin(theta234),
            ],
            dtype=float,
        )
        dR_dq[7, 3] = -self.L56 * np.sin(theta234)

        # dR_i_Z / d[q1, q2, q3, q4]。R_i_Zはq1には依存しない。
        dZ_dq = np.zeros((8, 4), dtype=float)
        dZ_dq[:, 1] = np.array(
            [
                0.4 * self.L3 * np.cos(theta2),
                0.7 * self.L3 * np.cos(theta2),
                self.L3 * np.cos(theta2),
                self.L3 * np.cos(theta2)
                + 0.4 * self.L4 * np.cos(theta23),
                self.L3 * np.cos(theta2)
                + 0.6 * self.L4 * np.cos(theta23),
                self.L3 * np.cos(theta2)
                + 0.8 * self.L4 * np.cos(theta23),
                self.L3 * np.cos(theta2)
                + self.L4 * np.cos(theta23),
                self.L3 * np.cos(theta2)
                + self.L4 * np.cos(theta23)
                + self.L56 * np.cos(theta234),
            ],
            dtype=float,
        )
        dZ_dq[:, 2] = np.array(
            [
                0.0,
                0.0,
                0.0,
                0.4 * self.L4 * np.cos(theta23),
                0.6 * self.L4 * np.cos(theta23),
                0.8 * self.L4 * np.cos(theta23),
                self.L4 * np.cos(theta23),
                self.L4 * np.cos(theta23)
                + self.L56 * np.cos(theta234),
            ],
            dtype=float,
        )
        dZ_dq[7, 3] = self.L56 * np.cos(theta234)

        jacobians = np.zeros((8, 3, 4), dtype=float)

        # x_i = -R_i sin(q1)
        jacobians[:, 0, 0] = -radii * np.cos(q1)
        jacobians[:, 0, 1:] = -dR_dq[:, 1:] * np.sin(q1)

        # y_i = R_i cos(q1)
        jacobians[:, 1, 0] = -radii * np.sin(q1)
        jacobians[:, 1, 1:] = dR_dq[:, 1:] * np.cos(q1)

        # z_i = R_i_Z
        jacobians[:, 2, :] = dZ_dq

        return jacobians


def numerical_jacobian(
    kinematics: AmirKinematics,
    q: np.ndarray,
    eps: float = 1.0e-7,
) -> np.ndarray:
    """
    順運動学を数値微分してヤコビアンを求める。
    解析ヤコビアンの確認用。
    """

    q = np.asarray(q, dtype=float)

    if q.shape != (4,):
        raise ValueError(
            "qは4要素の配列"
            "[q1, q2, q3, q4]にしてください。"
        )

    jacobian = np.zeros(
        (3, 4),
        dtype=float,
    )

    for index in range(4):
        q_plus = q.copy()
        q_minus = q.copy()

        q_plus[index] += eps
        q_minus[index] -= eps

        position_plus = (
            kinematics.forward_kinematics(q_plus)
        )

        position_minus = (
            kinematics.forward_kinematics(q_minus)
        )

        jacobian[:, index] = (
            position_plus - position_minus
        ) / (2.0 * eps)

    return jacobian


def main() -> None:
    """
    解析ヤコビアンと数値ヤコビアンの単体確認。
    """

    kinematics = AmirKinematics()

    q = np.array(
        [
            0.2,
            0.1,
            -0.4,
            0.3,
        ],
        dtype=float,
    )

    position = kinematics.forward_kinematics(q)
    analytic_jacobian = kinematics.jacobian(q)

    numeric_jacobian = numerical_jacobian(
        kinematics,
        q,
    )

    jacobian_error = (
        analytic_jacobian
        - numeric_jacobian
    )

    print("関節角度q:")
    print(q)

    print("\nエンドエフェクタ位置[x, y, z]:")
    print(position)

    print("\n解析ヤコビアン:")
    print(analytic_jacobian)

    print("\n数値ヤコビアン:")
    print(numeric_jacobian)

    print("\n解析値と数値微分の誤差:")
    print(jacobian_error)

    print("\n最大絶対誤差:")
    print(
        np.max(
            np.abs(jacobian_error)
        )
    )


if __name__ == "__main__":
    main()
