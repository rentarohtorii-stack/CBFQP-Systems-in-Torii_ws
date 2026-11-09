import numpy as np


class AmirKinematics:
    def __init__(self):
        self.L2 = 0.08
        self.L3 = 0.155
        self.L4 = 0.135
        self.L56 = 0.218

    def forward_kinematics(self, q):
        q1, q2, q3, q4 = q

        theta2 = q2 + 0.8
        theta23 = theta2 + q3 - 0.7
        theta234 = theta23 + q4 - 1.78896248

        radius = (
            self.L3 * np.sin(theta2)
            + self.L4 * np.sin(theta23)
            + self.L56 * np.sin(theta234)
        )

        x = -radius * np.sin(q1)
        y = -radius * np.cos(q1)

        z = (
            0.12
            + self.L2
            + self.L3 * np.cos(theta2)
            + self.L4 * np.cos(theta23)
            + self.L56 * np.cos(theta234)
        )

        return np.array([x, y, z])

    def jacobian(self, q):
        q1, q2, q3, q4 = q

        theta2 = q2 + 0.8
        theta23 = theta2 + q3 - 0.7
        theta234 = theta23 + q4 - 1.78896248

        radius = (
            self.L3 * np.sin(theta2)
            + self.L4 * np.sin(theta23)
            + self.L56 * np.sin(theta234)
        )

        dr_dq2 = (
            self.L3 * np.cos(theta2)
            + self.L4 * np.cos(theta23)
            + self.L56 * np.cos(theta234)
        )

        dr_dq3 = (
            self.L4 * np.cos(theta23)
            + self.L56 * np.cos(theta234)
        )

        dr_dq4 = self.L56 * np.cos(theta234)

        dz_dq2 = -(
            self.L3 * np.sin(theta2)
            + self.L4 * np.sin(theta23)
            + self.L56 * np.sin(theta234)
        )

        dz_dq3 = -(
            self.L4 * np.sin(theta23)
            + self.L56 * np.sin(theta234)
        )

        dz_dq4 = -self.L56 * np.sin(theta234)

        jacobian = np.array([
            [
                -radius * np.cos(q1),
                -dr_dq2 * np.sin(q1),
                -dr_dq3 * np.sin(q1),
                -dr_dq4 * np.sin(q1),
            ],
            [
                radius * np.sin(q1),
                -dr_dq2 * np.cos(q1),
                -dr_dq3 * np.cos(q1),
                -dr_dq4 * np.cos(q1),
            ],
            [
                0.0,
                dz_dq2,
                dz_dq3,
                dz_dq4,
            ],
        ])

        return jacobian