use std::{
    fmt::Display,
    ops::{Add, Mul},
};

use crate::{mat4::Mat4, quat::Quat, vec3::Vec3, EPSILON_F32, EPSILON_F64};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Quat2<T = f32>(pub [T; 8]);

macro_rules! float {
    ($(($t:tt, $epsilon:expr)),+) => {
        $(
            impl Quat2<$t> {
                #[inline]
                pub fn new() -> Self {
                    Self([0.0; 8])
                }

                #[inline]
                pub fn new_identity() -> Self {
                    Self([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0])
                }

                #[inline]
                pub fn from_values(x1: $t, y1: $t, z1: $t, w1: $t, x2: $t, y2: $t, z2: $t, w2: $t) -> Self {
                    Self([x1, y1, z1, w1, x2, y2, z2, w2])
                }

                #[inline]
                pub fn from_slice([x1, y1, z1, w1, x2, y2, z2, w2]: &[$t; 8]) -> Self {
                    Self([*x1, *y1, *z1, *w1, *x2, *y2, *z2, *w2])
                }

                #[inline]
                pub fn from_rotation_translation_values(x1: $t, y1: $t, z1: $t, w1: $t, x2: $t, y2: $t, z2: $t) -> Self {
                    let ax = x2 * 0.5;
                    let ay = y2 * 0.5;
                    let az = z2 * 0.5;
                    Self::from_values(
                        x1,
                        y1,
                        z1,
                        w1,
                        ax * w1 + ay * z1 - az * y1,
                        ay * w1 + az * x1 - ax * z1,
                        az * w1 + ax * y1 - ay * x1,
                        -ax * x1 - ay * y1 - az * z1,
                    )
                }

                #[inline]
                pub fn from_rotation_translation(q: &Quat::<$t>, t: &Vec3::<$t>) -> Self {
                    let ax = t.0[0] * 0.5;
                    let ay = t.0[1] * 0.5;
                    let az = t.0[2] * 0.5;
                    let bx = q.0[0];
                    let by = q.0[1];
                    let bz = q.0[2];
                    let bw = q.0[3];
                    Self::from_values(
                        bx,
                        by,
                        bz,
                        bw,
                        ax * bw + ay * bz - az * by,
                        ay * bw + az * bx - ax * bz,
                        az * bw + ax * by - ay * bx,
                        -ax * bx - ay * by - az * bz,
                    )
                }

                #[inline]
                pub fn from_translation(t: &Vec3::<$t>) -> Self {
                    Self::from_values(
                        0.0,
                        0.0,
                        0.0,
                        1.0,
                        t.0[0] * 0.5,
                        t.0[1] * 0.5,
                        t.0[2] * 0.5,
                        0.0,
                    )
                }

                #[inline]
                pub fn from_rotation(q: &Quat::<$t>) -> Self {
                    Self::from_values(
                        q.0[0],
                        q.0[1],
                        q.0[2],
                        q.0[3],
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                    )
                }

                #[inline]
                pub fn from_mat4(a: &Mat4::<$t>) -> Self {
                    let outer = a.rotation();
                    let t = a.translation();
                    Self::from_rotation_translation(&outer, &t)
                }
            }

            impl Quat2<$t> {
                #[inline]
                pub fn raw(&self) -> &[$t; 8] {
                    &self.0
                }

                #[inline]
                pub fn real(&self) -> Quat::<$t> {
                    Quat::<$t>::from_values(
                        self.0[0],
                        self.0[1],
                        self.0[2],
                        self.0[3],
                    )
                }

                #[inline]
                pub fn dual(&self) -> Quat::<$t> {
                    Quat::<$t>::from_values(
                        self.0[4],
                        self.0[5],
                        self.0[6],
                        self.0[7],
                    )
                }

                #[inline]
                pub fn translation(&self) -> Vec3::<$t> {
                    let ax = self.0[4];
                    let ay = self.0[5];
                    let az = self.0[6];
                    let aw = self.0[7];
                    let bx = -self.0[0];
                    let by = -self.0[1];
                    let bz = -self.0[2];
                    let bw = self.0[3];
                    Vec3::<$t>::from_values(
                        (ax * bw + aw * bx + ay * bz - az * by) * 2.0,
                        (ay * bw + aw * by + az * bx - ax * bz) * 2.0,
                        (az * bw + aw * bz + ax * by - ay * bx) * 2.0,
                    )
                }

                #[inline]
                pub fn set(&mut self, x1: $t, y1: $t, z1: $t, w1: $t, x2: $t, y2: $t, z2: $t, w2: $t) -> &mut Self {
                    self.0[0] = x1;
                    self.0[1] = y1;
                    self.0[2] = z1;
                    self.0[3] = w1;
                    self.0[4] = x2;
                    self.0[5] = y2;
                    self.0[6] = z2;
                    self.0[7] = w2;
                    self
                }

                #[inline]
                pub fn set_slice(&mut self, [x1, y1, z1, w1, x2, y2, z2, w2]: &[$t; 8]) -> &mut Self {
                    self.0[0] = *x1;
                    self.0[1] = *y1;
                    self.0[2] = *z1;
                    self.0[3] = *w1;
                    self.0[4] = *x2;
                    self.0[5] = *y2;
                    self.0[6] = *z2;
                    self.0[7] = *w2;
                    self
                }

                #[inline]
                pub fn set_zero(&mut self) -> &mut Self {
                    self.0[0] = 0.0;
                    self.0[1] = 0.0;
                    self.0[2] = 0.0;
                    self.0[3] = 0.0;
                    self.0[4] = 0.0;
                    self.0[5] = 0.0;
                    self.0[6] = 0.0;
                    self.0[7] = 0.0;
                    self
                }

                #[inline]
                pub fn set_identify(&mut self) -> &mut Self {
                    self.0[0] = 0.0;
                    self.0[1] = 0.0;
                    self.0[2] = 0.0;
                    self.0[3] = 1.0;
                    self.0[4] = 0.0;
                    self.0[5] = 0.0;
                    self.0[6] = 0.0;
                    self.0[7] = 0.0;
                    self
                }

                #[inline]
                pub fn set_real(&mut self, q: &Quat::<$t>) -> &mut Self {
                    self.0[0] = q.0[0];
                    self.0[1] = q.0[1];
                    self.0[2] = q.0[2];
                    self.0[3] = q.0[3];
                    self
                }

                #[inline]
                pub fn set_dual(&mut self, q: &Quat::<$t>) -> &mut Self {
                    self.0[4] = q.0[0];
                    self.0[5] = q.0[1];
                    self.0[6] = q.0[2];
                    self.0[7] = q.0[3];
                    self
                }

                #[inline]
                pub fn translate(&self, v: &Vec3::<$t>) -> Self {
                    let ax1 = self.0[0];
                    let ay1 = self.0[1];
                    let az1 = self.0[2];
                    let aw1 = self.0[3];
                    let bx1 = v.0[0] * 0.5;
                    let by1 = v.0[1] * 0.5;
                    let bz1 = v.0[2] * 0.5;
                    let ax2 = self.0[4];
                    let ay2 = self.0[5];
                    let az2 = self.0[6];
                    let aw2 = self.0[7];
                    Self::from_values(
                        ax1,
                        ay1,
                        az1,
                        aw1,
                        aw1 * bx1 + ay1 * bz1 - az1 * by1 + ax2,
                        aw1 * by1 + az1 * bx1 - ax1 * bz1 + ay2,
                        aw1 * bz1 + ax1 * by1 - ay1 * bx1 + az2,
                        -ax1 * bx1 - ay1 * by1 - az1 * bz1 + aw2,
                    )
                }

                #[inline]
                pub fn rotate_by_quat_append(&self, q: &Quat::<$t>) -> Self {
                    let mut out = Self::new();

                    let qx = q.0[0];
                    let qy = q.0[1];
                    let qz = q.0[2];
                    let qw = q.0[3];
                    let mut ax = self.0[0];
                    let mut ay = self.0[1];
                    let mut az = self.0[2];
                    let mut aw = self.0[3];
                    out.0[0] = ax * qw + aw * qx + ay * qz - az * qy;
                    out.0[1] = ay * qw + aw * qy + az * qx - ax * qz;
                    out.0[2] = az * qw + aw * qz + ax * qy - ay * qx;
                    out.0[3] = aw * qw - ax * qx - ay * qy - az * qz;

                    ax = self.0[4];
                    ay = self.0[5];
                    az = self.0[6];
                    aw = self.0[7];
                    out.0[4] = ax * qw + aw * qx + ay * qz - az * qy;
                    out.0[5] = ay * qw + aw * qy + az * qx - ax * qz;
                    out.0[6] = az * qw + aw * qz + ax * qy - ay * qx;
                    out.0[7] = aw * qw - ax * qx - ay * qy - az * qz;

                    out
                }

                #[inline]
                pub fn rotate_by_quat_prepend(&self, q: &Quat::<$t>) -> Self {
                    let mut out = Self::new();

                    let qx = q.0[0];
                    let qy = q.0[1];
                    let qz = q.0[2];
                    let qw = q.0[3];
                    let mut bx = self.0[0];
                    let mut by = self.0[1];
                    let mut bz = self.0[2];
                    let mut bw = self.0[3];

                    out.0[0] = qx * bw + qw * bx + qy * bz - qz * by;
                    out.0[1] = qy * bw + qw * by + qz * bx - qx * bz;
                    out.0[2] = qz * bw + qw * bz + qx * by - qy * bx;
                    out.0[3] = qw * bw - qx * bx - qy * by - qz * bz;

                    bx = self.0[4];
                    by = self.0[5];
                    bz = self.0[6];
                    bw = self.0[7];
                    out.0[4] = qx * bw + qw * bx + qy * bz - qz * by;
                    out.0[5] = qy * bw + qw * by + qz * bx - qx * bz;
                    out.0[6] = qz * bw + qw * bz + qx * by - qy * bx;
                    out.0[7] = qw * bw - qx * bx - qy * by - qz * bz;

                    out
                }

                #[inline]
                pub fn rotate_around_axis(&self, axis: &Vec3::<$t>, rad: $t) -> Self {
                    if rad.abs() < $epsilon {
                        return *self;
                    }

                    let mut out = Self::new();

                    let axis_length = (axis.0[0] * axis.0[0] + axis.0[1] * axis.0[1] + axis.0[2] * axis.0[2]).sqrt();
                    let rad = rad * 0.5;
                    let s = rad.sin();
                    let bx = (s * axis.0[0]) / axis_length;
                    let by = (s * axis.0[1]) / axis_length;
                    let bz = (s * axis.0[2]) / axis_length;
                    let bw = rad.cos();

                    let ax1 = self.0[0];
                    let ay1 = self.0[1];
                    let az1 = self.0[2];
                    let aw1 = self.0[3];
                    out.0[0] = ax1 * bw + aw1 * bx + ay1 * bz - az1 * by;
                    out.0[1] = ay1 * bw + aw1 * by + az1 * bx - ax1 * bz;
                    out.0[2] = az1 * bw + aw1 * bz + ax1 * by - ay1 * bx;
                    out.0[3] = aw1 * bw - ax1 * bx - ay1 * by - az1 * bz;

                    let ax = self.0[4];
                    let ay = self.0[5];
                    let az = self.0[6];
                    let aw = self.0[7];
                    out.0[4] = ax * bw + aw * bx + ay * bz - az * by;
                    out.0[5] = ay * bw + aw * by + az * bx - ax * bz;
                    out.0[6] = az * bw + aw * bz + ax * by - ay * bx;
                    out.0[7] = aw * bw - ax * bx - ay * by - az * bz;

                    out
                }

                #[inline]
                pub fn scale(&self, scale: $t) -> Self {
                    self.mul(scale)
                }

                #[inline]
                pub fn dot(&self, b: &Quat2<$t>) -> $t {
                    self.0[0] * b.0[0] + self.0[1] * b.0[1] + self.0[2] * b.0[2] + self.0[3] * b.0[3]
                }

                #[inline]
                pub fn lerp(&self, b: &Quat2<$t>, t: $t) -> Self {
                    let mt = 1.0 - t;
                    let t = if self.dot(b) < 0.0 {
                        -t
                    } else {
                        t
                    };

                    Self::from_values(
                        self.0[0] * mt + b.0[0] * t,
                        self.0[1] * mt + b.0[1] * t,
                        self.0[2] * mt + b.0[2] * t,
                        self.0[3] * mt + b.0[3] * t,
                        self.0[4] * mt + b.0[4] * t,
                        self.0[5] * mt + b.0[5] * t,
                        self.0[6] * mt + b.0[6] * t,
                        self.0[7] * mt + b.0[7] * t,
                    )
                }

                #[inline]
                pub fn squared_length(&self) -> $t {
                    let x = self.0[0];
                    let y = self.0[1];
                    let z = self.0[2];
                    let w = self.0[3];
                    x * x + y * y + z * z + w * w
                }

                #[inline]
                pub fn length(&self) -> $t {
                    self.squared_length().sqrt()
                }

                #[inline]
                pub fn normalize(&self) -> Self {
                    let mut magnitude = self.squared_length();
                    if magnitude > 0.0 {
                        magnitude = magnitude.sqrt();
                    }

                    let a0 = self.0[0] / magnitude;
                    let a1 = self.0[1] / magnitude;
                    let a2 = self.0[2] / magnitude;
                    let a3 = self.0[3] / magnitude;

                    let b0 = self.0[4];
                    let b1 = self.0[5];
                    let b2 = self.0[6];
                    let b3 = self.0[7];

                    let a_dot_b = a0 * b0 + a1 * b1 + a2 * b2 + a3 * b3;

                    Self::from_values(
                        a0,
                        a1,
                        a2,
                        a3,
                        (b0 - a0 * a_dot_b) / magnitude,
                        (b1 - a1 * a_dot_b) / magnitude,
                        (b2 - a2 * a_dot_b) / magnitude,
                        (b3 - a3 * a_dot_b) / magnitude,

                    )
                }

                #[inline]
                pub fn invert(&self) -> Self {
                    let sqlen = self.squared_length();
                    Self::from_values(
                        -self.0[0] / sqlen,
                        -self.0[1] / sqlen,
                        -self.0[2] / sqlen,
                        self.0[3] / sqlen,
                        -self.0[4] / sqlen,
                        -self.0[5] / sqlen,
                        -self.0[6] / sqlen,
                        self.0[7] / sqlen,
                    )
                }

                #[inline]
                pub fn conjugate(&self) -> Self {
                    Self::from_values(
                        -self.0[0],
                        -self.0[1],
                        -self.0[2],
                        self.0[3],
                        -self.0[4],
                        -self.0[5],
                        -self.0[6],
                        self.0[7],
                    )
                }

                #[inline]
                pub fn approximate_eq(&self, b: &Quat2<$t>) -> bool {
                    let a0 = self.0[0];
                    let a1 = self.0[1];
                    let a2 = self.0[2];
                    let a3 = self.0[3];
                    let a4 = self.0[4];
                    let a5 = self.0[5];
                    let a6 = self.0[6];
                    let a7 = self.0[7];
                    let b0 = b.0[0];
                    let b1 = b.0[1];
                    let b2 = b.0[2];
                    let b3 = b.0[3];
                    let b4 = b.0[4];
                    let b5 = b.0[5];
                    let b6 = b.0[6];
                    let b7 = b.0[7];
                    (
                        (a0 - b0).abs() <=
                            $epsilon * (1.0 as $t).max(a0.abs()).max(b0.abs()) &&
                        (a1 - b1).abs() <=
                            $epsilon * (1.0 as $t).max(a1.abs()).max(b1.abs()) &&
                        (a2 - b2).abs() <=
                            $epsilon * (1.0 as $t).max(a2.abs()).max(b2.abs()) &&
                        (a3 - b3).abs() <=
                            $epsilon * (1.0 as $t).max(a3.abs()).max(b3.abs()) &&
                        (a4 - b4).abs() <=
                            $epsilon * (1.0 as $t).max(a4.abs()).max(b4.abs()) &&
                        (a5 - b5).abs() <=
                            $epsilon * (1.0 as $t).max(a5.abs()).max(b5.abs()) &&
                        (a6 - b6).abs() <=
                            $epsilon * (1.0 as $t).max(a6.abs()).max(b6.abs()) &&
                        (a7 - b7).abs() <=
                            $epsilon * (1.0 as $t).max(a7.abs()).max(b7.abs())
                      )
                }
            }

            impl Add<Quat2<$t>> for Quat2<$t> {
                type Output = Quat2<$t>;

                #[inline]
                fn add(self, b: Quat2<$t>) -> Quat2<$t> {
                    Self::from_values(
                        self.0[0] + b.0[0],
                        self.0[1] + b.0[1],
                        self.0[2] + b.0[2],
                        self.0[3] + b.0[3],
                        self.0[4] + b.0[4],
                        self.0[5] + b.0[5],
                        self.0[6] + b.0[6],
                        self.0[7] + b.0[7],
                    )
                }
            }

            impl Mul<Quat2<$t>> for Quat2<$t> {
                type Output = Quat2<$t>;

                #[inline]
                fn mul(self, b: Quat2<$t>) -> Quat2<$t> {
                    let ax0 = self.0[0];
                    let ay0 = self.0[1];
                    let az0 = self.0[2];
                    let aw0 = self.0[3];
                    let bx1 = b.0[4];
                    let by1 = b.0[5];
                    let bz1 = b.0[6];
                    let bw1 = b.0[7];
                    let ax1 = self.0[4];
                    let ay1 = self.0[5];
                    let az1 = self.0[6];
                    let aw1 = self.0[7];
                    let bx0 = b.0[0];
                    let by0 = b.0[1];
                    let bz0 = b.0[2];
                    let bw0 = b.0[3];

                    Self::from_values(
                        ax0 * bw0 + aw0 * bx0 + ay0 * bz0 - az0 * by0,
                        ay0 * bw0 + aw0 * by0 + az0 * bx0 - ax0 * bz0,
                        az0 * bw0 + aw0 * bz0 + ax0 * by0 - ay0 * bx0,
                        aw0 * bw0 - ax0 * bx0 - ay0 * by0 - az0 * bz0,
                        ax0 * bw1 +
                        aw0 * bx1 +
                        ay0 * bz1 -
                        az0 * by1 +
                        ax1 * bw0 +
                        aw1 * bx0 +
                        ay1 * bz0 -
                        az1 * by0,
                        ay0 * bw1 +
                        aw0 * by1 +
                        az0 * bx1 -
                        ax0 * bz1 +
                        ay1 * bw0 +
                        aw1 * by0 +
                        az1 * bx0 -
                        ax1 * bz0,
                        az0 * bw1 +
                        aw0 * bz1 +
                        ax0 * by1 -
                        ay0 * bx1 +
                        az1 * bw0 +
                        aw1 * bz0 +
                        ax1 * by0 -
                        ay1 * bx0,
                        aw0 * bw1 -
                        ax0 * bx1 -
                        ay0 * by1 -
                        az0 * bz1 +
                        aw1 * bw0 -
                        ax1 * bx0 -
                        ay1 * by0 -
                        az1 * bz0
                    )
                }
            }

            impl Mul<$t> for Quat2<$t> {
                type Output = Quat2<$t>;

                #[inline]
                fn mul(self, b: $t) -> Quat2<$t> {
                    Self::from_values(
                        self.0[0] * b,
                        self.0[1] * b,
                        self.0[2] * b,
                        self.0[3] * b,
                        self.0[4] * b,
                        self.0[5] * b,
                        self.0[6] * b,
                        self.0[7] * b,
                    )
                }
            }

            impl Display for Quat2<$t> {
                fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                    let value = self.0.iter().map(|v| v.to_string()).collect::<Vec<_>>().join(", ");
                    f.write_fmt(format_args!("quat2({})", value))
                }
            }
        )+
    };
}

float! {
    (f64, EPSILON_F64),
    (f32, EPSILON_F32)
}
