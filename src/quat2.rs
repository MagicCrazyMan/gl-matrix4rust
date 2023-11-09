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
                pub fn rotate_x(&self, rad: $t) -> Self {
                    let bx = -self.0[0];
                    let by = -self.0[1];
                    let bz = -self.0[2];
                    let bw = self.0[3];
                    let ax = self.0[4];
                    let ay = self.0[5];
                    let az = self.0[6];
                    let aw = self.0[7];
                    let ax1 = ax * bw + aw * bx + ay * bz - az * by;
                    let ay1 = ay * bw + aw * by + az * bx - ax * bz;
                    let az1 = az * bw + aw * bz + ax * by - ay * bx;
                    let aw1 = aw * bw - ax * bx - ay * by - az * bz;
                    let quat = Quat::<$t>::from_values(
                        self.0[0],
                        self.0[1],
                        self.0[2],
                        self.0[3],
                    ).rotate_x(rad);
                    let bx = quat.0[0];
                    let by = quat.0[1];
                    let bz = quat.0[2];
                    let bw = quat.0[3];

                    Self::from_values(
                        bx,
                        by,
                        bz,
                        bw,
                        ax1 * bw + aw1 * bx + ay1 * bz - az1 * by,
                        ay1 * bw + aw1 * by + az1 * bx - ax1 * bz,
                        az1 * bw + aw1 * bz + ax1 * by - ay1 * bx,
                        aw1 * bw - ax1 * bx - ay1 * by - az1 * bz,
                    )
                }

                #[inline]
                pub fn rotate_y(&self, rad: $t) -> Self {
                    let bx = -self.0[0];
                    let by = -self.0[1];
                    let bz = -self.0[2];
                    let bw = self.0[3];
                    let ax = self.0[4];
                    let ay = self.0[5];
                    let az = self.0[6];
                    let aw = self.0[7];
                    let ax1 = ax * bw + aw * bx + ay * bz - az * by;
                    let ay1 = ay * bw + aw * by + az * bx - ax * bz;
                    let az1 = az * bw + aw * bz + ax * by - ay * bx;
                    let aw1 = aw * bw - ax * bx - ay * by - az * bz;
                    let quat = Quat::<$t>::from_values(
                        self.0[0],
                        self.0[1],
                        self.0[2],
                        self.0[3],
                    ).rotate_y(rad);
                    let bx = quat.0[0];
                    let by = quat.0[1];
                    let bz = quat.0[2];
                    let bw = quat.0[3];

                    Self::from_values(
                        bx,
                        by,
                        bz,
                        bw,
                        ax1 * bw + aw1 * bx + ay1 * bz - az1 * by,
                        ay1 * bw + aw1 * by + az1 * bx - ax1 * bz,
                        az1 * bw + aw1 * bz + ax1 * by - ay1 * bx,
                        aw1 * bw - ax1 * bx - ay1 * by - az1 * bz,
                    )
                }

                #[inline]
                pub fn rotate_z(&self, rad: $t) -> Self {
                    let bx = -self.0[0];
                    let by = -self.0[1];
                    let bz = -self.0[2];
                    let bw = self.0[3];
                    let ax = self.0[4];
                    let ay = self.0[5];
                    let az = self.0[6];
                    let aw = self.0[7];
                    let ax1 = ax * bw + aw * bx + ay * bz - az * by;
                    let ay1 = ay * bw + aw * by + az * bx - ax * bz;
                    let az1 = az * bw + aw * bz + ax * by - ay * bx;
                    let aw1 = aw * bw - ax * bx - ay * by - az * bz;
                    let quat = Quat::<$t>::from_values(
                        self.0[0],
                        self.0[1],
                        self.0[2],
                        self.0[3],
                    ).rotate_z(rad);
                    let bx = quat.0[0];
                    let by = quat.0[1];
                    let bz = quat.0[2];
                    let bw = quat.0[3];

                    Self::from_values(
                        bx,
                        by,
                        bz,
                        bw,
                        ax1 * bw + aw1 * bx + ay1 * bz - az1 * by,
                        ay1 * bw + aw1 * by + az1 * bx - ax1 * bz,
                        az1 * bw + aw1 * bz + ax1 * by - ay1 * bx,
                        aw1 * bw - ax1 * bx - ay1 * by - az1 * bz,
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

#[cfg(test)]
#[rustfmt::skip]
mod tests {
    macro_rules! float_test {
        ($t:tt, $epsilon:expr, $pi:expr, $e:expr, $sqrt2:expr) => {
            use std::sync::OnceLock;

            use crate::error::Error;
            use crate::quat::Quat;
            use crate::quat2::Quat2;
            use crate::vec3::Vec3;

            static QUAT2_A_RAW: [$t; 8] = [1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0];
            static QUAT2_B_RAW: [$t; 8] = [5.0, 6.0, 7.0, 8.0, 9.0, 8.0, 6.0, -4.0];

            static QUAT2_A: OnceLock<Quat2<$t>> = OnceLock::new();
            static QUAT2_B: OnceLock<Quat2<$t>> = OnceLock::new();

            fn quat2_a() -> &'static Quat2<$t> {
                QUAT2_A.get_or_init(|| {
                    Quat2::<$t>::from_slice(&QUAT2_A_RAW)
                })
            }

            fn quat2_b() -> &'static Quat2<$t> {
                QUAT2_B.get_or_init(|| {
                    Quat2::<$t>::from_slice(&QUAT2_B_RAW)
                })
            }

            #[test]
            fn new() {
                assert_eq!(
                    Quat2::<$t>::new().raw(),
                    &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                );
            }

            #[test]
            fn from_slice() {
                assert_eq!(
                    Quat2::<$t>::from_slice(&[3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]).raw(),
                    &[3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
                );
            }

            #[test]
            fn from_values() {
                assert_eq!(
                    Quat2::<$t>::from_values(3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0).raw(),
                    &[3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
                );
            }

            #[test]
            fn from_rotation() {
                assert_eq!(
                    Quat2::<$t>::from_rotation(&Quat::<$t>::from_values(1.0, 2.0, 3.0, 4.0)).raw(),
                    &[1.0, 2.0, 3.0, 4.0, 0.0, 0.0, 0.0, 0.0]
                );
            }

            #[test]
            fn from_translation() {
                assert_eq!(
                    Quat2::<$t>::from_translation(&Vec3::<$t>::from_values(1.0, 2.0, 3.0)).raw(),
                    &[0.0, 0.0, 0.0, 1.0, 0.5, 1.0, 1.5, 0.0]
                );
            }

            #[test]
            fn from_rotation_translation() {
                assert_eq!(
                    Quat2::<$t>::from_rotation_translation(
                        &Quat::<$t>::from_values(1.0, 2.0, 3.0, 4.0),
                        &Vec3::<$t>::from_values(1.0, 2.0, 3.0)
                    ).raw(),
                    &[1.0, 2.0, 3.0, 4.0, 2.0, 4.0, 6.0, -7.0]
                );
            }

            #[test]
            fn from_rotation_translation_values() {
                assert_eq!(
                    Quat2::<$t>::from_rotation_translation_values(
                        1.0, 2.0 , 3.0, 4.0,
                        1.0, 2.0 , 3.0
                    ).raw(),
                    &[1.0, 2.0, 3.0, 4.0, 2.0, 4.0, 6.0, -7.0]
                );
            }

            #[test]
            fn scale() {
                assert_eq!(
                    (*quat2_a() * 2.0).raw(),
                    &[2.0, 4.0, 6.0, 8.0, 4.0, 10.0, 12.0, -4.0]
                );
            }

            #[test]
            fn scale_add() {
                assert_eq!(
                    (*quat2_a() + *quat2_b() * 0.5).raw(),
                    &[3.5, 5.0, 6.5, 8.0, 6.5, 9.0, 9.0, -4.0]
                );
            }

            #[test]
            fn rotate_by_quat_append() {
                let quat2_rotation = Quat2::<$t>::from_values(2.0, 5.0, 2.0, -10.0, 0.0, 0.0, 0.0, 0.0);

                assert_eq!(
                    quat2_a().rotate_by_quat_append(&Quat::<$t>::from_values(2.0, 5.0, 2.0, -10.0)).raw(),
                    (*quat2_a() * quat2_rotation).raw()
                );
            }

            #[test]
            fn rotate_by_quat_prepend() {
                let quat2_rotation = Quat2::<$t>::from_values(2.0, 5.0, 2.0, -10.0, 0.0, 0.0, 0.0, 0.0);

                assert_eq!(
                    quat2_a().rotate_by_quat_prepend(&quat2_rotation.real()).raw(),
                    (quat2_rotation * *quat2_a()).raw()
                );
            }

            #[test]
            fn squared_length() {
                assert_eq!(
                    quat2_a().squared_length(),
                    30.0
                );
            }

            #[test]
            fn length() {
                assert_eq!(
                    quat2_a().length(),
                    5.477225575051661
                );
            }

            #[test]
            fn dot() {
                assert_eq!(
                    quat2_a().dot(quat2_b()),
                    70.0
                );
            }
            
            #[test]
            fn conjugate() {
                assert_eq!(
                    quat2_a().conjugate().raw(),
                    &[-1.0, -2.0, -3.0, 4.0, -2.0, -5.0, -6.0, -2.0]
                );
            }
            
            #[test]
            fn real() {
                assert_eq!(
                    quat2_a().real().raw(),
                    &[1.0, 2.0, 3.0, 4.0]
                );
            }

            
            #[test]
            fn dual() {
                assert_eq!(
                    quat2_a().dual().raw(),
                    &[2.0, 5.0, 6.0, -2.0]
                );
            }

            #[test]
            fn set() {
                let mut quat = Quat::<$t>::new();
                quat.set(3.0, 4.0, 5.0, 6.0);

                assert_eq!(
                    quat.raw(),
                    &[3.0, 4.0, 5.0, 6.0]
                );
            }

            #[test]
            fn set_slice() {
                let mut quat = Quat::<$t>::new();
                quat.set_slice(&[3.0, 4.0, 5.0, 6.0]);

                assert_eq!(
                    quat.raw(),
                    &[3.0, 4.0, 5.0, 6.0]
                );
            }

            #[test]
            fn set_real() {
                assert_eq!(
                    quat2_a().clone().set_real(&Quat::<$t>::from_values(4.0, 6.0, 8.0, -100.0)).raw(),
                    &[4.0, 6.0, 8.0, -100.0, 2.0, 5.0, 6.0, -2.0]
                );
            }

            #[test]
            fn set_dual() {
                let mut quat = Quat::<$t>::new();
                quat.set_slice(&[3.0, 4.0, 5.0, 6.0]);

                assert_eq!(
                    quat2_a().clone().set_dual(&Quat::<$t>::from_values(4.3, 6.0, 8.0, -100.0)).raw(),
                    &[1.0, 2.0, 3.0, 4.0, 4.3, 6.0, 8.0, -100.0]
                );
            }

            #[test]
            fn add() {
                assert_eq!(
                    (*quat2_a() + *quat2_b()).raw(),
                    &[6.0, 8.0, 10.0, 12.0, 11.0, 13.0, 12.0, -6.0]
                );
            }

            #[test]
            fn mul() {
                assert_eq!(
                    (*quat2_a() * *quat2_b()).raw(),
                    &[24.0, 48.0, 48.0, -6.0,  25.0, 89.0, 23.0, -157.0]
                );
            }

            #[test]
            fn mul_scalar() {
                assert_eq!(
                    (*quat2_a() * 2.0).raw(),
                    &[2.0, 4.0, 6.0, 8.0, 4.0, 10.0, 12.0, -4.0]
                );
            }

            #[test]
            fn mul_scalar_add() {
                assert_eq!(
                    (*quat2_a() + *quat2_b() * 0.5).raw(),
                    &[3.5, 5.0, 6.5, 8.0, 6.5, 9.0, 9.0, -4.0]
                );
            }

            #[test]
            fn approximate_eq() {
                let quat2_a = Quat2::<$t>::from_values(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0);
                let quat2_b = Quat2::<$t>::from_values(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0);
                let quat2_c = Quat2::<$t>::from_values(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0);
                let quat2_d = Quat2::<$t>::from_values(1e-16, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0);

                assert_eq!(
                    true,
                    quat2_a.approximate_eq(&quat2_b)
                );
                assert_eq!(
                    false,
                    quat2_a.approximate_eq(&quat2_c)
                );
                assert_eq!(
                    true,
                    quat2_a.approximate_eq(&quat2_d)
                );
            }

            #[test]
            fn display() {
                let out = quat2_a().to_string();
                assert_eq!(
                    out,
                    "quat2(1, 2, 3, 4, 2, 5, 6, -2)"
                );
            }
        };
    }

    mod f32 {
        float_test!(
            f32,
            crate::EPSILON_F32,
            std::f32::consts::PI,
            std::f32::consts::E,
            std::f32::consts::SQRT_2
        );

        #[test]
        fn from_mat4() {
            let quat_rotation = Quat::<f32>::from_values(1.0, 2.0, 3.0, 4.0).normalize();
            let quat2_a = Quat2::<f32>::from_rotation_translation(
                &quat_rotation,
                &Vec3::<f32>::from_values(1.0, -5.0, 3.0)
            ).normalize();

            assert_eq!(
                quat2_a.raw(),
                &[0.18257418, 0.36514837, 0.5477226, 0.73029673, -1.5518806, -1.8257418, 1.7344548, -1.1487366e-7],
            );
        }

        #[test]
        fn invert() {
            assert_eq!(
                quat2_a().invert().raw(),
                &[-0.0333333333, -0.06666666666, -0.1, 0.13333333333, -2.0/30.0, -5.0/30.0, -6.0/30.0, -2.0/30.0]
            );
            assert_eq!(
                quat2_a().invert().real().raw(),
                &[-0.033333335, -0.06666667, -0.1, 0.13333334],
            );
        }

        #[test]
        fn lerp() {
            assert_eq!(
                quat2_a().lerp(quat2_b(), 0.7).raw(),
                &[3.8, 4.7999997, 5.8, 6.8, 6.8999996, 7.1, 6.0, -3.4]
            );
        }

        #[test]
        fn normalize() {
            assert_eq!(
                Quat2::<f32>::from_values(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0).normalize().raw(),
                &[0.18257418, 0.36514837, 0.5477225, 0.73029673, 0.23126063, 0.6450954, 0.6937819, -0.9006993]
            );
            assert_eq!(
                Quat2::<f32>::from_values(5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).normalize().raw(),
                &[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            );
            assert_eq!(
                Quat2::<f32>::from_values(5.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 5.0).normalize().raw(),
                &[1.0, 0.0, 0.0, 0.0, 0.0, 0.4, 0.6, 1.0]
            );
        }

        #[test]
        fn rotate_around_axis() -> Result<(), Error> {
            let quat2_a = Quat2::<f32>::from_rotation_translation(
                &Quat::<f32>::from_values(1.0, 2.0, 3.0, 4.0),
                &Vec3::<f32>::from_values(-5.0, 4.0, 10.0),
            ).normalize();
            
            let result = quat2_a.rotate_around_axis(&Vec3::<f32>::from_values(1.0, 4.0, 2.0), 5.0);

            assert_eq!(
                result.raw(),
                &[-0.2416429, 0.11280663, -0.20036738, -0.942728, 1.3920522, -3.5945892, -4.512371, 0.17211652]
            );

            Ok(())
        }

        #[test]
        fn rotate_x() {
            let quat2_a = Quat2::<f32>::from_rotation_translation(
                &Quat::<f32>::from_values(1.0, 2.0, 3.0, 4.0),
                &Vec3::<f32>::from_values(-5.0, 4.0, 10.0),
            ).normalize();
            
            let result = quat2_a.rotate_x(5.0);

            assert_eq!(
                result.raw(),
                &[0.2907941, 0.03526041, -0.6573355, -0.6943381, 0.24487177, -1.5780439, -4.1414294, 3.943142]
            );
        }

        #[test]
        fn rotate_y() {
            let quat2_a = Quat2::<f32>::from_rotation_translation(
                &Quat::<f32>::from_values(1.0, 2.0, 3.0, 4.0),
                &Vec3::<f32>::from_values(-5.0, 4.0, 10.0),
            ).normalize();
            
            let result = quat2_a.rotate_y(5.0);

            assert_eq!(
                result.raw(),
                &[-0.4740648, 0.14452597, -0.32953882, -0.80360365, 0.6273011, -4.801378, -3.4312036, 0.17348075]
            );
        }

        #[test]
        fn rotate_z() {
            let quat2_a = Quat2::<f32>::from_rotation_translation(
                &Quat::<f32>::from_values(1.0, 2.0, 3.0, 4.0),
                &Vec3::<f32>::from_values(-5.0, 4.0, 10.0),
            ).normalize();
            
            let result = quat2_a.rotate_y(5.0);

            assert_eq!(
                result.raw(),
                &[-0.4740648, 0.14452597, -0.32953882, -0.80360365, 0.6273011, -4.801378, -3.4312036, 0.17348075]
            );
        }

        #[test]
        fn translation() {
            assert_eq!(
                Quat2::<f32>::from_translation(&Vec3::<f32>::from_values(1.0, 2.0, 3.0)).translation().raw(),
                &[1.0, 2.0, 3.0]
            );
            assert_eq!(
                Quat2::<f32>::from_translation(&Vec3::<f32>::from_values(1.0, 2.0, 3.0)).translation().normalize().raw(),
                &[0.26726124, 0.5345225, 0.8017837]
            );
            assert_ne!(
                Quat2::<f32>::from_rotation_translation(
                    &Quat::<f32>::from_values(2.0, 4.0, 6.0, 2.0),
                    &Vec3::<f32>::from_values(1.0, 2.0, 3.0)
                ).translation().raw(),
                &[1.0, 2.0, 3.0]
            );
            assert_eq!(
                Quat2::<f32>::from_rotation_translation(
                    &Quat::<f32>::from_values(2.0, 4.0, 6.0, 2.0),
                    &Vec3::<f32>::from_values(1.0, 2.0, 3.0)
                ).normalize().translation().raw(),
                &[0.9999999, 1.9999999, 2.9999998]
            );
        }

        #[test]
        fn translate() {
            let quat2_a = Quat2::<f32>::from_rotation_translation(
                &Quat::<f32>::from_values(1.0, 2.0, 3.0, 4.0),
                &Vec3::<f32>::from_values(-5.0, 4.0, 10.0),
            ).normalize();
            
            let result = quat2_a.translate(&Vec3::<f32>::from_values(1.0, 1.0, -1.0));

            assert_eq!(
                result.raw(),
                &[0.18257418, 0.36514837, 0.5477225, 0.73029673, -2.6473258, 4.4730673, 1.9170291, -3.012474]
            );
        }
    }

    mod f64 {
        float_test!(
            f64,
            crate::EPSILON_F64,
            std::f64::consts::PI,
            std::f64::consts::E,
            std::f64::consts::SQRT_2
        );

        #[test]
        fn from_mat4() {
            let quat_rotation = Quat::<f64>::from_values(1.0, 2.0, 3.0, 4.0).normalize();
            let quat2_a = Quat2::<f64>::from_rotation_translation(
                &quat_rotation,
                &Vec3::<f64>::from_values(1.0, -5.0, 3.0)
            ).normalize();

            assert_eq!(
                quat2_a.raw(),
                &[0.18257418583505539, 0.36514837167011077, 0.5477225575051662, 0.7302967433402215, -1.5518805795979707, -1.8257418583505538, 1.734454765433026, 0.0],
            );
        }

        #[test]
        fn invert() {
            assert_eq!(
                quat2_a().invert().raw(),
                &[-0.03333333333333333, -0.06666666666666667, -0.1, 0.13333333333333333, -0.06666666666666667, -0.16666666666666666, -0.2, -0.06666666666666667]
            );
            assert_eq!(
                quat2_a().invert().real().raw(),
                Quat::<f64>::from_values(1.0, 2.0, 3.0, 4.0).invert().raw(),
            );
        }

        #[test]
        fn lerp() {
            assert_eq!(
                quat2_a().lerp(quat2_b(), 0.7).raw(),
                &[3.8, 4.799999999999999, 5.8, 6.8, 6.9, 7.1, 6.0, -3.4]
            );
        }

        #[test]
        fn normalize() {
            assert_eq!(
                Quat2::<f64>::from_values(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0).normalize().raw(),
                &[0.18257418583505536, 0.3651483716701107, 0.5477225575051661, 0.7302967433402214, 0.23126063539107017, 0.6450954566171957, 0.6937819061732106, -0.900699316786273]
            );
            assert_eq!(
                Quat2::<f64>::from_values(5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).normalize().raw(),
                &[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            );
            assert_eq!(
                Quat2::<f64>::from_values(5.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 5.0).normalize().raw(),
                &[1.0, 0.0, 0.0, 0.0, 0.0, 0.4, 0.6, 1.0]
            );
        }

        #[test]
        fn rotate_around_axis() -> Result<(), Error> {
            let quat2_a = Quat2::<f64>::from_rotation_translation(
                &Quat::<f64>::from_values(1.0, 2.0, 3.0, 4.0),
                &Vec3::<f64>::from_values(-5.0, 4.0, 10.0),
            ).normalize();
            
            let result = quat2_a.rotate_around_axis(&Vec3::<f64>::from_values(1.0, 4.0, 2.0), 5.0);

            assert_eq!(
                result.raw(),
                &[-0.24164294714846665, 0.11280662947202075, -0.20036742052872042, -0.9427280876431084, 1.3920522306902268, -3.594589462350351, -4.512371117598661, 0.17211647582839384]
            );

            Ok(())
        }

        #[test]
        fn rotate_x() {
            let quat2_a = Quat2::<f64>::from_rotation_translation(
                &Quat::<f64>::from_values(1.0, 2.0, 3.0, 4.0),
                &Vec3::<f64>::from_values(-5.0, 4.0, 10.0),
            ).normalize();
            
            let result = quat2_a.rotate_x(5.0);

            assert_eq!(
                result.raw(),
                &[0.2907941144735252, 0.0352604066733514, -0.6573355589457817, -0.6943381378364758, 0.2448721933328691, -1.5780446006697788, -4.141429934812809, 3.943142267566019]
            );
        }

        #[test]
        fn rotate_y() {
            let quat2_a = Quat2::<f64>::from_rotation_translation(
                &Quat::<f64>::from_values(1.0, 2.0, 3.0, 4.0),
                &Vec3::<f64>::from_values(-5.0, 4.0, 10.0),
            ).normalize();
            
            let result = quat2_a.rotate_y(5.0);

            assert_eq!(
                result.raw(),
                &[-0.4740648367096534, 0.14452597112809118, -0.32953886558156226, -0.8036037022912156, 0.6273016689244582, -4.801378752084603, -3.431203765857, 0.17348029387749575]
            );
        }

        #[test]
        fn rotate_z() {
            let quat2_a = Quat2::<f64>::from_rotation_translation(
                &Quat::<f64>::from_values(1.0, 2.0, 3.0, 4.0),
                &Vec3::<f64>::from_values(-5.0, 4.0, 10.0),
            ).normalize();
            
            let result = quat2_a.rotate_y(5.0);

            assert_eq!(
                result.raw(),
                &[-0.4740648367096534, 0.14452597112809118, -0.32953886558156226, -0.8036037022912156, 0.6273016689244582, -4.801378752084603, -3.431203765857, 0.17348029387749575]
            );
        }

        #[test]
        fn translation() {
            assert_eq!(
                Quat2::<f32>::from_translation(&Vec3::<f32>::from_values(1.0, 2.0, 3.0)).translation().raw(),
                &[1.0, 2.0, 3.0]
            );
            assert_eq!(
                Quat2::<f32>::from_translation(&Vec3::<f32>::from_values(1.0, 2.0, 3.0)).translation().normalize().raw(),
                &[0.26726124, 0.5345225, 0.8017837]
            );
            assert_ne!(
                Quat2::<f32>::from_rotation_translation(
                    &Quat::<f32>::from_values(2.0, 4.0, 6.0, 2.0),
                    &Vec3::<f32>::from_values(1.0, 2.0, 3.0)
                ).translation().raw(),
                &[1.0, 2.0, 3.0]
            );
            assert_eq!(
                Quat2::<f32>::from_rotation_translation(
                    &Quat::<f32>::from_values(2.0, 4.0, 6.0, 2.0),
                    &Vec3::<f32>::from_values(1.0, 2.0, 3.0)
                ).normalize().translation().raw(),
                &[0.9999999, 1.9999999, 2.9999998]
            );
        }

        #[test]
        fn translate() {
            let quat2_a = Quat2::<f64>::from_rotation_translation(
                &Quat::<f64>::from_values(1.0, 2.0, 3.0, 4.0),
                &Vec3::<f64>::from_values(-5.0, 4.0, 10.0),
            ).normalize();
            
            let result = quat2_a.translate(&Vec3::<f64>::from_values(1.0, 1.0, -1.0));

            assert_eq!(
                result.raw(),
                &[0.18257418583505536, 0.3651483716701107, 0.5477225575051661, 0.7302967433402214, -2.647325694608303, 4.473067552958856, 1.9170289512680818, -3.0124740662784135]
            );
        }
    }
}
