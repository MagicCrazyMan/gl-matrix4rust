use std::{
    fmt::Display,
    ops::{Add, Div, Mul, Sub},
};

use crate::{mat4::Mat4, quat::Quat, EPSILON_F32, EPSILON_F64};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Vec4<T = f64>(pub [T; 4]);

macro_rules! float {
    ($(($t:tt, $epsilon:expr)),+) => {
        $(
            impl Vec4<$t> {
                #[inline]
                pub fn new() -> Vec4<$t> {
                    Self([0.0; 4])
                }

                #[inline]
                pub fn from_values(x: $t, y: $t, z: $t, w: $t) -> Vec4<$t> {
                    Self([x, y, z, w])
                }

                #[inline]
                pub fn from_slice([x, y, z, w]: &[$t; 4]) -> Self {
                    Self([*x, *y, *z, *w])
                }
            }

            impl Vec4<$t> {
                #[inline]
                pub fn raw(&self) -> &[$t; 4] {
                    &self.0
                }

                #[inline]
                pub fn set(&mut self, x: $t, y: $t, z: $t, w: $t) -> &mut Self {
                    self.0[0] = x;
                    self.0[1] = y;
                    self.0[2] = z;
                    self.0[3] = w;
                    self
                }

                #[inline]
                pub fn set_slice(&mut self, [x, y, z, w]: &[$t; 4]) -> &mut Self {
                    self.0[0] = *x;
                    self.0[1] = *y;
                    self.0[2] = *z;
                    self.0[3] = *w;
                    self
                }

                #[inline]
                pub fn set_zero(&mut self) -> &mut Self {
                    self.0[0] = 0.0;
                    self.0[1] = 0.0;
                    self.0[2] = 0.0;
                    self.0[3] = 0.0;
                    self
                }
                #[inline]
                pub fn ceil(&self) -> Self {
                    let mut out = Self::new();
                    out.0[0] = self.0[0].ceil();
                    out.0[1] = self.0[1].ceil();
                    out.0[2] = self.0[2].ceil();
                    out.0[3] = self.0[3].ceil();
                    out
                }

                #[inline]
                pub fn floor(&self) -> Self {
                    let mut out = Self::new();
                    out.0[0] = self.0[0].floor();
                    out.0[1] = self.0[1].floor();
                    out.0[2] = self.0[2].floor();
                    out.0[3] = self.0[3].floor();
                    out
                }

                #[inline]
                pub fn min(&self, b: &Vec4<$t>) -> Self {
                    let mut out = Self::new();
                    out.0[0] = self.0[0].min(b.0[0]);
                    out.0[1] = self.0[1].min(b.0[1]);
                    out.0[2] = self.0[2].min(b.0[2]);
                    out.0[3] = self.0[3].min(b.0[3]);
                    out
                }

                #[inline]
                pub fn max(&self, b: &Vec4<$t>) -> Self {
                    let mut out = Self::new();
                    out.0[0] = self.0[0].max(b.0[0]);
                    out.0[1] = self.0[1].max(b.0[1]);
                    out.0[2] = self.0[2].max(b.0[2]);
                    out.0[3] = self.0[3].max(b.0[3]);
                    out
                }

                #[inline]
                pub fn round(&self) -> Self {
                    let mut out = Self::new();
                    out.0[0] = self.0[0].round();
                    out.0[1] = self.0[1].round();
                    out.0[2] = self.0[2].round();
                    out.0[3] = self.0[3].round();
                    out
                }

                #[inline]
                pub fn scale(&self, scale: $t) -> Self {
                    self.mul(scale)
                }

                #[inline]
                pub fn squared_distance(&self, b: &Vec4<$t>) -> $t {
                    let x = b.0[0] - self.0[0];
                    let y = b.0[1] - self.0[1];
                    let z = b.0[2] - self.0[2];
                    let w = b.0[3] - self.0[3];
                    x * x + y * y + z * z + w * w
                }

                #[inline]
                pub fn distance(&self, b: &Vec4<$t>) -> $t {
                    self.squared_distance(b).sqrt()
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
                pub fn negate(&self) -> Self {
                    let mut out = Self::new();
                    out.0[0] = -self.0[0];
                    out.0[1] = -self.0[1];
                    out.0[2] = -self.0[2];
                    out.0[3] = -self.0[3];
                    out
                }

                #[inline]
                pub fn inverse(&self) -> Self {
                    let mut out = Self::new();
                    out.0[0] = 1.0 / self.0[0];
                    out.0[1] = 1.0 / self.0[1];
                    out.0[2] = 1.0 / self.0[2];
                    out.0[3] = 1.0 / self.0[3];
                    out
                }

                #[inline]
                pub fn normalize(&self) -> Self {
                    let mut len = self.squared_length();
                    if len > 0.0 {
                        len = 1.0 / len.sqrt();
                    }

                    Self::from_values(
                        self.0[0] * len,
                        self.0[1] * len,
                        self.0[2] * len,
                        self.0[3] * len
                    )
                }

                #[inline]
                pub fn dot(&self, b: &Vec4<$t>) -> $t {
                    self.0[0] * b.0[0] + self.0[1] * b.0[1] + self.0[2] * b.0[2] + self.0[3] * b.0[3]
                }

                #[inline]
                pub fn cross(&self, u: &Vec4<$t>, v: &Vec4<$t>, w: &Vec4<$t>) -> Self {
                    let a = v.0[0] * w.0[1] - v.0[1] * w.0[0];
                    let b = v.0[0] * w.0[2] - v.0[2] * w.0[0];
                    let c = v.0[0] * w.0[3] - v.0[3] * w.0[0];
                    let d = v.0[1] * w.0[2] - v.0[2] * w.0[1];
                    let e = v.0[1] * w.0[3] - v.0[3] * w.0[1];
                    let f = v.0[2] * w.0[3] - v.0[3] * w.0[2];
                    let g = u.0[0];
                    let h = u.0[1];
                    let i = u.0[2];
                    let j = u.0[3];

                    Self::from_values(
                        h * f - i * e + j * d,
                        -(g * f) + i * c - j * b,
                        g * e - h * c + j * a,
                        -(g * d) + h * b - i * a,
                    )
                }

                #[inline]
                pub fn lerp(&self, b: &Vec4<$t>, t: $t) -> Self {
                    let ax = self.0[0];
                    let ay = self.0[1];
                    let az = self.0[2];
                    let aw = self.0[3];

                    Self::from_values(
                        ax + t * (b.0[0] - ax),
                        ay + t * (b.0[1] - ay),
                        az + t * (b.0[2] - az),
                        aw + t * (b.0[3] - aw),
                    )
                }

                #[inline]
                pub fn transform_mat4(&self, m: &Mat4<$t>) -> Self {
                    let x = self.0[0];
                    let y = self.0[1];
                    let z = self.0[2];
                    let w = self.0[3];

                    Self::from_values(
                        m.0[0] * x + m.0[4] * y + m.0[8] * z + m.0[12] * w,
                        m.0[1] * x + m.0[5] * y + m.0[9] * z + m.0[13] * w,
                        m.0[2] * x + m.0[6] * y + m.0[10] * z + m.0[14] * w,
                        m.0[3] * x + m.0[7] * y + m.0[11] * z + m.0[15] * w,
                    )
                }

                #[inline]
                pub fn transform_quat(&self, q: &Quat<$t>) -> Self {
                    let x = self.0[0];
                    let y = self.0[1];
                    let z = self.0[2];
                    let qx = q.0[0];
                    let qy = q.0[1];
                    let qz = q.0[2];
                    let qw = q.0[3];
                  
                    // calculate quat * vec
                    let ix = qw * x + qy * z - qz * y;
                    let iy = qw * y + qz * x - qx * z;
                    let iz = qw * z + qx * y - qy * x;
                    let iw = -qx * x - qy * y - qz * z;

                    Self::from_values(
                        ix * qw + iw * -qx + iy * -qz - iz * -qy,
                        iy * qw + iw * -qy + iz * -qx - ix * -qz,
                        iz * qw + iw * -qz + ix * -qy - iy * -qx,
                        self.0[3],
                    )
                }

                #[inline]
                pub fn approximate_eq(&self, b: &Vec4<$t>) -> bool {
                    let a0 = self.0[0];
                    let a1 = self.0[1];
                    let a2 = self.0[2];
                    let a3 = self.0[3];
                    let b0 = b.0[0];
                    let b1 = b.0[1];
                    let b2 = b.0[2];
                    let b3 = b.0[3];
                    (
                      (a0 - b0).abs() <=
                        $epsilon * (1.0 as $t).max(a0.abs()).max(b0.abs()) &&
                      (a1 - b1).abs() <=
                        $epsilon * (1.0 as $t).max(a1.abs()).max(b1.abs()) &&
                      (a2 - b2).abs() <=
                        $epsilon * (1.0 as $t).max(a2.abs()).max(b2.abs()) &&
                      (a3 - b3).abs() <=
                        $epsilon * (1.0 as $t).max(a3.abs()).max(b3.abs())
                    )
                }
            }

            impl Add<Vec4<$t>> for Vec4<$t> {
                type Output = Vec4<$t>;

                #[inline]
                fn add(self, b: Vec4<$t>) -> Vec4<$t> {
                    let mut out = Vec4::<$t>::new();
                    out.0[0] = self.0[0] + b.0[0];
                    out.0[1] = self.0[1] + b.0[1];
                    out.0[2] = self.0[2] + b.0[2];
                    out.0[3] = self.0[3] + b.0[3];
                    out
                }
            }

            impl Sub<Vec4<$t>> for Vec4<$t> {
                type Output = Vec4<$t>;

                #[inline]
                fn sub(self, b: Vec4<$t>) -> Vec4<$t> {
                    let mut out = Vec4::<$t>::new();
                    out.0[0] = self.0[0] - b.0[0];
                    out.0[1] = self.0[1] - b.0[1];
                    out.0[2] = self.0[2] - b.0[2];
                    out.0[3] = self.0[3] - b.0[3];
                    out
                }
            }

            impl Mul<Vec4<$t>> for Vec4<$t> {
                type Output = Vec4<$t>;

                #[inline]
                fn mul(self, b: Vec4<$t>) -> Vec4<$t> {
                    let mut out = Vec4::<$t>::new();
                    out.0[0] = self.0[0] * b.0[0];
                    out.0[1] = self.0[1] * b.0[1];
                    out.0[2] = self.0[2] * b.0[2];
                    out.0[3] = self.0[3] * b.0[3];
                    out
                }
            }

            impl Mul<$t> for Vec4<$t> {
                type Output = Vec4<$t>;

                #[inline]
                fn mul(self, b: $t) -> Vec4<$t> {
                    let mut out = Vec4::<$t>::new();
                    out.0[0] = self.0[0] * b;
                    out.0[1] = self.0[1] * b;
                    out.0[2] = self.0[2] * b;
                    out.0[3] = self.0[3] * b;
                    out
                }
            }

            impl Div<Vec4<$t>> for Vec4<$t> {
                type Output = Vec4<$t>;

                #[inline]
                fn div(self, b: Vec4<$t>) -> Vec4<$t> {
                    let mut out = Vec4::<$t>::new();
                    out.0[0] = self.0[0] / b.0[0];
                    out.0[1] = self.0[1] / b.0[1];
                    out.0[2] = self.0[2] / b.0[2];
                    out.0[3] = self.0[3] / b.0[3];
                    out
                }
            }

            impl Div<$t> for Vec4<$t> {
                type Output = Vec4<$t>;

                #[inline]
                fn div(self, b: $t) -> Vec4<$t> {
                    let mut out = Vec4::<$t>::new();
                    out.0[0] = self.0[0] / b;
                    out.0[1] = self.0[1] / b;
                    out.0[2] = self.0[2] / b;
                    out.0[3] = self.0[3] / b;
                    out
                }
            }

            impl Display for Vec4<$t> {
                fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                    let value = self.0.iter().map(|v| v.to_string()).collect::<Vec<_>>().join(", ");
                    f.write_fmt(format_args!("vec4({})", value))
                }
            }
        )+
    };
}

float! {
    (f64, EPSILON_F64),
    (f32, EPSILON_F32)
}
