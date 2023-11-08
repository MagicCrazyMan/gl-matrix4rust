use std::{
    fmt::Display,
    ops::{Add, Div, Mul, Sub},
};

use crate::{mat3::Mat3, mat4::Mat4, quat::Quat, EPSILON_F32, EPSILON_F64};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Vec3<T = f32>(pub [T; 3]);

macro_rules! float {
    ($(($t:tt, $epsilon:expr)),+) => {
        $(
            impl Vec3<$t> {
                #[inline]
                pub fn new() -> Self {
                    Self([0.0; 3])
                }

                #[inline]
                pub fn from_values(x: $t, y: $t, z: $t) -> Self {
                    Self([x, y, z])
                }

                #[inline]
                pub fn from_slice([x, y, z]: &[$t; 3]) -> Self {
                    Self([*x, *y, *z])
                }
            }

            impl Vec3<$t> {
                #[inline]
                pub fn raw(&self) -> &[$t; 3] {
                    &self.0
                }

                #[inline]
                pub fn set(&mut self, x: $t, y: $t, z: $t) -> &mut Self {
                    self.0[0] = x;
                    self.0[1] = y;
                    self.0[2] = z;
                    self
                }

                #[inline]
                pub fn set_slice(&mut self, [x, y, z]: &[$t; 3]) -> &mut Self {
                    self.0[0] = *x;
                    self.0[1] = *y;
                    self.0[2] = *z;
                    self
                }

                #[inline]
                pub fn set_zero(&mut self) -> &mut Self {
                    self.0[0] = 0.0;
                    self.0[1] = 0.0;
                    self.0[2] = 0.0;
                    self
                }

                #[inline]
                pub fn ceil(&self) -> Self {
                    let mut out = Self::new();
                    out.0[0] = self.0[0].ceil();
                    out.0[1] = self.0[1].ceil();
                    out.0[2] = self.0[2].ceil();
                    out
                }

                #[inline]
                pub fn floor(&self) -> Self {
                    let mut out = Self::new();
                    out.0[0] = self.0[0].floor();
                    out.0[1] = self.0[1].floor();
                    out.0[2] = self.0[2].floor();
                    out
                }

                #[inline]
                pub fn min(&self, b: &Vec3<$t>) -> Self {
                    let mut out = Self::new();
                    out.0[0] = self.0[0].min(b.0[0]);
                    out.0[1] = self.0[1].min(b.0[1]);
                    out.0[2] = self.0[2].min(b.0[2]);
                    out
                }

                #[inline]
                pub fn max(&self, b: &Vec3<$t>) -> Self {
                    let mut out = Self::new();
                    out.0[0] = self.0[0].max(b.0[0]);
                    out.0[1] = self.0[1].max(b.0[1]);
                    out.0[2] = self.0[2].max(b.0[2]);
                    out
                }

                #[inline]
                pub fn round(&self) -> Self {
                    let mut out = Self::new();
                    out.0[0] = self.0[0].round();
                    out.0[1] = self.0[1].round();
                    out.0[2] = self.0[2].round();
                    out
                }

                #[inline]
                pub fn scale(&self, scale: $t) -> Self {
                    self.mul(scale)
                }

                #[inline]
                pub fn squared_distance(&self, b: &Vec3<$t>) -> $t {
                    let x = b.0[0] - self.0[0];
                    let y = b.0[1] - self.0[1];
                    let z = b.0[2] - self.0[2];
                    x * x + y * y + z * z
                }

                #[inline]
                pub fn distance(&self, b: &Vec3<$t>) -> $t {
                    self.squared_distance(b).sqrt()
                }

                #[inline]
                pub fn squared_length(&self) -> $t {
                    let x = self.0[0];
                    let y = self.0[1];
                    let z = self.0[2];
                    x * x + y * y + z * z
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
                    out
                }

                #[inline]
                pub fn inverse(&self) -> Self {
                    let mut out = Self::new();
                    out.0[0] = 1.0 / self.0[0];
                    out.0[1] = 1.0 / self.0[1];
                    out.0[2] = 1.0 / self.0[2];
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
                        self.0[2] * len
                    )
                }

                #[inline]
                pub fn dot(&self, b: &Vec3<$t>) -> $t {
                    self.0[0] * b.0[0] + self.0[1] * b.0[1] + self.0[2] * b.0[2]
                }

                #[inline]
                pub fn cross(&self, b: &Vec3<$t>) -> Self {
                    let ax = self.0[0];
                    let ay = self.0[1];
                    let az = self.0[2];
                    let bx = b.0[0];
                    let by = b.0[1];
                    let bz = b.0[2];

                    Self::from_values(
                        ay * bz - az * by,
                        az * bx - ax * bz,
                        ax * by - ay * bx
                    )
                }

                #[inline]
                pub fn lerp(&self, b: &Vec3<$t>, t: $t) -> Self {
                    let mut out = Self::new();
                    let ax = self.0[0];
                    let ay = self.0[1];
                    let az = self.0[2];
                    out.0[0] = ax + t * (b.0[0] - ax);
                    out.0[1] = ay + t * (b.0[1] - ay);
                    out.0[2] = az + t * (b.0[2] - az);
                    out
                }

                #[inline]
                pub fn slerp(&self, b: &Vec3<$t>, t: $t) -> Self {
                    let angle = self.dot(b).max(-1.0).min(1.0).acos();
                    let sin_total = angle.sin();

                    let ratio_a = ((1.0 - t) * angle).sin() / sin_total;
                    let ratio_b = (t * angle).sin() / sin_total;
                    Self::from_values(
                        ratio_a * self.0[0] + ratio_b * self.0[0],
                        ratio_a * self.0[1] + ratio_b * self.0[1],
                        ratio_a * self.0[2] + ratio_b * self.0[2],
                    )
                }

                #[inline]
                pub fn hermite(&self, b: &Vec3<$t>, c: &Vec3<$t>, d: &Vec3<$t>, t: $t) -> Self {
                    let factor_times2 = t * t;
                    let factor1 = factor_times2 * (2.0 * t - 3.0) + 1.0;
                    let factor2 = factor_times2 * (t - 2.0) + t;
                    let factor3 = factor_times2 * (t - 1.0);
                    let factor4 = factor_times2 * (3.0 - 2.0 * t);

                    Self::from_values(
                        self.0[0] * factor1 + b.0[0] * factor2 + c.0[0] * factor3 + d.0[0] * factor4,
                        self.0[1] * factor1 + b.0[1] * factor2 + c.0[1] * factor3 + d.0[1] * factor4,
                        self.0[2] * factor1 + b.0[2] * factor2 + c.0[2] * factor3 + d.0[2] * factor4,
                    )
                }

                #[inline]
                pub fn bezier(&self, b: &Vec3<$t>, c: &Vec3<$t>, d: &Vec3<$t>, t: $t) -> Self {
                    let inverse_factor = 1.0 - t;
                    let inverse_factor_times_two = inverse_factor * inverse_factor;
                    let factor_times2 = t * t;
                    let factor1 = inverse_factor_times_two * inverse_factor;
                    let factor2 = 3.0 * t * inverse_factor_times_two;
                    let factor3 = 3.0 * factor_times2 * inverse_factor;
                    let factor4 = factor_times2 * t;

                    Self::from_values(
                        self.0[0] * factor1 + b.0[0] * factor2 + c.0[0] * factor3 + d.0[0] * factor4,
                        self.0[1] * factor1 + b.0[1] * factor2 + c.0[1] * factor3 + d.0[1] * factor4,
                        self.0[2] * factor1 + b.0[2] * factor2 + c.0[2] * factor3 + d.0[2] * factor4,
                    )
                }

                #[inline]
                pub fn transform_mat3(&self, m: &Mat3<$t>) -> Self {
                    let x = self.0[0];
                    let y = self.0[1];
                    let z = self.0[2];

                    Self::from_values(
                        x * m.0[0] + y * m.0[3] + z * m.0[6],
                        x * m.0[1] + y * m.0[4] + z * m.0[7],
                        x * m.0[2] + y * m.0[5] + z * m.0[8],
                    )
                }

                #[inline]
                pub fn transform_quat(&self, q: &Quat<$t>) -> Self {
                    // benchmarks: https://jsperf.com/quaternion-transform-vec3-implementations-fixed
                    let qx = q.0[0];
                    let qy = q.0[1];
                    let qz = q.0[2];
                    let qw = q.0[3];
                    let x = self.0[0];
                    let y = self.0[1];
                    let z = self.0[2];
                    // var qvec = [qx, qy, qz];
                    // var uv = vec3.cross([], qvec, a);
                    let mut uvx = qy * z - qz * y;
                    let mut uvy = qz * x - qx * z;
                    let mut uvz = qx * y - qy * x;
                    // var uuv = vec3.cross([], qvec, uv);
                    let mut uuvx = qy * uvz - qz * uvy;
                    let mut uuvy = qz * uvx - qx * uvz;
                    let mut uuvz = qx * uvy - qy * uvx;
                    // vec3.scale(uv, uv, 2 * w);
                    let w2 = qw * 2.0;
                    uvx *= w2;
                    uvy *= w2;
                    uvz *= w2;
                    // vec3.scale(uuv, uuv, 2);
                    uuvx *= 2.0;
                    uuvy *= 2.0;
                    uuvz *= 2.0;
                    // return vec3.add(out, a, vec3.add(out, uv, uuv));

                    Self::from_values(
                        x + uvx + uuvx,
                        y + uvy + uuvy,
                        z + uvz + uuvz,
                    )
                }

                #[inline]
                pub fn transform_mat4(&self, m: &Mat4<$t>) -> Self {
                    let x = self.0[0];
                    let y = self.0[1];
                    let z = self.0[2];
                    let mut w = m.0[3] * x + m.0[7] * y + m.0[11] * z + m.0[15];
                    w = if w == 0.0 { 1.0 } else { w };

                    Self::from_values(
                        (m.0[0] * x + m.0[4] * y + m.0[8] * z + m.0[12]) / w,
                        (m.0[1] * x + m.0[5] * y + m.0[9] * z + m.0[13]) / w,
                        (m.0[2] * x + m.0[6] * y + m.0[10] * z + m.0[14]) / w,
                    )
                }

                #[inline]
                pub fn rotate_x(&self, b: &Vec3<$t>, rad: $t) -> Self {
                    let mut p = [0.0 as $t; 3];
                    let mut r = [0.0 as $t; 3];
                    //Translate point to the origin
                    p[0] = self.0[0] - b.0[0];
                    p[1] = self.0[1] - b.0[1];
                    p[2] = self.0[2] - b.0[2];

                    //perform rotation
                    r[0] = p[0];
                    r[1] = p[1] * rad.cos() - p[2] * rad.sin();
                    r[2] = p[1] * rad.sin() + p[2] * rad.cos();

                    Self::from_values(
                        r[0] + b.0[0],
                        r[1] + b.0[1],
                        r[2] + b.0[2],
                    )
                }

                #[inline]
                pub fn rotate_y(&self, b: &Vec3<$t>, rad: $t) -> Self {
                    let mut p = [0.0 as $t; 3];
                    let mut r = [0.0 as $t; 3];
                    //Translate point to the origin
                    p[0] = self.0[0] - b.0[0];
                    p[1] = self.0[1] - b.0[1];
                    p[2] = self.0[2] - b.0[2];

                    //perform rotation
                    r[0] = p[2] * rad.sin() + p[0] * rad.cos();
                    r[1] = p[1];
                    r[2] = p[2] * rad.cos() - p[0] * rad.sin();

                    Self::from_values(
                        r[0] + b.0[0],
                        r[1] + b.0[1],
                        r[2] + b.0[2],
                    )
                }

                #[inline]
                pub fn rotate_z(&self, b: &Vec3<$t>, rad: $t) -> Self {
                    let mut p = [0.0 as $t; 3];
                    let mut r = [0.0 as $t; 3];
                    //Translate point to the origin
                    p[0] = self.0[0] - b.0[0];
                    p[1] = self.0[1] - b.0[1];
                    p[2] = self.0[2] - b.0[2];

                    //perform rotation
                    r[0] = p[0] * rad.cos() - p[1] * rad.sin();
                    r[1] = p[0] * rad.sin() + p[1] * rad.cos();
                    r[2] = p[2];

                    Self::from_values(
                        r[0] + b.0[0],
                        r[1] + b.0[1],
                        r[2] + b.0[2],
                    )
                }

                #[inline]
                pub fn angle(&self, b: &Vec3<$t>) -> $t {
                    let ax = self.0[0];
                    let ay = self.0[1];
                    let az = self.0[2];
                    let bx = b.0[0];
                    let by = b.0[1];
                    let bz = b.0[2];
                    let mag = ((ax * ax + ay * ay + az * az) * (bx * bx + by * by + bz * bz)).sqrt();
                    let cosine = if mag == 0.0 {
                        mag
                    } else {
                        self.dot(b) / mag
                    };
                    // Math.min(Math.max(cosine, -1), 1) clamps the cosine between -1 and 1
                    cosine.max(-1.0).min(1.0).acos()
                }

                #[inline]
                pub fn approximate_eq(&self, b: &Vec3<$t>) -> bool {
                    let a0 = self.0[0];
                    let a1 = self.0[1];
                    let a2 = self.0[2];
                    let b0 = b.0[0];
                    let b1 = b.0[1];
                    let b2 = b.0[2];
                    (
                      (a0 - b0).abs() <=
                        $epsilon * (1.0 as $t).max(a0.abs()).max(b0.abs()) &&
                      (a1 - b1).abs() <=
                        $epsilon * (1.0 as $t).max(a1.abs()).max(b1.abs()) &&
                      (a2 - b2).abs() <=
                        $epsilon * (1.0 as $t).max(a2.abs()).max(b2.abs())
                    )
                }
            }

            impl Add<Vec3<$t>> for Vec3<$t> {
                type Output = Vec3<$t>;

                #[inline]
                fn add(self, b: Vec3<$t>) -> Vec3<$t> {
                    let mut out = Vec3::<$t>::new();
                    out.0[0] = self.0[0] + b.0[0];
                    out.0[1] = self.0[1] + b.0[1];
                    out.0[2] = self.0[2] + b.0[2];
                    out
                }
            }

            impl Sub<Vec3<$t>> for Vec3<$t> {
                type Output = Vec3<$t>;

                #[inline]
                fn sub(self, b: Vec3<$t>) -> Vec3<$t> {
                    let mut out = Vec3::<$t>::new();
                    out.0[0] = self.0[0] - b.0[0];
                    out.0[1] = self.0[1] - b.0[1];
                    out.0[2] = self.0[2] - b.0[2];
                    out
                }
            }

            impl Mul<Vec3<$t>> for Vec3<$t> {
                type Output = Vec3<$t>;

                #[inline]
                fn mul(self, b: Vec3<$t>) -> Vec3<$t> {
                    let mut out = Vec3::<$t>::new();
                    out.0[0] = self.0[0] * b.0[0];
                    out.0[1] = self.0[1] * b.0[1];
                    out.0[2] = self.0[2] * b.0[2];
                    out
                }
            }

            impl Mul<$t> for Vec3<$t> {
                type Output = Vec3<$t>;

                #[inline]
                fn mul(self, b: $t) -> Vec3<$t> {
                    let mut out = Vec3::<$t>::new();
                    out.0[0] = self.0[0] * b;
                    out.0[1] = self.0[1] * b;
                    out.0[2] = self.0[2] * b;
                    out
                }
            }

            impl Div<Vec3<$t>> for Vec3<$t> {
                type Output = Vec3<$t>;

                #[inline]
                fn div(self, b: Vec3<$t>) -> Vec3<$t> {
                    let mut out = Vec3::<$t>::new();
                    out.0[0] = self.0[0] / b.0[0];
                    out.0[1] = self.0[1] / b.0[1];
                    out.0[2] = self.0[2] / b.0[2];
                    out
                }
            }

            impl Div<$t> for Vec3<$t> {
                type Output = Vec3<$t>;

                #[inline]
                fn div(self, b: $t) -> Vec3<$t> {
                    let mut out = Vec3::<$t>::new();
                    out.0[0] = self.0[0] / b;
                    out.0[1] = self.0[1] / b;
                    out.0[2] = self.0[2] / b;
                    out
                }
            }

            impl Display for Vec3<$t> {
                fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                    let value = self.0.iter().map(|v| v.to_string()).collect::<Vec<_>>().join(", ");
                    f.write_fmt(format_args!("vec3({})", value))
                }
            }
        )+
    };
}

float! {
    (f64, EPSILON_F64),
    (f32, EPSILON_F32)
}
