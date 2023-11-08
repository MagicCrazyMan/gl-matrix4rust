use std::{
    fmt::Display,
    ops::{Add, Mul},
};

use crate::{mat3::Mat3, vec3::Vec3, EulerOrder, EPSILON_F32, EPSILON_F64};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Quat<T = f32>(pub [T; 4]);

macro_rules! float {
    ($(($t:tt, $epsilon:expr, $pi:expr)),+) => {
        $(
            impl Quat<$t> {
                #[inline]
                pub fn new() -> Self {
                    Self([0.0; 4])
                }

                #[inline]
                pub fn new_identity() -> Self {
                    Self([0.0, 0.0, 0.0, 1.0])
                }

                #[inline]
                pub fn from_values(x: $t, y: $t, z: $t, w: $t) -> Self {
                    Self([x, y, z, w])
                }

                #[inline]
                pub fn from_slice([x, y, z, w]: &[$t; 4]) -> Self {
                    Self([*x, *y, *z, *w])
                }

                #[inline]
                pub fn from_axis_angle(axis: &Vec3::<$t>, rad: $t) -> Self {
                    let rad = rad * 0.5;
                    let s = rad.sin();

                    Self::from_values(
                        s * axis.0[0],
                        s * axis.0[1],
                        s * axis.0[2],
                        rad.cos(),
                    )
                }

                #[inline]
                pub fn from_axes(view: &Vec3::<$t>, right: &Vec3::<$t>, up: &Vec3::<$t>) -> Self {
                    let mut matr = Mat3::<$t>::new();
                    matr.0[0] = right.0[0];
                    matr.0[3] = right.0[1];
                    matr.0[6] = right.0[2];

                    matr.0[1] = up.0[0];
                    matr.0[4] = up.0[1];
                    matr.0[7] = up.0[2];

                    matr.0[2] = -view.0[0];
                    matr.0[5] = -view.0[1];
                    matr.0[8] = -view.0[2];

                    Self::from_mat3(&matr).normalize()
                }

                #[inline]
                pub fn from_mat3(m: &Mat3::<$t>) -> Self {
                    let mut out = Self::new();

                    // Algorithm in Ken Shoemake's article in 1987 SIGGRAPH course notes
                    // article "Quaternion Calculus and Fast Animation".
                    let f_trace = m.0[0] + m.0[4] + m.0[8];
                    let mut f_root;

                    if f_trace > 0.0 {
                      // |w| > 1/2, may as well choose w > 1/2
                      f_root = (f_trace + 1.0).sqrt(); // 2w
                      out.0[3] = 0.5 * f_root;
                      f_root = 0.5 / f_root; // 1/(4w)
                      out.0[0] = (m.0[5] - m.0[7]) * f_root;
                      out.0[1] = (m.0[6] - m.0[2]) * f_root;
                      out.0[2] = (m.0[1] - m.0[3]) * f_root;
                    } else {
                      // |w| <= 1/2
                      let mut i = 0;
                      if m.0[4] > m.0[0] { i = 1 };
                      if m.0[8] > m.0[i * 3 + i] { i = 2 };
                      let j = (i + 1) % 3;
                      let k = (i + 2) % 3;

                      f_root = (m.0[i * 3 + i] - m.0[j * 3 + j] - m.0[k * 3 + k] + 1.0).sqrt();
                      out.0[i] = 0.5 * f_root;
                      f_root = 0.5 / f_root;
                      out.0[3] = (m.0[j * 3 + k] - m.0[k * 3 + j]) * f_root;
                      out.0[j] = (m.0[j * 3 + i] + m.0[i * 3 + j]) * f_root;
                      out.0[k] = (m.0[k * 3 + i] + m.0[i * 3 + k]) * f_root;
                    }

                    out
                }

                #[inline]
                pub fn from_euler(x: $t, y: $t, z: $t, order: EulerOrder) -> Self {
                    let half_to_rad = $pi / 360.0;
                    let x = x * half_to_rad;
                    let z = z * half_to_rad;
                    let y = y * half_to_rad;

                    let sx = x.sin();
                    let cx = x.cos();
                    let sy = y.sin();
                    let cy = y.cos();
                    let sz = z.sin();
                    let cz = z.cos();

                    match order {
                        EulerOrder::XYZ => {
                            Self::from_values(
                                sx * cy * cz + cx * sy * sz,
                                cx * sy * cz - sx * cy * sz,
                                cx * cy * sz + sx * sy * cz,
                                cx * cy * cz - sx * sy * sz,
                            )
                        },
                        EulerOrder::XZY => {
                            Self::from_values(
                                sx * cy * cz - cx * sy * sz,
                                cx * sy * cz - sx * cy * sz,
                                cx * cy * sz + sx * sy * cz,
                                cx * cy * cz + sx * sy * sz,
                            )
                        }
                        EulerOrder::YXZ => {
                            Self::from_values(
                                sx * cy * cz + cx * sy * sz,
                                cx * sy * cz - sx * cy * sz,
                                cx * cy * sz - sx * sy * cz,
                                cx * cy * cz + sx * sy * sz,
                            )
                        }
                        EulerOrder::YZX => {
                            Self::from_values(
                                sx * cy * cz + cx * sy * sz,
                                cx * sy * cz + sx * cy * sz,
                                cx * cy * sz - sx * sy * cz,
                                cx * cy * cz - sx * sy * sz,
                            )
                        }
                        EulerOrder::ZXY => {
                            Self::from_values(
                                sx * cy * cz - cx * sy * sz,
                                cx * sy * cz + sx * cy * sz,
                                cx * cy * sz + sx * sy * cz,
                                cx * cy * cz - sx * sy * sz,
                            )
                        }
                        EulerOrder::ZYX => {
                            Self::from_values(
                                sx * cy * cz - cx * sy * sz,
                                cx * sy * cz + sx * cy * sz,
                                cx * cy * sz - sx * sy * cz,
                                cx * cy * cz + sx * sy * sz,
                            )
                        }
                    }
                }

                #[inline]
                pub fn from_rotate_to(a: &Vec3::<$t>, b: &Vec3::<$t>) -> Self {
                    let dot = a.dot(b);
                    if (dot < -0.999999) {
                        let x_unit = Vec3::<$t>::from_values(1.0, 0.0, 0.0);
                        let y_unit = Vec3::<$t>::from_values(0.0, 1.0, 0.0);
                        let mut tmp = x_unit.cross(a);
                        if tmp.length() < 0.000001 { tmp = y_unit.cross(a) };
                        tmp = tmp.normalize();
                        Self::from_axis_angle(&tmp, $pi)
                    } else if (dot > 0.999999) {
                        Self::from_values(0.0, 0.0, 0.0, 1.0)
                    } else {
                        let tmp = a.cross(b);
                        let out = Self::from_values(
                            tmp.0[0],
                            tmp.0[1],
                            tmp.0[2],
                            1.0 + dot
                        );
                        out.normalize()
                    }
                }
            }

            impl Quat<$t> {
                #[inline]
                pub fn raw(&self) -> &[$t; 4] {
                    &self.0
                }

                #[inline]
                pub fn angle(&self, b: &Quat::<$t>) -> $t {
                    let dotproduct = self.dot(b);
                    (2.0 * dotproduct * dotproduct - 1.0).acos()
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
                pub fn set_identify(&mut self) -> &mut Self {
                    self.0[0] = 0.0;
                    self.0[1] = 0.0;
                    self.0[2] = 0.0;
                    self.0[3] = 1.0;
                    self
                }

                #[inline]
                pub fn rotate_x(&self, rad: $t) -> Self {
                    let rad = rad * 0.5;

                    let ax = self.0[0];
                    let ay = self.0[1];
                    let az = self.0[2];
                    let aw = self.0[3];
                    let bx = rad.sin();
                    let bw = rad.cos();

                    Self::from_values(
                        ax * bw + aw * bx,
                        ay * bw + az * bx,
                        az * bw - ay * bx,
                        aw * bw - ax * bx,
                    )
                }

                #[inline]
                pub fn rotate_y(&self, rad: $t) -> Self {
                    let rad = rad * 0.5;

                    let ax = self.0[0];
                    let ay = self.0[1];
                    let az = self.0[2];
                    let aw = self.0[3];
                    let by = rad.sin();
                    let bw = rad.cos();

                    Self::from_values(
                        ax * bw - az * by,
                        ay * bw + aw * by,
                        az * bw + ax * by,
                        aw * bw - ay * by,
                    )
                }

                #[inline]
                pub fn rotate_z(&self, rad: $t) -> Self {
                    let rad = rad * 0.5;

                    let ax = self.0[0];
                    let ay = self.0[1];
                    let az = self.0[2];
                    let aw = self.0[3];
                    let bz = rad.sin();
                    let bw = rad.cos();

                    Self::from_values(
                        ax * bw + ay * bz,
                        ay * bw - ax * bz,
                        az * bw + aw * bz,
                        aw * bw - az * bz,
                    )
                }

                #[inline]
                pub fn calculate_w(&self) -> Self {
                    let x = self.0[0];
                    let y = self.0[1];
                    let z = self.0[2];

                    Self::from_values(
                        x,
                        y,
                        z,
                        (1.0 - x * x - y * y - z * z).abs().sqrt(),
                    )
                }

                #[inline]
                pub fn exp(&self) -> Self {
                    let x = self.0[0];
                    let y = self.0[1];
                    let z = self.0[2];
                    let w = self.0[3];

                    let r = (x * x + y * y + z * z).sqrt();
                    let et = w.exp();
                    let s = if r > 0.0 { (et * r.sin()) / r } else { 0.0 };

                    Self::from_values(
                        x * s,
                        y * s,
                        z * s,
                        et * r.cos(),
                    )
                }

                #[inline]
                pub fn ln(&self) -> Self {
                    let x = self.0[0];
                    let y = self.0[1];
                    let z = self.0[2];
                    let w = self.0[3];

                    let r = (x * x + y * y + z * z).sqrt();
                    let t = if r > 0.0 { r.atan2(w) / r } else { 0.0 };

                    Self::from_values(
                        x * t,
                        y * t,
                        z * t,
                        0.5 * (x * x + y * y + z * z + w * w).ln(),
                    )
                }

                #[inline]
                pub fn scale(&self, scale: $t) -> Self {
                    self.mul(scale)
                }

                #[inline]
                pub fn pow(&self, b: $t) -> Self {
                    self.ln().scale(b).exp()
                }

                #[inline]
                pub fn dot(&self, b: &Quat<$t>) -> $t {
                    self.0[0] * b.0[0] + self.0[1] * b.0[1] + self.0[2] * b.0[2] + self.0[3] * b.0[3]
                }

                #[inline]
                pub fn lerp(&self, b: &Quat<$t>, t: $t) -> Self {
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
                pub fn slerp(&self, b: &Quat::<$t>, t: $t) -> Self {
                    let ax = self.0[0];
                    let ay = self.0[1];
                    let az = self.0[2];
                    let aw = self.0[3];
                    let mut bx = b.0[0];
                    let mut by = b.0[1];
                    let mut bz = b.0[2];
                    let mut bw = b.0[3];

                    let omega;
                    let mut cosom;
                    let sinom;
                    let scale0;
                    let scale1;

                    // calc cosine
                    cosom = ax * bx + ay * by + az * bz + aw * bw;
                    // adjust signs (if necessary)
                    if cosom < 0.0 {
                      cosom = -cosom;
                      bx = -bx;
                      by = -by;
                      bz = -bz;
                      bw = -bw;
                    }
                    // calculate coefficients
                    if 1.0 - cosom > $epsilon {
                      // standard case (slerp)
                      omega = cosom.acos();
                      sinom = omega.sin();
                      scale0 = (((1.0 - t) * omega) / sinom).sin();
                      scale1 = ((t * omega) / sinom).sin();
                    } else {
                      // "from" and "to" quaternions are very close
                      //  ... so we can do a linear interpolation
                      scale0 = 1.0 - t;
                      scale1 = t;
                    }

                    Self::from_values(
                        scale0 * ax + scale1 * bx,
                        scale0 * ay + scale1 * by,
                        scale0 * az + scale1 * bz,
                        scale0 * aw + scale1 * bw,
                    )
                }

                #[inline]
                pub fn sqlerp(&self, b: &Quat<$t>, c: &Quat<$t>, d: &Quat<$t>, t: $t) -> Self {
                    let tmp1 = self.slerp(d, t);
                    let tmp2 = b.slerp(c, t);
                    tmp1.slerp(&tmp2, 2.0 * t * (1.0 - t))
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
                pub fn random(&self) -> Self {
                    let u1 = rand::random::<$t>();
                    let u2 = rand::random::<$t>();
                    let u3 = rand::random::<$t>();
                    let sqrt1_minus_u1 = (1.0 - u1).sqrt();
                    let sqrt_u1 = u1.sqrt();
                    Self::from_values(
                        sqrt1_minus_u1 * (2.0 * $pi * u2).sin(),
                        sqrt1_minus_u1 * (2.0 * $pi * u2).cos(),
                        sqrt_u1 * (2.0 * $pi * u3).sin(),
                        sqrt_u1 * (2.0 * $pi * u3).cos(),
                    )
                }

                #[inline]
                pub fn invert(&self) -> Self {
                    let a0 = self.0[0];
                    let a1 = self.0[1];
                    let a2 = self.0[2];
                    let a3 = self.0[3];

                    let dot = a0 * a0 + a1 * a1 + a2 * a2 + a3 * a3;
                    let inv_dot = if dot != 0.0 { 1.0 / dot } else { 0.0 };

                    Self::from_values(
                        -a0 * inv_dot,
                        -a1 * inv_dot,
                        -a2 * inv_dot,
                        a3 * inv_dot,
                    )
                }

                #[inline]
                pub fn conjugate(&self) -> Self {
                    Self::from_values(
                        -self.0[0],
                        -self.0[1],
                        -self.0[2],
                        self.0[3],
                    )
                }

                #[inline]
                pub fn approximate_eq(&self, b: &Quat<$t>) -> bool {
                    self.dot(b).abs() >= 1.0 - $epsilon
                }
            }

            impl Add<Quat<$t>> for Quat<$t> {
                type Output = Quat<$t>;

                #[inline]
                fn add(self, b: Quat<$t>) -> Quat<$t> {
                    let mut out = Quat::<$t>::new();
                    out.0[0] = self.0[0] + b.0[0];
                    out.0[1] = self.0[1] + b.0[1];
                    out.0[2] = self.0[2] + b.0[2];
                    out.0[3] = self.0[3] + b.0[3];
                    out
                }
            }

            impl Mul<Quat<$t>> for Quat<$t> {
                type Output = Quat<$t>;

                #[inline]
                fn mul(self, b: Quat<$t>) -> Quat<$t> {
                    let ax = self.0[0];
                    let ay = self.0[1];
                    let az = self.0[2];
                    let aw = self.0[3];
                    let bx = b.0[0];
                    let by = b.0[1];
                    let bz = b.0[2];
                    let bw = b.0[3];

                    Quat::<$t>::from_values(
                        ax * bw + aw * bx + ay * bz - az * by,
                        ay * bw + aw * by + az * bx - ax * bz,
                        az * bw + aw * bz + ax * by - ay * bx,
                        aw * bw - ax * bx - ay * by - az * bz,
                    )
                }
            }

            impl Mul<$t> for Quat<$t> {
                type Output = Quat<$t>;

                #[inline]
                fn mul(self, b: $t) -> Quat<$t> {
                    Self::from_values(
                        self.0[0] * b,
                        self.0[1] * b,
                        self.0[2] * b,
                        self.0[3] * b,
                    )
                }
            }

            impl Display for Quat<$t> {
                fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                    let value = self.0.iter().map(|v| v.to_string()).collect::<Vec<_>>().join(", ");
                    f.write_fmt(format_args!("quat({})", value))
                }
            }
        )+
    };
}

float! {
    (f64, EPSILON_F64, std::f64::consts::PI),
    (f32, EPSILON_F32, std::f32::consts::PI)
}
