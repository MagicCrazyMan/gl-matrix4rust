use std::{
    fmt::Display,
    ops::{Add, Mul},
};

use crate::{mat3::Mat3, vec3::Vec3, EPSILON_F32, EPSILON_F64};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EulerOrder {
    XYZ,
    XZY,
    YZX,
    YXZ,
    ZXY,
    ZYX,
}

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
                    let matr = Mat3::<$t>::from_values(
                        right.0[0],
                        up.0[0],
                        -view.0[0],
                        right.0[1],
                        up.0[1],
                        -view.0[1],
                        right.0[2],
                        up.0[2],
                        -view.0[2],
                    );

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
                pub fn from_euler(x: $t, y: $t, z: $t) -> Self {
                    Self::from_euler_with_order(x, y, z, EulerOrder::ZYX)
                }

                #[inline]
                pub fn from_euler_with_order(x: $t, y: $t, z: $t, order: EulerOrder) -> Self {
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
                pub fn axis_angle(&self) -> (Vec3::<$t>, $t) {
                    let rad = self.0[3].acos() * 2.0;
                    let s = (rad / 2.0).sin();
                    if (s > $epsilon) {
                        (
                            Vec3::<$t>::from_values(
                                self.0[0] / s,
                                self.0[1] / s,
                                self.0[2] / s,
                            ),
                            rad
                        )
                    } else {
                        // If s is zero, return any axis (no rotation - axis does not matter)
                        (
                            Vec3::<$t>::from_values(
                                1.0,
                                0.0,
                                0.0,
                            ),
                            rad
                        )
                    }
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

#[cfg(test)]
#[rustfmt::skip]
mod tests {
    macro_rules! float_test {
        ($t:tt, $epsilon:expr, $pi:expr, $e:expr, $sqrt2:expr) => {
            use std::sync::OnceLock;

            use crate::error::Error;
            use crate::quat::Quat;
            use crate::vec3::Vec3;
            use crate::mat3::Mat3;
            use crate::mat4::Mat4;

            static QUAT_A_RAW: [$t; 4] = [1.0, 2.0, 3.0, 4.0];
            static QUAT_B_RAW: [$t; 4] = [5.0, 6.0, 7.0, 8.0];
            static QUAT_IDENTITY_RAW: [$t; 4] = [0.0, 0.0, 0.0, 1.0];

            static QUAT_A: OnceLock<Quat<$t>> = OnceLock::new();
            static QUAT_B: OnceLock<Quat<$t>> = OnceLock::new();
            static QUAT_IDENTITY: OnceLock<Quat<$t>> = OnceLock::new();

            fn quat_a() -> &'static Quat<$t> {
                QUAT_A.get_or_init(|| {
                    Quat::<$t>::from_slice(&QUAT_A_RAW)
                })
            }

            fn quat_b() -> &'static Quat<$t> {
                QUAT_B.get_or_init(|| {
                    Quat::<$t>::from_slice(&QUAT_B_RAW)
                })
            }

            fn quat_identity() -> &'static Quat<$t> {
                QUAT_IDENTITY.get_or_init(|| {
                    Quat::<$t>::from_slice(&QUAT_IDENTITY_RAW)
                })
            }

            #[test]
            fn new() {
                assert_eq!(
                    Quat::<$t>::new().raw(),
                    &[0.0, 0.0, 0.0, 0.0]
                );
            }

            #[test]
            fn new_identity() {
                assert_eq!(
                    Quat::<$t>::new_identity().raw(),
                    &[0.0, 0.0, 0.0, 1.0]
                );
            }

            #[test]
            fn from_slice() {
                assert_eq!(
                    Quat::<$t>::from_slice(&[3.0, 4.0, 5.0, 6.0]).raw(),
                    &[3.0, 4.0, 5.0, 6.0]
                );
            }

            #[test]
            fn from_values() {
                assert_eq!(
                    Quat::<$t>::from_values(3.0, 4.0, 5.0, 6.0).raw(),
                    &[3.0, 4.0, 5.0, 6.0]
                );
            }

            #[test]
            fn scale() {
                assert_eq!(
                    (*quat_a() * 2.0).raw(),
                    &[2.0, 4.0, 6.0, 8.0]
                );
            }

            #[test]
            fn scale_add() {
                assert_eq!(
                    (*quat_a() + *quat_b() * 0.5).raw(),
                    &[3.5, 5.0, 6.5, 8.0]
                );
            }

            #[test]
            fn squared_length() {
                assert_eq!(
                    quat_a().squared_length(),
                    30.0
                );
            }

            #[test]
            fn length() {
                assert_eq!(
                    quat_a().length(),
                    5.477225575051661
                );
            }

            #[test]
            fn normalize() {
                assert_eq!(
                    Quat::<$t>::from_values(5.0, 0.0, 0.0, 0.0).normalize().raw(),
                    &[1.0 ,0.0, 0.0, 0.0]
                );
            }

            #[test]
            fn dot() {
                assert_eq!(
                    quat_a().dot(quat_b()),
                    70.0
                );
            }

            #[test]
            fn lerp() {
                assert_eq!(
                    quat_a().lerp(quat_b(), 0.5).raw(),
                    &[3.0, 4.0, 5.0, 6.0]
                );
            }

            #[test]
            fn slerp() {
                assert_eq!(
                    quat_a().slerp(quat_b(), 0.5).raw(),
                    &[3.0, 4.0, 5.0, 6.0]
                );
            }
            
            #[test]
            fn conjugate() {
                assert_eq!(
                    quat_a().conjugate().raw(),
                    &[-1.0, -2.0, -3.0, 4.0]
                );
            }

            #[test]
            fn set() {
                let mut mat = Quat::<$t>::new();
                mat.set(3.0, 4.0, 5.0, 6.0);

                assert_eq!(
                    mat.raw(),
                    &[3.0, 4.0, 5.0, 6.0]
                );
            }

            #[test]
            fn set_slice() {
                let mut mat = Quat::<$t>::new();
                mat.set_slice(&[3.0, 4.0, 5.0, 6.0]);

                assert_eq!(
                    mat.raw(),
                    &[3.0, 4.0, 5.0, 6.0]
                );
            }

            #[test]
            fn add() {
                assert_eq!(
                    (*quat_a() + *quat_b()).raw(),
                    &[6.0, 8.0, 10.0, 12.0]
                );
            }

            #[test]
            fn mul() {
                assert_eq!(
                    (*quat_a() * *quat_b()).raw(),
                    &[24.0, 48.0, 48.0, -6.0]
                );
            }

            #[test]
            fn mul_scalar() {
                assert_eq!(
                    (*quat_a() * 2.0).raw(),
                    &[2.0, 4.0, 6.0, 8.0]
                );
            }

            #[test]
            fn mul_scalar_add() {
                assert_eq!(
                    (*quat_a() + *quat_b() * 0.5).raw(),
                    &[3.5, 5.0, 6.5, 8.0]
                );
            }

            #[test]
            fn approximate_eq() {
                let vec_a = Quat::<$t>::from_values(0.0, 0.0, 0.0, 1.0);
                let vec_b = Quat::<$t>::from_values(0.0, 0.0, 0.0, 1.0);
                let vec_c = Quat::<$t>::from_values(0.0, 1.0, 0.0, 0.0);
                let vec_d = Quat::<$t>::from_values(1e-16, 1.0, 2.0, 3.0);
                let vec_e = Quat::<$t>::from_values(0.0, -1.0, 0.0, 0.0);

                assert_eq!(
                    true,
                    vec_a.approximate_eq(&vec_b)
                );
                assert_eq!(
                    false,
                    vec_a.approximate_eq(&vec_c)
                );
                assert_eq!(
                    true,
                    vec_a.approximate_eq(&vec_d)
                );
                assert_eq!(
                    true,
                    vec_c.approximate_eq(&vec_e)
                );
            }

            #[test]
            fn display() {
                let out = quat_a().to_string();
                assert_eq!(
                    out,
                    "quat(1, 2, 3, 4)"
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
        fn from_mat3() -> Result<(), Error> {
            let mat = Mat3::<f32>::from_values(
                1.0, 0.0,  0.0,
                0.0, 0.0, -1.0,
                0.0, 1.0,  0.0,
            );
            let quat = Quat::<f32>::from_mat3(&mat);
            assert_eq!(
                quat.raw(),
                &[-0.70710677, 0.0, 0.0, 0.70710677]
            );

            assert_eq!(
                Vec3::<f32>::from_values(0.0, 1.0, 0.0).transform_quat(&quat).raw(),
                &[0.0, 5.9604645e-8, -0.99999994]
            );

            let mat = Mat3::<f32>::from_mat4(&Mat4::<f32>::from_look_at(
                &Vec3::<f32>::from_values(0.0, 0.0, 0.0),
                &Vec3::<f32>::from_values(0.0, 0.0, 1.0),
                &Vec3::<f32>::from_values(0.0, 1.0, 0.0),
            ))
                .invert()?
                .transpose();
            let quat = Quat::<f32>::from_mat3(&mat);
            assert_eq!(
                Vec3::<f32>::from_values(3.0, 2.0, -1.0).transform_quat(&quat.normalize()).raw(),
                Vec3::<f32>::from_values(3.0, 2.0, -1.0).transform_mat3(&mat).raw()
            );

            let mat = Mat3::<f32>::from_mat4(&Mat4::<f32>::from_look_at(
                &Vec3::<f32>::from_values(0.0, 0.0, 0.0),
                &Vec3::<f32>::from_values(-1.0, 0.0, 0.0),
                &Vec3::<f32>::from_values(0.0, -1.0, 0.0),
            ))
                .invert()?
                .transpose();
            let quat = Quat::<f32>::from_mat3(&mat);
            assert_eq!(
                Vec3::<f32>::from_values(3.0, 2.0, -1.0).transform_quat(&quat.normalize()).raw(),
                &[-1.0000005, -2.0000005, 3.0000005]
            );

            let mat = Mat3::<f32>::from_mat4(&Mat4::<f32>::from_look_at(
                &Vec3::<f32>::from_values(0.0, 0.0, 0.0),
                &Vec3::<f32>::from_values(0.0, 0.0, -1.0),
                &Vec3::<f32>::from_values(0.0, -1.0, 0.0),
            ))
                .invert()?
                .transpose();
            let quat = Quat::<f32>::from_mat3(&mat);
            assert_eq!(
                Vec3::<f32>::from_values(3.0, 2.0, -1.0).transform_quat(&quat.normalize()).raw(),
                Vec3::<f32>::from_values(3.0, 2.0, -1.0).transform_mat3(&mat).raw()
            );

            Ok(())
        }

        #[test]
        fn from_rotate_to() {
            assert_eq!(
                Quat::<f32>::from_rotate_to(
                    &Vec3::<f32>::from_values(0.0, 1.0, 0.0),
                    &Vec3::<f32>::from_values(1.0, 0.0, 0.0),
                ).raw(),
                &[0.0, 0.0, -0.70710677, 0.70710677]
            );

            let quat = Quat::<f32>::from_rotate_to(
                &Vec3::<f32>::from_values(0.0, 1.0, 0.0),
                &Vec3::<f32>::from_values(0.0, 1.0, 0.0),
            );
            assert_eq!(
                Vec3::<f32>::from_values(0.0, 1.0, 0.0).transform_quat(&quat).raw(),
                &[0.0, 1.0, 0.0]
            );

            let quat = Quat::<f32>::from_rotate_to(
                &Vec3::<f32>::from_values(1.0, 0.0, 0.0),
                &Vec3::<f32>::from_values(-1.0, 0.0, 0.0),
            );
            assert_eq!(
                Vec3::<f32>::from_values(1.0, 0.0, 0.0).transform_quat(&quat).raw(),
                &[-1.0, 8.742278e-8, 0.0]
            );

            let quat = Quat::<f32>::from_rotate_to(
                &Vec3::<f32>::from_values(0.0, 1.0, 0.0),
                &Vec3::<f32>::from_values(0.0, -1.0, 0.0),
            );
            assert_eq!(
                Vec3::<f32>::from_values(0.0, 1.0, 0.0).transform_quat(&quat).raw(),
                &[8.742278e-8, -1.0, 0.0]
            );

            let quat = Quat::<f32>::from_rotate_to(
                &Vec3::<f32>::from_values(0.0, 0.0, 1.0),
                &Vec3::<f32>::from_values(0.0, 0.0, -1.0),
            );
            assert_eq!(
                Vec3::<f32>::from_values(0.0, 0.0, 1.0).transform_quat(&quat).raw(),
                &[8.742278e-8, 0.0, -1.0]
            );
        }

        #[test]
        fn from_axes() {
            let quat = Quat::<f32>::from_axes(
                &Vec3::<f32>::from_values(-1.0, 0.0, 0.0),
                &Vec3::<f32>::from_values(0.0, 0.0, -1.0),
                &Vec3::<f32>::from_values(0.0, 1.0, 0.0),
            );
            assert_eq!(
                Vec3::<f32>::from_values(0.0, 0.0, -1.0).transform_quat(&quat).raw(),
                &[1.0000001, 0.0, 1.1920929e-7]
            );
            assert_eq!(
                Vec3::<f32>::from_values(1.0, 0.0, 0.0).transform_quat(&quat).raw(),
                &[-1.1920929e-7, 0.0, 1.0000001]
            );
            
            let quat = Quat::<f32>::from_axes(
                &Vec3::<f32>::from_values(0.0, 0.0, -1.0),
                &Vec3::<f32>::from_values(1.0, 0.0, 0.0),
                &Vec3::<f32>::from_values(0.0, 1.0, 0.0),
            );
            assert_eq!(
                quat.raw(),
                &[-0.0, 0.0, 0.0, 1.0]
            );
        }

        #[test]
        fn angle() {
            assert_eq!(
                quat_a().normalize().angle(&quat_a().normalize()),
                0.0
            );

            // precision consider
            assert_eq!(
                quat_a().normalize().angle(&quat_a().normalize().rotate_x(std::f32::consts::PI / 4.0)),
                0.7853986
            );

            // precision consider
            let quat_a = quat_a().normalize();
            let quat_b = quat_b().normalize();
            let dummy = (quat_a.conjugate() * quat_b).axis_angle();
            assert_eq!(
                quat_a.angle(&quat_b),
                0.5003915
            );
        }

        #[test]
        fn axis_angle() {
            let quat = Quat::<f32>::from_axis_angle(&Vec3::<f32>::from_values(0.0, 1.0, 0.0), 0.0);
            let (_, rad) = quat.axis_angle();
            assert_eq!(
                rad % (std::f32::consts::PI * 2.0),
                0.0
            );

            let quat = Quat::<f32>::from_axis_angle(&Vec3::<f32>::from_values(1.0, 0.0, 0.0), 0.7778);
            let (vec3, rad) = quat.axis_angle();
            assert_eq!(
                vec3.raw(),
                &[1.0, 0.0, 0.0]
            );
            assert_eq!(
                rad,
                0.7778
            );

            let quat = Quat::<f32>::from_axis_angle(&Vec3::<f32>::from_values(0.0, 1.0, 0.0), 0.879546);
            let (vec3, rad) = quat.axis_angle();
            assert_eq!(
                vec3.raw(),
                &[0.0, 1.0, 0.0]
            );
            assert_eq!(
                rad,
                0.879546
            );

            let quat = Quat::<f32>::from_axis_angle(&Vec3::<f32>::from_values(0.0, 0.0, 1.0), 0.123456);
            let (vec3, rad) = quat.axis_angle();
            assert_eq!(
                vec3.raw(),
                &[0.0, 0.0, 1.0000055]
            );
            assert_eq!(
                rad,
                0.12345532
            );

            let quat = Quat::<f32>::from_axis_angle(&Vec3::<f32>::from_values(0.707106, 0.0, 0.707106), std::f32::consts::PI * 0.5);
            let (vec3, rad) = quat.axis_angle();
            assert_eq!(
                vec3.raw(),
                &[0.707106, 0.0, 0.707106]
            );
            assert_eq!(
                rad,
                std::f32::consts::PI * 0.5
            );

            let quat = Quat::<f32>::from_axis_angle(&Vec3::<f32>::from_values(0.65538555, 0.49153915, 0.57346237), 8.8888);
            let (vec3, rad) = quat.axis_angle();
            assert_eq!(
                quat.raw(),
                &[-0.63199174, -0.4739938, -0.55299276, -0.2647929]
            );
            assert_eq!(
                rad > 0.0,
                true
            );
            assert_eq!(
                rad < std::f32::consts::PI * 2.0,
                true
            );
        }
    
        #[test]
        fn invert() {
            assert_eq!(
                quat_a().invert().raw(),
                &[-0.033333335, -0.06666667, -0.10000001, 0.13333334]
            );
        }

        #[test]
        fn pow() {
            // identity quat
            assert_eq!(
                quat_identity().pow(2.1).raw(),
                &QUAT_IDENTITY_RAW
            );

            // power of one
            assert_eq!(
                quat_a().normalize().pow(1.0).length(),
                1.0
            );

            // squared
            assert_eq!(
                quat_a().normalize().pow(2.0).raw(),
                &[0.26666668, 0.53333336, 0.8, 0.06666667]
            );
            assert_eq!(
                quat_a().normalize().pow(2.0).length(),
                1.0
            );

            // conjugate
            assert_eq!(
                quat_a().normalize().pow(-1.0).raw(),
                quat_a().normalize().conjugate().raw()
            );
            assert_eq!(
                quat_a().normalize().pow(-1.0).length(),
                1.0
            );

            // reversible
            assert_eq!(
                quat_a().normalize().pow(2.1).pow(1.0 / 2.1).raw(),
                quat_a().normalize().raw()
            );
            assert_eq!(
                quat_a().normalize().pow(2.1).pow(1.0 / 2.1).length(),
                1.0
            );
        }
    
        #[test]
        fn rotate_x() {
            let quat = quat_identity().rotate_x((90.0f32).to_radians());
            assert_eq!(
                Vec3::<f32>::from_values(0.0, 0.0, -1.0).transform_quat(&quat).raw(),
                &[0.0, 0.99999994, -5.9604645e-8]
            );
        }

        #[test]
        fn rotate_y() {
            let quat = quat_identity().rotate_y((90.0f32).to_radians());
            assert_eq!(
                Vec3::<f32>::from_values(0.0, 0.0, -1.0).transform_quat(&quat).raw(),
                &[-0.99999994, 0.0, -5.9604645e-8]
            );
        }

        #[test]
        fn rotate_z() {
            let quat = quat_identity().rotate_z((90.0f32).to_radians());
            assert_eq!(
                Vec3::<f32>::from_values(0.0, 1.0, 0.0).transform_quat(&quat).raw(),
                &[-0.99999994, 5.9604645e-8, 0.0]
            );
        }

        #[test]
        fn from_euler() {
            assert_eq!(
                Quat::<f32>::from_euler(-30.0, 30.0, 30.0).raw(),
                &[-0.3061862, 0.17677669, 0.3061862, 0.8838835]
            );

            let quat = Quat::<f32>::from_euler(-90.0, 0.0, 0.0);
            assert_eq!(
                Vec3::<f32>::from_values(0.0, 1.0, 0.0).transform_quat(&quat).raw(),
                &[0.0, 5.9604645e-8, -0.99999994]
            );
        }

        #[test]
        fn from_axis_angle() {
            assert_eq!(
                Quat::<f32>::from_axis_angle(
                    &Vec3::<f32>::from_values(1.0, 0.0, 0.0),
                    std::f32::consts::PI * 0.5,
                ).raw(),
                &[0.70710677, 0.0, 0.0, 0.70710677]
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
        fn from_mat3() -> Result<(), Error> {
            let mat = Mat3::<f64>::from_values(
                1.0, 0.0,  0.0,
                0.0, 0.0, -1.0,
                0.0, 1.0,  0.0,
            );
            let quat = Quat::<f64>::from_mat3(&mat);
            assert_eq!(
                quat.raw(),
                &[-0.7071067811865475, 0.0, 0.0, 0.7071067811865476]
            );

            assert_eq!(
                Vec3::<f64>::from_values(0.0, 1.0, 0.0).transform_quat(&quat).raw(),
                &[0.0, 2.220446049250313e-16, -1.0]
            );

            let mat = Mat3::<f64>::from_mat4(&Mat4::<f64>::from_look_at(
                &Vec3::<f64>::from_values(0.0, 0.0, 0.0),
                &Vec3::<f64>::from_values(0.0, 0.0, 1.0),
                &Vec3::<f64>::from_values(0.0, 1.0, 0.0),
            ))
                .invert()?
                .transpose();
            let quat = Quat::<f64>::from_mat3(&mat);
            assert_eq!(
                Vec3::<f64>::from_values(3.0, 2.0, -1.0).transform_quat(&quat.normalize()).raw(),
                Vec3::<f64>::from_values(3.0, 2.0, -1.0).transform_mat3(&mat).raw()
            );

            let mat = Mat3::<f64>::from_mat4(&Mat4::<f64>::from_look_at(
                &Vec3::<f64>::from_values(0.0, 0.0, 0.0),
                &Vec3::<f64>::from_values(-1.0, 0.0, 0.0),
                &Vec3::<f64>::from_values(0.0, -1.0, 0.0),
            ))
                .invert()?
                .transpose();
            let quat = Quat::<f64>::from_mat3(&mat);
            assert_eq!(
                Vec3::<f64>::from_values(3.0, 2.0, -1.0).transform_quat(&quat.normalize()).raw(),
                &[-0.9999999999999991, -2.0, 3.0]
            );

            let mat = Mat3::<f64>::from_mat4(&Mat4::<f64>::from_look_at(
                &Vec3::<f64>::from_values(0.0, 0.0, 0.0),
                &Vec3::<f64>::from_values(0.0, 0.0, -1.0),
                &Vec3::<f64>::from_values(0.0, -1.0, 0.0),
            ))
                .invert()?
                .transpose();
            let quat = Quat::<f64>::from_mat3(&mat);
            assert_eq!(
                Vec3::<f64>::from_values(3.0, 2.0, -1.0).transform_quat(&quat.normalize()).raw(),
                Vec3::<f64>::from_values(3.0, 2.0, -1.0).transform_mat3(&mat).raw()
            );

            Ok(())
        }

        #[test]
        fn from_rotate_to() {
            assert_eq!(
                Quat::<f64>::from_rotate_to(
                    &Vec3::<f64>::from_values(0.0, 1.0, 0.0),
                    &Vec3::<f64>::from_values(1.0, 0.0, 0.0),
                ).raw(),
                &[0.0, 0.0, -0.7071067811865475, 0.7071067811865475]
            );

            let quat = Quat::<f64>::from_rotate_to(
                &Vec3::<f64>::from_values(0.0, 1.0, 0.0),
                &Vec3::<f64>::from_values(0.0, 1.0, 0.0),
            );
            assert_eq!(
                Vec3::<f64>::from_values(0.0, 1.0, 0.0).transform_quat(&quat).raw(),
                &[0.0, 1.0, 0.0]
            );

            let quat = Quat::<f64>::from_rotate_to(
                &Vec3::<f64>::from_values(1.0, 0.0, 0.0),
                &Vec3::<f64>::from_values(-1.0, 0.0, 0.0),
            );
            assert_eq!(
                Vec3::<f64>::from_values(1.0, 0.0, 0.0).transform_quat(&quat).raw(),
                &[-1.0, -1.2246467991473532e-16, 0.0]
            );

            let quat = Quat::<f64>::from_rotate_to(
                &Vec3::<f64>::from_values(0.0, 1.0, 0.0),
                &Vec3::<f64>::from_values(0.0, -1.0, 0.0),
            );
            assert_eq!(
                Vec3::<f64>::from_values(0.0, 1.0, 0.0).transform_quat(&quat).raw(),
                &[-1.2246467991473532e-16, -1.0, 0.0]
            );

            let quat = Quat::<f64>::from_rotate_to(
                &Vec3::<f64>::from_values(0.0, 0.0, 1.0),
                &Vec3::<f64>::from_values(0.0, 0.0, -1.0),
            );
            assert_eq!(
                Vec3::<f64>::from_values(0.0, 0.0, 1.0).transform_quat(&quat).raw(),
                &[-1.2246467991473532e-16, 0.0, -1.0]
            );
        }

        #[test]
        fn from_axes() {
            let quat = Quat::<f64>::from_axes(
                &Vec3::<f64>::from_values(-1.0, 0.0, 0.0),
                &Vec3::<f64>::from_values(0.0, 0.0, -1.0),
                &Vec3::<f64>::from_values(0.0, 1.0, 0.0),
            );
            assert_eq!(
                Vec3::<f64>::from_values(0.0, 0.0, -1.0).transform_quat(&quat).raw(),
                &[1.0, 0.0, -2.220446049250313e-16]
            );
            assert_eq!(
                Vec3::<f64>::from_values(1.0, 0.0, 0.0).transform_quat(&quat).raw(),
                &[2.220446049250313e-16, 0.0, 1.0]
            );
            
            let quat = Quat::<f64>::from_axes(
                &Vec3::<f64>::from_values(0.0, 0.0, -1.0),
                &Vec3::<f64>::from_values(1.0, 0.0, 0.0),
                &Vec3::<f64>::from_values(0.0, 1.0, 0.0),
            );
            assert_eq!(
                quat.raw(),
                &[0.0, 0.0, 0.0, 1.0]
            );
        }

        #[test]
        fn angle() {
            assert_eq!(
                quat_a().normalize().angle(&quat_a().normalize()),
                4.2146848510894035e-8
            );

            // precision consider
            assert_eq!(
                quat_a().normalize().angle(&quat_a().normalize().rotate_x(std::f64::consts::PI / 4.0)),
                0.7853981633974491
            );

            let quat_a = quat_a().normalize();
            let quat_b = quat_b().normalize();
            let dummy = (quat_a.conjugate() * quat_b).axis_angle();
            assert_eq!(
                quat_a.angle(&quat_b),
                dummy.1
            );
        }

        #[test]
        fn axis_angle() {
            let quat = Quat::<f64>::from_axis_angle(&Vec3::<f64>::from_values(0.0, 1.0, 0.0), 0.0);
            let (_, rad) = quat.axis_angle();
            assert_eq!(
                rad % (std::f64::consts::PI * 2.0),
                0.0
            );

            let quat = Quat::<f64>::from_axis_angle(&Vec3::<f64>::from_values(1.0, 0.0, 0.0), 0.7778);
            let (vec3, rad) = quat.axis_angle();
            assert_eq!(
                vec3.raw(),
                &[1.0, 0.0, 0.0]
            );
            assert_eq!(
                rad,
                0.7778
            );

            let quat = Quat::<f64>::from_axis_angle(&Vec3::<f64>::from_values(0.0, 1.0, 0.0), 0.879546);
            let (vec3, rad) = quat.axis_angle();
            assert_eq!(
                vec3.raw(),
                &[0.0, 1.0000000000000002, 0.0]
            );
            assert_eq!(
                rad,
                0.8795459999999998
            );

            let quat = Quat::<f64>::from_axis_angle(&Vec3::<f64>::from_values(0.0, 0.0, 1.0), 0.123456);
            let (vec3, rad) = quat.axis_angle();
            assert_eq!(
                vec3.raw(),
                &[0.0, 0.0, 1.0000000000000138]
            );
            assert_eq!(
                rad,
                0.1234559999999983
            );

            let quat = Quat::<f64>::from_axis_angle(&Vec3::<f64>::from_values(0.707106, 0.0, 0.707106), std::f64::consts::PI * 0.5);
            let (vec3, rad) = quat.axis_angle();
            assert_eq!(
                vec3.raw(),
                &[0.707106, 0.0, 0.707106]
            );
            assert_eq!(
                rad,
                std::f64::consts::PI * 0.5
            );

            let quat = Quat::<f64>::from_axis_angle(&Vec3::<f64>::from_values(0.65538555, 0.49153915, 0.57346237), 8.8888);
            let (vec3, rad) = quat.axis_angle();
            assert_eq!(
                quat.raw(),
                &[-0.6319917917288304, -0.4739938317428059, -0.5529928310219251, -0.2647927364605179]
            );
            assert_eq!(
                rad > 0.0,
                true
            );
            assert_eq!(
                rad < std::f64::consts::PI * 2.0,
                true
            );
        }
        
        #[test]
        fn invert() {
            assert_eq!(
                quat_a().invert().raw(),
                &[-0.03333333333333333, -0.06666666666666667, -0.1, 0.13333333333333333]
            );
        }
   
        #[test]
        fn pow() {
            // identity quat
            assert_eq!(
                quat_identity().pow(2.1).raw(),
                &QUAT_IDENTITY_RAW
            );

            // power of one
            assert_eq!(
                quat_a().normalize().pow(1.0).length(),
                0.9999999999999999
            );

            // squared
            assert_eq!(
                quat_a().normalize().pow(2.0).raw(),
                &[0.26666666666666666, 0.5333333333333333, 0.7999999999999999, 0.06666666666666689]
            );
            assert_eq!(
                quat_a().normalize().pow(2.0).length(),
                1.0
            );

            // conjugate
            assert_eq!(
                quat_a().normalize().pow(-1.0).raw(),
                &[-0.1825741858350554, -0.3651483716701108, -0.5477225575051663, 0.7302967433402217]
            );
            assert_eq!(
                quat_a().normalize().pow(-1.0).length(),
                1.0000000000000002
            );

            // reversible
            assert_eq!(
                quat_a().normalize().pow(2.1).pow(1.0 / 2.1).raw(),
                quat_a().normalize().raw()
            );
            assert_eq!(
                quat_a().normalize().pow(2.1).pow(1.0 / 2.1).length(),
                0.9999999999999999
            );
        }
         
        #[test]
        fn rotate_x() {
            let quat = quat_identity().rotate_x((90.0f64).to_radians());
            assert_eq!(
                Vec3::<f64>::from_values(0.0, 0.0, -1.0).transform_quat(&quat).raw(),
                &[0.0, 1.0, -2.220446049250313e-16]
            );
        }

        #[test]
        fn rotate_y() {
            let quat = quat_identity().rotate_y((90.0f64).to_radians());
            assert_eq!(
                Vec3::<f64>::from_values(0.0, 0.0, -1.0).transform_quat(&quat).raw(),
                &[-1.0, 0.0, -2.220446049250313e-16]
            );
        }

        #[test]
        fn rotate_z() {
            let quat = quat_identity().rotate_z((90.0f64).to_radians());
            assert_eq!(
                Vec3::<f64>::from_values(0.0, 1.0, 0.0).transform_quat(&quat).raw(),
                &[-1.0, 2.220446049250313e-16, 0.0]
            );
        }

        #[test]
        fn from_euler() {
            assert_eq!(
                Quat::<f64>::from_euler(-30.0, 30.0, 30.0).raw(),
                &[-0.30618621784789724, 0.17677669529663687, 0.30618621784789724, 0.8838834764831845]
            );

            let quat = Quat::<f64>::from_euler(-90.0, 0.0, 0.0);
            assert_eq!(
                Vec3::<f64>::from_values(0.0, 1.0, 0.0).transform_quat(&quat).raw(),
                &[0.0, 2.220446049250313e-16, -1.0]
            );
        }

        #[test]
        fn from_axis_angle() {
            assert_eq!(
                Quat::<f64>::from_axis_angle(
                    &Vec3::<f64>::from_values(1.0, 0.0, 0.0),
                    std::f64::consts::PI * 0.5,
                ).raw(),
                &[0.7071067811865475, 0.0, 0.0, 0.7071067811865476]
            );
        }
    }
}
