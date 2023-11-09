use std::{
    fmt::Display,
    ops::{Add, Mul, Sub},
};

use crate::{error::Error, quat::Quat, quat2::Quat2, vec3::Vec3, EPSILON_F32, EPSILON_F64};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mat4<T = f32>(pub [T; 16]);

macro_rules! float {
    ($(($t:tt, $epsilon:expr, $pi:expr)),+) => {
        $(
            impl Mat4<$t> {
                #[inline]
                pub fn new() -> Self {
                    Self([0.0; 16])
                }

                #[inline]
                pub fn new_identity() -> Self {
                    let mut out = Self::new();
                    out.0[0] = 1.0;
                    out.0[5] = 1.0;
                    out.0[10] = 1.0;
                    out.0[15] = 1.0;
                    out
                }

                #[inline]
                pub fn from_values(
                    m00: $t,
                    m01: $t,
                    m02: $t,
                    m03: $t,
                    m10: $t,
                    m11: $t,
                    m12: $t,
                    m13: $t,
                    m20: $t,
                    m21: $t,
                    m22: $t,
                    m23: $t,
                    m30: $t,
                    m31: $t,
                    m32: $t,
                    m33: $t,
                ) -> Self {
                    let mut out = Self::new();
                    out.0[0] = m00;
                    out.0[1] = m01;
                    out.0[2] = m02;
                    out.0[3] = m03;
                    out.0[4] = m10;
                    out.0[5] = m11;
                    out.0[6] = m12;
                    out.0[7] = m13;
                    out.0[8] = m20;
                    out.0[9] = m21;
                    out.0[10] = m22;
                    out.0[11] = m23;
                    out.0[12] = m30;
                    out.0[13] = m31;
                    out.0[14] = m32;
                    out.0[15] = m33;

                    out
                }

                #[inline]
                pub fn from_slice(slice: &[$t; 16]) -> Self {
                    let mut out = Self::new();
                    out.0[0] = slice[0];
                    out.0[1] = slice[1];
                    out.0[2] = slice[2];
                    out.0[3] = slice[3];
                    out.0[4] = slice[4];
                    out.0[5] = slice[5];
                    out.0[6] = slice[6];
                    out.0[7] = slice[7];
                    out.0[8] = slice[8];
                    out.0[9] = slice[9];
                    out.0[10] = slice[10];
                    out.0[11] = slice[11];
                    out.0[12] = slice[12];
                    out.0[13] = slice[13];
                    out.0[14] = slice[14];
                    out.0[15] = slice[15];

                    out
                }

                #[inline]
                pub fn from_translation(v: &Vec3<$t>) -> Self {
                    let mut out = Self::new();
                    out.0[0] = 1.0;
                    out.0[1] = 0.0;
                    out.0[2] = 0.0;
                    out.0[3] = 0.0;
                    out.0[4] = 0.0;
                    out.0[5] = 1.0;
                    out.0[6] = 0.0;
                    out.0[7] = 0.0;
                    out.0[8] = 0.0;
                    out.0[9] = 0.0;
                    out.0[10] = 1.0;
                    out.0[11] = 0.0;
                    out.0[12] = v.0[0];
                    out.0[13] = v.0[1];
                    out.0[14] = v.0[2];
                    out.0[15] = 1.0;
                    out
                }

                #[inline]
                pub fn from_scaling(v: &Vec3<$t>) -> Self {
                    let mut out = Self::new();
                    out.0[0] = v.0[0];
                    out.0[1] = 0.0;
                    out.0[2] = 0.0;
                    out.0[3] = 0.0;
                    out.0[4] = 0.0;
                    out.0[5] = v.0[1];
                    out.0[6] = 0.0;
                    out.0[7] = 0.0;
                    out.0[8] = 0.0;
                    out.0[9] = 0.0;
                    out.0[10] = v.0[2];
                    out.0[11] = 0.0;
                    out.0[12] = 0.0;
                    out.0[13] = 0.0;
                    out.0[14] = 0.0;
                    out.0[15] = 1.0;
                    out
                }

                #[inline]
                pub fn from_rotation(rad: $t, axis: &Vec3<$t>) -> Result<Self, Error> {
                    let mut out = Self::new();

                    let mut x = axis.0[0];
                    let mut y = axis.0[1];
                    let mut z = axis.0[2];
                    let mut len = (x * x + y * y + z * z).sqrt();
                    let s;
                    let c;
                    let t;

                    if (len < $epsilon) {
                      return Err(Error::LengthSmallerThanEpsilon);
                    }

                    len = 1.0 / len;
                    x *= len;
                    y *= len;
                    z *= len;

                    s = rad.sin();
                    c = rad.cos();
                    t = 1.0 - c;

                    // Perform rotation-specific matrix multiplication
                    out.0[0] = x * x * t + c;
                    out.0[1] = y * x * t + z * s;
                    out.0[2] = z * x * t - y * s;
                    out.0[3] = 0.0;
                    out.0[4] = x * y * t - z * s;
                    out.0[5] = y * y * t + c;
                    out.0[6] = z * y * t + x * s;
                    out.0[7] = 0.0;
                    out.0[8] = x * z * t + y * s;
                    out.0[9] = y * z * t - x * s;
                    out.0[10] = z * z * t + c;
                    out.0[11] = 0.0;
                    out.0[12] = 0.0;
                    out.0[13] = 0.0;
                    out.0[14] = 0.0;
                    out.0[15] = 1.0;

                    Ok(out)
                }

                #[inline]
                pub fn from_x_rotation(rad: $t) -> Self {
                    let mut out = Self::new();

                    let s = rad.sin();
                    let c = rad.cos();

                    // Perform axis-specific matrix multiplication
                    out.0[0] = 1.0;
                    out.0[1] = 0.0;
                    out.0[2] = 0.0;
                    out.0[3] = 0.0;
                    out.0[4] = 0.0;
                    out.0[5] = c;
                    out.0[6] = s;
                    out.0[7] = 0.0;
                    out.0[8] = 0.0;
                    out.0[9] = -s;
                    out.0[10] = c;
                    out.0[11] = 0.0;
                    out.0[12] = 0.0;
                    out.0[13] = 0.0;
                    out.0[14] = 0.0;
                    out.0[15] = 1.0;
                    out
                }

                #[inline]
                pub fn from_y_rotation(rad: $t) -> Self {
                    let mut out = Self::new();

                    let s = rad.sin();
                    let c = rad.cos();

                    // Perform axis-specific matrix multiplication
                    out.0[0] = c;
                    out.0[1] = 0.0;
                    out.0[2] = -s;
                    out.0[3] = 0.0;
                    out.0[4] = 0.0;
                    out.0[5] = 1.0;
                    out.0[6] = 0.0;
                    out.0[7] = 0.0;
                    out.0[8] = s;
                    out.0[9] = 0.0;
                    out.0[10] = c;
                    out.0[11] = 0.0;
                    out.0[12] = 0.0;
                    out.0[13] = 0.0;
                    out.0[14] = 0.0;
                    out.0[15] = 1.0;
                    out
                }

                #[inline]
                pub fn from_z_rotation(rad: $t) -> Self {
                    let mut out = Self::new();

                    let s = rad.sin();
                    let c = rad.cos();

                    // Perform axis-specific matrix multiplication
                    out.0[0] = c;
                    out.0[1] = s;
                    out.0[2] = 0.0;
                    out.0[3] = 0.0;
                    out.0[4] = -s;
                    out.0[5] = c;
                    out.0[6] = 0.0;
                    out.0[7] = 0.0;
                    out.0[8] = 0.0;
                    out.0[9] = 0.0;
                    out.0[10] = 1.0;
                    out.0[11] = 0.0;
                    out.0[12] = 0.0;
                    out.0[13] = 0.0;
                    out.0[14] = 0.0;
                    out.0[15] = 1.0;
                    out
                }

                #[inline]
                pub fn from_rotation_translation(
                    q: &Quat2<$t>,
                    v: &Vec3<$t>
                ) -> Self {
                    let mut out = Self::new();

                    // Quaternion math
                    let x = q.0[0];
                    let y = q.0[1];
                    let z = q.0[2];
                    let w = q.0[3];
                    let x2 = x + x;
                    let y2 = y + y;
                    let z2 = z + z;

                    let xx = x * x2;
                    let xy = x * y2;
                    let xz = x * z2;
                    let yy = y * y2;
                    let yz = y * z2;
                    let zz = z * z2;
                    let wx = w * x2;
                    let wy = w * y2;
                    let wz = w * z2;

                    out.0[0] = 1.0 - (yy + zz);
                    out.0[1] = xy + wz;
                    out.0[2] = xz - wy;
                    out.0[3] = 0.0;
                    out.0[4] = xy - wz;
                    out.0[5] = 1.0 - (xx + zz);
                    out.0[6] = yz + wx;
                    out.0[7] = 0.0;
                    out.0[8] = xz + wy;
                    out.0[9] = yz - wx;
                    out.0[10] = 1.0 - (xx + yy);
                    out.0[11] = 0.0;
                    out.0[12] = v.0[0];
                    out.0[13] = v.0[1];
                    out.0[14] = v.0[2];
                    out.0[15] = 1.0;
                    out
                }

                #[inline]
                pub fn from_quat2(a: &Quat2<$t>) -> Self {
                    let mut translation = Vec3::<$t>::new();
                    let bx = -a.0[0];
                    let by = -a.0[1];
                    let bz = -a.0[2];
                    let bw = a.0[3];
                    let ax = a.0[4];
                    let ay = a.0[5];
                    let az = a.0[6];
                    let aw = a.0[7];

                    let magnitude = bx * bx + by * by + bz * bz + bw * bw;
                    //Only scale if it makes sense
                    if magnitude > 0.0 {
                      translation.0[0] = ((ax * bw + aw * bx + ay * bz - az * by) * 2.0) / magnitude;
                      translation.0[1] = ((ay * bw + aw * by + az * bx - ax * bz) * 2.0) / magnitude;
                      translation.0[2] = ((az * bw + aw * bz + ax * by - ay * bx) * 2.0) / magnitude;
                    } else {
                      translation.0[0] = (ax * bw + aw * bx + ay * bz - az * by) * 2.0;
                      translation.0[1] = (ay * bw + aw * by + az * bx - ax * bz) * 2.0;
                      translation.0[2] = (az * bw + aw * bz + ax * by - ay * bx) * 2.0;
                    }
                    Self::from_rotation_translation(&a, &translation)
                }

                #[inline]
                pub fn from_rotation_translation_scale(
                    q: &Quat2<$t>,
                    v: &Vec3<$t>,
                    s: &Vec3<$t>
                ) -> Self {
                    let mut out = Self::new();

                    // Quaternion math
                    let x = q.0[0];
                    let y = q.0[1];
                    let z = q.0[2];
                    let w = q.0[3];
                    let x2 = x + x;
                    let y2 = y + y;
                    let z2 = z + z;

                    let xx = x * x2;
                    let xy = x * y2;
                    let xz = x * z2;
                    let yy = y * y2;
                    let yz = y * z2;
                    let zz = z * z2;
                    let wx = w * x2;
                    let wy = w * y2;
                    let wz = w * z2;
                    let sx = s.0[0];
                    let sy = s.0[1];
                    let sz = s.0[2];

                    out.0[0] = (1.0 - (yy + zz)) * sx;
                    out.0[1] = (xy + wz) * sx;
                    out.0[2] = (xz - wy) * sx;
                    out.0[3] = 0.0;
                    out.0[4] = (xy - wz) * sy;
                    out.0[5] = (1.0 - (xx + zz)) * sy;
                    out.0[6] = (yz + wx) * sy;
                    out.0[7] = 0.0;
                    out.0[8] = (xz + wy) * sz;
                    out.0[9] = (yz - wx) * sz;
                    out.0[10] = (1.0 - (xx + yy)) * sz;
                    out.0[11] = 0.0;
                    out.0[12] = v.0[0];
                    out.0[13] = v.0[1];
                    out.0[14] = v.0[2];
                    out.0[15] = 1.0;
                    out
                }

                #[inline]
                pub fn from_rotation_translation_scale_origin(
                    q: &Quat2<$t>,
                    v: &Vec3<$t>,
                    s: &Vec3<$t>,
                    o: &Vec3<$t>
                ) -> Self {
                    let mut out = Self::new();

                    // Quaternion math
                    let x = q.0[0];
                    let y = q.0[1];
                    let z = q.0[2];
                    let w = q.0[3];
                    let x2 = x + x;
                    let y2 = y + y;
                    let z2 = z + z;

                    let xx = x * x2;
                    let xy = x * y2;
                    let xz = x * z2;
                    let yy = y * y2;
                    let yz = y * z2;
                    let zz = z * z2;
                    let wx = w * x2;
                    let wy = w * y2;
                    let wz = w * z2;

                    let sx = s.0[0];
                    let sy = s.0[1];
                    let sz = s.0[2];

                    let ox = o.0[0];
                    let oy = o.0[1];
                    let oz = o.0[2];

                    let out0 = (1.0 - (yy + zz)) * sx;
                    let out1 = (xy + wz) * sx;
                    let out2 = (xz - wy) * sx;
                    let out4 = (xy - wz) * sy;
                    let out5 = (1.0 - (xx + zz)) * sy;
                    let out6 = (yz + wx) * sy;
                    let out8 = (xz + wy) * sz;
                    let out9 = (yz - wx) * sz;
                    let out10 = (1.0 - (xx + yy)) * sz;

                    out.0[0] = out0;
                    out.0[1] = out1;
                    out.0[2] = out2;
                    out.0[3] = 0.0;
                    out.0[4] = out4;
                    out.0[5] = out5;
                    out.0[6] = out6;
                    out.0[7] = 0.0;
                    out.0[8] = out8;
                    out.0[9] = out9;
                    out.0[10] = out10;
                    out.0[11] = 0.0;
                    out.0[12] = v.0[0] + ox - (out0 * ox + out4 * oy + out8 * oz);
                    out.0[13] = v.0[1] + oy - (out1 * ox + out5 * oy + out9 * oz);
                    out.0[14] = v.0[2] + oz - (out2 * ox + out6 * oy + out10 * oz);
                    out.0[15] = 1.0;
                    out
                }

                #[inline]
                pub fn from_quat(q: &Quat<$t>) -> Self {
                    let mut out = Self::new();

                    // Quaternion math
                    let x = q.0[0];
                    let y = q.0[1];
                    let z = q.0[2];
                    let w = q.0[3];
                    let x2 = x + x;
                    let y2 = y + y;
                    let z2 = z + z;

                    let xx = x * x2;
                    let yx = y * x2;
                    let yy = y * y2;
                    let zx = z * x2;
                    let zy = z * y2;
                    let zz = z * z2;
                    let wx = w * x2;
                    let wy = w * y2;
                    let wz = w * z2;

                    out.0[0] = 1.0 - yy - zz;
                    out.0[1] = yx + wz;
                    out.0[2] = zx - wy;
                    out.0[3] = 0.0;

                    out.0[4] = yx - wz;
                    out.0[5] = 1.0 - xx - zz;
                    out.0[6] = zy + wx;
                    out.0[7] = 0.0;

                    out.0[8] = zx + wy;
                    out.0[9] = zy - wx;
                    out.0[10] = 1.0 - xx - yy;
                    out.0[11] = 0.0;

                    out.0[12] = 0.0;
                    out.0[13] = 0.0;
                    out.0[14] = 0.0;
                    out.0[15] = 1.0;

                    out
                }

                #[inline]
                pub fn from_frustum(
                    left: $t,
                    right: $t,
                    bottom: $t,
                    top: $t,
                    near: $t,
                    far: $t
                ) -> Self {
                    let mut out = Self::new();

                    let rl = 1.0 / (right - left);
                    let tb = 1.0 / (top - bottom);
                    let nf = 1.0 / (near - far);
                    out.0[0] = near * 2.0 * rl;
                    out.0[1] = 0.0;
                    out.0[2] = 0.0;
                    out.0[3] = 0.0;
                    out.0[4] = 0.0;
                    out.0[5] = near * 2.0 * tb;
                    out.0[6] = 0.0;
                    out.0[7] = 0.0;
                    out.0[8] = (right + left) * rl;
                    out.0[9] = (top + bottom) * tb;
                    out.0[10] = (far + near) * nf;
                    out.0[11] = -1.0;
                    out.0[12] = 0.0;
                    out.0[13] = 0.0;
                    out.0[14] = far * near * 2.0 * nf;
                    out.0[15] = 0.0;
                    out
                }

                #[inline]
                pub fn from_perspective_no(
                    fovy: $t,
                    aspect: $t,
                    near: $t,
                    far: Option<$t>
                ) -> Self {
                    let mut out = Self::new();

                    let f = 1.0 / (fovy / 2.0).tan();
                    out.0[0] = f / aspect;
                    out.0[1] = 0.0;
                    out.0[2] = 0.0;
                    out.0[3] = 0.0;
                    out.0[4] = 0.0;
                    out.0[5] = f;
                    out.0[6] = 0.0;
                    out.0[7] = 0.0;
                    out.0[8] = 0.0;
                    out.0[9] = 0.0;
                    out.0[11] = -1.0;
                    out.0[12] = 0.0;
                    out.0[13] = 0.0;
                    out.0[15] = 0.0;

                    match far {
                        Some(far) => {
                            if far.is_infinite() {
                                out.0[10] = -1.0;
                                out.0[14] = -2.0 * near;
                                out
                            } else {
                                let nf = 1.0 / (near - far);
                                out.0[10] = (far + near) * nf;
                                out.0[14] = 2.0 * far * near * nf;
                                out
                            }
                        }
                        None => {
                            out.0[10] = -1.0;
                            out.0[14] = -2.0 * near;
                            out
                        }
                    }
                }

                #[inline]
                pub fn from_perspective(
                    fovy: $t,
                    aspect: $t,
                    near: $t,
                    far: Option<$t>
                ) -> Self {
                    Self::from_perspective_no(fovy, aspect, near, far)
                }

                #[inline]
                pub fn from_perspective_zo(
                    fovy: $t,
                    aspect: $t,
                    near: $t,
                    far: Option<$t>
                ) -> Self {
                    let mut out = Self::new();

                    let f = 1.0 / (fovy / 2.0).tan();
                    out.0[0] = f / aspect;
                    out.0[1] = 0.0;
                    out.0[2] = 0.0;
                    out.0[3] = 0.0;
                    out.0[4] = 0.0;
                    out.0[5] = f;
                    out.0[6] = 0.0;
                    out.0[7] = 0.0;
                    out.0[8] = 0.0;
                    out.0[9] = 0.0;
                    out.0[11] = -1.0;
                    out.0[12] = 0.0;
                    out.0[13] = 0.0;
                    out.0[15] = 0.0;

                    match far {
                        Some(far) => {
                            if far.is_infinite() {
                                out.0[10] = -1.0;
                                out.0[14] = -near;
                                out
                            } else {
                                let nf = 1.0 / (near - far);
                                out.0[10] = far * nf;
                                out.0[14] = far * near * nf;
                                out
                            }
                        }
                        None => {
                            out.0[10] = -1.0;
                            out.0[14] = -near;
                            out
                        }
                    }
                }

                #[inline]
                pub fn from_perspective_from_field_of_view(
                    fov_left_degrees: $t,
                    fov_right_degrees: $t,
                    fov_down_degrees: $t,
                    fov_up_degrees: $t,
                    near: $t,
                    far: $t
                ) -> Self {
                    let mut out = Self::new();

                    let up_tan = ((fov_up_degrees * $pi) / 180.0).tan();
                    let down_tan = ((fov_down_degrees * $pi) / 180.0).tan();
                    let left_tan = ((fov_left_degrees * $pi) / 180.0).tan();
                    let right_tan = ((fov_right_degrees * $pi) / 180.0).tan();
                    let x_scale = 2.0 / (left_tan + right_tan);
                    let y_scale = 2.0 / (up_tan + down_tan);

                    out.0[0] = x_scale;
                    out.0[1] = 0.0;
                    out.0[2] = 0.0;
                    out.0[3] = 0.0;
                    out.0[4] = 0.0;
                    out.0[5] = y_scale;
                    out.0[6] = 0.0;
                    out.0[7] = 0.0;
                    out.0[8] = -((left_tan - right_tan) * x_scale * 0.5);
                    out.0[9] = (up_tan - down_tan) * y_scale * 0.5;
                    out.0[10] = far / (near - far);
                    out.0[11] = -1.0;
                    out.0[12] = 0.0;
                    out.0[13] = 0.0;
                    out.0[14] = (far * near) / (near - far);
                    out.0[15] = 0.0;
                    out
                }

                #[inline]
                pub fn from_ortho_no(
                    left: $t,
                    right: $t,
                    bottom: $t,
                    top: $t,
                    near: $t,
                    far: $t
                ) -> Self {
                    let mut out = Self::new();

                    let lr = 1.0 / (left - right);
                    let bt = 1.0 / (bottom - top);
                    let nf = 1.0 / (near - far);
                    out.0[0] = -2.0 * lr;
                    out.0[1] = 0.0;
                    out.0[2] = 0.0;
                    out.0[3] = 0.0;
                    out.0[4] = 0.0;
                    out.0[5] = -2.0 * bt;
                    out.0[6] = 0.0;
                    out.0[7] = 0.0;
                    out.0[8] = 0.0;
                    out.0[9] = 0.0;
                    out.0[10] = 2.0 * nf;
                    out.0[11] = 0.0;
                    out.0[12] = (left + right) * lr;
                    out.0[13] = (top + bottom) * bt;
                    out.0[14] = (far + near) * nf;
                    out.0[15] = 1.0;
                    out
                }

                #[inline]
                pub fn from_ortho(
                    left: $t,
                    right: $t,
                    bottom: $t,
                    top: $t,
                    near: $t,
                    far: $t
                ) -> Self {
                    Self::from_ortho_no(left, right, bottom, top, near, far)
                }

                #[inline]
                pub fn from_ortho_zo(
                    left: $t,
                    right: $t,
                    bottom: $t,
                    top: $t,
                    near: $t,
                    far: $t
                ) -> Self {
                    let mut out = Self::new();

                    let lr = 1.0 / (left - right);
                    let bt = 1.0 / (bottom - top);
                    let nf = 1.0 / (near - far);
                    out.0[0] = -2.0 * lr;
                    out.0[1] = 0.0;
                    out.0[2] = 0.0;
                    out.0[3] = 0.0;
                    out.0[4] = 0.0;
                    out.0[5] = -2.0 * bt;
                    out.0[6] = 0.0;
                    out.0[7] = 0.0;
                    out.0[8] = 0.0;
                    out.0[9] = 0.0;
                    out.0[10] = nf;
                    out.0[11] = 0.0;
                    out.0[12] = (left + right) * lr;
                    out.0[13] = (top + bottom) * bt;
                    out.0[14] = near * nf;
                    out.0[15] = 1.0;
                    out
                }

                #[inline]
                pub fn from_look_at(
                    eye: &Vec3<$t>,
                    center: &Vec3<$t>,
                    up: &Vec3<$t>
                ) -> Self {
                    let eye_x = eye.0[0];
                    let eye_y = eye.0[1];
                    let eye_z = eye.0[2];
                    let up_x = up.0[0];
                    let up_y = up.0[1];
                    let up_z = up.0[2];
                    let center_x = center.0[0];
                    let center_y = center.0[1];
                    let center_z = center.0[2];

                    if (
                      (eye_x - center_x).abs() < $epsilon &&
                      (eye_y - center_y).abs() < $epsilon &&
                      (eye_z - center_z).abs() < $epsilon
                    ) {
                      return Self::new();
                    }

                    let mut out = Self::new();

                    let mut z0 = eye_x - center_x;
                    let mut z1 = eye_y - center_y;
                    let mut z2 = eye_z - center_z;

                    let mut len = 1.0 / (z0 * z0 + z1 * z1 + z2 * z2).sqrt();
                    z0 *= len;
                    z1 *= len;
                    z2 *= len;

                    let mut x0 = up_y * z2 - up_z * z1;
                    let mut x1 = up_z * z0 - up_x * z2;
                    let mut x2 = up_x * z1 - up_y * z0;
                    len = (x0 * x0 + x1 * x1 + x2 * x2).sqrt();
                    if (len == 0.0) {
                      x0 = 0.0;
                      x1 = 0.0;
                      x2 = 0.0;
                    } else {
                      len = 1.0 / len;
                      x0 *= len;
                      x1 *= len;
                      x2 *= len;
                    }

                    let mut y0 = z1 * x2 - z2 * x1;
                    let mut y1 = z2 * x0 - z0 * x2;
                    let mut y2 = z0 * x1 - z1 * x0;

                    len = (y0 * y0 + y1 * y1 + y2 * y2).sqrt();
                    if (len == 0.0) {
                      y0 = 0.0;
                      y1 = 0.0;
                      y2 = 0.0;
                    } else {
                      len = 1.0 / len;
                      y0 *= len;
                      y1 *= len;
                      y2 *= len;
                    }

                    out.0[0] = x0;
                    out.0[1] = y0;
                    out.0[2] = z0;
                    out.0[3] = 0.0;
                    out.0[4] = x1;
                    out.0[5] = y1;
                    out.0[6] = z1;
                    out.0[7] = 0.0;
                    out.0[8] = x2;
                    out.0[9] = y2;
                    out.0[10] = z2;
                    out.0[11] = 0.0;
                    out.0[12] = -(x0 * eye_x + x1 * eye_y + x2 * eye_z);
                    out.0[13] = -(y0 * eye_x + y1 * eye_y + y2 * eye_z);
                    out.0[14] = -(z0 * eye_x + z1 * eye_y + z2 * eye_z);
                    out.0[15] = 1.0;

                    out
                }

                #[inline]
                pub fn from_target_to(
                    eye: &Vec3<$t>,
                    target: &Vec3<$t>,
                    up: &Vec3<$t>
                ) -> Self {
                    let mut out = Self::new();

                    let eye_x = eye.0[0];
                    let eye_y = eye.0[1];
                    let eye_z = eye.0[2];
                    let up_x = up.0[0];
                    let up_y = up.0[1];
                    let up_z = up.0[2];

                    let mut z0 = eye_x - target.0[0];
                    let mut z1 = eye_y - target.0[1];
                    let mut z2 = eye_z - target.0[2];

                    let mut len = z0 * z0 + z1 * z1 + z2 * z2;
                    if len > 0.0 {
                      len = 1.0 / len.sqrt();
                      z0 *= len;
                      z1 *= len;
                      z2 *= len;
                    }

                    let mut x0 = up_y * z2 - up_z * z1;
                    let mut x1 = up_z * z0 - up_x * z2;
                    let mut x2 = up_x * z1 - up_y * z0;

                    len = x0 * x0 + x1 * x1 + x2 * x2;
                    if (len > 0.0) {
                      len = 1.0 / len.sqrt();
                      x0 *= len;
                      x1 *= len;
                      x2 *= len;
                    }

                    out.0[0] = x0;
                    out.0[1] = x1;
                    out.0[2] = x2;
                    out.0[3] = 0.0;
                    out.0[4] = z1 * x2 - z2 * x1;
                    out.0[5] = z2 * x0 - z0 * x2;
                    out.0[6] = z0 * x1 - z1 * x0;
                    out.0[7] = 0.0;
                    out.0[8] = z0;
                    out.0[9] = z1;
                    out.0[10] = z2;
                    out.0[11] = 0.0;
                    out.0[12] = eye_x;
                    out.0[13] = eye_y;
                    out.0[14] = eye_z;
                    out.0[15] = 1.0;

                    out
                }
            }

            impl Mat4<$t> {
                #[inline]
                pub fn raw(&self) -> &[$t; 16] {
                    &self.0
                }

                #[inline]
                pub fn set(
                    &mut self,
                    m00: $t,
                    m01: $t,
                    m02: $t,
                    m03: $t,
                    m10: $t,
                    m11: $t,
                    m12: $t,
                    m13: $t,
                    m20: $t,
                    m21: $t,
                    m22: $t,
                    m23: $t,
                    m30: $t,
                    m31: $t,
                    m32: $t,
                    m33: $t,
                ) -> &mut Self {
                    self.0[0] = m00;
                    self.0[1] = m01;
                    self.0[2] = m02;
                    self.0[3] = m03;
                    self.0[4] = m10;
                    self.0[5] = m11;
                    self.0[6] = m12;
                    self.0[7] = m13;
                    self.0[8] = m20;
                    self.0[9] = m21;
                    self.0[10] = m22;
                    self.0[11] = m23;
                    self.0[12] = m30;
                    self.0[13] = m31;
                    self.0[14] = m32;
                    self.0[15] = m33;

                    self
                }

                #[inline]
                pub fn set_slice(
                    &mut self,
                    slice: &[$t; 16]
                ) -> &mut Self {
                    self.0 = slice.clone();
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
                    self.0[8] = 0.0;
                    self.0[9] = 0.0;
                    self.0[10] = 0.0;
                    self.0[11] = 0.0;
                    self.0[12] = 0.0;
                    self.0[13] = 0.0;
                    self.0[14] = 0.0;
                    self.0[15] = 0.0;
                    self
                }

                #[inline]
                pub fn set_identify(&mut self) -> &mut Self {
                    self.0[0] = 1.0;
                    self.0[1] = 0.0;
                    self.0[2] = 0.0;
                    self.0[3] = 0.0;
                    self.0[4] = 0.0;
                    self.0[5] = 1.0;
                    self.0[6] = 0.0;
                    self.0[7] = 0.0;
                    self.0[8] = 0.0;
                    self.0[9] = 0.0;
                    self.0[10] = 1.0;
                    self.0[11] = 0.0;
                    self.0[12] = 0.0;
                    self.0[13] = 0.0;
                    self.0[14] = 0.0;
                    self.0[15] = 1.0;
                    self
                }

                #[inline]
                pub fn transpose(&self) -> Self {
                    let mut out = Self::new();
                    out.0[0] = self.0[0];
                    out.0[1] = self.0[4];
                    out.0[2] = self.0[8];
                    out.0[3] = self.0[12];
                    out.0[4] = self.0[1];
                    out.0[5] = self.0[5];
                    out.0[6] = self.0[9];
                    out.0[7] = self.0[13];
                    out.0[8] = self.0[2];
                    out.0[9] = self.0[6];
                    out.0[10] = self.0[10];
                    out.0[11] = self.0[14];
                    out.0[12] = self.0[3];
                    out.0[13] = self.0[7];
                    out.0[14] = self.0[11];
                    out.0[15] = self.0[15];
                    out
                }

                #[inline]
                pub fn invert(&self) -> Result<Self, Error> {
                    let mut out = Self::new();

                    let a00 = self.0[0];
                    let a01 = self.0[1];
                    let a02 = self.0[2];
                    let a03 = self.0[3];
                    let a10 = self.0[4];
                    let a11 = self.0[5];
                    let a12 = self.0[6];
                    let a13 = self.0[7];
                    let a20 = self.0[8];
                    let a21 = self.0[9];
                    let a22 = self.0[10];
                    let a23 = self.0[11];
                    let a30 = self.0[12];
                    let a31 = self.0[13];
                    let a32 = self.0[14];
                    let a33 = self.0[15];

                    let b00 = a00 * a11 - a01 * a10;
                    let b01 = a00 * a12 - a02 * a10;
                    let b02 = a00 * a13 - a03 * a10;
                    let b03 = a01 * a12 - a02 * a11;
                    let b04 = a01 * a13 - a03 * a11;
                    let b05 = a02 * a13 - a03 * a12;
                    let b06 = a20 * a31 - a21 * a30;
                    let b07 = a20 * a32 - a22 * a30;
                    let b08 = a20 * a33 - a23 * a30;
                    let b09 = a21 * a32 - a22 * a31;
                    let b10 = a21 * a33 - a23 * a31;
                    let b11 = a22 * a33 - a23 * a32;

                    // Calculate the determinant
                    let mut det = b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;

                    if (det == 0.0) {
                        return Err(Error::ZeroDeterminant);
                    }
                    det = 1.0 / det;

                    out.0[0] = (a11 * b11 - a12 * b10 + a13 * b09) * det;
                    out.0[1] = (a02 * b10 - a01 * b11 - a03 * b09) * det;
                    out.0[2] = (a31 * b05 - a32 * b04 + a33 * b03) * det;
                    out.0[3] = (a22 * b04 - a21 * b05 - a23 * b03) * det;
                    out.0[4] = (a12 * b08 - a10 * b11 - a13 * b07) * det;
                    out.0[5] = (a00 * b11 - a02 * b08 + a03 * b07) * det;
                    out.0[6] = (a32 * b02 - a30 * b05 - a33 * b01) * det;
                    out.0[7] = (a20 * b05 - a22 * b02 + a23 * b01) * det;
                    out.0[8] = (a10 * b10 - a11 * b08 + a13 * b06) * det;
                    out.0[9] = (a01 * b08 - a00 * b10 - a03 * b06) * det;
                    out.0[10] = (a30 * b04 - a31 * b02 + a33 * b00) * det;
                    out.0[11] = (a21 * b02 - a20 * b04 - a23 * b00) * det;
                    out.0[12] = (a11 * b07 - a10 * b09 - a12 * b06) * det;
                    out.0[13] = (a00 * b09 - a01 * b07 + a02 * b06) * det;
                    out.0[14] = (a31 * b01 - a30 * b03 - a32 * b00) * det;
                    out.0[15] = (a20 * b03 - a21 * b01 + a22 * b00) * det;

                    Ok(out)
                }

                #[inline]
                pub fn adjoint(&self) -> Self {
                    let mut out = Self::new();

                    let a00 = self.0[0];
                    let a01 = self.0[1];
                    let a02 = self.0[2];
                    let a03 = self.0[3];
                    let a10 = self.0[4];
                    let a11 = self.0[5];
                    let a12 = self.0[6];
                    let a13 = self.0[7];
                    let a20 = self.0[8];
                    let a21 = self.0[9];
                    let a22 = self.0[10];
                    let a23 = self.0[11];
                    let a30 = self.0[12];
                    let a31 = self.0[13];
                    let a32 = self.0[14];
                    let a33 = self.0[15];

                    let b00 = a00 * a11 - a01 * a10;
                    let b01 = a00 * a12 - a02 * a10;
                    let b02 = a00 * a13 - a03 * a10;
                    let b03 = a01 * a12 - a02 * a11;
                    let b04 = a01 * a13 - a03 * a11;
                    let b05 = a02 * a13 - a03 * a12;
                    let b06 = a20 * a31 - a21 * a30;
                    let b07 = a20 * a32 - a22 * a30;
                    let b08 = a20 * a33 - a23 * a30;
                    let b09 = a21 * a32 - a22 * a31;
                    let b10 = a21 * a33 - a23 * a31;
                    let b11 = a22 * a33 - a23 * a32;

                    out.0[0] = a11 * b11 - a12 * b10 + a13 * b09;
                    out.0[1] = a02 * b10 - a01 * b11 - a03 * b09;
                    out.0[2] = a31 * b05 - a32 * b04 + a33 * b03;
                    out.0[3] = a22 * b04 - a21 * b05 - a23 * b03;
                    out.0[4] = a12 * b08 - a10 * b11 - a13 * b07;
                    out.0[5] = a00 * b11 - a02 * b08 + a03 * b07;
                    out.0[6] = a32 * b02 - a30 * b05 - a33 * b01;
                    out.0[7] = a20 * b05 - a22 * b02 + a23 * b01;
                    out.0[8] = a10 * b10 - a11 * b08 + a13 * b06;
                    out.0[9] = a01 * b08 - a00 * b10 - a03 * b06;
                    out.0[10] = a30 * b04 - a31 * b02 + a33 * b00;
                    out.0[11] = a21 * b02 - a20 * b04 - a23 * b00;
                    out.0[12] = a11 * b07 - a10 * b09 - a12 * b06;
                    out.0[13] = a00 * b09 - a01 * b07 + a02 * b06;
                    out.0[14] = a31 * b01 - a30 * b03 - a32 * b00;
                    out.0[15] = a20 * b03 - a21 * b01 + a22 * b00;

                    out
                }

                #[inline]
                pub fn determinant(&self) -> $t {
                    let a00 = self.0[0];
                    let a01 = self.0[1];
                    let a02 = self.0[2];
                    let a03 = self.0[3];
                    let a10 = self.0[4];
                    let a11 = self.0[5];
                    let a12 = self.0[6];
                    let a13 = self.0[7];
                    let a20 = self.0[8];
                    let a21 = self.0[9];
                    let a22 = self.0[10];
                    let a23 = self.0[11];
                    let a30 = self.0[12];
                    let a31 = self.0[13];
                    let a32 = self.0[14];
                    let a33 = self.0[15];

                    let b0 = a00 * a11 - a01 * a10;
                    let b1 = a00 * a12 - a02 * a10;
                    let b2 = a01 * a12 - a02 * a11;
                    let b3 = a20 * a31 - a21 * a30;
                    let b4 = a20 * a32 - a22 * a30;
                    let b5 = a21 * a32 - a22 * a31;
                    let b6 = a00 * b5 - a01 * b4 + a02 * b3;
                    let b7 = a10 * b5 - a11 * b4 + a12 * b3;
                    let b8 = a20 * b2 - a21 * b1 + a22 * b0;
                    let b9 = a30 * b2 - a31 * b1 + a32 * b0;

                    // Calculate the determinant
                    a13 * b6 - a03 * b7 + a33 * b8 - a23 * b9
                }

                #[inline]
                pub fn translate(&self, v: &Vec3<$t>) -> Self {
                    let mut out = Self::new();

                    let x = v.0[0];
                    let y = v.0[1];
                    let z = v.0[2];
                    let a00;
                    let a01;
                    let a02;
                    let a03;
                    let a10;
                    let a11;
                    let a12;
                    let a13;
                    let a20;
                    let a21;
                    let a22;
                    let a23;

                    a00 = self.0[0];
                    a01 = self.0[1];
                    a02 = self.0[2];
                    a03 = self.0[3];
                    a10 = self.0[4];
                    a11 = self.0[5];
                    a12 = self.0[6];
                    a13 = self.0[7];
                    a20 = self.0[8];
                    a21 = self.0[9];
                    a22 = self.0[10];
                    a23 = self.0[11];

                    out.0[0] = a00;
                    out.0[1] = a01;
                    out.0[2] = a02;
                    out.0[3] = a03;
                    out.0[4] = a10;
                    out.0[5] = a11;
                    out.0[6] = a12;
                    out.0[7] = a13;
                    out.0[8] = a20;
                    out.0[9] = a21;
                    out.0[10] = a22;
                    out.0[11] = a23;

                    out.0[12] = a00 * x + a10 * y + a20 * z + self.0[12];
                    out.0[13] = a01 * x + a11 * y + a21 * z + self.0[13];
                    out.0[14] = a02 * x + a12 * y + a22 * z + self.0[14];
                    out.0[15] = a03 * x + a13 * y + a23 * z + self.0[15];


                    out
                }

                #[inline]
                pub fn scale(&self, v: &Vec3<$t>) -> Self {
                    let mut out = Self::new();

                    let x = v.0[0];
                    let y = v.0[1];
                    let z = v.0[2];

                    out.0[0] = self.0[0] * x;
                    out.0[1] = self.0[1] * x;
                    out.0[2] = self.0[2] * x;
                    out.0[3] = self.0[3] * x;
                    out.0[4] = self.0[4] * y;
                    out.0[5] = self.0[5] * y;
                    out.0[6] = self.0[6] * y;
                    out.0[7] = self.0[7] * y;
                    out.0[8] = self.0[8] * z;
                    out.0[9] = self.0[9] * z;
                    out.0[10] = self.0[10] * z;
                    out.0[11] = self.0[11] * z;
                    out.0[12] = self.0[12];
                    out.0[13] = self.0[13];
                    out.0[14] = self.0[14];
                    out.0[15] = self.0[15];
                    out
                }

                #[inline]
                pub fn rotate(&self, axis: &Vec3<$t>, rad: $t) -> Result<Self, Error> {
                    let mut out = Self::new();

                    let mut x = axis.0[0];
                    let mut y = axis.0[1];
                    let mut z = axis.0[2];
                    let mut len = (x * x + y * y + z * z).sqrt();
                    let s;
                    let c;
                    let t;
                    let a00;
                    let a01;
                    let a02;
                    let a03;
                    let a10;
                    let a11;
                    let a12;
                    let a13;
                    let a20;
                    let a21;
                    let a22;
                    let a23;
                    let b00;
                    let b01;
                    let b02;
                    let b10;
                    let b11;
                    let b12;
                    let b20;
                    let b21;
                    let b22;

                    if len < $epsilon {
                      return Err(Error::LengthSmallerThanEpsilon);
                    }

                    len = 1.0 / len;
                    x *= len;
                    y *= len;
                    z *= len;

                    s = rad.sin();
                    c = rad.cos();
                    t = 1.0 - c;

                    a00 = self.0[0];
                    a01 = self.0[1];
                    a02 = self.0[2];
                    a03 = self.0[3];
                    a10 = self.0[4];
                    a11 = self.0[5];
                    a12 = self.0[6];
                    a13 = self.0[7];
                    a20 = self.0[8];
                    a21 = self.0[9];
                    a22 = self.0[10];
                    a23 = self.0[11];

                    // Construct the elements of the rotation matrix
                    b00 = x * x * t + c;
                    b01 = y * x * t + z * s;
                    b02 = z * x * t - y * s;
                    b10 = x * y * t - z * s;
                    b11 = y * y * t + c;
                    b12 = z * y * t + x * s;
                    b20 = x * z * t + y * s;
                    b21 = y * z * t - x * s;
                    b22 = z * z * t + c;

                    // Perform rotation-specific matrix multiplication
                    out.0[0] = a00 * b00 + a10 * b01 + a20 * b02;
                    out.0[1] = a01 * b00 + a11 * b01 + a21 * b02;
                    out.0[2] = a02 * b00 + a12 * b01 + a22 * b02;
                    out.0[3] = a03 * b00 + a13 * b01 + a23 * b02;
                    out.0[4] = a00 * b10 + a10 * b11 + a20 * b12;
                    out.0[5] = a01 * b10 + a11 * b11 + a21 * b12;
                    out.0[6] = a02 * b10 + a12 * b11 + a22 * b12;
                    out.0[7] = a03 * b10 + a13 * b11 + a23 * b12;
                    out.0[8] = a00 * b20 + a10 * b21 + a20 * b22;
                    out.0[9] = a01 * b20 + a11 * b21 + a21 * b22;
                    out.0[10] = a02 * b20 + a12 * b21 + a22 * b22;
                    out.0[11] = a03 * b20 + a13 * b21 + a23 * b22;

                    // If the source and destination differ, copy the unchanged last row
                    out.0[12] = self.0[12];
                    out.0[13] = self.0[13];
                    out.0[14] = self.0[14];
                    out.0[15] = self.0[15];

                    Ok(out)
                }

                #[inline]
                pub fn rotate_x(&self, rad: $t) -> Self {
                    let mut out = Self::new();

                    let s = rad.sin();
                    let c = rad.cos();
                    let a10 = self.0[4];
                    let a11 = self.0[5];
                    let a12 = self.0[6];
                    let a13 = self.0[7];
                    let a20 = self.0[8];
                    let a21 = self.0[9];
                    let a22 = self.0[10];
                    let a23 = self.0[11];

                    out.0[0] = self.0[0];
                    out.0[1] = self.0[1];
                    out.0[2] = self.0[2];
                    out.0[3] = self.0[3];
                    out.0[12] = self.0[12];
                    out.0[13] = self.0[13];
                    out.0[14] = self.0[14];
                    out.0[15] = self.0[15];

                    // Perform axis-specific matrix multiplication
                    out.0[4] = a10 * c + a20 * s;
                    out.0[5] = a11 * c + a21 * s;
                    out.0[6] = a12 * c + a22 * s;
                    out.0[7] = a13 * c + a23 * s;
                    out.0[8] = a20 * c - a10 * s;
                    out.0[9] = a21 * c - a11 * s;
                    out.0[10] = a22 * c - a12 * s;
                    out.0[11] = a23 * c - a13 * s;
                    out
                }

                #[inline]
                pub fn rotate_y(&self, rad: $t) -> Self {
                    let mut out = Self::new();

                    let s = rad.sin();
                    let c = rad.cos();
                    let a00 = self.0[0];
                    let a01 = self.0[1];
                    let a02 = self.0[2];
                    let a03 = self.0[3];
                    let a20 = self.0[8];
                    let a21 = self.0[9];
                    let a22 = self.0[10];
                    let a23 = self.0[11];

                    out.0[4] = self.0[4];
                    out.0[5] = self.0[5];
                    out.0[6] = self.0[6];
                    out.0[7] = self.0[7];
                    out.0[12] = self.0[12];
                    out.0[13] = self.0[13];
                    out.0[14] = self.0[14];
                    out.0[15] = self.0[15];


                    // Perform axis-specific matrix multiplication
                    out.0[0] = a00 * c - a20 * s;
                    out.0[1] = a01 * c - a21 * s;
                    out.0[2] = a02 * c - a22 * s;
                    out.0[3] = a03 * c - a23 * s;
                    out.0[8] = a00 * s + a20 * c;
                    out.0[9] = a01 * s + a21 * c;
                    out.0[10] = a02 * s + a22 * c;
                    out.0[11] = a03 * s + a23 * c;
                    out
                }

                #[inline]
                pub fn rotate_z(&self, rad: $t) -> Self {
                    let mut out = Self::new();

                    let s = rad.sin();
                    let c = rad.cos();
                    let a00 = self.0[0];
                    let a01 = self.0[1];
                    let a02 = self.0[2];
                    let a03 = self.0[3];
                    let a10 = self.0[4];
                    let a11 = self.0[5];
                    let a12 = self.0[6];
                    let a13 = self.0[7];

                    out.0[8] = self.0[8];
                    out.0[9] = self.0[9];
                    out.0[10] = self.0[10];
                    out.0[11] = self.0[11];
                    out.0[12] = self.0[12];
                    out.0[13] = self.0[13];
                    out.0[14] = self.0[14];
                    out.0[15] = self.0[15];

                    // Perform axis-specific matrix multiplication
                    out.0[0] = a00 * c + a10 * s;
                    out.0[1] = a01 * c + a11 * s;
                    out.0[2] = a02 * c + a12 * s;
                    out.0[3] = a03 * c + a13 * s;
                    out.0[4] = a10 * c - a00 * s;
                    out.0[5] = a11 * c - a01 * s;
                    out.0[6] = a12 * c - a02 * s;
                    out.0[7] = a13 * c - a03 * s;
                    out
                }

                #[inline]
                pub fn decompose(&self) -> (Quat<$t>, Vec3<$t>, Vec3<$t>) {
                    let mut out_r = Quat::<$t>::new();
                    let mut out_t = Vec3::<$t>::new();
                    let mut out_s = Vec3::<$t>::new();

                    out_t.0[0] = self.0[12];
                    out_t.0[1] = self.0[13];
                    out_t.0[2] = self.0[14];

                    let m11 = self.0[0];
                    let m12 = self.0[1];
                    let m13 = self.0[2];
                    let m21 = self.0[4];
                    let m22 = self.0[5];
                    let m23 = self.0[6];
                    let m31 = self.0[8];
                    let m32 = self.0[9];
                    let m33 = self.0[10];

                    out_s.0[0] = (m11 * m11 + m12 * m12 + m13 * m13).sqrt();
                    out_s.0[1] = (m21 * m21 + m22 * m22 + m23 * m23).sqrt();
                    out_s.0[2] = (m31 * m31 + m32 * m32 + m33 * m33).sqrt();

                    let is1 = 1.0 / out_s.0[0];
                    let is2 = 1.0 / out_s.0[1];
                    let is3 = 1.0 / out_s.0[2];

                    let sm11 = m11 * is1;
                    let sm12 = m12 * is2;
                    let sm13 = m13 * is3;
                    let sm21 = m21 * is1;
                    let sm22 = m22 * is2;
                    let sm23 = m23 * is3;
                    let sm31 = m31 * is1;
                    let sm32 = m32 * is2;
                    let sm33 = m33 * is3;

                    let trace = sm11 + sm22 + sm33;

                    if trace > 0.0 {
                        let s = (trace + 1.0).sqrt() * 2.0;
                        out_r.0[3] = 0.25 * s;
                        out_r.0[0] = (sm23 - sm32) / s;
                        out_r.0[1] = (sm31 - sm13) / s;
                        out_r.0[2] = (sm12 - sm21) / s;
                    } else if sm11 > sm22 && sm11 > sm33 {
                        let s = (1.0 + sm11 - sm22 - sm33).sqrt() * 2.0;
                        out_r.0[3] = (sm23 - sm32) / s;
                        out_r.0[0] = 0.25 * s;
                        out_r.0[1] = (sm12 + sm21) / s;
                        out_r.0[2] = (sm31 + sm13) / s;
                    } else if sm22 > sm33 {
                        let s = (1.0 + sm22 - sm11 - sm33).sqrt() * 2.0;
                        out_r.0[3] = (sm31 - sm13) / s;
                        out_r.0[0] = (sm12 + sm21) / s;
                        out_r.0[1] = 0.25 * s;
                        out_r.0[2] = (sm23 + sm32) / s;
                    } else {
                        let s = (1.0 + sm33 - sm11 - sm22).sqrt() * 2.0;
                        out_r.0[3] = (sm12 - sm21) / s;
                        out_r.0[0] = (sm31 + sm13) / s;
                        out_r.0[1] = (sm23 + sm32) / s;
                        out_r.0[2] = 0.25 * s;
                    }

                    (out_r, out_t, out_s)
                }

                #[inline]
                pub fn frob(&self) -> $t {
                    (
                        self.0[0] * self.0[0] +
                        self.0[1] * self.0[1] +
                        self.0[2] * self.0[2] +
                        self.0[3] * self.0[3] +
                        self.0[4] * self.0[4] +
                        self.0[5] * self.0[5] +
                        self.0[6] * self.0[6] +
                        self.0[7] * self.0[7] +
                        self.0[8] * self.0[8] +
                        self.0[9] * self.0[9] +
                        self.0[10] * self.0[10] +
                        self.0[11] * self.0[11] +
                        self.0[12] * self.0[12] +
                        self.0[13] * self.0[13] +
                        self.0[14] * self.0[14] +
                        self.0[15] * self.0[15]
                    ).sqrt()
                }

                #[inline]
                pub fn translation(&self) -> Vec3<$t> {
                    let mut out = Vec3::<$t>::new();

                    out.0[0] = self.0[12];
                    out.0[1] = self.0[13];
                    out.0[2] = self.0[14];

                    out
                }

                #[inline]
                pub fn scaling(&self) -> Vec3<$t> {
                    let mut out = Vec3::<$t>::new();

                    let m11 = self.0[0];
                    let m12 = self.0[1];
                    let m13 = self.0[2];
                    let m21 = self.0[4];
                    let m22 = self.0[5];
                    let m23 = self.0[6];
                    let m31 = self.0[8];
                    let m32 = self.0[9];
                    let m33 = self.0[10];

                    out.0[0] = (m11 * m11 + m12 * m12 + m13 * m13).sqrt();
                    out.0[1] = (m21 * m21 + m22 * m22 + m23 * m23).sqrt();
                    out.0[2] = (m31 * m31 + m32 * m32 + m33 * m33).sqrt();

                    out
                }

                #[inline]
                pub fn rotation(&self)  -> Quat<$t> {
                    let mut out = Quat::<$t>::new();

                    let scaling = self.scaling();

                    let is1 = 1.0 / scaling.0[0];
                    let is2 = 1.0 / scaling.0[1];
                    let is3 = 1.0 / scaling.0[2];

                    let sm11 = self.0[0] * is1;
                    let sm12 = self.0[1] * is2;
                    let sm13 = self.0[2] * is3;
                    let sm21 = self.0[4] * is1;
                    let sm22 = self.0[5] * is2;
                    let sm23 = self.0[6] * is3;
                    let sm31 = self.0[8] * is1;
                    let sm32 = self.0[9] * is2;
                    let sm33 = self.0[10] * is3;

                    let trace = sm11 + sm22 + sm33;

                    if trace > 0.0 {
                        let s = (trace + 1.0).sqrt() * 2.0;
                        out.0[3] = 0.25 * s;
                        out.0[0] = (sm23 - sm32) / s;
                        out.0[1] = (sm31 - sm13) / s;
                        out.0[2] = (sm12 - sm21) / s;
                    } else if sm11 > sm22 && sm11 > sm33 {
                        let s = (1.0 + sm11 - sm22 - sm33).sqrt() * 2.0;
                        out.0[3] = (sm23 - sm32) / s;
                        out.0[0] = 0.25 * s;
                        out.0[1] = (sm12 + sm21) / s;
                        out.0[2] = (sm31 + sm13) / s;
                    } else if sm22 > sm33 {
                        let s = (1.0 + sm22 - sm11 - sm33).sqrt() * 2.0;
                        out.0[3] = (sm31 - sm13) / s;
                        out.0[0] = (sm12 + sm21) / s;
                        out.0[1] = 0.25 * s;
                        out.0[2] = (sm23 + sm32) / s;
                    } else {
                        let s = (1.0 + sm33 - sm11 - sm22).sqrt() * 2.0;
                        out.0[3] = (sm12 - sm21) / s;
                        out.0[0] = (sm31 + sm13) / s;
                        out.0[1] = (sm23 + sm32) / s;
                        out.0[2] = 0.25 * s;
                    }

                    out
                }

                /// Returns whether or not the matrices have approximately the same elements in the same position.
                ///
                /// Refers to `equals` function in `glMatrix`. `exactEquals` is implemented with [`PartialEq`] and [`Eq`],
                #[inline]
                pub fn approximate_eq(&self, b: &Mat4<$t>)  -> bool {
                    let a0 = self.0[0];
                    let a1 = self.0[1];
                    let a2 = self.0[2];
                    let a3 = self.0[3];
                    let a4 = self.0[4];
                    let a5 = self.0[5];
                    let a6 = self.0[6];
                    let a7 = self.0[7];
                    let a8 = self.0[8];
                    let a9 = self.0[9];
                    let a10 = self.0[10];
                    let a11 = self.0[11];
                    let a12 = self.0[12];
                    let a13 = self.0[13];
                    let a14 = self.0[14];
                    let a15 = self.0[15];

                    let b0 = b.0[0];
                    let b1 = b.0[1];
                    let b2 = b.0[2];
                    let b3 = b.0[3];
                    let b4 = b.0[4];
                    let b5 = b.0[5];
                    let b6 = b.0[6];
                    let b7 = b.0[7];
                    let b8 = b.0[8];
                    let b9 = b.0[9];
                    let b10 = b.0[10];
                    let b11 = b.0[11];
                    let b12 = b.0[12];
                    let b13 = b.0[13];
                    let b14 = b.0[14];
                    let b15 = b.0[15];

                    return (
                      (a0 - b0).abs() <= $epsilon * (1.0 as $t).max(a0.abs()).max(b0.abs()) &&
                      (a1 - b1).abs() <= $epsilon * (1.0 as $t).max(a1.abs()).max(b1.abs()) &&
                      (a2 - b2).abs() <= $epsilon * (1.0 as $t).max(a2.abs()).max(b2.abs()) &&
                      (a3 - b3).abs() <= $epsilon * (1.0 as $t).max(a3.abs()).max(b3.abs()) &&
                      (a4 - b4).abs() <= $epsilon * (1.0 as $t).max(a4.abs()).max(b4.abs()) &&
                      (a5 - b5).abs() <= $epsilon * (1.0 as $t).max(a5.abs()).max(b5.abs()) &&
                      (a6 - b6).abs() <= $epsilon * (1.0 as $t).max(a6.abs()).max(b6.abs()) &&
                      (a7 - b7).abs() <= $epsilon * (1.0 as $t).max(a7.abs()).max(b7.abs()) &&
                      (a8 - b8).abs() <= $epsilon * (1.0 as $t).max(a8.abs()).max(b8.abs()) &&
                      (a9 - b9).abs() <= $epsilon * (1.0 as $t).max(a9.abs()).max(b9.abs()) &&
                      (a10 - b10).abs() <= $epsilon * (1.0 as $t).max(a10.abs()).max(b10.abs()) &&
                      (a11 - b11).abs() <= $epsilon * (1.0 as $t).max(a11.abs()).max(b11.abs()) &&
                      (a12 - b12).abs() <= $epsilon * (1.0 as $t).max(a12.abs()).max(b12.abs()) &&
                      (a13 - b13).abs() <= $epsilon * (1.0 as $t).max(a13.abs()).max(b13.abs()) &&
                      (a14 - b14).abs() <= $epsilon * (1.0 as $t).max(a14.abs()).max(b14.abs()) &&
                      (a15 - b15).abs() <= $epsilon * (1.0 as $t).max(a15.abs()).max(b15.abs())
                    );
                }

                // #[inline]
                // pub fn mul_to<'a, 'b, 'c>(&'a self, b: &'b Mat4<$t>, out: &'c mut Mat4<$t>) {
                //     let a00 = self.0[0];
                //     let a01 = self.0[1];
                //     let a02 = self.0[2];
                //     let a03 = self.0[3];
                //     let a10 = self.0[4];
                //     let a11 = self.0[5];
                //     let a12 = self.0[6];
                //     let a13 = self.0[7];
                //     let a20 = self.0[8];
                //     let a21 = self.0[9];
                //     let a22 = self.0[10];
                //     let a23 = self.0[11];
                //     let a30 = self.0[12];
                //     let a31 = self.0[13];
                //     let a32 = self.0[14];
                //     let a33 = self.0[15];

                //     // Cache only the current line of the second matrix
                //     let mut b0 = b.0[0];
                //     let mut b1 = b.0[1];
                //     let mut b2 = b.0[2];
                //     let mut b3 = b.0[3];
                //     out.0[0] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
                //     out.0[1] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
                //     out.0[2] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
                //     out.0[3] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;

                //     b0 = b.0[4];
                //     b1 = b.0[5];
                //     b2 = b.0[6];
                //     b3 = b.0[7];
                //     out.0[4] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
                //     out.0[5] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
                //     out.0[6] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
                //     out.0[7] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;

                //     b0 = b.0[8];
                //     b1 = b.0[9];
                //     b2 = b.0[10];
                //     b3 = b.0[11];
                //     out.0[8] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
                //     out.0[9] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
                //     out.0[10] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
                //     out.0[11] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;

                //     b0 = b.0[12];
                //     b1 = b.0[13];
                //     b2 = b.0[14];
                //     b3 = b.0[15];
                //     out.0[12] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
                //     out.0[13] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
                //     out.0[14] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
                //     out.0[15] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;
                // }
            }

            impl Add<Mat4<$t>> for Mat4<$t> {
                type Output = Mat4<$t>;

                #[inline]
                fn add(self, b: Mat4<$t>) -> Mat4<$t> {
                    let mut out = Mat4::<$t>::new_identity();
                    out.0[0] = self.0[0] + b.0[0];
                    out.0[1] = self.0[1] + b.0[1];
                    out.0[2] = self.0[2] + b.0[2];
                    out.0[3] = self.0[3] + b.0[3];
                    out.0[4] = self.0[4] + b.0[4];
                    out.0[5] = self.0[5] + b.0[5];
                    out.0[6] = self.0[6] + b.0[6];
                    out.0[7] = self.0[7] + b.0[7];
                    out.0[8] = self.0[8] + b.0[8];
                    out.0[9] = self.0[9] + b.0[9];
                    out.0[10] = self.0[10] + b.0[10];
                    out.0[11] = self.0[11] + b.0[11];
                    out.0[12] = self.0[12] + b.0[12];
                    out.0[13] = self.0[13] + b.0[13];
                    out.0[14] = self.0[14] + b.0[14];
                    out.0[15] = self.0[15] + b.0[15];
                    out
                }
            }

            impl Sub<Mat4<$t>> for Mat4<$t> {
                type Output = Mat4<$t>;

                #[inline]
                fn sub(self, b: Mat4<$t>) -> Mat4<$t> {
                    let mut out = Mat4::<$t>::new_identity();
                    out.0[0] = self.0[0] - b.0[0];
                    out.0[1] = self.0[1] - b.0[1];
                    out.0[2] = self.0[2] - b.0[2];
                    out.0[3] = self.0[3] - b.0[3];
                    out.0[4] = self.0[4] - b.0[4];
                    out.0[5] = self.0[5] - b.0[5];
                    out.0[6] = self.0[6] - b.0[6];
                    out.0[7] = self.0[7] - b.0[7];
                    out.0[8] = self.0[8] - b.0[8];
                    out.0[9] = self.0[9] - b.0[9];
                    out.0[10] = self.0[10] - b.0[10];
                    out.0[11] = self.0[11] - b.0[11];
                    out.0[12] = self.0[12] - b.0[12];
                    out.0[13] = self.0[13] - b.0[13];
                    out.0[14] = self.0[14] - b.0[14];
                    out.0[15] = self.0[15] - b.0[15];
                    out
                }
            }

            impl Mul<Mat4<$t>> for Mat4<$t> {
                type Output = Mat4<$t>;

                #[inline]
                fn mul(self, b: Mat4<$t>) -> Mat4<$t> {
                    let mut out = Mat4::<$t>::new_identity();
                    let a00 = self.0[0];
                    let a01 = self.0[1];
                    let a02 = self.0[2];
                    let a03 = self.0[3];
                    let a10 = self.0[4];
                    let a11 = self.0[5];
                    let a12 = self.0[6];
                    let a13 = self.0[7];
                    let a20 = self.0[8];
                    let a21 = self.0[9];
                    let a22 = self.0[10];
                    let a23 = self.0[11];
                    let a30 = self.0[12];
                    let a31 = self.0[13];
                    let a32 = self.0[14];
                    let a33 = self.0[15];

                    // Cache only the current line of the second matrix
                    let mut b0 = b.0[0];
                    let mut b1 = b.0[1];
                    let mut b2 = b.0[2];
                    let mut b3 = b.0[3];
                    out.0[0] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
                    out.0[1] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
                    out.0[2] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
                    out.0[3] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;

                    b0 = b.0[4];
                    b1 = b.0[5];
                    b2 = b.0[6];
                    b3 = b.0[7];
                    out.0[4] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
                    out.0[5] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
                    out.0[6] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
                    out.0[7] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;

                    b0 = b.0[8];
                    b1 = b.0[9];
                    b2 = b.0[10];
                    b3 = b.0[11];
                    out.0[8] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
                    out.0[9] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
                    out.0[10] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
                    out.0[11] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;

                    b0 = b.0[12];
                    b1 = b.0[13];
                    b2 = b.0[14];
                    b3 = b.0[15];
                    out.0[12] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
                    out.0[13] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
                    out.0[14] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
                    out.0[15] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;
                    out
                }
            }

            impl Mul<$t> for Mat4<$t> {
                type Output = Mat4<$t>;

                #[inline]
                fn mul(self, b: $t) -> Mat4<$t> {
                    let mut out = Mat4::<$t>::new_identity();
                    out.0[0] = self.0[0] * b;
                    out.0[1] = self.0[1] * b;
                    out.0[2] = self.0[2] * b;
                    out.0[3] = self.0[3] * b;
                    out.0[4] = self.0[4] * b;
                    out.0[5] = self.0[5] * b;
                    out.0[6] = self.0[6] * b;
                    out.0[7] = self.0[7] * b;
                    out.0[8] = self.0[8] * b;
                    out.0[9] = self.0[9] * b;
                    out.0[10] = self.0[10] * b;
                    out.0[11] = self.0[11] * b;
                    out.0[12] = self.0[12] * b;
                    out.0[13] = self.0[13] * b;
                    out.0[14] = self.0[14] * b;
                    out.0[15] = self.0[15] * b;
                    out
                }
            }

            impl Display for Mat4<$t> {
                fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                    let value = self.0.iter().map(|v| v.to_string()).collect::<Vec<_>>().join(", ");
                    f.write_fmt(format_args!("mat4({})", value))
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
        ($t:tt, $epsilon:expr, $pi:expr) => {
            use std::sync::OnceLock;

            use crate::error::Error;
            use crate::mat4::Mat4;
            use crate::vec3::Vec3;

            static MAT_A_RAW: [$t; 16] = [
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                1.0, 2.0, 3.0, 1.0
            ];
            static MAT_B_RAW: [$t; 16] = [
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                4.0, 5.0, 6.0, 1.0
            ];
            static MAT_IDENTITY_RAW: [$t; 16] = [
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0
            ];

            static MAT_A: OnceLock<Mat4<$t>> = OnceLock::new();
            static MAT_B: OnceLock<Mat4<$t>> = OnceLock::new();
            static MAT_IDENTITY: OnceLock<Mat4<$t>> = OnceLock::new();

            fn mat_a() -> &'static Mat4<$t> {
                MAT_A.get_or_init(|| {
                    Mat4::<$t>::from_slice(&MAT_A_RAW)
                })
            }

            fn mat_b() -> &'static Mat4<$t> {
                MAT_B.get_or_init(|| {
                    Mat4::<$t>::from_slice(&MAT_B_RAW)
                })
            }

            fn mat_identity() -> &'static Mat4<$t> {
                MAT_IDENTITY.get_or_init(|| {
                    Mat4::<$t>::from_slice(&MAT_IDENTITY_RAW)
                })
            }

            #[test]
            fn new() {
                assert_eq!(
                    Mat4::<$t>::new().raw(),
                    &[
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0
                    ]
                );
            }

            #[test]
            fn new_identity() {
                assert_eq!(
                    Mat4::<$t>::new_identity().raw(),
                    &MAT_IDENTITY_RAW
                );
            }

            #[test]
            fn from_slice() {
                assert_eq!(
                    Mat4::<$t>::from_slice(&[
                        1.0,  2.0,  3.0,  4.0,
                        5.0,  6.0,  7.0,  8.0,
                        9.0, 10.0, 11.0, 12.0,
                       13.0, 14.0, 15.0, 16.0
                    ]).raw(),
                    &[
                        1.0,  2.0,  3.0,  4.0,
                        5.0,  6.0,  7.0,  8.0,
                        9.0, 10.0, 11.0, 12.0,
                       13.0, 14.0, 15.0, 16.0
                    ]
                );
            }

            #[test]
            fn from_values() {
                assert_eq!(
                    Mat4::<$t>::from_values(
                         1.0,  2.0,  3.0,  4.0,
                         5.0,  6.0,  7.0,  8.0,
                         9.0, 10.0, 11.0, 12.0,
                        13.0, 14.0, 15.0, 16.0,
                    )
                    .raw(),
                    &[
                        1.0,  2.0,  3.0,  4.0,
                        5.0,  6.0,  7.0,  8.0,
                        9.0, 10.0, 11.0, 12.0,
                       13.0, 14.0, 15.0, 16.0
                    ]
                );
            }

            #[test]
            fn from_frustum() {
                assert_eq!(
                    Mat4::<$t>::from_frustum(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0) .raw(),
                    &[
                        -1.0,  0.0, 0.0,  0.0,
                         0.0, -1.0, 0.0,  0.0,
                         0.0,  0.0, 0.0, -1.0,
                         0.0,  0.0, 1.0,  0.0
                    ]
                );
            }

            #[test]
            fn from_look_at() {
                let out = Mat4::<$t>::from_look_at(
                    &Vec3::<$t>::from_values(0.0, 0.0, 1.0),
                    &Vec3::<$t>::from_values(0.0, 0.0, -1.0),
                    &Vec3::<$t>::from_values(0.0, 1.0, 0.0),
                );

                assert_eq!(
                    out.raw(),
                    &[
                        1.0, 0.0,  0.0, 0.0,
                        0.0, 1.0,  0.0, 0.0,
                        0.0, 0.0,  1.0, 0.0,
                        0.0, 0.0, -1.0, 1.0
                    ]
                );
            }

            #[test]
            fn from_target_to() {
                let out = Mat4::<$t>::from_target_to(
                    &Vec3::<$t>::from_values(0.0, 0.0, 1.0),
                    &Vec3::<$t>::from_values(0.0, 0.0, -1.0),
                    &Vec3::<$t>::from_values(0.0, 1.0, 0.0),
                );

                assert_eq!(
                    out.raw(),
                    &[
                        1.0, 0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0, 0.0,
                        0.0, 0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0, 1.0
                    ]
                );
                
                let scaling = out.scaling();
                assert_eq!(
                    scaling.raw(),
                    &[1.0, 1.0, 1.0]
                );
            }

            #[test]
            fn transpose() {
                assert_eq!(
                    mat_a().transpose().raw(),
                    &[
                        1.0, 0.0, 0.0, 1.0,
                        0.0, 1.0, 0.0, 2.0,
                        0.0, 0.0, 1.0, 3.0,
                        0.0, 0.0, 0.0, 1.0
                    ]
                );
            }

            #[test]
            fn invert() -> Result<(), Error> {
                assert_eq!(
                    mat_a().invert()?.raw(),
                    &[
                         1.0,  0.0,  0.0, 0.0,
                         0.0,  1.0,  0.0, 0.0,
                         0.0,  0.0,  1.0, 0.0,
                        -1.0, -2.0, -3.0, 1.0
                    ]
                );

                Ok(())
            }

            #[test]
            fn adjoint() {
                assert_eq!(
                    mat_a().adjoint().raw(),
                    &[
                         1.0,  0.0,  0.0, 0.0,
                         0.0,  1.0,  0.0, 0.0,
                         0.0,  0.0,  1.0, 0.0,
                        -1.0, -2.0, -3.0, 1.0
                    ]
                );
            }

            #[test]
            fn determinant() {
                assert_eq!(
                    mat_a().determinant(),
                    1.0
                );
            }

            #[test]
            fn translate() {
                let out = mat_a().translate(&Vec3::<$t>::from_values(4.0, 5.0, 6.0));
                assert_eq!(
                    out.raw(),
                    &[
                        1.0, 0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0, 0.0,
                        0.0, 0.0, 1.0, 0.0,
                        5.0, 7.0, 9.0, 1.0
                    ]
                );
            }

            #[test]
            fn scale() {
                let out = mat_a().scale(&Vec3::<$t>::from_values(4.0, 5.0, 6.0));
                assert_eq!(
                    out.raw(),
                    &[
                        4.0, 0.0, 0.0, 0.0,
                        0.0, 5.0, 0.0, 0.0,
                        0.0, 0.0, 6.0, 0.0,
                        1.0, 2.0, 3.0, 1.0
                    ]
                );
            }

            #[test]
            fn rotate_x() {
                let rad = $pi * 0.5;

                let out = mat_a().rotate_x(rad);
                assert_eq!(
                    out.raw(),
                    &[
                        1.0, 0.0, 0.0, 0.0,
                        0.0, rad.cos(), rad.sin(), 0.0,
                        0.0, -rad.sin(), rad.cos(), 0.0,
                        1.0, 2.0, 3.0, 1.0
                    ]
                );
            }

            #[test]
            fn rotate_y() {
                let rad = $pi * 0.5;

                let out = mat_a().rotate_y(rad);
                assert_eq!(
                    out.raw(),
                    &[
                        rad.cos(), 0.0, -rad.sin(), 0.0,
                        0.0, 1.0, 0.0, 0.0,
                        rad.sin(), 0.0, rad.cos(), 0.0,
                        1.0, 2.0, 3.0, 1.0
                    ]
                );
            }

            #[test]
            fn rotate_z() {
                let rad = $pi * 0.5;

                let out = mat_a().rotate_z(rad);
                assert_eq!(
                    out.raw(),
                    &[
                        rad.cos(), rad.sin(), 0.0, 0.0,
                        -rad.sin(), rad.cos(), 0.0, 0.0,
                        0.0, 0.0, 1.0, 0.0,
                        1.0, 2.0, 3.0, 1.0
                    ]
                );
            }

            #[test]
            fn translation() {
                let translation = mat_identity().translation();
                assert_eq!(
                    translation.raw(),
                    &[0.0, 0.0, 0.0]
                );
            }

            #[test]
            fn scaling() {
                let scaling = mat_identity().scaling();
                assert_eq!(
                    scaling.raw(),
                    &[1.0, 1.0, 1.0]
                );
            }

            #[test]
            fn rotation() {
                let rotation = mat_identity().rotation();
                assert_eq!(
                    rotation.raw(),
                    &[0.0, 0.0, 0.0, 1.0]
                );
            }

            #[test]
            fn decompose() {
                let (out_r, out_t, out_s) = mat_identity().decompose();
                assert_eq!(
                    out_r.raw(),
                    &[0.0, 0.0, 0.0, 1.0]
                );
                assert_eq!(
                    out_t.raw(),
                    &[0.0, 0.0, 0.0]
                );
                assert_eq!(
                    out_s.raw(),
                    &[1.0, 1.0, 1.0]
                );
            }

            #[test]
            fn frob() {
                let out = mat_a().frob();
                assert_eq!(
                    out,
                    (
                        (1.0 as $t).powi(2) +
                        (1.0 as $t).powi(2) +
                        (1.0 as $t).powi(2) +
                        (1.0 as $t).powi(2) +
                        (1.0 as $t).powi(2) +
                        (2.0 as $t).powi(2) +
                        (3.0 as $t).powi(2)
                    ).sqrt()
                );
            }

            #[test]
            fn set() {
                let mut mat = Mat4::<$t>::new();
                mat.set(
                     1.0,  2.0,  3.0,  4.0,
                     5.0,  6.0,  7.0,  8.0,
                     9.0, 10.0, 11.0, 12.0,
                    13.0, 14.0, 15.0, 16.0
                );

                assert_eq!(
                    mat.raw(),
                    &[
                         1.0,  2.0,  3.0,  4.0,
                         5.0,  6.0,  7.0,  8.0,
                         9.0, 10.0, 11.0, 12.0,
                        13.0, 14.0, 15.0, 16.0
                    ]
                );
            }

            #[test]
            fn set_slice() {
                let mut mat = Mat4::<$t>::new();
                mat.set_slice(&[
                     1.0,  2.0,  3.0,  4.0,
                     5.0,  6.0,  7.0,  8.0,
                     9.0, 10.0, 11.0, 12.0,
                    13.0, 14.0, 15.0, 16.0
                ]);

                assert_eq!(
                    mat.raw(),
                    &[
                         1.0,  2.0,  3.0,  4.0,
                         5.0,  6.0,  7.0,  8.0,
                         9.0, 10.0, 11.0, 12.0,
                        13.0, 14.0, 15.0, 16.0
                    ]
                );
            }

            #[test]
            fn add() {
                let mat_a = Mat4::<$t>::from_values(
                     1.0,  2.0,  3.0,  4.0,
                     5.0,  6.0,  7.0,  8.0,
                     9.0, 10.0, 11.0, 12.0,
                    13.0, 14.0, 15.0, 16.0
                );
                let mat_b = Mat4::<$t>::from_values(
                    17.0, 18.0, 19.0, 20.0,
                    21.0, 22.0, 23.0, 24.0,
                    25.0, 26.0, 27.0, 28.0,
                    29.0, 30.0, 31.0, 32.0
                );

                assert_eq!(
                    (mat_a + mat_b).raw(),
                    &[
                        18.0, 20.0, 22.0, 24.0,
                        26.0, 28.0, 30.0, 32.0,
                        34.0, 36.0, 38.0, 40.0,
                        42.0, 44.0, 46.0, 48.0
                    ]
                );
            }

            #[test]
            fn sub() {
                let mat_a = Mat4::<$t>::from_values(
                     1.0,  2.0,  3.0,  4.0,
                     5.0,  6.0,  7.0,  8.0,
                     9.0, 10.0, 11.0, 12.0,
                    13.0, 14.0, 15.0, 16.0
                );
                let mat_b = Mat4::<$t>::from_values(
                    17.0, 18.0, 19.0, 20.0,
                    21.0, 22.0, 23.0, 24.0,
                    25.0, 26.0, 27.0, 28.0,
                    29.0, 30.0, 31.0, 32.0
                );

                assert_eq!(
                    (mat_a - mat_b).raw(),
                    &[
                        -16.0, -16.0, -16.0, -16.0,
                        -16.0, -16.0, -16.0, -16.0,
                        -16.0, -16.0, -16.0, -16.0,
                        -16.0, -16.0, -16.0, -16.0
                    ]
                );
            }

            #[test]
            fn mul() {
                let out = *mat_a() * *mat_b();
                assert_eq!(
                    out.raw(),
                    &[
                        1.0, 0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0, 0.0,
                        0.0, 0.0, 1.0, 0.0,
                        5.0, 7.0, 9.0, 1.0
                    ]
                );
            }

            #[test]
            fn mul_scalar() {
                let mat = Mat4::<$t>::from_values(
                     1.0,  2.0,  3.0,  4.0,
                     5.0,  6.0,  7.0,  8.0,
                     9.0, 10.0, 11.0, 12.0,
                    13.0, 14.0, 15.0, 16.0
                );

                assert_eq!(
                    (mat * 2.0).raw(),
                    &[
                         2.0,  4.0,  6.0,  8.0,
                        10.0, 12.0, 14.0, 16.0,
                        18.0, 20.0, 22.0, 24.0,
                        26.0, 28.0, 30.0, 32.0
                    ]
                );
            }

            #[test]
            fn mul_scalar_add() {
                let mat_a = Mat4::<$t>::from_values(
                     1.0,  2.0,  3.0,  4.0,
                     5.0,  6.0,  7.0,  8.0,
                     9.0, 10.0, 11.0, 12.0,
                    13.0, 14.0, 15.0, 16.0
                );
                let mat_b = Mat4::<$t>::from_values(
                    17.0, 18.0, 19.0, 20.0,
                    21.0, 22.0, 23.0, 24.0,
                    25.0, 26.0, 27.0, 28.0,
                    29.0, 30.0, 31.0, 32.0
                );

                assert_eq!(
                    (mat_a + mat_b * 0.5).raw(),
                    &[
                         9.5, 11.0, 12.5, 14.0,
                        15.5, 17.0, 18.5, 20.0,
                        21.5, 23.0, 24.5, 26.0,
                        27.5, 29.0, 30.5, 32.0
                    ]
                );
            }

            #[test]
            fn approximate_eq() {
                let mat_a = Mat4::<$t>::from_values(
                     0.0,  1.0,  2.0,  3.0,
                     4.0,  5.0,  6.0,  7.0,
                     8.0,  9.0, 10.0, 11.0,
                    12.0, 13.0, 14.0, 15.0
                );
                let mat_b = Mat4::<$t>::from_values(
                     0.0,  1.0,  2.0,  3.0,
                     4.0,  5.0,  6.0,  7.0,
                     8.0,  9.0, 10.0, 11.0,
                    12.0, 13.0, 14.0, 15.0
                );
                let mat_c = Mat4::<$t>::from_values(
                     1.0,  2.0,  3.0,  4.0,
                     5.0,  6.0,  7.0,  8.0,
                     9.0, 10.0, 11.0, 12.0,
                    13.0, 14.0, 15.0, 16.0
                );
                let mat_d = Mat4::<$t>::from_values(
                    1e-16,  1.0,  2.0,  3.0,
                     4.0,  5.0,  6.0,  7.0,
                     8.0,  9.0, 10.0, 11.0,
                    12.0, 13.0, 14.0, 15.0
                );

                assert_eq!(
                    true,
                    mat_a.approximate_eq(&mat_b)
                );
                assert_eq!(
                    false,
                    mat_a.approximate_eq(&mat_c)
                );
                assert_eq!(
                    true,
                    mat_a.approximate_eq(&mat_d)
                );
            }

            #[test]
            fn display() {
                let out = mat_a().to_string();
                assert_eq!(
                    out,
                    "mat4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 2, 3, 1)"
                );
            }
        };
    }

    mod f32 {
        float_test!(f32, crate::EPSILON_F32, std::f32::consts::PI);

        #[test]
        fn rotate() -> Result<(), Error> {
            let rad = std::f32::consts::PI * 0.5;
            let axis = Vec3::<f32>::from_values(1.0, 0.0, 0.0);

            let out = mat_a().rotate(&axis, rad)?;
            assert_eq!(
                out.raw(),
                &[
                    0.99999994, 0.0, 0.0, 0.0,
                    0.0, rad.cos(), rad.sin(), 0.0,
                    0.0, -rad.sin(), rad.cos(), 0.0,
                    1.0, 2.0, 3.0, 1.0
                ]
            );

            Ok(())
        }

        #[test]
        fn from_perspective_no() {
            assert_eq!(
                Mat4::<f32>::from_perspective_no(std::f32::consts::PI * 0.5, 1.0, 0.0, Some(1.0)).raw(),
                &[
                    1.0, 0.0,  0.0,  0.0,
                    0.0, 1.0,  0.0,  0.0,
                    0.0, 0.0, -1.0, -1.0,
                    0.0, 0.0,  0.0,  0.0
                ]
            );

            assert_eq!(
                Mat4::<f32>::from_perspective_no(std::f32::consts::PI * 45.0 / 180.0, 640.0 / 480.0, 0.1, Some(200.0)).raw(),
                &[
                    1.81066, 0.0, 0.0, 0.0,
                    0.0, 2.4142134, 0.0, 0.0,
                    0.0, 0.0, -1.0010005, -1.0,
                    0.0, 0.0, -0.20010006, 0.0
                ]
            );

            assert_eq!(
                Mat4::<f32>::from_perspective_no(std::f32::consts::PI * 45.0 / 180.0, 640.0 / 480.0, 0.1, None).raw(),
                &[
                    1.81066, 0.0, 0.0, 0.0,
                    0.0, 2.4142134, 0.0, 0.0,
                    0.0, 0.0, -1.0, -1.0,
                    0.0, 0.0, -0.2, 0.0
                ]
            );

            assert_eq!(
                Mat4::<f32>::from_perspective_no(std::f32::consts::PI * 45.0 / 180.0, 640.0 / 480.0, 0.1, Some(f32::INFINITY)).raw(),
                &[
                    1.81066, 0.0, 0.0, 0.0,
                    0.0, 2.4142134, 0.0, 0.0,
                    0.0, 0.0, -1.0, -1.0,
                    0.0, 0.0, -0.2, 0.0
                ]
            );
        }
    }

    mod f64 {
        float_test!(f64, crate::EPSILON_F64, std::f64::consts::PI);
        
        #[test]
        fn rotate() -> Result<(), Error> {
            let rad = std::f64::consts::PI * 0.5;
            let axis = Vec3::<f64>::from_values(1.0, 0.0, 0.0);

            let out = mat_a().rotate(&axis, rad)?;
            assert_eq!(
                out.raw(),
                &[
                    1.0, 0.0, 0.0, 0.0,
                    0.0, rad.cos(), rad.sin(), 0.0,
                    0.0, -rad.sin(), rad.cos(), 0.0,
                    1.0, 2.0, 3.0, 1.0
                ]
            );

            Ok(())
        }

        #[test]
        fn from_perspective_no() {
            assert_eq!(
                Mat4::<f64>::from_perspective_no(std::f64::consts::PI * 0.5, 1.0, 0.0, Some(1.0)).raw(),
                &[
                    1.0000000000000002, 0.0,  0.0,  0.0,
                    0.0, 1.0000000000000002,  0.0,  0.0,
                    0.0, 0.0, -1.0, -1.0,
                    0.0, 0.0,  0.0,  0.0
                ]
            );

            assert_eq!(
                Mat4::<f64>::from_perspective_no(std::f64::consts::PI * 45.0 / 180.0, 640.0 / 480.0, 0.1, Some(200.0)).raw(),
                &[
                    1.8106601717798212, 0.0, 0.0, 0.0,
                    0.0, 2.414213562373095, 0.0, 0.0,
                    0.0, 0.0, -1.001000500250125, -1.0,
                    0.0, 0.0, -0.2001000500250125, 0.0
                ]
            );

            assert_eq!(
                Mat4::<f64>::from_perspective_no(std::f64::consts::PI * 45.0 / 180.0, 640.0 / 480.0, 0.1, None).raw(),
                &[
                    1.8106601717798212, 0.0, 0.0, 0.0,
                    0.0, 2.414213562373095, 0.0, 0.0,
                    0.0, 0.0, -1.0, -1.0,
                    0.0, 0.0, -0.2, 0.0
                ]
            );

            assert_eq!(
                Mat4::<f64>::from_perspective_no(std::f64::consts::PI * 45.0 / 180.0, 640.0 / 480.0, 0.1, Some(f64::INFINITY)).raw(),
                &[
                    1.8106601717798212, 0.0, 0.0, 0.0,
                    0.0, 2.414213562373095, 0.0, 0.0,
                    0.0, 0.0, -1.0, -1.0,
                    0.0, 0.0, -0.2, 0.0
                ]
            );
        }
    }
}
