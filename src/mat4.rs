use crate::{quat::Quat, quat2::Quat2, vec3::Vec3, EPSILON_F32, EPSILON_F64};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mat4<T = f64>(pub [T; 16]);

macro_rules! float {
    ($(($t:tt, $epsilon:expr)),+) => {
        $(
            impl Mat4<$t> {
                pub fn new() -> Mat4<$t> {
                    Self([0.0; 16])
                }

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
                ) -> Mat4<$t> {
                    let mut out = Self([0.0; 16]);
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

                pub fn set_by_values(
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
                ) -> &mut Mat4<$t> {
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

                /// Sets values of a matrix by a vector translation.
                ///
                /// This method refers to `fromTranslation` in [`glMatrix`](https://glmatrix.net/docs/module-mat4.html).
                pub fn set_by_translation(
                    &mut self,
                    v: &Vec3<$t>,
                ) -> &mut Mat4<$t> {
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
                    self.0[12] = v.0[0];
                    self.0[13] = v.0[1];
                    self.0[14] = v.0[2];
                    self.0[15] = 1.0;
                    self
                }

                /// Sets values of a matrix by a vector scaling.
                ///
                /// This method refers to `fromScaling` in [`glMatrix`](https://glmatrix.net/docs/module-mat4.html).
                pub fn set_by_scaling(
                    &mut self,
                    v: &Vec3<$t>
                ) -> &mut Mat4<$t> {
                    self.0[0] = v.0[0];
                    self.0[1] = 0.0;
                    self.0[2] = 0.0;
                    self.0[3] = 0.0;
                    self.0[4] = 0.0;
                    self.0[5] = v.0[1];
                    self.0[6] = 0.0;
                    self.0[7] = 0.0;
                    self.0[8] = 0.0;
                    self.0[9] = 0.0;
                    self.0[10] = v.0[2];
                    self.0[11] = 0.0;
                    self.0[12] = 0.0;
                    self.0[13] = 0.0;
                    self.0[14] = 0.0;
                    self.0[15] = 1.0;
                    self
                }

                /// Sets values of a matrix by a given angle around a given axis.
                ///
                /// This method refers to `fromRotation` in [`glMatrix`](https://glmatrix.net/docs/module-mat4.html).
                pub fn set_by_rotation(
                    &mut self,
                    rad: $t,
                    axis: &Vec3<$t>
                ) -> Option<&mut Mat4<$t>> {
                    let mut x = axis.0[0];
                    let mut y = axis.0[1];
                    let mut z = axis.0[2];
                    let mut len = (x * x + y * y + z * z).sqrt();
                    let s;
                    let c;
                    let t;

                    if (len < $epsilon) {
                      return None;
                    }

                    len = 1.0 / len;
                    x *= len;
                    y *= len;
                    z *= len;

                    s = rad.sin();
                    c = rad.cos();
                    t = 1.0 - c;

                    // Perform rotation-specific matrix multiplication
                    self.0[0] = x * x * t + c;
                    self.0[1] = y * x * t + z * s;
                    self.0[2] = z * x * t - y * s;
                    self.0[3] = 0.0;
                    self.0[4] = x * y * t - z * s;
                    self.0[5] = y * y * t + c;
                    self.0[6] = z * y * t + x * s;
                    self.0[7] = 0.0;
                    self.0[8] = x * z * t + y * s;
                    self.0[9] = y * z * t - x * s;
                    self.0[10] = z * z * t + c;
                    self.0[11] = 0.0;
                    self.0[12] = 0.0;
                    self.0[13] = 0.0;
                    self.0[14] = 0.0;
                    self.0[15] = 1.0;
                    Some(self)
                }

                /// Sets values of a matrix by a given angle around the X axis.
                ///
                /// This method refers to `fromXRotation` in [`glMatrix`](https://glmatrix.net/docs/module-mat4.html).
                pub fn set_by_x_rotation(
                    &mut self,
                    rad: $t
                ) -> &mut Mat4<$t> {
                    let s = rad.sin();
                    let c = rad.cos();

                    // Perform axis-specific matrix multiplication
                    self.0[0] = 1.0;
                    self.0[1] = 0.0;
                    self.0[2] = 0.0;
                    self.0[3] = 0.0;
                    self.0[4] = 0.0;
                    self.0[5] = c;
                    self.0[6] = s;
                    self.0[7] = 0.0;
                    self.0[8] = 0.0;
                    self.0[9] = -s;
                    self.0[10] = c;
                    self.0[11] = 0.0;
                    self.0[12] = 0.0;
                    self.0[13] = 0.0;
                    self.0[14] = 0.0;
                    self.0[15] = 1.0;
                    self
                }

                /// Sets values of a matrix by a given angle around the Y axis.
                ///
                /// This method refers to `fromYRotation` in [`glMatrix`](https://glmatrix.net/docs/module-mat4.html).
                pub fn set_by_y_rotation(
                    &mut self,
                    rad: $t
                ) -> &mut Mat4<$t> {
                    let s = rad.sin();
                    let c = rad.cos();

                    // Perform axis-specific matrix multiplication
                    self.0[0] = c;
                    self.0[1] = 0.0;
                    self.0[2] = -s;
                    self.0[3] = 0.0;
                    self.0[4] = 0.0;
                    self.0[5] = 1.0;
                    self.0[6] = 0.0;
                    self.0[7] = 0.0;
                    self.0[8] = s;
                    self.0[9] = 0.0;
                    self.0[10] = c;
                    self.0[11] = 0.0;
                    self.0[12] = 0.0;
                    self.0[13] = 0.0;
                    self.0[14] = 0.0;
                    self.0[15] = 1.0;
                    self
                }

                /// Sets values of a matrix by a given angle around the Z axis.
                ///
                /// This method refers to `fromZRotation` in [`glMatrix`](https://glmatrix.net/docs/module-mat4.html).
                pub fn set_by_z_rotation(
                    &mut self,
                    rad: $t
                ) -> &mut Mat4<$t> {
                    let s = rad.sin();
                    let c = rad.cos();

                    // Perform axis-specific matrix multiplication
                    self.0[0] = c;
                    self.0[1] = s;
                    self.0[2] = 0.0;
                    self.0[3] = 0.0;
                    self.0[4] = -s;
                    self.0[5] = c;
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

                /// Sets values of a matrix by a quaternion rotation and vector translation.
                ///
                /// This method refers to `fromRotationTranslation` in [`glMatrix`](https://glmatrix.net/docs/module-mat4.html).
                pub fn set_by_rotation_translation(
                    &mut self,
                    q: &Quat2<$t>,
                    v: &Vec3<$t>,
                ) -> &mut Mat4<$t> {
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

                    self.0[0] = 1.0 - (yy + zz);
                    self.0[1] = xy + wz;
                    self.0[2] = xz - wy;
                    self.0[3] = 0.0;
                    self.0[4] = xy - wz;
                    self.0[5] = 1.0 - (xx + zz);
                    self.0[6] = yz + wx;
                    self.0[7] = 0.0;
                    self.0[8] = xz + wy;
                    self.0[9] = yz - wx;
                    self.0[10] = 1.0 - (xx + yy);
                    self.0[11] = 0.0;
                    self.0[12] = v.0[0];
                    self.0[13] = v.0[1];
                    self.0[14] = v.0[2];
                    self.0[15] = 1.0;
                    self
                }

                /// Sets values of a matrix by a dual quat.
                ///
                /// This method refers to `fromQuat2` in [`glMatrix`](https://glmatrix.net/docs/module-mat4.html).
                pub fn set_by_quat2(
                    &mut self,
                    a: &Quat2<$t>
                ) -> &mut Mat4<$t> {
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
                    self.set_by_rotation_translation(&a, &translation);
                    self
                }

                /// Sets values of a matrix by a quaternion rotation, vector translation and vector scale.
                ///
                /// This method refers to `fromRotationTranslationScale` in [`glMatrix`](https://glmatrix.net/docs/module-mat4.html).
                pub fn set_by_rotation_translation_scale(
                    &mut self,
                    q: &Quat2<$t>,
                    v: &Vec3<$t>,
                    s: &Vec3<$t>
                ) -> &mut Mat4<$t> {
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

                    self.0[0] = (1.0 - (yy + zz)) * sx;
                    self.0[1] = (xy + wz) * sx;
                    self.0[2] = (xz - wy) * sx;
                    self.0[3] = 0.0;
                    self.0[4] = (xy - wz) * sy;
                    self.0[5] = (1.0 - (xx + zz)) * sy;
                    self.0[6] = (yz + wx) * sy;
                    self.0[7] = 0.0;
                    self.0[8] = (xz + wy) * sz;
                    self.0[9] = (yz - wx) * sz;
                    self.0[10] = (1.0 - (xx + yy)) * sz;
                    self.0[11] = 0.0;
                    self.0[12] = v.0[0];
                    self.0[13] = v.0[1];
                    self.0[14] = v.0[2];
                    self.0[15] = 1.0;
                    self
                }

                /// Sets values of a matrix by a quaternion rotation, vector translation and vector scale, rotating and scaling around the given origin.
                ///
                /// This method refers to `fromRotationTranslationScaleOrigin` in [`glMatrix`](https://glmatrix.net/docs/module-mat4.html).
                pub fn set_by_rotation_translation_scale_origin(
                    &mut self,
                    q: &Quat2<$t>,
                    v: &Vec3<$t>,
                    s: &Vec3<$t>,
                    o: &Vec3<$t>
                ) -> &mut Mat4<$t> {
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
                  
                    self.0[0] = out0;
                    self.0[1] = out1;
                    self.0[2] = out2;
                    self.0[3] = 0.0;
                    self.0[4] = out4;
                    self.0[5] = out5;
                    self.0[6] = out6;
                    self.0[7] = 0.0;
                    self.0[8] = out8;
                    self.0[9] = out9;
                    self.0[10] = out10;
                    self.0[11] = 0.0;
                    self.0[12] = v.0[0] + ox - (out0 * ox + out4 * oy + out8 * oz);
                    self.0[13] = v.0[1] + oy - (out1 * ox + out5 * oy + out9 * oz);
                    self.0[14] = v.0[2] + oz - (out2 * ox + out6 * oy + out10 * oz);
                    self.0[15] = 1.0;
                    self
                }

                /// Sets values of a matrix by a given quaternion.
                ///
                /// This method refers to `fromQuat` in [`glMatrix`](https://glmatrix.net/docs/module-mat4.html).
                pub fn set_by_quat(
                    &mut self,
                    q: &Quat<$t>
                ) -> &mut Mat4<$t> {
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
                  
                    self.0[0] = 1.0 - yy - zz;
                    self.0[1] = yx + wz;
                    self.0[2] = zx - wy;
                    self.0[3] = 0.0;
                  
                    self.0[4] = yx - wz;
                    self.0[5] = 1.0 - xx - zz;
                    self.0[6] = zy + wx;
                    self.0[7] = 0.0;
                  
                    self.0[8] = zx + wy;
                    self.0[9] = zy - wx;
                    self.0[10] = 1.0 - xx - yy;
                    self.0[11] = 0.0;
                  
                    self.0[12] = 0.0;
                    self.0[13] = 0.0;
                    self.0[14] = 0.0;
                    self.0[15] = 1.0;

                    self
                }

                /// Sets values of a matrix by a given bounds.
                ///
                /// This method refers to `frustum` in [`glMatrix`](https://glmatrix.net/docs/module-mat4.html).
                pub fn set_by_frustum(
                    &mut self,
                    left: $t,
                    right: $t,
                    bottom: $t,
                    top: $t,
                    near: $t,
                    far: $t
                ) -> &mut Mat4<$t> {
                    let rl = 1.0 / (right - left);
                    let tb = 1.0 / (top - bottom);
                    let nf = 1.0 / (near - far);
                    self.0[0] = near * 2.0 * rl;
                    self.0[1] = 0.0;
                    self.0[2] = 0.0;
                    self.0[3] = 0.0;
                    self.0[4] = 0.0;
                    self.0[5] = near * 2.0 * tb;
                    self.0[6] = 0.0;
                    self.0[7] = 0.0;
                    self.0[8] = (right + left) * rl;
                    self.0[9] = (top + bottom) * tb;
                    self.0[10] = (far + near) * nf;
                    self.0[11] = -1.0;
                    self.0[12] = 0.0;
                    self.0[13] = 0.0;
                    self.0[14] = far * near * 2.0 * nf;
                    self.0[15] = 0.0;
                    self
                }

                pub fn transpose<'a, 'b>(&'a self, out: &'b mut Mat4<$t>) -> &'b mut Mat4<$t> {
                    if out == self {
                        let a01 = self.0[1];
                        let a02 = self.0[2];
                        let a03 = self.0[3];
                        let a12 = self.0[6];
                        let a13 = self.0[7];
                        let a23 = self.0[11];

                        out.0[1] = self.0[4];
                        out.0[2] = self.0[8];
                        out.0[3] = self.0[12];
                        out.0[4] = a01;
                        out.0[6] = self.0[9];
                        out.0[7] = self.0[13];
                        out.0[8] = a02;
                        out.0[9] = a12;
                        out.0[11] = self.0[14];
                        out.0[12] = a03;
                        out.0[13] = a13;
                        out.0[14] = a23;
                    } else {
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
                    }

                    out
                }

                pub fn identity(&mut self) -> &mut Mat4<$t> {
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

                pub fn invert<'a, 'b>(&'a self, out: &'b mut Mat4<$t>) -> Option<&'b mut Mat4<$t>> {
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
                        return None;
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

                    Some(out)
                }

                pub fn adjoint<'a, 'b>(&'a self, out: &'b mut Mat4<$t>) -> &'b mut Mat4<$t> {
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

                pub fn multiply<'a, 'b>(&self, b: &'a Mat4<$t>, out: &'b mut Mat4<$t>) -> &'b mut Mat4<$t> {
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

                pub fn translate<'a, 'b>(&self, v: &'a Vec3<$t>, out: &'b mut Mat4<$t>) -> &'b mut Mat4<$t> {
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

                    if (self == out) {
                      out.0[12] = self.0[0] * x + self.0[4] * y + self.0[8] * z + self.0[12];
                      out.0[13] = self.0[1] * x + self.0[5] * y + self.0[9] * z + self.0[13];
                      out.0[14] = self.0[2] * x + self.0[6] * y + self.0[10] * z + self.0[14];
                      out.0[15] = self.0[3] * x + self.0[7] * y + self.0[11] * z + self.0[15];
                    } else {
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
                    }

                    out
                }

                pub fn scale<'a, 'b>(&self, v: &'a Vec3<$t>, out: &'b mut Mat4<$t>) -> &'b mut Mat4<$t> {
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

                pub fn rotate<'a, 'b>(&self, rad: $t, axis: &Vec3<$t>, out: &'b mut Mat4<$t>) -> Option<&'b mut Mat4<$t>> {
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
                      return None;
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

                    if (self != out) {
                      // If the source and destination differ, copy the unchanged last row
                      out.0[12] = self.0[12];
                      out.0[13] = self.0[13];
                      out.0[14] = self.0[14];
                      out.0[15] = self.0[15];
                    }

                    Some(out)
                }

                pub fn rotate_x<'a, 'b>(&self, rad: $t, out: &'b mut Mat4<$t>) -> &'b mut Mat4<$t> {
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

                    if (self != out) {
                      // If the source and destination differ, copy the unchanged rows
                      out.0[0] = self.0[0];
                      out.0[1] = self.0[1];
                      out.0[2] = self.0[2];
                      out.0[3] = self.0[3];
                      out.0[12] = self.0[12];
                      out.0[13] = self.0[13];
                      out.0[14] = self.0[14];
                      out.0[15] = self.0[15];
                    }

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

                pub fn rotate_y<'a, 'b>(&self, rad: $t, out: &'b mut Mat4<$t>) -> &'b mut Mat4<$t> {
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

                    if (self != out) {
                        // If the source and destination differ, copy the unchanged rows
                        out.0[4] = self.0[4];
                        out.0[5] = self.0[5];
                        out.0[6] = self.0[6];
                        out.0[7] = self.0[7];
                        out.0[12] = self.0[12];
                        out.0[13] = self.0[13];
                        out.0[14] = self.0[14];
                        out.0[15] = self.0[15];
                    }

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

                pub fn rotate_z<'a, 'b>(&self, rad: $t, out: &'b mut Mat4<$t>) -> &'b mut Mat4<$t> {
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

                    if (self != out) {
                        // If the source and destination differ, copy the unchanged rows
                        out.0[8] = self.0[8];
                        out.0[9] = self.0[9];
                        out.0[10] = self.0[10];
                        out.0[11] = self.0[11];
                        out.0[12] = self.0[12];
                        out.0[13] = self.0[13];
                        out.0[14] = self.0[14];
                        out.0[15] = self.0[15];
                    }

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

                pub fn decompose<'a, 'b, 'c, 'd>(
                    &'a self,
                    out_r: &'b mut Quat<$t>,
                    out_t: &'c mut Vec3<$t>,
                    out_s: &'d mut Vec3<$t>
                ) -> (&'b mut Quat<$t>, &'c mut Vec3<$t>, &'d mut Vec3<$t>) {
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
            }

            impl Mat4<$t> {
                pub fn translation<'a, 'b>(&'a self, out: &'b mut Vec3<$t>)  -> &'b mut Vec3<$t>{
                    out.0[0] = self.0[12];
                    out.0[1] = self.0[13];
                    out.0[2] = self.0[14];

                    out
                }

                pub fn scaling<'a, 'b>(&'a self, out: &'b mut Vec3<$t>)  -> &'b mut Vec3<$t>{
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

                pub fn rotation<'a, 'b>(&'a self, out: &'b mut Quat<$t>)  -> &'b mut Quat<$t>{
                    let mut scaling = Vec3::<$t>::new();
                    self.scaling(&mut scaling);

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
            }
        )+
    };
}

float! {
    (f64, EPSILON_F64),
    (f32, EPSILON_F32)
}
