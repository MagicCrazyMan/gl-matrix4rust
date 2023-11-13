use std::{
    fmt::Display,
    ops::{Add, Mul, Sub},
};

use half::f16;
use num_traits::{Float, FloatConst};

use crate::{error::Error, quat::Quat, quat2::Quat2, vec3::Vec3};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mat4<T = f32>(pub [T; 16]);

impl<T: Float> Mat4<T> {
    #[inline(always)]
    pub fn new() -> Self {
        Self([T::zero(); 16])
    }

    #[inline(always)]
    pub fn new_identity() -> Self {
        Self([
            T::one(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_values(
        m00: T,
        m01: T,
        m02: T,
        m03: T,
        m10: T,
        m11: T,
        m12: T,
        m13: T,
        m20: T,
        m21: T,
        m22: T,
        m23: T,
        m30: T,
        m31: T,
        m32: T,
        m33: T,
    ) -> Self {
        Self([
            m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33,
        ])
    }

    #[inline(always)]
    pub fn from_slice(slice: &[T; 16]) -> Self {
        Self(slice.clone())
    }

    #[inline(always)]
    pub fn from_translation(v: impl AsRef<Vec3<T>>) -> Self {
        let v = v.as_ref();

        Self([
            T::one(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
            T::zero(),
            v.0[0],
            v.0[1],
            v.0[2],
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_scaling(v: impl AsRef<Vec3<T>>) -> Self {
        let v = v.as_ref();

        Self([
            v.0[0],
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            v.0[1],
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            v.0[2],
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_rotation(rad: T, axis: impl AsRef<Vec3<T>>) -> Result<Self, Error> {
        let axis = axis.as_ref();

        let mut x = axis.0[0];
        let mut y = axis.0[1];
        let mut z = axis.0[2];
        let mut len = (x * x + y * y + z * z).sqrt();
        let s;
        let c;
        let t;

        if len < T::from(0.000001).unwrap() {
            return Err(Error::LengthSmallerThanEpsilon);
        }

        len = T::one() / len;
        x = x * len;
        y = y * len;
        z = z * len;

        s = rad.sin();
        c = rad.cos();
        t = T::one() - c;

        // Perform rotation-specific matrix multiplication
        Ok(Self([
            x * x * t + c,
            y * x * t + z * s,
            z * x * t - y * s,
            T::zero(),
            x * y * t - z * s,
            y * y * t + c,
            z * y * t + x * s,
            T::zero(),
            x * z * t + y * s,
            y * z * t - x * s,
            z * z * t + c,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
        ]))
    }

    #[inline(always)]
    pub fn from_x_rotation(rad: T) -> Self {
        let s = rad.sin();
        let c = rad.cos();

        // Perform axis-specific matrix multiplication
        Self([
            T::one(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            c,
            s,
            T::zero(),
            T::zero(),
            -s,
            c,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_y_rotation(rad: T) -> Self {
        let s = rad.sin();
        let c = rad.cos();

        // Perform axis-specific matrix multiplication
        Self([
            c,
            T::zero(),
            -s,
            T::zero(),
            T::zero(),
            T::one(),
            T::zero(),
            T::zero(),
            s,
            T::zero(),
            c,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_z_rotation(rad: T) -> Self {
        let s = rad.sin();
        let c = rad.cos();

        // Perform axis-specific matrix multiplication
        Self([
            c,
            s,
            T::zero(),
            T::zero(),
            -s,
            c,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_rotation_translation(q: impl AsRef<Quat2<T>>, v: impl AsRef<Vec3<T>>) -> Self {
        let q = q.as_ref();
        let v = v.as_ref();

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

        Self([
            T::one() - (yy + zz),
            xy + wz,
            xz - wy,
            T::zero(),
            xy - wz,
            T::one() - (xx + zz),
            yz + wx,
            T::zero(),
            xz + wy,
            yz - wx,
            T::one() - (xx + yy),
            T::zero(),
            v.0[0],
            v.0[1],
            v.0[2],
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_quat2(a: impl AsRef<Quat2<T>>) -> Self {
        let a = a.as_ref();

        let mut translation = Vec3::<T>::new();
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
        if magnitude > T::zero() {
            translation.0[0] =
                ((ax * bw + aw * bx + ay * bz - az * by) * T::from(2.0).unwrap()) / magnitude;
            translation.0[1] =
                ((ay * bw + aw * by + az * bx - ax * bz) * T::from(2.0).unwrap()) / magnitude;
            translation.0[2] =
                ((az * bw + aw * bz + ax * by - ay * bx) * T::from(2.0).unwrap()) / magnitude;
        } else {
            translation.0[0] = (ax * bw + aw * bx + ay * bz - az * by) * T::from(2.0).unwrap();
            translation.0[1] = (ay * bw + aw * by + az * bx - ax * bz) * T::from(2.0).unwrap();
            translation.0[2] = (az * bw + aw * bz + ax * by - ay * bx) * T::from(2.0).unwrap();
        }
        Self::from_rotation_translation(&a, &translation)
    }

    #[inline(always)]
    pub fn from_rotation_translation_scale(
        q: impl AsRef<Quat2<T>>,
        v: impl AsRef<Vec3<T>>,
        s: impl AsRef<Vec3<T>>,
    ) -> Self {
        let q = q.as_ref();
        let v = v.as_ref();
        let s = s.as_ref();

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

        Self([
            (T::one() - (yy + zz)) * sx,
            (xy + wz) * sx,
            (xz - wy) * sx,
            T::zero(),
            (xy - wz) * sy,
            (T::one() - (xx + zz)) * sy,
            (yz + wx) * sy,
            T::zero(),
            (xz + wy) * sz,
            (yz - wx) * sz,
            (T::one() - (xx + yy)) * sz,
            T::zero(),
            v.0[0],
            v.0[1],
            v.0[2],
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_rotation_translation_scale_origin(
        q: impl AsRef<Quat2<T>>,
        v: impl AsRef<Vec3<T>>,
        s: impl AsRef<Vec3<T>>,
        o: impl AsRef<Vec3<T>>,
    ) -> Self {
        let q = q.as_ref();
        let v = v.as_ref();
        let s = s.as_ref();
        let o = o.as_ref();

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

        let out0 = (T::one() - (yy + zz)) * sx;
        let out1 = (xy + wz) * sx;
        let out2 = (xz - wy) * sx;
        let out4 = (xy - wz) * sy;
        let out5 = (T::one() - (xx + zz)) * sy;
        let out6 = (yz + wx) * sy;
        let out8 = (xz + wy) * sz;
        let out9 = (yz - wx) * sz;
        let out10 = (T::one() - (xx + yy)) * sz;

        Self([
            out0,
            out1,
            out2,
            T::zero(),
            out4,
            out5,
            out6,
            T::zero(),
            out8,
            out9,
            out10,
            T::zero(),
            v.0[0] + ox - (out0 * ox + out4 * oy + out8 * oz),
            v.0[1] + oy - (out1 * ox + out5 * oy + out9 * oz),
            v.0[2] + oz - (out2 * ox + out6 * oy + out10 * oz),
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_quat(q: impl AsRef<Quat<T>>) -> Self {
        let q = q.as_ref();

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

        Self([
            T::one() - yy - zz,
            yx + wz,
            zx - wy,
            T::zero(),
            yx - wz,
            T::one() - xx - zz,
            zy + wx,
            T::zero(),
            zx + wy,
            zy - wx,
            T::one() - xx - yy,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_frustum(left: T, right: T, bottom: T, top: T, near: T, far: T) -> Self {
        let rl = T::one() / (right - left);
        let tb = T::one() / (top - bottom);
        let nf = T::one() / (near - far);

        Self([
            near * T::from(2.0).unwrap() * rl,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            near * T::from(2.0).unwrap() * tb,
            T::zero(),
            T::zero(),
            (right + left) * rl,
            (top + bottom) * tb,
            (far + near) * nf,
            -T::one(),
            T::zero(),
            T::zero(),
            far * near * T::from(2.0).unwrap() * nf,
            T::zero(),
        ])
    }

    #[inline(always)]
    pub fn from_perspective_no(fovy: T, aspect: T, near: T, far: Option<T>) -> Self {
        let mut out = Self::new();

        let f = T::one() / (fovy / T::from(2.0).unwrap()).tan();
        out.0[0] = f / aspect;
        out.0[1] = T::zero();
        out.0[2] = T::zero();
        out.0[3] = T::zero();
        out.0[4] = T::zero();
        out.0[5] = f;
        out.0[6] = T::zero();
        out.0[7] = T::zero();
        out.0[8] = T::zero();
        out.0[9] = T::zero();
        out.0[11] = -T::one();
        out.0[12] = T::zero();
        out.0[13] = T::zero();
        out.0[15] = T::zero();

        match far {
            Some(far) => {
                if far.is_infinite() {
                    out.0[10] = -T::one();
                    out.0[14] = -T::from(2.0).unwrap() * near;
                    out
                } else {
                    let nf = T::one() / (near - far);
                    out.0[10] = (far + near) * nf;
                    out.0[14] = T::from(2.0).unwrap() * far * near * nf;
                    out
                }
            }
            None => {
                out.0[10] = -T::one();
                out.0[14] = -T::from(2.0).unwrap() * near;
                out
            }
        }
    }

    #[inline(always)]
    pub fn from_perspective(fovy: T, aspect: T, near: T, far: Option<T>) -> Self {
        Self::from_perspective_no(fovy, aspect, near, far)
    }

    #[inline(always)]
    pub fn from_perspective_zo(fovy: T, aspect: T, near: T, far: Option<T>) -> Self {
        let mut out = Self::new();

        let f = T::one() / (fovy / T::from(2.0).unwrap()).tan();
        out.0[0] = f / aspect;
        out.0[1] = T::zero();
        out.0[2] = T::zero();
        out.0[3] = T::zero();
        out.0[4] = T::zero();
        out.0[5] = f;
        out.0[6] = T::zero();
        out.0[7] = T::zero();
        out.0[8] = T::zero();
        out.0[9] = T::zero();
        out.0[11] = -T::one();
        out.0[12] = T::zero();
        out.0[13] = T::zero();
        out.0[15] = T::zero();

        match far {
            Some(far) => {
                if far.is_infinite() {
                    out.0[10] = -T::one();
                    out.0[14] = -near;
                    out
                } else {
                    let nf = T::one() / (near - far);
                    out.0[10] = far * nf;
                    out.0[14] = far * near * nf;
                    out
                }
            }
            None => {
                out.0[10] = -T::one();
                out.0[14] = -near;
                out
            }
        }
    }

    #[inline(always)]
    pub fn from_ortho_no(left: T, right: T, bottom: T, top: T, near: T, far: T) -> Self {
        let lr = T::one() / (left - right);
        let bt = T::one() / (bottom - top);
        let nf = T::one() / (near - far);

        Self([
            -T::from(2.0).unwrap() * lr,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            -T::from(2.0).unwrap() * bt,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::from(2.0).unwrap() * nf,
            T::zero(),
            (left + right) * lr,
            (top + bottom) * bt,
            (far + near) * nf,
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_ortho(left: T, right: T, bottom: T, top: T, near: T, far: T) -> Self {
        Self::from_ortho_no(left, right, bottom, top, near, far)
    }

    #[inline(always)]
    pub fn from_ortho_zo(left: T, right: T, bottom: T, top: T, near: T, far: T) -> Self {
        let lr = T::one() / (left - right);
        let bt = T::one() / (bottom - top);
        let nf = T::one() / (near - far);

        Self([
            -T::from(2.0).unwrap() * lr,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            -T::from(2.0).unwrap() * bt,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            nf,
            T::zero(),
            (left + right) * lr,
            (top + bottom) * bt,
            near * nf,
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_look_at(
        eye: impl AsRef<Vec3<T>>,
        center: impl AsRef<Vec3<T>>,
        up: impl AsRef<Vec3<T>>,
    ) -> Self {
        let eye = eye.as_ref();
        let center = center.as_ref();
        let up = up.as_ref();

        let eye_x = eye.0[0];
        let eye_y = eye.0[1];
        let eye_z = eye.0[2];
        let up_x = up.0[0];
        let up_y = up.0[1];
        let up_z = up.0[2];
        let center_x = center.0[0];
        let center_y = center.0[1];
        let center_z = center.0[2];

        if (eye_x - center_x).abs() < T::from(0.000001).unwrap()
            && (eye_y - center_y).abs() < T::from(0.000001).unwrap()
            && (eye_z - center_z).abs() < T::from(0.000001).unwrap()
        {
            return Self::new();
        }

        let mut z0 = eye_x - center_x;
        let mut z1 = eye_y - center_y;
        let mut z2 = eye_z - center_z;

        let mut len = T::one() / (z0 * z0 + z1 * z1 + z2 * z2).sqrt();
        z0 = z0 * len;
        z1 = z1 * len;
        z2 = z2 * len;

        let mut x0 = up_y * z2 - up_z * z1;
        let mut x1 = up_z * z0 - up_x * z2;
        let mut x2 = up_x * z1 - up_y * z0;
        len = (x0 * x0 + x1 * x1 + x2 * x2).sqrt();
        if len == T::zero() {
            x0 = T::zero();
            x1 = T::zero();
            x2 = T::zero();
        } else {
            len = T::one() / len;
            x0 = x0 * len;
            x1 = x1 * len;
            x2 = x2 * len;
        }

        let mut y0 = z1 * x2 - z2 * x1;
        let mut y1 = z2 * x0 - z0 * x2;
        let mut y2 = z0 * x1 - z1 * x0;

        len = (y0 * y0 + y1 * y1 + y2 * y2).sqrt();
        if len == T::zero() {
            y0 = T::zero();
            y1 = T::zero();
            y2 = T::zero();
        } else {
            len = T::one() / len;
            y0 = y0 * len;
            y1 = y1 * len;
            y2 = y2 * len;
        }

        Self([
            x0,
            y0,
            z0,
            T::zero(),
            x1,
            y1,
            z1,
            T::zero(),
            x2,
            y2,
            z2,
            T::zero(),
            -(x0 * eye_x + x1 * eye_y + x2 * eye_z),
            -(y0 * eye_x + y1 * eye_y + y2 * eye_z),
            -(z0 * eye_x + z1 * eye_y + z2 * eye_z),
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_target_to(
        eye: impl AsRef<Vec3<T>>,
        target: impl AsRef<Vec3<T>>,
        up: impl AsRef<Vec3<T>>,
    ) -> Self {
        let eye = eye.as_ref();
        let target = target.as_ref();
        let up = up.as_ref();

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
        if len > T::zero() {
            len = T::one() / len.sqrt();
            z0 = z0 * len;
            z1 = z1 * len;
            z2 = z2 * len;
        }

        let mut x0 = up_y * z2 - up_z * z1;
        let mut x1 = up_z * z0 - up_x * z2;
        let mut x2 = up_x * z1 - up_y * z0;

        len = x0 * x0 + x1 * x1 + x2 * x2;
        if len > T::zero() {
            len = T::one() / len.sqrt();
            x0 = x0 * len;
            x1 = x1 * len;
            x2 = x2 * len;
        }

        Self([
            x0,
            x1,
            x2,
            T::zero(),
            z1 * x2 - z2 * x1,
            z2 * x0 - z0 * x2,
            z0 * x1 - z1 * x0,
            T::zero(),
            z0,
            z1,
            z2,
            T::zero(),
            eye_x,
            eye_y,
            eye_z,
            T::one(),
        ])
    }
}

impl<T: Float> Mat4<T> {
    #[inline(always)]
    pub fn raw(&self) -> &[T; 16] {
        &self.0
    }

    #[inline(always)]
    pub fn set(
        &mut self,
        m00: T,
        m01: T,
        m02: T,
        m03: T,
        m10: T,
        m11: T,
        m12: T,
        m13: T,
        m20: T,
        m21: T,
        m22: T,
        m23: T,
        m30: T,
        m31: T,
        m32: T,
        m33: T,
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

    #[inline(always)]
    pub fn set_slice(&mut self, slice: &[T; 16]) -> &mut Self {
        self.0 = slice.clone();
        self
    }

    #[inline(always)]
    pub fn set_zero(&mut self) -> &mut Self {
        self.0[0] = T::zero();
        self.0[1] = T::zero();
        self.0[2] = T::zero();
        self.0[3] = T::zero();
        self.0[4] = T::zero();
        self.0[5] = T::zero();
        self.0[6] = T::zero();
        self.0[7] = T::zero();
        self.0[8] = T::zero();
        self.0[9] = T::zero();
        self.0[10] = T::zero();
        self.0[11] = T::zero();
        self.0[12] = T::zero();
        self.0[13] = T::zero();
        self.0[14] = T::zero();
        self.0[15] = T::zero();
        self
    }

    #[inline(always)]
    pub fn set_identify(&mut self) -> &mut Self {
        self.0[0] = T::one();
        self.0[1] = T::zero();
        self.0[2] = T::zero();
        self.0[3] = T::zero();
        self.0[4] = T::zero();
        self.0[5] = T::one();
        self.0[6] = T::zero();
        self.0[7] = T::zero();
        self.0[8] = T::zero();
        self.0[9] = T::zero();
        self.0[10] = T::one();
        self.0[11] = T::zero();
        self.0[12] = T::zero();
        self.0[13] = T::zero();
        self.0[14] = T::zero();
        self.0[15] = T::one();
        self
    }

    #[inline(always)]
    pub fn transpose(&self) -> Self {
        Self([
            self.0[0], self.0[4], self.0[8], self.0[12], self.0[1], self.0[5], self.0[9],
            self.0[13], self.0[2], self.0[6], self.0[10], self.0[14], self.0[3], self.0[7],
            self.0[11], self.0[15],
        ])
    }

    #[inline(always)]
    pub fn invert(&self) -> Result<Self, Error> {
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

        if det == T::zero() {
            return Err(Error::ZeroDeterminant);
        }
        det = T::one() / det;

        Ok(Self([
            (a11 * b11 - a12 * b10 + a13 * b09) * det,
            (a02 * b10 - a01 * b11 - a03 * b09) * det,
            (a31 * b05 - a32 * b04 + a33 * b03) * det,
            (a22 * b04 - a21 * b05 - a23 * b03) * det,
            (a12 * b08 - a10 * b11 - a13 * b07) * det,
            (a00 * b11 - a02 * b08 + a03 * b07) * det,
            (a32 * b02 - a30 * b05 - a33 * b01) * det,
            (a20 * b05 - a22 * b02 + a23 * b01) * det,
            (a10 * b10 - a11 * b08 + a13 * b06) * det,
            (a01 * b08 - a00 * b10 - a03 * b06) * det,
            (a30 * b04 - a31 * b02 + a33 * b00) * det,
            (a21 * b02 - a20 * b04 - a23 * b00) * det,
            (a11 * b07 - a10 * b09 - a12 * b06) * det,
            (a00 * b09 - a01 * b07 + a02 * b06) * det,
            (a31 * b01 - a30 * b03 - a32 * b00) * det,
            (a20 * b03 - a21 * b01 + a22 * b00) * det,
        ]))
    }

    #[inline(always)]
    pub fn adjoint(&self) -> Self {
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

        Self([
            a11 * b11 - a12 * b10 + a13 * b09,
            a02 * b10 - a01 * b11 - a03 * b09,
            a31 * b05 - a32 * b04 + a33 * b03,
            a22 * b04 - a21 * b05 - a23 * b03,
            a12 * b08 - a10 * b11 - a13 * b07,
            a00 * b11 - a02 * b08 + a03 * b07,
            a32 * b02 - a30 * b05 - a33 * b01,
            a20 * b05 - a22 * b02 + a23 * b01,
            a10 * b10 - a11 * b08 + a13 * b06,
            a01 * b08 - a00 * b10 - a03 * b06,
            a30 * b04 - a31 * b02 + a33 * b00,
            a21 * b02 - a20 * b04 - a23 * b00,
            a11 * b07 - a10 * b09 - a12 * b06,
            a00 * b09 - a01 * b07 + a02 * b06,
            a31 * b01 - a30 * b03 - a32 * b00,
            a20 * b03 - a21 * b01 + a22 * b00,
        ])
    }

    #[inline(always)]
    pub fn determinant(&self) -> T {
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

    #[inline(always)]
    pub fn translate(&self, v: impl AsRef<Vec3<T>>) -> Self {
        let v = v.as_ref();

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

        Self([
            a00,
            a01,
            a02,
            a03,
            a10,
            a11,
            a12,
            a13,
            a20,
            a21,
            a22,
            a23,
            a00 * x + a10 * y + a20 * z + self.0[12],
            a01 * x + a11 * y + a21 * z + self.0[13],
            a02 * x + a12 * y + a22 * z + self.0[14],
            a03 * x + a13 * y + a23 * z + self.0[15],
        ])
    }

    #[inline(always)]
    pub fn scale(&self, v: impl AsRef<Vec3<T>>) -> Self {
        let v = v.as_ref();

        let x = v.0[0];
        let y = v.0[1];
        let z = v.0[2];

        Self([
            self.0[0] * x,
            self.0[1] * x,
            self.0[2] * x,
            self.0[3] * x,
            self.0[4] * y,
            self.0[5] * y,
            self.0[6] * y,
            self.0[7] * y,
            self.0[8] * z,
            self.0[9] * z,
            self.0[10] * z,
            self.0[11] * z,
            self.0[12],
            self.0[13],
            self.0[14],
            self.0[15],
        ])
    }

    #[inline(always)]
    pub fn rotate(&self, axis: impl AsRef<Vec3<T>>, rad: T) -> Result<Self, Error> {
        let axis = axis.as_ref();

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

        if len < T::from(0.000001).unwrap() {
            return Err(Error::LengthSmallerThanEpsilon);
        }

        len = T::one() / len;
        x = x * len;
        y = y * len;
        z = z * len;

        s = rad.sin();
        c = rad.cos();
        t = T::one() - c;

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

        Ok(Self([
            // Perform rotation-specific matrix multiplication
            a00 * b00 + a10 * b01 + a20 * b02,
            a01 * b00 + a11 * b01 + a21 * b02,
            a02 * b00 + a12 * b01 + a22 * b02,
            a03 * b00 + a13 * b01 + a23 * b02,
            a00 * b10 + a10 * b11 + a20 * b12,
            a01 * b10 + a11 * b11 + a21 * b12,
            a02 * b10 + a12 * b11 + a22 * b12,
            a03 * b10 + a13 * b11 + a23 * b12,
            a00 * b20 + a10 * b21 + a20 * b22,
            a01 * b20 + a11 * b21 + a21 * b22,
            a02 * b20 + a12 * b21 + a22 * b22,
            a03 * b20 + a13 * b21 + a23 * b22,
            // If the source and destination differ, copy the unchanged last row
            self.0[12],
            self.0[13],
            self.0[14],
            self.0[15],
        ]))
    }

    #[inline(always)]
    pub fn rotate_x(&self, rad: T) -> Self {
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

    #[inline(always)]
    pub fn rotate_y(&self, rad: T) -> Self {
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

    #[inline(always)]
    pub fn rotate_z(&self, rad: T) -> Self {
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

        Self([
            // Perform axis-specific matrix multiplication
            a00 * c + a10 * s,
            a01 * c + a11 * s,
            a02 * c + a12 * s,
            a03 * c + a13 * s,
            a10 * c - a00 * s,
            a11 * c - a01 * s,
            a12 * c - a02 * s,
            a13 * c - a03 * s,
            self.0[8],
            self.0[9],
            self.0[10],
            self.0[11],
            self.0[12],
            self.0[13],
            self.0[14],
            self.0[15],
        ])
    }

    #[inline(always)]
    pub fn decompose(&self) -> (Quat<T>, Vec3<T>, Vec3<T>) {
        let mut out_r = Quat::<T>::new();
        let mut out_t = Vec3::<T>::new();
        let mut out_s = Vec3::<T>::new();

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

        let is1 = T::one() / out_s.0[0];
        let is2 = T::one() / out_s.0[1];
        let is3 = T::one() / out_s.0[2];

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

        if trace > T::zero() {
            let s = (trace + T::one()).sqrt() * T::from(2.0).unwrap();
            out_r.0[3] = T::from(0.25).unwrap() * s;
            out_r.0[0] = (sm23 - sm32) / s;
            out_r.0[1] = (sm31 - sm13) / s;
            out_r.0[2] = (sm12 - sm21) / s;
        } else if sm11 > sm22 && sm11 > sm33 {
            let s = (T::one() + sm11 - sm22 - sm33).sqrt() * T::from(2.0).unwrap();
            out_r.0[3] = (sm23 - sm32) / s;
            out_r.0[0] = T::from(0.25).unwrap() * s;
            out_r.0[1] = (sm12 + sm21) / s;
            out_r.0[2] = (sm31 + sm13) / s;
        } else if sm22 > sm33 {
            let s = (T::one() + sm22 - sm11 - sm33).sqrt() * T::from(2.0).unwrap();
            out_r.0[3] = (sm31 - sm13) / s;
            out_r.0[0] = (sm12 + sm21) / s;
            out_r.0[1] = T::from(0.25).unwrap() * s;
            out_r.0[2] = (sm23 + sm32) / s;
        } else {
            let s = (T::one() + sm33 - sm11 - sm22).sqrt() * T::from(2.0).unwrap();
            out_r.0[3] = (sm12 - sm21) / s;
            out_r.0[0] = (sm31 + sm13) / s;
            out_r.0[1] = (sm23 + sm32) / s;
            out_r.0[2] = T::from(0.25).unwrap() * s;
        }

        (out_r, out_t, out_s)
    }

    #[inline(always)]
    pub fn frob(&self) -> T {
        (self.0[0] * self.0[0]
            + self.0[1] * self.0[1]
            + self.0[2] * self.0[2]
            + self.0[3] * self.0[3]
            + self.0[4] * self.0[4]
            + self.0[5] * self.0[5]
            + self.0[6] * self.0[6]
            + self.0[7] * self.0[7]
            + self.0[8] * self.0[8]
            + self.0[9] * self.0[9]
            + self.0[10] * self.0[10]
            + self.0[11] * self.0[11]
            + self.0[12] * self.0[12]
            + self.0[13] * self.0[13]
            + self.0[14] * self.0[14]
            + self.0[15] * self.0[15])
            .sqrt()
    }

    #[inline(always)]
    pub fn translation(&self) -> Vec3<T> {
        let mut out = Vec3::<T>::new();

        out.0[0] = self.0[12];
        out.0[1] = self.0[13];
        out.0[2] = self.0[14];

        out
    }

    #[inline(always)]
    pub fn scaling(&self) -> Vec3<T> {
        let mut out = Vec3::<T>::new();

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

    #[inline(always)]
    pub fn rotation(&self) -> Quat<T> {
        let mut out = Quat::<T>::new();

        let scaling = self.scaling();

        let is1 = T::one() / scaling.0[0];
        let is2 = T::one() / scaling.0[1];
        let is3 = T::one() / scaling.0[2];

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

        if trace > T::zero() {
            let s = (trace + T::one()).sqrt() * T::from(2.0).unwrap();
            out.0[3] = T::from(0.25).unwrap() * s;
            out.0[0] = (sm23 - sm32) / s;
            out.0[1] = (sm31 - sm13) / s;
            out.0[2] = (sm12 - sm21) / s;
        } else if sm11 > sm22 && sm11 > sm33 {
            let s = (T::one() + sm11 - sm22 - sm33).sqrt() * T::from(2.0).unwrap();
            out.0[3] = (sm23 - sm32) / s;
            out.0[0] = T::from(0.25).unwrap() * s;
            out.0[1] = (sm12 + sm21) / s;
            out.0[2] = (sm31 + sm13) / s;
        } else if sm22 > sm33 {
            let s = (T::one() + sm22 - sm11 - sm33).sqrt() * T::from(2.0).unwrap();
            out.0[3] = (sm31 - sm13) / s;
            out.0[0] = (sm12 + sm21) / s;
            out.0[1] = T::from(0.25).unwrap() * s;
            out.0[2] = (sm23 + sm32) / s;
        } else {
            let s = (T::one() + sm33 - sm11 - sm22).sqrt() * T::from(2.0).unwrap();
            out.0[3] = (sm12 - sm21) / s;
            out.0[0] = (sm31 + sm13) / s;
            out.0[1] = (sm23 + sm32) / s;
            out.0[2] = T::from(0.25).unwrap() * s;
        }

        out
    }

    /// Returns whether or not the matrices have approximately the same elements in the same position.
    ///
    /// Refers to `equals` function in `glMatrix`. `exactEquals` is implemented with [`PartialEq`] and [`Eq`],
    #[inline(always)]
    pub fn approximate_eq(&self, b: impl AsRef<Self>) -> bool {
        let b = b.as_ref();

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

        (a0 - b0).abs() <= T::from(0.000001).unwrap() * T::one().max(a0.abs()).max(b0.abs())
            && (a1 - b1).abs() <= T::from(0.000001).unwrap() * T::one().max(a1.abs()).max(b1.abs())
            && (a2 - b2).abs() <= T::from(0.000001).unwrap() * T::one().max(a2.abs()).max(b2.abs())
            && (a3 - b3).abs() <= T::from(0.000001).unwrap() * T::one().max(a3.abs()).max(b3.abs())
            && (a4 - b4).abs() <= T::from(0.000001).unwrap() * T::one().max(a4.abs()).max(b4.abs())
            && (a5 - b5).abs() <= T::from(0.000001).unwrap() * T::one().max(a5.abs()).max(b5.abs())
            && (a6 - b6).abs() <= T::from(0.000001).unwrap() * T::one().max(a6.abs()).max(b6.abs())
            && (a7 - b7).abs() <= T::from(0.000001).unwrap() * T::one().max(a7.abs()).max(b7.abs())
            && (a8 - b8).abs() <= T::from(0.000001).unwrap() * T::one().max(a8.abs()).max(b8.abs())
            && (a9 - b9).abs() <= T::from(0.000001).unwrap() * T::one().max(a9.abs()).max(b9.abs())
            && (a10 - b10).abs()
                <= T::from(0.000001).unwrap() * T::one().max(a10.abs()).max(b10.abs())
            && (a11 - b11).abs()
                <= T::from(0.000001).unwrap() * T::one().max(a11.abs()).max(b11.abs())
            && (a12 - b12).abs()
                <= T::from(0.000001).unwrap() * T::one().max(a12.abs()).max(b12.abs())
            && (a13 - b13).abs()
                <= T::from(0.000001).unwrap() * T::one().max(a13.abs()).max(b13.abs())
            && (a14 - b14).abs()
                <= T::from(0.000001).unwrap() * T::one().max(a14.abs()).max(b14.abs())
            && (a15 - b15).abs()
                <= T::from(0.000001).unwrap() * T::one().max(a15.abs()).max(b15.abs())
    }
}

impl<T: Float + FloatConst> Mat4<T> {
    #[inline(always)]
    pub fn from_perspective_from_field_of_view(
        fov_left_degrees: T,
        fov_right_degrees: T,
        fov_down_degrees: T,
        fov_up_degrees: T,
        near: T,
        far: T,
    ) -> Self {
        let up_tan = ((fov_up_degrees * T::PI()) / T::from(180.0).unwrap()).tan();
        let down_tan = ((fov_down_degrees * T::PI()) / T::from(180.0).unwrap()).tan();
        let left_tan = ((fov_left_degrees * T::PI()) / T::from(180.0).unwrap()).tan();
        let right_tan = ((fov_right_degrees * T::PI()) / T::from(180.0).unwrap()).tan();
        let x_scale = T::from(2.0).unwrap() / (left_tan + right_tan);
        let y_scale = T::from(2.0).unwrap() / (up_tan + down_tan);

        Self([
            x_scale,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            y_scale,
            T::zero(),
            T::zero(),
            -((left_tan - right_tan) * x_scale * T::from(0.5).unwrap()),
            (up_tan - down_tan) * y_scale * T::from(0.5).unwrap(),
            far / (near - far),
            -T::one(),
            T::zero(),
            T::zero(),
            (far * near) / (near - far),
            T::zero(),
        ])
    }
}

impl<T: Float> Add<Mat4<T>> for Mat4<T> {
    type Output = Self;

    #[inline(always)]
    fn add(self, b: Self) -> Self {
        let mut out = Mat4::<T>::new_identity();
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

impl<T: Float> Sub<Mat4<T>> for Mat4<T> {
    type Output = Self;

    #[inline(always)]
    fn sub(self, b: Self) -> Self {
        let mut out = Mat4::<T>::new_identity();
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

impl<T: Float> Mul<Mat4<T>> for Mat4<T> {
    type Output = Self;

    #[inline(always)]
    fn mul(self, b: Self) -> Self {
        let mut out = Mat4::<T>::new_identity();
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

impl<T: Float> Mul<T> for Mat4<T> {
    type Output = Self;

    #[inline(always)]
    fn mul(self, b: T) -> Self {
        let mut out = Mat4::<T>::new_identity();
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

impl<T> AsRef<Mat4<T>> for Mat4<T> {
    fn as_ref(&self) -> &Self {
        self
    }
}

impl AsRef<[u8]> for Mat4<f64> {
    fn as_ref(&self) -> &[u8] {
        unsafe { std::mem::transmute::<&[f64; 16], &[u8; 128]>(&self.0) }
    }
}

impl AsRef<[u8]> for Mat4<f32> {
    fn as_ref(&self) -> &[u8] {
        unsafe { std::mem::transmute::<&[f32; 16], &[u8; 64]>(&self.0) }
    }
}

impl AsRef<[u8]> for Mat4<f16> {
    fn as_ref(&self) -> &[u8] {
        unsafe { std::mem::transmute::<&[f16; 16], &[u8; 32]>(&self.0) }
    }
}

impl<T: Float> Default for Mat4<T> {
    fn default() -> Self {
        Self::new_identity()
    }
}

impl<T: Display> Display for Mat4<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let value = self
            .0
            .iter()
            .map(|v| v.to_string())
            .collect::<Vec<_>>()
            .join(", ");
        f.write_fmt(format_args!("mat4({})", value))
    }
}

/// tests only for f32
#[cfg(test)]
mod tests {
    use std::sync::OnceLock;

    use crate::{error::Error, vec3::Vec3};

    use super::Mat4;

    static MAT_A_RAW: [f32; 16] = [
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
    ];
    static MAT_B_RAW: [f32; 16] = [
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 4.0, 5.0, 6.0, 1.0,
    ];
    static MAT_IDENTITY_RAW: [f32; 16] = [
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ];

    static MAT_A: OnceLock<Mat4> = OnceLock::new();
    static MAT_B: OnceLock<Mat4> = OnceLock::new();
    static MAT_IDENTITY: OnceLock<Mat4> = OnceLock::new();

    fn mat_a() -> &'static Mat4 {
        MAT_A.get_or_init(|| Mat4::from_slice(&MAT_A_RAW))
    }

    fn mat_b() -> &'static Mat4 {
        MAT_B.get_or_init(|| Mat4::from_slice(&MAT_B_RAW))
    }

    fn mat_identity() -> &'static Mat4 {
        MAT_IDENTITY.get_or_init(|| Mat4::from_slice(&MAT_IDENTITY_RAW))
    }

    #[test]
    fn new() {
        assert_eq!(
            Mat4::<f32>::new().raw(),
            &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        );
    }

    #[test]
    fn new_identity() {
        assert_eq!(Mat4::<f32>::new_identity().raw(), &MAT_IDENTITY_RAW);
    }

    #[test]
    fn from_slice() {
        assert_eq!(
            Mat4::from_slice(&[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ])
            .raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn from_values() {
        assert_eq!(
            Mat4::from_values(
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0,
            )
            .raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn from_frustum() {
        assert_eq!(
            Mat4::from_frustum(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0).raw(),
            &[-1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0]
        );
    }

    #[test]
    fn from_look_at() {
        let out = Mat4::from_look_at(
            Vec3::from_values(0.0, 0.0, 1.0),
            Vec3::from_values(0.0, 0.0, -1.0),
            Vec3::from_values(0.0, 1.0, 0.0),
        );

        assert_eq!(
            out.raw(),
            &[1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0, 1.0]
        );
    }

    #[test]
    fn from_target_to() {
        let out = Mat4::from_target_to(
            Vec3::from_values(0.0, 0.0, 1.0),
            Vec3::from_values(0.0, 0.0, -1.0),
            Vec3::from_values(0.0, 1.0, 0.0),
        );

        assert_eq!(
            out.raw(),
            &[1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0]
        );

        let scaling = out.scaling();
        assert_eq!(scaling.raw(), &[1.0, 1.0, 1.0]);
    }

    #[test]
    fn transpose() {
        assert_eq!(
            mat_a().transpose().raw(),
            &[1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 2.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0, 1.0]
        );
    }

    #[test]
    fn invert() -> Result<(), Error> {
        assert_eq!(
            mat_a().invert()?.raw(),
            &[1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, -2.0, -3.0, 1.0]
        );

        Ok(())
    }

    #[test]
    fn adjoint() {
        assert_eq!(
            mat_a().adjoint().raw(),
            &[1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, -2.0, -3.0, 1.0]
        );
    }

    #[test]
    fn determinant() {
        assert_eq!(mat_a().determinant(), 1.0);
    }

    #[test]
    fn translate() {
        let out = mat_a().translate(Vec3::from_values(4.0, 5.0, 6.0));
        assert_eq!(
            out.raw(),
            &[1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 5.0, 7.0, 9.0, 1.0]
        );
    }

    #[test]
    fn scale() {
        let out = mat_a().scale(Vec3::from_values(4.0, 5.0, 6.0));
        assert_eq!(
            out.raw(),
            &[4.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 1.0, 2.0, 3.0, 1.0]
        );
    }

    #[test]
    fn rotate_x() {
        let rad = std::f32::consts::PI * 0.5;

        let out = mat_a().rotate_x(rad);
        assert_eq!(
            out.raw(),
            &[
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                rad.cos(),
                rad.sin(),
                0.0,
                0.0,
                -rad.sin(),
                rad.cos(),
                0.0,
                1.0,
                2.0,
                3.0,
                1.0
            ]
        );
    }

    #[test]
    fn rotate_y() {
        let rad = std::f32::consts::PI * 0.5;

        let out = mat_a().rotate_y(rad);
        assert_eq!(
            out.raw(),
            &[
                rad.cos(),
                0.0,
                -rad.sin(),
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                rad.sin(),
                0.0,
                rad.cos(),
                0.0,
                1.0,
                2.0,
                3.0,
                1.0
            ]
        );
    }

    #[test]
    fn rotate_z() {
        let rad = std::f32::consts::PI * 0.5;

        let out = mat_a().rotate_z(rad);
        assert_eq!(
            out.raw(),
            &[
                rad.cos(),
                rad.sin(),
                0.0,
                0.0,
                -rad.sin(),
                rad.cos(),
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                1.0,
                2.0,
                3.0,
                1.0
            ]
        );
    }

    #[test]
    fn translation() {
        let translation = mat_identity().translation();
        assert_eq!(translation.raw(), &[0.0, 0.0, 0.0]);
    }

    #[test]
    fn scaling() {
        let scaling = mat_identity().scaling();
        assert_eq!(scaling.raw(), &[1.0, 1.0, 1.0]);
    }

    #[test]
    fn rotation() {
        let rotation = mat_identity().rotation();
        assert_eq!(rotation.raw(), &[0.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn decompose() {
        let (out_r, out_t, out_s) = mat_identity().decompose();
        assert_eq!(out_r.raw(), &[0.0, 0.0, 0.0, 1.0]);
        assert_eq!(out_t.raw(), &[0.0, 0.0, 0.0]);
        assert_eq!(out_s.raw(), &[1.0, 1.0, 1.0]);
    }

    #[test]
    fn frob() {
        let out = mat_a().frob();
        assert_eq!(
            out,
            (1.0f32.powi(2)
                + 1.0f32.powi(2)
                + 1.0f32.powi(2)
                + 1.0f32.powi(2)
                + 1.0f32.powi(2)
                + 2.0f32.powi(2)
                + 3.0f32.powi(2))
            .sqrt()
        );
    }

    #[test]
    fn set() {
        let mut mat = Mat4::new();
        mat.set(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );

        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn set_slice() {
        let mut mat = Mat4::new();
        mat.set_slice(&[
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        ]);

        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn add() {
        let mat_a = Mat4::from_values(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        let mat_b = Mat4::from_values(
            17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0,
            31.0, 32.0,
        );

        assert_eq!(
            (mat_a + mat_b).raw(),
            &[
                18.0, 20.0, 22.0, 24.0, 26.0, 28.0, 30.0, 32.0, 34.0, 36.0, 38.0, 40.0, 42.0, 44.0,
                46.0, 48.0
            ]
        );
    }

    #[test]
    fn sub() {
        let mat_a = Mat4::from_values(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        let mat_b = Mat4::from_values(
            17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0,
            31.0, 32.0,
        );

        assert_eq!(
            (mat_a - mat_b).raw(),
            &[
                -16.0, -16.0, -16.0, -16.0, -16.0, -16.0, -16.0, -16.0, -16.0, -16.0, -16.0, -16.0,
                -16.0, -16.0, -16.0, -16.0
            ]
        );
    }

    #[test]
    fn mul() {
        let out = *mat_a() * *mat_b();
        assert_eq!(
            out.raw(),
            &[1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 5.0, 7.0, 9.0, 1.0]
        );
    }

    #[test]
    fn mul_scalar() {
        let mat = Mat4::from_values(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );

        assert_eq!(
            (mat * 2.0).raw(),
            &[
                2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0, 22.0, 24.0, 26.0, 28.0,
                30.0, 32.0
            ]
        );
    }

    #[test]
    fn mul_scalar_add() {
        let mat_a = Mat4::from_values(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        let mat_b = Mat4::from_values(
            17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0,
            31.0, 32.0,
        );

        assert_eq!(
            (mat_a + mat_b * 0.5).raw(),
            &[
                9.5, 11.0, 12.5, 14.0, 15.5, 17.0, 18.5, 20.0, 21.5, 23.0, 24.5, 26.0, 27.5, 29.0,
                30.5, 32.0
            ]
        );
    }

    #[test]
    fn approximate_eq() {
        let mat_a = Mat4::from_values(
            0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
        );
        let mat_b = Mat4::from_values(
            0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
        );
        let mat_c = Mat4::from_values(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        let mat_d = Mat4::from_values(
            1e-16, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
        );

        assert_eq!(true, mat_a.approximate_eq(mat_b));
        assert_eq!(false, mat_a.approximate_eq(mat_c));
        assert_eq!(true, mat_a.approximate_eq(mat_d));
    }

    #[test]
    fn display() {
        let out = mat_a().to_string();
        assert_eq!(out, "mat4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 2, 3, 1)");
    }

    #[test]
    fn rotate() -> Result<(), Error> {
        let rad = std::f32::consts::PI * 0.5;
        let axis = Vec3::<f32>::from_values(1.0, 0.0, 0.0);

        let out = mat_a().rotate(axis, rad)?;
        assert_eq!(
            out.raw(),
            &[
                0.99999994,
                0.0,
                0.0,
                0.0,
                0.0,
                rad.cos(),
                rad.sin(),
                0.0,
                0.0,
                -rad.sin(),
                rad.cos(),
                0.0,
                1.0,
                2.0,
                3.0,
                1.0
            ]
        );

        Ok(())
    }

    #[test]
    fn from_perspective_no() {
        assert_eq!(
            Mat4::<f32>::from_perspective_no(std::f32::consts::PI * 0.5, 1.0, 0.0, Some(1.0)).raw(),
            &[1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0]
        );

        assert_eq!(
            Mat4::<f32>::from_perspective_no(
                std::f32::consts::PI * 45.0 / 180.0,
                640.0 / 480.0,
                0.1,
                Some(200.0)
            )
            .raw(),
            &[
                1.81066,
                0.0,
                0.0,
                0.0,
                0.0,
                2.4142134,
                0.0,
                0.0,
                0.0,
                0.0,
                -1.0010005,
                -1.0,
                0.0,
                0.0,
                -0.20010006,
                0.0
            ]
        );

        assert_eq!(
            Mat4::<f32>::from_perspective_no(
                std::f32::consts::PI * 45.0 / 180.0,
                640.0 / 480.0,
                0.1,
                None
            )
            .raw(),
            &[
                1.81066, 0.0, 0.0, 0.0, 0.0, 2.4142134, 0.0, 0.0, 0.0, 0.0, -1.0, -1.0, 0.0, 0.0,
                -0.2, 0.0
            ]
        );

        assert_eq!(
            Mat4::<f32>::from_perspective_no(
                std::f32::consts::PI * 45.0 / 180.0,
                640.0 / 480.0,
                0.1,
                Some(f32::INFINITY)
            )
            .raw(),
            &[
                1.81066, 0.0, 0.0, 0.0, 0.0, 2.4142134, 0.0, 0.0, 0.0, 0.0, -1.0, -1.0, 0.0, 0.0,
                -0.2, 0.0
            ]
        );
    }

    #[test]
    fn test_u8_slice() {
        let bin: &[u8] = mat_a().as_ref();
        bin.chunks(4).enumerate().for_each(|(index, bin)| {
            let value = f32::from_ne_bytes(bin.try_into().unwrap());
            assert_eq!(mat_a().0[index], value);
        });
    }
}
