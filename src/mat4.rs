use std::{
    fmt::{Debug, Display},
    ops::{Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign},
    slice::SliceIndex,
};

use crate::{error::Error, quat::Quat, vec3::Vec3, ApproximateEq};

pub struct Mat4<T = f64>([T; 16]);

impl<T: Debug> Debug for Mat4<T> {
    #[inline(always)]
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_tuple("Mat4").field(&self.0).finish()
    }
}

impl<T: Copy> Copy for Mat4<T> {}

impl<T: Clone> Clone for Mat4<T> {
    #[inline(always)]
    fn clone(&self) -> Self {
        Self([
            self.0[0].clone(),
            self.0[1].clone(),
            self.0[2].clone(),
            self.0[3].clone(),
            self.0[4].clone(),
            self.0[5].clone(),
            self.0[6].clone(),
            self.0[7].clone(),
            self.0[8].clone(),
            self.0[9].clone(),
            self.0[10].clone(),
            self.0[11].clone(),
            self.0[12].clone(),
            self.0[13].clone(),
            self.0[14].clone(),
            self.0[15].clone(),
        ])
    }
}

impl<T: PartialEq> PartialEq for Mat4<T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl<T: Display> Display for Mat4<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_fmt(format_args!(
            "mat4({}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {})",
            self.0[0],
            self.0[1],
            self.0[2],
            self.0[3],
            self.0[4],
            self.0[5],
            self.0[6],
            self.0[7],
            self.0[8],
            self.0[9],
            self.0[10],
            self.0[11],
            self.0[12],
            self.0[13],
            self.0[14],
            self.0[15],
        ))
    }
}

impl<T> Mat4<T> {
    #[inline(always)]
    pub const fn new(
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
    pub const fn from_slice(values: [T; 16]) -> Self {
        Self(values)
    }
}

impl<T> Mat4<T> {
    #[inline(always)]
    pub fn raw(&self) -> &[T; 16] {
        &self.0
    }

    #[inline(always)]
    pub fn raw_mut(&mut self) -> &mut [T; 16] {
        &mut self.0
    }

    #[inline(always)]
    pub fn m00(&self) -> &T {
        &self.0[0]
    }

    #[inline(always)]
    pub fn m01(&self) -> &T {
        &self.0[1]
    }

    #[inline(always)]
    pub fn m02(&self) -> &T {
        &self.0[2]
    }

    #[inline(always)]
    pub fn m03(&self) -> &T {
        &self.0[3]
    }

    #[inline(always)]
    pub fn m10(&self) -> &T {
        &self.0[4]
    }

    #[inline(always)]
    pub fn m11(&self) -> &T {
        &self.0[5]
    }

    #[inline(always)]
    pub fn m12(&self) -> &T {
        &self.0[6]
    }

    #[inline(always)]
    pub fn m13(&self) -> &T {
        &self.0[7]
    }

    #[inline(always)]
    pub fn m20(&self) -> &T {
        &self.0[8]
    }

    #[inline(always)]
    pub fn m21(&self) -> &T {
        &self.0[9]
    }

    #[inline(always)]
    pub fn m22(&self) -> &T {
        &self.0[10]
    }

    #[inline(always)]
    pub fn m23(&self) -> &T {
        &self.0[11]
    }

    #[inline(always)]
    pub fn m30(&self) -> &T {
        &self.0[12]
    }

    #[inline(always)]
    pub fn m31(&self) -> &T {
        &self.0[13]
    }

    #[inline(always)]
    pub fn m32(&self) -> &T {
        &self.0[14]
    }

    #[inline(always)]
    pub fn m33(&self) -> &T {
        &self.0[15]
    }

    #[inline(always)]
    pub fn m00_mut(&mut self) -> &mut T {
        &mut self.0[0]
    }

    #[inline(always)]
    pub fn m01_mut(&mut self) -> &mut T {
        &mut self.0[1]
    }

    #[inline(always)]
    pub fn m02_mut(&mut self) -> &mut T {
        &mut self.0[2]
    }

    #[inline(always)]
    pub fn m03_mut(&mut self) -> &mut T {
        &mut self.0[3]
    }

    #[inline(always)]
    pub fn m10_mut(&mut self) -> &mut T {
        &mut self.0[4]
    }

    #[inline(always)]
    pub fn m11_mut(&mut self) -> &mut T {
        &mut self.0[5]
    }

    #[inline(always)]
    pub fn m12_mut(&mut self) -> &mut T {
        &mut self.0[6]
    }

    #[inline(always)]
    pub fn m13_mut(&mut self) -> &mut T {
        &mut self.0[7]
    }

    #[inline(always)]
    pub fn m20_mut(&mut self) -> &mut T {
        &mut self.0[8]
    }

    #[inline(always)]
    pub fn m21_mut(&mut self) -> &mut T {
        &mut self.0[9]
    }

    #[inline(always)]
    pub fn m22_mut(&mut self) -> &mut T {
        &mut self.0[10]
    }

    #[inline(always)]
    pub fn m23_mut(&mut self) -> &mut T {
        &mut self.0[11]
    }

    #[inline(always)]
    pub fn m30_mut(&mut self) -> &mut T {
        &mut self.0[12]
    }

    #[inline(always)]
    pub fn m31_mut(&mut self) -> &mut T {
        &mut self.0[13]
    }

    #[inline(always)]
    pub fn m32_mut(&mut self) -> &mut T {
        &mut self.0[14]
    }

    #[inline(always)]
    pub fn m33_mut(&mut self) -> &mut T {
        &mut self.0[15]
    }

    #[inline(always)]
    pub fn set_m00(&mut self, m00: T) {
        self.0[0] = m00;
    }

    #[inline(always)]
    pub fn set_m01(&mut self, m01: T) {
        self.0[1] = m01;
    }

    #[inline(always)]
    pub fn set_m02(&mut self, m02: T) {
        self.0[2] = m02;
    }

    #[inline(always)]
    pub fn set_m03(&mut self, m03: T) {
        self.0[3] = m03;
    }

    #[inline(always)]
    pub fn set_m10(&mut self, m10: T) {
        self.0[4] = m10;
    }

    #[inline(always)]
    pub fn set_m11(&mut self, m11: T) {
        self.0[5] = m11;
    }

    #[inline(always)]
    pub fn set_m12(&mut self, m12: T) {
        self.0[6] = m12;
    }

    #[inline(always)]
    pub fn set_m13(&mut self, m13: T) {
        self.0[7] = m13;
    }

    #[inline(always)]
    pub fn set_m20(&mut self, m20: T) {
        self.0[8] = m20;
    }

    #[inline(always)]
    pub fn set_m21(&mut self, m21: T) {
        self.0[9] = m21;
    }

    #[inline(always)]
    pub fn set_m22(&mut self, m22: T) {
        self.0[10] = m22;
    }

    #[inline(always)]
    pub fn set_m23(&mut self, m23: T) {
        self.0[11] = m23;
    }

    #[inline(always)]
    pub fn set_m30(&mut self, m30: T) {
        self.0[12] = m30;
    }

    #[inline(always)]
    pub fn set_m31(&mut self, m31: T) {
        self.0[13] = m31;
    }

    #[inline(always)]
    pub fn set_m32(&mut self, m32: T) {
        self.0[14] = m32;
    }

    #[inline(always)]
    pub fn set_m33(&mut self, m33: T) {
        self.0[15] = m33;
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
    ) {
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
    }
}

impl<T, I> Index<I> for Mat4<T>
where
    I: SliceIndex<[T], Output = T>,
{
    type Output = T;

    #[inline(always)]
    fn index(&self, index: I) -> &Self::Output {
        self.0.index(index)
    }
}

impl<T, I> IndexMut<I> for Mat4<T>
where
    I: SliceIndex<[T], Output = T>,
{
    #[inline(always)]
    fn index_mut(&mut self, index: I) -> &mut Self::Output {
        self.0.index_mut(index)
    }
}

macro_rules! basic_constructors {
    ($(($t: ident, $zero: expr, $one: expr)),+) => {
       $(
        impl Mat4<$t> {
            #[inline(always)]
            pub const fn new_zero() -> Self {
                Self([
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    $zero
                ])
            }

            #[inline(always)]
            pub const fn new_identity() -> Self {
                Self([
                    $one,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    $one,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    $one,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    $one,
                ])
            }
        }
       )+
    };
}

macro_rules! decimal_constructors {
    ($(($t: ident, $epsilon: expr, $zero: expr, $half: expr, $one: expr, $two: expr, $pp: expr, $pi: expr)),+) => {
       $(
        impl Mat4<$t> {

            #[inline(always)]
            pub fn from_translation(v: &Vec3::<$t>) -> Self {
                Self([
                    $one,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    $one,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    $one,
                    $zero,
                    *v.x(),
                    *v.y(),
                    *v.z(),
                    $one,
                ])
            }

            #[inline(always)]
            pub fn from_scaling(v: &Vec3::<$t>) -> Self {
                Self([
                    *v.x(),
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    *v.y(),
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    *v.z(),
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    $one,
                ])
            }

            #[inline(always)]
            pub fn from_rotation(rad: $t, axis: &Vec3::<$t>) -> Result<Self, Error> {
                let mut x = *axis.x();
                let mut y = *axis.y();
                let mut z = *axis.z();
                let mut len = (x * x + y * y + z * z).sqrt();
                let s;
                let c;
                let t;

                if len < $epsilon {
                    return Err(Error::LengthSmallerThanEpsilon);
                }

                len = $one / len;
                x = x * len;
                y = y * len;
                z = z * len;

                s = rad.sin();
                c = rad.cos();
                t = $one - c;

                // Perform rotation-specific matrix multiplication
                Ok(Self::new(
                    x * x * t + c,
                    y * x * t + z * s,
                    z * x * t - y * s,
                    $zero,
                    x * y * t - z * s,
                    y * y * t + c,
                    z * y * t + x * s,
                    $zero,
                    x * z * t + y * s,
                    y * z * t - x * s,
                    z * z * t + c,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    $one,
                ))
            }

            #[inline(always)]
            pub fn from_x_rotation(rad: $t) -> Self {
                let s = rad.sin();
                let c = rad.cos();

                // Perform axis-specific matrix multiplication
                Self([
                    $one,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    c,
                    s,
                    $zero,
                    $zero,
                    -s,
                    c,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    $one,
                ])
            }

            #[inline(always)]
            pub fn from_y_rotation(rad: $t) -> Self {
                let s = rad.sin();
                let c = rad.cos();

                // Perform axis-specific matrix multiplication
                Self([
                    c,
                    $zero,
                    -s,
                    $zero,
                    $zero,
                    $one,
                    $zero,
                    $zero,
                    s,
                    $zero,
                    c,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    $one,
                ])
            }


            #[inline(always)]
            pub fn from_z_rotation(rad: $t) -> Self {
                let s = rad.sin();
                let c = rad.cos();

                // Perform axis-specific matrix multiplication
                Self([
                    c,
                    s,
                    $zero,
                    $zero,
                    -s,
                    c,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    $one,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    $one,
                ])
            }

            #[inline(always)]
            pub fn from_rotation_translation(q: &Quat::<$t>, v: &Vec3::<$t>) -> Self {
                // Quaternion math
                let x = *q.x();
                let y = *q.y();
                let z = *q.z();
                let w = *q.w();
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

                Self::new(
                    $one - (yy + zz),
                    xy + wz,
                    xz - wy,
                    $zero,
                    xy - wz,
                    $one - (xx + zz),
                    yz + wx,
                    $zero,
                    xz + wy,
                    yz - wx,
                    $one - (xx + yy),
                    $zero,
                    *v.x(),
                    *v.y(),
                    *v.z(),
                    $one,
                )
            }

            #[inline(always)]
            pub fn from_quat(q: &Quat::<$t>) -> Self {
                // Quaternion math
                let x = q.x();
                let y = q.y();
                let z = q.z();
                let w = q.w();
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

                Self::new(
                    $one - yy - zz,
                    yx + wz,
                    zx - wy,
                    $zero,
                    yx - wz,
                    $one - xx - zz,
                    zy + wx,
                    $zero,
                    zx + wy,
                    zy - wx,
                    $one - xx - yy,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    $one,
                )
            }

            // #[inline(always)]
            // pub fn from_quat2<Q: AsQuat2<T> + ?Sized>(a: &Q) -> Self {
            //     let mut translation = Vec3::<T>::new();
            //     let bx = -a.x1();
            //     let by = -a.y1();
            //     let bz = -a.z1();
            //     let bw = a.w1();
            //     let ax = a.x2();
            //     let ay = a.y2();
            //     let az = a.z2();
            //     let aw = a.w2();

            //     let magnitude = bx * bx + by * by + bz * bz + bw * bw;
            //     //Only scale if it makes sense
            //     if magnitude > $zero {
            //         translation.0[0] =
            //             ((ax * bw + aw * bx + ay * bz - az * by) * $two) / magnitude;
            //         translation.0[1] =
            //             ((ay * bw + aw * by + az * bx - ax * bz) * $two) / magnitude;
            //         translation.0[2] =
            //             ((az * bw + aw * bz + ax * by - ay * bx) * $two) / magnitude;
            //     } else {
            //         translation.0[0] = (ax * bw + aw * bx + ay * bz - az * by) * $two;
            //         translation.0[1] = (ay * bw + aw * by + az * bx - ax * bz) * $two;
            //         translation.0[2] = (az * bw + aw * bz + ax * by - ay * bx) * $two;
            //     }
            //     Self::from_rotation_translation(&(a.x1(), a.y1(), a.z1(), a.w1()), &translation)
            // }

            #[inline(always)]
            pub fn from_rotation_translation_scale(q: &Quat::<$t>, v: &Vec3::<$t>, s: &Vec3::<$t>) -> Self {
                // Quaternion math
                let x = *q.x();
                let y = *q.y();
                let z = *q.z();
                let w = *q.w();
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
                let sx = *s.x();
                let sy = *s.y();
                let sz = *s.z();

                Self::new(
                    ($one - (yy + zz)) * sx,
                    (xy + wz) * sx,
                    (xz - wy) * sx,
                    $zero,
                    (xy - wz) * sy,
                    ($one - (xx + zz)) * sy,
                    (yz + wx) * sy,
                    $zero,
                    (xz + wy) * sz,
                    (yz - wx) * sz,
                    ($one - (xx + yy)) * sz,
                    $zero,
                    *v.x(),
                    *v.y(),
                    *v.z(),
                    $one,
                )
            }

            #[inline(always)]
            pub fn from_rotation_translation_scale_origin<Q, V1, V2, V3>(
                q: &Quat::<$t>,
                v: &Vec3::<$t>,
                s: &Vec3::<$t>,
                o: &Vec3::<$t>,
            ) -> Self {
                // Quaternion math
                let x = *q.x();
                let y = *q.y();
                let z = *q.z();
                let w = *q.w();
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

                let sx = *s.x();
                let sy = *s.y();
                let sz = *s.z();

                let ox = *o.x();
                let oy = *o.y();
                let oz = *o.z();

                let out0 = ($one - (yy + zz)) * sx;
                let out1 = (xy + wz) * sx;
                let out2 = (xz - wy) * sx;
                let out4 = (xy - wz) * sy;
                let out5 = ($one - (xx + zz)) * sy;
                let out6 = (yz + wx) * sy;
                let out8 = (xz + wy) * sz;
                let out9 = (yz - wx) * sz;
                let out10 = ($one - (xx + yy)) * sz;

                Self([
                    out0,
                    out1,
                    out2,
                    $zero,
                    out4,
                    out5,
                    out6,
                    $zero,
                    out8,
                    out9,
                    out10,
                    $zero,
                    *v.x() + ox - (out0 * ox + out4 * oy + out8 * oz),
                    *v.y() + oy - (out1 * ox + out5 * oy + out9 * oz),
                    *v.z() + oz - (out2 * ox + out6 * oy + out10 * oz),
                    $one,
                ])
            }

            #[inline(always)]
            pub fn from_frustum(left: $t, right: $t, bottom: $t, top: $t, near: $t, far: $t) -> Self {
                let rl = $one / (right - left);
                let tb = $one / (top - bottom);
                let nf = $one / (near - far);

                Self([
                    near * $two * rl,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    near * $two * tb,
                    $zero,
                    $zero,
                    (right + left) * rl,
                    (top + bottom) * tb,
                    (far + near) * nf,
                    -$one,
                    $zero,
                    $zero,
                    far * near * $two * nf,
                    $zero,
                ])
            }

            #[inline(always)]
            pub fn from_perspective_no(fovy: $t, aspect: $t, near: $t, far: Option<$t>) -> Self {
                let mut out = Self::new_zero();

                let f = $one / (fovy / $two).tan();
                out.0[0] = f / aspect;
                out.0[1] = $zero;
                out.0[2] = $zero;
                out.0[3] = $zero;
                out.0[4] = $zero;
                out.0[5] = f;
                out.0[6] = $zero;
                out.0[7] = $zero;
                out.0[8] = $zero;
                out.0[9] = $zero;
                out.0[11] = -$one;
                out.0[12] = $zero;
                out.0[13] = $zero;
                out.0[15] = $zero;

                match far {
                    Some(far) => {
                        if far.is_infinite() {
                            out.0[10] = -$one;
                            out.0[14] = -$two * near;
                            out
                        } else {
                            let nf = $one / (near - far);
                            out.0[10] = (far + near) * nf;
                            out.0[14] = $two * far * near * nf;
                            out
                        }
                    }
                    None => {
                        out.0[10] = -$one;
                        out.0[14] = -$two * near;
                        out
                    }
                }
            }

            #[inline(always)]
            pub fn from_perspective(fovy: $t, aspect: $t, near: $t, far: Option<$t>) -> Self {
                Self::from_perspective_no(fovy, aspect, near, far)
            }

            #[inline(always)]
            pub fn from_perspective_zo(fovy: $t, aspect: $t, near: $t, far: Option<$t>) -> Self {
                let mut out = Self::new_zero();

                let f = $one / (fovy / $two).tan();
                out.0[0] = f / aspect;
                out.0[1] = $zero;
                out.0[2] = $zero;
                out.0[3] = $zero;
                out.0[4] = $zero;
                out.0[5] = f;
                out.0[6] = $zero;
                out.0[7] = $zero;
                out.0[8] = $zero;
                out.0[9] = $zero;
                out.0[11] = -$one;
                out.0[12] = $zero;
                out.0[13] = $zero;
                out.0[15] = $zero;

                match far {
                    Some(far) => {
                        if far.is_infinite() {
                            out.0[10] = -$one;
                            out.0[14] = -near;
                            out
                        } else {
                            let nf = $one / (near - far);
                            out.0[10] = far * nf;
                            out.0[14] = far * near * nf;
                            out
                        }
                    }
                    None => {
                        out.0[10] = -$one;
                        out.0[14] = -near;
                        out
                    }
                }
            }

            #[inline(always)]
            pub fn from_ortho_no(left: $t, right: $t, bottom: $t, top: $t, near: $t, far: $t) -> Self {
                let lr = $one / (left - right);
                let bt = $one / (bottom - top);
                let nf = $one / (near - far);

                Self([
                    -$two * lr,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    -$two * bt,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    $two * nf,
                    $zero,
                    (left + right) * lr,
                    (top + bottom) * bt,
                    (far + near) * nf,
                    $one,
                ])
            }

            #[inline(always)]
            pub fn from_ortho(left: $t, right: $t, bottom: $t, top: $t, near: $t, far: $t) -> Self {
                Self::from_ortho_no(left, right, bottom, top, near, far)
            }

            #[inline(always)]
            pub fn from_ortho_zo(left: $t, right: $t, bottom: $t, top: $t, near: $t, far: $t) -> Self {
                let lr = $one / (left - right);
                let bt = $one / (bottom - top);
                let nf = $one / (near - far);

                Self([
                    -$two * lr,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    -$two * bt,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    nf,
                    $zero,
                    (left + right) * lr,
                    (top + bottom) * bt,
                    near * nf,
                    $one,
                ])
            }

            #[inline(always)]
            pub fn from_look_at(eye: &Vec3::<$t>, center: &Vec3::<$t>, up: &Vec3::<$t>) -> Self {
                let eye_x = eye.x();
                let eye_y = eye.y();
                let eye_z = eye.z();
                let up_x = up.x();
                let up_y = up.y();
                let up_z = up.z();
                let center_x = center.x();
                let center_y = center.y();
                let center_z = center.z();

                if (eye_x - center_x).abs() < $epsilon
                    && (eye_y - center_y).abs() < $epsilon
                    && (eye_z - center_z).abs() < $epsilon
                {
                    return Self::new_zero();
                }

                let mut z0 = eye_x - center_x;
                let mut z1 = eye_y - center_y;
                let mut z2 = eye_z - center_z;

                let mut len = $one / (z0 * z0 + z1 * z1 + z2 * z2).sqrt();
                z0 = z0 * len;
                z1 = z1 * len;
                z2 = z2 * len;

                let mut x0 = up_y * z2 - up_z * z1;
                let mut x1 = up_z * z0 - up_x * z2;
                let mut x2 = up_x * z1 - up_y * z0;
                len = (x0 * x0 + x1 * x1 + x2 * x2).sqrt();
                if len == $zero {
                    x0 = $zero;
                    x1 = $zero;
                    x2 = $zero;
                } else {
                    len = $one / len;
                    x0 = x0 * len;
                    x1 = x1 * len;
                    x2 = x2 * len;
                }

                let mut y0 = z1 * x2 - z2 * x1;
                let mut y1 = z2 * x0 - z0 * x2;
                let mut y2 = z0 * x1 - z1 * x0;

                len = (y0 * y0 + y1 * y1 + y2 * y2).sqrt();
                if len == $zero {
                    y0 = $zero;
                    y1 = $zero;
                    y2 = $zero;
                } else {
                    len = $one / len;
                    y0 = y0 * len;
                    y1 = y1 * len;
                    y2 = y2 * len;
                }

                Self([
                    x0,
                    y0,
                    z0,
                    $zero,
                    x1,
                    y1,
                    z1,
                    $zero,
                    x2,
                    y2,
                    z2,
                    $zero,
                    -(x0 * eye_x + x1 * eye_y + x2 * eye_z),
                    -(y0 * eye_x + y1 * eye_y + y2 * eye_z),
                    -(z0 * eye_x + z1 * eye_y + z2 * eye_z),
                    $one,
                ])
            }

            #[inline(always)]
            pub fn from_target_to(eye: &Vec3::<$t>, target: &Vec3::<$t>, up: &Vec3::<$t>) -> Self {
                let eye_x = *eye.x();
                let eye_y = *eye.y();
                let eye_z = *eye.z();
                let up_x = *up.x();
                let up_y = *up.y();
                let up_z = *up.z();

                let mut z0 = eye_x - target.x();
                let mut z1 = eye_y - target.y();
                let mut z2 = eye_z - target.z();

                let mut len = z0 * z0 + z1 * z1 + z2 * z2;
                if len > $zero {
                    len = $one / len.sqrt();
                    z0 = z0 * len;
                    z1 = z1 * len;
                    z2 = z2 * len;
                }

                let mut x0 = up_y * z2 - up_z * z1;
                let mut x1 = up_z * z0 - up_x * z2;
                let mut x2 = up_x * z1 - up_y * z0;

                len = x0 * x0 + x1 * x1 + x2 * x2;
                if len > $zero {
                    len = $one / len.sqrt();
                    x0 = x0 * len;
                    x1 = x1 * len;
                    x2 = x2 * len;
                }

                Self([
                    x0,
                    x1,
                    x2,
                    $zero,
                    z1 * x2 - z2 * x1,
                    z2 * x0 - z0 * x2,
                    z0 * x1 - z1 * x0,
                    $zero,
                    z0,
                    z1,
                    z2,
                    $zero,
                    eye_x,
                    eye_y,
                    eye_z,
                    $one,
                ])
            }

            #[inline(always)]
            pub fn from_perspective_from_field_of_view(
                fov_left_degrees: $t,
                fov_right_degrees: $t,
                fov_down_degrees: $t,
                fov_up_degrees: $t,
                near: $t,
                far: $t,
            ) -> Self {
                let up_tan = ((fov_up_degrees * $pi) / $pp).tan();
                let down_tan = ((fov_down_degrees * $pi) / $pp).tan();
                let left_tan = ((fov_left_degrees * $pi) / $pp).tan();
                let right_tan = ((fov_right_degrees * $pi) / $pp).tan();
                let x_scale = $two / (left_tan + right_tan);
                let y_scale = $two / (up_tan + down_tan);

                Self([
                    x_scale,
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                    y_scale,
                    $zero,
                    $zero,
                    -((left_tan - right_tan) * x_scale * $half),
                    (up_tan - down_tan) * y_scale * $half,
                    far / (near - far),
                    -$one,
                    $zero,
                    $zero,
                    (far * near) / (near - far),
                    $zero,
                ])
            }
        }
       )+
    };
}

macro_rules! neg {
    ($($t: ident),+) => {
       $(
        impl Neg for Mat4<$t> {
            type Output = Mat4<$t>;

            #[inline(always)]
            fn neg(mut self) -> Self::Output {
                self.0[0] = -self.0[0];
                self.0[1] = -self.0[1];
                self.0[2] = -self.0[2];
                self.0[3] = -self.0[3];
                self.0[4] = -self.0[4];
                self.0[5] = -self.0[5];
                self.0[6] = -self.0[6];
                self.0[7] = -self.0[7];
                self.0[8] = -self.0[8];
                self.0[9] = -self.0[9];
                self.0[10] = -self.0[10];
                self.0[11] = -self.0[11];
                self.0[12] = -self.0[12];
                self.0[13] = -self.0[13];
                self.0[14] = -self.0[14];
                self.0[15] = -self.0[15];
                self
            }
        }
       )+
    };
}

macro_rules! math {
    ($(($t: ident, $epsilon: expr, $zero: expr, $hhalf: expr, $one: expr, $two: expr)),+) => {
       $(
        impl Add for Mat4<$t> {
            type Output = Mat4<$t>;

            #[inline(always)]
            fn add(mut self, rhs: Self) -> Self::Output {
                self.0[0] = self.0[0] + rhs.0[0];
                self.0[1] = self.0[1] + rhs.0[1];
                self.0[2] = self.0[2] + rhs.0[2];
                self.0[3] = self.0[3] + rhs.0[3];
                self.0[4] = self.0[4] + rhs.0[4];
                self.0[5] = self.0[5] + rhs.0[5];
                self.0[6] = self.0[6] + rhs.0[6];
                self.0[7] = self.0[7] + rhs.0[7];
                self.0[8] = self.0[8] + rhs.0[8];
                self.0[9] = self.0[9] + rhs.0[9];
                self.0[10] = self.0[10] + rhs.0[10];
                self.0[11] = self.0[11] + rhs.0[11];
                self.0[12] = self.0[12] + rhs.0[12];
                self.0[13] = self.0[13] + rhs.0[13];
                self.0[14] = self.0[14] + rhs.0[14];
                self.0[15] = self.0[15] + rhs.0[15];
                self
            }
        }

        impl Add<$t> for Mat4<$t> {
            type Output = Mat4<$t>;

            #[inline(always)]
            fn add(mut self, rhs: $t) -> Self::Output {
                self.0[0] = self.0[0] + rhs;
                self.0[1] = self.0[1] + rhs;
                self.0[2] = self.0[2] + rhs;
                self.0[3] = self.0[3] + rhs;
                self.0[4] = self.0[4] + rhs;
                self.0[5] = self.0[5] + rhs;
                self.0[6] = self.0[6] + rhs;
                self.0[7] = self.0[7] + rhs;
                self.0[8] = self.0[8] + rhs;
                self.0[9] = self.0[9] + rhs;
                self.0[10] = self.0[10] + rhs;
                self.0[11] = self.0[11] + rhs;
                self.0[12] = self.0[12] + rhs;
                self.0[13] = self.0[13] + rhs;
                self.0[14] = self.0[14] + rhs;
                self.0[15] = self.0[15] + rhs;
                self
            }
        }

        impl Add<Mat4<$t>> for $t {
            type Output = Mat4<$t>;

            #[inline(always)]
            fn add(self, mut rhs: Mat4<$t>) -> Self::Output {
                rhs.0[0] = self + rhs.0[0];
                rhs.0[1] = self + rhs.0[1];
                rhs.0[2] = self + rhs.0[2];
                rhs.0[3] = self + rhs.0[3];
                rhs.0[4] = self + rhs.0[4];
                rhs.0[5] = self + rhs.0[5];
                rhs.0[6] = self + rhs.0[6];
                rhs.0[7] = self + rhs.0[7];
                rhs.0[8] = self + rhs.0[8];
                rhs.0[9] = self + rhs.0[9];
                rhs.0[10] = self + rhs.0[10];
                rhs.0[11] = self + rhs.0[11];
                rhs.0[12] = self + rhs.0[12];
                rhs.0[13] = self + rhs.0[13];
                rhs.0[14] = self + rhs.0[14];
                rhs.0[15] = self + rhs.0[15];
                rhs
            }
        }

        impl AddAssign for Mat4<$t> {
            #[inline(always)]
            fn add_assign(&mut self, rhs: Self) {
                self.0[0] += rhs.0[0];
                self.0[1] += rhs.0[1];
                self.0[2] += rhs.0[2];
                self.0[3] += rhs.0[3];
                self.0[4] += rhs.0[4];
                self.0[5] += rhs.0[5];
                self.0[6] += rhs.0[6];
                self.0[7] += rhs.0[7];
                self.0[8] += rhs.0[8];
                self.0[9] += rhs.0[9];
                self.0[10] += rhs.0[10];
                self.0[11] += rhs.0[11];
                self.0[12] += rhs.0[12];
                self.0[13] += rhs.0[13];
                self.0[14] += rhs.0[14];
                self.0[15] += rhs.0[15];
            }
        }

        impl AddAssign<$t> for Mat4<$t> {
            #[inline(always)]
            fn add_assign(&mut self, rhs: $t) {
                self.0[0] += rhs;
                self.0[1] += rhs;
                self.0[2] += rhs;
                self.0[3] += rhs;
                self.0[4] += rhs;
                self.0[5] += rhs;
                self.0[6] += rhs;
                self.0[7] += rhs;
                self.0[8] += rhs;
                self.0[9] += rhs;
                self.0[10] += rhs;
                self.0[11] += rhs;
                self.0[12] += rhs;
                self.0[13] += rhs;
                self.0[14] += rhs;
                self.0[15] += rhs;
            }
        }

        impl Sub for Mat4<$t> {
            type Output = Mat4<$t>;

            #[inline(always)]
            fn sub(mut self, rhs: Self) -> Self::Output {
                self.0[0] = self.0[0] - rhs.0[0];
                self.0[1] = self.0[1] - rhs.0[1];
                self.0[2] = self.0[2] - rhs.0[2];
                self.0[3] = self.0[3] - rhs.0[3];
                self.0[4] = self.0[4] - rhs.0[4];
                self.0[5] = self.0[5] - rhs.0[5];
                self.0[6] = self.0[6] - rhs.0[6];
                self.0[7] = self.0[7] - rhs.0[7];
                self.0[8] = self.0[8] - rhs.0[8];
                self.0[9] = self.0[9] - rhs.0[9];
                self.0[10] = self.0[10] - rhs.0[10];
                self.0[11] = self.0[11] - rhs.0[11];
                self.0[12] = self.0[12] - rhs.0[12];
                self.0[13] = self.0[13] - rhs.0[13];
                self.0[14] = self.0[14] - rhs.0[14];
                self.0[15] = self.0[15] - rhs.0[15];
                self
            }
        }

        impl Sub<$t> for Mat4<$t> {
            type Output = Mat4<$t>;

            #[inline(always)]
            fn sub(mut self, rhs: $t) -> Self::Output {
                self.0[0] = self.0[0] - rhs;
                self.0[1] = self.0[1] - rhs;
                self.0[2] = self.0[2] - rhs;
                self.0[3] = self.0[3] - rhs;
                self.0[4] = self.0[4] - rhs;
                self.0[5] = self.0[5] - rhs;
                self.0[6] = self.0[6] - rhs;
                self.0[7] = self.0[7] - rhs;
                self.0[8] = self.0[8] - rhs;
                self.0[9] = self.0[9] - rhs;
                self.0[10] = self.0[10] - rhs;
                self.0[11] = self.0[11] - rhs;
                self.0[12] = self.0[12] - rhs;
                self.0[13] = self.0[13] - rhs;
                self.0[14] = self.0[14] - rhs;
                self.0[15] = self.0[15] - rhs;
                self
            }
        }

        impl Sub<Mat4<$t>> for $t {
            type Output = Mat4<$t>;

            #[inline(always)]
            fn sub(self, mut rhs: Mat4<$t>) -> Self::Output {
                rhs.0[0] = self - rhs.0[0];
                rhs.0[1] = self - rhs.0[1];
                rhs.0[2] = self - rhs.0[2];
                rhs.0[3] = self - rhs.0[3];
                rhs.0[4] = self - rhs.0[4];
                rhs.0[5] = self - rhs.0[5];
                rhs.0[6] = self - rhs.0[6];
                rhs.0[7] = self - rhs.0[7];
                rhs.0[8] = self - rhs.0[8];
                rhs.0[9] = self - rhs.0[9];
                rhs.0[10] = self - rhs.0[10];
                rhs.0[11] = self - rhs.0[11];
                rhs.0[12] = self - rhs.0[12];
                rhs.0[13] = self - rhs.0[13];
                rhs.0[14] = self - rhs.0[14];
                rhs.0[15] = self - rhs.0[15];
                rhs
            }
        }

        impl SubAssign for Mat4<$t> {
            #[inline(always)]
            fn sub_assign(&mut self, rhs: Self) {
                self.0[0] -= rhs.0[0];
                self.0[1] -= rhs.0[1];
                self.0[2] -= rhs.0[2];
                self.0[3] -= rhs.0[3];
                self.0[4] -= rhs.0[4];
                self.0[5] -= rhs.0[5];
                self.0[6] -= rhs.0[6];
                self.0[7] -= rhs.0[7];
                self.0[8] -= rhs.0[8];
                self.0[9] -= rhs.0[9];
                self.0[10] -= rhs.0[10];
                self.0[11] -= rhs.0[11];
                self.0[12] -= rhs.0[12];
                self.0[13] -= rhs.0[13];
                self.0[14] -= rhs.0[14];
                self.0[15] -= rhs.0[15];
            }
        }

        impl SubAssign<$t> for Mat4<$t> {
            #[inline(always)]
            fn sub_assign(&mut self, rhs: $t) {
                self.0[0] -= rhs;
                self.0[1] -= rhs;
                self.0[2] -= rhs;
                self.0[3] -= rhs;
                self.0[4] -= rhs;
                self.0[5] -= rhs;
                self.0[6] -= rhs;
                self.0[7] -= rhs;
                self.0[8] -= rhs;
                self.0[9] -= rhs;
                self.0[10] -= rhs;
                self.0[11] -= rhs;
                self.0[12] -= rhs;
                self.0[13] -= rhs;
                self.0[14] -= rhs;
                self.0[15] -= rhs;
            }
        }

        impl Mul for Mat4<$t> {
            type Output = Mat4<$t>;

            #[inline(always)]
            fn mul(mut self, rhs: Self) -> Self::Output {
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

                let b00 = rhs.0[0];
                let b01 = rhs.0[1];
                let b02 = rhs.0[2];
                let b03 = rhs.0[3];
                let b10 = rhs.0[4];
                let b11 = rhs.0[5];
                let b12 = rhs.0[6];
                let b13 = rhs.0[7];
                let b20 = rhs.0[8];
                let b21 = rhs.0[9];
                let b22 = rhs.0[10];
                let b23 = rhs.0[11];
                let b30 = rhs.0[12];
                let b31 = rhs.0[13];
                let b32 = rhs.0[14];
                let b33 = rhs.0[15];

                // Cache only the current line of the second matrix
                let mut b0 = b00;
                let mut b1 = b01;
                let mut b2 = b02;
                let mut b3 = b03;
                self.0[0] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
                self.0[1] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
                self.0[2] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
                self.0[3] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;

                b0 = b10;
                b1 = b11;
                b2 = b12;
                b3 = b13;
                self.0[4] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
                self.0[5] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
                self.0[6] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
                self.0[7] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;

                b0 = b20;
                b1 = b21;
                b2 = b22;
                b3 = b23;
                self.0[8] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
                self.0[9] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
                self.0[10] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
                self.0[11] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;

                b0 = b30;
                b1 = b31;
                b2 = b32;
                b3 = b33;
                self.0[12] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
                self.0[13] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
                self.0[14] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
                self.0[15] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;
                self
            }
        }

        impl Mul<$t> for Mat4<$t> {
            type Output = Mat4<$t>;

            #[inline(always)]
            fn mul(mut self, rhs: $t) -> Self::Output {
                self.0[0] = self.0[0] * rhs;
                self.0[1] = self.0[1] * rhs;
                self.0[2] = self.0[2] * rhs;
                self.0[3] = self.0[3] * rhs;
                self.0[4] = self.0[4] * rhs;
                self.0[5] = self.0[5] * rhs;
                self.0[6] = self.0[6] * rhs;
                self.0[7] = self.0[7] * rhs;
                self.0[8] = self.0[8] * rhs;
                self.0[9] = self.0[9] * rhs;
                self.0[10] = self.0[10] * rhs;
                self.0[11] = self.0[11] * rhs;
                self.0[12] = self.0[12] * rhs;
                self.0[13] = self.0[13] * rhs;
                self.0[14] = self.0[14] * rhs;
                self.0[15] = self.0[15] * rhs;
                self
            }
        }

        impl Mul<Mat4<$t>> for $t {
            type Output = Mat4<$t>;

            #[inline(always)]
            fn mul(self, mut rhs: Mat4<$t>) -> Self::Output {
                rhs.0[0] = self * rhs.0[0];
                rhs.0[1] = self * rhs.0[1];
                rhs.0[2] = self * rhs.0[2];
                rhs.0[3] = self * rhs.0[3];
                rhs.0[4] = self * rhs.0[4];
                rhs.0[5] = self * rhs.0[5];
                rhs.0[6] = self * rhs.0[6];
                rhs.0[7] = self * rhs.0[7];
                rhs.0[8] = self * rhs.0[8];
                rhs.0[9] = self * rhs.0[9];
                rhs.0[10] = self * rhs.0[10];
                rhs.0[11] = self * rhs.0[11];
                rhs.0[12] = self * rhs.0[12];
                rhs.0[13] = self * rhs.0[13];
                rhs.0[14] = self * rhs.0[14];
                rhs.0[15] = self * rhs.0[15];
                rhs
            }
        }

        impl MulAssign for Mat4<$t> {
            #[inline(always)]
            fn mul_assign(&mut self, rhs: Self) {
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

                let b00 = rhs.0[0];
                let b01 = rhs.0[1];
                let b02 = rhs.0[2];
                let b03 = rhs.0[3];
                let b10 = rhs.0[4];
                let b11 = rhs.0[5];
                let b12 = rhs.0[6];
                let b13 = rhs.0[7];
                let b20 = rhs.0[8];
                let b21 = rhs.0[9];
                let b22 = rhs.0[10];
                let b23 = rhs.0[11];
                let b30 = rhs.0[12];
                let b31 = rhs.0[13];
                let b32 = rhs.0[14];
                let b33 = rhs.0[15];

                // Cache only the current line of the second matrix
                let mut b0 = b00;
                let mut b1 = b01;
                let mut b2 = b02;
                let mut b3 = b03;
                self.0[0] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
                self.0[1] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
                self.0[2] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
                self.0[3] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;

                b0 = b10;
                b1 = b11;
                b2 = b12;
                b3 = b13;
                self.0[4] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
                self.0[5] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
                self.0[6] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
                self.0[7] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;

                b0 = b20;
                b1 = b21;
                b2 = b22;
                b3 = b23;
                self.0[8] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
                self.0[9] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
                self.0[10] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
                self.0[11] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;

                b0 = b30;
                b1 = b31;
                b2 = b32;
                b3 = b33;
                self.0[12] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
                self.0[13] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
                self.0[14] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
                self.0[15] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;
            }
        }

        impl MulAssign<$t> for Mat4<$t> {
            #[inline(always)]
            fn mul_assign(&mut self, rhs: $t) {
                self.0[0] *= rhs;
                self.0[1] *= rhs;
                self.0[2] *= rhs;
                self.0[3] *= rhs;
                self.0[4] *= rhs;
                self.0[5] *= rhs;
                self.0[6] *= rhs;
                self.0[7] *= rhs;
                self.0[8] *= rhs;
                self.0[9] *= rhs;
                self.0[10] *= rhs;
                self.0[11] *= rhs;
                self.0[12] *= rhs;
                self.0[13] *= rhs;
                self.0[14] *= rhs;
                self.0[15] *= rhs;
            }
        }

        impl Div<$t> for Mat4<$t> {
            type Output = Mat4<$t>;

            #[inline(always)]
            fn div(mut self, rhs: $t) -> Self::Output {
                self.0[0] = self.0[0] / rhs;
                self.0[1] = self.0[1] / rhs;
                self.0[2] = self.0[2] / rhs;
                self.0[3] = self.0[3] / rhs;
                self.0[4] = self.0[4] / rhs;
                self.0[5] = self.0[5] / rhs;
                self.0[6] = self.0[6] / rhs;
                self.0[7] = self.0[7] / rhs;
                self.0[8] = self.0[8] / rhs;
                self.0[9] = self.0[9] / rhs;
                self.0[10] = self.0[10] / rhs;
                self.0[11] = self.0[11] / rhs;
                self.0[12] = self.0[12] / rhs;
                self.0[13] = self.0[13] / rhs;
                self.0[14] = self.0[14] / rhs;
                self.0[15] = self.0[15] / rhs;
                self
            }
        }

        impl Div<Mat4<$t>> for $t {
            type Output = Mat4<$t>;

            #[inline(always)]
            fn div(self, mut rhs: Mat4<$t>) -> Self::Output {
                rhs.0[0] = self / rhs.0[0];
                rhs.0[1] = self / rhs.0[1];
                rhs.0[2] = self / rhs.0[2];
                rhs.0[3] = self / rhs.0[3];
                rhs.0[4] = self / rhs.0[4];
                rhs.0[5] = self / rhs.0[5];
                rhs.0[6] = self / rhs.0[6];
                rhs.0[7] = self / rhs.0[7];
                rhs.0[8] = self / rhs.0[8];
                rhs.0[9] = self / rhs.0[9];
                rhs.0[10] = self / rhs.0[10];
                rhs.0[11] = self / rhs.0[11];
                rhs.0[12] = self / rhs.0[12];
                rhs.0[13] = self / rhs.0[13];
                rhs.0[14] = self / rhs.0[14];
                rhs.0[15] = self / rhs.0[15];
                rhs
            }
        }

        impl DivAssign<$t> for Mat4<$t> {
            #[inline(always)]
            fn div_assign(&mut self, rhs: $t) {
                self.0[0] = self.0[0] / rhs;
                self.0[1] = self.0[1] / rhs;
                self.0[2] = self.0[2] / rhs;
                self.0[3] = self.0[3] / rhs;
                self.0[4] = self.0[4] / rhs;
                self.0[5] = self.0[5] / rhs;
                self.0[6] = self.0[6] / rhs;
                self.0[7] = self.0[7] / rhs;
                self.0[8] = self.0[8] / rhs;
                self.0[9] = self.0[9] / rhs;
                self.0[10] = self.0[10] / rhs;
                self.0[11] = self.0[11] / rhs;
                self.0[12] = self.0[12] / rhs;
                self.0[13] = self.0[13] / rhs;
                self.0[14] = self.0[14] / rhs;
                self.0[15] = self.0[15] / rhs;
            }
        }

        impl Mat4<$t> {
            #[inline(always)]
            pub fn set_zero(&mut self) -> &mut Self {
                self.0[0] = $zero;
                self.0[1] = $zero;
                self.0[2] = $zero;
                self.0[3] = $zero;
                self.0[4] = $zero;
                self.0[5] = $zero;
                self.0[6] = $zero;
                self.0[7] = $zero;
                self.0[8] = $zero;
                self.0[9] = $zero;
                self.0[10] = $zero;
                self.0[11] = $zero;
                self.0[12] = $zero;
                self.0[13] = $zero;
                self.0[14] = $zero;
                self.0[15] = $zero;
                self
            }

            #[inline(always)]
            pub fn set_identify(&mut self) -> &mut Self {
                self.0[0] = $one;
                self.0[1] = $zero;
                self.0[2] = $zero;
                self.0[3] = $zero;
                self.0[4] = $zero;
                self.0[5] = $one;
                self.0[6] = $zero;
                self.0[7] = $zero;
                self.0[8] = $zero;
                self.0[9] = $zero;
                self.0[10] = $one;
                self.0[11] = $zero;
                self.0[12] = $zero;
                self.0[13] = $zero;
                self.0[14] = $zero;
                self.0[15] = $one;
                self
            }

            #[inline(always)]
            pub fn copy(&mut self, b: &Self) -> &mut Self {
                self.0[0] = b.0[0];
                self.0[1] = b.0[1];
                self.0[2] = b.0[2];
                self.0[3] = b.0[3];
                self.0[4] = b.0[4];
                self.0[5] = b.0[5];
                self.0[6] = b.0[6];
                self.0[7] = b.0[7];
                self.0[8] = b.0[8];
                self.0[9] = b.0[9];
                self.0[10] = b.0[10];
                self.0[11] = b.0[11];
                self.0[12] = b.0[12];
                self.0[13] = b.0[13];
                self.0[14] = b.0[14];
                self.0[15] = b.0[15];
                self
            }

            #[inline(always)]
            pub fn transpose(&self) -> Self {
                let m00 = self.0[0];
                let m01 = self.0[1];
                let m02 = self.0[2];
                let m03 = self.0[3];
                let m10 = self.0[4];
                let m11 = self.0[5];
                let m12 = self.0[6];
                let m13 = self.0[7];
                let m20 = self.0[8];
                let m21 = self.0[9];
                let m22 = self.0[10];
                let m23 = self.0[11];
                let m30 = self.0[12];
                let m31 = self.0[13];
                let m32 = self.0[14];
                let m33 = self.0[15];

                Self::new(m00, m10, m20, m30, m01, m11, m21, m31, m02, m12, m22, m32, m03, m13, m23, m33)
            }

            #[inline(always)]
            pub fn transpose_in_place(&mut self) -> &mut Self {
                let m00 = self.0[0];
                let m01 = self.0[1];
                let m02 = self.0[2];
                let m03 = self.0[3];
                let m10 = self.0[4];
                let m11 = self.0[5];
                let m12 = self.0[6];
                let m13 = self.0[7];
                let m20 = self.0[8];
                let m21 = self.0[9];
                let m22 = self.0[10];
                let m23 = self.0[11];
                let m30 = self.0[12];
                let m31 = self.0[13];
                let m32 = self.0[14];
                let m33 = self.0[15];

                self.0[0] = m00;
                self.0[1] = m10;
                self.0[2] = m20;
                self.0[3] = m30;
                self.0[4] = m01;
                self.0[5] = m11;
                self.0[6] = m21;
                self.0[7] = m31;
                self.0[8] = m02;
                self.0[9] = m12;
                self.0[10] = m22;
                self.0[11] = m32;
                self.0[12] = m03;
                self.0[13] = m13;
                self.0[14] = m23;
                self.0[15] = m33;
                self
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

                Self::new(
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
                )
            }

            #[inline(always)]
            pub fn adjoint_in_place(&mut self) -> &mut Self {
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


                self.0[0] = a11 * b11 - a12 * b10 + a13 * b09;
                self.0[1] = a02 * b10 - a01 * b11 - a03 * b09;
                self.0[2] = a31 * b05 - a32 * b04 + a33 * b03;
                self.0[3] = a22 * b04 - a21 * b05 - a23 * b03;
                self.0[4] = a12 * b08 - a10 * b11 - a13 * b07;
                self.0[5] = a00 * b11 - a02 * b08 + a03 * b07;
                self.0[6] = a32 * b02 - a30 * b05 - a33 * b01;
                self.0[7] = a20 * b05 - a22 * b02 + a23 * b01;
                self.0[8] = a10 * b10 - a11 * b08 + a13 * b06;
                self.0[9] = a01 * b08 - a00 * b10 - a03 * b06;
                self.0[10] = a30 * b04 - a31 * b02 + a33 * b00;
                self.0[11] = a21 * b02 - a20 * b04 - a23 * b00;
                self.0[12] = a11 * b07 - a10 * b09 - a12 * b06;
                self.0[13] = a00 * b09 - a01 * b07 + a02 * b06;
                self.0[14] = a31 * b01 - a30 * b03 - a32 * b00;
                self.0[15] = a20 * b03 - a21 * b01 + a22 * b00;
                self
            }

            #[inline(always)]
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

                if det == $zero {
                    return Err(Error::ZeroDeterminant);
                }
                det = $one / det;

                Ok(Self::new(
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
                ))
            }

            #[inline(always)]
            pub fn invert_in_place(&mut self) -> Result<&mut Self, Error> {
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

                if det == $zero {
                    return Err(Error::ZeroDeterminant);
                }
                det = $one / det;

                self.0[0] = (a11 * b11 - a12 * b10 + a13 * b09) * det;
                self.0[1] = (a02 * b10 - a01 * b11 - a03 * b09) * det;
                self.0[2] = (a31 * b05 - a32 * b04 + a33 * b03) * det;
                self.0[3] = (a22 * b04 - a21 * b05 - a23 * b03) * det;
                self.0[4] = (a12 * b08 - a10 * b11 - a13 * b07) * det;
                self.0[5] = (a00 * b11 - a02 * b08 + a03 * b07) * det;
                self.0[6] = (a32 * b02 - a30 * b05 - a33 * b01) * det;
                self.0[7] = (a20 * b05 - a22 * b02 + a23 * b01) * det;
                self.0[8] = (a10 * b10 - a11 * b08 + a13 * b06) * det;
                self.0[9] = (a01 * b08 - a00 * b10 - a03 * b06) * det;
                self.0[10] = (a30 * b04 - a31 * b02 + a33 * b00) * det;
                self.0[11] = (a21 * b02 - a20 * b04 - a23 * b00) * det;
                self.0[12] = (a11 * b07 - a10 * b09 - a12 * b06) * det;
                self.0[13] = (a00 * b09 - a01 * b07 + a02 * b06) * det;
                self.0[14] = (a31 * b01 - a30 * b03 - a32 * b00) * det;
                self.0[15] = (a20 * b03 - a21 * b01 + a22 * b00) * det;
                Ok(self)
            }

            #[inline(always)]
            pub fn scale(&self, v: &Vec3::<$t>) -> Self {
                let x = *v.x();
                let y = *v.y();
                let z = *v.z();

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

                Self::new(
                    a00 * x,
                    a01 * x,
                    a02 * x,
                    a03 * x,
                    a10 * y,
                    a11 * y,
                    a12 * y,
                    a13 * y,
                    a20 * z,
                    a21 * z,
                    a22 * z,
                    a23 * z,
                    a30,
                    a31,
                    a32,
                    a33,
                )
            }

            #[inline(always)]
            pub fn scale_in_place(&mut self, v: &Vec3::<$t>) -> &mut Self {
                let x = *v.x();
                let y = *v.y();
                let z = *v.z();

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

                self.0[0] = a00 * x;
                self.0[1] = a01 * x;
                self.0[2] = a02 * x;
                self.0[3] = a03 * x;
                self.0[4] = a10 * y;
                self.0[5] = a11 * y;
                self.0[6] = a12 * y;
                self.0[7] = a13 * y;
                self.0[8] = a20 * z;
                self.0[9] = a21 * z;
                self.0[10] = a22 * z;
                self.0[11] = a23 * z;
                self.0[12] = a30;
                self.0[13] = a31;
                self.0[14] = a32;
                self.0[15] = a33;
                self
            }

            #[inline(always)]
            pub fn rotate(&self, axis: &Vec3::<$t>, rad: $t) -> Result<Self, Error> {
                let mut x = *axis.x();
                let mut y = *axis.y();
                let mut z = *axis.z();
                let mut len = (x * x + y * y + z * z).sqrt();
                let s;
                let c;
                let t;

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

                len = $one / len;
                x = x * len;
                y = y * len;
                z = z * len;

                s = rad.sin();
                c = rad.cos();
                t = $one - c;

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

                Ok(Self::new(
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
                    a30,
                    a31,
                    a32,
                    a33,
                ))
            }

            #[inline(always)]
            pub fn rotate_in_place(&mut self, axis: &Vec3::<$t>, rad: $t) -> Result<&mut Self, Error> {
                let mut x = *axis.x();
                let mut y = *axis.y();
                let mut z = *axis.z();
                let mut len = (x * x + y * y + z * z).sqrt();
                let s;
                let c;
                let t;

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

                len = $one / len;
                x = x * len;
                y = y * len;
                z = z * len;

                s = rad.sin();
                c = rad.cos();
                t = $one - c;

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

                self.0[0] = a00 * b00 + a10 * b01 + a20 * b02;
                self.0[1] = a01 * b00 + a11 * b01 + a21 * b02;
                self.0[2] = a02 * b00 + a12 * b01 + a22 * b02;
                self.0[3] = a03 * b00 + a13 * b01 + a23 * b02;
                self.0[4] = a00 * b10 + a10 * b11 + a20 * b12;
                self.0[5] = a01 * b10 + a11 * b11 + a21 * b12;
                self.0[6] = a02 * b10 + a12 * b11 + a22 * b12;
                self.0[7] = a03 * b10 + a13 * b11 + a23 * b12;
                self.0[8] = a00 * b20 + a10 * b21 + a20 * b22;
                self.0[9] = a01 * b20 + a11 * b21 + a21 * b22;
                self.0[10] = a02 * b20 + a12 * b21 + a22 * b22;
                self.0[11] = a03 * b20 + a13 * b21 + a23 * b22;
                self.0[12] = a30;
                self.0[13] = a31;
                self.0[14] = a32;
                self.0[15] = a33;
                Ok(self)
            }

            #[inline(always)]
            pub fn rotate_x(&self, rad: $t) -> Self {
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
                let a20 = self.0[8];
                let a21 = self.0[9];
                let a22 = self.0[10];
                let a23 = self.0[11];
                let a30 = self.0[12];
                let a31 = self.0[13];
                let a32 = self.0[14];
                let a33 = self.0[15];

                // Perform axis-specific matrix multiplication
                Self::new(
                    a00,
                    a01,
                    a02,
                    a03,
                    a10 * c + a20 * s,
                    a11 * c + a21 * s,
                    a12 * c + a22 * s,
                    a13 * c + a23 * s,
                    a20 * c - a10 * s,
                    a21 * c - a11 * s,
                    a22 * c - a12 * s,
                    a23 * c - a13 * s,
                    a30,
                    a31,
                    a32,
                    a33,
                )
            }

            #[inline(always)]
            pub fn rotate_x_in_place(&mut self, rad: $t) -> &mut Self {
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
                let a20 = self.0[8];
                let a21 = self.0[9];
                let a22 = self.0[10];
                let a23 = self.0[11];
                let a30 = self.0[12];
                let a31 = self.0[13];
                let a32 = self.0[14];
                let a33 = self.0[15];

                // Perform axis-specific matrix multiplication
                self.0[0] = a00;
                self.0[1] = a01;
                self.0[2] = a02;
                self.0[3] = a03;
                self.0[4] = a10 * c + a20 * s;
                self.0[5] = a11 * c + a21 * s;
                self.0[6] = a12 * c + a22 * s;
                self.0[7] = a13 * c + a23 * s;
                self.0[8] = a20 * c - a10 * s;
                self.0[9] = a21 * c - a11 * s;
                self.0[10] = a22 * c - a12 * s;
                self.0[11] = a23 * c - a13 * s;
                self.0[12] = a30;
                self.0[13] = a31;
                self.0[14] = a32;
                self.0[15] = a33;
                self
            }

            #[inline(always)]
            pub fn rotate_y(&self, rad: $t) -> Self {
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
                let a20 = self.0[8];
                let a21 = self.0[9];
                let a22 = self.0[10];
                let a23 = self.0[11];
                let a30 = self.0[12];
                let a31 = self.0[13];
                let a32 = self.0[14];
                let a33 = self.0[15];

                // Perform axis-specific matrix multiplication
                Self::new(
                    a00 * c - a20 * s,
                    a01 * c - a21 * s,
                    a02 * c - a22 * s,
                    a03 * c - a23 * s,
                    a10,
                    a11,
                    a12,
                    a13,
                    a00 * s + a20 * c,
                    a01 * s + a21 * c,
                    a02 * s + a22 * c,
                    a03 * s + a23 * c,
                    a30,
                    a31,
                    a32,
                    a33,
                )
            }

            #[inline(always)]
            pub fn rotate_y_in_place(&mut self, rad: $t) -> &mut Self {
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
                let a20 = self.0[8];
                let a21 = self.0[9];
                let a22 = self.0[10];
                let a23 = self.0[11];
                let a30 = self.0[12];
                let a31 = self.0[13];
                let a32 = self.0[14];
                let a33 = self.0[15];

                // Perform axis-specific matrix multiplication
                self.0[0] = a00 * c - a20 * s;
                self.0[1] = a01 * c - a21 * s;
                self.0[2] = a02 * c - a22 * s;
                self.0[3] = a03 * c - a23 * s;
                self.0[4] = a10;
                self.0[5] = a11;
                self.0[6] = a12;
                self.0[7] = a13;
                self.0[8] = a00 * s + a20 * c;
                self.0[9] = a01 * s + a21 * c;
                self.0[10] = a02 * s + a22 * c;
                self.0[11] = a03 * s + a23 * c;
                self.0[12] = a30;
                self.0[13] = a31;
                self.0[14] = a32;
                self.0[15] = a33;
                self
            }

            #[inline(always)]
            pub fn rotate_z(&self, rad: $t) -> Self {
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
                let a20 = self.0[8];
                let a21 = self.0[9];
                let a22 = self.0[10];
                let a23 = self.0[11];
                let a30 = self.0[12];
                let a31 = self.0[13];
                let a32 = self.0[14];
                let a33 = self.0[15];

                // Perform axis-specific matrix multiplication
                Self::new(
                    a00 * c + a10 * s,
                    a01 * c + a11 * s,
                    a02 * c + a12 * s,
                    a03 * c + a13 * s,
                    a10 * c - a00 * s,
                    a11 * c - a01 * s,
                    a12 * c - a02 * s,
                    a13 * c - a03 * s,
                    a20,
                    a21,
                    a22,
                    a23,
                    a30,
                    a31,
                    a32,
                    a33,
                )
            }

            #[inline(always)]
            pub fn rotate_z_in_place(&mut self, rad: $t) -> &mut Self {
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
                let a20 = self.0[8];
                let a21 = self.0[9];
                let a22 = self.0[10];
                let a23 = self.0[11];
                let a30 = self.0[12];
                let a31 = self.0[13];
                let a32 = self.0[14];
                let a33 = self.0[15];

                // Perform axis-specific matrix multiplication
                self.0[0] = a00 * c + a10 * s;
                self.0[1] = a01 * c + a11 * s;
                self.0[2] = a02 * c + a12 * s;
                self.0[3] = a03 * c + a13 * s;
                self.0[4] = a10 * c - a00 * s;
                self.0[5] = a11 * c - a01 * s;
                self.0[6] = a12 * c - a02 * s;
                self.0[7] = a13 * c - a03 * s;
                self.0[8] = a20;
                self.0[9] = a21;
                self.0[10] = a22;
                self.0[11] = a23;
                self.0[12] = a30;
                self.0[13] = a31;
                self.0[14] = a32;
                self.0[15] = a33;
                self
            }

            #[inline(always)]
            pub fn translate(&self, v: &Vec3::<$t>) -> Self {
                let x = v.x();
                let y = v.y();
                let z = v.z();
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

                Self::new(
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
                    a00 * x + a10 * y + a20 * z + a30,
                    a01 * x + a11 * y + a21 * z + a31,
                    a02 * x + a12 * y + a22 * z + a32,
                    a03 * x + a13 * y + a23 * z + a33,
                )
            }

            #[inline(always)]
            pub fn translate_in_place(&mut self, v: &Vec3::<$t>) -> &mut Self {
                let x = v.x();
                let y = v.y();
                let z = v.z();
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

                self.0[0] = a00;
                self.0[1] = a01;
                self.0[2] = a02;
                self.0[3] = a03;
                self.0[4] = a10;
                self.0[5] = a11;
                self.0[6] = a12;
                self.0[7] = a13;
                self.0[8] = a20;
                self.0[9] = a21;
                self.0[10] = a22;
                self.0[11] = a23;
                self.0[12] = a00 * x + a10 * y + a20 * z + a30;
                self.0[13] = a01 * x + a11 * y + a21 * z + a31;
                self.0[14] = a02 * x + a12 * y + a22 * z + a32;
                self.0[15] = a03 * x + a13 * y + a23 * z + a33;
                self
            }

            #[inline(always)]
            pub fn decompose(&self) -> (Quat<$t>, Vec3<$t>, Vec3<$t>) {
                let mut out_r = Quat::<$t>::new_zero();
                let mut out_t = Vec3::<$t>::new_zero();
                let mut out_s = Vec3::<$t>::new_zero();

                let a00 = *self.m00();
                let a01 = *self.m01();
                let a02 = *self.m02();
                let a10 = *self.m10();
                let a11 = *self.m11();
                let a12 = *self.m12();
                let a20 = *self.m20();
                let a21 = *self.m21();
                let a22 = *self.m22();
                let a30 = *self.m30();
                let a31 = *self.m31();
                let a32 = *self.m32();

                out_t.set_x(a30);
                out_t.set_y(a31);
                out_t.set_z(a32);

                let m11 = a00;
                let m12 = a01;
                let m13 = a02;
                let m21 = a10;
                let m22 = a11;
                let m23 = a12;
                let m31 = a20;
                let m32 = a21;
                let m33 = a22;

                out_s.set_x((m11 * m11 + m12 * m12 + m13 * m13).sqrt());
                out_s.set_y((m21 * m21 + m22 * m22 + m23 * m23).sqrt());
                out_s.set_z((m31 * m31 + m32 * m32 + m33 * m33).sqrt());

                let is1 = $one / out_s.x();
                let is2 = $one / out_s.y();
                let is3 = $one / out_s.z();

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

                if trace > $zero {
                    let s = (trace + $one).sqrt() * $two;
                    out_r.set_w($hhalf * s);
                    out_r.set_x((sm23 - sm32) / s);
                    out_r.set_y((sm31 - sm13) / s);
                    out_r.set_z((sm12 - sm21) / s);
                } else if sm11 > sm22 && sm11 > sm33 {
                    let s = ($one + sm11 - sm22 - sm33).sqrt() * $two;
                    out_r.set_w((sm23 - sm32) / s);
                    out_r.set_x($hhalf * s);
                    out_r.set_y((sm12 + sm21) / s);
                    out_r.set_z((sm31 + sm13) / s);
                } else if sm22 > sm33 {
                    let s = ($one + sm22 - sm11 - sm33).sqrt() * $two;
                    out_r.set_w((sm31 - sm13) / s);
                    out_r.set_x((sm12 + sm21) / s);
                    out_r.set_y($hhalf * s);
                    out_r.set_z((sm23 + sm32) / s);
                } else {
                    let s = ($one + sm33 - sm11 - sm22).sqrt() * $two;
                    out_r.set_w((sm12 - sm21) / s);
                    out_r.set_x((sm31 + sm13) / s);
                    out_r.set_y((sm23 + sm32) / s);
                    out_r.set_z($hhalf * s);
                }

                (out_r, out_t, out_s)
            }

            #[inline(always)]
            pub fn frob(&self) -> $t {
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

                (a00 * a00
                    + a01 * a01
                    + a02 * a02
                    + a03 * a03
                    + a10 * a10
                    + a11 * a11
                    + a12 * a12
                    + a13 * a13
                    + a20 * a20
                    + a21 * a21
                    + a22 * a22
                    + a23 * a23
                    + a30 * a30
                    + a31 * a31
                    + a32 * a32
                    + a33 * a33)
                    .sqrt()
            }

            #[inline(always)]
            pub fn translation(&self) -> Vec3<$t> {
                Vec3::<$t>::new(self.0[12], self.0[13], self.0[14])
            }

            #[inline(always)]
            pub fn scaling(&self) -> Vec3<$t> {
                let m00 = self.0[0];
                let m01 = self.0[1];
                let m02 = self.0[2];
                let m10 = self.0[4];
                let m11 = self.0[5];
                let m12 = self.0[6];
                let m20 = self.0[8];
                let m21 = self.0[9];
                let m22 = self.0[10];

                Vec3::<$t>::new(
                    (m00 * m00 + m01 * m01 + m02 * m02).sqrt(),
                    (m10 * m10 + m11 * m11 + m12 * m12).sqrt(),
                    (m20 * m20 + m21 * m21 + m22 * m22).sqrt(),
                )
            }

            #[inline(always)]
            pub fn rotation(&self) -> Quat<$t> {
                let mut out = Quat::<$t>::new_zero();

                let scaling = self.scaling();

                let is1 = $one / scaling.x();
                let is2 = $one / scaling.y();
                let is3 = $one / scaling.z();

                let sm11 = self.0[5] * is1;
                let sm12 = self.0[6] * is2;
                let sm13 = self.0[7] * is3;
                let sm21 = self.0[9] * is1;
                let sm22 = self.0[10] * is2;
                let sm23 = self.0[11] * is3;
                let sm31 = self.0[13] * is1;
                let sm32 = self.0[14] * is2;
                let sm33 = self.0[15] * is3;

                let trace = sm11 + sm22 + sm33;

                if trace > $zero {
                    let s = (trace + $one).sqrt() * $two;
                    out.set_w($hhalf * s);
                    out.set_x((sm23 - sm32) / s);
                    out.set_y((sm31 - sm13) / s);
                    out.set_z((sm12 - sm21) / s);
                } else if sm11 > sm22 && sm11 > sm33 {
                    let s = ($one + sm11 - sm22 - sm33).sqrt() * $two;
                    out.set_w((sm23 - sm32) / s);
                    out.set_x($hhalf * s);
                    out.set_y((sm12 + sm21) / s);
                    out.set_z((sm31 + sm13) / s);
                } else if sm22 > sm33 {
                    let s = ($one + sm22 - sm11 - sm33).sqrt() * $two;
                    out.set_w((sm31 - sm13) / s);
                    out.set_x((sm12 + sm21) / s);
                    out.set_y($hhalf * s);
                    out.set_z((sm23 + sm32) / s);
                } else {
                    let s = ($one + sm33 - sm11 - sm22).sqrt() * $two;
                    out.set_w((sm12 - sm21) / s);
                    out.set_x((sm31 + sm13) / s);
                    out.set_y((sm23 + sm32) / s);
                    out.set_z($hhalf * s);
                }

                out
            }
        }

        impl ApproximateEq for Mat4<$t> {
            #[inline(always)]
            fn approximate_eq(&self, other: &Self) -> bool {
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

                let b0 = other.0[0];
                let b1 = other.0[1];
                let b2 = other.0[2];
                let b3 = other.0[3];
                let b4 = other.0[4];
                let b5 = other.0[5];
                let b6 = other.0[6];
                let b7 = other.0[7];
                let b8 = other.0[8];
                let b9 = other.0[9];
                let b10 = other.0[10];
                let b11 = other.0[11];
                let b12 = other.0[12];
                let b13 = other.0[13];
                let b14 = other.0[14];
                let b15 = other.0[15];

                (a0 - b0).abs() <= $epsilon * $one.max(a0.abs()).max(b0.abs())
                    && (a1 - b1).abs() <= $epsilon * $one.max(a1.abs()).max(b1.abs())
                    && (a2 - b2).abs() <= $epsilon * $one.max(a2.abs()).max(b2.abs())
                    && (a3 - b3).abs() <= $epsilon * $one.max(a3.abs()).max(b3.abs())
                    && (a4 - b4).abs() <= $epsilon * $one.max(a4.abs()).max(b4.abs())
                    && (a5 - b5).abs() <= $epsilon * $one.max(a5.abs()).max(b5.abs())
                    && (a6 - b6).abs() <= $epsilon * $one.max(a6.abs()).max(b6.abs())
                    && (a7 - b7).abs() <= $epsilon * $one.max(a7.abs()).max(b7.abs())
                    && (a8 - b8).abs() <= $epsilon * $one.max(a8.abs()).max(b8.abs())
                    && (a9 - b9).abs() <= $epsilon * $one.max(a9.abs()).max(b9.abs())
                    && (a10 - b10).abs() <= $epsilon * $one.max(a10.abs()).max(b10.abs())
                    && (a11 - b11).abs() <= $epsilon * $one.max(a11.abs()).max(b11.abs())
                    && (a12 - b12).abs() <= $epsilon * $one.max(a12.abs()).max(b12.abs())
                    && (a13 - b13).abs() <= $epsilon * $one.max(a13.abs()).max(b13.abs())
                    && (a14 - b14).abs() <= $epsilon * $one.max(a14.abs()).max(b14.abs())
                    && (a15 - b15).abs() <= $epsilon * $one.max(a15.abs()).max(b15.abs())
            }
        }
       )+
    };
}

basic_constructors! {
    (u8, 0u8, 1u8),
    (u16, 0u16, 1u16),
    (u32, 0u32, 1u32),
    (u64, 0u64, 1u64),
    (u128, 0u128, 1u128),
    (usize, 0usize, 1usize),
    (i8, 0i8, 1i8),
    (i16, 0i16, 1i16),
    (i32, 0i32, 1i32),
    (i64, 0i64, 1i64),
    (i128, 0i128, 1i128),
    (isize, 0isize, 1isize),
    (f32, 0.0f32, 1.0f32),
    (f64, 0.0f64, 1.0f64)
}
decimal_constructors! {
    (f32, super::EPSILON_F32, 0.0f32, 0.5f32, 1.0f32, 2.0f32, 180.0f32, std::f32::consts::PI),
    (f64, super::EPSILON_F64, 0.0f64, 0.5f64, 1.0f64, 2.0f64, 180.0f64, std::f64::consts::PI)
}
neg!(i8, i16, i32, i64, i128, isize, f32, f64);
math! {
    (f32, super::EPSILON_F32, 0.0f32, 0.25f32, 1.0f32, 2.0f32),
    (f64, super::EPSILON_F64, 0.0f64, 0.25f64, 1.0f64, 2.0f64)
}

#[cfg(test)]
mod tests {
    use crate::{error::Error, quat::Quat, vec3::Vec3, ApproximateEq};

    use super::Mat4;

    #[test]
    fn new() {
        assert_eq!(
            Mat4::new(
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            )
            .raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn new_zero() {
        assert_eq!(
            Mat4::<f64>::new_zero().raw(),
            &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        );
    }

    #[test]
    fn new_identity() {
        assert_eq!(
            Mat4::<f64>::new_identity().raw(),
            &[1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        );
    }

    #[test]
    fn from_slice() {
        assert_eq!(
            Mat4::from_slice([
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
    fn from_frustum() {
        assert_eq!(
            Mat4::<f64>::from_frustum(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0).raw(),
            &[-1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0]
        );
    }

    #[test]
    fn from_look_at() {
        assert_eq!(
            Mat4::<f64>::from_look_at(
                &Vec3::new(0.0, 0.0, 1.0),
                &Vec3::new(0.0, 0.0, -1.0),
                &Vec3::new(0.0, 1.0, 0.0),
            )
            .approximate_eq(&Mat4::new(
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0, 1.0
            )),
            true
        );
    }

    #[test]
    fn from_target_to() {
        assert_eq!(
            Mat4::<f64>::from_target_to(
                &Vec3::new(0.0, 0.0, 1.0),
                &Vec3::new(0.0, 0.0, -1.0),
                &Vec3::new(0.0, 1.0, 0.0),
            )
            .approximate_eq(&Mat4::new(
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0
            )),
            true
        );
    }

    #[test]
    fn from_perspective_no() {
        assert_eq!(
            Mat4::<f64>::from_perspective_no(std::f64::consts::PI * 0.5, 1.0, 0.0, Some(1.0))
                .approximate_eq(&Mat4::new(
                    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0,
                    0.0
                )),
            true
        );

        assert_eq!(
            Mat4::<f64>::from_perspective_no(
                std::f64::consts::PI * 45.0 / 180.0,
                640.0 / 480.0,
                0.1,
                Some(200.0)
            )
            .approximate_eq(&Mat4::new(
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
            )),
            true
        );

        assert_eq!(
            Mat4::<f64>::from_perspective_no(
                std::f64::consts::PI * 45.0 / 180.0,
                640.0 / 480.0,
                0.1,
                None
            )
            .approximate_eq(&Mat4::new(
                1.81066, 0.0, 0.0, 0.0, 0.0, 2.4142134, 0.0, 0.0, 0.0, 0.0, -1.0, -1.0, 0.0, 0.0,
                -0.2, 0.0
            )),
            true
        );

        assert_eq!(
            Mat4::<f64>::from_perspective_no(
                std::f64::consts::PI * 45.0 / 180.0,
                640.0 / 480.0,
                0.1,
                Some(f64::INFINITY)
            )
            .approximate_eq(&Mat4::new(
                1.81066, 0.0, 0.0, 0.0, 0.0, 2.4142134, 0.0, 0.0, 0.0, 0.0, -1.0, -1.0, 0.0, 0.0,
                -0.2, 0.0
            )),
            true
        );
    }

    #[test]
    fn raw() {
        assert_eq!(
            Mat4::new(
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            )
            .raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn raw_mut() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        (*mat.raw_mut()) = [
            11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0,
            25.0, 26.0,
        ];
        assert_eq!(
            mat.raw(),
            &[
                11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0,
                25.0, 26.0
            ]
        );
    }

    #[test]
    fn m00() {
        assert_eq!(
            Mat4::<f64>::new(
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            )
            .m00(),
            &1.0
        );
    }

    #[test]
    fn m01() {
        assert_eq!(
            Mat4::<f64>::new(
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            )
            .m01(),
            &2.0
        );
    }

    #[test]
    fn m02() {
        assert_eq!(
            Mat4::<f64>::new(
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            )
            .m02(),
            &3.0
        );
    }

    #[test]
    fn m03() {
        assert_eq!(
            Mat4::<f64>::new(
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            )
            .m03(),
            &4.0
        );
    }

    #[test]
    fn m10() {
        assert_eq!(
            Mat4::<f64>::new(
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            )
            .m10(),
            &5.0
        );
    }

    #[test]
    fn m11() {
        assert_eq!(
            Mat4::<f64>::new(
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            )
            .m11(),
            &6.0
        );
    }

    #[test]
    fn m12() {
        assert_eq!(
            Mat4::<f64>::new(
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            )
            .m12(),
            &7.0
        );
    }

    #[test]
    fn m13() {
        assert_eq!(
            Mat4::<f64>::new(
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            )
            .m13(),
            &8.0
        );
    }

    #[test]
    fn m20() {
        assert_eq!(
            Mat4::<f64>::new(
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            )
            .m20(),
            &9.0
        );
    }

    #[test]
    fn m21() {
        assert_eq!(
            Mat4::<f64>::new(
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            )
            .m21(),
            &10.0
        );
    }

    #[test]
    fn m22() {
        assert_eq!(
            Mat4::<f64>::new(
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            )
            .m22(),
            &11.0
        );
    }

    #[test]
    fn m23() {
        assert_eq!(
            Mat4::<f64>::new(
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            )
            .m23(),
            &12.0
        );
    }

    #[test]
    fn m30() {
        assert_eq!(
            Mat4::<f64>::new(
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            )
            .m30(),
            &13.0
        );
    }

    #[test]
    fn m31() {
        assert_eq!(
            Mat4::<f64>::new(
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            )
            .m31(),
            &14.0
        );
    }

    #[test]
    fn m32() {
        assert_eq!(
            Mat4::<f64>::new(
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            )
            .m32(),
            &15.0
        );
    }

    #[test]
    fn m33() {
        assert_eq!(
            Mat4::<f64>::new(
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            )
            .m33(),
            &16.0
        );
    }

    #[test]
    fn m00_mut() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        (*mat.m00_mut()) = 10.0;
        assert_eq!(
            mat.raw(),
            &[
                10.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn m01_mut() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        (*mat.m01_mut()) = 10.0;
        assert_eq!(
            mat.raw(),
            &[
                1.0, 10.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn m02_mut() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        (*mat.m02_mut()) = 10.0;
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 10.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn m03_mut() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        (*mat.m03_mut()) = 10.0;
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 10.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn m10_mut() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        (*mat.m10_mut()) = 10.0;
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 10.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn m11_mut() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        (*mat.m11_mut()) = 10.0;
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 10.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn m12_mut() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        (*mat.m12_mut()) = 10.0;
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 10.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn m13_mut() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        (*mat.m13_mut()) = 10.0;
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 10.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn m20_mut() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        (*mat.m20_mut()) = 10.0;
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 10.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn m21_mut() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        (*mat.m21_mut()) = 12.0;
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 12.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn m22_mut() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        (*mat.m22_mut()) = 10.0;
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 10.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn m23_mut() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        (*mat.m23_mut()) = 10.0;
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 10.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn m30_mut() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        (*mat.m30_mut()) = 10.0;
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 10.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn m31_mut() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        (*mat.m31_mut()) = 10.0;
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 10.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn m32_mut() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        (*mat.m32_mut()) = 10.0;
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 10.0,
                16.0
            ]
        );
    }

    #[test]
    fn m33_mut() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        (*mat.m33_mut()) = 10.0;
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                10.0
            ]
        );
    }

    #[test]
    fn set_m00() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        mat.set_m00(10.0);
        assert_eq!(
            mat.raw(),
            &[
                10.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn set_m01() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        mat.set_m01(10.0);
        assert_eq!(
            mat.raw(),
            &[
                1.0, 10.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn set_m02() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        mat.set_m02(10.0);
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 10.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn set_m03() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        mat.set_m03(10.0);
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 10.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn set_m10() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        mat.set_m10(10.0);
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 10.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn set_m11() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        mat.set_m11(10.0);
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 10.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn set_m12() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        mat.set_m12(10.0);
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 10.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn set_m13() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        mat.set_m13(10.0);
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 10.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn set_m20() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        mat.set_m20(10.0);
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 10.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn set_m21() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        mat.set_m21(12.0);
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 12.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn set_m22() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        mat.set_m22(10.0);
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 10.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn set_m23() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        mat.set_m23(10.0);
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 10.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn set_m30() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        mat.set_m30(10.0);
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 10.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn set_m31() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        mat.set_m31(10.0);
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 10.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn set_m32() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        mat.set_m32(10.0);
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 10.0,
                16.0
            ]
        );
    }

    #[test]
    fn set_m33() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        mat.set_m33(10.0);
        assert_eq!(
            mat.raw(),
            &[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                10.0
            ]
        );
    }

    #[test]
    fn set() {
        let mut mat = Mat4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        mat.set(
            10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
            10.0, 10.0,
        );
        assert_eq!(
            mat.raw(),
            &[
                10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
                10.0, 10.0
            ]
        );
    }

    #[test]
    fn set_zero() {
        let mut mat = Mat4::<f64>::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        mat.set_zero();
        assert_eq!(
            mat.raw(),
            &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        );
    }

    #[test]
    fn set_identify() {
        let mut mat = Mat4::<f64>::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        mat.set_identify();
        assert_eq!(
            mat.raw(),
            &[1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        );
    }

    #[test]
    fn add_mat4_mat4() {
        let mat0 = Mat4::<f64>::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        let mat1 = Mat4::<f64>::new(
            17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0,
            31.0, 32.0,
        );
        assert_eq!(
            (mat0 + mat1).approximate_eq(&Mat4::new(
                18.0, 20.0, 22.0, 24.0, 26.0, 28.0, 30.0, 32.0, 34.0, 36.0, 38.0, 40.0, 42.0, 44.0,
                46.0, 48.0
            )),
            true
        );
    }

    #[test]
    fn add_mat4_scalar() {
        let mat = Mat4::<f64>::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        let scalar = 2.0;
        assert_eq!(
            (mat + scalar).approximate_eq(&Mat4::new(
                3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0,
                18.0
            )),
            true
        );
    }

    #[test]
    fn add_scalar_mat4() {
        let scalar = 1.0;
        let mat = Mat4::<f64>::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        assert_eq!(
            (scalar + mat).approximate_eq(&Mat4::new(
                2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
                17.0
            )),
            true
        );
    }

    #[test]
    fn add_assign_mat4_mat4() {
        let mut mat0 = Mat4::<f64>::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        let mat1 = Mat4::<f64>::new(
            17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0,
            31.0, 32.0,
        );
        mat0 += mat1;
        assert_eq!(
            mat0.approximate_eq(&Mat4::new(
                18.0, 20.0, 22.0, 24.0, 26.0, 28.0, 30.0, 32.0, 34.0, 36.0, 38.0, 40.0, 42.0, 44.0,
                46.0, 48.0
            )),
            true
        );
    }

    #[test]
    fn add_assign_mat4_scalar() {
        let mut mat = Mat4::<f64>::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        let scalar = 2.0;
        mat += scalar;
        assert_eq!(
            mat.approximate_eq(&Mat4::new(
                3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0,
                18.0
            )),
            true
        );
    }

    #[test]
    fn sub_mat4_mat4() {
        let mat0 = Mat4::<f64>::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        let mat1 = Mat4::<f64>::new(
            17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0,
            31.0, 32.0,
        );
        assert_eq!(
            (mat0 - mat1).approximate_eq(&Mat4::new(
                -16.0, -16.0, -16.0, -16.0, -16.0, -16.0, -16.0, -16.0, -16.0, -16.0, -16.0, -16.0,
                -16.0, -16.0, -16.0, -16.0
            )),
            true
        );
    }

    #[test]
    fn sub_mat4_scalar() {
        let mat = Mat4::<f64>::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        let scalar = 1.0;
        assert_eq!(
            (mat - scalar).approximate_eq(&Mat4::new(
                0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0,
                15.0
            )),
            true
        );
    }

    #[test]
    fn sub_scalar_mat4() {
        let scalar = 1.0;
        let mat = Mat4::<f64>::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        assert_eq!(
            (scalar - mat).approximate_eq(&Mat4::new(
                0.0, -1.0, -2.0, -3.0, -4.0, -5.0, -6.0, -7.0, -8.0, -9.0, -10.0, -11.0, -12.0,
                -13.0, -14.0, -15.0
            )),
            true
        );
    }

    #[test]
    fn sub_assign_mat4_mat4() {
        let mut mat0 = Mat4::<f64>::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        let mat1 = Mat4::<f64>::new(
            17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0,
            31.0, 32.0,
        );
        mat0 -= mat1;
        assert_eq!(
            mat0.approximate_eq(&Mat4::new(
                -16.0, -16.0, -16.0, -16.0, -16.0, -16.0, -16.0, -16.0, -16.0, -16.0, -16.0, -16.0,
                -16.0, -16.0, -16.0, -16.0
            )),
            true
        );
    }

    #[test]
    fn sub_assign_mat4_scalar() {
        let mut mat = Mat4::<f64>::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        let scalar = 1.0;
        mat -= scalar;
        assert_eq!(
            mat.approximate_eq(&Mat4::new(
                0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0,
                15.0
            )),
            true
        );
    }

    #[test]
    fn mul_mat4_mat4() {
        let mat0 = Mat4::<f64>::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
        );
        let mat1 = Mat4::<f64>::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 4.0, 5.0, 6.0, 1.0,
        );
        assert_eq!(
            (mat0 * mat1).approximate_eq(&Mat4::new(
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 5.0, 7.0, 9.0, 1.0
            )),
            true
        );
    }

    #[test]
    fn mul_mat4_scalar() {
        let mat = Mat4::<f64>::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        let scalar = 2.0;
        assert_eq!(
            (mat * scalar).approximate_eq(&Mat4::new(
                2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0, 22.0, 24.0, 26.0, 28.0,
                30.0, 32.0
            )),
            true
        );
    }

    #[test]
    fn mul_scalar_mat4() {
        let scalar = 3.0;
        let mat = Mat4::<f64>::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        assert_eq!(
            (scalar * mat).approximate_eq(&Mat4::new(
                3.0, 6.0, 9.0, 12.0, 15.0, 18.0, 21.0, 24.0, 27.0, 30.0, 33.0, 36.0, 39.0, 42.0,
                45.0, 48.0
            )),
            true
        );
    }

    #[test]
    fn mul_assign_mat4_mat4() {
        let mut mat0 = Mat4::<f64>::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
        );
        let mat1 = Mat4::<f64>::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 4.0, 5.0, 6.0, 1.0,
        );
        mat0 *= mat1;
        assert_eq!(
            mat0.approximate_eq(&Mat4::new(
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 5.0, 7.0, 9.0, 1.0
            )),
            true
        );
    }

    #[test]
    fn mul_assign_mat4_scalar() {
        let mut mat = Mat4::<f64>::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        let scalar = 4.0;
        mat *= scalar;
        assert_eq!(
            mat.approximate_eq(&Mat4::new(
                4.0, 8.0, 12.0, 16.0, 20.0, 24.0, 28.0, 32.0, 36.0, 40.0, 44.0, 48.0, 52.0, 56.0,
                60.0, 64.0
            )),
            true
        );
    }

    #[test]
    fn neg() {
        let mat = Mat4::<f64>::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        assert_eq!(
            (-mat).approximate_eq(&Mat4::new(
                -1.0, -2.0, -3.0, -4.0, -5.0, -6.0, -7.0, -8.0, -9.0, -10.0, -11.0, -12.0, -13.0,
                -14.0, -15.0, -16.0
            )),
            true
        );
    }

    #[test]
    fn transpose() {
        let mat = Mat4::<f64>::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
        );
        assert_eq!(
            mat.transpose().approximate_eq(&Mat4::new(
                1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 2.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0, 1.0
            )),
            true
        );
    }

    #[test]
    fn transpose_in_place() {
        let mut mat = Mat4::<f64>::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
        );
        mat.transpose_in_place();
        assert_eq!(
            mat.approximate_eq(&Mat4::new(
                1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 2.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0, 1.0
            )),
            true
        );
    }

    #[test]
    fn invert() -> Result<(), Error> {
        let mat = Mat4::<f64>::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
        );
        assert_eq!(
            mat.invert()?.approximate_eq(&Mat4::new(
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, -2.0, -3.0, 1.0
            )),
            true
        );

        Ok(())
    }

    #[test]
    fn invert_in_place() -> Result<(), Error> {
        let mut mat = Mat4::<f64>::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
        );
        mat.invert_in_place()?;
        assert_eq!(
            mat.approximate_eq(&Mat4::new(
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, -2.0, -3.0, 1.0
            )),
            true
        );

        Ok(())
    }

    #[test]
    fn adjoint() {
        let mat = Mat4::<f64>::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
        );
        assert_eq!(
            mat.adjoint().approximate_eq(&Mat4::new(
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, -2.0, -3.0, 1.0
            )),
            true
        );
    }

    #[test]
    fn adjoint_in_place() {
        let mut mat = Mat4::<f64>::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
        );
        mat.adjoint_in_place();
        assert_eq!(
            mat.approximate_eq(&Mat4::new(
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, -2.0, -3.0, 1.0
            )),
            true
        );
    }

    #[test]
    fn determinant() {
        let mat = Mat4::<f64>::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
        );
        assert_eq!(mat.determinant().approximate_eq(&1.0), true);
    }

    #[test]
    fn translate() {
        let mat = Mat4::<f64>::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
        );
        assert_eq!(
            mat.translate(&Vec3::new(4.0, 5.0, 6.0))
                .approximate_eq(&Mat4::new(
                    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 5.0, 7.0, 9.0, 1.0
                )),
            true
        );
    }

    #[test]
    fn translate_in_place() {
        let mut mat = Mat4::<f64>::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
        );
        mat.translate_in_place(&Vec3::new(4.0, 5.0, 6.0));
        assert_eq!(
            mat.approximate_eq(&Mat4::new(
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 5.0, 7.0, 9.0, 1.0
            )),
            true
        );
    }

    #[test]
    fn rotate() -> Result<(), Error> {
        let mat = Mat4::<f64>::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
        );
        let axis = Vec3::<f64>::new(1.0, 0.0, 0.0);
        let rad = std::f64::consts::PI * 0.5;

        assert_eq!(
            mat.rotate(&axis, rad)?.approximate_eq(&Mat4::new(
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
            )),
            true
        );

        Ok(())
    }

    #[test]
    fn rotate_in_place() -> Result<(), Error> {
        let mut mat = Mat4::<f64>::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
        );
        let axis = Vec3::<f64>::new(1.0, 0.0, 0.0);
        let rad = std::f64::consts::PI * 0.5;
        mat.rotate_in_place(&axis, rad)?;

        assert_eq!(
            mat.approximate_eq(&Mat4::new(
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
            )),
            true
        );

        Ok(())
    }

    #[test]
    fn rotate_x() {
        let mat = Mat4::<f64>::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
        );
        let rad = std::f64::consts::PI * 0.5;

        assert_eq!(
            mat.rotate_x(rad).approximate_eq(&Mat4::new(
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
            )),
            true
        );
    }

    #[test]
    fn rotate_x_in_place() {
        let mut mat = Mat4::<f64>::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
        );
        let rad = std::f64::consts::PI * 0.5;

        mat.rotate_x_in_place(rad);
        assert_eq!(
            mat.approximate_eq(&Mat4::new(
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
            )),
            true
        );
    }

    #[test]
    fn rotate_y() {
        let mat = Mat4::<f64>::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
        );
        let rad = std::f64::consts::PI * 0.5;

        assert_eq!(
            mat.rotate_y(rad).approximate_eq(&Mat4::new(
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
            )),
            true
        );
    }

    #[test]
    fn rotate_y_in_place() {
        let mut mat = Mat4::<f64>::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
        );
        let rad = std::f64::consts::PI * 0.5;

        mat.rotate_y_in_place(rad);
        assert_eq!(
            mat.approximate_eq(&Mat4::new(
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
            )),
            true
        );
    }

    #[test]
    fn rotate_z() {
        let mat = Mat4::<f64>::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
        );
        let rad = std::f64::consts::PI * 0.5;

        assert_eq!(
            mat.rotate_z(rad).approximate_eq(&Mat4::new(
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
            )),
            true
        );
    }

    #[test]
    fn rotate_z_in_place() {
        let mut mat = Mat4::<f64>::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
        );
        let rad = std::f64::consts::PI * 0.5;

        mat.rotate_z_in_place(rad);
        assert_eq!(
            mat.approximate_eq(&Mat4::new(
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
            )),
            true
        );
    }

    #[test]
    fn scale() {
        let mat = Mat4::<f64>::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
        );
        assert_eq!(
            mat.scale(&Vec3::new(4.0, 5.0, 6.0))
                .approximate_eq(&Mat4::new(
                    4.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 1.0, 2.0, 3.0, 1.0
                )),
            true
        );
    }

    #[test]
    fn scale_in_place() {
        let mut mat = Mat4::<f64>::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
        );
        mat.scale_in_place(&Vec3::new(4.0, 5.0, 6.0));
        assert_eq!(
            mat.approximate_eq(&Mat4::new(
                4.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 1.0, 2.0, 3.0, 1.0
            )),
            true
        );
    }

    #[test]
    fn translation() {
        let translation = Mat4::<f64>::new_identity().translation();
        assert_eq!(translation.approximate_eq(&Vec3::new(0.0, 0.0, 0.0)), true);
    }

    #[test]
    fn scaling() {
        let scaling = Mat4::<f64>::new_identity().scaling();
        assert_eq!(scaling.approximate_eq(&Vec3::new(1.0, 1.0, 1.0)), true);
    }

    #[test]
    fn rotation() {
        let rotation = Mat4::<f64>::new_identity().rotation();
        assert_eq!(
            rotation.approximate_eq(&Quat::new(0.0, 0.0, 0.0, 1.0)),
            true
        );
    }

    #[test]
    fn decompose() {
        let (out_r, out_t, out_s) = Mat4::<f64>::new_identity().decompose();
        assert_eq!(out_r.approximate_eq(&Quat::new(0.0, 0.0, 0.0, 1.0)), true);
        assert_eq!(out_t.approximate_eq(&Vec3::new(0.0, 0.0, 0.0)), true);
        assert_eq!(out_s.approximate_eq(&Vec3::new(1.0, 1.0, 1.0)), true);
    }

    #[test]
    fn frob() {
        let mat = Mat4::<f64>::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
        );
        assert_eq!(mat.frob().approximate_eq(&4.242640687119285), true);
    }

    #[test]
    fn display() {
        let mat = Mat4::<f64>::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        assert_eq!(
            mat.to_string(),
            "mat4(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16)"
        );
    }
}
