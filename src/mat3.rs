use std::{
    fmt::{Debug, Display},
    ops::{Add, AddAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign},
    slice::SliceIndex,
};

use crate::{error::Error, mat2d::Mat2d, mat4::Mat4, quat::Quat, vec2::Vec2, ApproximateEq};

pub struct Mat3<T = f64>([T; 9]);

impl<T: Debug> Debug for Mat3<T> {
    #[inline(always)]
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_tuple("Mat3").field(&self.0).finish()
    }
}

impl<T: Copy> Copy for Mat3<T> {}

impl<T: Clone> Clone for Mat3<T> {
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
        ])
    }
}

impl<T: PartialEq> PartialEq for Mat3<T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl<T: Display> Display for Mat3<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_fmt(format_args!(
            "mat3({}, {}, {}, {}, {}, {}, {}, {}, {})",
            self.0[0],
            self.0[1],
            self.0[2],
            self.0[3],
            self.0[4],
            self.0[5],
            self.0[6],
            self.0[7],
            self.0[8],
        ))
    }
}

impl<T> Mat3<T> {
    #[inline(always)]
    pub const fn new(
        m00: T,
        m01: T,
        m02: T,
        m10: T,
        m11: T,
        m12: T,
        m20: T,
        m21: T,
        m22: T,
    ) -> Self {
        Self([m00, m01, m02, m10, m11, m12, m20, m21, m22])
    }

    #[inline(always)]
    pub const fn from_slice(values: [T; 9]) -> Self {
        Self(values)
    }
}

impl<T> Mat3<T> {
    #[inline(always)]
    pub fn raw(&self) -> &[T; 9] {
        &self.0
    }

    #[inline(always)]
    pub fn raw_mut(&mut self) -> &mut [T; 9] {
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
    pub fn m10(&self) -> &T {
        &self.0[3]
    }

    #[inline(always)]
    pub fn m11(&self) -> &T {
        &self.0[4]
    }

    #[inline(always)]
    pub fn m12(&self) -> &T {
        &self.0[5]
    }

    #[inline(always)]
    pub fn m20(&self) -> &T {
        &self.0[6]
    }

    #[inline(always)]
    pub fn m21(&self) -> &T {
        &self.0[7]
    }

    #[inline(always)]
    pub fn m22(&self) -> &T {
        &self.0[8]
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
    pub fn m10_mut(&mut self) -> &mut T {
        &mut self.0[3]
    }

    #[inline(always)]
    pub fn m11_mut(&mut self) -> &mut T {
        &mut self.0[4]
    }

    #[inline(always)]
    pub fn m12_mut(&mut self) -> &mut T {
        &mut self.0[5]
    }

    #[inline(always)]
    pub fn m20_mut(&mut self) -> &mut T {
        &mut self.0[6]
    }

    #[inline(always)]
    pub fn m21_mut(&mut self) -> &mut T {
        &mut self.0[7]
    }

    #[inline(always)]
    pub fn m22_mut(&mut self) -> &mut T {
        &mut self.0[8]
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
    pub fn set_m10(&mut self, m10: T) {
        self.0[3] = m10;
    }

    #[inline(always)]
    pub fn set_m11(&mut self, m11: T) {
        self.0[4] = m11;
    }

    #[inline(always)]
    pub fn set_m12(&mut self, m12: T) {
        self.0[5] = m12;
    }

    #[inline(always)]
    pub fn set_m20(&mut self, m20: T) {
        self.0[6] = m20;
    }

    #[inline(always)]
    pub fn set_m21(&mut self, m21: T) {
        self.0[7] = m21;
    }

    #[inline(always)]
    pub fn set_m22(&mut self, m22: T) {
        self.0[8] = m22;
    }

    #[inline(always)]
    pub fn set(&mut self, m00: T, m01: T, m02: T, m10: T, m11: T, m12: T, m20: T, m21: T, m22: T) {
        self.0[0] = m00;
        self.0[1] = m01;
        self.0[2] = m02;
        self.0[3] = m10;
        self.0[4] = m11;
        self.0[5] = m12;
        self.0[6] = m20;
        self.0[7] = m21;
        self.0[8] = m22;
    }
}

impl<T, I> Index<I> for Mat3<T>
where
    I: SliceIndex<[T], Output = T>,
{
    type Output = T;

    #[inline(always)]
    fn index(&self, index: I) -> &Self::Output {
        self.0.index(index)
    }
}

impl<T, I> IndexMut<I> for Mat3<T>
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
        impl Mat3<$t> {
            #[inline(always)]
            pub const fn new_zero() -> Self {
                Self([$zero, $zero, $zero, $zero, $zero, $zero, $zero, $zero, $zero])
            }

            #[inline(always)]
            pub const fn new_identity() -> Self {
                Self([$one, $zero, $zero, $zero, $one, $zero, $zero, $zero, $one])
            }
        }
       )+
    };
}

macro_rules! decimal_constructors {
    ($(($t: ident, $zero: expr, $one: expr, $two: expr)),+) => {
       $(
        impl Mat3<$t> {

            #[inline(always)]
            pub fn from_translation(v: &Vec2::<$t>) -> Self {
                Self([
                    $one,
                    $zero,
                    $zero,
                    $zero,
                    $one,
                    $zero,
                    *v.x(),
                    *v.y(),
                    $one
                ])
            }

            #[inline(always)]
            pub fn from_scaling(v: &Vec2::<$t>) -> Self {
                Self([
                    *v.x(),
                    $zero,
                    $zero,
                    $zero,
                    *v.y(),
                    $zero,
                    $zero,
                    $zero,
                    $one
                ])
            }

            #[inline(always)]
            pub fn from_rotation(rad: $t) -> Self {
                let s = rad.sin();
                let c = rad.cos();
                Self([
                    c,
                    s,
                    $zero,
                    -s,
                    c,
                    $zero,
                    $zero,
                    $zero,
                    $one,
                ])
            }

            #[inline(always)]
            pub fn from_mat_2d(mat: &Mat2d::<$t>) -> Self {
                Self([
                    *mat.a(),
                    *mat.b(),
                    $zero,
                    *mat.c(),
                    *mat.d(),
                    $zero,
                    *mat.tx(),
                    *mat.ty(),
                    $one,
                ])
            }

            #[inline(always)]
            pub fn from_mat4(mat: &Mat4::<$t>) -> Self {
                Self::new(
                    *mat.m00(),
                    *mat.m01(),
                    *mat.m02(),
                    *mat.m10(),
                    *mat.m11(),
                    *mat.m12(),
                    *mat.m20(),
                    *mat.m21(),
                    *mat.m22(),
                )
            }

            #[inline(always)]
            pub fn from_quat(q: &Quat::<$t>) -> Self {
                let x = *q.x();
                let y = *q.y();
                let z = *q.z();
                let w = *q.w();
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
                    yx - wz,
                    $one - xx - zz,
                    zy + wx,
                    zx + wy,
                    zy - wx,
                    $one - xx - yy,
                )
            }

            #[inline(always)]
            pub fn from_projection(width: $t, height: $t) -> Self {
                Self([
                    $two / width,
                    $zero,
                    $zero,
                    $zero,
                    -$two / height,
                    $zero,
                    -$one,
                    $one,
                    $one,
                ])
            }

            #[inline(always)]
            pub fn normal_matrix_from_mat4(a: &Mat4::<$t>) -> Result<Self, Error> {
                let a00 = *a.m00();
                let a01 = *a.m01();
                let a02 = *a.m02();
                let a03 = *a.m03();
                let a10 = *a.m10();
                let a11 = *a.m11();
                let a12 = *a.m12();
                let a13 = *a.m13();
                let a20 = *a.m20();
                let a21 = *a.m21();
                let a22 = *a.m22();
                let a23 = *a.m23();
                let a30 = *a.m30();
                let a31 = *a.m31();
                let a32 = *a.m32();
                let a33 = *a.m33();

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
                    (a12 * b08 - a10 * b11 - a13 * b07) * det,
                    (a10 * b10 - a11 * b08 + a13 * b06) * det,
                    (a02 * b10 - a01 * b11 - a03 * b09) * det,
                    (a00 * b11 - a02 * b08 + a03 * b07) * det,
                    (a01 * b08 - a00 * b10 - a03 * b06) * det,
                    (a31 * b05 - a32 * b04 + a33 * b03) * det,
                    (a32 * b02 - a30 * b05 - a33 * b01) * det,
                    (a30 * b04 - a31 * b02 + a33 * b00) * det,
                ))
            }
        }
       )+
    };
}

macro_rules! neg {
    ($($t: ident),+) => {
       $(
        impl Neg for Mat3<$t> {
            type Output = Mat3<$t>;

            #[inline(always)]
            fn neg(self) -> Self::Output {
                Self([
                    -self.0[0],
                    -self.0[1],
                    -self.0[2],
                    -self.0[3],
                    -self.0[4],
                    -self.0[5],
                    -self.0[6],
                    -self.0[7],
                    -self.0[8],
                ])
            }
        }
       )+
    };
}

macro_rules! math {
    ($(($t: ident, $epsilon: expr, $zero: expr, $one: expr)),+) => {
       $(
        impl Add for Mat3<$t> {
            type Output = Mat3<$t>;

            #[inline(always)]
            fn add(self, rhs: Self) -> Self::Output {
                Self([
                    self.0[0] + rhs.0[0],
                    self.0[1] + rhs.0[1],
                    self.0[2] + rhs.0[2],
                    self.0[3] + rhs.0[3],
                    self.0[4] + rhs.0[4],
                    self.0[5] + rhs.0[5],
                    self.0[6] + rhs.0[6],
                    self.0[7] + rhs.0[7],
                    self.0[8] + rhs.0[8],
                ])
            }
        }

        impl Add<$t> for Mat3<$t> {
            type Output = Mat3<$t>;

            #[inline(always)]
            fn add(self, rhs: $t) -> Self::Output {
                Self([
                    self.0[0] + rhs,
                    self.0[1] + rhs,
                    self.0[2] + rhs,
                    self.0[3] + rhs,
                    self.0[4] + rhs,
                    self.0[5] + rhs,
                    self.0[6] + rhs,
                    self.0[7] + rhs,
                    self.0[8] + rhs,
                ])
            }
        }

        impl Add<Mat3<$t>> for $t {
            type Output = Mat3<$t>;

            #[inline(always)]
            fn add(self, rhs: Mat3<$t>) -> Self::Output {
                Mat3::<$t>([
                    self + rhs.0[0],
                    self + rhs.0[1],
                    self + rhs.0[2],
                    self + rhs.0[3],
                    self + rhs.0[4],
                    self + rhs.0[5],
                    self + rhs.0[6],
                    self + rhs.0[7],
                    self + rhs.0[8],
                ])
            }
        }

        impl AddAssign for Mat3<$t> {
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
            }
        }

        impl AddAssign<$t> for Mat3<$t> {
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
            }
        }

        impl Sub for Mat3<$t> {
            type Output = Mat3<$t>;

            #[inline(always)]
            fn sub(self, rhs: Self) -> Self::Output {
                Self([
                    self.0[0] - rhs.0[0],
                    self.0[1] - rhs.0[1],
                    self.0[2] - rhs.0[2],
                    self.0[3] - rhs.0[3],
                    self.0[4] - rhs.0[4],
                    self.0[5] - rhs.0[5],
                    self.0[6] - rhs.0[6],
                    self.0[7] - rhs.0[7],
                    self.0[8] - rhs.0[8],
                ])
            }
        }

        impl Sub<$t> for Mat3<$t> {
            type Output = Mat3<$t>;

            #[inline(always)]
            fn sub(self, rhs: $t) -> Self::Output {
                Self([
                    self.0[0] - rhs,
                    self.0[1] - rhs,
                    self.0[2] - rhs,
                    self.0[3] - rhs,
                    self.0[4] - rhs,
                    self.0[5] - rhs,
                    self.0[6] - rhs,
                    self.0[7] - rhs,
                    self.0[8] - rhs,
                ])
            }
        }

        impl Sub<Mat3<$t>> for $t {
            type Output = Mat3<$t>;

            #[inline(always)]
            fn sub(self, rhs: Mat3<$t>) -> Self::Output {
                Mat3::<$t>([
                    self - rhs.0[0],
                    self - rhs.0[1],
                    self - rhs.0[2],
                    self - rhs.0[3],
                    self - rhs.0[4],
                    self - rhs.0[5],
                    self - rhs.0[6],
                    self - rhs.0[7],
                    self - rhs.0[8],
                ])
            }
        }

        impl SubAssign for Mat3<$t> {
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
            }
        }

        impl SubAssign<$t> for Mat3<$t> {
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
            }
        }

        impl Mul for Mat3<$t> {
            type Output = Mat3<$t>;

            #[inline(always)]
            fn mul(self, rhs: Self) -> Self::Output {
                let a00 = self.0[0];
                let a01 = self.0[1];
                let a02 = self.0[2];
                let a10 = self.0[3];
                let a11 = self.0[4];
                let a12 = self.0[5];
                let a20 = self.0[6];
                let a21 = self.0[7];
                let a22 = self.0[8];

                let b00 = rhs.0[0];
                let b01 = rhs.0[1];
                let b02 = rhs.0[2];
                let b10 = rhs.0[3];
                let b11 = rhs.0[4];
                let b12 = rhs.0[5];
                let b20 = rhs.0[6];
                let b21 = rhs.0[7];
                let b22 = rhs.0[8];

                Self([
                    b00 * a00 + b01 * a10 + b02 * a20,
                    b00 * a01 + b01 * a11 + b02 * a21,
                    b00 * a02 + b01 * a12 + b02 * a22,
                    b10 * a00 + b11 * a10 + b12 * a20,
                    b10 * a01 + b11 * a11 + b12 * a21,
                    b10 * a02 + b11 * a12 + b12 * a22,
                    b20 * a00 + b21 * a10 + b22 * a20,
                    b20 * a01 + b21 * a11 + b22 * a21,
                    b20 * a02 + b21 * a12 + b22 * a22,
                ])
            }
        }

        impl Mul<$t> for Mat3<$t> {
            type Output = Mat3<$t>;

            #[inline(always)]
            fn mul(self, rhs: $t) -> Self::Output {
                Self([
                    self.0[0] * rhs,
                    self.0[1] * rhs,
                    self.0[2] * rhs,
                    self.0[3] * rhs,
                    self.0[4] * rhs,
                    self.0[5] * rhs,
                    self.0[6] * rhs,
                    self.0[7] * rhs,
                    self.0[8] * rhs,
                ])
            }
        }

        impl Mul<Mat3<$t>> for $t {
            type Output = Mat3<$t>;

            #[inline(always)]
            fn mul(self, rhs: Mat3<$t>) -> Self::Output {
                Mat3::<$t>([
                    self * rhs.0[0],
                    self * rhs.0[1],
                    self * rhs.0[2],
                    self * rhs.0[3],
                    self * rhs.0[4],
                    self * rhs.0[5],
                    self * rhs.0[6],
                    self * rhs.0[7],
                    self * rhs.0[8],
                ])
            }
        }

        impl MulAssign for Mat3<$t> {
            #[inline(always)]
            fn mul_assign(&mut self, rhs: Self) {
                let a00 = self.0[0];
                let a01 = self.0[1];
                let a02 = self.0[2];
                let a10 = self.0[3];
                let a11 = self.0[4];
                let a12 = self.0[5];
                let a20 = self.0[6];
                let a21 = self.0[7];
                let a22 = self.0[8];

                let b00 = rhs.0[0];
                let b01 = rhs.0[1];
                let b02 = rhs.0[2];
                let b10 = rhs.0[3];
                let b11 = rhs.0[4];
                let b12 = rhs.0[5];
                let b20 = rhs.0[6];
                let b21 = rhs.0[7];
                let b22 = rhs.0[8];

                self.0[0] = b00 * a00 + b01 * a10 + b02 * a20;
                self.0[1] = b00 * a01 + b01 * a11 + b02 * a21;
                self.0[2] = b00 * a02 + b01 * a12 + b02 * a22;
                self.0[3] = b10 * a00 + b11 * a10 + b12 * a20;
                self.0[4] = b10 * a01 + b11 * a11 + b12 * a21;
                self.0[5] = b10 * a02 + b11 * a12 + b12 * a22;
                self.0[6] = b20 * a00 + b21 * a10 + b22 * a20;
                self.0[7] = b20 * a01 + b21 * a11 + b22 * a21;
                self.0[8] = b20 * a02 + b21 * a12 + b22 * a22;
            }
        }

        impl MulAssign<$t> for Mat3<$t> {
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
            }
        }

        impl Mat3<$t> {
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
                self
            }

            #[inline(always)]
            pub fn set_identify(&mut self) -> &mut Self {
                self.0[0] = $one;
                self.0[1] = $zero;
                self.0[2] = $zero;
                self.0[3] = $zero;
                self.0[4] = $one;
                self.0[5] = $zero;
                self.0[6] = $zero;
                self.0[7] = $zero;
                self.0[8] = $one;
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
                self
            }

            #[inline(always)]
            pub fn transpose(&self) -> Self {
                let m00 = self.0[0];
                let m01 = self.0[1];
                let m02 = self.0[2];
                let m10 = self.0[3];
                let m11 = self.0[4];
                let m12 = self.0[5];
                let m20 = self.0[6];
                let m21 = self.0[7];
                let m22 = self.0[8];

                Self::new(m00, m10, m20, m01, m11, m21, m02, m12, m22)
            }

            #[inline(always)]
            pub fn transpose_in_place(&mut self) -> &mut Self {
                let m00 = self.0[0];
                let m01 = self.0[1];
                let m02 = self.0[2];
                let m10 = self.0[3];
                let m11 = self.0[4];
                let m12 = self.0[5];
                let m20 = self.0[6];
                let m21 = self.0[7];
                let m22 = self.0[8];

                self.0[0] = m00;
                self.0[1] = m10;
                self.0[2] = m20;
                self.0[3] = m01;
                self.0[4] = m11;
                self.0[5] = m21;
                self.0[6] = m02;
                self.0[7] = m12;
                self.0[8] = m22;
                self
            }

            #[inline(always)]
            pub fn adjoint(&self) -> Self {
                let m00 = self.0[0];
                let m01 = self.0[1];
                let m02 = self.0[2];
                let m10 = self.0[3];
                let m11 = self.0[4];
                let m12 = self.0[5];
                let m20 = self.0[6];
                let m21 = self.0[7];
                let m22 = self.0[8];

                Self::new(
                    m11 * m22 - m12 * m21,
                    m02 * m21 - m01 * m22,
                    m01 * m12 - m02 * m11,
                    m12 * m20 - m10 * m22,
                    m00 * m22 - m02 * m20,
                    m02 * m10 - m00 * m12,
                    m10 * m21 - m11 * m20,
                    m01 * m20 - m00 * m21,
                    m00 * m11 - m01 * m10,
                )
            }

            #[inline(always)]
            pub fn adjoint_in_place(&mut self) -> &mut Self {
                let m00 = self.0[0];
                let m01 = self.0[1];
                let m02 = self.0[2];
                let m10 = self.0[3];
                let m11 = self.0[4];
                let m12 = self.0[5];
                let m20 = self.0[6];
                let m21 = self.0[7];
                let m22 = self.0[8];


                self.0[0] = m11 * m22 - m12 * m21;
                self.0[1] = m02 * m21 - m01 * m22;
                self.0[2] = m01 * m12 - m02 * m11;
                self.0[3] = m12 * m20 - m10 * m22;
                self.0[4] = m00 * m22 - m02 * m20;
                self.0[5] = m02 * m10 - m00 * m12;
                self.0[6] = m10 * m21 - m11 * m20;
                self.0[7] = m01 * m20 - m00 * m21;
                self.0[8] = m00 * m11 - m01 * m10;
                self
            }

            #[inline(always)]
            pub fn determinant(&self) -> $t {
                let a00 = self.0[0];
                let a01 = self.0[1];
                let a02 = self.0[2];
                let a10 = self.0[3];
                let a11 = self.0[4];
                let a12 = self.0[5];
                let a20 = self.0[6];
                let a21 = self.0[7];
                let a22 = self.0[8];
                a00 * (a22 * a11 - a12 * a21)
                    + a01 * (-a22 * a10 + a12 * a20)
                    + a02 * (a21 * a10 - a11 * a20)
            }

            #[inline(always)]
            pub fn invert(&self) -> Result<Self, Error> {
                let a00 = self.0[0];
                let a01 = self.0[1];
                let a02 = self.0[2];
                let a10 = self.0[3];
                let a11 = self.0[4];
                let a12 = self.0[5];
                let a20 = self.0[6];
                let a21 = self.0[7];
                let a22 = self.0[8];

                let b01 = a22 * a11 - a12 * a21;
                let b11 = -a22 * a10 + a12 * a20;
                let b21 = a21 * a10 - a11 * a20;

                // Calculate the determinant
                let mut det = a00 * b01 + a01 * b11 + a02 * b21;

                if det == $zero {
                    return Err(Error::ZeroDeterminant);
                }
                det = $one / det;

                Ok(Self::new(
                    b01 * det,
                    (-a22 * a01 + a02 * a21) * det,
                    (a12 * a01 - a02 * a11) * det,
                    b11 * det,
                    (a22 * a00 - a02 * a20) * det,
                    (-a12 * a00 + a02 * a10) * det,
                    b21 * det,
                    (-a21 * a00 + a01 * a20) * det,
                    (a11 * a00 - a01 * a10) * det,
                ))
            }

            #[inline(always)]
            pub fn invert_in_place(&mut self) -> Result<&mut Self, Error> {
                let a00 = self.0[0];
                let a01 = self.0[1];
                let a02 = self.0[2];
                let a10 = self.0[3];
                let a11 = self.0[4];
                let a12 = self.0[5];
                let a20 = self.0[6];
                let a21 = self.0[7];
                let a22 = self.0[8];

                let b01 = a22 * a11 - a12 * a21;
                let b11 = -a22 * a10 + a12 * a20;
                let b21 = a21 * a10 - a11 * a20;

                // Calculate the determinant
                let mut det = a00 * b01 + a01 * b11 + a02 * b21;

                if det == $zero {
                    return Err(Error::ZeroDeterminant);
                }
                det = $one / det;

                self.0[0] = b01 * det;
                self.0[1] = (-a22 * a01 + a02 * a21) * det;
                self.0[2] = (a12 * a01 - a02 * a11) * det;
                self.0[3] = b11 * det;
                self.0[4] = (a22 * a00 - a02 * a20) * det;
                self.0[5] = (-a12 * a00 + a02 * a10) * det;
                self.0[6] = b21 * det;
                self.0[7] = (-a21 * a00 + a01 * a20) * det;
                self.0[8] = (a11 * a00 - a01 * a10) * det;
                Ok(self)
            }

            #[inline(always)]
            pub fn rotate(&self, rad: $t) -> Self {
                let a00 = self.0[0];
                let a01 = self.0[1];
                let a02 = self.0[2];
                let a10 = self.0[3];
                let a11 = self.0[4];
                let a12 = self.0[5];
                let a20 = self.0[6];
                let a21 = self.0[7];
                let a22 = self.0[8];

                let s = rad.sin();
                let c = rad.cos();

                Self::new(
                    c * a00 + s * a10,
                    c * a01 + s * a11,
                    c * a02 + s * a12,
                    c * a10 - s * a00,
                    c * a11 - s * a01,
                    c * a12 - s * a02,
                    a20,
                    a21,
                    a22,
                )
            }

            #[inline(always)]
            pub fn rotate_in_place(&mut self, rad: $t) -> &mut Self {
                let a00 = self.0[0];
                let a01 = self.0[1];
                let a02 = self.0[2];
                let a10 = self.0[3];
                let a11 = self.0[4];
                let a12 = self.0[5];
                let a20 = self.0[6];
                let a21 = self.0[7];
                let a22 = self.0[8];

                let s = rad.sin();
                let c = rad.cos();

                self.0[0] = c * a00 + s * a10;
                self.0[1] = c * a01 + s * a11;
                self.0[2] = c * a02 + s * a12;
                self.0[3] = c * a10 - s * a00;
                self.0[4] = c * a11 - s * a01;
                self.0[5] = c * a12 - s * a02;
                self.0[6] = a20;
                self.0[7] = a21;
                self.0[8] = a22;
                self
            }

            #[inline(always)]
            pub fn translate(&self, v: &Vec2::<$t>) -> Self {
                let a00 = self.0[0];
                let a01 = self.0[1];
                let a02 = self.0[2];
                let a10 = self.0[3];
                let a11 = self.0[4];
                let a12 = self.0[5];
                let a20 = self.0[6];
                let a21 = self.0[7];
                let a22 = self.0[8];
                let x = v.x();
                let y = v.y();

                Self::new(
                    a00,
                    a01,
                    a02,
                    a10,
                    a11,
                    a12,
                    x * a00 + y * a10 + a20,
                    x * a01 + y * a11 + a21,
                    x * a02 + y * a12 + a22,
                )
            }

            #[inline(always)]
            pub fn translate_in_place(&mut self, v: &Vec2::<$t>) -> &mut Self {
                let a00 = self.0[0];
                let a01 = self.0[1];
                let a02 = self.0[2];
                let a10 = self.0[3];
                let a11 = self.0[4];
                let a12 = self.0[5];
                let a20 = self.0[6];
                let a21 = self.0[7];
                let a22 = self.0[8];
                let x = v.x();
                let y = v.y();

                self.0[0] = a00;
                self.0[1] = a01;
                self.0[2] = a02;
                self.0[3] = a10;
                self.0[4] = a11;
                self.0[5] = a12;
                self.0[6] = x * a00 + y * a10 + a20;
                self.0[7] = x * a01 + y * a11 + a21;
                self.0[8] = x * a02 + y * a12 + a22;
                self
            }

            #[inline(always)]
            pub fn scale(&self, v: &Vec2::<$t>) -> Self {
                let x = v.x();
                let y = v.y();

                let a00 = self.0[0];
                let a01 = self.0[1];
                let a02 = self.0[2];
                let a10 = self.0[3];
                let a11 = self.0[4];
                let a12 = self.0[5];
                let a20 = self.0[6];
                let a21 = self.0[7];
                let a22 = self.0[8];

                Self::new(
                    x * a00,
                    x * a01,
                    x * a02,
                    y * a10,
                    y * a11,
                    y * a12,
                    a20,
                    a21,
                    a22,
                )
            }

            #[inline(always)]
            pub fn scale_in_place(&mut self, v: &Vec2::<$t>) -> &mut Self {
                let x = v.x();
                let y = v.y();

                let a00 = *self.m00();
                let a01 = *self.m01();
                let a02 = *self.m02();
                let a10 = *self.m10();
                let a11 = *self.m11();
                let a12 = *self.m12();
                let a20 = *self.m20();
                let a21 = *self.m21();
                let a22 = *self.m22();

                self.0[0] = x * a00;
                self.0[1] = x * a01;
                self.0[2] = x * a02;
                self.0[3] = y * a10;
                self.0[4] = y * a11;
                self.0[5] = y * a12;
                self.0[6] = a20;
                self.0[7] = a21;
                self.0[8] = a22;
                self
            }

            #[inline(always)]
            pub fn frob(&self) -> $t {
                let a00 = self.0[0];
                let a01 = self.0[1];
                let a02 = self.0[2];
                let a10 = self.0[3];
                let a11 = self.0[4];
                let a12 = self.0[5];
                let a20 = self.0[6];
                let a21 = self.0[7];
                let a22 = self.0[8];

                (a00 * a00
                    + a01 * a01
                    + a02 * a02
                    + a10 * a10
                    + a11 * a11
                    + a12 * a12
                    + a20 * a20
                    + a21 * a21
                    + a22 * a22)
                    .sqrt()
            }
        }

        impl ApproximateEq for Mat3<$t> {
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

                let b0 = other.0[0];
                let b1 = other.0[1];
                let b2 = other.0[2];
                let b3 = other.0[3];
                let b4 = other.0[4];
                let b5 = other.0[5];
                let b6 = other.0[6];
                let b7 = other.0[7];
                let b8 = other.0[8];

                (a0 - b0).abs() <= $epsilon * $one.max(a0.abs()).max(b0.abs())
                    && (a1 - b1).abs() <= $epsilon * $one.max(a1.abs()).max(b1.abs())
                    && (a2 - b2).abs() <= $epsilon * $one.max(a2.abs()).max(b2.abs())
                    && (a3 - b3).abs() <= $epsilon * $one.max(a3.abs()).max(b3.abs())
                    && (a4 - b4).abs() <= $epsilon * $one.max(a4.abs()).max(b4.abs())
                    && (a5 - b5).abs() <= $epsilon * $one.max(a5.abs()).max(b5.abs())
                    && (a6 - b6).abs() <= $epsilon * $one.max(a6.abs()).max(b6.abs())
                    && (a7 - b7).abs() <= $epsilon * $one.max(a7.abs()).max(b7.abs())
                    && (a8 - b8).abs() <= $epsilon * $one.max(a8.abs()).max(b8.abs())
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
    (f32, 0.0f32, 1.0f32, 2.0f32),
    (f64, 0.0f64, 1.0f64, 2.0f64)
}
neg!(i8, i16, i32, i64, i128, isize, f32, f64);
math! {
    (f32, super::EPSILON_F32, 0.0f32, 1.0f32),
    (f64, super::EPSILON_F64, 0.0f64, 1.0f64)
}

#[cfg(feature = "gl")]
impl super::GLF32<9> for Mat3<f32> {
    #[inline(always)]
    fn gl_f32(&self) -> [f32; 9] {
        self.0.clone()
    }
}

#[cfg(feature = "gl")]
impl super::GLF32<9> for Mat3<f64> {
    #[inline(always)]
    fn gl_f32(&self) -> [f32; 9] {
        [
            self.0[0] as f32,
            self.0[1] as f32,
            self.0[2] as f32,
            self.0[3] as f32,
            self.0[4] as f32,
            self.0[5] as f32,
            self.0[6] as f32,
            self.0[7] as f32,
            self.0[8] as f32,
        ]
    }
}

#[cfg(feature = "gl")]
impl super::GLF32Borrowed<9> for Mat3<f32> {
    #[inline(always)]
    fn gl_f32_borrowed(&self) -> &[f32; 9] {
        &self.0
    }
}

#[cfg(test)]
mod tests {
    use crate::{error::Error, mat4::Mat4, quat::Quat, vec3::Vec3, ApproximateEq};

    use super::Mat3;

    #[test]
    fn new() {
        assert_eq!(
            Mat3::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 1.0).raw(),
            &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 1.0]
        );
    }

    #[test]
    fn new_zero() {
        assert_eq!(
            Mat3::<f64>::new_zero().raw(),
            &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        );
    }

    #[test]
    fn new_identity() {
        assert_eq!(
            Mat3::<f64>::new_identity().raw(),
            &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        );
    }

    #[test]
    fn from_slice() {
        assert_eq!(
            Mat3::from_slice([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 1.0]).raw(),
            &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 1.0]
        );
    }

    #[test]
    fn from_quat() {
        let q = Quat::<f64>::new(0.0, -0.7071067811865475, 0.0, 0.7071067811865475);
        let m = Mat3::<f64>::from_quat(&q);
        let v = Vec3::<f64>::new(0.0, 0.0, -1.0);

        assert_eq!((m * v).approximate_eq(&(q * v)), true);
        assert_eq!((m * v).approximate_eq(&Vec3::new(1.0, 0.0, 0.0)), true);
    }

    #[test]
    fn from_mat4() {
        let mat4 = Mat4::<f64>::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );

        assert_eq!(
            Mat3::<f64>::from_mat4(&mat4)
                .approximate_eq(&Mat3::new(1.0, 2.0, 3.0, 5.0, 6.0, 7.0, 9.0, 10.0, 11.0)),
            true
        );
    }

    #[test]
    fn normal_matrix_from_mat4() -> Result<(), Error> {
        let mat4 = Mat4::<f64>::new_identity()
            .translate(&Vec3::new(2.0, 4.0, 6.0))
            .rotate_x(std::f64::consts::PI / 2.0);

        assert_eq!(
            Mat3::<f64>::normal_matrix_from_mat4(&mat4)?
                .approximate_eq(&Mat3::new(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0,)),
            true
        );

        let mat4 = mat4.scale(&Vec3::new(2.0, 3.0, 4.0));

        assert_eq!(
            Mat3::<f64>::normal_matrix_from_mat4(&mat4)?.approximate_eq(&Mat3::new(
                0.5,
                0.0,
                0.0,
                0.0,
                0.0,
                0.3333333333333333,
                0.0,
                -0.25,
                0.0,
            )),
            true
        );

        Ok(())
    }

    #[test]
    fn from_projection() {
        assert_eq!(
            Mat3::<f64>::from_projection(100.0, 200.0)
                .approximate_eq(&Mat3::new(0.02, 0.0, 0.0, 0.0, -0.01, 0.0, -1.0, 1.0, 1.0)),
            true
        );
    }

    #[test]
    fn raw() {
        assert_eq!(
            Mat3::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 1.0).raw(),
            &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 1.0]
        );
    }

    #[test]
    fn raw_mut() {
        let mut mat = Mat3::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 1.0);
        (*mat.raw_mut()) = [11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0];
        assert_eq!(
            mat.raw(),
            &[11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0]
        );
    }

    #[test]
    fn m00() {
        assert_eq!(
            Mat3::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0).m00(),
            &1.0
        );
    }

    #[test]
    fn m01() {
        assert_eq!(
            Mat3::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0).m01(),
            &2.0
        );
    }

    #[test]
    fn m02() {
        assert_eq!(
            Mat3::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0).m02(),
            &3.0
        );
    }

    #[test]
    fn m10() {
        assert_eq!(
            Mat3::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0).m10(),
            &4.0
        );
    }

    #[test]
    fn m11() {
        assert_eq!(
            Mat3::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0).m11(),
            &5.0
        );
    }

    #[test]
    fn m12() {
        assert_eq!(
            Mat3::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0).m12(),
            &6.0
        );
    }

    #[test]
    fn m20() {
        assert_eq!(
            Mat3::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0).m20(),
            &7.0
        );
    }

    #[test]
    fn m21() {
        assert_eq!(
            Mat3::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0).m21(),
            &8.0
        );
    }

    #[test]
    fn m22() {
        assert_eq!(
            Mat3::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0).m22(),
            &9.0
        );
    }

    #[test]
    fn m00_mut() {
        let mut mat = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        (*mat.m00_mut()) = 10.0;
        assert_eq!(mat.raw(), &[10.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
    }

    #[test]
    fn m01_mut() {
        let mut mat = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        (*mat.m01_mut()) = 10.0;
        assert_eq!(mat.raw(), &[1.0, 10.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
    }

    #[test]
    fn m02_mut() {
        let mut mat = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        (*mat.m02_mut()) = 10.0;
        assert_eq!(mat.raw(), &[1.0, 2.0, 10.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
    }

    #[test]
    fn m10_mut() {
        let mut mat = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        (*mat.m10_mut()) = 10.0;
        assert_eq!(mat.raw(), &[1.0, 2.0, 3.0, 10.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
    }

    #[test]
    fn m11_mut() {
        let mut mat = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        (*mat.m11_mut()) = 10.0;
        assert_eq!(mat.raw(), &[1.0, 2.0, 3.0, 4.0, 10.0, 6.0, 7.0, 8.0, 9.0]);
    }

    #[test]
    fn m12_mut() {
        let mut mat = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        (*mat.m12_mut()) = 10.0;
        assert_eq!(mat.raw(), &[1.0, 2.0, 3.0, 4.0, 5.0, 10.0, 7.0, 8.0, 9.0]);
    }

    #[test]
    fn m20_mut() {
        let mut mat = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        (*mat.m20_mut()) = 10.0;
        assert_eq!(mat.raw(), &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 10.0, 8.0, 9.0]);
    }

    #[test]
    fn m21_mut() {
        let mut mat = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        (*mat.m21_mut()) = 10.0;
        assert_eq!(mat.raw(), &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 10.0, 9.0]);
    }

    #[test]
    fn m22_mut() {
        let mut mat = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        (*mat.m22_mut()) = 10.0;
        assert_eq!(mat.raw(), &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 10.0]);
    }

    #[test]
    fn set_m00() {
        let mut mat = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        mat.set_m00(10.0);
        assert_eq!(mat.raw(), &[10.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
    }

    #[test]
    fn set_m01() {
        let mut mat = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        mat.set_m01(10.0);
        assert_eq!(mat.raw(), &[1.0, 10.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
    }

    #[test]
    fn set_m02() {
        let mut mat = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        mat.set_m02(10.0);
        assert_eq!(mat.raw(), &[1.0, 2.0, 10.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
    }

    #[test]
    fn set_m10() {
        let mut mat = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        mat.set_m10(10.0);
        assert_eq!(mat.raw(), &[1.0, 2.0, 3.0, 10.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
    }

    #[test]
    fn set_m11() {
        let mut mat = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        mat.set_m11(10.0);
        assert_eq!(mat.raw(), &[1.0, 2.0, 3.0, 4.0, 10.0, 6.0, 7.0, 8.0, 9.0]);
    }

    #[test]
    fn set_m12() {
        let mut mat = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        mat.set_m12(10.0);
        assert_eq!(mat.raw(), &[1.0, 2.0, 3.0, 4.0, 5.0, 10.0, 7.0, 8.0, 9.0]);
    }

    #[test]
    fn set_m20() {
        let mut mat = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        mat.set_m20(10.0);
        assert_eq!(mat.raw(), &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 10.0, 8.0, 9.0]);
    }

    #[test]
    fn set_m21() {
        let mut mat = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        mat.set_m21(10.0);
        assert_eq!(mat.raw(), &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 10.0, 9.0]);
    }

    #[test]
    fn set_m22() {
        let mut mat = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        mat.set_m22(10.0);
        assert_eq!(mat.raw(), &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 10.0]);
    }

    #[test]
    fn set() {
        let mut mat = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        mat.set(10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0);
        assert_eq!(
            mat.raw(),
            &[10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
        );
    }

    #[test]
    fn set_zero() {
        let mut mat = Mat3::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        mat.set_zero();
        assert_eq!(mat.raw(), &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn set_identify() {
        let mut mat = Mat3::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        mat.set_identify();
        assert_eq!(mat.raw(), &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn add_mat3_mat3() {
        let mat0 = Mat3::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let mat1 = Mat3::<f64>::new(10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0);
        assert_eq!(
            (mat0 + mat1).approximate_eq(&Mat3::new(
                11.0, 13.0, 15.0, 17.0, 19.0, 21.0, 23.0, 25.0, 27.0
            )),
            true
        );
    }

    #[test]
    fn add_mat3_scalar() {
        let mat = Mat3::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let scalar = 1.0;
        assert_eq!(
            (mat + scalar).approximate_eq(&Mat3::new(2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0)),
            true
        );
    }

    #[test]
    fn add_scalar_mat3() {
        let scalar = 1.0;
        let mat = Mat3::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        assert_eq!(
            (scalar + mat).approximate_eq(&Mat3::new(2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0)),
            true
        );
    }

    #[test]
    fn add_assign_mat3_mat3() {
        let mut mat0 = Mat3::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let mat1 = Mat3::<f64>::new(10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0);
        mat0 += mat1;
        assert_eq!(
            mat0.approximate_eq(&Mat3::new(
                11.0, 13.0, 15.0, 17.0, 19.0, 21.0, 23.0, 25.0, 27.0
            )),
            true
        );
    }

    #[test]
    fn add_assign_mat3_scalar() {
        let mut mat = Mat3::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let scalar = 1.0;
        mat += scalar;
        assert_eq!(
            mat.approximate_eq(&Mat3::new(2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0)),
            true
        );
    }

    #[test]
    fn sub_mat3_mat3() {
        let mat0 = Mat3::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let mat1 = Mat3::<f64>::new(10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0);
        assert_eq!(
            (mat0 - mat1).approximate_eq(&Mat3::new(
                -9.0, -9.0, -9.0, -9.0, -9.0, -9.0, -9.0, -9.0, -9.0
            )),
            true
        );
    }

    #[test]
    fn sub_mat3_scalar() {
        let mat = Mat3::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let scalar = 1.0;
        assert_eq!(
            (mat - scalar).approximate_eq(&Mat3::new(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0)),
            true
        );
    }

    #[test]
    fn sub_scalar_mat3() {
        let scalar = 1.0;
        let mat = Mat3::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        assert_eq!(
            (scalar - mat).approximate_eq(&Mat3::new(
                0.0, -1.0, -2.0, -3.0, -4.0, -5.0, -6.0, -7.0, -8.0
            )),
            true
        );
    }

    #[test]
    fn sub_assign_mat3_mat3() {
        let mut mat0 = Mat3::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let mat1 = Mat3::<f64>::new(10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0);
        mat0 -= mat1;
        assert_eq!(
            mat0.approximate_eq(&Mat3::new(
                -9.0, -9.0, -9.0, -9.0, -9.0, -9.0, -9.0, -9.0, -9.0
            )),
            true
        );
    }

    #[test]
    fn sub_assign_mat3_scalar() {
        let mut mat = Mat3::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let scalar = 1.0;
        mat -= scalar;
        assert_eq!(
            mat.approximate_eq(&Mat3::new(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0)),
            true
        );
    }

    #[test]
    fn mul_mat3_mat3() {
        let mat0 = Mat3::<f64>::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 1.0);
        let mat1 = Mat3::<f64>::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 3.0, 4.0, 1.0);
        assert_eq!(
            (mat0 * mat1).approximate_eq(&Mat3::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 4.0, 6.0, 1.0)),
            true
        );
    }

    #[test]
    fn mul_mat3_scalar() {
        let mat = Mat3::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let scalar = 2.0;
        assert_eq!(
            (mat * scalar)
                .approximate_eq(&Mat3::new(2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0)),
            true
        );
    }

    #[test]
    fn mul_scalar_mat3() {
        let scalar = 3.0;
        let mat = Mat3::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        assert_eq!(
            (scalar * mat).approximate_eq(&Mat3::new(
                3.0, 6.0, 9.0, 12.0, 15.0, 18.0, 21.0, 24.0, 27.0
            )),
            true
        );
    }

    #[test]
    fn mul_assign_mat3_mat3() {
        let mut mat0 = Mat3::<f64>::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 1.0);
        let mat1 = Mat3::<f64>::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 3.0, 4.0, 1.0);
        mat0 *= mat1;
        assert_eq!(
            mat0.approximate_eq(&Mat3::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 4.0, 6.0, 1.0)),
            true
        );
    }

    #[test]
    fn mul_assign_mat3_scalar() {
        let mut mat = Mat3::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let scalar = 4.0;
        mat *= scalar;
        assert_eq!(
            mat.approximate_eq(&Mat3::new(
                4.0, 8.0, 12.0, 16.0, 20.0, 24.0, 28.0, 32.0, 36.0
            )),
            true
        );
    }

    #[test]
    fn neg() {
        let mat = Mat3::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        assert_eq!(
            (-mat).approximate_eq(&Mat3::new(
                -1.0, -2.0, -3.0, -4.0, -5.0, -6.0, -7.0, -8.0, -9.0
            )),
            true
        );
    }

    #[test]
    fn frob() {
        let mat = Mat3::<f64>::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 1.0);
        assert_eq!(mat.frob().approximate_eq(&2.8284271247461903), true);
    }

    #[test]
    fn determinant() {
        let mat = Mat3::<f64>::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 1.0);
        assert_eq!(mat.determinant().approximate_eq(&1.0), true);
    }

    #[test]
    fn invert() -> Result<(), Error> {
        let mat = Mat3::<f64>::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 1.0);
        assert_eq!(
            mat.invert()?
                .approximate_eq(&Mat3::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, -2.0, 1.0)),
            true
        );
        Ok(())
    }

    #[test]
    fn invert_in_place() -> Result<(), Error> {
        let mut mat = Mat3::<f64>::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 1.0);
        mat.invert_in_place()?;
        assert_eq!(
            mat.approximate_eq(&Mat3::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, -2.0, 1.0)),
            true
        );
        Ok(())
    }

    #[test]
    fn adjoint() -> Result<(), Error> {
        let mat = Mat3::<f64>::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 1.0);
        assert_eq!(
            mat.adjoint()
                .approximate_eq(&Mat3::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, -2.0, 1.0)),
            true
        );
        Ok(())
    }

    #[test]
    fn adjoint_in_place() -> Result<(), Error> {
        let mut mat = Mat3::<f64>::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 1.0);
        mat.adjoint_in_place();
        assert_eq!(
            mat.approximate_eq(&Mat3::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, -2.0, 1.0)),
            true
        );
        Ok(())
    }

    #[test]
    fn display() {
        let mat = Mat3::<f64>::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 1.0);
        assert_eq!(mat.to_string(), "mat3(1, 0, 0, 0, 1, 0, 1, 2, 1)");
    }
}
