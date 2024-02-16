use std::{
    fmt::{Debug, Display},
    ops::{Add, AddAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign},
    slice::SliceIndex,
};

use crate::{error::Error, vec2::Vec2, ApproximateEq};

pub struct Mat2<T = f64>([T; 4]);

impl<T: Debug> Debug for Mat2<T> {
    #[inline(always)]
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_tuple("Mat2").field(&self.0).finish()
    }
}

impl<T: Copy> Copy for Mat2<T> {}

impl<T: Clone> Clone for Mat2<T> {
    #[inline(always)]
    fn clone(&self) -> Self {
        Self([
            self.0[0].clone(),
            self.0[1].clone(),
            self.0[2].clone(),
            self.0[3].clone(),
        ])
    }
}

impl<T: PartialEq> PartialEq for Mat2<T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl<T: Display> Display for Mat2<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_fmt(format_args!(
            "mat2({}, {}, {}, {})",
            self.0[0], self.0[1], self.0[2], self.0[3]
        ))
    }
}

impl<T> Mat2<T> {
    #[inline(always)]
    pub const fn new(m00: T, m01: T, m10: T, m11: T) -> Self {
        Self([m00, m01, m10, m11])
    }

    #[inline(always)]
    pub const fn from_slice(values: [T; 4]) -> Self {
        Self(values)
    }
}

impl<T> Mat2<T> {
    #[inline(always)]
    pub fn raw(&self) -> &[T; 4] {
        &self.0
    }

    #[inline(always)]
    pub fn raw_mut(&mut self) -> &mut [T; 4] {
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
    pub fn m10(&self) -> &T {
        &self.0[2]
    }

    #[inline(always)]
    pub fn m11(&self) -> &T {
        &self.0[3]
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
    pub fn m10_mut(&mut self) -> &mut T {
        &mut self.0[2]
    }

    #[inline(always)]
    pub fn m11_mut(&mut self) -> &mut T {
        &mut self.0[3]
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
    pub fn set_m10(&mut self, m10: T) {
        self.0[2] = m10;
    }

    #[inline(always)]
    pub fn set_m11(&mut self, m11: T) {
        self.0[3] = m11;
    }

    #[inline(always)]
    pub fn set(&mut self, m00: T, m01: T, m10: T, m11: T) {
        self.0[0] = m00;
        self.0[1] = m01;
        self.0[2] = m10;
        self.0[3] = m11;
    }
}

impl<T: Clone> Mat2<T> {
    #[inline(always)]
    pub fn transpose(&self) -> Self {
        let m00 = self.m00().clone();
        let m01 = self.m01().clone();
        let m10 = self.m10().clone();
        let m11 = self.m11().clone();

        Self::new(m00, m10, m01, m11)
    }

    #[inline(always)]
    pub fn transpose_in_place(&mut self) -> &mut Self {
        self.0.swap(1, 2);
        self
    }
}

impl<T, I> Index<I> for Mat2<T>
where
    I: SliceIndex<[T], Output = T>,
{
    type Output = T;

    #[inline(always)]
    fn index(&self, index: I) -> &Self::Output {
        self.0.index(index)
    }
}

impl<T, I> IndexMut<I> for Mat2<T>
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
        impl Mat2<$t> {
            #[inline(always)]
            pub const fn new_zero() -> Self {
                Self([$zero, $zero, $zero, $zero])
            }

            #[inline(always)]
            pub const fn new_identity() -> Self {
                Self([$one, $zero, $zero, $one])
            }
        }
       )+
    };
}

macro_rules! neg {
    ($($t: ident),+) => {
       $(
        impl Neg for Mat2<$t> {
            type Output = Mat2<$t>;

            #[inline(always)]
            fn neg(self) -> Self::Output {
                Self([
                    -self.0[0],
                    -self.0[1],
                    -self.0[2],
                    -self.0[3],
                ])
            }
        }
       )+
    };
}

macro_rules! math {
    ($(($t: ident, $epsilon: expr, $zero: expr, $one: expr)),+) => {
       $(
        impl Add for Mat2<$t> {
            type Output = Mat2<$t>;

            #[inline(always)]
            fn add(self, rhs: Self) -> Self::Output {
                Self([
                    self.0[0] + rhs.0[0],
                    self.0[1] + rhs.0[1],
                    self.0[2] + rhs.0[2],
                    self.0[3] + rhs.0[3],
                ])
            }
        }

        impl Add<$t> for Mat2<$t> {
            type Output = Mat2<$t>;

            #[inline(always)]
            fn add(self, rhs: $t) -> Self::Output {
                Self([
                    self.0[0] + rhs,
                    self.0[1] + rhs,
                    self.0[2] + rhs,
                    self.0[3] + rhs,

                ])
            }
        }

        impl Add<Mat2<$t>> for $t {
            type Output = Mat2<$t>;

            #[inline(always)]
            fn add(self, rhs: Mat2<$t>) -> Self::Output {
                Mat2::<$t>([
                    self + rhs.0[0],
                    self + rhs.0[1],
                    self + rhs.0[2],
                    self + rhs.0[3],
                ])
            }
        }

        impl AddAssign for Mat2<$t> {
            #[inline(always)]
            fn add_assign(&mut self, rhs: Self) {
                self.0[0] += rhs.0[0];
                self.0[1] += rhs.0[1];
                self.0[2] += rhs.0[2];
                self.0[3] += rhs.0[3];
            }
        }

        impl AddAssign<$t> for Mat2<$t> {
            #[inline(always)]
            fn add_assign(&mut self, rhs: $t) {
                self.0[0] += rhs;
                self.0[1] += rhs;
                self.0[2] += rhs;
                self.0[3] += rhs;
            }
        }

        impl Sub for Mat2<$t> {
            type Output = Mat2<$t>;

            #[inline(always)]
            fn sub(self, rhs: Self) -> Self::Output {
                Self([
                    self.0[0] - rhs.0[0],
                    self.0[1] - rhs.0[1],
                    self.0[2] - rhs.0[2],
                    self.0[3] - rhs.0[3],
                ])
            }
        }

        impl Sub<$t> for Mat2<$t> {
            type Output = Mat2<$t>;

            #[inline(always)]
            fn sub(self, rhs: $t) -> Self::Output {
                Self([
                    self.0[0] - rhs,
                    self.0[1] - rhs,
                    self.0[2] - rhs,
                    self.0[3] - rhs,
                ])
            }
        }

        impl Sub<Mat2<$t>> for $t {
            type Output = Mat2<$t>;

            #[inline(always)]
            fn sub(self, rhs: Mat2<$t>) -> Self::Output {
                Mat2::<$t>([
                    self - rhs.0[0],
                    self - rhs.0[1],
                    self - rhs.0[2],
                    self - rhs.0[3],
                ])
            }
        }

        impl SubAssign for Mat2<$t> {
            #[inline(always)]
            fn sub_assign(&mut self, rhs: Self) {
                self.0[0] -= rhs.0[0];
                self.0[1] -= rhs.0[1];
                self.0[2] -= rhs.0[2];
                self.0[3] -= rhs.0[3];
            }
        }

        impl SubAssign<$t> for Mat2<$t> {
            #[inline(always)]
            fn sub_assign(&mut self, rhs: $t) {
                self.0[0] -= rhs;
                self.0[1] -= rhs;
                self.0[2] -= rhs;
                self.0[3] -= rhs;
            }
        }

        impl Mul for Mat2<$t> {
            type Output = Mat2<$t>;

            #[inline(always)]
            fn mul(self, rhs: Self) -> Self::Output {
                let a0 = self.0[0];
                let a1 = self.0[1];
                let a2 = self.0[2];
                let a3 = self.0[3];

                let b0 = rhs.m00();
                let b1 = rhs.m01();
                let b2 = rhs.m10();
                let b3 = rhs.m11();

                Self([
                    a0 * b0 + a2 * b1,
                    a1 * b0 + a3 * b1,
                    a0 * b2 + a2 * b3,
                    a1 * b2 + a3 * b3,
                ])
            }
        }

        impl Mul<$t> for Mat2<$t> {
            type Output = Mat2<$t>;

            #[inline(always)]
            fn mul(self, rhs: $t) -> Self::Output {
                Self([
                    self.0[0] * rhs,
                    self.0[1] * rhs,
                    self.0[2] * rhs,
                    self.0[3] * rhs,
                ])
            }
        }

        impl Mul<Mat2<$t>> for $t {
            type Output = Mat2<$t>;

            #[inline(always)]
            fn mul(self, rhs: Mat2<$t>) -> Self::Output {
                Mat2::<$t>([
                    self * rhs.0[0],
                    self * rhs.0[1],
                    self * rhs.0[2],
                    self * rhs.0[3],
                ])
            }
        }

        impl MulAssign for Mat2<$t> {
            #[inline(always)]
            fn mul_assign(&mut self, rhs: Self) {
                let a0 = self.0[0];
                let a1 = self.0[1];
                let a2 = self.0[2];
                let a3 = self.0[3];

                let b0 = rhs.0[0];
                let b1 = rhs.0[1];
                let b2 = rhs.0[2];
                let b3 = rhs.0[3];

                self.0[0] = a0 * b0 + a2 * b1;
                self.0[1] = a1 * b0 + a3 * b1;
                self.0[2] = a0 * b2 + a2 * b3;
                self.0[3] = a1 * b2 + a3 * b3;
            }
        }

        impl MulAssign<$t> for Mat2<$t> {
            #[inline(always)]
            fn mul_assign(&mut self, rhs: $t) {
                self.0[0] *= rhs;
                self.0[1] *= rhs;
                self.0[2] *= rhs;
                self.0[3] *= rhs;
            }
        }

        impl Mat2<$t> {
            #[inline(always)]
            pub fn set_zero(&mut self) -> &mut Self {
                self.0[0] = $zero;
                self.0[1] = $zero;
                self.0[2] = $zero;
                self.0[3] = $zero;
                self
            }

            #[inline(always)]
            pub fn set_identify(&mut self) -> &mut Self {
                self.0[0] = $one;
                self.0[1] = $zero;
                self.0[2] = $zero;
                self.0[3] = $one;
                self
            }

            #[inline(always)]
            pub fn copy(&mut self, b: &Self) -> &mut Self {
                self.0[0] = b.0[0];
                self.0[1] = b.0[1];
                self.0[2] = b.0[2];
                self.0[3] = b.0[3];
                self
            }

            #[inline(always)]
            pub fn determinant(&self) -> $t {
                let a0 = self.m00();
                let a1 = self.m01();
                let a2 = self.m10();
                let a3 = self.m11();
                a0 * a3 - a2 * a1
            }

            #[inline(always)]
            pub fn invert(&self) -> Result<Self, Error> {
                let a0 = self.0[0];
                let a1 = self.0[1];
                let a2 = self.0[2];
                let a3 = self.0[3];

                // Calculate the determinant
                let mut det = a0 * a3 - a2 * a1;
                if det == $zero {
                    return Err(Error::ZeroDeterminant);
                }
                det = $one / det;

                Ok(Self::new(a3 * det, -a1 * det, -a2 * det, a0 * det))
            }

            #[inline(always)]
            pub fn invert_in_place(&mut self) -> Result<&mut Self, Error> {
                let a0 = self.0[0];
                let a1 = self.0[1];
                let a2 = self.0[2];
                let a3 = self.0[3];

                // Calculate the determinant
                let mut det = a0 * a3 - a2 * a1;
                if det == $zero {
                    return Err(Error::ZeroDeterminant);
                }
                det = $one / det;

                self.0[0] = a3 * det;
                self.0[1] = -a1 * det;
                self.0[2] = -a2 * det;
                self.0[3] = a0 * det;
                Ok(self)
            }

            #[inline(always)]
            pub fn adjoint(&self) -> Self {
                let a0 = self.0[0];
                let a1 = self.0[1];
                let a2 = self.0[2];
                let a3 = self.0[3];

                Self::new(a3, -a1, -a2, a0)
            }

            #[inline(always)]
            pub fn adjoint_in_place(&mut self) -> &mut Self {
                let a0 = self.0[0];
                let a1 = self.0[1];
                let a2 = self.0[2];
                let a3 = self.0[3];

                self.0[0] = a3;
                self.0[1] = -a1;
                self.0[2] = -a2;
                self.0[3] = a0;
                self
            }

            #[inline(always)]
            pub fn scale(&self, v: &Vec2::<$t>) -> Self {
                let a0 = self.0[0];
                let a1 = self.0[1];
                let a2 = self.0[2];
                let a3 = self.0[3];

                let v0 = v.x();
                let v1 = v.y();

                Self::new(a0 * v0, a1 * v0, a2 * v1, a3 * v1)
            }

            #[inline(always)]
            pub fn scale_in_place(&mut self, v: &Vec2::<$t>) -> &mut Self {
                let a0 = self.0[0];
                let a1 = self.0[1];
                let a2 = self.0[2];
                let a3 = self.0[3];

                let v0 = v.x();
                let v1 = v.y();

                self.0[0] = a0 * v0;
                self.0[1] = a1 * v0;
                self.0[2] = a2 * v1;
                self.0[3] = a3 * v1;
                self
            }

            #[inline(always)]
            pub fn rotate(&self, rad: $t) -> Self {
                let a0 = self.0[0];
                let a1 = self.0[1];
                let a2 = self.0[2];
                let a3 = self.0[3];

                let s = rad.sin();
                let c = rad.cos();

                Self::new(
                    a0 * c + a2 * s,
                    a1 * c + a3 * s,
                    a0 * -s + a2 * c,
                    a1 * -s + a3 * c,
                )
            }

            #[inline(always)]
            pub fn rotate_in_place(&mut self, rad: $t) -> &mut Self {
                let a0 = self.0[0];
                let a1 = self.0[1];
                let a2 = self.0[2];
                let a3 = self.0[3];

                let s = rad.sin();
                let c = rad.cos();

                self.0[0] = a0 * c + a2 * s;
                self.0[1] = a1 * c + a3 * s;
                self.0[2] = a0 * -s + a2 * c;
                self.0[3] = a1 * -s + a3 * c;
                self
            }

            #[inline(always)]
            pub fn frob(&self) -> $t {
                let a0 = self.0[0];
                let a1 = self.0[1];
                let a2 = self.0[2];
                let a3 = self.0[3];

                (a0 * a0 + a1 * a1 + a2 * a2 + a3 * a3).sqrt()
            }

            #[inline(always)]
            pub fn ldu(&self, l: &Self, d: &Self, u: &Self) -> (Self, Self, Self) {
                let mut l = Self::new(l.0[0], l.0[1], l.0[2], l.0[3]);
                let d = Self::new(d.0[0], d.0[1], d.0[2], d.0[3]);
                let mut u = Self::new(u.0[0], u.0[1], u.0[2], u.0[3]);

                l.0[2] = self.0[2] / self.0[0];
                u.0[0] = self.0[0];
                u.0[1] = self.0[1];
                u.0[3] = self.0[3] - l.0[2] * u.0[1];
                (l, d, u)
            }
        }

        impl ApproximateEq for Mat2<$t> {
            #[inline(always)]
            fn approximate_eq(&self, other: &Self) -> bool {
                let a0 = self.m00();
                let a1 = self.m01();
                let a2 = self.m10();
                let a3 = self.m11();

                let b0 = other.m00();
                let b1 = other.m01();
                let b2 = other.m10();
                let b3 = other.m11();

                (a0 - b0).abs() <= $epsilon * $one.max(a0.abs()).max(b0.abs())
                    && (a1 - b1).abs() <= $epsilon * $one.max(a1.abs()).max(b1.abs())
                    && (a2 - b2).abs() <= $epsilon * $one.max(a2.abs()).max(b2.abs())
                    && (a3 - b3).abs() <= $epsilon * $one.max(a3.abs()).max(b3.abs())
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
neg!(i8, i16, i32, i64, i128, isize, f32, f64);
math! {
    (f32, super::EPSILON_F32, 0.0f32, 1.0f32),
    (f64, super::EPSILON_F64, 0.0f64, 1.0f64)
}

#[cfg(feature = "gl")]
impl super::GLF32<4> for Mat2<f32> {
    #[inline(always)]
    fn gl_f32(&self) -> [f32; 4] {
        self.0.clone()
    }
}

#[cfg(feature = "gl")]
impl super::GLF32<4> for Mat2<f64> {
    #[inline(always)]
    fn gl_f32(&self) -> [f32; 4] {
        [
            self.0[0] as f32,
            self.0[1] as f32,
            self.0[2] as f32,
            self.0[3] as f32,
        ]
    }
}

#[cfg(feature = "gl")]
impl super::GLF32Borrowed<4> for Mat2<f32> {
    #[inline(always)]
    fn gl_f32_borrowed(&self) -> &[f32; 4] {
        &self.0
    }
}

#[cfg(test)]
mod tests {
    use crate::{error::Error, vec2::Vec2, ApproximateEq};

    use super::Mat2;

    #[test]
    fn new() {
        assert_eq!(Mat2::new(2.0, 3.0, 4.0, 5.0).raw(), &[2.0, 3.0, 4.0, 5.0]);
    }

    #[test]
    fn new_zero() {
        assert_eq!(Mat2::<f64>::new_zero().raw(), &[0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn new_identity() {
        assert_eq!(Mat2::<f64>::new_identity().raw(), &[1.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn from_slice() {
        assert_eq!(
            Mat2::from_slice([1.0, 2.0, 3.0, 4.0]).raw(),
            &[1.0, 2.0, 3.0, 4.0]
        );
    }

    #[test]
    fn raw() {
        assert_eq!(Mat2::new(2.0, 3.0, 4.0, 5.0).raw(), &[2.0, 3.0, 4.0, 5.0]);
    }

    #[test]
    fn raw_mut() {
        let mut mat = Mat2::new(2.0, 3.0, 4.0, 5.0);
        (*mat.raw_mut()) = [1.0, 2.0, 3.0, 4.0];
        assert_eq!(mat.raw(), &[1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn m00() {
        assert_eq!(Mat2::<f64>::new(2.0, 3.0, 4.0, 5.0).m00(), &2.0);
    }

    #[test]
    fn m01() {
        assert_eq!(Mat2::<f64>::new(2.0, 3.0, 4.0, 5.0).m01(), &3.0);
    }

    #[test]
    fn m10() {
        assert_eq!(Mat2::<f64>::new(2.0, 3.0, 4.0, 5.0).m10(), &4.0);
    }

    #[test]
    fn m11() {
        assert_eq!(Mat2::<f64>::new(2.0, 3.0, 4.0, 5.0).m11(), &5.0);
    }

    #[test]
    fn m00_mut() {
        let mut mat = Mat2::new(2.0, 3.0, 4.0, 5.0);
        (*mat.m00_mut()) = 6.0;
        assert_eq!(mat.raw(), &[6.0, 3.0, 4.0, 5.0]);
    }

    #[test]
    fn m01_mut() {
        let mut mat = Mat2::new(2.0, 3.0, 4.0, 5.0);
        (*mat.m01_mut()) = 6.0;
        assert_eq!(mat.raw(), &[2.0, 6.0, 4.0, 5.0]);
    }

    #[test]
    fn m10_mut() {
        let mut mat = Mat2::new(2.0, 3.0, 4.0, 5.0);
        (*mat.m10_mut()) = 6.0;
        assert_eq!(mat.raw(), &[2.0, 3.0, 6.0, 5.0]);
    }

    #[test]
    fn m11_mut() {
        let mut mat = Mat2::new(2.0, 3.0, 4.0, 5.0);
        (*mat.m11_mut()) = 6.0;
        assert_eq!(mat.raw(), &[2.0, 3.0, 4.0, 6.0]);
    }

    #[test]
    fn set_m00() {
        let mut mat = Mat2::new(2.0, 3.0, 4.0, 5.0);
        mat.set_m00(6.0);
        assert_eq!(mat.raw(), &[6.0, 3.0, 4.0, 5.0]);
    }

    #[test]
    fn set_m01() {
        let mut mat = Mat2::new(2.0, 3.0, 4.0, 5.0);
        mat.set_m01(6.0);
        assert_eq!(mat.raw(), &[2.0, 6.0, 4.0, 5.0]);
    }

    #[test]
    fn set_m10() {
        let mut mat = Mat2::new(2.0, 3.0, 4.0, 5.0);
        mat.set_m10(6.0);
        assert_eq!(mat.raw(), &[2.0, 3.0, 6.0, 5.0]);
    }

    #[test]
    fn set_m11() {
        let mut mat = Mat2::new(2.0, 3.0, 4.0, 5.0);
        mat.set_m11(6.0);
        assert_eq!(mat.raw(), &[2.0, 3.0, 4.0, 6.0]);
    }

    #[test]
    fn set() {
        let mut mat = Mat2::new(2.0, 3.0, 4.0, 5.0);
        mat.set(22.0, 22.0, 22.0, 22.0);
        assert_eq!(mat.raw(), &[22.0, 22.0, 22.0, 22.0]);
    }

    #[test]
    fn set_zero() {
        let mut mat = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        mat.set_zero();
        assert_eq!(mat.raw(), &[0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn set_identify() {
        let mut mat = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.00);
        mat.set_identify();
        assert_eq!(mat.raw(), &[1.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn add_mat2_mat2() {
        let mat0 = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let mat1 = Mat2::<f64>::new(5.0, 6.0, 7.0, 8.0);
        assert_eq!(
            (mat0 + mat1).approximate_eq(&Mat2::new(6.0, 8.0, 10.0, 12.0)),
            true
        );
    }

    #[test]
    fn add_mat2_scalar() {
        let mat = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let scalar = 1.0;
        assert_eq!(
            (mat + scalar).approximate_eq(&Mat2::new(2.0, 3.0, 4.0, 5.0)),
            true
        );
    }

    #[test]
    fn add_scalar_mat2() {
        let scalar = 1.0;
        let mat = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(
            (scalar + mat).approximate_eq(&Mat2::new(2.0, 3.0, 4.0, 5.0)),
            true
        );
    }

    #[test]
    fn add_assign_mat2_mat2() {
        let mut mat0 = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let mat1 = Mat2::<f64>::new(5.0, 6.0, 7.0, 8.0);
        mat0 += mat1;
        assert_eq!(mat0.approximate_eq(&Mat2::new(6.0, 8.0, 10.0, 12.0)), true);
    }

    #[test]
    fn add_assign_mat2_scalar() {
        let mut mat = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let scalar = 1.0;
        mat += scalar;
        assert_eq!(mat.approximate_eq(&Mat2::new(2.0, 3.0, 4.0, 5.0)), true);
    }

    #[test]
    fn sub_mat2_mat2() {
        let mat0 = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let mat1 = Mat2::<f64>::new(5.0, 6.0, 7.0, 8.0);
        assert_eq!(
            (mat0 - mat1).approximate_eq(&Mat2::new(-4.0, -4.0, -4.0, -4.0)),
            true
        );
    }

    #[test]
    fn sub_mat2_scalar() {
        let mat = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let scalar = 1.0;
        assert_eq!(
            (mat - scalar).approximate_eq(&Mat2::new(0.0, 1.0, 2.0, 3.0)),
            true
        );
    }

    #[test]
    fn sub_scalar_mat2() {
        let scalar = 1.0;
        let mat = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(
            (scalar - mat).approximate_eq(&Mat2::new(0.0, -1.0, -2.0, -3.0)),
            true
        );
    }

    #[test]
    fn sub_assign_mat2_mat2() {
        let mut mat0 = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let mat1 = Mat2::<f64>::new(5.0, 6.0, 7.0, 8.0);
        mat0 -= mat1;
        assert_eq!(
            mat0.approximate_eq(&Mat2::new(-4.0, -4.0, -4.0, -4.0)),
            true
        );
    }

    #[test]
    fn sub_assign_mat2_scalar() {
        let mut mat = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let scalar = 1.0;
        mat -= scalar;
        assert_eq!(mat.approximate_eq(&Mat2::new(0.0, 1.0, 2.0, 3.0)), true);
    }

    #[test]
    fn mul_mat2_mat2() {
        let mat0 = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let mat1 = Mat2::<f64>::new(5.0, 6.0, 7.0, 8.0);
        assert_eq!(
            (mat0 * mat1).approximate_eq(&Mat2::new(23.0, 34.0, 31.0, 46.0)),
            true
        );
    }

    #[test]
    fn mul_mat2_scalar() {
        let mat = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let scalar = 2.0;
        assert_eq!(
            (mat * scalar).approximate_eq(&Mat2::new(2.0, 4.0, 6.0, 8.0)),
            true
        );
    }

    #[test]
    fn mul_scalar_mat2() {
        let scalar = 3.0;
        let mat = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(
            (scalar * mat).approximate_eq(&Mat2::new(3.0, 6.0, 9.0, 12.0)),
            true
        );
    }

    #[test]
    fn mul_assign_mat2_mat2() {
        let mut mat0 = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let mat1 = Mat2::<f64>::new(5.0, 6.0, 7.0, 8.0);
        mat0 *= mat1;
        assert_eq!(
            mat0.approximate_eq(&Mat2::new(23.0, 34.0, 31.0, 46.0)),
            true
        );
    }

    #[test]
    fn mul_assign_mat2_scalar() {
        let mut mat = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let scalar = 4.0;
        mat *= scalar;
        assert_eq!(mat.approximate_eq(&Mat2::new(4.0, 8.0, 12.0, 16.0)), true);
    }

    #[test]
    fn neg() {
        let mat = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(
            (-mat).approximate_eq(&Mat2::new(-1.0, -2.0, -3.0, -4.0)),
            true
        );
    }

    #[test]
    fn determinant() {
        let mat = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(mat.determinant().approximate_eq(&-2.0), true);
    }

    #[test]
    fn invert() -> Result<(), Error> {
        let mat = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(
            mat.invert()?
                .approximate_eq(&Mat2::new(-2.0, 1.0, 1.5, -0.5)),
            true
        );
        Ok(())
    }

    #[test]
    fn invert_in_place() -> Result<(), Error> {
        let mut mat = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        mat.invert_in_place()?;
        assert_eq!(mat.approximate_eq(&Mat2::new(-2.0, 1.0, 1.5, -0.5)), true);
        Ok(())
    }

    #[test]
    fn adjoint() {
        let mat = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(
            mat.adjoint()
                .approximate_eq(&Mat2::new(4.0, -2.0, -3.0, 1.0)),
            true
        );
    }

    #[test]
    fn adjoint_in_place() {
        let mut mat = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        mat.adjoint_in_place();
        assert_eq!(mat.approximate_eq(&Mat2::new(4.0, -2.0, -3.0, 1.0)), true);
    }

    #[test]
    fn scale() {
        let mat = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(
            mat.scale(&Vec2::new(2.0, 3.0))
                .approximate_eq(&Mat2::new(2.0, 4.0, 9.0, 12.0)),
            true
        );
    }

    #[test]
    fn scale_in_place() {
        let mut mat = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        mat.scale_in_place(&Vec2::new(2.0, 3.0));
        assert_eq!(mat.approximate_eq(&Mat2::new(2.0, 4.0, 9.0, 12.0)), true);
    }

    #[test]
    fn rotate() {
        let mat = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(
            mat.rotate(std::f64::consts::PI * 0.5)
                .approximate_eq(&Mat2::new(3.0, 4.0, -1.0, -2.0)),
            true
        );
    }

    #[test]
    fn rotate_in_place() {
        let mut mat = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        mat.rotate_in_place(std::f64::consts::PI * 0.5);
        assert_eq!(mat.approximate_eq(&Mat2::new(3.0, 4.0, -1.0, -2.0)), true);
    }

    #[test]
    fn frob() {
        let mat = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(mat.frob().approximate_eq(&5.477225575051661), true);
    }

    #[test]
    fn ldu() {
        let l = Mat2::<f64>::new_identity();
        let d = Mat2::<f64>::new_identity();
        let u = Mat2::<f64>::new_identity();
        let mat = Mat2::<f64>::new(4.0, 3.0, 6.0, 3.0);

        let (l, d, u) = mat.ldu(&l, &d, &u);

        assert_eq!(l.approximate_eq(&Mat2::new(1.0, 0.0, 1.5, 1.0)), true);
        assert_eq!(d.approximate_eq(&Mat2::new(1.0, 0.0, 0.0, 1.0)), true);
        assert_eq!(u.approximate_eq(&Mat2::new(4.0, 3.0, 0.0, -1.5)), true);
    }

    #[test]
    fn display() {
        let mat = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(mat.to_string(), "mat2(1, 2, 3, 4)");
    }
}
