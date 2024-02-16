use std::{
    fmt::{Debug, Display},
    ops::{Add, AddAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign},
    slice::SliceIndex,
};

use crate::{error::Error, vec2::Vec2, ApproximateEq};
pub struct Mat2d<T = f64>([T; 6]);

impl<T: Debug> Debug for Mat2d<T> {
    #[inline(always)]
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_tuple("Mat2d").field(&self.0).finish()
    }
}

impl<T: Copy> Copy for Mat2d<T> {}

impl<T: Clone> Clone for Mat2d<T> {
    #[inline(always)]
    fn clone(&self) -> Self {
        Self([
            self.0[0].clone(),
            self.0[1].clone(),
            self.0[2].clone(),
            self.0[3].clone(),
            self.0[4].clone(),
            self.0[5].clone(),
        ])
    }
}

impl<T: PartialEq> PartialEq for Mat2d<T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl<T: Display> Display for Mat2d<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_fmt(format_args!(
            "mat2d({}, {}, {}, {}, {}, {})",
            self.0[0], self.0[1], self.0[2], self.0[3], self.0[4], self.0[5]
        ))
    }
}

impl<T> Mat2d<T> {
    #[inline(always)]
    pub const fn new(a: T, b: T, c: T, d: T, tx: T, ty: T) -> Self {
        Self([a, b, c, d, tx, ty])
    }

    #[inline(always)]
    pub const fn from_slice(values: [T; 6]) -> Self {
        Self(values)
    }
}

impl<T> Mat2d<T> {
    #[inline(always)]
    pub fn raw(&self) -> &[T; 6] {
        &self.0
    }

    #[inline(always)]
    pub fn raw_mut(&mut self) -> &mut [T; 6] {
        &mut self.0
    }

    #[inline(always)]
    pub fn a(&self) -> &T {
        &self.0[0]
    }

    #[inline(always)]
    pub fn b(&self) -> &T {
        &self.0[1]
    }

    #[inline(always)]
    pub fn c(&self) -> &T {
        &self.0[2]
    }

    #[inline(always)]
    pub fn d(&self) -> &T {
        &self.0[3]
    }

    #[inline(always)]
    pub fn tx(&self) -> &T {
        &self.0[4]
    }

    #[inline(always)]
    pub fn ty(&self) -> &T {
        &self.0[5]
    }

    #[inline(always)]
    pub fn a_mut(&mut self) -> &mut T {
        &mut self.0[0]
    }

    #[inline(always)]
    pub fn b_mut(&mut self) -> &mut T {
        &mut self.0[1]
    }

    #[inline(always)]
    pub fn c_mut(&mut self) -> &mut T {
        &mut self.0[2]
    }

    #[inline(always)]
    pub fn d_mut(&mut self) -> &mut T {
        &mut self.0[3]
    }

    #[inline(always)]
    pub fn tx_mut(&mut self) -> &mut T {
        &mut self.0[4]
    }

    #[inline(always)]
    pub fn ty_mut(&mut self) -> &mut T {
        &mut self.0[5]
    }

    #[inline(always)]
    pub fn set_a(&mut self, a: T) {
        self.0[0] = a;
    }

    #[inline(always)]
    pub fn set_b(&mut self, b: T) {
        self.0[1] = b;
    }

    #[inline(always)]
    pub fn set_c(&mut self, c: T) {
        self.0[2] = c;
    }

    #[inline(always)]
    pub fn set_d(&mut self, d: T) {
        self.0[3] = d;
    }

    #[inline(always)]
    pub fn set_tx(&mut self, tx: T) {
        self.0[4] = tx;
    }

    #[inline(always)]
    pub fn set_ty(&mut self, ty: T) {
        self.0[5] = ty;
    }

    #[inline(always)]
    pub fn set(&mut self, a: T, b: T, c: T, d: T, tx: T, ty: T) {
        self.0[0] = a;
        self.0[1] = b;
        self.0[2] = c;
        self.0[3] = d;
        self.0[4] = tx;
        self.0[5] = ty;
    }
}

impl<T, I> Index<I> for Mat2d<T>
where
    I: SliceIndex<[T], Output = T>,
{
    type Output = T;

    #[inline(always)]
    fn index(&self, index: I) -> &Self::Output {
        self.0.index(index)
    }
}

impl<T, I> IndexMut<I> for Mat2d<T>
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
        impl Mat2d<$t> {
            #[inline(always)]
            pub const fn new_zero() -> Self {
                Self([$zero, $zero, $zero, $zero, $zero, $zero])
            }

            #[inline(always)]
            pub const fn new_identity() -> Self {
                Self([$one, $zero, $zero, $one, $zero, $zero])
            }
        }
       )+
    };
}

macro_rules! decimal_constructors {
    ($(($t: ident, $zero: expr, $one: expr)),+) => {
       $(
        impl Mat2d<$t> {
            #[inline(always)]
            pub fn from_scaling(v: &Vec2::<$t>) -> Self {
                Self([*v.x(), $zero, $zero, *v.y(), $zero, $zero])
            }

            #[inline(always)]
            pub fn from_translation(v: &Vec2::<$t>) -> Self {
                Self([$one, $zero, $zero, $one, *v.x(), *v.y()])
            }

            #[inline(always)]
            pub fn from_rotation(rad: $t) -> Self {
                let s = rad.sin();
                let c = rad.cos();
                Self([c, s, -s, c, $zero, $zero])
            }
        }
       )+
    };
}

macro_rules! neg {
    ($($t: ident),+) => {
       $(
        impl Neg for Mat2d<$t> {
            type Output = Mat2d<$t>;

            #[inline(always)]
            fn neg(self) -> Self::Output {
                Self([
                    -self.0[0],
                    -self.0[1],
                    -self.0[2],
                    -self.0[3],
                    -self.0[4],
                    -self.0[5],
                ])
            }
        }
       )+
    };
}

macro_rules! math {
    ($(($t: ident, $epsilon: expr, $zero: expr, $one: expr)),+) => {
       $(
        impl Add for Mat2d<$t> {
            type Output = Mat2d<$t>;

            #[inline(always)]
            fn add(self, rhs: Self) -> Self::Output {
                Self([
                    self.0[0] + rhs.0[0],
                    self.0[1] + rhs.0[1],
                    self.0[2] + rhs.0[2],
                    self.0[3] + rhs.0[3],
                    self.0[4] + rhs.0[4],
                    self.0[5] + rhs.0[5],
                ])
            }
        }

        impl Add<$t> for Mat2d<$t> {
            type Output = Mat2d<$t>;

            #[inline(always)]
            fn add(self, rhs: $t) -> Self::Output {
                Self([
                    self.0[0] + rhs,
                    self.0[1] + rhs,
                    self.0[2] + rhs,
                    self.0[3] + rhs,
                    self.0[4] + rhs,
                    self.0[5] + rhs,
                ])
            }
        }

        impl Add<Mat2d<$t>> for $t {
            type Output = Mat2d<$t>;

            #[inline(always)]
            fn add(self, rhs: Mat2d<$t>) -> Self::Output {
                Mat2d::<$t>([
                    self + rhs.0[0],
                    self + rhs.0[1],
                    self + rhs.0[2],
                    self + rhs.0[3],
                    self + rhs.0[4],
                    self + rhs.0[5],
                ])
            }
        }

        impl AddAssign for Mat2d<$t> {
            #[inline(always)]
            fn add_assign(&mut self, rhs: Self) {
                self.0[0] += rhs.0[0];
                self.0[1] += rhs.0[1];
                self.0[2] += rhs.0[2];
                self.0[3] += rhs.0[3];
                self.0[4] += rhs.0[4];
                self.0[5] += rhs.0[5];
            }
        }

        impl AddAssign<$t> for Mat2d<$t> {
            #[inline(always)]
            fn add_assign(&mut self, rhs: $t) {
                self.0[0] += rhs;
                self.0[1] += rhs;
                self.0[2] += rhs;
                self.0[3] += rhs;
                self.0[4] += rhs;
                self.0[5] += rhs;
            }
        }

        impl Sub for Mat2d<$t> {
            type Output = Mat2d<$t>;

            #[inline(always)]
            fn sub(self, rhs: Self) -> Self::Output {
                Self([
                    self.0[0] - rhs.0[0],
                    self.0[1] - rhs.0[1],
                    self.0[2] - rhs.0[2],
                    self.0[3] - rhs.0[3],
                    self.0[4] - rhs.0[4],
                    self.0[5] - rhs.0[5],
                ])
            }
        }

        impl Sub<$t> for Mat2d<$t> {
            type Output = Mat2d<$t>;

            #[inline(always)]
            fn sub(self, rhs: $t) -> Self::Output {
                Self([
                    self.0[0] - rhs,
                    self.0[1] - rhs,
                    self.0[2] - rhs,
                    self.0[3] - rhs,
                    self.0[4] - rhs,
                    self.0[5] - rhs,
                ])
            }
        }

        impl Sub<Mat2d<$t>> for $t {
            type Output = Mat2d<$t>;

            #[inline(always)]
            fn sub(self, rhs: Mat2d<$t>) -> Self::Output {
                Mat2d::<$t>([
                    self - rhs.0[0],
                    self - rhs.0[1],
                    self - rhs.0[2],
                    self - rhs.0[3],
                    self - rhs.0[4],
                    self - rhs.0[5],
                ])
            }
        }

        impl SubAssign for Mat2d<$t> {
            #[inline(always)]
            fn sub_assign(&mut self, rhs: Self) {
                self.0[0] -= rhs.0[0];
                self.0[1] -= rhs.0[1];
                self.0[2] -= rhs.0[2];
                self.0[3] -= rhs.0[3];
                self.0[4] -= rhs.0[4];
                self.0[5] -= rhs.0[5];
            }
        }

        impl SubAssign<$t> for Mat2d<$t> {
            #[inline(always)]
            fn sub_assign(&mut self, rhs: $t) {
                self.0[0] -= rhs;
                self.0[1] -= rhs;
                self.0[2] -= rhs;
                self.0[3] -= rhs;
                self.0[4] -= rhs;
                self.0[5] -= rhs;
            }
        }

        impl Mul for Mat2d<$t> {
            type Output = Mat2d<$t>;

            #[inline(always)]
            fn mul(self, rhs: Self) -> Self::Output {
                let a0 = self.0[0];
                let a1 = self.0[1];
                let a2 = self.0[2];
                let a3 = self.0[3];
                let a4 = self.0[4];
                let a5 = self.0[5];

                let b0 = rhs.0[0];
                let b1 = rhs.0[1];
                let b2 = rhs.0[2];
                let b3 = rhs.0[3];
                let b4 = rhs.0[4];
                let b5 = rhs.0[5];

                Self([
                    a0 * b0 + a2 * b1,
                    a1 * b0 + a3 * b1,
                    a0 * b2 + a2 * b3,
                    a1 * b2 + a3 * b3,
                    a0 * b4 + a2 * b5 + a4,
                    a1 * b4 + a3 * b5 + a5,
                ])
            }
        }

        impl Mul<$t> for Mat2d<$t> {
            type Output = Mat2d<$t>;

            #[inline(always)]
            fn mul(self, rhs: $t) -> Self::Output {
                Self([
                    self.0[0] * rhs,
                    self.0[1] * rhs,
                    self.0[2] * rhs,
                    self.0[3] * rhs,
                    self.0[4] * rhs,
                    self.0[5] * rhs,
                ])
            }
        }

        impl Mul<Mat2d<$t>> for $t {
            type Output = Mat2d<$t>;

            #[inline(always)]
            fn mul(self, rhs: Mat2d<$t>) -> Self::Output {
                Mat2d::<$t>([
                    self * rhs.0[0],
                    self * rhs.0[1],
                    self * rhs.0[2],
                    self * rhs.0[3],
                    self * rhs.0[4],
                    self * rhs.0[5],
                ])
            }
        }

        impl MulAssign for Mat2d<$t> {
            #[inline(always)]
            fn mul_assign(&mut self, rhs: Self) {
                let a0 = self.0[0];
                let a1 = self.0[1];
                let a2 = self.0[2];
                let a3 = self.0[3];
                let a4 = self.0[4];
                let a5 = self.0[5];

                let b0 = rhs.0[0];
                let b1 = rhs.0[1];
                let b2 = rhs.0[2];
                let b3 = rhs.0[3];
                let b4 = rhs.0[4];
                let b5 = rhs.0[5];

                self.0[0] = a0 * b0 + a2 * b1;
                self.0[1] = a1 * b0 + a3 * b1;
                self.0[2] = a0 * b2 + a2 * b3;
                self.0[3] = a1 * b2 + a3 * b3;
                self.0[4] = a0 * b4 + a2 * b5 + a4;
                self.0[5] = a1 * b4 + a3 * b5 + a5;
            }
        }

        impl MulAssign<$t> for Mat2d<$t> {
            #[inline(always)]
            fn mul_assign(&mut self, rhs: $t) {
                self.0[0] *= rhs;
                self.0[1] *= rhs;
                self.0[2] *= rhs;
                self.0[3] *= rhs;
                self.0[4] *= rhs;
                self.0[5] *= rhs;
            }
        }

        impl Mat2d<$t> {
            #[inline(always)]
            pub fn set_zero(&mut self) -> &mut Self {
                self.0[0] = $zero;
                self.0[1] = $zero;
                self.0[2] = $zero;
                self.0[3] = $zero;
                self.0[4] = $zero;
                self.0[5] = $zero;
                self
            }

            #[inline(always)]
            pub fn set_identify(&mut self) -> &mut Self {
                self.0[0] = $one;
                self.0[1] = $zero;
                self.0[2] = $zero;
                self.0[3] = $one;
                self.0[4] = $zero;
                self.0[5] = $zero;
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
                self
            }
            #[inline(always)]
            pub fn determinant(&self) -> $t {
                let a = self.a();
                let b = self.b();
                let c = self.c();
                let d = self.d();
                a * d - b * c
            }

            #[inline(always)]
            pub fn invert(&self) -> Result<Self, Error> {
                let aa = self.0[0];
                let ab = self.0[1];
                let ac = self.0[2];
                let ad = self.0[3];
                let atx = self.0[4];
                let aty = self.0[5];

                // Calculate the determinant
                let mut det = aa * ad - ab * ac;

                if det == $zero {
                    return Err(Error::ZeroDeterminant);
                }
                det = $one / det;

                Ok(Self::new(
                    ad * det,
                    -ab * det,
                    -ac * det,
                    aa * det,
                    (ac * aty - ad * atx) * det,
                    (ab * atx - aa * aty) * det,
                ))
            }

            #[inline(always)]
            pub fn invert_in_place(&mut self) -> Result<&mut Self, Error> {
                let aa = self.0[0];
                let ab = self.0[1];
                let ac = self.0[2];
                let ad = self.0[3];
                let atx = self.0[4];
                let aty = self.0[5];

                // Calculate the determinant
                let mut det = aa * ad - ab * ac;

                if det == $zero {
                    return Err(Error::ZeroDeterminant);
                }
                det = $one / det;

                self.0[0] = ad * det;
                self.0[1] = -ab * det;
                self.0[2] = -ac * det;
                self.0[3] = aa * det;
                self.0[4] = (ac * aty - ad * atx) * det;
                self.0[5] = (ab * atx - aa * aty) * det;
                Ok(self)
            }

            #[inline(always)]
            pub fn scale(&self, v: &Vec2::<$t>) -> Self {
                let a0 = self.0[0];
                let a1 = self.0[1];
                let a2 = self.0[2];
                let a3 = self.0[3];
                let a4 = self.0[4];
                let a5 = self.0[5];
                let v0 = *v.x();
                let v1 = *v.y();

                Self::new(a0 * v0, a1 * v0, a2 * v1, a3 * v1, a4, a5)
            }

            #[inline(always)]
            pub fn scale_in_place(&mut self, v: &Vec2::<$t>) -> &mut Self {
                let a0 = self.0[0];
                let a1 = self.0[1];
                let a2 = self.0[2];
                let a3 = self.0[3];
                let a4 = self.0[4];
                let a5 = self.0[5];
                let v0 = *v.x();
                let v1 = *v.y();

                self.0[0] = a0 * v0;
                self.0[1] = a1 * v0;
                self.0[2] = a2 * v1;
                self.0[3] = a3 * v1;
                self.0[4] = a4;
                self.0[5] = a5;
                self
            }

            #[inline(always)]
            pub fn rotate(&self, rad: $t) -> Self {
                let a0 = self.0[0];
                let a1 = self.0[1];
                let a2 = self.0[2];
                let a3 = self.0[3];
                let a4 = self.0[4];
                let a5 = self.0[5];

                let s = rad.sin();
                let c = rad.cos();

                Self::new(
                    a0 * c + a2 * s,
                    a1 * c + a3 * s,
                    a0 * -s + a2 * c,
                    a1 * -s + a3 * c,
                    a4,
                    a5,
                )
            }

            #[inline(always)]
            pub fn rotate_in_place(&mut self, rad: $t) -> &mut Self {
                let a0 = self.0[0];
                let a1 = self.0[1];
                let a2 = self.0[2];
                let a3 = self.0[3];
                let a4 = self.0[4];
                let a5 = self.0[5];

                let s = rad.sin();
                let c = rad.cos();

                self.0[0] = a0 * c + a2 * s;
                self.0[1] = a1 * c + a3 * s;
                self.0[2] = a0 * -s + a2 * c;
                self.0[3] = a1 * -s + a3 * c;
                self.0[4] = a4;
                self.0[5] = a5;
                self
            }

            #[inline(always)]
            pub fn translate(&self, v: &Vec2::<$t>) -> Self {
                let a0 = self.0[0];
                let a1 = self.0[1];
                let a2 = self.0[2];
                let a3 = self.0[3];
                let a4 = self.0[4];
                let a5 = self.0[5];
                let v0 = v.x();
                let v1 = v.y();

                Self::new(
                    a0,
                    a1,
                    a2,
                    a3,
                    a0 * v0 + a2 * v1 + a4,
                    a1 * v0 + a3 * v1 + a5,
                )
            }

            #[inline(always)]
            pub fn translate_in_place(&mut self, v: &Vec2::<$t>) -> &mut Self {
                let a0 = self.0[0];
                let a1 = self.0[1];
                let a2 = self.0[2];
                let a3 = self.0[3];
                let a4 = self.0[4];
                let a5 = self.0[5];
                let v0 = v.x();
                let v1 = v.y();

                self.0[0] = a0;
                self.0[1] = a1;
                self.0[2] = a2;
                self.0[3] = a3;
                self.0[4] = a0 * v0 + a2 * v1 + a4;
                self.0[5] = a1 * v0 + a3 * v1 + a5;
                self
            }

            #[inline(always)]
            pub fn frob(&self) -> $t {
                let a = self.0[0];
                let b = self.0[1];
                let c = self.0[2];
                let d = self.0[3];
                let tx = self.0[4];
                let ty = self.0[5];

                (a * a + b * b + c * c + d * d + tx * tx + ty * ty + 1.0).sqrt()
            }
        }

        impl ApproximateEq for Mat2d<$t> {
            #[inline(always)]
            fn approximate_eq(&self, other: &Self) -> bool {
                let a0 = self.0[0];
                let a1 = self.0[1];
                let a2 = self.0[2];
                let a3 = self.0[3];
                let a4 = self.0[4];
                let a5 = self.0[5];

                let b0 = other.0[0];
                let b1 = other.0[1];
                let b2 = other.0[2];
                let b3 = other.0[3];
                let b4 = other.0[4];
                let b5 = other.0[5];

                (a0 - b0).abs() <= $epsilon * $one.max(a0.abs()).max(b0.abs())
                    && (a1 - b1).abs() <= $epsilon * $one.max(a1.abs()).max(b1.abs())
                    && (a2 - b2).abs() <= $epsilon * $one.max(a2.abs()).max(b2.abs())
                    && (a3 - b3).abs() <= $epsilon * $one.max(a3.abs()).max(b3.abs())
                    && (a4 - b4).abs() <= $epsilon * $one.max(a4.abs()).max(b4.abs())
                    && (a5 - b5).abs() <= $epsilon * $one.max(a5.abs()).max(b5.abs())
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
    (f32, 0.0f32, 1.0f32),
    (f64, 0.0f64, 1.0f64)
}
neg!(i8, i16, i32, i64, i128, isize, f32, f64);
math! {
    (f32, super::EPSILON_F32, 0.0f32, 1.0f32),
    (f64, super::EPSILON_F64, 0.0f64, 1.0f64)
}

#[cfg(feature = "gl")]
impl super::GLF32<6> for Mat2d<f32> {
    #[inline(always)]
    fn gl_f32(&self) -> [f32; 6] {
        self.0.clone()
    }
}

#[cfg(feature = "gl")]
impl super::GLF32<6> for Mat2d<f64> {
    #[inline(always)]
    fn gl_f32(&self) -> [f32; 6] {
        [
            self.0[0] as f32,
            self.0[1] as f32,
            self.0[2] as f32,
            self.0[3] as f32,
            self.0[4] as f32,
            self.0[5] as f32,
        ]
    }
}

#[cfg(feature = "gl")]
impl super::GLF32Borrowed<6> for Mat2d<f32> {
    #[inline(always)]
    fn gl_f32_borrowed(&self) -> &[f32; 6] {
        &self.0
    }
}

#[cfg(test)]
mod tests {
    use crate::{error::Error, vec2::Vec2, ApproximateEq};

    use super::Mat2d;

    #[test]
    fn new() {
        assert_eq!(
            Mat2d::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0).raw(),
            &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        );
    }

    #[test]
    fn new_zero() {
        assert_eq!(
            Mat2d::<f64>::new_zero().raw(),
            &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        );
    }

    #[test]
    fn new_identity() {
        assert_eq!(
            Mat2d::<f64>::new_identity().raw(),
            &[1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        );
    }

    #[test]
    fn from_slice() {
        assert_eq!(
            Mat2d::from_slice([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]).raw(),
            &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        );
    }

    #[test]
    fn raw() {
        assert_eq!(
            Mat2d::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0).raw(),
            &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        );
    }

    #[test]
    fn raw_mut() {
        let mut mat = Mat2d::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        (*mat.raw_mut()) = [11.0, 12.0, 13.0, 14.0, 15.0, 16.0];
        assert_eq!(mat.raw(), &[11.0, 12.0, 13.0, 14.0, 15.0, 16.0]);
    }

    #[test]
    fn a() {
        assert_eq!(Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0).a(), &1.0);
    }

    #[test]
    fn b() {
        assert_eq!(Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0).b(), &2.0);
    }

    #[test]
    fn c() {
        assert_eq!(Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0).c(), &3.0);
    }

    #[test]
    fn d() {
        assert_eq!(Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0).d(), &4.0);
    }

    #[test]
    fn tx() {
        assert_eq!(Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0).tx(), &5.0);
    }

    #[test]
    fn ty() {
        assert_eq!(Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0).ty(), &6.0);
    }

    #[test]
    fn a_mut() {
        let mut mat = Mat2d::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        (*mat.a_mut()) = 10.0;
        assert_eq!(mat.raw(), &[10.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn b_mut() {
        let mut mat = Mat2d::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        (*mat.b_mut()) = 10.0;
        assert_eq!(mat.raw(), &[1.0, 10.0, 3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn c_mut() {
        let mut mat = Mat2d::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        (*mat.c_mut()) = 10.0;
        assert_eq!(mat.raw(), &[1.0, 2.0, 10.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn d_mut() {
        let mut mat = Mat2d::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        (*mat.d_mut()) = 10.0;
        assert_eq!(mat.raw(), &[1.0, 2.0, 3.0, 10.0, 5.0, 6.0]);
    }

    #[test]
    fn tx_mut() {
        let mut mat = Mat2d::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        (*mat.tx_mut()) = 10.0;
        assert_eq!(mat.raw(), &[1.0, 2.0, 3.0, 4.0, 10.0, 6.0]);
    }

    #[test]
    fn ty_mut() {
        let mut mat = Mat2d::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        (*mat.ty_mut()) = 10.0;
        assert_eq!(mat.raw(), &[1.0, 2.0, 3.0, 4.0, 5.0, 10.0]);
    }

    #[test]
    fn set_a() {
        let mut mat = Mat2d::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        mat.set_a(10.0);
        assert_eq!(mat.raw(), &[10.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn set_b() {
        let mut mat = Mat2d::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        mat.set_b(10.0);
        assert_eq!(mat.raw(), &[1.0, 10.0, 3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn set_c() {
        let mut mat = Mat2d::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        mat.set_c(10.0);
        assert_eq!(mat.raw(), &[1.0, 2.0, 10.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn set_d() {
        let mut mat = Mat2d::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        mat.set_d(10.0);
        assert_eq!(mat.raw(), &[1.0, 2.0, 3.0, 10.0, 5.0, 6.0]);
    }

    #[test]
    fn set_tx() {
        let mut mat = Mat2d::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        mat.set_tx(10.0);
        assert_eq!(mat.raw(), &[1.0, 2.0, 3.0, 4.0, 10.0, 6.0]);
    }

    #[test]
    fn set_ty() {
        let mut mat = Mat2d::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        mat.set_ty(10.0);
        assert_eq!(mat.raw(), &[1.0, 2.0, 3.0, 4.0, 5.0, 10.0]);
    }

    #[test]
    fn set() {
        let mut mat = Mat2d::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        mat.set(10.0, 10.0, 10.0, 10.0, 10.0, 10.0);
        assert_eq!(mat.raw(), &[10.0, 10.0, 10.0, 10.0, 10.0, 10.0]);
    }

    #[test]
    fn set_zero() {
        let mut mat = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        mat.set_zero();
        assert_eq!(mat.raw(), &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn set_identify() {
        let mut mat = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        mat.set_identify();
        assert_eq!(mat.raw(), &[1.0, 0.0, 0.0, 1.0, 0.0, 0.0]);
    }

    #[test]
    fn add_mat2d_mat2d() {
        let mat0 = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let mat1 = Mat2d::<f64>::new(7.0, 8.0, 9.0, 10.0, 11.0, 12.0);
        assert_eq!(
            (mat0 + mat1).approximate_eq(&Mat2d::new(8.0, 10.0, 12.0, 14.0, 16.0, 18.0)),
            true
        );
    }

    #[test]
    fn add_mat2d_scalar() {
        let mat = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let scalar = 1.0;
        assert_eq!(
            (mat + scalar).approximate_eq(&Mat2d::new(2.0, 3.0, 4.0, 5.0, 6.0, 7.0)),
            true
        );
    }

    #[test]
    fn add_scalar_mat2d() {
        let scalar = 1.0;
        let mat = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        assert_eq!(
            (scalar + mat).approximate_eq(&Mat2d::new(2.0, 3.0, 4.0, 5.0, 6.0, 7.0)),
            true
        );
    }

    #[test]
    fn add_assign_mat2d_mat2d() {
        let mut mat0 = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let mat1 = Mat2d::<f64>::new(7.0, 8.0, 9.0, 10.0, 11.0, 12.0);
        mat0 += mat1;
        assert_eq!(
            mat0.approximate_eq(&Mat2d::new(8.0, 10.0, 12.0, 14.0, 16.0, 18.0)),
            true
        );
    }

    #[test]
    fn add_assign_mat2d_scalar() {
        let mut mat = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let scalar = 1.0;
        mat += scalar;
        assert_eq!(
            mat.approximate_eq(&Mat2d::new(2.0, 3.0, 4.0, 5.0, 6.0, 7.0)),
            true
        );
    }

    #[test]
    fn sub_mat2d_mat2d() {
        let mat0 = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let mat1 = Mat2d::<f64>::new(7.0, 8.0, 9.0, 10.0, 11.0, 12.0);
        assert_eq!(
            (mat0 - mat1).approximate_eq(&Mat2d::new(-6.0, -6.0, -6.0, -6.0, -6.0, -6.0)),
            true
        );
    }

    #[test]
    fn sub_mat2d_scalar() {
        let mat = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let scalar = 1.0;
        assert_eq!(
            (mat - scalar).approximate_eq(&Mat2d::new(0.0, 1.0, 2.0, 3.0, 4.0, 5.0)),
            true
        );
    }

    #[test]
    fn sub_scalar_mat2d() {
        let scalar = 1.0;
        let mat = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        assert_eq!(
            (scalar - mat).approximate_eq(&Mat2d::new(0.0, -1.0, -2.0, -3.0, -4.0, -5.0)),
            true
        );
    }

    #[test]
    fn sub_assign_mat2d_mat2d() {
        let mut mat0 = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let mat1 = Mat2d::<f64>::new(7.0, 8.0, 9.0, 10.0, 11.0, 12.0);
        mat0 -= mat1;
        assert_eq!(
            mat0.approximate_eq(&Mat2d::new(-6.0, -6.0, -6.0, -6.0, -6.0, -6.0)),
            true
        );
    }

    #[test]
    fn sub_assign_mat2d_scalar() {
        let mut mat = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let scalar = 1.0;
        mat -= scalar;
        assert_eq!(
            mat.approximate_eq(&Mat2d::new(0.0, 1.0, 2.0, 3.0, 4.0, 5.0)),
            true
        );
    }

    #[test]
    fn mul_mat2d_mat2d() {
        let mat0 = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let mat1 = Mat2d::<f64>::new(7.0, 8.0, 9.0, 10.0, 11.0, 12.0);
        assert_eq!(
            (mat0 * mat1).approximate_eq(&Mat2d::new(31.0, 46.0, 39.0, 58.0, 52.0, 76.0)),
            true
        );
    }

    #[test]
    fn mul_mat2d_scalar() {
        let mat = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let scalar = 2.0;
        assert_eq!(
            (mat * scalar).approximate_eq(&Mat2d::new(2.0, 4.0, 6.0, 8.0, 10.0, 12.0)),
            true
        );
    }

    #[test]
    fn mul_scalar_mat2d() {
        let scalar = 3.0;
        let mat = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        assert_eq!(
            (scalar * mat).approximate_eq(&Mat2d::new(3.0, 6.0, 9.0, 12.0, 15.0, 18.0)),
            true
        );
    }

    #[test]
    fn mul_assign_mat2d_mat2d() {
        let mut mat0 = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let mat1 = Mat2d::<f64>::new(7.0, 8.0, 9.0, 10.0, 11.0, 12.0);
        mat0 *= mat1;
        assert_eq!(
            mat0.approximate_eq(&Mat2d::new(31.0, 46.0, 39.0, 58.0, 52.0, 76.0)),
            true
        );
    }

    #[test]
    fn mul_assign_mat2d_scalar() {
        let mut mat = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let scalar = 4.0;
        mat *= scalar;
        assert_eq!(
            mat.approximate_eq(&Mat2d::new(4.0, 8.0, 12.0, 16.0, 20.0, 24.0)),
            true
        );
    }

    #[test]
    fn neg() {
        let mat = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        assert_eq!(
            (-mat).approximate_eq(&Mat2d::new(-1.0, -2.0, -3.0, -4.0, -5.0, -6.0)),
            true
        );
    }

    #[test]
    fn determinant() {
        let mat = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        assert_eq!(mat.determinant().approximate_eq(&-2.0), true);
    }

    #[test]
    fn invert() -> Result<(), Error> {
        let mat = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        assert_eq!(
            mat.invert()?
                .approximate_eq(&Mat2d::new(-2.0, 1.0, 1.5, -0.5, 1.0, -2.0)),
            true
        );
        Ok(())
    }

    #[test]
    fn invert_in_place() -> Result<(), Error> {
        let mut mat = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        mat.invert_in_place()?;
        assert_eq!(
            mat.approximate_eq(&Mat2d::new(-2.0, 1.0, 1.5, -0.5, 1.0, -2.0)),
            true
        );
        Ok(())
    }

    #[test]
    fn translate() {
        let mat = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        assert_eq!(
            mat.translate(&Vec2::new(2.0, 3.0))
                .approximate_eq(&Mat2d::new(1.0, 2.0, 3.0, 4.0, 16.0, 22.0)),
            true
        );
    }

    #[test]
    fn translate_in_place() {
        let mut mat = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        mat.translate_in_place(&Vec2::new(2.0, 3.0));
        assert_eq!(
            mat.approximate_eq(&Mat2d::new(1.0, 2.0, 3.0, 4.0, 16.0, 22.0)),
            true
        );
    }

    #[test]
    fn rotate() {
        let mat = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        println!("{mat}");
        assert_eq!(
            mat.rotate(std::f64::consts::PI * 0.5)
                .approximate_eq(&Mat2d::new(3.0, 4.0, -1.0, -2.0, 5.0, 6.0)),
            true
        );
    }

    #[test]
    fn rotate_in_place() {
        let mut mat = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        mat.rotate_in_place(std::f64::consts::PI * 0.5);
        assert_eq!(
            mat.approximate_eq(&Mat2d::new(3.0, 4.0, -1.0, -2.0, 5.0, 6.0)),
            true
        );
    }

    #[test]
    fn frob() {
        let mat = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        assert_eq!(mat.frob().approximate_eq(&9.591663046625438), true);
    }

    #[test]
    fn display() {
        let mat = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        assert_eq!(mat.to_string(), "mat2d(1, 2, 3, 4, 5, 6)");
    }
}
