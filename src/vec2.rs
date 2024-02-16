use std::{
    fmt::{Debug, Display},
    ops::{Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign},
    slice::SliceIndex,
};

use crate::{mat2::Mat2, mat2d::Mat2d, mat3::Mat3, mat4::Mat4, vec3::Vec3, ApproximateEq};

pub struct Vec2<T = f64>([T; 2]);

impl<T: Debug> Debug for Vec2<T> {
    #[inline(always)]
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_tuple("Vec2").field(&self.0).finish()
    }
}

impl<T: Copy> Copy for Vec2<T> {}

impl<T: Clone> Clone for Vec2<T> {
    #[inline(always)]
    fn clone(&self) -> Self {
        Self([self.0[0].clone(), self.0[1].clone()])
    }
}

impl<T: PartialEq> PartialEq for Vec2<T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl<T: Display> Display for Vec2<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_fmt(format_args!("vec2({}, {})", self.0[0], self.0[1]))
    }
}

impl<T> Vec2<T> {
    #[inline(always)]
    pub const fn new(x: T, y: T) -> Self {
        Self([x, y])
    }

    #[inline(always)]
    pub const fn from_slice(values: [T; 2]) -> Self {
        Self(values)
    }
}

impl<T> Vec2<T> {
    #[inline(always)]
    pub fn raw(&self) -> &[T; 2] {
        &self.0
    }

    #[inline(always)]
    pub fn raw_mut(&mut self) -> &mut [T; 2] {
        &mut self.0
    }

    #[inline(always)]
    pub fn x(&self) -> &T {
        &self.0[0]
    }

    #[inline(always)]
    pub fn y(&self) -> &T {
        &self.0[1]
    }

    #[inline(always)]
    pub fn r(&self) -> &T {
        &self.0[0]
    }

    #[inline(always)]
    pub fn g(&self) -> &T {
        &self.0[1]
    }

    #[inline(always)]
    pub fn s(&self) -> &T {
        &self.0[0]
    }

    #[inline(always)]
    pub fn t(&self) -> &T {
        &self.0[1]
    }

    #[inline(always)]
    pub fn x_mut(&mut self) -> &mut T {
        &mut self.0[0]
    }

    #[inline(always)]
    pub fn y_mut(&mut self) -> &mut T {
        &mut self.0[1]
    }

    #[inline(always)]
    pub fn r_mut(&mut self) -> &mut T {
        &mut self.0[0]
    }

    #[inline(always)]
    pub fn g_mut(&mut self) -> &mut T {
        &mut self.0[1]
    }

    #[inline(always)]
    pub fn s_mut(&mut self) -> &mut T {
        &mut self.0[0]
    }

    #[inline(always)]
    pub fn t_mut(&mut self) -> &mut T {
        &mut self.0[1]
    }

    #[inline(always)]
    pub fn set_x(&mut self, x: T) {
        self.0[0] = x;
    }

    #[inline(always)]
    pub fn set_y(&mut self, y: T) {
        self.0[1] = y;
    }

    #[inline(always)]
    pub fn set_r(&mut self, r: T) {
        self.0[0] = r;
    }

    #[inline(always)]
    pub fn set_g(&mut self, g: T) {
        self.0[1] = g;
    }

    #[inline(always)]
    pub fn set_s(&mut self, s: T) {
        self.0[0] = s;
    }

    #[inline(always)]
    pub fn set_t(&mut self, t: T) {
        self.0[1] = t;
    }

    #[inline(always)]
    pub fn set(&mut self, x: T, y: T) {
        self.0[0] = x;
        self.0[1] = y;
    }
}

impl<T, I> Index<I> for Vec2<T>
where
    I: SliceIndex<[T], Output = T>,
{
    type Output = T;

    #[inline(always)]
    fn index(&self, index: I) -> &Self::Output {
        self.0.index(index)
    }
}

impl<T, I> IndexMut<I> for Vec2<T>
where
    I: SliceIndex<[T], Output = T>,
{
    #[inline(always)]
    fn index_mut(&mut self, index: I) -> &mut Self::Output {
        self.0.index_mut(index)
    }
}

macro_rules! basic_constructors {
    ($(($t: ident, $zero: expr)),+) => {
       $(
        impl Vec2<$t> {
            #[inline(always)]
            pub const fn new_zero() -> Self {
                Self([$zero, $zero])
            }
        }

        #[cfg(feature = "rand")]
        impl rand::distributions::Distribution<Vec2<$t>> for rand::distributions::Standard {
            fn sample<R: rand::prelude::Rng + ?Sized>(&self, rng: &mut R) -> Vec2<$t> {
                Vec2::new(
                    rng.gen::<$t>(),
                    rng.gen::<$t>(),
                )
            }
        }
       )+
    };
}

macro_rules! decimal_constructors {
    ($(($t: ident, $pi: expr)),+) => {
       $(
        impl Vec2<$t> {
            #[inline(always)]
            pub fn random() -> Self {
                let r = rand::random::<$t>() * 2.0 * $pi;
                Self([r.cos(), r.sin()])
            }

            #[inline(always)]
            pub fn random_with_scale(scale: $t) -> Self {
                let r = rand::random::<$t>() * 2.0 * $pi;
                Self([r.cos() * scale, r.sin() * scale])
            }
        }
       )+
    };
}

macro_rules! neg {
    ($($t: ident),+) => {
       $(
        impl Neg for Vec2<$t> {
            type Output = Vec2<$t>;

            #[inline(always)]
            fn neg(self) -> Self::Output {
                Self([
                    -self.0[0],
                    -self.0[1],
                ])
            }
        }
       )+
    };
}

macro_rules! math {
    ($(($t: ident, $epsilon: expr, $zero: expr, $one: expr, $pi: expr)),+) => {
       $(
        impl Add for Vec2<$t> {
            type Output = Vec2<$t>;

            #[inline(always)]
            fn add(self, rhs: Self) -> Self::Output {
                Self([
                    self.0[0] + rhs.0[0],
                    self.0[1] + rhs.0[1],
                ])
            }
        }

        impl Add<$t> for Vec2<$t> {
            type Output = Vec2<$t>;

            #[inline(always)]
            fn add(self, rhs: $t) -> Self::Output {
                Self([
                    self.0[0] + rhs,
                    self.0[1] + rhs,
                ])
            }
        }

        impl Add<Vec2<$t>> for $t {
            type Output = Vec2<$t>;

            #[inline(always)]
            fn add(self, rhs: Vec2<$t>) -> Self::Output {
                Vec2::<$t>([
                    self + rhs.0[0],
                    self + rhs.0[1],
                ])
            }
        }

        impl AddAssign for Vec2<$t> {
            #[inline(always)]
            fn add_assign(&mut self, rhs: Self) {
                self.0[0] += rhs.0[0];
                self.0[1] += rhs.0[1];
            }
        }

        impl AddAssign<$t> for Vec2<$t> {
            #[inline(always)]
            fn add_assign(&mut self, rhs: $t) {
                self.0[0] += rhs;
                self.0[1] += rhs;
            }
        }

        impl Sub for Vec2<$t> {
            type Output = Vec2<$t>;

            #[inline(always)]
            fn sub(self, rhs: Self) -> Self::Output {
                Self([
                    self.0[0] - rhs.0[0],
                    self.0[1] - rhs.0[1],
                ])
            }
        }

        impl Sub<$t> for Vec2<$t> {
            type Output = Vec2<$t>;

            #[inline(always)]
            fn sub(self, rhs: $t) -> Self::Output {
                Self([
                    self.0[0] - rhs,
                    self.0[1] - rhs,
                ])
            }
        }

        impl Sub<Vec2<$t>> for $t {
            type Output = Vec2<$t>;

            #[inline(always)]
            fn sub(self, rhs: Vec2<$t>) -> Self::Output {
                Vec2::<$t>([
                    self - rhs.0[0],
                    self - rhs.0[1],
                ])
            }
        }

        impl SubAssign for Vec2<$t> {
            #[inline(always)]
            fn sub_assign(&mut self, rhs: Self) {
                self.0[0] -= rhs.0[0];
                self.0[1] -= rhs.0[1];
            }
        }

        impl SubAssign<$t> for Vec2<$t> {
            #[inline(always)]
            fn sub_assign(&mut self, rhs: $t) {
                self.0[0] -= rhs;
                self.0[1] -= rhs;
            }
        }

        impl Mul for Vec2<$t> {
            type Output = Vec2<$t>;

            #[inline(always)]
            fn mul(self, rhs: Self) -> Self::Output {
                Self([
                    self.0[0] * rhs.0[0],
                    self.0[1] * rhs.0[1],
                ])
            }
        }

        impl Mul<$t> for Vec2<$t> {
            type Output = Vec2<$t>;

            #[inline(always)]
            fn mul(self, rhs: $t) -> Self::Output {
                Self([
                    self.0[0] * rhs,
                    self.0[1] * rhs,
                ])
            }
        }

        impl Mul<Vec2<$t>> for $t {
            type Output = Vec2<$t>;

            #[inline(always)]
            fn mul(self, rhs: Vec2<$t>) -> Self::Output {
                Vec2::<$t>([
                    self * rhs.0[0],
                    self * rhs.0[1],
                ])
            }
        }

        impl Mul<Vec2<$t>> for Mat2<$t> {
            type Output = Vec2<$t>;

            #[inline(always)]
            fn mul(self, rhs: Vec2<$t>) -> Self::Output {
                Vec2::<$t>([
                    self.m00() * rhs.0[0] + self.m10() * rhs.0[1],
                    self.m01() * rhs.0[0] + self.m11() * rhs.0[1],
                ])
            }
        }

        impl Mul<Vec2<$t>> for Mat2d<$t> {
            type Output = Vec2<$t>;

            #[inline(always)]
            fn mul(self, rhs: Vec2<$t>) -> Self::Output {
                Vec2::<$t>([
                    self.a() * rhs.0[0] + self.c() * rhs.0[1] + self.tx(),
                    self.b() * rhs.0[0] + self.d() * rhs.0[1] + self.ty(),
                ])
            }
        }

        impl Mul<Vec2<$t>> for Mat3<$t> {
            type Output = Vec2<$t>;

            #[inline(always)]
            fn mul(self, rhs: Vec2<$t>) -> Self::Output {
                Vec2::<$t>([
                    self.m00() * rhs.0[0] + self.m10() * rhs.0[1] + self.m20(),
                    self.m01() * rhs.0[0] + self.m11() * rhs.0[1] + self.m21(),
                ])
            }
        }

        impl Mul<Vec2<$t>> for Mat4<$t> {
            type Output = Vec2<$t>;

            #[inline(always)]
            fn mul(self, rhs: Vec2<$t>) -> Self::Output {
                Vec2::<$t>([
                    self.m00() * rhs.0[0] + self.m10() * rhs.0[1] + self.m30(),
                    self.m01() * rhs.0[0] + self.m11() * rhs.0[1] + self.m31(),
                ])
            }
        }

        impl MulAssign for Vec2<$t> {
            #[inline(always)]
            fn mul_assign(&mut self, rhs: Self) {
                self.0[0] *= rhs.0[0];
                self.0[1] *= rhs.0[1];
            }
        }

        impl MulAssign<$t> for Vec2<$t> {
            #[inline(always)]
            fn mul_assign(&mut self, rhs: $t) {
                self.0[0] *= rhs;
                self.0[1] *= rhs;
            }
        }

        impl Div for Vec2<$t> {
            type Output = Vec2<$t>;

            #[inline(always)]
            fn div(self, rhs: Self) -> Self::Output {
                Self([
                    self.0[0] / rhs.0[0],
                    self.0[1] / rhs.0[1],
                
                ])
            }
        }

        impl Div<$t> for Vec2<$t> {
            type Output = Vec2<$t>;

            #[inline(always)]
            fn div(self, rhs: $t) -> Self::Output {
                Self([
                    self.0[0] / rhs,
                    self.0[1] / rhs,
                ])
            }
        }

        impl Div<Vec2<$t>> for $t {
            type Output = Vec2<$t>;

            #[inline(always)]
            fn div(self, rhs: Vec2<$t>) -> Self::Output {
                Vec2::<$t>([
                    self / rhs.0[0],
                    self / rhs.0[1],
                ])
            }
        }

        impl DivAssign for Vec2<$t> {
            #[inline(always)]
            fn div_assign(&mut self, rhs: Self) {
                self.0[0] /= rhs.0[0];
                self.0[1] /= rhs.0[1];
            }
        }

        impl DivAssign<$t> for Vec2<$t> {
            #[inline(always)]
            fn div_assign(&mut self, rhs: $t) {
                self.0[0] /= rhs;
                self.0[1] /= rhs;
            }
        }

        impl Vec2<$t> {
            #[inline(always)]
            pub fn min(&self, other: &Self) -> Self {
                Self::new(self.0[0].min(other.0[0]), self.0[1].min(other.0[1]))
            }

            #[inline(always)]
            pub fn min_in_place(&mut self, other: &Self) -> &mut Self {
                self.0[0] = self.0[0].min(other.0[0]);
                self.0[1] = self.0[1].min(other.0[1]);
                self
            }

            #[inline(always)]
            pub fn max(&self, other: &Self) -> Self {
                Self::new(self.0[0].max(other.0[0]), self.0[1].max(other.0[1]))
            }

            #[inline(always)]
            pub fn max_in_place(&mut self, other: &Self) -> &mut Self {
                self.0[0] = self.0[0].max(other.0[0]);
                self.0[1] = self.0[1].max(other.0[1]);
                self
            }

            #[inline(always)]
            pub fn ceil(&self) -> Self {
                Self::new(self.0[0].ceil(), self.0[1].ceil())
            }

            #[inline(always)]
            pub fn ceil_in_place(&mut self) -> &mut  Self {
                self.0[0] = self.0[0].ceil();
                self.0[1] = self.0[1].ceil();
                self
            }

            #[inline(always)]
            pub fn floor(&self) -> Self {
                Self::new(self.0[0].floor(), self.0[1].floor())
            }

            #[inline(always)]
            pub fn floor_in_place(&mut self) -> &mut  Self {
                self.0[0] = self.0[0].floor();
                self.0[1] = self.0[1].floor();
                self
            }

            #[inline(always)]
            pub fn round(&self) -> Self {
                Self::new(self.0[0].round(), self.0[1].round())
            }

            #[inline(always)]
            pub fn round_in_place(&mut self) -> &mut  Self {
                self.0[0] = self.0[0].round();
                self.0[1] = self.0[1].round();
                self
            }

            #[inline(always)]
            pub fn squared_distance(&self, b: &Self) -> $t {
                let x = b.x() - self.x();
                let y = b.y() - self.y();
                x * x + y * y
            }

            #[inline(always)]
            pub fn distance(&self, b: &Self) -> $t {
                self.squared_distance(b).sqrt()
            }

            #[inline(always)]
            pub fn squared_length(&self) -> $t {
                let x = self.x();
                let y = self.y();
                x * x + y * y
            }

            #[inline(always)]
            pub fn length(&self) -> $t {
                self.squared_length().sqrt()
            }

            #[inline(always)]
            pub fn inverse(&self) -> Self {
                let x = self.x();
                let y = self.y();

                Self::new($one / x, $one / y)
            }

            #[inline(always)]
            pub fn inverse_in_place(&mut self) -> &mut Self {
                self.0[0] = $one / self.0[0];
                self.0[1] = $one / self.0[1];
                self
            }

            #[inline(always)]
            pub fn normalize(&self) -> Self {
                let mut len = self.squared_length();
                if len > $zero {
                    len = $one / len.sqrt();
                }

                let x = self.x();
                let y = self.y();

                Self::new(x * len, y * len)
            }

            #[inline(always)]
            pub fn normalize_in_place(&mut self) -> &mut Self {
                let mut len = self.squared_length();
                if len > $zero {
                    len = $one / len.sqrt();
                }

                self.0[0] *= len;
                self.0[1] *= len;
                self
            }

            #[inline(always)]
            pub fn dot(&self, b: &Self) -> $t {
                let x = self.x();
                let y = self.y();

                x * b.x() + y * b.y()
            }

            #[inline(always)]
            pub fn cross(&self, b: &Self) -> Vec3<$t> {
                let x = self.x();
                let y = self.y();

                Vec3::new($zero, $zero, x * b.y() - y * b.x())
            }

            #[inline(always)]
            pub fn lerp(&self, b: &Self, t: $t) -> Self {
                let ax = self.x();
                let ay = self.y();

                Self::new(ax + t * (b.x() - ax), ay + t * (b.y() - ay))
            }

            #[inline(always)]
            pub fn rotate(&self, b: &Self, rad: $t) -> Self
            {
                //Translate point to the origin
                let p0 = self.x() - b.x();
                let p1 = self.y() - b.y();
                let sin_c = rad.sin();
                let cos_c = rad.cos();

                //perform rotation and translate to correct position
                Self::new(
                    p0 * cos_c - p1 * sin_c + b.x(),
                    p0 * sin_c + p1 * cos_c + b.y(),
                )
            }

            #[inline(always)]
            pub fn rotate_in_place(&mut self, b: &Self, rad: $t) -> &mut Self
            {
                //Translate point to the origin
                let p0 = self.x() - b.x();
                let p1 = self.y() - b.y();
                let sin_c = rad.sin();
                let cos_c = rad.cos();

                //perform rotation and translate to correct position
                self.0[0] = p0 * cos_c - p1 * sin_c + b.x();
                self.0[1] = p0 * sin_c + p1 * cos_c + b.y();
                self
            }

            #[inline(always)]
            pub fn angle(&self, b: &Self) -> $t {
                let x1 = self.x();
                let y1 = self.y();
                let x2 = b.x();
                let y2 = b.y();
                // mag is the product of the magnitudes of a and b
                let mag = ((x1 * x1 + y1 * y1) * (x2 * x2 + y2 * y2)).sqrt();
                // mag &&.. short circuits if mag == 0
                let cosine = if mag == $zero {
                    mag
                } else {
                    (x1 * x2 + y1 * y2) / mag
                };
                // Math.min(Math.max(cosine, -1), 1) clamps the cosine between -1 and 1
                cosine.max(-$one).min($one).acos()
            }

            #[inline(always)]
            pub fn set_zero(&mut self) -> &mut Self {
                self.0[0] = $zero;
                self.0[1] = $zero;
                self
            }

            #[inline(always)]
            pub fn copy(&mut self, b: &Self) -> &mut Self {
                self.0[0] = b.0[0];
                self.0[1] = b.0[1];
                self
            }
        }

        impl ApproximateEq for Vec2<$t> {
            #[inline(always)]
            fn approximate_eq(&self, other: &Self) -> bool {
                let a0 = self.x();
                let a1 = self.y();
                let b0 = other.x();
                let b1 = other.y();

                (a0 - b0).abs() <= $epsilon * $one.max(a0.abs()).max(b0.abs())
                    && (a1 - b1).abs() <= $epsilon * $one.max(a1.abs()).max(b1.abs())
            }
        }
       )+
    };
}

basic_constructors! {
    (u8, 0u8),
    (u16, 0u16),
    (u32, 0u32),
    (u64, 0u64),
    (u128, 0u128),
    (usize, 0usize),
    (i8, 0i8),
    (i16, 0i16),
    (i32, 0i32),
    (i64, 0i64),
    (i128, 0i128),
    (isize, 0isize),
    (f32, 0.0f32),
    (f64, 0.0f64)
}
decimal_constructors! {
    (f32, std::f32::consts::PI),
    (f64, std::f64::consts::PI)
}
neg!(i8, i16, i32, i64, i128, isize, f32, f64);
math! {
    (f32, super::EPSILON_F32, 0.0f32, 1.0f32, std::f32::consts::PI),
    (f64, super::EPSILON_F64, 0.0f64, 1.0f64, std::f64::consts::PI)
}

#[cfg(feature = "gl")]
impl super::GLF32<2> for Vec2<f32> {
    #[inline(always)]
    fn gl_f32(&self) -> [f32; 2] {
        self.0.clone()
    }
}

#[cfg(feature = "gl")]
impl super::GLF32<2> for Vec2<f64> {
    #[inline(always)]
    fn gl_f32(&self) -> [f32; 2] {
        [self.0[0] as f32, self.0[1] as f32]
    }
}

#[cfg(feature = "gl")]
impl super::GLF32Borrowed<2> for Vec2<f32> {
    #[inline(always)]
    fn gl_f32_borrowed(&self) -> &[f32; 2] {
        &self.0
    }
}

#[cfg(test)]
mod tests {
    use crate::{mat2::Mat2, mat2d::Mat2d, vec3::Vec3, ApproximateEq};

    use super::Vec2;

    #[test]
    fn new() {
        assert_eq!(Vec2::new(2.0, 3.0).raw(), &[2.0, 3.0]);
    }

    #[test]
    fn new_zero() {
        assert_eq!(Vec2::<f64>::new_zero().raw(), &[0.0, 0.0]);
    }

    #[test]
    fn from_slice() {
        assert_eq!(Vec2::from_slice([2.0, 3.0]).raw(), &[2.0, 3.0]);
    }

    #[test]
    fn raw() {
        assert_eq!(Vec2::new(2.0, 3.0).raw(), &[2.0, 3.0]);
    }

    #[test]
    fn raw_mut() {
        let mut vec = Vec2::new(2.0, 3.0);
        (*vec.raw_mut()) = [4.0, 4.0];
        assert_eq!(vec.raw(), &[4.0, 4.0]);
    }

    #[test]
    fn x() {
        assert_eq!(Vec2::<f64>::new(2.0, 3.0).x(), &2.0);
    }

    #[test]
    fn y() {
        assert_eq!(Vec2::<f64>::new(2.0, 3.0).y(), &3.0);
    }

    #[test]
    fn x_mut() {
        let mut vec = Vec2::new(2.0, 3.0);
        (*vec.x_mut()) = 5.0;
        assert_eq!(vec.raw(), &[5.0, 3.0]);
    }

    #[test]
    fn y_mut() {
        let mut vec = Vec2::new(2.0, 3.0);
        (*vec.y_mut()) = 5.0;
        assert_eq!(vec.raw(), &[2.0, 5.0]);
    }

    #[test]
    fn set_x() {
        let mut vec = Vec2::new(2.0, 3.0);
        vec.set_x(5.0);
        assert_eq!(vec.raw(), &[5.0, 3.0]);
    }

    #[test]
    fn set_y() {
        let mut vec = Vec2::new(2.0, 3.0);
        vec.set_y(5.0);
        assert_eq!(vec.raw(), &[2.0, 5.0]);
    }

    #[test]
    fn r() {
        assert_eq!(Vec2::<f64>::new(2.0, 3.0).x(), &2.0);
    }

    #[test]
    fn g() {
        assert_eq!(Vec2::<f64>::new(2.0, 3.0).y(), &3.0);
    }

    #[test]
    fn r_mut() {
        let mut vec = Vec2::new(2.0, 3.0);
        (*vec.r_mut()) = 5.0;
        assert_eq!(vec.raw(), &[5.0, 3.0]);
    }

    #[test]
    fn g_mut() {
        let mut vec = Vec2::new(2.0, 3.0);
        (*vec.g_mut()) = 5.0;
        assert_eq!(vec.raw(), &[2.0, 5.0]);
    }

    #[test]
    fn set_r() {
        let mut vec = Vec2::new(2.0, 3.0);
        vec.set_r(5.0);
        assert_eq!(vec.raw(), &[5.0, 3.0]);
    }

    #[test]
    fn set_g() {
        let mut vec = Vec2::new(2.0, 3.0);
        vec.set_g(5.0);
        assert_eq!(vec.raw(), &[2.0, 5.0]);
    }

    #[test]
    fn s() {
        assert_eq!(Vec2::<f64>::new(2.0, 3.0).x(), &2.0);
    }

    #[test]
    fn t() {
        assert_eq!(Vec2::<f64>::new(2.0, 3.0).y(), &3.0);
    }

    #[test]
    fn s_mut() {
        let mut vec = Vec2::new(2.0, 3.0);
        (*vec.s_mut()) = 5.0;
        assert_eq!(vec.raw(), &[5.0, 3.0]);
    }

    #[test]
    fn t_mut() {
        let mut vec = Vec2::new(2.0, 3.0);
        (*vec.t_mut()) = 5.0;
        assert_eq!(vec.raw(), &[2.0, 5.0]);
    }

    #[test]
    fn set_s() {
        let mut vec = Vec2::new(2.0, 3.0);
        vec.set_s(5.0);
        assert_eq!(vec.raw(), &[5.0, 3.0]);
    }

    #[test]
    fn set_t() {
        let mut vec = Vec2::new(2.0, 3.0);
        vec.set_t(5.0);
        assert_eq!(vec.raw(), &[2.0, 5.0]);
    }

    #[test]
    fn set() {
        let mut vec = Vec2::new(2.0, 3.0);
        vec.set(5.0, 5.0);
        assert_eq!(vec.raw(), &[5.0, 5.0]);
    }

    #[test]
    fn set_zero() {
        let mut vec = Vec2::<f64>::new(1.0, 2.0);
        vec.set_zero();
        assert_eq!(vec.raw(), &[0.0, 0.0]);
    }

    #[test]
    fn add_vec2_vec2() {
        let vec0 = Vec2::<f64>::new(2.0, 3.0);
        let vec1 = Vec2::<f64>::new(1.0, 2.0);
        assert_eq!((vec0 + vec1).approximate_eq(&Vec2::new(3.0, 5.0)), true);
    }

    #[test]
    fn add_vec2_scalar() {
        let vec = Vec2::<f64>::new(2.0, 3.0);
        let scalar = 1.0;
        assert_eq!((vec + scalar).approximate_eq(&Vec2::new(3.0, 4.0)), true);
    }

    #[test]
    fn add_scalar_vec2() {
        let scalar = 1.0;
        let vec = Vec2::<f64>::new(2.0, 3.0);
        assert_eq!((scalar + vec).approximate_eq(&Vec2::new(3.0, 4.0)), true);
    }

    #[test]
    fn add_assign_vec2_vec2() {
        let mut vec0 = Vec2::<f64>::new(2.0, 3.0);
        let vec1 = Vec2::<f64>::new(1.0, 2.0);
        vec0 += vec1;
        assert_eq!(vec0.approximate_eq(&Vec2::new(3.0, 5.0)), true);
    }

    #[test]
    fn add_assign_vec2_scalar() {
        let mut vec = Vec2::<f64>::new(2.0, 3.0);
        let scalar = 1.0;
        vec += scalar;
        assert_eq!(vec.approximate_eq(&Vec2::new(3.0, 4.0)), true);
    }

    #[test]
    fn sub_vec2_vec2() {
        let vec0 = Vec2::<f64>::new(2.0, 2.0);
        let vec1 = Vec2::<f64>::new(1.0, 3.0);
        assert_eq!((vec0 - vec1).approximate_eq(&Vec2::new(1.0, -1.0)), true);
    }

    #[test]
    fn sub_vec2_scalar() {
        let vec = Vec2::<f64>::new(2.0, 3.0);
        let scalar = 1.0;
        assert_eq!((vec - scalar).approximate_eq(&Vec2::new(1.0, 2.0)), true);
    }

    #[test]
    fn sub_scalar_vec2() {
        let scalar = 1.0;
        let vec = Vec2::<f64>::new(2.0, 3.0);
        assert_eq!((scalar - vec).approximate_eq(&Vec2::new(-1.0, -2.0)), true);
    }

    #[test]
    fn sub_assign_vec2_vec2() {
        let mut vec0 = Vec2::<f64>::new(2.0, 3.0);
        let vec1 = Vec2::<f64>::new(1.0, 2.0);
        vec0 -= vec1;
        assert_eq!(vec0.approximate_eq(&Vec2::new(1.0, 1.0)), true);
    }

    #[test]
    fn sub_assign_vec2_scalar() {
        let mut vec = Vec2::<f64>::new(2.0, 3.0);
        let scalar = 1.0;
        vec -= scalar;
        assert_eq!(vec.approximate_eq(&Vec2::new(1.0, 2.0)), true);
    }

    #[test]
    fn mul_vec2_vec2() {
        let vec0 = Vec2::<f64>::new(2.0, 2.0);
        let vec1 = Vec2::<f64>::new(1.0, 3.0);
        assert_eq!((vec0 * vec1).approximate_eq(&Vec2::new(2.0, 6.0)), true);
    }

    #[test]
    fn mul_vec2_scalar() {
        let vec = Vec2::<f64>::new(2.0, 3.0);
        let scalar = 1.0;
        assert_eq!((vec * scalar).approximate_eq(&Vec2::new(2.0, 3.0)), true);
    }

    #[test]
    fn mul_scalar_vec2() {
        let scalar = 3.0;
        let vec = Vec2::<f64>::new(2.0, 3.0);
        assert_eq!((scalar * vec).approximate_eq(&Vec2::new(6.0, 9.0)), true);
    }

    #[test]
    fn mul_mat2_vec2() {
        let mat = Mat2::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let vec = Vec2::<f64>::new(1.0, 2.0);
        assert_eq!((mat * vec).approximate_eq(&Vec2::new(7.0, 10.0)), true);
    }

    #[test]
    fn mul_mat2d_vec2() {
        let mat = Mat2d::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let vec = Vec2::<f64>::new(1.0, 2.0);
        assert_eq!((mat * vec).approximate_eq(&Vec2::new(12.0, 16.0)), true);
    }

    #[test]
    fn mul_assign_vec2_vec2() {
        let mut vec0 = Vec2::<f64>::new(2.0, 3.0);
        let vec1 = Vec2::<f64>::new(1.0, 2.0);
        vec0 *= vec1;
        assert_eq!(vec0.approximate_eq(&Vec2::new(2.0, 6.0)), true);
    }

    #[test]
    fn mul_assign_vec2_scalar() {
        let mut vec = Vec2::<f64>::new(2.0, 3.0);
        let scalar = 4.0;
        vec *= scalar;
        assert_eq!(vec.approximate_eq(&Vec2::new(8.0, 12.0)), true);
    }

    #[test]
    fn div_vec2_vec2() {
        let vec0 = Vec2::<f64>::new(2.0, 2.0);
        let vec1 = Vec2::<f64>::new(1.0, 3.0);
        assert_eq!(
            (vec0 / vec1).approximate_eq(&Vec2::new(2.0, 0.6666666666666667)),
            true
        );
    }

    #[test]
    fn div_vec2_scalar() {
        let vec = Vec2::<f64>::new(2.0, 3.0);
        let scalar = 2.0;
        assert_eq!((vec / scalar).approximate_eq(&Vec2::new(1.0, 1.5)), true);
    }

    #[test]
    fn div_scalar_vec2() {
        let scalar = 2.0;
        let vec = Vec2::<f64>::new(2.0, 3.0);
        assert_eq!(
            (scalar / vec).approximate_eq(&Vec2::new(1.0, 0.6666666666666667)),
            true
        );
    }

    #[test]
    fn div_assign_vec2_vec2() {
        let mut vec0 = Vec2::<f64>::new(2.0, 3.0);
        let vec1 = Vec2::<f64>::new(1.0, 2.0);
        vec0 /= vec1;
        assert_eq!(vec0.approximate_eq(&Vec2::new(2.0, 1.5)), true);
    }

    #[test]
    fn div_assign_vec2_scalar() {
        let mut vec = Vec2::<f64>::new(2.0, 3.0);
        let scalar = 4.0;
        vec /= scalar;
        assert_eq!(vec.approximate_eq(&Vec2::new(0.5, 0.75)), true);
    }

    #[test]
    fn neg() {
        let vec = Vec2::<f64>::new(2.0, 3.0);
        assert_eq!((-vec).approximate_eq(&Vec2::new(-2.0, -3.0)), true);
    }

    #[test]
    fn min() {
        assert_eq!(
            Vec2::<f64>::new(1.0, 2.0)
                .min(&Vec2::new(3.0, 4.0))
                .approximate_eq(&Vec2::new(1.0, 2.0)),
            true
        );
    }

    #[test]
    fn min_in_place() {
        let mut vec0 = Vec2::<f64>::new(2.0, 2.0);
        let vec1 = Vec2::<f64>::new(1.0, 3.0);
        vec0.min_in_place(&vec1);
        assert_eq!(vec0.approximate_eq(&Vec2::new(1.0, 2.0)), true);
    }

    #[test]
    fn max() {
        assert_eq!(
            Vec2::<f64>::new(1.0, 2.0)
                .max(&Vec2::new(3.0, 4.0))
                .approximate_eq(&Vec2::new(3.0, 4.0)),
            true
        );
    }

    #[test]
    fn max_in_place() {
        let mut vec0 = Vec2::<f64>::new(2.0, 2.0);
        let vec1 = Vec2::<f64>::new(1.0, 3.0);
        vec0.max_in_place(&vec1);
        assert_eq!(vec0.approximate_eq(&Vec2::new(2.0, 3.0)), true);
    }

    #[test]
    fn ceil() {
        let vec = Vec2::new(std::f64::consts::E, std::f64::consts::PI);
        assert_eq!(vec.ceil().approximate_eq(&Vec2::new(3.0, 4.0)), true);
    }

    #[test]
    fn ceil_in_place() {
        let mut vec = Vec2::new(std::f64::consts::E, std::f64::consts::PI);
        vec.ceil_in_place();
        assert_eq!(vec.approximate_eq(&Vec2::new(3.0, 4.0)), true);
    }

    #[test]
    fn floor() {
        let vec = Vec2::new(std::f64::consts::E, std::f64::consts::PI);
        assert_eq!(vec.floor().approximate_eq(&Vec2::new(2.0, 3.0)), true);
    }

    #[test]
    fn floor_in_place() {
        let mut vec = Vec2::new(std::f64::consts::E, std::f64::consts::PI);
        vec.floor_in_place();
        assert_eq!(vec.approximate_eq(&Vec2::new(2.0, 3.0)), true);
    }

    #[test]
    fn round() {
        let vec = Vec2::new(std::f64::consts::E, std::f64::consts::PI);
        assert_eq!(vec.round().approximate_eq(&Vec2::new(3.0, 3.0)), true);
    }

    #[test]
    fn round_in_place() {
        let mut vec = Vec2::new(std::f64::consts::E, std::f64::consts::PI);
        vec.round_in_place();
        assert_eq!(vec.approximate_eq(&Vec2::new(3.0, 3.0)), true);
    }

    #[test]
    fn squared_distance() {
        let vec0 = Vec2::<f64>::new(1.0, 2.0);
        let vec1 = Vec2::<f64>::new(3.0, 4.0);
        assert_eq!(vec0.squared_distance(&vec1).approximate_eq(&8.0), true);
    }

    #[test]
    fn distance() {
        let vec0 = Vec2::<f64>::new(1.0, 2.0);
        let vec1 = Vec2::<f64>::new(3.0, 4.0);
        assert_eq!(
            vec0.distance(&vec1).approximate_eq(&2.8284271247461903),
            true
        );
    }

    #[test]
    fn squared_length() {
        let vec = Vec2::<f64>::new(1.0, 2.0);
        assert_eq!(vec.squared_length().approximate_eq(&5.0), true);
    }

    #[test]
    fn length() {
        let vec = Vec2::<f64>::new(1.0, 2.0);
        assert_eq!(vec.length().approximate_eq(&2.23606797749979), true);
    }

    #[test]
    fn inverse() {
        let vec = Vec2::<f64>::new(1.0, 2.0);
        assert_eq!(vec.inverse().approximate_eq(&Vec2::new(1.0, 0.5)), true);
    }

    #[test]
    fn inverse_in_place() {
        let mut vec = Vec2::<f64>::new(1.0, 2.0);
        vec.inverse_in_place();
        assert_eq!(vec.approximate_eq(&Vec2::new(1.0, 0.5)), true);
    }

    #[test]
    fn normalize() {
        let vec = Vec2::<f64>::new(5.0, 2.0);
        assert_eq!(
            vec.normalize()
                .approximate_eq(&Vec2::new(0.9284766908852592, 0.37139067635410367)),
            true
        );
    }

    #[test]
    fn normalize_in_place() {
        let mut vec = Vec2::<f64>::new(5.0, 2.0);
        vec.normalize_in_place();
        assert_eq!(
            vec.approximate_eq(&Vec2::new(0.9284766908852592, 0.37139067635410367)),
            true
        );
    }

    #[test]
    fn dot() {
        let vec0 = Vec2::<f64>::new(1.0, 2.0);
        let vec1: Vec2 = Vec2::<f64>::new(3.0, 4.0);
        assert_eq!(vec0.dot(&vec1).approximate_eq(&11.0), true);
    }

    #[test]
    fn cross() {
        let vec0 = Vec2::<f64>::new(1.0, 2.0);
        let vec1: Vec2 = Vec2::<f64>::new(3.0, 4.0);
        assert_eq!(
            vec0.cross(&vec1).approximate_eq(&Vec3::new(0.0, 0.0, -2.0)),
            true
        );
    }

    #[test]
    fn lerp() {
        let vec0 = Vec2::<f64>::new(1.0, 2.0);
        let vec1 = Vec2::<f64>::new(3.0, 4.0);
        let t = 0.5;
        assert_eq!(
            vec0.lerp(&vec1, t).approximate_eq(&Vec2::new(2.0, 3.0)),
            true
        );
    }

    #[test]
    fn rotate() {
        let vec0 = Vec2::<f64>::new(0.0, 1.0);
        let vec1 = Vec2::<f64>::new(0.0, 0.0);
        assert_eq!(
            vec0.rotate(&vec1, std::f64::consts::PI)
                .approximate_eq(&Vec2::new(-1.2246467991473532e-16, -1.0)),
            true
        );

        let vec0 = Vec2::<f64>::new(6.0, -5.0);
        let vec1: Vec2 = Vec2::<f64>::new(0.0, -5.0);
        assert_eq!(
            vec0.rotate(&vec1, std::f64::consts::PI)
                .approximate_eq(&Vec2::new(-6.0, -4.999999999999999)),
            true
        );
    }

    #[test]
    fn rotate_in_place() {
        let mut vec0 = Vec2::<f64>::new(0.0, 1.0);
        let vec1 = Vec2::<f64>::new(0.0, 0.0);
        vec0.rotate_in_place(&vec1, std::f64::consts::PI);
        assert_eq!(
            vec0.approximate_eq(&Vec2::new(-1.2246467991473532e-16, -1.0)),
            true
        );

        let mut vec0 = Vec2::<f64>::new(6.0, -5.0);
        let vec1: Vec2 = Vec2::<f64>::new(0.0, -5.0);
        vec0.rotate_in_place(&vec1, std::f64::consts::PI);
        assert_eq!(
            vec0.approximate_eq(&Vec2::new(-6.0, -4.999999999999999)),
            true
        );
    }

    #[test]
    fn angle() {
        let vec0 = Vec2::<f64>::new(1.0, 0.0);
        let vec1 = Vec2::<f64>::new(1.0, 2.0);
        assert_eq!(vec0.angle(&vec1).approximate_eq(&1.1071487177940904), true);
    }

    #[test]
    fn display() {
        let vec = Vec2::<f64>::new(1.0, 2.0);
        assert_eq!(vec.to_string(), "vec2(1, 2)");
    }
}
