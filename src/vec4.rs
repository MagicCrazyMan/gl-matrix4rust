use std::{
    fmt::{Debug, Display},
    ops::{Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign},
    slice::SliceIndex,
};

use crate::{mat4::Mat4, quat::Quat, ApproximateEq};

pub struct Vec4<T = f64>([T; 4]);

impl<T: Debug> Debug for Vec4<T> {
    #[inline(always)]
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_tuple("Vec4").field(&self.0).finish()
    }
}

impl<T: Copy> Copy for Vec4<T> {}

impl<T: Clone> Clone for Vec4<T> {
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

impl<T: PartialEq> PartialEq for Vec4<T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl<T: Display> Display for Vec4<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_fmt(format_args!(
            "vec4({}, {}, {}, {})",
            self.0[0], self.0[1], self.0[2], self.0[3]
        ))
    }
}

impl<T> Vec4<T> {
    #[inline(always)]
    pub const fn new(x: T, y: T, z: T, w: T) -> Self {
        Self([x, y, z, w])
    }

    #[inline(always)]
    pub const fn from_slice(values: [T; 4]) -> Self {
        Self(values)
    }
}

impl<T> Vec4<T> {
    #[inline(always)]
    pub fn raw(&self) -> &[T; 4] {
        &self.0
    }

    #[inline(always)]
    pub fn raw_mut(&mut self) -> &mut [T; 4] {
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
    pub fn z(&self) -> &T {
        &self.0[2]
    }

    #[inline(always)]
    pub fn w(&self) -> &T {
        &self.0[3]
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
    pub fn b(&self) -> &T {
        &self.0[2]
    }

    #[inline(always)]
    pub fn a(&self) -> &T {
        &self.0[3]
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
    pub fn z_mut(&mut self) -> &mut T {
        &mut self.0[2]
    }

    #[inline(always)]
    pub fn w_mut(&mut self) -> &mut T {
        &mut self.0[3]
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
    pub fn b_mut(&mut self) -> &mut T {
        &mut self.0[2]
    }

    #[inline(always)]
    pub fn a_mut(&mut self) -> &mut T {
        &mut self.0[3]
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
    pub fn set_z(&mut self, z: T) {
        self.0[2] = z;
    }

    #[inline(always)]
    pub fn set_w(&mut self, w: T) {
        self.0[3] = w;
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
    pub fn set_b(&mut self, b: T) {
        self.0[2] = b;
    }

    #[inline(always)]
    pub fn set_a(&mut self, a: T) {
        self.0[3] = a;
    }

    #[inline(always)]
    pub fn set(&mut self, x: T, y: T, z: T, w: T) {
        self.0[0] = x;
        self.0[1] = y;
        self.0[2] = z;
        self.0[3] = w;
    }
}

impl<T, I> Index<I> for Vec4<T>
where
    I: SliceIndex<[T], Output = T>,
{
    type Output = T;

    #[inline(always)]
    fn index(&self, index: I) -> &Self::Output {
        self.0.index(index)
    }
}

impl<T, I> IndexMut<I> for Vec4<T>
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
        impl Vec4<$t> {
            #[inline(always)]
            pub const fn new_zero() -> Self {
                Self([$zero, $zero, $zero, $zero])
            }
        }

        #[cfg(feature = "rand")]
        impl rand::distributions::Distribution<Vec4<$t>> for rand::distributions::Standard {
            fn sample<R: rand::prelude::Rng + ?Sized>(&self, rng: &mut R) -> Vec4<$t> {
                Vec4::new(
                    rng.gen::<$t>(),
                    rng.gen::<$t>(),
                    rng.gen::<$t>(),
                    rng.gen::<$t>(),
                )
            }
        }
       )+
    };
}

macro_rules! decimal_constructors {
    ($($t: ident),+) => {
       $(
        impl Vec4<$t> {
            #[inline(always)]
            pub fn random() -> Self {
                // Marsaglia, George. Choosing a Point from the Surface of a
                // Sphere. Ann. Math. Statist. 43 (1972), no. 2, 645--646.
                // http://projecteuclid.org/euclid.aoms/1177692644;
                let v1;
                let v2;
                let v3;
                let v4;
                let s1;
                let s2;
                let mut rand;

                rand = rand::random::<$t>();
                v1 = rand * 2.0 - 1.0;
                v2 = (4.0 * rand::random::<$t>() - 2.0) * (rand * -rand + rand).sqrt();
                s1 = v1 * v1 + v2 * v2;

                rand = rand::random::<$t>();
                v3 = rand * 2.0 - 1.0;
                v4 = (4.0 * rand::random::<$t>() - 2.0) * (rand * -rand + rand).sqrt();
                s2 = v3 * v3 + v4 * v4;

                let d = ((1.0 - s1) / s2).sqrt();

                Self([v1, v2, v3 * d, v4 * d])
            }

            #[inline(always)]
            pub fn random_with_scale(scale: $t) -> Self {
                let v1;
                let v2;
                let v3;
                let v4;
                let s1;
                let s2;
                let mut rand;

                rand = rand::random::<$t>();
                v1 = rand * 2.0 - 1.0;
                v2 = (4.0 * rand::random::<$t>() - 2.0) * (rand * -rand + rand).sqrt();
                s1 = v1 * v1 + v2 * v2;

                rand = rand::random::<$t>();
                v3 = rand * 2.0 - 1.0;
                v4 = (4.0 * rand::random::<$t>() - 2.0) * (rand * -rand + rand).sqrt();
                s2 = v3 * v3 + v4 * v4;

                let d = ((1.0 - s1) / s2).sqrt();

                Self([scale * v1, scale * v2, scale * v3 * d, scale * v4 * d])
            }
        }
       )+
    };
}

macro_rules! neg {
    ($($t: ident),+) => {
       $(
        impl Neg for Vec4<$t> {
            type Output = Vec4<$t>;

            #[inline(always)]
            fn neg(mut self) -> Self::Output {
                self.0[0] = -self.0[0];
                self.0[1] = -self.0[1];
                self.0[2] = -self.0[2];
                self.0[3] = -self.0[3];
                self
            }
        }
       )+
    };
}

macro_rules! math {
    ($(($t: ident, $epsilon: expr, $zero: expr, $one: expr)),+) => {
       $(
        impl Add for Vec4<$t> {
            type Output = Vec4<$t>;

            #[inline(always)]
            fn add(mut self, rhs: Self) -> Self::Output {
                self.0[0] = self.0[0] + rhs.0[0];
                self.0[1] = self.0[1] + rhs.0[1];
                self.0[2] = self.0[2] + rhs.0[2];
                self.0[3] = self.0[3] + rhs.0[3];
                self
            }
        }

        impl Add<$t> for Vec4<$t> {
            type Output = Vec4<$t>;

            #[inline(always)]
            fn add(mut self, rhs: $t) -> Self::Output {
                self.0[0] = self.0[0] + rhs;
                self.0[1] = self.0[1] + rhs;
                self.0[2] = self.0[2] + rhs;
                self.0[3] = self.0[3] + rhs;
                self
            }
        }

        impl Add<Vec4<$t>> for $t {
            type Output = Vec4<$t>;

            #[inline(always)]
            fn add(self, mut rhs: Vec4<$t>) -> Self::Output {
                rhs.0[0] = self + rhs.0[0];
                rhs.0[1] = self + rhs.0[1];
                rhs.0[2] = self + rhs.0[2];
                rhs.0[3] = self + rhs.0[3];
                rhs
            }
        }

        impl AddAssign for Vec4<$t> {
            #[inline(always)]
            fn add_assign(&mut self, rhs: Self) {
                self.0[0] += rhs.0[0];
                self.0[1] += rhs.0[1];
                self.0[2] += rhs.0[2];
                self.0[3] += rhs.0[3];
            }
        }

        impl AddAssign<$t> for Vec4<$t> {
            #[inline(always)]
            fn add_assign(&mut self, rhs: $t) {
                self.0[0] += rhs;
                self.0[1] += rhs;
                self.0[2] += rhs;
                self.0[3] += rhs;
            }
        }

        impl Sub for Vec4<$t> {
            type Output = Vec4<$t>;

            #[inline(always)]
            fn sub(mut self, rhs: Self) -> Self::Output {
                self.0[0] = self.0[0] - rhs.0[0];
                self.0[1] = self.0[1] - rhs.0[1];
                self.0[2] = self.0[2] - rhs.0[2];
                self.0[3] = self.0[3] - rhs.0[3];
                self
            }
        }

        impl Sub<$t> for Vec4<$t> {
            type Output = Vec4<$t>;

            #[inline(always)]
            fn sub(mut self, rhs: $t) -> Self::Output {
                self.0[0] = self.0[0] - rhs;
                self.0[1] = self.0[1] - rhs;
                self.0[2] = self.0[2] - rhs;
                self.0[3] = self.0[3] - rhs;
                self
            }
        }

        impl Sub<Vec4<$t>> for $t {
            type Output = Vec4<$t>;

            #[inline(always)]
            fn sub(self, mut rhs: Vec4<$t>) -> Self::Output {
                rhs.0[0] = self - rhs.0[0];
                rhs.0[1] = self - rhs.0[1];
                rhs.0[2] = self - rhs.0[2];
                rhs.0[3] = self - rhs.0[3];
                rhs
            }
        }

        impl SubAssign for Vec4<$t> {
            #[inline(always)]
            fn sub_assign(&mut self, rhs: Self) {
                self.0[0] -= rhs.0[0];
                self.0[1] -= rhs.0[1];
                self.0[2] -= rhs.0[2];
                self.0[3] -= rhs.0[3];
            }
        }

        impl SubAssign<$t> for Vec4<$t> {
            #[inline(always)]
            fn sub_assign(&mut self, rhs: $t) {
                self.0[0] -= rhs;
                self.0[1] -= rhs;
                self.0[2] -= rhs;
                self.0[3] -= rhs;
            }
        }

        impl Mul for Vec4<$t> {
            type Output = Vec4<$t>;

            #[inline(always)]
            fn mul(mut self, rhs: Self) -> Self::Output {
                self.0[0] = self.0[0] * rhs.0[0];
                self.0[1] = self.0[1] * rhs.0[1];
                self.0[2] = self.0[2] * rhs.0[2];
                self.0[3] = self.0[3] * rhs.0[3];
                self
            }
        }

        impl Mul<$t> for Vec4<$t> {
            type Output = Vec4<$t>;

            #[inline(always)]
            fn mul(mut self, rhs: $t) -> Self::Output {
                self.0[0] = self.0[0] * rhs;
                self.0[1] = self.0[1] * rhs;
                self.0[2] = self.0[2] * rhs;
                self.0[3] = self.0[3] * rhs;
                self
            }
        }

        impl Mul<Vec4<$t>> for $t {
            type Output = Vec4<$t>;

            #[inline(always)]
            fn mul(self, mut rhs: Vec4<$t>) -> Self::Output {
                rhs.0[0] = self * rhs.0[0];
                rhs.0[1] = self * rhs.0[1];
                rhs.0[2] = self * rhs.0[2];
                rhs.0[3] = self * rhs.0[3];
                rhs
            }
        }

        impl Mul<Vec4<$t>> for Mat4<$t> {
            type Output = Vec4<$t>;

            #[inline(always)]
            fn mul(self, mut rhs: Vec4<$t>) -> Self::Output {
                let x = rhs.0[0];
                let y = rhs.0[1];
                let z = rhs.0[2];
                let w = rhs.0[3];

                let a00 = self.m00();
                let a01 = self.m01();
                let a02 = self.m02();
                let a03 = self.m03();
                let a10 = self.m10();
                let a11 = self.m11();
                let a12 = self.m12();
                let a13 = self.m13();
                let a20 = self.m20();
                let a21 = self.m21();
                let a22 = self.m22();
                let a23 = self.m23();
                let a30 = self.m30();
                let a31 = self.m31();
                let a32 = self.m32();
                let a33 = self.m33();

                rhs.0[0] = a00 * x + a10 * y + a20 * z + a30 * w;
                rhs.0[1] = a01 * x + a11 * y + a21 * z + a31 * w;
                rhs.0[2] = a02 * x + a12 * y + a22 * z + a32 * w;
                rhs.0[3] = a03 * x + a13 * y + a23 * z + a33 * w;
                rhs
            }
        }

        impl Mul<Vec4<$t>> for Quat<$t> {
            type Output = Vec4<$t>;

            #[inline(always)]
            fn mul(self, mut rhs: Vec4<$t>) -> Self::Output {
                let x = *self.x();
                let y = *self.y();
                let z = *self.z();
                let w = *self.w();
                let qx = rhs.0[0];
                let qy = rhs.0[1];
                let qz = rhs.0[2];
                let qw = rhs.0[3];

                // calculate quat * vec
                let ix = qw * x + qy * z - qz * y;
                let iy = qw * y + qz * x - qx * z;
                let iz = qw * z + qx * y - qy * x;
                let iw = -qx * x - qy * y - qz * z;

                rhs.0[0] = ix * qw + iw * -qx + iy * -qz - iz * -qy;
                rhs.0[1] = iy * qw + iw * -qy + iz * -qx - ix * -qz;
                rhs.0[2] = iz * qw + iw * -qz + ix * -qy - iy * -qx;
                rhs.0[3] = w;
                rhs
            }
        }

        impl Mul<Quat<$t>> for Vec4<$t> {
            type Output = Vec4<$t>;

            #[inline(always)]
            fn mul(mut self, rhs: Quat<$t>) -> Self::Output {
                let x = *rhs.x();
                let y = *rhs.y();
                let z = *rhs.z();
                let w = *rhs.w();
                let qx = self.0[0];
                let qy = self.0[1];
                let qz = self.0[2];
                let qw = self.0[3];

                // calculate quat * vec
                let ix = qw * x + qy * z - qz * y;
                let iy = qw * y + qz * x - qx * z;
                let iz = qw * z + qx * y - qy * x;
                let iw = -qx * x - qy * y - qz * z;

                self.0[0] = ix * qw + iw * -qx + iy * -qz - iz * -qy;
                self.0[1] = iy * qw + iw * -qy + iz * -qx - ix * -qz;
                self.0[2] = iz * qw + iw * -qz + ix * -qy - iy * -qx;
                self.0[3] = w;
                self
            }
        }

        impl MulAssign for Vec4<$t> {
            #[inline(always)]
            fn mul_assign(&mut self, rhs: Self) {
                self.0[0] *= rhs.0[0];
                self.0[1] *= rhs.0[1];
                self.0[2] *= rhs.0[2];
                self.0[3] *= rhs.0[3];
            }
        }

        impl MulAssign<$t> for Vec4<$t> {
            #[inline(always)]
            fn mul_assign(&mut self, rhs: $t) {
                self.0[0] *= rhs;
                self.0[1] *= rhs;
                self.0[2] *= rhs;
                self.0[3] *= rhs;
            }
        }

        impl Div for Vec4<$t> {
            type Output = Vec4<$t>;

            #[inline(always)]
            fn div(mut self, rhs: Self) -> Self::Output {
                self.0[0] = self.0[0] / rhs.0[0];
                self.0[1] = self.0[1] / rhs.0[1];
                self.0[2] = self.0[2] / rhs.0[2];
                self.0[3] = self.0[3] / rhs.0[3];
                self
            }
        }

        impl Div<$t> for Vec4<$t> {
            type Output = Vec4<$t>;

            #[inline(always)]
            fn div(mut self, rhs: $t) -> Self::Output {
                self.0[0] = self.0[0] / rhs;
                self.0[1] = self.0[1] / rhs;
                self.0[2] = self.0[2] / rhs;
                self.0[3] = self.0[3] / rhs;
                self
            }
        }

        impl Div<Vec4<$t>> for $t {
            type Output = Vec4<$t>;

            #[inline(always)]
            fn div(self, mut rhs: Vec4<$t>) -> Self::Output {
                rhs.0[0] = self / rhs.0[0];
                rhs.0[1] = self / rhs.0[1];
                rhs.0[2] = self / rhs.0[2];
                rhs.0[3] = self / rhs.0[3];
                rhs
            }
        }

        impl DivAssign for Vec4<$t> {
            #[inline(always)]
            fn div_assign(&mut self, rhs: Self) {
                self.0[0] /= rhs.0[0];
                self.0[1] /= rhs.0[1];
                self.0[2] /= rhs.0[2];
                self.0[3] /= rhs.0[3];
            }
        }

        impl DivAssign<$t> for Vec4<$t> {
            #[inline(always)]
            fn div_assign(&mut self, rhs: $t) {
                self.0[0] /= rhs;
                self.0[1] /= rhs;
                self.0[2] /= rhs;
                self.0[3] /= rhs;
            }
        }

        impl Vec4<$t> {
            #[inline(always)]
            pub fn min(&self, other: &Self) -> Self {
                Self::new(
                    self.0[0].min(other.0[0]),
                    self.0[1].min(other.0[1]),
                    self.0[2].min(other.0[2]),
                    self.0[3].min(other.0[3])
                )
            }

            #[inline(always)]
            pub fn min_in_place(&mut self, other: &Self) -> &mut Self {
                self.0[0] = self.0[0].min(other.0[0]);
                self.0[1] = self.0[1].min(other.0[1]);
                self.0[2] = self.0[2].min(other.0[2]);
                self.0[3] = self.0[3].min(other.0[3]);
                self
            }

            #[inline(always)]
            pub fn max(&self, other: &Self) -> Self {
                Self::new(
                    self.0[0].max(other.0[0]),
                    self.0[1].max(other.0[1]),
                    self.0[2].max(other.0[2]),
                    self.0[3].max(other.0[3])
                )
            }

            #[inline(always)]
            pub fn max_in_place(&mut self, other: &Self) -> &mut Self {
                self.0[0] = self.0[0].max(other.0[0]);
                self.0[1] = self.0[1].max(other.0[1]);
                self.0[2] = self.0[2].max(other.0[2]);
                self.0[3] = self.0[3].max(other.0[3]);
                self
            }

            #[inline(always)]
            pub fn ceil(&self) -> Self {
                Self::new(
                    self.0[0].ceil(),
                    self.0[1].ceil(),
                    self.0[2].ceil(),
                    self.0[3].ceil()
                )
            }

            #[inline(always)]
            pub fn ceil_in_place(&mut self) -> &mut  Self {
                self.0[0] = self.0[0].ceil();
                self.0[1] = self.0[1].ceil();
                self.0[2] = self.0[2].ceil();
                self.0[3] = self.0[3].ceil();
                self
            }

            #[inline(always)]
            pub fn floor(&self) -> Self {
                Self::new(
                    self.0[0].floor(),
                    self.0[1].floor(),
                    self.0[2].floor(),
                    self.0[3].floor()
                )
            }

            #[inline(always)]
            pub fn floor_in_place(&mut self) -> &mut  Self {
                self.0[0] = self.0[0].floor();
                self.0[1] = self.0[1].floor();
                self.0[2] = self.0[2].floor();
                self.0[3] = self.0[3].floor();
                self
            }

            #[inline(always)]
            pub fn round(&self) -> Self {
                Self::new(
                    self.0[0].round(),
                    self.0[1].round(),
                    self.0[2].round(),
                    self.0[3].round()
                )
            }

            #[inline(always)]
            pub fn round_in_place(&mut self) -> &mut  Self {
                self.0[0] = self.0[0].round();
                self.0[1] = self.0[1].round();
                self.0[2] = self.0[2].round();
                self.0[3] = self.0[3].round();
                self
            }

            #[inline(always)]
            pub fn squared_distance(&self, b: &Self) -> $t {
                let x = b.x() - self.x();
                let y = b.y() - self.y();
                let z = b.y() - self.y();
                let w = b.w() - self.w();
                x * x + y * y + z * z + w * w
            }

            #[inline(always)]
            pub fn distance(&self, b: &Self) -> $t {
                self.squared_distance(b).sqrt()
            }

            #[inline(always)]
            pub fn squared_length(&self) -> $t {
                let x = self.x();
                let y = self.y();
                let z = self.z();
                let w = self.w();
                x * x + y * y + z * z + w * w
            }

            #[inline(always)]
            pub fn length(&self) -> $t {
                self.squared_length().sqrt()
            }

            #[inline(always)]
            pub fn inverse(&self) -> Self {
                let x = self.x();
                let y = self.y();
                let z = self.z();
                let w = self.w();

                Self::new($one / x, $one / y, $one / z, $one / w)
            }

            #[inline(always)]
            pub fn inverse_in_place(&mut self) -> &mut Self {
                self.0[0] = $one / self.0[0];
                self.0[1] = $one / self.0[1];
                self.0[2] = $one / self.0[2];
                self.0[3] = $one / self.0[3];
                self
            }

            #[inline(always)]
            pub fn normalize(&self) -> Self {
                let mut len = self.squared_length();
                if len > $zero {
                    len = $one / len.sqrt();
                }

                Self::new(self.x() * len, self.y() * len, self.z() * len, self.w() * len)
            }

            #[inline(always)]
            pub fn normalize_in_place(&mut self) -> &mut Self {
                let mut len = self.squared_length();
                if len > $zero {
                    len = $one / len.sqrt();
                }

                self.0[0] *= len;
                self.0[1] *= len;
                self.0[2] *= len;
                self.0[3] *= len;
                self
            }

            #[inline(always)]
            pub fn dot(&self, b: &Self) -> $t {
                self.x() * b.x() + self.y() * b.y() + self.z() * b.z() + self.w() * b.w()
            }

            #[inline(always)]
            pub fn cross(&self, v: &Self, w: &Self) -> Self {
                let a = v.x() * w.y() - v.y() * w.x();
                let b = v.x() * w.z() - v.z() * w.x();
                let c = v.x() * w.w() - v.w() * w.x();
                let d = v.y() * w.z() - v.z() * w.y();
                let e = v.y() * w.w() - v.w() * w.y();
                let f = v.z() * w.w() - v.w() * w.z();
                let g = self.x();
                let h = self.y();
                let i = self.z();
                let j = self.w();

                Self::new(
                    h * f - i * e + j * d,
                    -(g * f) + i * c - j * b,
                    g * e - h * c + j * a,
                    -(g * d) + h * b - i * a,
                )
            }

            #[inline(always)]
            pub fn lerp(&self, b: &Self, t: $t) -> Self {
                let ax = self.x();
                let ay = self.y();
                let az = self.z();
                let aw = self.w();

                Self::new(
                    ax + t * (b.x() - ax),
                    ay + t * (b.y() - ay),
                    az + t * (b.z() - az),
                    aw + t * (b.w() - aw),
                )
            }

            #[inline(always)]
            pub fn set_zero(&mut self) -> &mut Self {
                self.0[0] = $zero;
                self.0[1] = $zero;
                self.0[2] = $zero;
                self.0[3] = $zero;
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
        }

        impl ApproximateEq for Vec4<$t> {
            #[inline(always)]
            fn approximate_eq(&self, other: &Self) -> bool {
                let a0 = self.x();
                let a1 = self.y();
                let a2 = self.z();
                let a3 = self.w();
                let b0 = other.x();
                let b1 = other.y();
                let b2 = other.z();
                let b3 = other.w();

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
decimal_constructors!(f32, f64);
neg!(i8, i16, i32, i64, i128, isize, f32, f64);
math! {
    (f32, super::EPSILON_F32, 0.0f32, 1.0f32),
    (f64, super::EPSILON_F64, 0.0f64, 1.0f64)
}

#[cfg(test)]
mod tests {
    use crate::ApproximateEq;

    use super::Vec4;

    #[test]
    fn new() {
        assert_eq!(Vec4::new(2.0, 3.0, 4.0, 5.0).raw(), &[2.0, 3.0, 4.0, 5.0]);
    }

    #[test]
    fn new_zero() {
        assert_eq!(Vec4::<f64>::new_zero().raw(), &[0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn from_slice() {
        assert_eq!(
            Vec4::from_slice([2.0, 3.0, 4.0, 5.0]).raw(),
            &[2.0, 3.0, 4.0, 5.0]
        );
    }

    #[test]
    fn raw() {
        assert_eq!(Vec4::new(2.0, 3.0, 4.0, 5.0).raw(), &[2.0, 3.0, 4.0, 5.0]);
    }

    #[test]
    fn raw_mut() {
        let mut vec = Vec4::new(2.0, 3.0, 4.0, 5.0);
        (*vec.raw_mut()) = [11.0, 11.0, 11.0, 11.0];
        assert_eq!(vec.raw(), &[11.0, 11.0, 11.0, 11.0]);
    }

    #[test]
    fn x() {
        assert_eq!(Vec4::<f64>::new(2.0, 3.0, 4.0, 5.0).x(), &2.0);
    }

    #[test]
    fn y() {
        assert_eq!(Vec4::<f64>::new(2.0, 3.0, 4.0, 5.0).y(), &3.0);
    }

    #[test]
    fn z() {
        assert_eq!(Vec4::<f64>::new(2.0, 3.0, 4.0, 5.0).z(), &4.0);
    }

    #[test]
    fn w() {
        assert_eq!(Vec4::<f64>::new(2.0, 3.0, 4.0, 5.0).w(), &5.0);
    }

    #[test]
    fn x_mut() {
        let mut vec = Vec4::new(2.0, 3.0, 4.0, 5.0);
        (*vec.x_mut()) = 6.0;
        assert_eq!(vec.raw(), &[6.0, 3.0, 4.0, 5.0]);
    }

    #[test]
    fn y_mut() {
        let mut vec = Vec4::new(2.0, 3.0, 4.0, 5.0);
        (*vec.y_mut()) = 6.0;
        assert_eq!(vec.raw(), &[2.0, 6.0, 4.0, 5.0]);
    }

    #[test]
    fn z_mut() {
        let mut vec = Vec4::new(2.0, 3.0, 4.0, 5.0);
        (*vec.z_mut()) = 6.0;
        assert_eq!(vec.raw(), &[2.0, 3.0, 6.0, 5.0]);
    }

    #[test]
    fn w_mut() {
        let mut vec = Vec4::new(2.0, 3.0, 4.0, 5.0);
        (*vec.w_mut()) = 6.0;
        assert_eq!(vec.raw(), &[2.0, 3.0, 4.0, 6.0]);
    }

    #[test]
    fn set_x() {
        let mut vec = Vec4::new(2.0, 3.0, 4.0, 5.0);
        vec.set_x(6.0);
        assert_eq!(vec.raw(), &[6.0, 3.0, 4.0, 5.0]);
    }

    #[test]
    fn set_y() {
        let mut vec = Vec4::new(2.0, 3.0, 4.0, 5.0);
        vec.set_y(6.0);
        assert_eq!(vec.raw(), &[2.0, 6.0, 4.0, 5.0]);
    }

    #[test]
    fn set_z() {
        let mut vec = Vec4::new(2.0, 3.0, 4.0, 5.0);
        vec.set_z(6.0);
        assert_eq!(vec.raw(), &[2.0, 3.0, 6.0, 5.0]);
    }

    #[test]
    fn set_w() {
        let mut vec = Vec4::new(2.0, 3.0, 4.0, 5.0);
        vec.set_w(6.0);
        assert_eq!(vec.raw(), &[2.0, 3.0, 4.0, 6.0]);
    }

    #[test]
    fn r() {
        assert_eq!(Vec4::<f64>::new(2.0, 3.0, 4.0, 5.0).r(), &2.0);
    }

    #[test]
    fn g() {
        assert_eq!(Vec4::<f64>::new(2.0, 3.0, 4.0, 5.0).g(), &3.0);
    }

    #[test]
    fn b() {
        assert_eq!(Vec4::<f64>::new(2.0, 3.0, 4.0, 5.0).b(), &4.0);
    }

    #[test]
    fn a() {
        assert_eq!(Vec4::<f64>::new(2.0, 3.0, 4.0, 5.0).a(), &5.0);
    }

    #[test]
    fn r_mut() {
        let mut vec = Vec4::new(2.0, 3.0, 4.0, 5.0);
        (*vec.r_mut()) = 6.0;
        assert_eq!(vec.raw(), &[6.0, 3.0, 4.0, 5.0]);
    }

    #[test]
    fn g_mut() {
        let mut vec = Vec4::new(2.0, 3.0, 4.0, 5.0);
        (*vec.g_mut()) = 6.0;
        assert_eq!(vec.raw(), &[2.0, 6.0, 4.0, 5.0]);
    }

    #[test]
    fn b_mut() {
        let mut vec = Vec4::new(2.0, 3.0, 4.0, 5.0);
        (*vec.b_mut()) = 6.0;
        assert_eq!(vec.raw(), &[2.0, 3.0, 6.0, 5.0]);
    }

    #[test]
    fn a_mut() {
        let mut vec = Vec4::new(2.0, 3.0, 4.0, 5.0);
        (*vec.a_mut()) = 6.0;
        assert_eq!(vec.raw(), &[2.0, 3.0, 4.0, 6.0]);
    }

    #[test]
    fn set_r() {
        let mut vec = Vec4::new(2.0, 3.0, 4.0, 5.0);
        vec.set_r(6.0);
        assert_eq!(vec.raw(), &[6.0, 3.0, 4.0, 5.0]);
    }

    #[test]
    fn set_g() {
        let mut vec = Vec4::new(2.0, 3.0, 4.0, 5.0);
        vec.set_g(6.0);
        assert_eq!(vec.raw(), &[2.0, 6.0, 4.0, 5.0]);
    }

    #[test]
    fn set_b() {
        let mut vec = Vec4::new(2.0, 3.0, 4.0, 5.0);
        vec.set_b(6.0);
        assert_eq!(vec.raw(), &[2.0, 3.0, 6.0, 5.0]);
    }

    #[test]
    fn set_a() {
        let mut vec = Vec4::new(2.0, 3.0, 4.0, 5.0);
        vec.set_a(6.0);
        assert_eq!(vec.raw(), &[2.0, 3.0, 4.0, 6.0]);
    }

    #[test]
    fn set() {
        let mut vec = Vec4::new(2.0, 3.0, 4.0, 5.0);
        vec.set(22.0, 22.0, 22.0, 22.0);
        assert_eq!(vec.raw(), &[22.0, 22.0, 22.0, 22.0]);
    }

    #[test]
    fn set_zero() {
        let mut vec = Vec4::<f64>::new(2.0, 3.0, 4.0, 5.0);
        vec.set_zero();
        assert_eq!(vec.raw(), &[0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn add_vec4_vec4() {
        let vec0 = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let vec1 = Vec4::<f64>::new(5.0, 6.0, 7.0, 8.0);
        assert_eq!(
            (vec0 + vec1).approximate_eq(&Vec4::new(6.0, 8.0, 10.0, 12.0)),
            true
        );
    }

    #[test]
    fn add_vec4_scalar() {
        let vec = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let scalar = 1.0;
        assert_eq!(
            (vec + scalar).approximate_eq(&Vec4::new(2.0, 3.0, 4.0, 5.0)),
            true
        );
    }

    #[test]
    fn add_scalar_vec4() {
        let scalar = 1.0;
        let vec = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(
            (scalar + vec).approximate_eq(&Vec4::new(2.0, 3.0, 4.0, 5.0)),
            true
        );
    }

    #[test]
    fn add_assign_vec4_vec4() {
        let mut vec0 = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let vec1 = Vec4::<f64>::new(5.0, 6.0, 7.0, 8.0);
        vec0 += vec1;
        assert_eq!(vec0.approximate_eq(&Vec4::new(6.0, 8.0, 10.0, 12.0)), true);
    }

    #[test]
    fn add_assign_vec4_scalar() {
        let mut vec = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let scalar = 1.0;
        vec += scalar;
        assert_eq!(vec.approximate_eq(&Vec4::new(2.0, 3.0, 4.0, 5.0)), true);
    }

    #[test]
    fn sub_vec4_vec4() {
        let vec0 = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let vec1 = Vec4::<f64>::new(5.0, 6.0, 7.0, 8.0);
        assert_eq!(
            (vec0 - vec1).approximate_eq(&Vec4::new(-4.0, -4.0, -4.0, -4.0)),
            true
        );
    }

    #[test]
    fn sub_vec4_scalar() {
        let vec = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let scalar = 1.0;
        assert_eq!(
            (vec - scalar).approximate_eq(&Vec4::new(0.0, 1.0, 2.0, 3.0)),
            true
        );
    }

    #[test]
    fn sub_scalar_vec4() {
        let scalar = 1.0;
        let vec = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(
            (scalar - vec).approximate_eq(&Vec4::new(0.0, -1.0, -2.0, -3.0)),
            true
        );
    }

    #[test]
    fn sub_assign_vec4_vec4() {
        let mut vec0 = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let vec1 = Vec4::<f64>::new(5.0, 6.0, 7.0, 8.0);
        vec0 -= vec1;
        assert_eq!(
            vec0.approximate_eq(&Vec4::new(-4.0, -4.0, -4.0, -4.0)),
            true
        );
    }

    #[test]
    fn sub_assign_vec4_scalar() {
        let mut vec = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let scalar = 1.0;
        vec -= scalar;
        assert_eq!(vec.approximate_eq(&Vec4::new(0.0, 1.0, 2.0, 3.0)), true);
    }

    #[test]
    fn mul_vec4_vec4() {
        let vec0 = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let vec1 = Vec4::<f64>::new(5.0, 6.0, 7.0, 8.0);
        assert_eq!(
            (vec0 * vec1).approximate_eq(&Vec4::new(5.0, 12.0, 21.0, 32.0)),
            true
        );
    }

    #[test]
    fn mul_vec4_scalar() {
        let vec = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let scalar = 2.0;
        assert_eq!(
            (vec * scalar).approximate_eq(&Vec4::new(2.0, 4.0, 6.0, 8.0)),
            true
        );
    }

    #[test]
    fn mul_scalar_vec4() {
        let scalar = 3.0;
        let vec = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(
            (scalar * vec).approximate_eq(&Vec4::new(3.0, 6.0, 9.0, 12.0)),
            true
        );
    }

    #[test]
    fn mul_assign_vec4_vec4() {
        let mut vec0 = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let vec1 = Vec4::<f64>::new(5.0, 6.0, 7.0, 8.0);
        vec0 *= vec1;
        assert_eq!(vec0.approximate_eq(&Vec4::new(5.0, 12.0, 21.0, 32.0)), true);
    }

    #[test]
    fn mul_assign_vec4_scalar() {
        let mut vec = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let scalar = 4.0;
        vec *= scalar;
        assert_eq!(vec.approximate_eq(&Vec4::new(4.0, 8.0, 12.0, 16.0)), true);
    }

    #[test]
    fn div_vec4_vec4() {
        let vec0 = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let vec1 = Vec4::<f64>::new(5.0, 6.0, 7.0, 8.0);
        assert_eq!(
            (vec0 / vec1).approximate_eq(&Vec4::new(
                0.2,
                0.3333333333333333,
                0.42857142857142855,
                0.5
            )),
            true
        );
    }

    #[test]
    fn div_vec4_scalar() {
        let vec = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let scalar = 2.0;
        assert_eq!(
            (vec / scalar).approximate_eq(&Vec4::new(0.5, 1.0, 1.5, 2.0)),
            true
        );
    }

    #[test]
    fn div_scalar_vec4() {
        let scalar = 2.0;
        let vec = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(
            (scalar / vec).approximate_eq(&Vec4::new(2.0, 1.0, 0.6666666666666667, 0.5)),
            true
        );
    }

    #[test]
    fn div_assign_vec4_vec4() {
        let mut vec0 = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let vec1 = Vec4::<f64>::new(5.0, 6.0, 7.0, 8.0);
        vec0 /= vec1;
        assert_eq!(
            vec0.approximate_eq(&Vec4::new(
                0.2,
                0.3333333333333333,
                0.42857142857142855,
                0.5
            )),
            true
        );
    }

    #[test]
    fn div_assign_vec4_scalar() {
        let mut vec = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let scalar = 4.0;
        vec /= scalar;
        assert_eq!(vec.approximate_eq(&Vec4::new(0.25, 0.5, 0.75, 1.0)), true);
    }

    #[test]
    fn neg() {
        let vec = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(
            (-vec).approximate_eq(&Vec4::new(-1.0, -2.0, -3.0, -4.0)),
            true
        );
    }

    #[test]
    fn min() {
        let vec0 = Vec4::<f64>::new(1.0, 10.0, 3.0, 30.0);
        let vec1 = Vec4::<f64>::new(4.0, 5.0, 18.0, 4.0);
        assert_eq!(
            vec0.min(&vec1)
                .approximate_eq(&Vec4::new(1.0, 5.0, 3.0, 4.0)),
            true
        );
    }

    #[test]
    fn min_in_place() {
        let mut vec0 = Vec4::<f64>::new(1.0, 10.0, 3.0, 30.0);
        let vec1 = Vec4::<f64>::new(4.0, 5.0, 18.0, 4.0);
        vec0.min_in_place(&vec1);
        assert_eq!(vec0.approximate_eq(&Vec4::new(1.0, 5.0, 3.0, 4.0)), true);
    }

    #[test]
    fn max() {
        let vec0 = Vec4::<f64>::new(1.0, 10.0, 3.0, 30.0);
        let vec1 = Vec4::<f64>::new(4.0, 5.0, 18.0, 4.0);
        assert_eq!(
            vec0.max(&vec1)
                .approximate_eq(&Vec4::new(4.0, 10.0, 18.0, 30.0)),
            true
        );
    }

    #[test]
    fn max_in_place() {
        let mut vec0 = Vec4::<f64>::new(1.0, 10.0, 3.0, 30.0);
        let vec1 = Vec4::<f64>::new(4.0, 5.0, 18.0, 4.0);
        vec0.max_in_place(&vec1);
        assert_eq!(vec0.approximate_eq(&Vec4::new(4.0, 10.0, 18.0, 30.0)), true);
    }

    #[test]
    fn ceil() {
        let vec = Vec4::new(
            std::f64::consts::E,
            std::f64::consts::PI,
            std::f64::consts::SQRT_2,
            0.5f64.sqrt(),
        );
        assert_eq!(
            vec.ceil().approximate_eq(&Vec4::new(3.0, 4.0, 2.0, 1.0)),
            true
        );
    }

    #[test]
    fn ceil_in_place() {
        let mut vec = Vec4::new(
            std::f64::consts::E,
            std::f64::consts::PI,
            std::f64::consts::SQRT_2,
            0.5f64.sqrt(),
        );
        vec.ceil_in_place();
        assert_eq!(vec.approximate_eq(&Vec4::new(3.0, 4.0, 2.0, 1.0)), true);
    }

    #[test]
    fn floor() {
        let vec = Vec4::new(
            std::f64::consts::E,
            std::f64::consts::PI,
            std::f64::consts::SQRT_2,
            0.5f64.sqrt(),
        );
        assert_eq!(
            vec.floor().approximate_eq(&Vec4::new(2.0, 3.0, 1.0, 0.0)),
            true
        );
    }

    #[test]
    fn floor_in_place() {
        let mut vec = Vec4::new(
            std::f64::consts::E,
            std::f64::consts::PI,
            std::f64::consts::SQRT_2,
            0.5f64.sqrt(),
        );
        vec.floor_in_place();
        assert_eq!(vec.approximate_eq(&Vec4::new(2.0, 3.0, 1.0, 0.0)), true);
    }

    #[test]
    fn round() {
        let vec = Vec4::new(
            std::f64::consts::E,
            std::f64::consts::PI,
            std::f64::consts::SQRT_2,
            0.5f64.sqrt(),
        );
        assert_eq!(
            vec.round().approximate_eq(&Vec4::new(3.0, 3.0, 1.0, 1.0)),
            true
        );
    }

    #[test]
    fn round_in_place() {
        let mut vec = Vec4::new(
            std::f64::consts::E,
            std::f64::consts::PI,
            std::f64::consts::SQRT_2,
            0.5f64.sqrt(),
        );
        vec.round_in_place();
        assert_eq!(vec.approximate_eq(&Vec4::new(3.0, 3.0, 1.0, 1.0)), true);
    }

    #[test]
    fn squared_distance() {
        let vec0 = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let vec1 = Vec4::<f64>::new(5.0, 6.0, 7.0, 8.0);
        assert_eq!(vec0.squared_distance(&vec1).approximate_eq(&64.0), true);
    }

    #[test]
    fn distance() {
        let vec0 = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let vec1 = Vec4::<f64>::new(5.0, 6.0, 7.0, 8.0);
        assert_eq!(vec0.distance(&vec1).approximate_eq(&8.0), true);
    }

    #[test]
    fn squared_length() {
        let vec = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(vec.squared_length().approximate_eq(&30.0), true);
    }

    #[test]
    fn length() {
        let vec = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(vec.length().approximate_eq(&5.477225575051661), true);
    }

    #[test]
    fn inverse() {
        let vec = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(
            vec.inverse()
                .approximate_eq(&Vec4::new(1.0, 0.5, 0.3333333333333333, 0.25)),
            true
        );
    }

    #[test]
    fn inverse_in_place() {
        let mut vec = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        vec.inverse_in_place();
        assert_eq!(
            vec.approximate_eq(&Vec4::new(1.0, 0.5, 0.3333333333333333, 0.25)),
            true
        );
    }

    #[test]
    fn normalize() {
        let vec = Vec4::<f64>::new(5.0, 2.0, 7.0, 10.0);
        assert_eq!(
            vec.normalize().approximate_eq(&Vec4::new(
                0.37476584449793077,
                0.1499063377991723,
                0.5246721822971031,
                0.7495316889958615
            )),
            true
        );
    }

    #[test]
    fn normalize_in_place() {
        let mut vec0 = Vec4::<f64>::new(5.0, 2.0, 7.0, 10.0);
        vec0.normalize_in_place();
        assert_eq!(
            vec0.approximate_eq(&Vec4::new(
                0.37476584449793077,
                0.1499063377991723,
                0.5246721822971031,
                0.7495316889958615
            )),
            true
        );
    }

    #[test]
    fn dot() {
        let vec0 = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let vec1 = Vec4::<f64>::new(5.0, 6.0, 7.0, 8.0);
        assert_eq!(vec0.dot(&vec1).approximate_eq(&70.0), true);
    }

    #[test]
    fn cross() {
        let vec0 = Vec4::<f64>::new(1.0, 0.0, 0.0, 0.0);
        let vec1 = Vec4::<f64>::new(0.0, 1.0, 0.0, 0.0);
        let vec2 = Vec4::<f64>::new(0.0, 1.0, 1.0, 0.0);
        assert_eq!(
            vec0.cross(&vec1, &vec2)
                .approximate_eq(&Vec4::new(0.0, 0.0, 0.0, -1.0)),
            true
        );
    }

    #[test]
    fn lerp() {
        let vec0 = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let vec1 = Vec4::<f64>::new(5.0, 6.0, 7.0, 8.0);
        assert_eq!(
            vec0.lerp(&vec1, 0.5)
                .approximate_eq(&Vec4::new(3.0, 4.0, 5.0, 6.0)),
            true
        );
    }

    #[test]
    fn display() {
        let vec = Vec4::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(vec.to_string(), "vec4(1, 2, 3, 4)");
    }
}
