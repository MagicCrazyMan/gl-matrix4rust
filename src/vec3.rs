use std::{
    fmt::{Debug, Display},
    ops::{Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign},
    slice::SliceIndex,
};

use crate::{mat3::Mat3, mat4::Mat4, quat::Quat, ApproximateEq};

pub struct Vec3<T = f64>([T; 3]);

impl<T: Debug> Debug for Vec3<T> {
    #[inline(always)]
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_tuple("Vec3").field(&self.0).finish()
    }
}

impl<T: Copy> Copy for Vec3<T> {}

impl<T: Clone> Clone for Vec3<T> {
    #[inline(always)]
    fn clone(&self) -> Self {
        Self([self.0[0].clone(), self.0[1].clone(), self.0[2].clone()])
    }
}

impl<T: PartialEq> PartialEq for Vec3<T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl<T: Display> Display for Vec3<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_fmt(format_args!(
            "vec3({}, {}, {})",
            self.0[0], self.0[1], self.0[2]
        ))
    }
}

impl<T> Vec3<T> {
    #[inline(always)]
    pub const fn new(x: T, y: T, z: T) -> Self {
        Self([x, y, z])
    }

    #[inline(always)]
    pub const fn from_slice(values: [T; 3]) -> Self {
        Self(values)
    }
}

impl<T> Vec3<T> {
    #[inline(always)]
    pub fn raw(&self) -> &[T; 3] {
        &self.0
    }

    #[inline(always)]
    pub fn raw_mut(&mut self) -> &mut [T; 3] {
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
    pub fn s(&self) -> &T {
        &self.0[0]
    }

    #[inline(always)]
    pub fn t(&self) -> &T {
        &self.0[1]
    }

    #[inline(always)]
    pub fn u(&self) -> &T {
        &self.0[2]
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
    pub fn s_mut(&mut self) -> &mut T {
        &mut self.0[0]
    }

    #[inline(always)]
    pub fn t_mut(&mut self) -> &mut T {
        &mut self.0[1]
    }

    #[inline(always)]
    pub fn u_mut(&mut self) -> &mut T {
        &mut self.0[2]
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
    pub fn set_s(&mut self, s: T) {
        self.0[0] = s;
    }

    #[inline(always)]
    pub fn set_t(&mut self, t: T) {
        self.0[1] = t;
    }

    #[inline(always)]
    pub fn set_u(&mut self, u: T) {
        self.0[2] = u;
    }

    #[inline(always)]
    pub fn set(&mut self, x: T, y: T, z: T) {
        self.0[0] = x;
        self.0[1] = y;
        self.0[2] = z;
    }
}

impl<T, I> Index<I> for Vec3<T>
where
    I: SliceIndex<[T], Output = T>,
{
    type Output = T;

    #[inline(always)]
    fn index(&self, index: I) -> &Self::Output {
        self.0.index(index)
    }
}

impl<T, I> IndexMut<I> for Vec3<T>
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
        impl Vec3<$t> {
            #[inline(always)]
            pub const fn new_zero() -> Self {
                Self([$zero, $zero, $zero])
            }
        }

        #[cfg(feature = "rand")]
        impl rand::distributions::Distribution<Vec3<$t>> for rand::distributions::Standard {
            fn sample<R: rand::prelude::Rng + ?Sized>(&self, rng: &mut R) -> Vec3<$t> {
                Vec3::new(
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
    ($(($t: ident, $one: expr, $two: expr, $pi: expr)),+) => {
       $(
        impl Vec3<$t> {
            #[inline(always)]
            pub fn random() -> Self {
                let r = rand::random::<$t>() * $two * $pi;
                let z = rand::random::<$t>() * $two - $one;
                let z_scale = ($one - z * z).sqrt();

                Self([r.cos() * z_scale, r.sin() * z_scale, z])
            }

            #[inline(always)]
            pub fn random_with_scale(scale: $t) -> Self {
                let r = rand::random::<$t>() * $two * $pi;
                let z = rand::random::<$t>() * $two - $one;
                let z_scale = ($one - z * z).sqrt() * scale;

                Self([r.cos() * z_scale, r.sin() * z_scale, z * scale])
            }
        }
       )+
    };
}

macro_rules! neg {
    ($($t: ident),+) => {
       $(
        impl Neg for Vec3<$t> {
            type Output = Vec3<$t>;

            #[inline(always)]
            fn neg(mut self) -> Self::Output {
                self.0[0] = -self.0[0];
                self.0[1] = -self.0[1];
                self.0[2] = -self.0[2];
                self
            }
        }
       )+
    };
}

macro_rules! math {
    ($(($t: ident, $epsilon: expr, $zero: expr, $one: expr, $two: expr, $three: expr, $pi: expr)),+) => {
       $(
        impl Add for Vec3<$t> {
            type Output = Vec3<$t>;

            #[inline(always)]
            fn add(mut self, rhs: Self) -> Self::Output {
                self.0[0] = self.0[0] + rhs.0[0];
                self.0[1] = self.0[1] + rhs.0[1];
                self.0[2] = self.0[2] + rhs.0[2];
                self
            }
        }

        impl Add<$t> for Vec3<$t> {
            type Output = Vec3<$t>;

            #[inline(always)]
            fn add(mut self, rhs: $t) -> Self::Output {
                self.0[0] = self.0[0] + rhs;
                self.0[1] = self.0[1] + rhs;
                self.0[2] = self.0[2] + rhs;
                self
            }
        }

        impl Add<Vec3<$t>> for $t {
            type Output = Vec3<$t>;

            #[inline(always)]
            fn add(self, mut rhs: Vec3<$t>) -> Self::Output {
                rhs.0[0] = self + rhs.0[0];
                rhs.0[1] = self + rhs.0[1];
                rhs.0[2] = self + rhs.0[2];
                rhs
            }
        }

        impl AddAssign for Vec3<$t> {
            #[inline(always)]
            fn add_assign(&mut self, rhs: Self) {
                self.0[0] += rhs.0[0];
                self.0[1] += rhs.0[1];
                self.0[2] += rhs.0[2];
            }
        }

        impl AddAssign<$t> for Vec3<$t> {
            #[inline(always)]
            fn add_assign(&mut self, rhs: $t) {
                self.0[0] += rhs;
                self.0[1] += rhs;
                self.0[2] += rhs;
            }
        }

        impl Sub for Vec3<$t> {
            type Output = Vec3<$t>;

            #[inline(always)]
            fn sub(mut self, rhs: Self) -> Self::Output {
                self.0[0] = self.0[0] - rhs.0[0];
                self.0[1] = self.0[1] - rhs.0[1];
                self.0[2] = self.0[2] - rhs.0[2];
                self
            }
        }

        impl Sub<$t> for Vec3<$t> {
            type Output = Vec3<$t>;

            #[inline(always)]
            fn sub(mut self, rhs: $t) -> Self::Output {
                self.0[0] = self.0[0] - rhs;
                self.0[1] = self.0[1] - rhs;
                self.0[2] = self.0[2] - rhs;
                self
            }
        }

        impl Sub<Vec3<$t>> for $t {
            type Output = Vec3<$t>;

            #[inline(always)]
            fn sub(self, mut rhs: Vec3<$t>) -> Self::Output {
                rhs.0[0] = self - rhs.0[0];
                rhs.0[1] = self - rhs.0[1];
                rhs.0[2] = self - rhs.0[2];
                rhs
            }
        }

        impl SubAssign for Vec3<$t> {
            #[inline(always)]
            fn sub_assign(&mut self, rhs: Self) {
                self.0[0] -= rhs.0[0];
                self.0[1] -= rhs.0[1];
                self.0[2] -= rhs.0[2];
            }
        }

        impl SubAssign<$t> for Vec3<$t> {
            #[inline(always)]
            fn sub_assign(&mut self, rhs: $t) {
                self.0[0] -= rhs;
                self.0[1] -= rhs;
                self.0[2] -= rhs;
            }
        }

        impl Mul for Vec3<$t> {
            type Output = Vec3<$t>;

            #[inline(always)]
            fn mul(mut self, rhs: Self) -> Self::Output {
                self.0[0] = self.0[0] * rhs.0[0];
                self.0[1] = self.0[1] * rhs.0[1];
                self.0[2] = self.0[2] * rhs.0[2];
                self
            }
        }

        impl Mul<$t> for Vec3<$t> {
            type Output = Vec3<$t>;

            #[inline(always)]
            fn mul(mut self, rhs: $t) -> Self::Output {
                self.0[0] = self.0[0] * rhs;
                self.0[1] = self.0[1] * rhs;
                self.0[2] = self.0[2] * rhs;
                self
            }
        }

        impl Mul<Vec3<$t>> for $t {
            type Output = Vec3<$t>;

            #[inline(always)]
            fn mul(self, mut rhs: Vec3<$t>) -> Self::Output {
                rhs.0[0] = self * rhs.0[0];
                rhs.0[1] = self * rhs.0[1];
                rhs.0[2] = self * rhs.0[2];
                rhs
            }
        }

        impl Mul<Vec3<$t>> for Mat3<$t> {
            type Output = Vec3<$t>;

            #[inline(always)]
            fn mul(self, mut rhs: Vec3<$t>) -> Self::Output {
                let x = rhs.0[0];
                let y = rhs.0[1];
                let z = rhs.0[2];

                rhs.0[0] = x * self.m00() + y * self.m10() + z * self.m20();
                rhs.0[1] = x * self.m01() + y * self.m11() + z * self.m21();
                rhs.0[2] = x * self.m02() + y * self.m12() + z * self.m22();
                rhs
            }
        }

        impl Mul<Vec3<$t>> for Mat4<$t> {
            type Output = Vec3<$t>;

            #[inline(always)]
            fn mul(self, mut rhs: Vec3<$t>) -> Self::Output {
                let x = rhs.0[0];
                let y = rhs.0[1];
                let z = rhs.0[2];

                let [m00, m01, m02, m03, m04, m05, m06, m07, m08, m09, m10, m11, m12, m13, m14, m15] = self.raw();

                let mut w = *m03 * x + *m07 * y + *m11 * z + *m15;
                w = if w == $zero { $one } else { w };

                rhs.0[0] = (*m00 * x + *m04 * y + *m08 * z + *m12) / w;
                rhs.0[1] = (*m01 * x + *m05 * y + *m09 * z + *m13) / w;
                rhs.0[2] = (*m02 * x + *m06 * y + *m10 * z + *m14) / w;
                rhs
            }
        }

        impl Mul<Vec3<$t>> for Quat<$t> {
            type Output = Vec3<$t>;

            #[inline(always)]
            fn mul(self, mut rhs: Vec3<$t>) -> Self::Output {
                // benchmarks: https://jsperf.com/quaternion-transform-vec3-implementations-fixed
                let qx = *self.x();
                let qy = *self.y();
                let qz = *self.z();
                let qw = *self.w();
                let x = rhs.0[0];
                let y = rhs.0[1];
                let z = rhs.0[2];
                // var qvec = [qx, qy, qz];
                // var uv = vec3.cross([], qvec, a);
                let mut uvx = qy * z - qz * y;
                let mut uvy = qz * x - qx * z;
                let mut uvz = qx * y - qy * x;
                // var uuv = vec3.cross([], qvec, uv);
                let mut uuvx = qy * uvz - qz * uvy;
                let mut uuvy = qz * uvx - qx * uvz;
                let mut uuvz = qx * uvy - qy * uvx;
                // vec3.scale(uv, uv, 2 * w);
                let w2 = qw * $two;
                uvx = uvx * w2;
                uvy = uvy * w2;
                uvz = uvz * w2;
                // vec3.scale(uuv, uuv, 2);
                uuvx = uuvx * $two;
                uuvy = uuvy * $two;
                uuvz = uuvz * $two;
                // return vec3.add(out, a, vec3.add(out, uv, uuv));

                rhs.0[0] = x + uvx + uuvx;
                rhs.0[1] = y + uvy + uuvy;
                rhs.0[2] = z + uvz + uuvz;
                rhs
            }
        }

        impl Mul<Quat<$t>> for Vec3<$t> {
            type Output = Vec3<$t>;

            #[inline(always)]
            fn mul(mut self, rhs: Quat<$t>) -> Self::Output {
                // benchmarks: https://jsperf.com/quaternion-transform-vec3-implementations-fixed
                let qx = *rhs.x();
                let qy = *rhs.y();
                let qz = *rhs.z();
                let qw = *rhs.w();
                let x = self.0[0];
                let y = self.0[1];
                let z = self.0[2];
                // var qvec = [qx, qy, qz];
                // var uv = vec3.cross([], qvec, a);
                let mut uvx = qy * z - qz * y;
                let mut uvy = qz * x - qx * z;
                let mut uvz = qx * y - qy * x;
                // var uuv = vec3.cross([], qvec, uv);
                let mut uuvx = qy * uvz - qz * uvy;
                let mut uuvy = qz * uvx - qx * uvz;
                let mut uuvz = qx * uvy - qy * uvx;
                // vec3.scale(uv, uv, 2 * w);
                let w2 = qw * $two;
                uvx = uvx * w2;
                uvy = uvy * w2;
                uvz = uvz * w2;
                // vec3.scale(uuv, uuv, 2);
                uuvx = uuvx * $two;
                uuvy = uuvy * $two;
                uuvz = uuvz * $two;
                // return vec3.add(out, a, vec3.add(out, uv, uuv));

                self.0[0] = x + uvx + uuvx;
                self.0[1] = y + uvy + uuvy;
                self.0[2] = z + uvz + uuvz;
                self
            }
        }

        impl MulAssign for Vec3<$t> {
            #[inline(always)]
            fn mul_assign(&mut self, rhs: Self) {
                self.0[0] *= rhs.0[0];
                self.0[1] *= rhs.0[1];
                self.0[2] *= rhs.0[2];
            }
        }

        impl MulAssign<$t> for Vec3<$t> {
            #[inline(always)]
            fn mul_assign(&mut self, rhs: $t) {
                self.0[0] *= rhs;
                self.0[1] *= rhs;
                self.0[2] *= rhs;
            }
        }

        impl Div for Vec3<$t> {
            type Output = Vec3<$t>;

            #[inline(always)]
            fn div(mut self, rhs: Self) -> Self::Output {
                self.0[0] = self.0[0] / rhs.0[0];
                self.0[1] = self.0[1] / rhs.0[1];
                self.0[2] = self.0[2] / rhs.0[2];
                self
            }
        }

        impl Div<$t> for Vec3<$t> {
            type Output = Vec3<$t>;

            #[inline(always)]
            fn div(mut self, rhs: $t) -> Self::Output {
                self.0[0] = self.0[0] / rhs;
                self.0[1] = self.0[1] / rhs;
                self.0[2] = self.0[2] / rhs;
                self
            }
        }

        impl Div<Vec3<$t>> for $t {
            type Output = Vec3<$t>;

            #[inline(always)]
            fn div(self, mut rhs: Vec3<$t>) -> Self::Output {
                rhs.0[0] = self / rhs.0[0];
                rhs.0[1] = self / rhs.0[1];
                rhs.0[2] = self / rhs.0[2];
                rhs
            }
        }

        impl DivAssign for Vec3<$t> {
            #[inline(always)]
            fn div_assign(&mut self, rhs: Self) {
                self.0[0] /= rhs.0[0];
                self.0[1] /= rhs.0[1];
                self.0[2] /= rhs.0[2];
            }
        }

        impl DivAssign<$t> for Vec3<$t> {
            #[inline(always)]
            fn div_assign(&mut self, rhs: $t) {
                self.0[0] /= rhs;
                self.0[1] /= rhs;
                self.0[2] /= rhs;
            }
        }

        impl Vec3<$t> {
            #[inline(always)]
            pub fn min(&self, other: &Self) -> Self {
                Self::new(
                    self.0[0].min(other.0[0]),
                    self.0[1].min(other.0[1]),
                    self.0[2].min(other.0[2])
                )
            }

            #[inline(always)]
            pub fn min_in_place(&mut self, other: &Self) -> &mut Self {
                self.0[0] = self.0[0].min(other.0[0]);
                self.0[1] = self.0[1].min(other.0[1]);
                self.0[2] = self.0[2].min(other.0[2]);
                self
            }

            #[inline(always)]
            pub fn max(&self, other: &Self) -> Self {
                Self::new(
                    self.0[0].max(other.0[0]),
                    self.0[1].max(other.0[1]),
                    self.0[2].max(other.0[2])
                )
            }

            #[inline(always)]
            pub fn max_in_place(&mut self, other: &Self) -> &mut Self {
                self.0[0] = self.0[0].max(other.0[0]);
                self.0[1] = self.0[1].max(other.0[1]);
                self.0[2] = self.0[2].max(other.0[2]);
                self
            }

            #[inline(always)]
            pub fn ceil(&self) -> Self {
                Self::new(
                    self.0[0].ceil(),
                    self.0[1].ceil(),
                    self.0[2].ceil()
                )
            }

            #[inline(always)]
            pub fn ceil_in_place(&mut self) -> &mut  Self {
                self.0[0] = self.0[0].ceil();
                self.0[1] = self.0[1].ceil();
                self.0[2] = self.0[2].ceil();
                self
            }

            #[inline(always)]
            pub fn floor(&self) -> Self {
                Self::new(
                    self.0[0].floor(),
                    self.0[1].floor(),
                    self.0[2].floor()
                )
            }

            #[inline(always)]
            pub fn floor_in_place(&mut self) -> &mut  Self {
                self.0[0] = self.0[0].floor();
                self.0[1] = self.0[1].floor();
                self.0[2] = self.0[2].floor();
                self
            }

            #[inline(always)]
            pub fn round(&self) -> Self {
                Self::new(
                    self.0[0].round(),
                    self.0[1].round(),
                    self.0[2].round()
                )
            }

            #[inline(always)]
            pub fn round_in_place(&mut self) -> &mut  Self {
                self.0[0] = self.0[0].round();
                self.0[1] = self.0[1].round();
                self.0[2] = self.0[2].round();
                self
            }

            #[inline(always)]
            pub fn squared_distance(&self, b: &Self) -> $t {
                let x = b.x() - self.x();
                let y = b.y() - self.y();
                let z = b.y() - self.y();
                x * x + y * y + z * z
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
                x * x + y * y + z * z
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

                Self::new($one / x, $one / y, $one / z)
            }

            #[inline(always)]
            pub fn inverse_in_place(&mut self) -> &mut Self {
                self.0[0] = $one / self.0[0];
                self.0[1] = $one / self.0[1];
                self.0[2] = $one / self.0[2];
                self
            }

            #[inline(always)]
            pub fn normalize(&self) -> Self {
                let mut len = self.squared_length();
                if len > $zero {
                    len = $one / len.sqrt();
                }

                Self::new(self.x() * len, self.y() * len, self.z() * len)
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
                self
            }

            #[inline(always)]
            pub fn dot(&self, b: &Self) -> $t {
                self.x() * b.x() + self.y() * b.y() + self.z() * b.z()
            }

            #[inline(always)]
            pub fn cross(&self, b: &Self) -> Self {
                let ax = self.x();
                let ay = self.y();
                let az = self.z();
                let bx = b.x();
                let by = b.y();
                let bz = b.z();

                Self::new(ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx)
            }

            #[inline(always)]
            pub fn lerp(&self, b: &Self, t: $t) -> Self {
                let ax = self.x();
                let ay = self.y();
                let az = self.z();

                Self::new(
                    ax + t * (b.x() - ax),
                    ay + t * (b.y() - ay),
                    az + t * (b.z() - az)
                )
            }

            #[inline(always)]
            pub fn slerp(&self, b: &Self, t: $t) -> Self {
                let angle = self.dot(b).max(-$one).min($one).acos();
                let sin_total = angle.sin();

                let ratio_a = (($one - t) * angle).sin() / sin_total;
                let ratio_b = (t * angle).sin() / sin_total;

                let ax = self.x();
                let ay = self.y();
                let az = self.z();

                Self::new(
                    ratio_a * ax + ratio_b * b.x(),
                    ratio_a * ay + ratio_b * b.y(),
                    ratio_a * az + ratio_b * b.z(),
                )
            }

            #[inline(always)]
            pub fn hermite(&self, b: &Self, c: &Self, d: &Self, t: $t) -> Self {
                let factor_times2 = t * t;
                let factor1 =
                    factor_times2 * ($two * t - $three) + $one;
                let factor2 = factor_times2 * (t - $two) + t;
                let factor3 = factor_times2 * (t - $one);
                let factor4 = factor_times2 * ($three - $two * t);

                let ax = self.x();
                let ay = self.y();
                let az = self.z();

                Self::new(
                    ax * factor1 + b.x() * factor2 + c.x() * factor3 + d.x() * factor4,
                    ay * factor1 + b.y() * factor2 + c.y() * factor3 + d.y() * factor4,
                    az * factor1 + b.z() * factor2 + c.z() * factor3 + d.z() * factor4,
                )
            }

            #[inline(always)]
            pub fn bezier<V1, V2, V3>(&self, b: &Self, c: &Self, d: &Self, t: $t) -> Self {
                let inverse_factor = $one - t;
                let inverse_factor_times_two = inverse_factor * inverse_factor;
                let factor_times2 = t * t;
                let factor1 = inverse_factor_times_two * inverse_factor;
                let factor2 = $three * t * inverse_factor_times_two;
                let factor3 = $three * factor_times2 * inverse_factor;
                let factor4 = factor_times2 * t;

                let ax = self.x();
                let ay = self.y();
                let az = self.z();

                Self::new(
                    ax * factor1 + b.x() * factor2 + c.x() * factor3 + d.x() * factor4,
                    ay * factor1 + b.y() * factor2 + c.y() * factor3 + d.y() * factor4,
                    az * factor1 + b.z() * factor2 + c.z() * factor3 + d.z() * factor4,
                )
            }

            #[inline(always)]
            pub fn rotate_x(&self, b: &Self, rad: $t) -> Self {
                let mut p = [$zero; 3];
                let mut r = [$zero; 3];

                //Translate point to the origin
                p[0] = self.x() - b.x();
                p[1] = self.y() - b.y();
                p[2] = self.z() - b.z();

                //perform rotation
                r[0] = p[0];
                r[1] = p[1] * rad.cos() - p[2] * rad.sin();
                r[2] = p[1] * rad.sin() + p[2] * rad.cos();

                Self::new(r[0] + b.x(), r[1] + b.y(), r[2] + b.z())
            }

            #[inline(always)]
            pub fn rotate_x_in_place(&mut self, b: &Self, rad: $t) -> &mut Self {
                let mut p = [$zero; 3];
                let mut r = [$zero; 3];

                //Translate point to the origin
                p[0] = self.x() - b.x();
                p[1] = self.y() - b.y();
                p[2] = self.z() - b.z();

                //perform rotation
                r[0] = p[0];
                r[1] = p[1] * rad.cos() - p[2] * rad.sin();
                r[2] = p[1] * rad.sin() + p[2] * rad.cos();

                self.0[0] = r[0] + b.x();
                self.0[1] = r[1] + b.y();
                self.0[2] = r[2] + b.z();
                self
            }

            #[inline(always)]
            pub fn rotate_y(&self, b: &Self, rad: $t) -> Self {
                let mut p = [$zero; 3];
                let mut r = [$zero; 3];

                //Translate point to the origin
                p[0] = self.x() - b.x();
                p[1] = self.y() - b.y();
                p[2] = self.z() - b.z();

                //perform rotation
                r[0] = p[2] * rad.sin() + p[0] * rad.cos();
                r[1] = p[1];
                r[2] = p[2] * rad.cos() - p[0] * rad.sin();

                Self::new(r[0] + b.x(), r[1] + b.y(), r[2] + b.z())
            }

            #[inline(always)]
            pub fn rotate_y_in_place(&mut self, b: &Self, rad: $t) -> &mut Self {
                let mut p = [$zero; 3];
                let mut r = [$zero; 3];

                //Translate point to the origin
                p[0] = self.x() - b.x();
                p[1] = self.y() - b.y();
                p[2] = self.z() - b.z();

                //perform rotation
                r[0] = p[2] * rad.sin() + p[0] * rad.cos();
                r[1] = p[1];
                r[2] = p[2] * rad.cos() - p[0] * rad.sin();

                self.0[0] = r[0] + b.x();
                self.0[1] = r[1] + b.y();
                self.0[2] = r[2] + b.z();
                self
            }

            #[inline(always)]
            pub fn rotate_z(&self, b: &Self, rad: $t) -> Self {
                let mut p = [$zero; 3];
                let mut r = [$zero; 3];

                //Translate point to the origin
                p[0] = self.x() - b.x();
                p[1] = self.y() - b.y();
                p[2] = self.z() - b.z();

                //perform rotation
                r[0] = p[0] * rad.cos() - p[1] * rad.sin();
                r[1] = p[0] * rad.sin() + p[1] * rad.cos();
                r[2] = p[2];

                Self::new(r[0] + b.x(), r[1] + b.y(), r[2] + b.z())
            }

            #[inline(always)]
            pub fn rotate_z_in_place(&mut self, b: &Self, rad: $t) -> &mut Self {
                let mut p = [$zero; 3];
                let mut r = [$zero; 3];

                //Translate point to the origin
                p[0] = self.x() - b.x();
                p[1] = self.y() - b.y();
                p[2] = self.z() - b.z();

                //perform rotation
                r[0] = p[0] * rad.cos() - p[1] * rad.sin();
                r[1] = p[0] * rad.sin() + p[1] * rad.cos();
                r[2] = p[2];

                self.0[0] = r[0] + b.x();
                self.0[1] = r[1] + b.y();
                self.0[2] = r[2] + b.z();
                self
            }

            #[inline(always)]
            pub fn angle(&self, b: &Self) -> $t {
                let ax = self.x();
                let ay = self.y();
                let az = self.z();
                let bx = b.x();
                let by = b.y();
                let bz = b.z();
                let mag = ((ax * ax + ay * ay + az * az) * (bx * bx + by * by + bz * bz)).sqrt();
                let cosine = if mag == $zero {
                    mag
                } else {
                    self.dot(b) / mag
                };
                // Math.min(Math.max(cosine, -1), 1) clamps the cosine between -1 and 1
                cosine.max(-$one).min($one).acos()
            }

            #[inline(always)]
            pub fn set_zero(&mut self) -> &mut Self {
                self.0[0] = $zero;
                self.0[1] = $zero;
                self.0[2] = $zero;
                self
            }

            #[inline(always)]
            pub fn copy(&mut self, b: &Self) -> &mut Self {
                self.0[0] = b.0[0];
                self.0[1] = b.0[1];
                self.0[2] = b.0[2];
                self
            }
        }

        impl ApproximateEq for Vec3<$t> {
            #[inline(always)]
            fn approximate_eq(&self, other: &Self) -> bool {
                let a0 = self.x();
                let a1 = self.y();
                let a2 = self.z();
                let b0 = other.x();
                let b1 = other.y();
                let b2 = other.z();

                (a0 - b0).abs() <= $epsilon * $one.max(a0.abs()).max(b0.abs())
                    && (a1 - b1).abs() <= $epsilon * $one.max(a1.abs()).max(b1.abs())
                    && (a2 - b2).abs() <= $epsilon * $one.max(a2.abs()).max(b2.abs())
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
    (f32, 1.0f32, 2.0f32, std::f32::consts::PI),
    (f64, 1.0f64, 2.0f64, std::f64::consts::PI)
}
neg!(i8, i16, i32, i64, i128, isize, f32, f64);
math! {
    (f32, super::EPSILON_F32, 0.0f32, 1.0f32, 2.0f32, 3.0f32, std::f32::consts::PI),
    (f64, super::EPSILON_F64, 0.0f64, 1.0f64, 2.0f64, 3.0f64, std::f64::consts::PI)
}

#[cfg(feature = "gl")]
impl super::GLF32<3> for Vec3<f32> {
    #[inline(always)]
    fn gl_f32(&self) -> [f32; 3] {
        self.0.clone()
    }
}

#[cfg(feature = "gl")]
impl super::GLF32<3> for Vec3<f64> {
    #[inline(always)]
    fn gl_f32(&self) -> [f32; 3] {
        [self.0[0] as f32, self.0[1] as f32, self.0[2] as f32]
    }
}

#[cfg(feature = "gl")]
impl super::GLF32Borrowed<3> for Vec3<f32> {
    #[inline(always)]
    fn gl_f32_borrowed(&self) -> &[f32; 3] {
        &self.0
    }
}

#[cfg(test)]
mod tests {
    use crate::{mat3::Mat3, mat4::Mat4, quat::Quat, ApproximateEq};

    use super::Vec3;

    #[test]
    fn new() {
        assert_eq!(Vec3::new(2.0, 3.0, 4.0).raw(), &[2.0, 3.0, 4.0]);
    }

    #[test]
    fn new_zero() {
        assert_eq!(Vec3::<f64>::new_zero().raw(), &[0.0, 0.0, 0.0]);
    }

    #[test]
    fn from_slice() {
        assert_eq!(Vec3::from_slice([2.0, 3.0, 4.0]).raw(), &[2.0, 3.0, 4.0]);
    }

    #[test]
    fn raw() {
        assert_eq!(Vec3::new(2.0, 3.0, 4.0).raw(), &[2.0, 3.0, 4.0]);
    }

    #[test]
    fn raw_mut() {
        let mut vec = Vec3::new(2.0, 3.0, 4.0);
        (*vec.raw_mut()) = [11.0, 11.0, 11.0];
        assert_eq!(vec.raw(), &[11.0, 11.0, 11.0]);
    }

    #[test]
    fn x() {
        assert_eq!(Vec3::<f64>::new(2.0, 3.0, 4.0).x(), &2.0);
    }

    #[test]
    fn y() {
        assert_eq!(Vec3::<f64>::new(2.0, 3.0, 4.0).y(), &3.0);
    }

    #[test]
    fn z() {
        assert_eq!(Vec3::<f64>::new(2.0, 3.0, 4.0).z(), &4.0);
    }

    #[test]
    fn x_mut() {
        let mut vec = Vec3::new(2.0, 3.0, 4.0);
        (*vec.x_mut()) = 5.0;
        assert_eq!(vec.raw(), &[5.0, 3.0, 4.0]);
    }

    #[test]
    fn y_mut() {
        let mut vec = Vec3::new(2.0, 3.0, 4.0);
        (*vec.y_mut()) = 5.0;
        assert_eq!(vec.raw(), &[2.0, 5.0, 4.0]);
    }

    #[test]
    fn z_mut() {
        let mut vec = Vec3::new(2.0, 3.0, 4.0);
        (*vec.z_mut()) = 5.0;
        assert_eq!(vec.raw(), &[2.0, 3.0, 5.0]);
    }

    #[test]
    fn set_x() {
        let mut vec = Vec3::new(2.0, 3.0, 4.0);
        vec.set_x(5.0);
        assert_eq!(vec.raw(), &[5.0, 3.0, 4.0]);
    }

    #[test]
    fn set_y() {
        let mut vec = Vec3::new(2.0, 3.0, 4.0);
        vec.set_y(5.0);
        assert_eq!(vec.raw(), &[2.0, 5.0, 4.0]);
    }

    #[test]
    fn set_z() {
        let mut vec = Vec3::new(2.0, 3.0, 4.0);
        vec.set_z(5.0);
        assert_eq!(vec.raw(), &[2.0, 3.0, 5.0]);
    }

    #[test]
    fn r() {
        assert_eq!(Vec3::<f64>::new(2.0, 3.0, 4.0).r(), &2.0);
    }

    #[test]
    fn g() {
        assert_eq!(Vec3::<f64>::new(2.0, 3.0, 4.0).g(), &3.0);
    }

    #[test]
    fn b() {
        assert_eq!(Vec3::<f64>::new(2.0, 3.0, 4.0).b(), &4.0);
    }

    #[test]
    fn r_mut() {
        let mut vec = Vec3::new(2.0, 3.0, 4.0);
        (*vec.r_mut()) = 5.0;
        assert_eq!(vec.raw(), &[5.0, 3.0, 4.0]);
    }

    #[test]
    fn g_mut() {
        let mut vec = Vec3::new(2.0, 3.0, 4.0);
        (*vec.g_mut()) = 5.0;
        assert_eq!(vec.raw(), &[2.0, 5.0, 4.0]);
    }

    #[test]
    fn b_mut() {
        let mut vec = Vec3::new(2.0, 3.0, 4.0);
        (*vec.b_mut()) = 5.0;
        assert_eq!(vec.raw(), &[2.0, 3.0, 5.0]);
    }

    #[test]
    fn set_r() {
        let mut vec = Vec3::new(2.0, 3.0, 4.0);
        vec.set_r(5.0);
        assert_eq!(vec.raw(), &[5.0, 3.0, 4.0]);
    }

    #[test]
    fn set_g() {
        let mut vec = Vec3::new(2.0, 3.0, 4.0);
        vec.set_g(5.0);
        assert_eq!(vec.raw(), &[2.0, 5.0, 4.0]);
    }

    #[test]
    fn set_b() {
        let mut vec = Vec3::new(2.0, 3.0, 4.0);
        vec.set_b(5.0);
        assert_eq!(vec.raw(), &[2.0, 3.0, 5.0]);
    }

    #[test]
    fn s() {
        assert_eq!(Vec3::<f64>::new(2.0, 3.0, 4.0).s(), &2.0);
    }

    #[test]
    fn t() {
        assert_eq!(Vec3::<f64>::new(2.0, 3.0, 4.0).t(), &3.0);
    }

    #[test]
    fn u() {
        assert_eq!(Vec3::<f64>::new(2.0, 3.0, 4.0).u(), &4.0);
    }

    #[test]
    fn s_mut() {
        let mut vec = Vec3::new(2.0, 3.0, 4.0);
        (*vec.s_mut()) = 5.0;
        assert_eq!(vec.raw(), &[5.0, 3.0, 4.0]);
    }

    #[test]
    fn t_mut() {
        let mut vec = Vec3::new(2.0, 3.0, 4.0);
        (*vec.t_mut()) = 5.0;
        assert_eq!(vec.raw(), &[2.0, 5.0, 4.0]);
    }

    #[test]
    fn u_mut() {
        let mut vec = Vec3::new(2.0, 3.0, 4.0);
        (*vec.u_mut()) = 5.0;
        assert_eq!(vec.raw(), &[2.0, 3.0, 5.0]);
    }

    #[test]
    fn set_s() {
        let mut vec = Vec3::new(2.0, 3.0, 4.0);
        vec.set_s(5.0);
        assert_eq!(vec.raw(), &[5.0, 3.0, 4.0]);
    }

    #[test]
    fn set_t() {
        let mut vec = Vec3::new(2.0, 3.0, 4.0);
        vec.set_t(5.0);
        assert_eq!(vec.raw(), &[2.0, 5.0, 4.0]);
    }

    #[test]
    fn set_u() {
        let mut vec = Vec3::new(2.0, 3.0, 4.0);
        vec.set_u(5.0);
        assert_eq!(vec.raw(), &[2.0, 3.0, 5.0]);
    }

    #[test]
    fn set() {
        let mut vec = Vec3::new(2.0, 3.0, 4.0);
        vec.set(5.0, 5.0, 5.0);
        assert_eq!(vec.raw(), &[5.0, 5.0, 5.0]);
    }

    #[test]
    fn set_zero() {
        let mut vec = Vec3::<f64>::new(2.0, 3.0, 4.0);
        vec.set_zero();
        assert_eq!(vec.raw(), &[0.0, 0.0, 0.0]);
    }

    #[test]
    fn add_vec3_vec3() {
        let vec0 = Vec3::<f64>::new(1.0, 2.0, 3.0);
        let vec1 = Vec3::<f64>::new(4.0, 5.0, 6.0);
        assert_eq!(
            (vec0 + vec1).approximate_eq(&Vec3::new(5.0, 7.0, 9.0)),
            true
        );
    }

    #[test]
    fn add_vec3_scalar() {
        let vec = Vec3::<f64>::new(1.0, 2.0, 3.0);
        let scalar = 1.0;
        assert_eq!(
            (vec + scalar).approximate_eq(&Vec3::new(2.0, 3.0, 4.0)),
            true
        );
    }

    #[test]
    fn add_scalar_vec3() {
        let scalar = 1.0;
        let vec = Vec3::<f64>::new(1.0, 2.0, 3.0);
        assert_eq!(
            (scalar + vec).approximate_eq(&Vec3::new(2.0, 3.0, 4.0)),
            true
        );
    }

    #[test]
    fn add_assign_vec3_vec3() {
        let mut vec0 = Vec3::<f64>::new(1.0, 2.0, 3.0);
        let vec1 = Vec3::<f64>::new(4.0, 5.0, 6.0);
        vec0 += vec1;
        assert_eq!(vec0.approximate_eq(&Vec3::new(5.0, 7.0, 9.0)), true);
    }

    #[test]
    fn add_assign_vec3_scalar() {
        let mut vec = Vec3::<f64>::new(1.0, 2.0, 3.0);
        let scalar = 1.0;
        vec += scalar;
        assert_eq!(vec.approximate_eq(&Vec3::new(2.0, 3.0, 4.0)), true);
    }

    #[test]
    fn sub_vec3_vec3() {
        let vec0 = Vec3::<f64>::new(1.0, 2.0, 3.0);
        let vec1 = Vec3::<f64>::new(4.0, 5.0, 6.0);
        assert_eq!(
            (vec0 - vec1).approximate_eq(&Vec3::new(-3.0, -3.0, -3.0)),
            true
        );
    }

    #[test]
    fn sub_vec3_scalar() {
        let vec = Vec3::<f64>::new(1.0, 2.0, 3.0);
        let scalar = 1.0;
        assert_eq!(
            (vec - scalar).approximate_eq(&Vec3::new(0.0, 1.0, 2.0)),
            true
        );
    }

    #[test]
    fn sub_scalar_vec3() {
        let scalar = 1.0;
        let vec = Vec3::<f64>::new(1.0, 2.0, 3.0);
        assert_eq!(
            (scalar - vec).approximate_eq(&Vec3::new(0.0, -1.0, -2.0)),
            true
        );
    }

    #[test]
    fn sub_assign_vec3_vec3() {
        let mut vec0 = Vec3::<f64>::new(1.0, 2.0, 3.0);
        let vec1 = Vec3::<f64>::new(4.0, 5.0, 6.0);
        vec0 -= vec1;
        assert_eq!(vec0.approximate_eq(&Vec3::new(-3.0, -3.0, -3.0)), true);
    }

    #[test]
    fn sub_assign_vec3_scalar() {
        let mut vec = Vec3::<f64>::new(1.0, 2.0, 3.0);
        let scalar = 1.0;
        vec -= scalar;
        assert_eq!(vec.approximate_eq(&Vec3::new(0.0, 1.0, 2.0)), true);
    }

    #[test]
    fn mul_vec3_vec3() {
        let vec0 = Vec3::<f64>::new(1.0, 2.0, 3.0);
        let vec1 = Vec3::<f64>::new(4.0, 5.0, 6.0);
        assert_eq!(
            (vec0 * vec1).approximate_eq(&Vec3::new(4.0, 10.0, 18.0)),
            true
        );
    }

    #[test]
    fn mul_vec3_scalar() {
        let vec = Vec3::<f64>::new(1.0, 2.0, 3.0);
        let scalar = 2.0;
        assert_eq!(
            (vec * scalar).approximate_eq(&Vec3::new(2.0, 4.0, 6.0)),
            true
        );
    }

    #[test]
    fn mul_scalar_vec3() {
        let scalar = 3.0;
        let vec = Vec3::<f64>::new(1.0, 2.0, 3.0);
        assert_eq!(
            (scalar * vec).approximate_eq(&Vec3::new(3.0, 6.0, 9.0)),
            true
        );
    }

    #[test]
    fn mul_mat3_vec3() {
        let mat = Mat3::<f64>::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        let vec = Vec3::<f64>::new(1.0, 2.0, 3.0);
        assert_eq!((mat * vec).approximate_eq(&Vec3::new(1.0, 2.0, 3.0)), true);
    }

    #[test]
    fn mul_mat4_vec3() {
        let vec = Vec3::<f64>::new(1.0, 2.0, 3.0);
        let mat = Mat4::<f64>::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        );
        assert_eq!((mat * vec).approximate_eq(&Vec3::new(1.0, 2.0, 3.0)), true);
    }

    #[test]
    fn mul_quat_vec3() {
        let quat = Quat::<f64>::new(
            0.18257418567011074,
            0.3651483713402215,
            0.5477225570103322,
            0.730296742680443,
        );
        let vec = Vec3::<f64>::new(1.0, 2.0, 3.0);
        assert_eq!((quat * vec).approximate_eq(&Vec3::new(1.0, 2.0, 3.0)), true);
    }

    #[test]
    fn mul_vec3_quat() {
        let vec = Vec3::<f64>::new(1.0, 2.0, 3.0);
        let quat = Quat::<f64>::new(
            0.18257418567011074,
            0.3651483713402215,
            0.5477225570103322,
            0.730296742680443,
        );
        assert_eq!((vec * quat).approximate_eq(&Vec3::new(1.0, 2.0, 3.0)), true);
    }

    #[test]
    fn mul_assign_vec3_vec3() {
        let mut vec0 = Vec3::<f64>::new(1.0, 2.0, 3.0);
        let vec1 = Vec3::<f64>::new(4.0, 5.0, 6.0);
        vec0 *= vec1;
        assert_eq!(vec0.approximate_eq(&Vec3::new(4.0, 10.0, 18.0)), true);
    }

    #[test]
    fn mul_assign_vec3_scalar() {
        let mut vec = Vec3::<f64>::new(1.0, 2.0, 3.0);
        let scalar = 4.0;
        vec *= scalar;
        assert_eq!(vec.approximate_eq(&Vec3::new(4.0, 8.0, 12.0)), true);
    }

    #[test]
    fn div_vec3_vec3() {
        let vec0 = Vec3::<f64>::new(1.0, 2.0, 3.0);
        let vec1 = Vec3::<f64>::new(4.0, 5.0, 6.0);
        assert_eq!(
            (vec0 / vec1).approximate_eq(&Vec3::new(0.25, 0.4, 0.5)),
            true
        );
    }

    #[test]
    fn div_vec3_scalar() {
        let vec = Vec3::<f64>::new(1.0, 2.0, 3.0);
        let scalar = 2.0;
        assert_eq!(
            (vec / scalar).approximate_eq(&Vec3::new(0.5, 1.0, 1.5)),
            true
        );
    }

    #[test]
    fn div_scalar_vec3() {
        let scalar = 2.0;
        let vec = Vec3::<f64>::new(1.0, 2.0, 3.0);
        assert_eq!(
            (scalar / vec).approximate_eq(&Vec3::new(2.0, 1.0, 0.6666666666666667)),
            true
        );
    }

    #[test]
    fn div_assign_vec3_vec3() {
        let mut vec0 = Vec3::<f64>::new(1.0, 2.0, 3.0);
        let vec1 = Vec3::<f64>::new(4.0, 5.0, 6.0);
        vec0 /= vec1;
        assert_eq!(vec0.approximate_eq(&Vec3::new(0.25, 0.4, 0.5)), true);
    }

    #[test]
    fn div_assign_vec3_scalar() {
        let mut vec = Vec3::<f64>::new(1.0, 2.0, 3.0);
        let scalar = 4.0;
        vec /= scalar;
        assert_eq!(vec.approximate_eq(&Vec3::new(0.25, 0.5, 0.75)), true);
    }

    #[test]
    fn neg() {
        let vec = Vec3::<f64>::new(1.0, 2.0, 3.0);
        assert_eq!((-vec).approximate_eq(&Vec3::new(-1.0, -2.0, -3.0)), true);
    }

    #[test]
    fn min() {
        let vec0 = Vec3::<f64>::new(1.0, 10.0, 3.0);
        let vec1 = Vec3::<f64>::new(4.0, 5.0, 18.0);
        assert_eq!(
            vec0.min(&vec1).approximate_eq(&Vec3::new(1.0, 5.0, 3.0)),
            true
        );
    }

    #[test]
    fn min_in_place() {
        let mut vec0 = Vec3::<f64>::new(1.0, 10.0, 3.0);
        let vec1 = Vec3::<f64>::new(4.0, 5.0, 18.0);
        vec0.min_in_place(&vec1);
        assert_eq!(vec0.approximate_eq(&Vec3::new(1.0, 5.0, 3.0)), true);
    }

    #[test]
    fn max() {
        let vec0 = Vec3::<f64>::new(1.0, 10.0, 3.0);
        let vec1 = Vec3::<f64>::new(4.0, 5.0, 18.0);
        assert_eq!(
            vec0.max(&vec1).approximate_eq(&Vec3::new(4.0, 10.0, 18.0)),
            true
        );
    }

    #[test]
    fn max_in_place() {
        let mut vec0 = Vec3::<f64>::new(1.0, 10.0, 3.0);
        let vec1 = Vec3::<f64>::new(4.0, 5.0, 18.0);
        vec0.max_in_place(&vec1);
        assert_eq!(vec0.approximate_eq(&Vec3::new(4.0, 10.0, 18.0)), true);
    }

    #[test]
    fn ceil() {
        let vec = Vec3::new(
            std::f64::consts::E,
            std::f64::consts::PI,
            std::f64::consts::SQRT_2,
        );
        assert_eq!(vec.ceil().approximate_eq(&Vec3::new(3.0, 4.0, 2.0)), true);
    }

    #[test]
    fn ceil_in_place() {
        let mut vec = Vec3::new(
            std::f64::consts::E,
            std::f64::consts::PI,
            std::f64::consts::SQRT_2,
        );
        vec.ceil_in_place();
        assert_eq!(vec.approximate_eq(&Vec3::new(3.0, 4.0, 2.0)), true);
    }

    #[test]
    fn floor() {
        let vec = Vec3::new(
            std::f64::consts::E,
            std::f64::consts::PI,
            std::f64::consts::SQRT_2,
        );
        assert_eq!(vec.floor().approximate_eq(&Vec3::new(2.0, 3.0, 1.0)), true);
    }

    #[test]
    fn floor_in_place() {
        let mut vec = Vec3::new(
            std::f64::consts::E,
            std::f64::consts::PI,
            std::f64::consts::SQRT_2,
        );
        vec.floor_in_place();
        assert_eq!(vec.approximate_eq(&Vec3::new(2.0, 3.0, 1.0)), true);
    }

    #[test]
    fn round() {
        let vec = Vec3::new(
            std::f64::consts::E,
            std::f64::consts::PI,
            std::f64::consts::SQRT_2,
        );
        assert_eq!(vec.round().approximate_eq(&Vec3::new(3.0, 3.0, 1.0)), true);
    }

    #[test]
    fn round_in_place() {
        let mut vec = Vec3::new(
            std::f64::consts::E,
            std::f64::consts::PI,
            std::f64::consts::SQRT_2,
        );
        vec.round_in_place();
        assert_eq!(vec.approximate_eq(&Vec3::new(3.0, 3.0, 1.0)), true);
    }

    #[test]
    fn squared_distance() {
        let vec0 = Vec3::<f64>::new(1.0, 2.0, 3.0);
        let vec1 = Vec3::<f64>::new(4.0, 5.0, 6.0);
        assert_eq!(vec0.squared_distance(&vec1).approximate_eq(&27.0), true);
    }

    #[test]
    fn distance() {
        let vec0 = Vec3::<f64>::new(1.0, 2.0, 3.0);
        let vec1 = Vec3::<f64>::new(4.0, 5.0, 6.0);
        assert_eq!(
            vec0.distance(&vec1).approximate_eq(&5.196152422706632),
            true
        );
    }

    #[test]
    fn squared_length() {
        let vec = Vec3::<f64>::new(1.0, 2.0, 3.0);
        assert_eq!(vec.squared_length().approximate_eq(&14.0), true);
    }

    #[test]
    fn length() {
        let vec = Vec3::<f64>::new(1.0, 2.0, 3.0);
        assert_eq!(vec.length().approximate_eq(&3.7416573867739413), true);
    }

    #[test]
    fn inverse() {
        let vec = Vec3::<f64>::new(1.0, 2.0, 3.0);
        assert_eq!(
            vec.inverse()
                .approximate_eq(&Vec3::new(1.0, 0.5, 0.3333333333333333)),
            true
        );
    }

    #[test]
    fn inverse_in_place() {
        let mut vec = Vec3::<f64>::new(1.0, 2.0, 3.0);
        vec.inverse_in_place();
        assert_eq!(
            vec.approximate_eq(&Vec3::new(1.0, 0.5, 0.3333333333333333)),
            true
        );
    }

    #[test]
    fn normalize() {
        let vec = Vec3::<f64>::new(5.0, 2.0, 7.0);
        assert_eq!(
            vec.normalize().approximate_eq(&Vec3::new(
                0.5661385170722978,
                0.22645540682891913,
                0.792593923901217
            )),
            true
        );
    }

    #[test]
    fn normalize_in_place() {
        let mut vec0 = Vec3::<f64>::new(5.0, 2.0, 7.0);
        vec0.normalize_in_place();
        assert_eq!(
            vec0.approximate_eq(&Vec3::new(
                0.5661385170722978,
                0.22645540682891913,
                0.792593923901217
            )),
            true
        );
    }

    #[test]
    fn dot() {
        let vec0 = Vec3::<f64>::new(1.0, 2.0, 3.0);
        let vec1 = Vec3::<f64>::new(4.0, 5.0, 6.0);
        assert_eq!(vec0.dot(&vec1).approximate_eq(&32.0), true);
    }

    #[test]
    fn lerp() {
        let vec0 = Vec3::<f64>::new(1.0, 2.0, 3.0);
        let vec1 = Vec3::<f64>::new(4.0, 5.0, 6.0);
        let t = 0.5;
        assert_eq!(
            vec0.lerp(&vec1, t)
                .approximate_eq(&Vec3::new(2.5, 3.5, 4.5)),
            true
        );
    }

    #[test]
    fn slerp() {
        let vec0 = Vec3::<f64>::new(1.0, 0.0, 0.0);
        let vec1 = Vec3::<f64>::new(0.0, 1.0, 0.0);
        assert_eq!(
            vec0.slerp(&vec1, 0.0)
                .approximate_eq(&Vec3::new(1.0, 0.0, 0.0)),
            true
        );
        assert_eq!(
            vec0.slerp(&vec1, 1.0)
                .approximate_eq(&Vec3::new(0.0, 1.0, 0.0)),
            true
        );
        assert_eq!(
            vec0.slerp(&vec1, 0.5).approximate_eq(&Vec3::new(
                0.7071067811865475,
                0.7071067811865475,
                0.0
            )),
            true
        );
    }

    #[test]
    fn rotate_x() {
        let vec0 = Vec3::<f64>::new(0.0, 1.0, 0.0);
        let vec1 = Vec3::<f64>::new(0.0, 0.0, 0.0);
        assert_eq!(
            vec0.rotate_x(&vec1, std::f64::consts::PI)
                .approximate_eq(&Vec3::new(0.0, -1.0, 1.2246467991473532e-16)),
            true
        );

        let vec0 = Vec3::<f64>::new(2.0, 7.0, 0.0);
        let vec1 = Vec3::<f64>::new(2.0, 5.0, 0.0);
        assert_eq!(
            vec0.rotate_x(&vec1, std::f64::consts::PI)
                .approximate_eq(&Vec3::new(2.0, 3.0, 2.4492935982947064e-16)),
            true
        );
    }

    #[test]
    fn rotate_x_in_place() {
        let mut vec0 = Vec3::<f64>::new(0.0, 1.0, 0.0);
        let vec1 = Vec3::<f64>::new(0.0, 0.0, 0.0);
        vec0.rotate_x_in_place(&vec1, std::f64::consts::PI);
        assert_eq!(
            vec0.approximate_eq(&Vec3::new(0.0, -1.0, 1.2246467991473532e-16)),
            true
        );

        let mut vec0 = Vec3::<f64>::new(2.0, 7.0, 0.0);
        let vec1 = Vec3::<f64>::new(2.0, 5.0, 0.0);
        vec0.rotate_x_in_place(&vec1, std::f64::consts::PI);
        assert_eq!(
            vec0.approximate_eq(&Vec3::new(2.0, 3.0, 2.4492935982947064e-16)),
            true
        );
    }

    #[test]
    fn rotate_y() {
        let vec0 = Vec3::<f64>::new(1.0, 0.0, 0.0);
        let vec1 = Vec3::<f64>::new(0.0, 0.0, 0.0);
        assert_eq!(
            vec0.rotate_y(&vec1, std::f64::consts::PI)
                .approximate_eq(&Vec3::new(-1.0, 0.0, -1.2246467991473532e-16)),
            true
        );

        let vec0 = Vec3::<f64>::new(-2.0, 3.0, 10.0);
        let vec1 = Vec3::<f64>::new(-4.0, 3.0, 10.0);
        assert_eq!(
            vec0.rotate_y(&vec1, std::f64::consts::PI)
                .approximate_eq(&Vec3::new(-6.0, 3.0, 10.0)),
            true
        );
    }

    #[test]
    fn rotate_y_in_place() {
        let mut vec0 = Vec3::<f64>::new(1.0, 0.0, 0.0);
        let vec1 = Vec3::<f64>::new(0.0, 0.0, 0.0);
        vec0.rotate_y_in_place(&vec1, std::f64::consts::PI);
        assert_eq!(
            vec0.approximate_eq(&Vec3::new(-1.0, 0.0, -1.2246467991473532e-16)),
            true
        );

        let mut vec0 = Vec3::<f64>::new(-2.0, 3.0, 10.0);
        let vec1 = Vec3::<f64>::new(-4.0, 3.0, 10.0);
        vec0.rotate_y_in_place(&vec1, std::f64::consts::PI);
        assert_eq!(vec0.approximate_eq(&Vec3::new(-6.0, 3.0, 10.0)), true);
    }

    #[test]
    fn rotate_z() {
        let vec0 = Vec3::<f64>::new(0.0, 1.0, 0.0);
        let vec1 = Vec3::<f64>::new(0.0, 0.0, 0.0);
        assert_eq!(
            vec0.rotate_z(&vec1, std::f64::consts::PI)
                .approximate_eq(&Vec3::new(-1.2246467991473532e-16, -1.0, 0.0)),
            true
        );

        let vec0 = Vec3::<f64>::new(0.0, 6.0, -5.0);
        let vec1 = Vec3::<f64>::new(0.0, 0.0, -5.0);
        assert_eq!(
            vec0.rotate_z(&vec1, std::f64::consts::PI)
                .approximate_eq(&Vec3::new(-7.347880794884119e-16, -6.0, -5.0)),
            true
        );
    }

    #[test]
    fn rotate_z_in_place() {
        let mut vec0 = Vec3::<f64>::new(0.0, 1.0, 0.0);
        let vec1 = Vec3::<f64>::new(0.0, 0.0, 0.0);
        vec0.rotate_z_in_place(&vec1, std::f64::consts::PI);
        assert_eq!(
            vec0.approximate_eq(&Vec3::new(-1.2246467991473532e-16, -1.0, 0.0)),
            true
        );

        let mut vec0 = Vec3::<f64>::new(0.0, 6.0, -5.0);
        let vec1 = Vec3::<f64>::new(0.0, 0.0, -5.0);
        vec0.rotate_z_in_place(&vec1, std::f64::consts::PI);
        assert_eq!(
            vec0.approximate_eq(&Vec3::new(-7.347880794884119e-16, -6.0, -5.0)),
            true
        );
    }

    #[test]
    fn angle() {
        let vec0 = Vec3::<f64>::new(1.0, 2.0, 3.0);
        let vec1 = Vec3::<f64>::new(4.0, 5.0, 6.0);
        assert_eq!(vec0.angle(&vec1).approximate_eq(&0.2257261285527342), true);
    }

    #[test]
    fn display() {
        let vec = Vec3::<f64>::new(1.0, 2.0, 3.0);
        assert_eq!(vec.to_string(), "vec3(1, 2, 3)");
    }
}
