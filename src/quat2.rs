use std::{
    fmt::{Debug, Display},
    ops::{Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Sub, SubAssign},
    slice::SliceIndex,
};

use crate::{mat4::Mat4, quat::Quat, vec3::Vec3, ApproximateEq};

pub struct Quat2<T = f64>([T; 8]);

impl<T: Debug> Debug for Quat2<T> {
    #[inline(always)]
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_tuple("Quat2").field(&self.0).finish()
    }
}

impl<T: Copy> Copy for Quat2<T> {}

impl<T: Clone> Clone for Quat2<T> {
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
        ])
    }
}

impl<T: PartialEq> PartialEq for Quat2<T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl<T: Display> Display for Quat2<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_fmt(format_args!(
            "quat2({}, {}, {}, {}, {}, {}, {}, {})",
            self.0[0], self.0[1], self.0[2], self.0[3], self.0[4], self.0[5], self.0[6], self.0[7]
        ))
    }
}

impl<T> Quat2<T> {
    #[inline(always)]
    pub const fn new(x1: T, y1: T, z1: T, w1: T, x2: T, y2: T, z2: T, w2: T) -> Self {
        Self([x1, y1, z1, w1, x2, y2, z2, w2])
    }

    #[inline(always)]
    pub const fn from_slice(values: [T; 8]) -> Self {
        Self(values)
    }
}

impl<T> Quat2<T> {
    #[inline(always)]
    pub fn raw(&self) -> &[T; 8] {
        &self.0
    }

    #[inline(always)]
    pub fn raw_mut(&mut self) -> &mut [T; 8] {
        &mut self.0
    }

    #[inline(always)]
    pub fn x1(&self) -> &T {
        &self.0[0]
    }

    #[inline(always)]
    pub fn y1(&self) -> &T {
        &self.0[1]
    }

    #[inline(always)]
    pub fn z1(&self) -> &T {
        &self.0[2]
    }

    #[inline(always)]
    pub fn w1(&self) -> &T {
        &self.0[3]
    }

    #[inline(always)]
    pub fn x2(&self) -> &T {
        &self.0[4]
    }

    #[inline(always)]
    pub fn y2(&self) -> &T {
        &self.0[5]
    }

    #[inline(always)]
    pub fn z2(&self) -> &T {
        &self.0[6]
    }

    #[inline(always)]
    pub fn w2(&self) -> &T {
        &self.0[7]
    }

    #[inline(always)]
    pub fn x1_mut(&mut self) -> &mut T {
        &mut self.0[0]
    }

    #[inline(always)]
    pub fn y1_mut(&mut self) -> &mut T {
        &mut self.0[1]
    }

    #[inline(always)]
    pub fn z1_mut(&mut self) -> &mut T {
        &mut self.0[2]
    }

    #[inline(always)]
    pub fn w1_mut(&mut self) -> &mut T {
        &mut self.0[3]
    }

    #[inline(always)]
    pub fn x2_mut(&mut self) -> &mut T {
        &mut self.0[4]
    }

    #[inline(always)]
    pub fn y2_mut(&mut self) -> &mut T {
        &mut self.0[5]
    }

    #[inline(always)]
    pub fn z2_mut(&mut self) -> &mut T {
        &mut self.0[6]
    }

    #[inline(always)]
    pub fn w2_mut(&mut self) -> &mut T {
        &mut self.0[7]
    }

    #[inline(always)]
    pub fn set_x1(&mut self, x1: T) {
        self.0[0] = x1;
    }

    #[inline(always)]
    pub fn set_y1(&mut self, y1: T) {
        self.0[1] = y1;
    }

    #[inline(always)]
    pub fn set_z1(&mut self, z1: T) {
        self.0[2] = z1;
    }

    #[inline(always)]
    pub fn set_w1(&mut self, w1: T) {
        self.0[3] = w1;
    }

    #[inline(always)]
    pub fn set_x2(&mut self, x2: T) {
        self.0[4] = x2;
    }

    #[inline(always)]
    pub fn set_y2(&mut self, y2: T) {
        self.0[5] = y2;
    }

    #[inline(always)]
    pub fn set_z2(&mut self, z2: T) {
        self.0[6] = z2;
    }

    #[inline(always)]
    pub fn set_w2(&mut self, w2: T) {
        self.0[7] = w2;
    }

    #[inline(always)]
    pub fn set(&mut self, x1: T, y1: T, z1: T, w1: T, x2: T, y2: T, z2: T, w2: T) {
        self.0[0] = x1;
        self.0[1] = y1;
        self.0[2] = z1;
        self.0[3] = w1;
        self.0[4] = x2;
        self.0[5] = y2;
        self.0[6] = z2;
        self.0[7] = w2;
    }
}

impl<T, I> Index<I> for Quat2<T>
where
    I: SliceIndex<[T], Output = T>,
{
    type Output = T;

    #[inline(always)]
    fn index(&self, index: I) -> &Self::Output {
        self.0.index(index)
    }
}

impl<T, I> IndexMut<I> for Quat2<T>
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
        impl Quat2<$t> {
            #[inline(always)]
            pub const fn new_zero() -> Self {
                Self([$zero, $zero, $zero, $zero, $zero, $zero, $zero, $zero])
            }

            #[inline(always)]
            pub fn new_identity() -> Self {
                Self([$zero, $zero, $zero, $one, $zero, $zero, $zero, $zero])
            }
        }

        #[cfg(feature = "rand")]
        impl rand::distributions::Distribution<Quat2<$t>> for rand::distributions::Standard {
            fn sample<R: rand::prelude::Rng + ?Sized>(&self, rng: &mut R) -> Quat2<$t> {
                Quat2::new(
                    rng.gen::<$t>(),
                    rng.gen::<$t>(),
                    rng.gen::<$t>(),
                    rng.gen::<$t>(),
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
    ($(($t: ident, $zero: expr, $half: expr, $one: expr, $two: expr)),+) => {
       $(
        impl Quat2<$t> {
            #[inline(always)]
            pub fn from_rotation_translation_values(
                x1: $t,
                y1: $t,
                z1: $t,
                w1: $t,
                x2: $t,
                y2: $t,
                z2: $t,
            ) -> Self {
                let ax = x2 * $half;
                let ay = y2 * $half;
                let az = z2 * $half;
                Self::new(
                    x1,
                    y1,
                    z1,
                    w1,
                    ax * w1 + ay * z1 - az * y1,
                    ay * w1 + az * x1 - ax * z1,
                    az * w1 + ax * y1 - ay * x1,
                    -ax * x1 - ay * y1 - az * z1,
                )
            }

            #[inline(always)]
            pub fn from_rotation_translation(q: &Quat::<$t>, t: &Vec3::<$t>) -> Self {
                let ax = t.x() * $half;
                let ay = t.y() * $half;
                let az = t.z() * $half;
                let bx = *q.x();
                let by = *q.y();
                let bz = *q.z();
                let bw = *q.w();
                Self::new(
                    bx,
                    by,
                    bz,
                    bw,
                    ax * bw + ay * bz - az * by,
                    ay * bw + az * bx - ax * bz,
                    az * bw + ax * by - ay * bx,
                    -ax * bx - ay * by - az * bz,
                )
            }

            #[inline(always)]
            pub fn from_translation(t: &Vec3::<$t>) -> Self {
                Self([
                    $zero,
                    $zero,
                    $zero,
                    $one,
                    t.x() * $half,
                    t.y() * $half,
                    t.z() * $half,
                    $zero,
                ])
            }

            #[inline(always)]
            pub fn from_rotation(q: &Quat::<$t>) -> Self {
                Self([
                    *q.x(),
                    *q.y(),
                    *q.z(),
                    *q.w(),
                    $zero,
                    $zero,
                    $zero,
                    $zero,
                ])
            }

            #[inline(always)]
            pub fn from_mat4(a: &Mat4::<$t>) -> Self {
                let outer = a.rotation();
                let t = a.translation();
                Self::from_rotation_translation(&outer, &t)
            }
        }
       )+
    };
}

macro_rules! math {
    ($(($t: ident, $epsilon: expr, $zero: expr, $half: expr, $one: expr, $two: expr)),+) => {
       $(
        impl Add for Quat2<$t> {
            type Output = Quat2<$t>;

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
                self
            }
        }

        impl Add<$t> for Quat2<$t> {
            type Output = Quat2<$t>;

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
                self
            }
        }

        impl Add<Quat2<$t>> for $t {
            type Output = Quat2<$t>;

            #[inline(always)]
            fn add(self, mut rhs: Quat2::<$t>) -> Self::Output {
                rhs.0[0] = self + rhs.0[0];
                rhs.0[1] = self + rhs.0[1];
                rhs.0[2] = self + rhs.0[2];
                rhs.0[3] = self + rhs.0[3];
                rhs.0[4] = self + rhs.0[4];
                rhs.0[5] = self + rhs.0[5];
                rhs.0[6] = self + rhs.0[6];
                rhs.0[7] = self + rhs.0[7];
                rhs
            }
        }

        impl AddAssign for Quat2<$t> {
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
            }
        }

        impl AddAssign<$t> for Quat2<$t> {
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
            }
        }

        impl Sub for Quat2<$t> {
            type Output = Quat2<$t>;

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
                self
            }
        }

        impl Sub<$t> for Quat2<$t> {
            type Output = Quat2<$t>;

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
                self
            }
        }

        impl Sub<Quat2<$t>> for $t {
            type Output = Quat2<$t>;

            #[inline(always)]
            fn sub(self, mut rhs: Quat2<$t>) -> Self::Output {
                rhs.0[0] = self - rhs.0[0];
                rhs.0[1] = self - rhs.0[1];
                rhs.0[2] = self - rhs.0[2];
                rhs.0[3] = self - rhs.0[3];
                rhs.0[4] = self - rhs.0[4];
                rhs.0[5] = self - rhs.0[5];
                rhs.0[6] = self - rhs.0[6];
                rhs.0[7] = self - rhs.0[7];
                rhs
            }
        }

        impl SubAssign for Quat2<$t> {
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
            }
        }

        impl SubAssign<$t> for Quat2<$t> {
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
            }
        }

        impl Mul for Quat2<$t> {
            type Output = Quat2<$t>;

            #[inline(always)]
            fn mul(mut self, rhs: Self) -> Self::Output {
                let ax0 = self.0[0];
                let ay0 = self.0[1];
                let az0 = self.0[2];
                let aw0 = self.0[3];
                let bx1 = rhs.0[4];
                let by1 = rhs.0[5];
                let bz1 = rhs.0[6];
                let bw1 = rhs.0[7];
                let ax1 = self.0[4];
                let ay1 = self.0[5];
                let az1 = self.0[6];
                let aw1 = self.0[7];
                let bx0 = rhs.0[0];
                let by0 = rhs.0[1];
                let bz0 = rhs.0[2];
                let bw0 = rhs.0[3];

                self.0[0] = ax0 * bw0 + aw0 * bx0 + ay0 * bz0 - az0 * by0;
                self.0[1] = ay0 * bw0 + aw0 * by0 + az0 * bx0 - ax0 * bz0;
                self.0[2] = az0 * bw0 + aw0 * bz0 + ax0 * by0 - ay0 * bx0;
                self.0[3] = aw0 * bw0 - ax0 * bx0 - ay0 * by0 - az0 * bz0;
                self.0[4] = ax0 * bw1 + aw0 * bx1 + ay0 * bz1 - az0 * by1 + ax1 * bw0 + aw1 * bx0 + ay1 * bz0
                        - az1 * by0;
                self.0[5] = ay0 * bw1 + aw0 * by1 + az0 * bx1 - ax0 * bz1 + ay1 * bw0 + aw1 * by0 + az1 * bx0
                        - ax1 * bz0;
                self.0[6] = az0 * bw1 + aw0 * bz1 + ax0 * by1 - ay0 * bx1 + az1 * bw0 + aw1 * bz0 + ax1 * by0
                        - ay1 * bx0;
                self.0[7] = aw0 * bw1 - ax0 * bx1 - ay0 * by1 - az0 * bz1 + aw1 * bw0
                        - ax1 * bx0
                        - ay1 * by0
                        - az1 * bz0;
                self
            }
        }

        impl Mul<$t> for Quat2<$t> {
            type Output = Quat2<$t>;

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
                self
            }
        }

        impl Mul<Quat2<$t>> for $t {
            type Output = Quat2<$t>;

            #[inline(always)]
            fn mul(self, mut rhs: Quat2<$t>) -> Self::Output {
                rhs.0[0] = self * rhs.0[0];
                rhs.0[1] = self * rhs.0[1];
                rhs.0[2] = self * rhs.0[2];
                rhs.0[3] = self * rhs.0[3];
                rhs.0[4] = self * rhs.0[4];
                rhs.0[5] = self * rhs.0[5];
                rhs.0[6] = self * rhs.0[6];
                rhs.0[7] = self * rhs.0[7];
                rhs
            }
        }

        impl MulAssign for Quat2<$t> {
            #[inline(always)]
            fn mul_assign(&mut self, rhs: Self) {
                let ax0 = self.0[0];
                let ay0 = self.0[1];
                let az0 = self.0[2];
                let aw0 = self.0[3];
                let bx1 = rhs.0[4];
                let by1 = rhs.0[5];
                let bz1 = rhs.0[6];
                let bw1 = rhs.0[7];
                let ax1 = self.0[4];
                let ay1 = self.0[5];
                let az1 = self.0[6];
                let aw1 = self.0[7];
                let bx0 = rhs.0[0];
                let by0 = rhs.0[1];
                let bz0 = rhs.0[2];
                let bw0 = rhs.0[3];

                self.0[0] = ax0 * bw0 + aw0 * bx0 + ay0 * bz0 - az0 * by0;
                self.0[1] = ay0 * bw0 + aw0 * by0 + az0 * bx0 - ax0 * bz0;
                self.0[2] = az0 * bw0 + aw0 * bz0 + ax0 * by0 - ay0 * bx0;
                self.0[3] = aw0 * bw0 - ax0 * bx0 - ay0 * by0 - az0 * bz0;
                self.0[4] = ax0 * bw1 + aw0 * bx1 + ay0 * bz1 - az0 * by1 + ax1 * bw0 + aw1 * bx0 + ay1 * bz0
                        - az1 * by0;
                self.0[5] = ay0 * bw1 + aw0 * by1 + az0 * bx1 - ax0 * bz1 + ay1 * bw0 + aw1 * by0 + az1 * bx0
                        - ax1 * bz0;
                self.0[6] = az0 * bw1 + aw0 * bz1 + ax0 * by1 - ay0 * bx1 + az1 * bw0 + aw1 * bz0 + ax1 * by0
                        - ay1 * bx0;
                self.0[7] = aw0 * bw1 - ax0 * bx1 - ay0 * by1 - az0 * bz1 + aw1 * bw0
                        - ax1 * bx0
                        - ay1 * by0
                        - az1 * bz0;
            }
        }

        impl MulAssign<$t> for Quat2<$t> {
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
            }
        }

        impl Div<$t> for Quat2<$t> {
            type Output = Quat2<$t>;

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
                self
            }
        }

        impl Div<Quat2<$t>> for $t {
            type Output = Quat2<$t>;

            #[inline(always)]
            fn div(self, mut rhs: Quat2<$t>) -> Self::Output {
                rhs.0[0] = self / rhs.0[0];
                rhs.0[1] = self / rhs.0[1];
                rhs.0[2] = self / rhs.0[2];
                rhs.0[3] = self / rhs.0[3];
                rhs.0[4] = self / rhs.0[4];
                rhs.0[5] = self / rhs.0[5];
                rhs.0[6] = self / rhs.0[6];
                rhs.0[7] = self / rhs.0[7];
                rhs
            }
        }

        impl DivAssign<$t> for Quat2<$t> {
            #[inline(always)]
            fn div_assign(&mut self, rhs: $t) {
                self.0[0] /= rhs;
                self.0[1] /= rhs;
                self.0[2] /= rhs;
                self.0[3] /= rhs;
                self.0[4] /= rhs;
                self.0[5] /= rhs;
                self.0[6] /= rhs;
                self.0[7] /= rhs;
            }
        }

        impl Quat2<$t> {
            #[inline(always)]
            pub fn translation(&self) -> Vec3<$t> {
                let bx = -self.0[0];
                let by = -self.0[1];
                let bz = -self.0[2];
                let bw = self.0[3];
                let ax = self.0[4];
                let ay = self.0[5];
                let az = self.0[6];
                let aw = self.0[7];
                Vec3::<$t>::new(
                    (ax * bw + aw * bx + ay * bz - az * by) * $two,
                    (ay * bw + aw * by + az * bx - ax * bz) * $two,
                    (az * bw + aw * bz + ax * by - ay * bx) * $two,
                )
            }

            #[inline(always)]
            pub fn translate(&self, v: &Vec3::<$t>) -> Self {
                let ax1 = self.0[0];
                let ay1 = self.0[1];
                let az1 = self.0[2];
                let aw1 = self.0[3];
                let bx1 = *v.x() * $half;
                let by1 = *v.y() * $half;
                let bz1 = *v.z() * $half;
                let ax2 = self.0[4];
                let ay2 = self.0[5];
                let az2 = self.0[6];
                let aw2 = self.0[7];

                Self::new(
                    ax1,
                    ay1,
                    az1,
                    aw1,
                    aw1 * bx1 + ay1 * bz1 - az1 * by1 + ax2,
                    aw1 * by1 + az1 * bx1 - ax1 * bz1 + ay2,
                    aw1 * bz1 + ax1 * by1 - ay1 * bx1 + az2,
                    -ax1 * bx1 - ay1 * by1 - az1 * bz1 + aw2,
                )
            }
            #[inline(always)]
            pub fn translate_in_place(&mut self, v: &Vec3::<$t>) -> &mut Self {
                let ax1 = self.0[0];
                let ay1 = self.0[1];
                let az1 = self.0[2];
                let aw1 = self.0[3];
                let bx1 = *v.x() * $half;
                let by1 = *v.y() * $half;
                let bz1 = *v.z() * $half;
                let ax2 = self.0[4];
                let ay2 = self.0[5];
                let az2 = self.0[6];
                let aw2 = self.0[7];

                self.0[0] = ax1;
                self.0[1] = ay1;
                self.0[2] = az1;
                self.0[3] = aw1;
                self.0[4] = aw1 * bx1 + ay1 * bz1 - az1 * by1 + ax2;
                self.0[5] = aw1 * by1 + az1 * bx1 - ax1 * bz1 + ay2;
                self.0[6] = aw1 * bz1 + ax1 * by1 - ay1 * bx1 + az2;
                self.0[7] = -ax1 * bx1 - ay1 * by1 - az1 * bz1 + aw2;
                self
            }

            #[inline(always)]
            pub fn rotate_x(&self, rad: $t) -> Self {
                let bx = -self.0[0];
                let by = -self.0[1];
                let bz = -self.0[2];
                let bw = self.0[3];
                let ax = self.0[4];
                let ay = self.0[5];
                let az = self.0[6];
                let aw = self.0[7];
                let ax1 = ax * bw + aw * bx + ay * bz - az * by;
                let ay1 = ay * bw + aw * by + az * bx - ax * bz;
                let az1 = az * bw + aw * bz + ax * by - ay * bx;
                let aw1 = aw * bw - ax * bx - ay * by - az * bz;
                let quat = Quat::<$t>::new(self.0[0], self.0[1], self.0[2], self.0[3]).rotate_x(rad);
                let bx = *quat.x();
                let by = *quat.y();
                let bz = *quat.z();
                let bw = *quat.w();

                Self::new(
                    bx,
                    by,
                    bz,
                    bw,
                    ax1 * bw + aw1 * bx + ay1 * bz - az1 * by,
                    ay1 * bw + aw1 * by + az1 * bx - ax1 * bz,
                    az1 * bw + aw1 * bz + ax1 * by - ay1 * bx,
                    aw1 * bw - ax1 * bx - ay1 * by - az1 * bz,
                )
            }

            #[inline(always)]
            pub fn rotate_x_in_place(&mut self, rad: $t) -> &mut Self {
                let bx = -self.0[0];
                let by = -self.0[1];
                let bz = -self.0[2];
                let bw = self.0[3];
                let ax = self.0[4];
                let ay = self.0[5];
                let az = self.0[6];
                let aw = self.0[7];
                let ax1 = ax * bw + aw * bx + ay * bz - az * by;
                let ay1 = ay * bw + aw * by + az * bx - ax * bz;
                let az1 = az * bw + aw * bz + ax * by - ay * bx;
                let aw1 = aw * bw - ax * bx - ay * by - az * bz;
                let quat = Quat::<$t>::new(self.0[0], self.0[1], self.0[2], self.0[3]).rotate_x(rad);
                let bx = *quat.x();
                let by = *quat.y();
                let bz = *quat.z();
                let bw = *quat.w();

                self.0[0] = bx;
                self.0[1] = by;
                self.0[2] = bz;
                self.0[3] = bw;
                self.0[4] = ax1 * bw + aw1 * bx + ay1 * bz - az1 * by;
                self.0[5] = ay1 * bw + aw1 * by + az1 * bx - ax1 * bz;
                self.0[6] = az1 * bw + aw1 * bz + ax1 * by - ay1 * bx;
                self.0[7] = aw1 * bw - ax1 * bx - ay1 * by - az1 * bz;
                self
            }

            #[inline(always)]
            pub fn rotate_y(&self, rad: $t) -> Self {
                let bx = -self.0[0];
                let by = -self.0[1];
                let bz = -self.0[2];
                let bw = self.0[3];
                let ax = self.0[4];
                let ay = self.0[5];
                let az = self.0[6];
                let aw = self.0[7];
                let ax1 = ax * bw + aw * bx + ay * bz - az * by;
                let ay1 = ay * bw + aw * by + az * bx - ax * bz;
                let az1 = az * bw + aw * bz + ax * by - ay * bx;
                let aw1 = aw * bw - ax * bx - ay * by - az * bz;
                let quat = Quat::<$t>::new(self.0[0], self.0[1], self.0[2], self.0[3]).rotate_y(rad);
                let bx = *quat.x();
                let by = *quat.y();
                let bz = *quat.z();
                let bw = *quat.w();

                Self::new(
                    bx,
                    by,
                    bz,
                    bw,
                    ax1 * bw + aw1 * bx + ay1 * bz - az1 * by,
                    ay1 * bw + aw1 * by + az1 * bx - ax1 * bz,
                    az1 * bw + aw1 * bz + ax1 * by - ay1 * bx,
                    aw1 * bw - ax1 * bx - ay1 * by - az1 * bz,
                )
            }

            #[inline(always)]
            pub fn rotate_y_in_place(&mut self, rad: $t) -> &mut Self {
                let bx = -self.0[0];
                let by = -self.0[1];
                let bz = -self.0[2];
                let bw = self.0[3];
                let ax = self.0[4];
                let ay = self.0[5];
                let az = self.0[6];
                let aw = self.0[7];
                let ax1 = ax * bw + aw * bx + ay * bz - az * by;
                let ay1 = ay * bw + aw * by + az * bx - ax * bz;
                let az1 = az * bw + aw * bz + ax * by - ay * bx;
                let aw1 = aw * bw - ax * bx - ay * by - az * bz;
                let quat = Quat::<$t>::new(self.0[0], self.0[1], self.0[2], self.0[3]).rotate_y(rad);
                let bx = *quat.x();
                let by = *quat.y();
                let bz = *quat.z();
                let bw = *quat.w();

                self.0[0] = bx;
                self.0[1] = by;
                self.0[2] = bz;
                self.0[3] = bw;
                self.0[4] = ax1 * bw + aw1 * bx + ay1 * bz - az1 * by;
                self.0[5] = ay1 * bw + aw1 * by + az1 * bx - ax1 * bz;
                self.0[6] = az1 * bw + aw1 * bz + ax1 * by - ay1 * bx;
                self.0[7] = aw1 * bw - ax1 * bx - ay1 * by - az1 * bz;
                self
            }

            #[inline(always)]
            pub fn rotate_z(&self, rad: $t) -> Self {
                let bx = -self.0[0];
                let by = -self.0[1];
                let bz = -self.0[2];
                let bw = self.0[3];
                let ax = self.0[4];
                let ay = self.0[5];
                let az = self.0[6];
                let aw = self.0[7];
                let ax1 = ax * bw + aw * bx + ay * bz - az * by;
                let ay1 = ay * bw + aw * by + az * bx - ax * bz;
                let az1 = az * bw + aw * bz + ax * by - ay * bx;
                let aw1 = aw * bw - ax * bx - ay * by - az * bz;
                let quat = Quat::<$t>::new(self.0[0], self.0[1], self.0[2], self.0[3]).rotate_z(rad);
                let bx = *quat.x();
                let by = *quat.y();
                let bz = *quat.z();
                let bw = *quat.w();

                Self::new(
                    bx,
                    by,
                    bz,
                    bw,
                    ax1 * bw + aw1 * bx + ay1 * bz - az1 * by,
                    ay1 * bw + aw1 * by + az1 * bx - ax1 * bz,
                    az1 * bw + aw1 * bz + ax1 * by - ay1 * bx,
                    aw1 * bw - ax1 * bx - ay1 * by - az1 * bz,
                )
            }

            #[inline(always)]
            pub fn rotate_z_in_place(&mut self, rad: $t) -> &mut Self {
                let bx = -self.0[0];
                let by = -self.0[1];
                let bz = -self.0[2];
                let bw = self.0[3];
                let ax = self.0[4];
                let ay = self.0[5];
                let az = self.0[6];
                let aw = self.0[7];
                let ax1 = ax * bw + aw * bx + ay * bz - az * by;
                let ay1 = ay * bw + aw * by + az * bx - ax * bz;
                let az1 = az * bw + aw * bz + ax * by - ay * bx;
                let aw1 = aw * bw - ax * bx - ay * by - az * bz;
                let quat = Quat::<$t>::new(self.0[0], self.0[1], self.0[2], self.0[3]).rotate_z(rad);
                let bx = *quat.x();
                let by = *quat.y();
                let bz = *quat.z();
                let bw = *quat.w();

                self.0[0] = bx;
                self.0[1] = by;
                self.0[2] = bz;
                self.0[3] = bw;
                self.0[4] = ax1 * bw + aw1 * bx + ay1 * bz - az1 * by;
                self.0[5] = ay1 * bw + aw1 * by + az1 * bx - ax1 * bz;
                self.0[6] = az1 * bw + aw1 * bz + ax1 * by - ay1 * bx;
                self.0[7] = aw1 * bw - ax1 * bx - ay1 * by - az1 * bz;
                self
            }

            #[inline(always)]
            pub fn rotate_by_quat_append(&self, q: &Quat::<$t>) -> Self {
                let qx = *q.x();
                let qy = *q.y();
                let qz = *q.z();
                let qw = *q.w();
                let mut ax = self.0[0];
                let mut ay = self.0[1];
                let mut az = self.0[2];
                let mut aw = self.0[3];
                let x1 = ax * qw + aw * qx + ay * qz - az * qy;
                let y1 = ay * qw + aw * qy + az * qx - ax * qz;
                let z1 = az * qw + aw * qz + ax * qy - ay * qx;
                let w1 = aw * qw - ax * qx - ay * qy - az * qz;

                ax = self.0[4];
                ay = self.0[5];
                az = self.0[6];
                aw = self.0[7];
                let x2 = ax * qw + aw * qx + ay * qz - az * qy;
                let y2 = ay * qw + aw * qy + az * qx - ax * qz;
                let z2 = az * qw + aw * qz + ax * qy - ay * qx;
                let w2 = aw * qw - ax * qx - ay * qy - az * qz;

                Self::new(x1, y1, z1, w1, x2, y2, z2, w2)
            }

            #[inline(always)]
            pub fn rotate_by_quat_append_in_place(&mut self, q: &Quat::<$t>) -> &mut Self {
                let qx = *q.x();
                let qy = *q.y();
                let qz = *q.z();
                let qw = *q.w();
                let mut ax = self.0[0];
                let mut ay = self.0[1];
                let mut az = self.0[2];
                let mut aw = self.0[3];
                let x1 = ax * qw + aw * qx + ay * qz - az * qy;
                let y1 = ay * qw + aw * qy + az * qx - ax * qz;
                let z1 = az * qw + aw * qz + ax * qy - ay * qx;
                let w1 = aw * qw - ax * qx - ay * qy - az * qz;

                ax = self.0[4];
                ay = self.0[5];
                az = self.0[6];
                aw = self.0[7];
                let x2 = ax * qw + aw * qx + ay * qz - az * qy;
                let y2 = ay * qw + aw * qy + az * qx - ax * qz;
                let z2 = az * qw + aw * qz + ax * qy - ay * qx;
                let w2 = aw * qw - ax * qx - ay * qy - az * qz;

                self.0[0] = x1;
                self.0[1] = y1;
                self.0[2] = z1;
                self.0[3] = w1;
                self.0[4] = x2;
                self.0[5] = y2;
                self.0[6] = z2;
                self.0[7] = w2;
                self
            }

            #[inline(always)]
            pub fn rotate_by_quat_prepend(&self, q: &Quat::<$t>) -> Self {
                let qx = *q.x();
                let qy = *q.y();
                let qz = *q.z();
                let qw = *q.w();
                let mut bx = self.0[0];
                let mut by = self.0[1];
                let mut bz = self.0[2];
                let mut bw = self.0[3];
                let x1 = qx * bw + qw * bx + qy * bz - qz * by;
                let y1 = qy * bw + qw * by + qz * bx - qx * bz;
                let z1 = qz * bw + qw * bz + qx * by - qy * bx;
                let w1 = qw * bw - qx * bx - qy * by - qz * bz;

                bx = self.0[4];
                by = self.0[5];
                bz = self.0[6];
                bw = self.0[7];
                let x2 = qx * bw + qw * bx + qy * bz - qz * by;
                let y2 = qy * bw + qw * by + qz * bx - qx * bz;
                let z2 = qz * bw + qw * bz + qx * by - qy * bx;
                let w2 = qw * bw - qx * bx - qy * by - qz * bz;

                Self::new(x1, y1, z1, w1, x2, y2, z2, w2)
            }

            #[inline(always)]
            pub fn rotate_by_quat_prepend_in_place(&mut self, q: &Quat::<$t>) -> &mut Self {
                let qx = *q.x();
                let qy = *q.y();
                let qz = *q.z();
                let qw = *q.w();
                let mut bx = self.0[0];
                let mut by = self.0[1];
                let mut bz = self.0[2];
                let mut bw = self.0[3];
                let x1 = qx * bw + qw * bx + qy * bz - qz * by;
                let y1 = qy * bw + qw * by + qz * bx - qx * bz;
                let z1 = qz * bw + qw * bz + qx * by - qy * bx;
                let w1 = qw * bw - qx * bx - qy * by - qz * bz;

                bx = self.0[4];
                by = self.0[5];
                bz = self.0[6];
                bw = self.0[7];
                let x2 = qx * bw + qw * bx + qy * bz - qz * by;
                let y2 = qy * bw + qw * by + qz * bx - qx * bz;
                let z2 = qz * bw + qw * bz + qx * by - qy * bx;
                let w2 = qw * bw - qx * bx - qy * by - qz * bz;

                self.0[0] = x1;
                self.0[1] = y1;
                self.0[2] = z1;
                self.0[3] = w1;
                self.0[4] = x2;
                self.0[5] = y2;
                self.0[6] = z2;
                self.0[7] = w2;
                self
            }

            #[inline(always)]
            pub fn rotate_around_axis(&self, axis: &Vec3::<$t>, rad: $t) -> Self {
                if rad.abs() < $epsilon {
                    return Self::new(
                        self.0[0],
                        self.0[1],
                        self.0[2],
                        self.0[3],
                        self.0[4],
                        self.0[5],
                        self.0[6],
                        self.0[7],
                    );
                }

                let axis_length = (axis.x() * axis.x() + axis.y() * axis.y() + axis.z() * axis.z()).sqrt();
                let rad = rad * $half;
                let s = rad.sin();
                let bx = (s * axis.x()) / axis_length;
                let by = (s * axis.y()) / axis_length;
                let bz = (s * axis.z()) / axis_length;
                let bw = rad.cos();

                let ax1 = self.0[0];
                let ay1 = self.0[1];
                let az1 = self.0[2];
                let aw1 = self.0[3];
                let x1 = ax1 * bw + aw1 * bx + ay1 * bz - az1 * by;
                let y1 = ay1 * bw + aw1 * by + az1 * bx - ax1 * bz;
                let z1 = az1 * bw + aw1 * bz + ax1 * by - ay1 * bx;
                let w1 = aw1 * bw - ax1 * bx - ay1 * by - az1 * bz;

                let ax = self.0[4];
                let ay = self.0[5];
                let az = self.0[6];
                let aw = self.0[7];
                let x2 = ax * bw + aw * bx + ay * bz - az * by;
                let y2 = ay * bw + aw * by + az * bx - ax * bz;
                let z2 = az * bw + aw * bz + ax * by - ay * bx;
                let w2 = aw * bw - ax * bx - ay * by - az * bz;

                Self::new(x1, y1, z1, w1, x2, y2, z2, w2)
            }

            #[inline(always)]
            pub fn rotate_around_axis_in_place(&mut self, axis: &Vec3::<$t>, rad: $t) -> &mut Self {
                if rad.abs() < $epsilon {
                    return self;
                }

                let axis_length = (axis.x() * axis.x() + axis.y() * axis.y() + axis.z() * axis.z()).sqrt();
                let rad = rad * $half;
                let s = rad.sin();
                let bx = (s * axis.x()) / axis_length;
                let by = (s * axis.y()) / axis_length;
                let bz = (s * axis.z()) / axis_length;
                let bw = rad.cos();

                let ax1 = self.0[0];
                let ay1 = self.0[1];
                let az1 = self.0[2];
                let aw1 = self.0[3];
                let x1 = ax1 * bw + aw1 * bx + ay1 * bz - az1 * by;
                let y1 = ay1 * bw + aw1 * by + az1 * bx - ax1 * bz;
                let z1 = az1 * bw + aw1 * bz + ax1 * by - ay1 * bx;
                let w1 = aw1 * bw - ax1 * bx - ay1 * by - az1 * bz;

                let ax = self.0[4];
                let ay = self.0[5];
                let az = self.0[6];
                let aw = self.0[7];
                let x2 = ax * bw + aw * bx + ay * bz - az * by;
                let y2 = ay * bw + aw * by + az * bx - ax * bz;
                let z2 = az * bw + aw * bz + ax * by - ay * bx;
                let w2 = aw * bw - ax * bx - ay * by - az * bz;

                self.0[0] = x1;
                self.0[1] = y1;
                self.0[2] = z1;
                self.0[3] = w1;
                self.0[4] = x2;
                self.0[5] = y2;
                self.0[6] = z2;
                self.0[7] = w2;
                self
            }

            #[inline(always)]
            pub fn scale(&self, scale: $t) -> Self {
                let x1 = self.0[0];
                let y1 = self.0[1];
                let z1 = self.0[2];
                let w1 = self.0[3];
                let x2 = self.0[4];
                let y2 = self.0[5];
                let z2 = self.0[6];
                let w2 = self.0[7];

                Self::new(
                    x1 * scale,
                    y1 * scale,
                    z1 * scale,
                    w1 * scale,
                    x2 * scale,
                    y2 * scale,
                    z2 * scale,
                    w2 * scale,
                )
            }

            #[inline(always)]
            pub fn scale_in_place(&mut self, scale: $t) -> &mut Self {
                let x1 = self.0[0];
                let y1 = self.0[1];
                let z1 = self.0[2];
                let w1 = self.0[3];
                let x2 = self.0[4];
                let y2 = self.0[5];
                let z2 = self.0[6];
                let w2 = self.0[7];

                self.0[0] = x1 * scale;
                self.0[1] = y1 * scale;
                self.0[2] = z1 * scale;
                self.0[3] = w1 * scale;
                self.0[4] = x2 * scale;
                self.0[5] = y2 * scale;
                self.0[6] = z2 * scale;
                self.0[7] = w2 * scale;
                self
            }

            #[inline(always)]
            pub fn dot(&self, b: &Self) -> $t {
                self.0[0] * b.x1() + self.0[1] * b.y1() + self.0[2] * b.z1() + self.0[3] * b.w1()
            }

            #[inline(always)]
            pub fn lerp(&self, b: &Self, t: $t) -> Self {
                let mt = $one - t;
                let t = if self.dot(b) < $zero { -t } else { t };

                let bx = self.0[0];
                let by = self.0[1];
                let bz = self.0[2];
                let bw = self.0[3];
                let ax = self.0[4];
                let ay = self.0[5];
                let az = self.0[6];
                let aw = self.0[7];

                Self::new(
                    bx * mt + b.0[0] * t,
                    by * mt + b.0[1] * t,
                    bz * mt + b.0[2] * t,
                    bw * mt + b.0[3] * t,
                    ax * mt + b.0[4] * t,
                    ay * mt + b.0[5] * t,
                    az * mt + b.0[6] * t,
                    aw * mt + b.0[7] * t,
                )
            }

            #[inline(always)]
            pub fn squared_length(&self) -> $t {
                let x = self.0[0];
                let y = self.0[1];
                let z = self.0[2];
                let w = self.0[3];
                x * x + y * y + z * z + w * w
            }

            #[inline(always)]
            pub fn length(&self) -> $t {
                self.squared_length().sqrt()
            }

            #[inline(always)]
            pub fn normalize(&self) -> Self {
                let mut magnitude = self.squared_length();
                if magnitude > $zero {
                    magnitude = magnitude.sqrt();
                }

                let a0 = self.0[0] / magnitude;
                let a1 = self.0[1] / magnitude;
                let a2 = self.0[2] / magnitude;
                let a3 = self.0[3] / magnitude;

                let b0 = self.0[4];
                let b1 = self.0[5];
                let b2 = self.0[6];
                let b3 = self.0[7];

                let a_dot_b = a0 * b0 + a1 * b1 + a2 * b2 + a3 * b3;

                Self::new(
                    a0,
                    a1,
                    a2,
                    a3,
                    (b0 - a0 * a_dot_b) / magnitude,
                    (b1 - a1 * a_dot_b) / magnitude,
                    (b2 - a2 * a_dot_b) / magnitude,
                    (b3 - a3 * a_dot_b) / magnitude,
                )
            }

            #[inline(always)]
            pub fn normalize_in_place(&mut self) -> &mut Self {
                let mut magnitude = self.squared_length();
                if magnitude > $zero {
                    magnitude = magnitude.sqrt();
                }

                let a0 = self.0[0] / magnitude;
                let a1 = self.0[1] / magnitude;
                let a2 = self.0[2] / magnitude;
                let a3 = self.0[3] / magnitude;

                let b0 = self.0[4];
                let b1 = self.0[5];
                let b2 = self.0[6];
                let b3 = self.0[7];

                let a_dot_b = a0 * b0 + a1 * b1 + a2 * b2 + a3 * b3;

                self.0[0] = a0;
                self.0[1] = a1;
                self.0[2] = a2;
                self.0[3] = a3;
                self.0[4] = (b0 - a0 * a_dot_b) / magnitude;
                self.0[5] = (b1 - a1 * a_dot_b) / magnitude;
                self.0[6] = (b2 - a2 * a_dot_b) / magnitude;
                self.0[7] = (b3 - a3 * a_dot_b) / magnitude;
                self
            }

            #[inline(always)]
            pub fn invert(&self) -> Self {
                let sqlen = self.squared_length();

                let bx = self.0[0];
                let by = self.0[1];
                let bz = self.0[2];
                let bw = self.0[3];
                let ax = self.0[4];
                let ay = self.0[5];
                let az = self.0[6];
                let aw = self.0[7];

                Self::new(
                    -bx / sqlen,
                    -by / sqlen,
                    -bz / sqlen,
                    bw / sqlen,
                    -ax / sqlen,
                    -ay / sqlen,
                    -az / sqlen,
                    aw / sqlen,
                )
            }

            #[inline(always)]
            pub fn invert_in_place(&mut self) -> &mut Self {
                let sqlen = self.squared_length();

                let bx = self.0[0];
                let by = self.0[1];
                let bz = self.0[2];
                let bw = self.0[3];
                let ax = self.0[4];
                let ay = self.0[5];
                let az = self.0[6];
                let aw = self.0[7];

                self.0[0] = -bx / sqlen;
                self.0[1] = -by / sqlen;
                self.0[2] = -bz / sqlen;
                self.0[3] = bw / sqlen;
                self.0[4] = -ax / sqlen;
                self.0[5] = -ay / sqlen;
                self.0[6] = -az / sqlen;
                self.0[7] = aw / sqlen;
                self
            }

            #[inline(always)]
            pub fn conjugate(&self) -> Self {
                let bx = self.0[0];
                let by = self.0[1];
                let bz = self.0[2];
                let bw = self.0[3];
                let ax = self.0[4];
                let ay = self.0[5];
                let az = self.0[6];
                let aw = self.0[7];

                Self::new(-bx, -by, -bz, bw, -ax, -ay, -az, aw)
            }

            #[inline(always)]
            pub fn conjugate_in_place(&mut self) -> &mut Self {
                let bx = self.0[0];
                let by = self.0[1];
                let bz = self.0[2];
                let bw = self.0[3];
                let ax = self.0[4];
                let ay = self.0[5];
                let az = self.0[6];
                let aw = self.0[7];

                self.0[0] = -bx;
                self.0[1] = -by;
                self.0[2] = -bz;
                self.0[3] = bw;
                self.0[4] = -ax;
                self.0[5] = -ay;
                self.0[6] = -az;
                self.0[7] = aw;
                self
            }

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
                self
            }

            #[inline(always)]
            pub fn set_identify(&mut self) -> &mut Self {
                self.0[0] = $zero;
                self.0[1] = $zero;
                self.0[2] = $zero;
                self.0[3] = $one;
                self.0[4] = $zero;
                self.0[5] = $zero;
                self.0[6] = $zero;
                self.0[7] = $zero;
                self
            }

            #[inline(always)]
            pub fn real(&self) -> Quat::<$t> {
                Quat::<$t>::new(
                    self.0[0],
                    self.0[1],
                    self.0[2],
                    self.0[3],
                )
            }

            #[inline(always)]
            pub fn dual(&self) -> Quat::<$t> {
                Quat::<$t>::new(
                    self.0[4],
                    self.0[5],
                    self.0[6],
                    self.0[7],
                )
            }

            #[inline(always)]
            pub fn set_real(&mut self, q: &Quat::<$t>) -> &mut Self {
                self.0[0] = *q.x();
                self.0[1] = *q.y();
                self.0[2] = *q.z();
                self.0[3] = *q.w();
                self
            }

            #[inline(always)]
            pub fn set_dual(&mut self, q: &Quat::<$t>) -> &mut Self {
                self.0[4] = *q.x();
                self.0[5] = *q.y();
                self.0[6] = *q.z();
                self.0[7] = *q.w();
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
                self
            }
        }

        impl ApproximateEq for Quat2<$t> {
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
                let b0 = other.0[0];
                let b1 = other.0[1];
                let b2 = other.0[2];
                let b3 = other.0[3];
                let b4 = other.0[4];
                let b5 = other.0[5];
                let b6 = other.0[6];
                let b7 = other.0[7];

                (a0 - b0).abs() <= $epsilon * $one.max(a0.abs()).max(b0.abs())
                    && (a1 - b1).abs() <= $epsilon * $one.max(a1.abs()).max(b1.abs())
                    && (a2 - b2).abs() <= $epsilon * $one.max(a2.abs()).max(b2.abs())
                    && (a3 - b3).abs() <= $epsilon * $one.max(a3.abs()).max(b3.abs())
                    && (a4 - b4).abs() <= $epsilon * $one.max(a4.abs()).max(b4.abs())
                    && (a5 - b5).abs() <= $epsilon * $one.max(a5.abs()).max(b5.abs())
                    && (a6 - b6).abs() <= $epsilon * $one.max(a6.abs()).max(b6.abs())
                    && (a7 - b7).abs() <= $epsilon * $one.max(a7.abs()).max(b7.abs())
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
    (f32, 0.0f32, 0.5f32, 1.0f32, 2.0f32),
    (f64, 0.0f64, 0.5f64, 1.0f64, 2.0f64)
}
math! {
    (f32, super::EPSILON_F32, 0.0f32, 0.5f32, 1.0f32, 2.0f32),
    (f64, super::EPSILON_F64, 0.0f64, 0.5f64, 1.0f64, 2.0f64)
}

#[cfg(feature = "gl")]
impl super::GLF32<8> for Quat2<f32> {
    #[inline(always)]
    fn gl_f32(&self) -> [f32; 8] {
        self.0.clone()
    }
}

#[cfg(feature = "gl")]
impl super::GLF32<8> for Quat2<f64> {
    #[inline(always)]
    fn gl_f32(&self) -> [f32; 8] {
        [
            self.0[0] as f32,
            self.0[1] as f32,
            self.0[2] as f32,
            self.0[3] as f32,
            self.0[4] as f32,
            self.0[5] as f32,
            self.0[6] as f32,
            self.0[7] as f32,
        ]
    }
}

#[cfg(test)]
mod tests {
    use crate::{quat::Quat, vec3::Vec3, ApproximateEq};

    use super::Quat2;

    #[test]
    fn new() {
        assert_eq!(
            Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0).raw(),
            &[1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0]
        );
    }

    #[test]
    fn new_zero() {
        assert_eq!(
            Quat2::<f64>::new_zero().raw(),
            &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        );
    }

    #[test]
    fn new_identity() {
        assert_eq!(
            Quat2::<f64>::new_identity().raw(),
            &[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
        );
    }

    #[test]
    fn from_slice() {
        assert_eq!(
            Quat2::from_slice([3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]).raw(),
            &[3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
        );
    }

    #[test]
    fn raw() {
        assert_eq!(
            Quat2::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0).raw(),
            &[1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0]
        );
    }

    #[test]
    fn raw_mut() {
        let mut quat = Quat2::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        (*quat.raw_mut()) = [11.0, 11.0, 11.0, 11.0, 11.0, 11.0, 11.0, 11.0];
        assert_eq!(
            quat.raw(),
            &[11.0, 11.0, 11.0, 11.0, 11.0, 11.0, 11.0, 11.0]
        );
    }

    #[test]
    fn x1() {
        assert_eq!(
            Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0).x1(),
            &1.0
        );
    }

    #[test]
    fn y1() {
        assert_eq!(
            Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0).y1(),
            &2.0
        );
    }

    #[test]
    fn z1() {
        assert_eq!(
            Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0).z1(),
            &3.0
        );
    }

    #[test]
    fn w1() {
        assert_eq!(
            Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0).w1(),
            &4.0
        );
    }

    #[test]
    fn x2() {
        assert_eq!(
            Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0).x2(),
            &2.0
        );
    }

    #[test]
    fn y2() {
        assert_eq!(
            Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0).y2(),
            &5.0
        );
    }

    #[test]
    fn z2() {
        assert_eq!(
            Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0).z2(),
            &6.0
        );
    }

    #[test]
    fn w2() {
        assert_eq!(
            Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0).w2(),
            &-2.0
        );
    }

    #[test]
    fn x1_mut() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        (*quat.x1_mut()) = 6.0;
        assert_eq!(quat.raw(), &[6.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0]);
    }

    #[test]
    fn y1_mut() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        (*quat.y1_mut()) = 6.0;
        assert_eq!(quat.raw(), &[1.0, 6.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0]);
    }

    #[test]
    fn z1_mut() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        (*quat.z1_mut()) = 6.0;
        assert_eq!(quat.raw(), &[1.0, 2.0, 6.0, 4.0, 2.0, 5.0, 6.0, -2.0]);
    }

    #[test]
    fn w1_mut() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        (*quat.w1_mut()) = 6.0;
        assert_eq!(quat.raw(), &[1.0, 2.0, 3.0, 6.0, 2.0, 5.0, 6.0, -2.0]);
    }

    #[test]
    fn x2_mut() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        (*quat.x2_mut()) = 6.0;
        assert_eq!(quat.raw(), &[1.0, 2.0, 3.0, 4.0, 6.0, 5.0, 6.0, -2.0]);
    }

    #[test]
    fn y2_mut() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        (*quat.y2_mut()) = 6.0;
        assert_eq!(quat.raw(), &[1.0, 2.0, 3.0, 4.0, 2.0, 6.0, 6.0, -2.0]);
    }

    #[test]
    fn z2_mut() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        (*quat.z2_mut()) = 10.0;
        assert_eq!(quat.raw(), &[1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 10.0, -2.0]);
    }

    #[test]
    fn w2_mut() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        (*quat.w2_mut()) = 6.0;
        assert_eq!(quat.raw(), &[1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, 6.0]);
    }

    #[test]
    fn set_x1() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        quat.set_x1(6.0);
        assert_eq!(quat.raw(), &[6.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0]);
    }

    #[test]
    fn set_y1() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        quat.set_y1(6.0);
        assert_eq!(quat.raw(), &[1.0, 6.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0]);
    }

    #[test]
    fn set_z1() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        quat.set_z1(6.0);
        assert_eq!(quat.raw(), &[1.0, 2.0, 6.0, 4.0, 2.0, 5.0, 6.0, -2.0]);
    }

    #[test]
    fn set_w1() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        quat.set_w1(6.0);
        assert_eq!(quat.raw(), &[1.0, 2.0, 3.0, 6.0, 2.0, 5.0, 6.0, -2.0]);
    }

    #[test]
    fn set_x2() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        quat.set_x2(6.0);
        assert_eq!(quat.raw(), &[1.0, 2.0, 3.0, 4.0, 6.0, 5.0, 6.0, -2.0]);
    }

    #[test]
    fn set_y2() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        quat.set_y2(6.0);
        assert_eq!(quat.raw(), &[1.0, 2.0, 3.0, 4.0, 2.0, 6.0, 6.0, -2.0]);
    }

    #[test]
    fn set_z2() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        quat.set_z2(10.0);
        assert_eq!(quat.raw(), &[1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 10.0, -2.0]);
    }

    #[test]
    fn set_w2() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        quat.set_w2(6.0);
        assert_eq!(quat.raw(), &[1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, 6.0]);
    }

    #[test]
    fn set() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        quat.set(22.0, 22.0, 22.0, 22.0, 22.0, 22.0, 22.0, 22.0);
        assert_eq!(
            quat.raw(),
            &[22.0, 22.0, 22.0, 22.0, 22.0, 22.0, 22.0, 22.0]
        );
    }

    #[test]
    fn set_zero() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        quat.set_zero();
        assert_eq!(quat.raw(), &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn set_identify() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        quat.set_identify();
        assert_eq!(quat.raw(), &[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn set_real() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        assert_eq!(
            quat.set_real(&Quat::new(4.0, 6.0, 8.0, -100.0)).raw(),
            &[4.0, 6.0, 8.0, -100.0, 2.0, 5.0, 6.0, -2.0]
        );
    }

    #[test]
    fn set_dual() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        assert_eq!(
            quat.set_dual(&Quat::new(4.3, 6.0, 8.0, -100.0)).raw(),
            &[1.0, 2.0, 3.0, 4.0, 4.3, 6.0, 8.0, -100.0]
        );
    }

    #[test]
    fn add_quat_quat() {
        let quat0 = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        let quat1 = Quat2::<f64>::new(5.0, 6.0, 7.0, 8.0, 9.0, 8.0, 6.0, -4.0);
        assert_eq!(
            (quat0 + quat1).approximate_eq(&Quat2::<f64>::new(
                6.0, 8.0, 10.0, 12.0, 11.0, 13.0, 12.0, -6.0
            )),
            true
        );
    }

    #[test]
    fn add_quat_scalar() {
        let quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        let scalar = 1.0;
        assert_eq!(
            (quat + scalar)
                .approximate_eq(&Quat2::<f64>::new(2.0, 3.0, 4.0, 5.0, 3.0, 6.0, 7.0, -1.0)),
            true
        );
    }

    #[test]
    fn add_scalar_quat() {
        let scalar = 1.0;
        let quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        assert_eq!(
            (scalar + quat)
                .approximate_eq(&Quat2::<f64>::new(2.0, 3.0, 4.0, 5.0, 3.0, 6.0, 7.0, -1.0)),
            true
        );
    }

    #[test]
    fn add_assign_quat_quat() {
        let mut quat0 = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        let quat1 = Quat2::<f64>::new(5.0, 6.0, 7.0, 8.0, 9.0, 8.0, 6.0, -4.0);
        quat0 += quat1;
        assert_eq!(
            quat0.approximate_eq(&Quat2::<f64>::new(
                6.0, 8.0, 10.0, 12.0, 11.0, 13.0, 12.0, -6.0
            )),
            true
        );
    }

    #[test]
    fn add_assign_quat_scalar() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        let scalar = 1.0;
        quat += scalar;
        assert_eq!(
            quat.approximate_eq(&Quat2::<f64>::new(2.0, 3.0, 4.0, 5.0, 3.0, 6.0, 7.0, -1.0)),
            true
        );
    }

    #[test]
    fn sub_quat_quat() {
        let quat0 = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        let quat1 = Quat2::<f64>::new(5.0, 6.0, 7.0, 8.0, 9.0, 8.0, 6.0, -4.0);
        assert_eq!(
            (quat0 - quat1).approximate_eq(&Quat2::<f64>::new(
                -4.0, -4.0, -4.0, -4.0, -7.0, -3.0, 0.0, 2.0
            )),
            true
        );
    }

    #[test]
    fn sub_quat_scalar() {
        let quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        let scalar = 1.0;
        assert_eq!(
            (quat - scalar)
                .approximate_eq(&Quat2::<f64>::new(0.0, 1.0, 2.0, 3.0, 1.0, 4.0, 5.0, -3.0)),
            true
        );
    }

    #[test]
    fn sub_scalar_quat() {
        let scalar = 1.0;
        let quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        assert_eq!(
            (scalar - quat).approximate_eq(&Quat2::<f64>::new(
                0.0, -1.0, -2.0, -3.0, -1.0, -4.0, -5.0, 3.0
            )),
            true
        );
    }

    #[test]
    fn sub_assign_quat_quat() {
        let mut quat0 = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        let quat1 = Quat2::<f64>::new(5.0, 6.0, 7.0, 8.0, 9.0, 8.0, 6.0, -4.0);
        quat0 -= quat1;
        assert_eq!(
            quat0.approximate_eq(&Quat2::<f64>::new(
                -4.0, -4.0, -4.0, -4.0, -7.0, -3.0, 0.0, 2.0
            )),
            true
        );
    }

    #[test]
    fn sub_assign_quat_scalar() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        let scalar = 1.0;
        quat -= scalar;
        assert_eq!(
            quat.approximate_eq(&Quat2::<f64>::new(0.0, 1.0, 2.0, 3.0, 1.0, 4.0, 5.0, -3.0)),
            true
        );
    }

    #[test]
    fn mul_quat_quat() {
        let quat0 = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        let quat1 = Quat2::<f64>::new(5.0, 6.0, 7.0, 8.0, 9.0, 8.0, 6.0, -4.0);
        assert_eq!(
            (quat0 * quat1).approximate_eq(&Quat2::<f64>::new(
                24.0, 48.0, 48.0, -6.0, 25.0, 89.0, 23.0, -157.0
            )),
            true
        );
    }

    #[test]
    fn mul_quat_scalar() {
        let quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        let scalar = 2.0;
        assert_eq!(
            (quat * scalar).approximate_eq(&Quat2::<f64>::new(
                2.0, 4.0, 6.0, 8.0, 4.0, 10.0, 12.0, -4.0
            )),
            true
        );
    }

    #[test]
    fn mul_scalar_quat() {
        let scalar = 3.0;
        let quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        assert_eq!(
            (scalar * quat).approximate_eq(&Quat2::<f64>::new(
                3.0, 6.0, 9.0, 12.0, 6.0, 15.0, 18.0, -6.0
            )),
            true
        );
    }

    #[test]
    fn mul_assign_quat_quat() {
        let mut quat0 = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        let quat1 = Quat2::<f64>::new(5.0, 6.0, 7.0, 8.0, 9.0, 8.0, 6.0, -4.0);
        quat0 *= quat1;
        assert_eq!(
            quat0.approximate_eq(&Quat2::<f64>::new(
                24.0, 48.0, 48.0, -6.0, 25.0, 89.0, 23.0, -157.0
            )),
            true
        );
    }

    #[test]
    fn mul_assign_quat_scalar() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        let scalar = 4.0;
        quat *= scalar;
        assert_eq!(
            quat.approximate_eq(&Quat2::<f64>::new(
                4.0, 8.0, 12.0, 16.0, 8.0, 20.0, 24.0, -8.0
            )),
            true
        );
    }

    #[test]
    fn div_quat_scalar() {
        let quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        let scalar = 2.0;
        assert_eq!(
            (quat / scalar)
                .approximate_eq(&Quat2::<f64>::new(0.5, 1.0, 1.5, 2.0, 1.0, 2.5, 3.0, -1.0)),
            true
        );
    }

    #[test]
    fn div_scalar_quat() {
        let scalar = 2.0;
        let quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        assert_eq!(
            (scalar / quat).approximate_eq(&Quat2::<f64>::new(
                2.0,
                1.0,
                0.6666666666666667,
                0.5,
                1.0,
                0.4,
                0.3333333333333333,
                -1.0
            )),
            true
        );
    }

    #[test]
    fn div_assign_quat_scalar() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        let scalar = 4.0;
        quat /= scalar;
        assert_eq!(
            quat.approximate_eq(&Quat2::<f64>::new(
                0.25, 0.5, 0.75, 1.0, 0.5, 1.25, 1.5, -0.5
            )),
            true
        );
    }

    #[test]
    fn from_rotation() {
        assert_eq!(
            Quat2::<f64>::from_rotation(&Quat::new(1.0, 2.0, 3.0, 4.0))
                .approximate_eq(&Quat2::new(1.0, 2.0, 3.0, 4.0, 0.0, 0.0, 0.0, 0.0)),
            true
        );
    }

    #[test]
    fn from_translation() {
        assert_eq!(
            Quat2::<f64>::from_translation(&Vec3::new(1.0, 2.0, 3.0))
                .approximate_eq(&Quat2::new(0.0, 0.0, 0.0, 1.0, 0.5, 1.0, 1.5, 0.0)),
            true
        );
    }

    #[test]
    fn from_rotation_translation() {
        assert_eq!(
            Quat2::<f64>::from_rotation_translation(
                &Quat::new(1.0, 2.0, 3.0, 4.0),
                &Vec3::new(1.0, 2.0, 3.0)
            )
            .approximate_eq(&Quat2::new(1.0, 2.0, 3.0, 4.0, 2.0, 4.0, 6.0, -7.0)),
            true
        );
    }

    #[test]
    fn from_rotation_translation_values() {
        assert_eq!(
            Quat2::<f64>::from_rotation_translation_values(1.0, 2.0, 3.0, 4.0, 1.0, 2.0, 3.0)
                .approximate_eq(&Quat2::new(1.0, 2.0, 3.0, 4.0, 2.0, 4.0, 6.0, -7.0)),
            true
        );
    }

    #[test]
    fn from_mat4() {
        let quat = Quat2::<f64>::from_rotation_translation(
            &Quat::<f64>::new(1.0, 2.0, 3.0, 4.0).normalize(),
            &Vec3::<f64>::new(1.0, -5.0, 3.0),
        )
        .normalize();

        assert_eq!(
            quat.approximate_eq(&Quat2::new(
                0.18257418583505539,
                0.36514837167011077,
                0.5477225575051662,
                0.7302967433402215,
                -1.5518805795979707,
                -1.8257418583505538,
                1.734454765433026,
                0.0
            )),
            true
        );
    }

    #[test]
    fn rotate_by_quat_append() {
        let quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);

        assert_eq!(
            quat.rotate_by_quat_append(&Quat::new(2.0, 5.0, 2.0, -10.0))
                .approximate_eq(&(quat * Quat2::new(2.0, 5.0, 2.0, -10.0, 0.0, 0.0, 0.0, 0.0))),
            true
        );
    }

    #[test]
    fn rotate_by_quat_append_in_place() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        quat.rotate_by_quat_append_in_place(&Quat::new(2.0, 5.0, 2.0, -10.0));

        assert_eq!(
            quat.approximate_eq(
                &(Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0)
                    * Quat2::new(2.0, 5.0, 2.0, -10.0, 0.0, 0.0, 0.0, 0.0))
            ),
            true
        );
    }

    #[test]
    fn rotate_by_quat_prepend() {
        let quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);

        assert_eq!(
            quat.rotate_by_quat_prepend(&Quat::new(2.0, 5.0, 2.0, -10.0))
                .approximate_eq(&(Quat2::new(2.0, 5.0, 2.0, -10.0, 0.0, 0.0, 0.0, 0.0) * quat)),
            true
        );
    }

    #[test]
    fn rotate_by_quat_prepend_in_place() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        quat.rotate_by_quat_prepend_in_place(&Quat::new(2.0, 5.0, 2.0, -10.0));

        assert_eq!(
            quat.approximate_eq(
                &(Quat2::new(2.0, 5.0, 2.0, -10.0, 0.0, 0.0, 0.0, 0.0)
                    * Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0))
            ),
            true
        );
    }

    #[test]
    fn squared_length() {
        let quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        assert_eq!(quat.squared_length().approximate_eq(&30.0), true);
    }

    #[test]
    fn length() {
        let quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        assert_eq!(quat.length().approximate_eq(&5.477225575051661), true);
    }

    #[test]
    fn dot() {
        let quat0 = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        let quat1 = Quat2::<f64>::new(5.0, 6.0, 7.0, 8.0, 9.0, 8.0, 6.0, -4.0);
        assert_eq!(quat0.dot(&quat1).approximate_eq(&70.0), true);
    }

    #[test]
    fn conjugate() {
        let quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        assert_eq!(
            quat.conjugate()
                .approximate_eq(&Quat2::new(-1.0, -2.0, -3.0, 4.0, -2.0, -5.0, -6.0, -2.0)),
            true
        );
    }

    #[test]
    fn conjugate_in_place() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        quat.conjugate_in_place();
        assert_eq!(
            quat.approximate_eq(&Quat2::new(-1.0, -2.0, -3.0, 4.0, -2.0, -5.0, -6.0, -2.0)),
            true
        );
    }

    #[test]
    fn invert() {
        let quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        let quat = quat.invert();
        assert_eq!(
            quat.approximate_eq(&Quat2::new(
                -0.03333333333333333,
                -0.06666666666666667,
                -0.1,
                0.13333333333333333,
                -0.06666666666666667,
                -0.16666666666666666,
                -0.2,
                -0.06666666666666667
            )),
            true
        );
        let real = quat.real();
        assert_eq!(real.x().approximate_eq(&-0.033333), true);
        assert_eq!(real.y().approximate_eq(&-0.066666), true);
        assert_eq!(real.z().approximate_eq(&-0.1), true);
        assert_eq!(real.w().approximate_eq(&0.133333), true);
    }

    #[test]
    fn invert_in_place() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        quat.invert_in_place();
        assert_eq!(
            quat.approximate_eq(&Quat2::new(
                -0.03333333333333333,
                -0.06666666666666667,
                -0.1,
                0.13333333333333333,
                -0.06666666666666667,
                -0.16666666666666666,
                -0.2,
                -0.06666666666666667
            )),
            true
        );
        assert_eq!(quat.real().x().approximate_eq(&-0.033333), true);
        assert_eq!(quat.real().y().approximate_eq(&-0.066666), true);
        assert_eq!(quat.real().z().approximate_eq(&-0.1), true);
        assert_eq!(quat.real().w().approximate_eq(&0.133333), true);
    }

    #[test]
    fn normalize() {
        let quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        assert_eq!(
            quat.normalize().approximate_eq(&Quat2::new(
                0.18257418583505536,
                0.3651483716701107,
                0.5477225575051661,
                0.7302967433402214,
                0.23126063539107017,
                0.6450954566171957,
                0.6937819061732106,
                -0.900699316786273
            )),
            true
        );

        let quat = Quat2::<f64>::new(5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        assert_eq!(
            quat.normalize()
                .approximate_eq(&Quat2::new(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
            true
        );

        let quat = Quat2::<f64>::new(5.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 5.0);
        assert_eq!(
            quat.normalize()
                .approximate_eq(&Quat2::new(1.0, 0.0, 0.0, 0.0, 0.0, 0.4, 0.6, 1.0)),
            true
        );
    }

    #[test]
    fn normalize_in_place() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        quat.normalize_in_place();
        assert_eq!(
            quat.approximate_eq(&Quat2::new(
                0.18257418583505536,
                0.3651483716701107,
                0.5477225575051661,
                0.7302967433402214,
                0.23126063539107017,
                0.6450954566171957,
                0.6937819061732106,
                -0.900699316786273
            )),
            true
        );

        let mut quat = Quat2::<f64>::new(5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        quat.normalize_in_place();
        assert_eq!(
            quat.approximate_eq(&Quat2::new(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
            true
        );

        let mut quat = Quat2::<f64>::new(5.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 5.0);
        quat.normalize_in_place();
        assert_eq!(
            quat.approximate_eq(&Quat2::new(1.0, 0.0, 0.0, 0.0, 0.0, 0.4, 0.6, 1.0)),
            true
        );
    }

    #[test]
    fn rotate_around_axis() {
        let quat = Quat2::<f64>::from_rotation_translation(
            &Quat::new(1.0, 2.0, 3.0, 4.0),
            &Vec3::new(-5.0, 4.0, 10.0),
        );
        let quat = quat.normalize();
        let quat = quat.rotate_around_axis(&Vec3::new(1.0, 4.0, 2.0), 5.0);

        assert_eq!(
            quat.approximate_eq(&Quat2::new(
                -0.24164294714846665,
                0.11280662947202075,
                -0.20036742052872042,
                -0.9427280876431084,
                1.3920522306902268,
                -3.594589462350351,
                -4.512371117598661,
                0.17211647582839384
            )),
            true
        );
    }

    #[test]
    fn rotate_around_axis_in_place() {
        let mut quat = Quat2::<f64>::from_rotation_translation(
            &Quat::new(1.0, 2.0, 3.0, 4.0),
            &Vec3::new(-5.0, 4.0, 10.0),
        );
        quat.normalize_in_place();
        quat.rotate_around_axis_in_place(&Vec3::new(1.0, 4.0, 2.0), 5.0);

        assert_eq!(
            quat.approximate_eq(&Quat2::new(
                -0.24164294714846665,
                0.11280662947202075,
                -0.20036742052872042,
                -0.9427280876431084,
                1.3920522306902268,
                -3.594589462350351,
                -4.512371117598661,
                0.17211647582839384
            )),
            true
        );
    }

    #[test]
    fn rotate_x() {
        let quat = Quat2::<f64>::from_rotation_translation(
            &Quat::new(1.0, 2.0, 3.0, 4.0),
            &Vec3::new(-5.0, 4.0, 10.0),
        );
        let quat = quat.normalize();
        let quat = quat.rotate_x(5.0);

        assert_eq!(
            quat.approximate_eq(&Quat2::new(
                0.2907941144735252,
                0.0352604066733514,
                -0.6573355589457817,
                -0.6943381378364758,
                0.2448721933328691,
                -1.5780446006697788,
                -4.141429934812809,
                3.943142267566019
            )),
            true
        );
    }

    #[test]
    fn rotate_x_in_place() {
        let mut quat = Quat2::<f64>::from_rotation_translation(
            &Quat::new(1.0, 2.0, 3.0, 4.0),
            &Vec3::new(-5.0, 4.0, 10.0),
        );
        let quat = quat.normalize_in_place();
        let quat = quat.rotate_x_in_place(5.0);

        assert_eq!(
            quat.approximate_eq(&Quat2::new(
                0.2907941144735252,
                0.0352604066733514,
                -0.6573355589457817,
                -0.6943381378364758,
                0.2448721933328691,
                -1.5780446006697788,
                -4.141429934812809,
                3.943142267566019
            )),
            true
        );
    }

    #[test]
    fn rotate_y() {
        let quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        let quat = quat.normalize();
        let quat = quat.rotate_y(-2.0);

        assert_eq!(
            quat.approximate_eq(&Quat2::new(
                0.5595378934640505,
                -0.4172330126231385,
                0.14230488084091206,
                0.7018427743049626,
                0.7087479983877976,
                1.1064589038272759,
                0.18025284906515548,
                0.05617919142129002
            )),
            true
        );
    }

    #[test]
    fn rotate_y_in_place() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        let quat = quat.normalize_in_place();
        let quat = quat.rotate_y_in_place(-2.0);

        assert_eq!(
            quat.approximate_eq(&Quat2::new(
                0.5595378934640505,
                -0.4172330126231385,
                0.14230488084091206,
                0.7018427743049626,
                0.7087479983877976,
                1.1064589038272759,
                0.18025284906515548,
                0.05617919142129002
            )),
            true
        );
    }

    #[test]
    fn rotate_z() {
        let quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        let quat = quat.normalize();
        let quat = quat.rotate_z(1.0);

        assert_eq!(
            quat.approximate_eq(&Quat2::new(
                0.3352853764985674,
                0.2329171161011585,
                0.8307946747373117,
                0.37830350482312175,
                0.5122255376107139,
                0.4552522688016444,
                0.1770326475419759,
                -1.123054777959148
            )),
            true
        );
    }

    #[test]
    fn rotate_z_in_place() {
        let mut quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        let quat = quat.normalize_in_place();
        let quat = quat.rotate_z_in_place(1.0);

        assert_eq!(
            quat.approximate_eq(&Quat2::new(
                0.3352853764985674,
                0.2329171161011585,
                0.8307946747373117,
                0.37830350482312175,
                0.5122255376107139,
                0.4552522688016444,
                0.1770326475419759,
                -1.123054777959148
            )),
            true
        );
    }

    #[test]
    fn translation() {
        let quat = Quat2::<f64>::from_translation(&Vec3::new(1.0, 2.0, 3.0));
        assert_eq!(
            quat.translation().approximate_eq(&Vec3::new(1.0, 2.0, 3.0)),
            true
        );
        assert_eq!(
            quat.translation().normalize().approximate_eq(&Vec3::new(
                0.2672612419124244,
                0.5345224838248488,
                0.8017837257372732
            )),
            true
        );

        let quat = Quat2::<f64>::from_rotation_translation(
            &Quat::new(2.0, 4.0, 6.0, 2.0),
            &Vec3::new(1.0, 2.0, 3.0),
        );
        assert_ne!(
            quat.translation().approximate_eq(&Vec3::new(1.0, 2.0, 3.0)),
            true
        );
        assert_eq!(
            quat.normalize()
                .translation()
                .approximate_eq(&Vec3::new(1.0, 2.0, 3.0)),
            true
        );
    }

    #[test]
    fn translate() {
        let quat = Quat2::<f64>::from_rotation_translation(
            &Quat::new(1.0, 2.0, 3.0, 4.0),
            &Vec3::new(-5.0, 4.0, 10.0),
        );
        let quat = quat.normalize();
        let quat = quat.translate(&Vec3::new(1.0, 1.0, -1.0));

        assert_eq!(
            quat.approximate_eq(&Quat2::new(
                0.18257418583505536,
                0.3651483716701107,
                0.5477225575051661,
                0.7302967433402214,
                -2.647325694608303,
                4.473067552958856,
                1.9170289512680818,
                -3.0124740662784135
            )),
            true
        );
    }

    #[test]
    fn translate_in_place() {
        let mut quat = Quat2::<f64>::from_rotation_translation(
            &Quat::new(1.0, 2.0, 3.0, 4.0),
            &Vec3::new(-5.0, 4.0, 10.0),
        );
        quat.normalize_in_place();
        quat.translate_in_place(&Vec3::new(1.0, 1.0, -1.0));

        assert_eq!(
            quat.approximate_eq(&Quat2::new(
                0.18257418583505536,
                0.3651483716701107,
                0.5477225575051661,
                0.7302967433402214,
                -2.647325694608303,
                4.473067552958856,
                1.9170289512680818,
                -3.0124740662784135
            )),
            true
        );
    }

    #[test]
    fn lerp() {
        let quat0 = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        let quat1 = Quat2::<f64>::new(5.0, 6.0, 7.0, 8.0, 9.0, 8.0, 6.0, -4.0);
        assert_eq!(
            quat0.lerp(&quat1, 0.7).approximate_eq(&Quat2::new(
                3.8,
                4.799999999999999,
                5.8,
                6.8,
                6.9,
                7.1,
                6.0,
                -3.4
            )),
            true
        );
    }

    #[test]
    fn display() {
        let quat = Quat2::<f64>::new(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0);
        assert_eq!(quat.to_string(), "quat2(1, 2, 3, 4, 2, 5, 6, -2)");
    }
}
