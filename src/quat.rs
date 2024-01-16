use std::{
    fmt::{Debug, Display},
    ops::{Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Sub, SubAssign},
    slice::SliceIndex,
};

use crate::{mat3::Mat3, vec3::Vec3, ApproximateEq};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EulerOrder {
    XYZ,
    XZY,
    YZX,
    YXZ,
    ZXY,
    ZYX,
}

pub struct Quat<T = f64>([T; 4]);

impl<T: Debug> Debug for Quat<T> {
    #[inline(always)]
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_tuple("Quat").field(&self.0).finish()
    }
}

impl<T: Copy> Copy for Quat<T> {}

impl<T: Clone> Clone for Quat<T> {
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

impl<T: PartialEq> PartialEq for Quat<T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl<T: Display> Display for Quat<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_fmt(format_args!(
            "quat({}, {}, {}, {})",
            self.0[0], self.0[1], self.0[2], self.0[3]
        ))
    }
}

impl<T> Quat<T> {
    #[inline(always)]
    pub const fn new(x: T, y: T, z: T, w: T) -> Self {
        Self([x, y, z, w])
    }

    #[inline(always)]
    pub const fn from_slice(values: [T; 4]) -> Self {
        Self(values)
    }
}

impl<T> Quat<T> {
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
    pub fn set(&mut self, x: T, y: T, z: T, w: T) {
        self.0[0] = x;
        self.0[1] = y;
        self.0[2] = z;
        self.0[3] = w;
    }
}

impl<T, I> Index<I> for Quat<T>
where
    I: SliceIndex<[T], Output = T>,
{
    type Output = T;

    #[inline(always)]
    fn index(&self, index: I) -> &Self::Output {
        self.0.index(index)
    }
}

impl<T, I> IndexMut<I> for Quat<T>
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
        impl Quat<$t> {
            #[inline(always)]
            pub const fn new_zero() -> Self {
                Self([$zero, $zero, $zero, $zero])
            }

            #[inline(always)]
            pub fn new_identity() -> Self {
                Self([$zero, $zero, $zero, $one])
            }
        }

        #[cfg(feature = "rand")]
        impl rand::distributions::Distribution<Quat<$t>> for rand::distributions::Standard {
            fn sample<R: rand::prelude::Rng + ?Sized>(&self, rng: &mut R) -> Quat<$t> {
                Quat::new(
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
    ($(($t: ident, $zero: expr, $half: expr, $one: expr, $two: expr, $d360: expr, $o1: expr, $o2: expr, $pi: expr)),+) => {
       $(
        impl Quat<$t> {
            #[inline(always)]
            pub fn from_axis_angle(axis: &Vec3::<$t>, rad: $t) -> Self {
                let rad = rad * $half;
                let s = rad.sin();

                Self::new(s * axis.x(), s * axis.y(), s * axis.z(), rad.cos())
            }


            #[inline(always)]
            pub fn from_axes(view: &Vec3::<$t>, right: &Vec3::<$t>, up: &Vec3::<$t>) -> Self {
                let matr = Mat3::<$t>::new(
                    *right.x(),
                    *up.x(),
                    -view.x(),
                    *right.y(),
                    *up.y(),
                    -view.y(),
                    *right.z(),
                    *up.z(),
                    -view.z(),
                );

                Self::from_mat3(&matr).normalize()
            }

            #[inline(always)]
            pub fn from_mat3(m: &Mat3::<$t>) -> Self {
                let mut out = Self::new_zero();

                // Algorithm in Ken Shoemake's article in 1987 SIGGRAPH course notes
                // article "Quaternion Calculus and Fast Animation".
                let f_trace = m[0] + m[4] + m[8];
                let mut f_root;

                if f_trace > $zero {
                    // |w| > 1/2, may as well choose w > 1/2
                    f_root = (f_trace + $one).sqrt(); // 2w
                    out.0[3] = $half * f_root;
                    f_root = $half / f_root; // 1/(4w)
                    out.0[0] = (m[5] - m[7]) * f_root;
                    out.0[1] = (m[6] - m[2]) * f_root;
                    out.0[2] = (m[1] - m[3]) * f_root;
                } else {
                    // |w| <= 1/2
                    let mut i = 0;
                    if m[4] > m[0] {
                        i = 1
                    };
                    if m[8] > m[i * 3 + i] {
                        i = 2
                    };
                    let j = (i + 1) % 3;
                    let k = (i + 2) % 3;

                    f_root = (m[i * 3 + i] - m[j * 3 + j] - m[k * 3 + k] + $one).sqrt();
                    out.0[i] = $half * f_root;
                    f_root = $half / f_root;
                    out.0[3] = (m[j * 3 + k] - m[k * 3 + j]) * f_root;
                    out.0[j] = (m[j * 3 + i] + m[i * 3 + j]) * f_root;
                    out.0[k] = (m[k * 3 + i] + m[i * 3 + k]) * f_root;
                }

                out
            }

            #[inline(always)]
            pub fn from_euler_with_order(x: $t, y: $t, z: $t, order: EulerOrder) -> Self {
                let half_to_rad = $pi / $d360;
                let x = x * half_to_rad;
                let z = z * half_to_rad;
                let y = y * half_to_rad;

                let sx = x.sin();
                let cx = x.cos();
                let sy = y.sin();
                let cy = y.cos();
                let sz = z.sin();
                let cz = z.cos();

                match order {
                    EulerOrder::XYZ => Self::new(
                        sx * cy * cz + cx * sy * sz,
                        cx * sy * cz - sx * cy * sz,
                        cx * cy * sz + sx * sy * cz,
                        cx * cy * cz - sx * sy * sz,
                    ),
                    EulerOrder::XZY => Self::new(
                        sx * cy * cz - cx * sy * sz,
                        cx * sy * cz - sx * cy * sz,
                        cx * cy * sz + sx * sy * cz,
                        cx * cy * cz + sx * sy * sz,
                    ),
                    EulerOrder::YXZ => Self::new(
                        sx * cy * cz + cx * sy * sz,
                        cx * sy * cz - sx * cy * sz,
                        cx * cy * sz - sx * sy * cz,
                        cx * cy * cz + sx * sy * sz,
                    ),
                    EulerOrder::YZX => Self::new(
                        sx * cy * cz + cx * sy * sz,
                        cx * sy * cz + sx * cy * sz,
                        cx * cy * sz - sx * sy * cz,
                        cx * cy * cz - sx * sy * sz,
                    ),
                    EulerOrder::ZXY => Self::new(
                        sx * cy * cz - cx * sy * sz,
                        cx * sy * cz + sx * cy * sz,
                        cx * cy * sz + sx * sy * cz,
                        cx * cy * cz - sx * sy * sz,
                    ),
                    EulerOrder::ZYX => Self::new(
                        sx * cy * cz - cx * sy * sz,
                        cx * sy * cz + sx * cy * sz,
                        cx * cy * sz - sx * sy * cz,
                        cx * cy * cz + sx * sy * sz,
                    ),
                }
            }

            #[inline(always)]
            pub fn from_euler(x: $t, y: $t, z: $t) -> Self {
                Self::from_euler_with_order(x, y, z, EulerOrder::ZYX)
            }

            #[inline(always)]
            pub fn from_rotation_to(a: &Vec3::<$t>, b: &Vec3::<$t>) -> Self {
                let dot = a.dot(b);
                if dot < -$o1 {
                    let x_unit = Vec3::<$t>::new($one, $zero, $zero);

                    let mut tmp = x_unit.cross(a);
                    if tmp.length() < $o2 {
                        let y_unit = Vec3::<$t>::new($zero, $one, $zero);
                        tmp = y_unit.cross(a);
                    };

                    Self::from_axis_angle(&tmp.normalize(), $pi)
                } else if dot > $o1 {
                    Self([$zero, $zero, $zero, $one])
                } else {
                    let tmp = a.cross(b);
                    Self([*tmp.x(), *tmp.y(), *tmp.z(), $one + dot]).normalize()
                }
            }

            #[inline(always)]
            pub fn random() -> Self {
                let u1 = rand::random::<$t>();
                let u2 = rand::random::<$t>();
                let u3 = rand::random::<$t>();
                let sqrt1_minus_u1 = ($one - u1).sqrt();
                let sqrt_u1 = u1.sqrt();
                Self([
                    sqrt1_minus_u1 * ($two * $pi * u2).sin(),
                    sqrt1_minus_u1 * ($two * $pi * u2).cos(),
                    sqrt_u1 * ($two * $pi * u3).sin(),
                    sqrt_u1 * ($two * $pi * u3).cos(),
                ])
            }
        }
       )+
    };
}

macro_rules! math {
    ($(($t: ident, $epsilon: expr, $zero: expr, $half: expr, $one: expr, $two: expr, $pi: expr)),+) => {
       $(
        impl Add for Quat<$t> {
            type Output = Quat<$t>;

            #[inline(always)]
            fn add(mut self, rhs: Self) -> Self::Output {
                self.0[0] = self.0[0] + rhs.0[0];
                self.0[1] = self.0[1] + rhs.0[1];
                self.0[2] = self.0[2] + rhs.0[2];
                self.0[3] = self.0[3] + rhs.0[3];
                self
            }
        }

        impl Add<$t> for Quat<$t> {
            type Output = Quat<$t>;

            #[inline(always)]
            fn add(mut self, rhs: $t) -> Self::Output {
                self.0[0] = self.0[0] + rhs;
                self.0[1] = self.0[1] + rhs;
                self.0[2] = self.0[2] + rhs;
                self.0[3] = self.0[3] + rhs;
                self
            }
        }

        impl Add<Quat<$t>> for $t {
            type Output = Quat<$t>;

            #[inline(always)]
            fn add(self, mut rhs: Quat<$t>) -> Self::Output {
                rhs.0[0] = self + rhs.0[0];
                rhs.0[1] = self + rhs.0[1];
                rhs.0[2] = self + rhs.0[2];
                rhs.0[3] = self + rhs.0[3];
                rhs
            }
        }

        impl AddAssign for Quat<$t> {
            #[inline(always)]
            fn add_assign(&mut self, rhs: Self) {
                self.0[0] += rhs.0[0];
                self.0[1] += rhs.0[1];
                self.0[2] += rhs.0[2];
                self.0[3] += rhs.0[3];
            }
        }

        impl AddAssign<$t> for Quat<$t> {
            #[inline(always)]
            fn add_assign(&mut self, rhs: $t) {
                self.0[0] += rhs;
                self.0[1] += rhs;
                self.0[2] += rhs;
                self.0[3] += rhs;
            }
        }

        impl Sub for Quat<$t> {
            type Output = Quat<$t>;

            #[inline(always)]
            fn sub(mut self, rhs: Self) -> Self::Output {
                self.0[0] = self.0[0] - rhs.0[0];
                self.0[1] = self.0[1] - rhs.0[1];
                self.0[2] = self.0[2] - rhs.0[2];
                self.0[3] = self.0[3] - rhs.0[3];
                self
            }
        }

        impl Sub<$t> for Quat<$t> {
            type Output = Quat<$t>;

            #[inline(always)]
            fn sub(mut self, rhs: $t) -> Self::Output {
                self.0[0] = self.0[0] - rhs;
                self.0[1] = self.0[1] - rhs;
                self.0[2] = self.0[2] - rhs;
                self.0[3] = self.0[3] - rhs;
                self
            }
        }

        impl Sub<Quat<$t>> for $t {
            type Output = Quat<$t>;

            #[inline(always)]
            fn sub(self, mut rhs: Quat<$t>) -> Self::Output {
                rhs.0[0] = self - rhs.0[0];
                rhs.0[1] = self - rhs.0[1];
                rhs.0[2] = self - rhs.0[2];
                rhs.0[3] = self - rhs.0[3];
                rhs
            }
        }

        impl SubAssign for Quat<$t> {
            #[inline(always)]
            fn sub_assign(&mut self, rhs: Self) {
                self.0[0] -= rhs.0[0];
                self.0[1] -= rhs.0[1];
                self.0[2] -= rhs.0[2];
                self.0[3] -= rhs.0[3];
            }
        }

        impl SubAssign<$t> for Quat<$t> {
            #[inline(always)]
            fn sub_assign(&mut self, rhs: $t) {
                self.0[0] -= rhs;
                self.0[1] -= rhs;
                self.0[2] -= rhs;
                self.0[3] -= rhs;
            }
        }

        impl Mul for Quat<$t> {
            type Output = Quat<$t>;

            #[inline(always)]
            fn mul(mut self, rhs: Self) -> Self::Output {
                let ax = self.0[0];
                let ay = self.0[1];
                let az = self.0[2];
                let aw = self.0[3];
                let bx = rhs.0[0];
                let by = rhs.0[1];
                let bz = rhs.0[2];
                let bw = rhs.0[3];

                self.0[0] = ax * bw + aw * bx + ay * bz - az * by;
                self.0[1] = ay * bw + aw * by + az * bx - ax * bz;
                self.0[2] = az * bw + aw * bz + ax * by - ay * bx;
                self.0[3] = aw * bw - ax * bx - ay * by - az * bz;
                self
            }
        }

        impl Mul<$t> for Quat<$t> {
            type Output = Quat<$t>;

            #[inline(always)]
            fn mul(mut self, rhs: $t) -> Self::Output {
                self.0[0] = self.0[0] * rhs;
                self.0[1] = self.0[1] * rhs;
                self.0[2] = self.0[2] * rhs;
                self.0[3] = self.0[3] * rhs;
                self
            }
        }

        impl Mul<Quat<$t>> for $t {
            type Output = Quat<$t>;

            #[inline(always)]
            fn mul(self, mut rhs: Quat<$t>) -> Self::Output {
                rhs.0[0] = self * rhs.0[0];
                rhs.0[1] = self * rhs.0[1];
                rhs.0[2] = self * rhs.0[2];
                rhs.0[3] = self * rhs.0[3];
                rhs
            }
        }

        impl MulAssign for Quat<$t> {
            #[inline(always)]
            fn mul_assign(&mut self, rhs: Self) {
                let ax = self.0[0];
                let ay = self.0[1];
                let az = self.0[2];
                let aw = self.0[3];
                let bx = rhs.0[0];
                let by = rhs.0[1];
                let bz = rhs.0[2];
                let bw = rhs.0[3];

                self.0[0] = ax * bw + aw * bx + ay * bz - az * by;
                self.0[1] = ay * bw + aw * by + az * bx - ax * bz;
                self.0[2] = az * bw + aw * bz + ax * by - ay * bx;
                self.0[3] = aw * bw - ax * bx - ay * by - az * bz;
            }
        }

        impl MulAssign<$t> for Quat<$t> {
            #[inline(always)]
            fn mul_assign(&mut self, rhs: $t) {
                self.0[0] *= rhs;
                self.0[1] *= rhs;
                self.0[2] *= rhs;
                self.0[3] *= rhs;
            }
        }

        impl Div<$t> for Quat<$t> {
            type Output = Quat<$t>;

            #[inline(always)]
            fn div(mut self, rhs: $t) -> Self::Output {
                self.0[0] = self.0[0] / rhs;
                self.0[1] = self.0[1] / rhs;
                self.0[2] = self.0[2] / rhs;
                self.0[3] = self.0[3] / rhs;
                self
            }
        }

        impl Div<Quat<$t>> for $t {
            type Output = Quat<$t>;

            #[inline(always)]
            fn div(self, mut rhs: Quat<$t>) -> Self::Output {
                rhs.0[0] = self / rhs.0[0];
                rhs.0[1] = self / rhs.0[1];
                rhs.0[2] = self / rhs.0[2];
                rhs.0[3] = self / rhs.0[2];
                rhs
            }
        }

        impl DivAssign<$t> for Quat<$t> {
            #[inline(always)]
            fn div_assign(&mut self, rhs: $t) {
                self.0[0] /= rhs;
                self.0[1] /= rhs;
                self.0[2] /= rhs;
                self.0[3] /= rhs;
            }
        }

        impl Quat<$t> {
            #[inline(always)]
            pub fn axis_angle(&self) -> (Vec3<$t>, $t) {
                let rad = self.w().acos() * $two;
                let s = (rad / $two).sin();
                if s > $epsilon {
                    (
                        Vec3::<$t>::new(self.x() / s, self.y() / s, self.z() / s),
                        rad,
                    )
                } else {
                    // If s is zero, return any axis (no rotation - axis does not matter)
                    (Vec3::<$t>::new($one, $zero, $zero), rad)
                }
            }

            #[inline(always)]
            pub fn angle(&self, b: &Self) -> $t {
                let dotproduct = self.dot(b);
                ($two * dotproduct * dotproduct - $one).acos()
            }

            #[inline(always)]
            pub fn dot(&self, b: &Self) -> $t {
                let ax = self.0[0];
                let ay = self.0[1];
                let az = self.0[2];
                let aw = self.0[3];
                let bx = b.0[0];
                let by = b.0[1];
                let bz = b.0[2];
                let bw = b.0[3];

                ax * bx + ay * by + az * bz + aw * bw
            }

            #[inline(always)]
            pub fn exp(&self) -> Self {
                let x = self.0[0];
                let y = self.0[1];
                let z = self.0[2];
                let w = self.0[3];

                let r = (x * x + y * y + z * z).sqrt();
                let et = w.exp();
                let s = if r > $zero {
                    (et * r.sin()) / r
                } else {
                    $zero
                };

                Self::new(x * s, y * s, z * s, et * r.cos())
            }

            #[inline(always)]
            pub fn exp_in_place(&mut self) -> &mut Self {
                let x = self.0[0];
                let y = self.0[1];
                let z = self.0[2];
                let w = self.0[3];

                let r = (x * x + y * y + z * z).sqrt();
                let et = w.exp();
                let s = if r > $zero {
                    (et * r.sin()) / r
                } else {
                    $zero
                };

                self.0[0] = x * s;
                self.0[1] = y * s;
                self.0[2] = z * s;
                self.0[3] = et * r.cos();
                self
            }

            #[inline(always)]
            pub fn ln(&self) -> Self {
                let x = self.0[0];
                let y = self.0[1];
                let z = self.0[2];
                let w = self.0[3];

                let r = (x * x + y * y + z * z).sqrt();
                let t = if r > $zero {
                    r.atan2(w) / r
                } else {
                    $zero
                };

                Self::new(
                    x * t,
                    y * t,
                    z * t,
                    $half * (x * x + y * y + z * z + w * w).ln(),
                )
            }

            #[inline(always)]
            pub fn ln_in_place(&mut self) -> &mut Self {
                let x = self.0[0];
                let y = self.0[1];
                let z = self.0[2];
                let w = self.0[3];

                let r = (x * x + y * y + z * z).sqrt();
                let t = if r > $zero {
                    r.atan2(w) / r
                } else {
                    $zero
                };

                self.0[0] = x * t;
                self.0[1] = y * t;
                self.0[2] = z * t;
                self.0[3] = $half * (x * x + y * y + z * z + w * w).ln();
                self
            }

            #[inline(always)]
            pub fn pow(&self, b: $t) -> Self {
                (self.ln() * b).exp()
            }

            #[inline(always)]
            pub fn pow_in_place(&mut self, b: $t) -> &mut Self {
                self.ln_in_place();
                Self::mul_assign(self, b);
                self.exp_in_place()
            }


            #[inline(always)]
            pub fn calculate_w(&self) -> Self {
                let x = self.0[0];
                let y = self.0[1];
                let z = self.0[2];

                Self::new(x, y, z, ($one - x * x - y * y - z * z).abs().sqrt())
            }


            #[inline(always)]
            pub fn calculate_w_in_place(&mut self) ->&mut Self {
                let x = self.0[0];
                let y = self.0[1];
                let z = self.0[2];

                self.0[3] = ($one - x * x - y * y - z * z).abs().sqrt();
                self
            }

            #[inline(always)]
            pub fn rotate_x(&self, rad: $t) -> Self {
                let rad = rad * $half;

                let ax = self.0[0];
                let ay = self.0[1];
                let az = self.0[2];
                let aw = self.0[3];
                let bx = rad.sin();
                let bw = rad.cos();

                Self::new(
                    ax * bw + aw * bx,
                    ay * bw + az * bx,
                    az * bw - ay * bx,
                    aw * bw - ax * bx,
                )
            }

            #[inline(always)]
            pub fn rotate_x_in_place(&mut self, rad: $t) -> &mut Self {
                let rad = rad * $half;

                let ax = self.0[0];
                let ay = self.0[1];
                let az = self.0[2];
                let aw = self.0[3];
                let bx = rad.sin();
                let bw = rad.cos();

                self.0[0] = ax * bw + aw * bx;
                self.0[1] = ay * bw + az * bx;
                self.0[2] = az * bw - ay * bx;
                self.0[3] = aw * bw - ax * bx;
                self
            }

            #[inline(always)]
            pub fn rotate_y(&self, rad: $t) -> Self {
                let rad = rad * $half;

                let ax = self.0[0];
                let ay = self.0[1];
                let az = self.0[2];
                let aw = self.0[3];
                let by = rad.sin();
                let bw = rad.cos();

                Self::new(
                    ax * bw - az * by,
                    ay * bw + aw * by,
                    az * bw + ax * by,
                    aw * bw - ay * by,
                )
            }

            #[inline(always)]
            pub fn rotate_y_in_place(&mut self, rad: $t) -> &mut Self {
                let rad = rad * $half;

                let ax = self.0[0];
                let ay = self.0[1];
                let az = self.0[2];
                let aw = self.0[3];
                let by = rad.sin();
                let bw = rad.cos();

                self.0[0] = ax * bw - az * by;
                self.0[1] = ay * bw + aw * by;
                self.0[2] = az * bw + ax * by;
                self.0[3] = aw * bw - ay * by;
                self
            }

            #[inline(always)]
            pub fn rotate_z(&self, rad: $t) -> Self {
                let rad = rad * $half;

                let ax = self.0[0];
                let ay = self.0[1];
                let az = self.0[2];
                let aw = self.0[3];
                let bz = rad.sin();
                let bw = rad.cos();

                Self::new(
                    ax * bw + ay * bz,
                    ay * bw - ax * bz,
                    az * bw + aw * bz,
                    aw * bw - az * bz,
                )
            }

            #[inline(always)]
            pub fn rotate_z_in_place(&mut self, rad: $t) -> &mut Self {
                let rad = rad * $half;

                let ax = self.0[0];
                let ay = self.0[1];
                let az = self.0[2];
                let aw = self.0[3];
                let bz = rad.sin();
                let bw = rad.cos();

                self.0[0] = ax * bw + ay * bz;
                self.0[1] = ay * bw - ax * bz;
                self.0[2] = az * bw + aw * bz;
                self.0[3] = aw * bw - az * bz;
                self
            }

            #[inline(always)]
            pub fn lerp(&self, b: &Self, t: $t) -> Self {
                let ax = self.0[0];
                let ay = self.0[1];
                let az = self.0[2];
                let aw = self.0[3];
                let bx = b.0[0];
                let by = b.0[1];
                let bz = b.0[2];
                let bw = b.0[3];

                Self::new(
                    ax + t * (bx - ax),
                    ay + t * (by - ay),
                    az + t * (bz - az),
                    aw + t * (bw - aw),
                )
            }

            #[inline(always)]
            pub fn slerp(&self, b: &Self, t: $t) -> Self {
                let ax = self.0[0];
                let ay = self.0[1];
                let az = self.0[2];
                let aw = self.0[3];
                let mut bx = b.0[0];
                let mut by = b.0[1];
                let mut bz = b.0[2];
                let mut bw = b.0[3];

                let omega;
                let mut cosom;
                let sinom;
                let scale0;
                let scale1;

                // calc cosine
                cosom = ax * bx + ay * by + az * bz + aw * bw;
                // adjust signs (if necessary)
                if cosom < $zero {
                    cosom = -cosom;
                    bx = -bx;
                    by = -by;
                    bz = -bz;
                    bw = -bw;
                }
                // calculate coefficients
                if $one - cosom > $epsilon {
                    // standard case (slerp)
                    omega = cosom.acos();
                    sinom = omega.sin();
                    scale0 = ((($one - t) * omega) / sinom).sin();
                    scale1 = ((t * omega) / sinom).sin();
                } else {
                    // "from" and "to" quaternions are very close
                    //  ... so we can do a linear interpolation
                    scale0 = $one - t;
                    scale1 = t;
                }

                Self::new(
                    scale0 * ax + scale1 * bx,
                    scale0 * ay + scale1 * by,
                    scale0 * az + scale1 * bz,
                    scale0 * aw + scale1 * bw,
                )
            }

            #[inline(always)]
            pub fn sqlerp(&self, b: &Self, c: &Self, d: &Self, t: $t) -> Self {
                let tmp1 = self.slerp(d, t);
                let mut tmp2 = Self::new_zero();
                tmp2.copy(b).slerp(c, t);
                tmp1.slerp(&tmp2, $two * t * ($one - t))
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
                let mut len = self.squared_length();
                if len > $zero {
                    len = $one / len.sqrt();
                }

                let x = self.0[0];
                let y = self.0[1];
                let z = self.0[2];
                let w = self.0[3];

                Self::new(x * len, y * len, z * len, w * len)
            }

            #[inline(always)]
            pub fn normalize_in_place(&mut self) -> &mut Self {
                let mut len = self.squared_length();
                if len > $zero {
                    len = $one / len.sqrt();
                }

                let x = self.0[0];
                let y = self.0[1];
                let z = self.0[2];
                let w = self.0[3];

                self.0[0] = x * len;
                self.0[1] = y * len;
                self.0[2] = z * len;
                self.0[3] = w * len;
                self
            }

            #[inline(always)]
            pub fn invert(&self) -> Self {
                let a0 = self.0[0];
                let a1 = self.0[1];
                let a2 = self.0[2];
                let a3 = self.0[3];

                let dot = a0 * a0 + a1 * a1 + a2 * a2 + a3 * a3;
                let inv_dot = if dot != $zero {
                    $one / dot
                } else {
                    $zero
                };

                Self::new(-a0 * inv_dot, -a1 * inv_dot, -a2 * inv_dot, a3 * inv_dot)
            }

            #[inline(always)]
            pub fn invert_in_place(&mut self) -> &mut Self {
                let a0 = self.0[0];
                let a1 = self.0[1];
                let a2 = self.0[2];
                let a3 = self.0[3];

                let dot = a0 * a0 + a1 * a1 + a2 * a2 + a3 * a3;
                let inv_dot = if dot != $zero {
                    $one / dot
                } else {
                    $zero
                };

                self.0[0] = -a0 * inv_dot;
                self.0[1] = -a1 * inv_dot;
                self.0[2] = -a2 * inv_dot;
                self.0[3] = a3 * inv_dot;
                self
            }

            #[inline(always)]
            pub fn conjugate(&self) -> Self {
                let a0 = self.0[0];
                let a1 = self.0[1];
                let a2 = self.0[2];
                let a3 = self.0[3];

                Self::new(-a0, -a1, -a2, a3)
            }

            #[inline(always)]
            pub fn conjugate_in_place(&mut self) -> &mut Self {
                let a0 = self.0[0];
                let a1 = self.0[1];
                let a2 = self.0[2];
                let a3 = self.0[3];

                self.0[0] = -a0;
                self.0[1] = -a1;
                self.0[2] = -a2;
                self.0[3] = a3;
                self
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
            pub fn set_identify(&mut self) -> &mut Self {
                self.0[0] = $zero;
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
        }

        impl ApproximateEq for Quat<$t> {
            #[inline(always)]
            fn approximate_eq(&self, other: &Self) -> bool {
                self.dot(other).abs() >= $one - $epsilon
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
    (f32, 0.0f32, 0.5f32, 1.0f32, 2.0f32, 360.0f32, 0.999999f32, 0.00001f32, std::f32::consts::PI),
    (f64, 0.0f64, 0.5f64, 1.0f64, 2.0f64, 360.0f64, 0.999999f64, 0.00001f64, std::f64::consts::PI)
}
math! {
    (f32, super::EPSILON_F32, 0.0f32, 0.5f32, 1.0f32, 2.0f32, std::f32::consts::PI),
    (f64, super::EPSILON_F64, 0.0f64, 0.5f64, 1.0f64, 2.0f64, std::f64::consts::PI)
}

#[cfg(test)]
mod tests {
    use crate::{error::Error, mat3::Mat3, mat4::Mat4, vec3::Vec3, ApproximateEq};

    use super::Quat;

    #[test]
    fn new() {
        assert_eq!(Quat::new(2.0, 3.0, 4.0, 5.0).raw(), &[2.0, 3.0, 4.0, 5.0]);
    }

    #[test]
    fn new_zero() {
        assert_eq!(Quat::<f64>::new_zero().raw(), &[0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn new_identity() {
        assert_eq!(Quat::<f64>::new_identity().raw(), &[0.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn from_slice() {
        assert_eq!(
            Quat::from_slice([2.0, 3.0, 4.0, 5.0]).raw(),
            &[2.0, 3.0, 4.0, 5.0]
        );
    }

    #[test]
    fn raw() {
        assert_eq!(Quat::new(2.0, 3.0, 4.0, 5.0).raw(), &[2.0, 3.0, 4.0, 5.0]);
    }

    #[test]
    fn raw_mut() {
        let mut quat = Quat::new(2.0, 3.0, 4.0, 5.0);
        (*quat.raw_mut()) = [11.0, 11.0, 11.0, 11.0];
        assert_eq!(quat.raw(), &[11.0, 11.0, 11.0, 11.0]);
    }

    #[test]
    fn x() {
        assert_eq!(Quat::<f64>::new(2.0, 3.0, 4.0, 5.0).x(), &2.0);
    }

    #[test]
    fn y() {
        assert_eq!(Quat::<f64>::new(2.0, 3.0, 4.0, 5.0).y(), &3.0);
    }

    #[test]
    fn z() {
        assert_eq!(Quat::<f64>::new(2.0, 3.0, 4.0, 5.0).z(), &4.0);
    }

    #[test]
    fn w() {
        assert_eq!(Quat::<f64>::new(2.0, 3.0, 4.0, 5.0).w(), &5.0);
    }

    #[test]
    fn x_mut() {
        let mut quat = Quat::new(2.0, 3.0, 4.0, 5.0);
        (*quat.x_mut()) = 6.0;
        assert_eq!(quat.raw(), &[6.0, 3.0, 4.0, 5.0]);
    }

    #[test]
    fn y_mut() {
        let mut quat = Quat::new(2.0, 3.0, 4.0, 5.0);
        (*quat.y_mut()) = 6.0;
        assert_eq!(quat.raw(), &[2.0, 6.0, 4.0, 5.0]);
    }

    #[test]
    fn z_mut() {
        let mut quat = Quat::new(2.0, 3.0, 4.0, 5.0);
        (*quat.z_mut()) = 6.0;
        assert_eq!(quat.raw(), &[2.0, 3.0, 6.0, 5.0]);
    }

    #[test]
    fn w_mut() {
        let mut quat = Quat::new(2.0, 3.0, 4.0, 5.0);
        (*quat.w_mut()) = 6.0;
        assert_eq!(quat.raw(), &[2.0, 3.0, 4.0, 6.0]);
    }

    #[test]
    fn set_x() {
        let mut quat = Quat::new(2.0, 3.0, 4.0, 5.0);
        quat.set_x(6.0);
        assert_eq!(quat.raw(), &[6.0, 3.0, 4.0, 5.0]);
    }

    #[test]
    fn set_y() {
        let mut quat = Quat::new(2.0, 3.0, 4.0, 5.0);
        quat.set_y(6.0);
        assert_eq!(quat.raw(), &[2.0, 6.0, 4.0, 5.0]);
    }

    #[test]
    fn set_z() {
        let mut quat = Quat::new(2.0, 3.0, 4.0, 5.0);
        quat.set_z(6.0);
        assert_eq!(quat.raw(), &[2.0, 3.0, 6.0, 5.0]);
    }

    #[test]
    fn set_w() {
        let mut quat = Quat::new(2.0, 3.0, 4.0, 5.0);
        quat.set_w(6.0);
        assert_eq!(quat.raw(), &[2.0, 3.0, 4.0, 6.0]);
    }

    #[test]
    fn set() {
        let mut quat = Quat::new(2.0, 3.0, 4.0, 5.0);
        quat.set(22.0, 22.0, 22.0, 22.0);
        assert_eq!(quat.raw(), &[22.0, 22.0, 22.0, 22.0]);
    }

    #[test]
    fn set_zero() {
        let mut quat = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        quat.set_zero();
        assert_eq!(quat.raw(), &[0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn set_identify() {
        let mut quat = Quat::<f64>::new(1.0, 2.0, 3.0, 4.00);
        quat.set_identify();
        assert_eq!(quat.raw(), &[0.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn add_quat_quat() {
        let quat0 = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let quat1 = Quat::<f64>::new(5.0, 6.0, 7.0, 8.0);
        assert_eq!(
            (quat0 + quat1).approximate_eq(&Quat::new(6.0, 8.0, 10.0, 12.0)),
            true
        );
    }

    #[test]
    fn add_quat_scalar() {
        let quat = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let scalar = 1.0;
        assert_eq!(
            (quat + scalar).approximate_eq(&Quat::new(2.0, 3.0, 4.0, 5.0)),
            true
        );
    }

    #[test]
    fn add_scalar_quat() {
        let scalar = 1.0;
        let quat = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(
            (scalar + quat).approximate_eq(&Quat::new(2.0, 3.0, 4.0, 5.0)),
            true
        );
    }

    #[test]
    fn add_assign_quat_quat() {
        let mut quat0 = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let quat1 = Quat::<f64>::new(5.0, 6.0, 7.0, 8.0);
        quat0 += quat1;
        assert_eq!(quat0.approximate_eq(&Quat::new(6.0, 8.0, 10.0, 12.0)), true);
    }

    #[test]
    fn add_assign_quat_scalar() {
        let mut quat = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let scalar = 1.0;
        quat += scalar;
        assert_eq!(quat.approximate_eq(&Quat::new(2.0, 3.0, 4.0, 5.0)), true);
    }

    #[test]
    fn sub_quat_quat() {
        let quat0 = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let quat1 = Quat::<f64>::new(5.0, 6.0, 7.0, 8.0);
        assert_eq!(
            (quat0 - quat1).approximate_eq(&Quat::new(-4.0, -4.0, -4.0, -4.0)),
            true
        );
    }

    #[test]
    fn sub_quat_scalar() {
        let quat = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let scalar = 1.0;
        assert_eq!(
            (quat - scalar).approximate_eq(&Quat::new(0.0, 1.0, 2.0, 3.0)),
            true
        );
    }

    #[test]
    fn sub_scalar_quat() {
        let scalar = 1.0;
        let quat = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(
            (scalar - quat).approximate_eq(&Quat::new(0.0, -1.0, -2.0, -3.0)),
            true
        );
    }

    #[test]
    fn sub_assign_quat_quat() {
        let mut quat0 = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let quat1 = Quat::<f64>::new(5.0, 6.0, 7.0, 8.0);
        quat0 -= quat1;
        assert_eq!(
            quat0.approximate_eq(&Quat::new(-4.0, -4.0, -4.0, -4.0)),
            true
        );
    }

    #[test]
    fn sub_assign_quat_scalar() {
        let mut quat = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let scalar = 1.0;
        quat -= scalar;
        assert_eq!(quat.approximate_eq(&Quat::new(0.0, 1.0, 2.0, 3.0)), true);
    }

    #[test]
    fn mul_quat_quat() {
        let quat0 = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let quat1 = Quat::<f64>::new(5.0, 6.0, 7.0, 8.0);
        assert_eq!(
            (quat0 * quat1).approximate_eq(&Quat::new(24.0, 48.0, 48.0, -6.0)),
            true
        );
    }

    #[test]
    fn mul_quat_scalar() {
        let quat = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let scalar = 2.0;
        assert_eq!(
            (quat * scalar).approximate_eq(&Quat::new(2.0, 4.0, 6.0, 8.0)),
            true
        );
    }

    #[test]
    fn mul_scalar_quat() {
        let scalar = 3.0;
        let quat = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(
            (scalar * quat).approximate_eq(&Quat::new(3.0, 6.0, 9.0, 12.0)),
            true
        );
    }

    #[test]
    fn mul_assign_quat_quat() {
        let mut quat0 = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let quat1 = Quat::<f64>::new(5.0, 6.0, 7.0, 8.0);
        quat0 *= quat1;
        assert_eq!(
            quat0.approximate_eq(&Quat::new(5.0, 12.0, 21.0, 32.0)),
            true
        );
    }

    #[test]
    fn mul_assign_quat_scalar() {
        let mut quat = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let scalar = 4.0;
        quat *= scalar;
        assert_eq!(quat.approximate_eq(&Quat::new(4.0, 8.0, 12.0, 16.0)), true);
    }

    #[test]
    fn div_quat_scalar() {
        let quat = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let scalar = 2.0;
        assert_eq!(
            (quat / scalar).approximate_eq(&Quat::new(0.5, 1.0, 1.5, 2.0)),
            true
        );
    }

    #[test]
    fn div_scalar_quat() {
        let scalar = 2.0;
        let quat = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(
            (scalar / quat).approximate_eq(&Quat::new(2.0, 1.0, 0.6666666666666667, 0.5)),
            true
        );
    }

    #[test]
    fn div_assign_quat_scalar() {
        let mut quat = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let scalar = 4.0;
        quat /= scalar;
        assert_eq!(quat.approximate_eq(&Quat::new(0.25, 0.5, 0.75, 1.0)), true);
    }

    #[test]
    fn from_mat3() -> Result<(), Error> {
        let mat = Mat3::<f64>::new(1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0);
        let quat = Quat::<f64>::from_mat3(&mat);
        assert_eq!(
            quat.approximate_eq(&Quat::new(
                -0.7071067811865475,
                0.0,
                0.0,
                0.7071067811865476
            )),
            true
        );

        let vec = Vec3::<f64>::new(0.0, 1.0, 0.0);
        assert_eq!(
            (vec * quat).approximate_eq(&Vec3::new(0.0, 0.0, -1.0)),
            true
        );

        let mat = Mat3::<f64>::from_mat4(&Mat4::<f64>::from_look_at(
            &Vec3::new(0.0, 0.0, 0.0),
            &Vec3::new(0.0, 0.0, 1.0),
            &Vec3::new(0.0, 1.0, 0.0),
        ))
        .invert()?
        .transpose();
        let quat = Quat::<f64>::from_mat3(&mat).normalize();
        let vec = Vec3::<f64>::new(3.0, 2.0, -1.0);
        assert_eq!((quat * vec).approximate_eq(&(mat * vec)), true);

        let mat = Mat3::<f64>::from_mat4(&Mat4::<f64>::from_look_at(
            &Vec3::new(0.0, 0.0, 0.0),
            &Vec3::new(-1.0, 0.0, 0.0),
            &Vec3::new(0.0, -1.0, 0.0),
        ))
        .invert()?
        .transpose();
        let quat = Quat::<f64>::from_mat3(&mat).normalize();
        let vec = Vec3::<f64>::new(3.0, 2.0, -1.0);
        assert_eq!(
            (quat * vec).approximate_eq(&Vec3::new(-1.0, -2.0, 3.0)),
            true
        );

        let mat = Mat3::<f64>::from_mat4(&Mat4::<f64>::from_look_at(
            &Vec3::new(0.0, 0.0, 0.0),
            &Vec3::new(0.0, 0.0, -1.0),
            &Vec3::new(0.0, -1.0, 0.0),
        ))
        .invert()?
        .transpose();
        let quat = Quat::<f64>::from_mat3(&mat).normalize();
        let vec = Vec3::<f64>::new(3.0, 2.0, -1.0);
        assert_eq!((quat * vec).approximate_eq(&(mat * vec)), true);

        Ok(())
    }

    #[test]
    fn from_rotate_to() {
        assert_eq!(
            Quat::<f64>::from_rotation_to(&Vec3::new(0.0, 1.0, 0.0), &Vec3::new(1.0, 0.0, 0.0))
                .approximate_eq(&Quat::new(
                    0.0,
                    0.0,
                    -0.7071067811865475,
                    0.7071067811865475
                )),
            true
        );

        let quat =
            Quat::<f64>::from_rotation_to(&Vec3::new(0.0, 1.0, 0.0), &Vec3::new(0.0, 1.0, 0.0));
        let vec = Vec3::<f64>::new(0.0, 1.0, 0.0);
        assert_eq!((quat * vec).approximate_eq(&Vec3::new(0.0, 1.0, 0.0)), true);

        let quat =
            Quat::<f64>::from_rotation_to(&Vec3::new(1.0, 0.0, 0.0), &Vec3::new(-1.0, 0.0, 0.0));
        let vec = Vec3::<f64>::new(1.0, 0.0, 0.0);
        assert_eq!(
            (quat * vec).approximate_eq(&Vec3::new(-1.0, 0.0, 0.0)),
            true
        );

        let quat =
            Quat::<f64>::from_rotation_to(&Vec3::new(0.0, 1.0, 0.0), &Vec3::new(0.0, -1.0, 0.0));
        let vec = Vec3::<f64>::new(0.0, 1.0, 0.0);
        assert_eq!(
            (quat * vec).approximate_eq(&Vec3::new(0.0, -1.0, 0.0)),
            true
        );

        let quat =
            Quat::<f64>::from_rotation_to(&Vec3::new(0.0, 0.0, 1.0), &Vec3::new(0.0, 0.0, -1.0));
        let vec = Vec3::<f64>::new(0.0, 0.0, 1.0);
        assert_eq!(
            (quat * vec).approximate_eq(&Vec3::new(0.0, 0.0, -1.0)),
            true
        );
    }

    #[test]
    fn from_axes() {
        let quat = Quat::<f64>::from_axes(
            &Vec3::new(-1.0, 0.0, 0.0),
            &Vec3::new(0.0, 0.0, -1.0),
            &Vec3::new(0.0, 1.0, 0.0),
        );
        let vec = Vec3::<f64>::new(0.0, 0.0, -1.0);
        assert_eq!((quat * vec).approximate_eq(&Vec3::new(1.0, 0.0, 0.0)), true);

        let vec = Vec3::<f64>::new(1.0, 0.0, 0.0);
        assert_eq!((quat * vec).approximate_eq(&Vec3::new(0.0, 0.0, 1.0)), true);

        let quat = Quat::<f64>::from_axes(
            &Vec3::new(0.0, 0.0, -1.0),
            &Vec3::new(1.0, 0.0, 0.0),
            &Vec3::new(0.0, 1.0, 0.0),
        );
        assert_eq!(quat.approximate_eq(&Quat::new(0.0, 0.0, 0.0, 1.0)), true);
    }

    #[test]
    fn from_euler() {
        assert_eq!(
            Quat::<f64>::from_euler(-30.0, 30.0, 30.0).approximate_eq(&Quat::new(
                -0.30618621784789724,
                0.17677669529663687,
                0.30618621784789724,
                0.8838834764831845
            )),
            true
        );

        let quat = Quat::<f64>::from_euler(-90.0, 0.0, 0.0);
        let vec = Vec3::<f64>::new(0.0, 1.0, 0.0);
        assert_eq!(
            (quat * vec).approximate_eq(&Vec3::new(0.0, 0.0, -1.0)),
            true
        );
    }

    #[test]
    fn from_axis_angle() {
        assert_eq!(
            Quat::<f64>::from_axis_angle(&Vec3::new(1.0, 0.0, 0.0), std::f64::consts::PI * 0.5)
                .approximate_eq(&Quat::new(0.7071067811865475, 0.0, 0.0, 0.7071067811865476)),
            true
        );
    }

    #[test]
    fn squared_length() {
        let quat = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(quat.squared_length().approximate_eq(&30.0), true);
    }

    #[test]
    fn length() {
        let quat = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(quat.length().approximate_eq(&5.477225575051661), true);
    }

    #[test]
    fn dot() {
        let quat0 = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let quat1 = Quat::<f64>::new(5.0, 6.0, 7.0, 8.0);
        assert_eq!(quat0.dot(&quat1).approximate_eq(&70.0), true);
    }

    #[test]
    fn lerp() {
        let quat0 = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let quat1 = Quat::<f64>::new(5.0, 6.0, 7.0, 8.0);
        assert_eq!(
            quat0
                .lerp(&quat1, 0.5)
                .approximate_eq(&Quat::new(3.0, 4.0, 5.0, 6.0)),
            true
        );
    }

    #[test]
    fn slerp() {
        let quat0 = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let quat1 = Quat::<f64>::new(5.0, 6.0, 7.0, 8.0);
        assert_eq!(
            quat0
                .slerp(&quat1, 0.5)
                .approximate_eq(&Quat::new(3.0, 4.0, 5.0, 6.0)),
            true
        );
    }

    #[test]
    fn conjugate() {
        let quat = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(
            quat.conjugate()
                .approximate_eq(&Quat::new(-1.0, -2.0, -3.0, 4.0)),
            true
        );
    }

    #[test]
    fn conjugate_in_place() {
        let mut quat = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        quat.conjugate_in_place();
        assert_eq!(quat.approximate_eq(&Quat::new(-1.0, -2.0, -3.0, 4.0)), true);
    }

    #[test]
    fn normalize() {
        let quat = Quat::<f64>::new(5.0, 0.0, 0.0, 0.0);
        assert_eq!(
            quat.normalize()
                .approximate_eq(&Quat::new(1.0, 0.0, 0.0, 0.0)),
            true
        );
    }

    #[test]
    fn normalize_in_place() {
        let mut quat = Quat::<f64>::new(5.0, 0.0, 0.0, 0.0);
        quat.normalize_in_place();
        assert_eq!(quat.approximate_eq(&Quat::new(1.0, 0.0, 0.0, 0.0)), true);
    }

    #[test]
    fn angle() {
        let mut quat0 = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        quat0.normalize_in_place();
        assert_eq!(quat0.angle(&quat0).approximate_eq(&0.0), true);

        // precision consider
        assert_eq!(
            quat0.angle(&quat0.rotate_x(std::f64::consts::PI / 4.0)),
            0.7853981633974491
        );

        // precision consider
        let mut quat1 = Quat::<f64>::new(5.0, 6.0, 7.0, 8.0);
        quat1.normalize_in_place();
        assert_eq!(quat0.angle(&quat1), 0.5003918408450226);
    }

    #[test]
    fn invert() {
        let quat = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        let iquat = quat.invert();
        assert_eq!(iquat.x().approximate_eq(&-0.033333), true);
        assert_eq!(iquat.y().approximate_eq(&-0.066666), true);
        assert_eq!(iquat.z().approximate_eq(&-0.1), true);
        assert_eq!(iquat.w().approximate_eq(&0.133333), true);
    }

    #[test]
    fn invert_in_place() {
        let mut quat = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        quat.invert_in_place();
        assert_eq!(quat.x().approximate_eq(&-0.033333), true);
        assert_eq!(quat.y().approximate_eq(&-0.066666), true);
        assert_eq!(quat.z().approximate_eq(&-0.1), true);
        assert_eq!(quat.w().approximate_eq(&0.133333), true);
    }

    #[test]
    fn pow() {
        // identity quat
        assert_eq!(
            Quat::<f64>::new_identity()
                .pow(2.1)
                .approximate_eq(&Quat::<f64>::new_identity()),
            true
        );

        let quat = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);

        // power of one
        assert_eq!(
            quat.normalize().pow(1.0).length().approximate_eq(&1.0),
            true
        );

        // squared
        assert_eq!(
            quat.normalize().pow(2.0).approximate_eq(&Quat::new(
                0.26666666666666666,
                0.5333333333333333,
                0.7999999999999999,
                0.06666666666666689
            )),
            true
        );
        assert_eq!(
            quat.normalize().pow(2.0).length().approximate_eq(&1.0),
            true
        );

        // conjugate
        assert_eq!(
            quat.normalize()
                .pow(-1.0)
                .approximate_eq(&quat.normalize().conjugate()),
            true
        );
        assert_eq!(
            quat.normalize().pow(-1.0).length().approximate_eq(&1.0),
            true
        );

        // reversible
        assert_eq!(
            quat.normalize()
                .pow(2.1)
                .pow(1.0 / 2.1)
                .approximate_eq(&quat.normalize()),
            true
        );
        assert_eq!(
            quat.normalize()
                .pow(2.1)
                .pow(1.0 / 2.1)
                .length()
                .approximate_eq(&1.0),
            true
        );
    }

    #[test]
    fn pow_in_place() {
        // power of one
        let mut quat = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        quat.normalize_in_place();
        quat.pow_in_place(1.0);
        assert_eq!(quat.length().approximate_eq(&1.0), true);

        // squared
        let mut quat = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        quat.normalize_in_place();
        quat.pow_in_place(2.0);
        assert_eq!(
            quat.approximate_eq(&Quat::new(
                0.26666666666666666,
                0.5333333333333333,
                0.7999999999999999,
                0.06666666666666689
            )),
            true
        );
        assert_eq!(quat.length().approximate_eq(&1.0), true);

        // conjugate
        let mut quat = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        quat.normalize_in_place();
        quat.pow_in_place(-1.0);
        assert_eq!(
            quat.approximate_eq(&Quat::<f64>::new(1.0, 2.0, 3.0, 4.0).normalize().conjugate()),
            true
        );
        assert_eq!(quat.length().approximate_eq(&1.0), true);

        // reversible
        let mut quat = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        quat.normalize_in_place();
        quat.pow_in_place(2.1);
        quat.pow_in_place(1.0 / 2.1);
        assert_eq!(
            quat.approximate_eq(&Quat::<f64>::new(1.0, 2.0, 3.0, 4.0).normalize()),
            true
        );
        assert_eq!(quat.length().approximate_eq(&1.0), true);
    }

    #[test]
    fn axis_angle() {
        let quat = Quat::<f64>::from_axis_angle(&Vec3::new(0.0, 1.0, 0.0), 0.0);
        let (_, rad) = quat.axis_angle();
        assert_eq!(
            (rad % (std::f64::consts::PI * 2.0)).approximate_eq(&0.0),
            true
        );

        let quat = Quat::<f64>::from_axis_angle(&Vec3::new(1.0, 0.0, 0.0), 0.7778);
        let (vec3, rad) = quat.axis_angle();
        assert_eq!(vec3.approximate_eq(&Vec3::new(1.0, 0.0, 0.0)), true);
        assert_eq!(rad.approximate_eq(&0.7778), true);

        let quat = Quat::<f64>::from_axis_angle(&Vec3::new(0.0, 1.0, 0.0), 0.879546);
        let (vec3, rad) = quat.axis_angle();
        assert_eq!(vec3.approximate_eq(&Vec3::new(0.0, 1.0, 0.0)), true);
        assert_eq!(rad.approximate_eq(&0.8795459999999998), true);

        let quat = Quat::<f64>::from_axis_angle(&Vec3::new(0.0, 0.0, 1.0), 0.123456);
        let (vec3, rad) = quat.axis_angle();
        assert_eq!(vec3.approximate_eq(&Vec3::new(0.0, 0.0, 1.0)), true);
        assert_eq!(rad.approximate_eq(&0.1234559999999983), true);

        let quat = Quat::<f64>::from_axis_angle(
            &Vec3::new(0.707106, 0.0, 0.707106),
            std::f64::consts::PI * 0.5,
        );
        let (vec3, rad) = quat.axis_angle();
        assert_eq!(
            vec3.approximate_eq(&Vec3::new(0.707106, 0.0, 0.707106)),
            true
        );
        assert_eq!(rad.approximate_eq(&(std::f64::consts::PI * 0.5)), true);

        let quat =
            Quat::<f64>::from_axis_angle(&Vec3::new(0.65538555, 0.49153915, 0.57346237), 8.8888);
        let (_, rad) = quat.axis_angle();
        assert_eq!(
            quat.approximate_eq(&Quat::new(-0.63199174, -0.4739938, -0.55299276, -0.2647929)),
            true
        );
        assert_eq!(rad > 0.0, true);
        assert_eq!(rad < std::f64::consts::PI * 2.0, true);
    }

    #[test]
    fn rotate_x() {
        let vec = Vec3::<f64>::new(0.0, 0.0, -1.0);
        let quat = Quat::<f64>::new_identity();
        let quat = quat.rotate_x(90.0f64.to_radians());

        assert_eq!((vec * quat).approximate_eq(&Vec3::new(0.0, 1.0, 0.0)), true);
    }

    #[test]
    fn rotate_x_in_place() {
        let vec = Vec3::<f64>::new(0.0, 0.0, -1.0);
        let mut quat = Quat::<f64>::new_identity();
        quat.rotate_x_in_place(90.0f64.to_radians());

        assert_eq!((vec * quat).approximate_eq(&Vec3::new(0.0, 1.0, 0.0)), true);
    }

    #[test]
    fn rotate_y() {
        let vec = Vec3::<f64>::new(0.0, 0.0, -1.0);
        let quat = Quat::<f64>::new_identity();
        let quat = quat.rotate_y(90.0f64.to_radians());

        assert_eq!(
            (vec * quat).approximate_eq(&Vec3::new(-1.0, 0.0, 0.0)),
            true
        );
    }

    #[test]
    fn rotate_y_in_place() {
        let vec = Vec3::<f64>::new(0.0, 0.0, -1.0);
        let mut quat = Quat::<f64>::new_identity();
        quat.rotate_y_in_place(90.0f64.to_radians());

        assert_eq!(
            (vec * quat).approximate_eq(&Vec3::new(-1.0, 0.0, 0.0)),
            true
        );
    }

    #[test]
    fn rotate_z() {
        let vec = Vec3::<f64>::new(0.0, 1.0, 0.0);
        let quat = Quat::<f64>::new_identity();
        let quat = quat.rotate_z(90.0f64.to_radians());

        assert_eq!(
            (vec * quat).approximate_eq(&Vec3::new(-1.0, 0.0, 0.0)),
            true
        );
    }

    #[test]
    fn rotate_z_in_place() {
        let vec = Vec3::<f64>::new(0.0, 1.0, 0.0);
        let mut quat = Quat::<f64>::new_identity();
        quat.rotate_z_in_place(90.0f64.to_radians());

        assert_eq!(
            (vec * quat).approximate_eq(&Vec3::new(-1.0, 0.0, 0.0)),
            true
        );
    }

    #[test]
    fn display() {
        let quat = Quat::<f64>::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(quat.to_string(), "quat(1, 2, 3, 4)");
    }
}
