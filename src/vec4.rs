use std::{
    fmt::Display,
    ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub, SubAssign},
};

use half::f16;
use num_traits::Float;

use crate::{epsilon, mat4::AsMat4, quat::AsQuat};

pub trait AsVec4<T: Float> {
    fn from_values(x: T, y: T, z: T, w: T) -> Self;

    fn x(&self) -> T;

    fn y(&self) -> T;

    fn z(&self) -> T;

    fn w(&self) -> T;

    fn set_x(&mut self, x: T) -> &mut Self;

    fn set_y(&mut self, y: T) -> &mut Self;

    fn set_z(&mut self, z: T) -> &mut Self;

    fn set_w(&mut self, w: T) -> &mut Self;

    #[inline(always)]
    fn to_raw(&self) -> [T; 4] {
        [self.x(), self.y(), self.z(), self.w()]
    }

    #[inline(always)]
    fn to_gl(&self) -> [f32; 4] {
        [
            self.x().to_f32().unwrap(),
            self.y().to_f32().unwrap(),
            self.z().to_f32().unwrap(),
            self.w().to_f32().unwrap(),
        ]
    }

    #[inline(always)]
    fn to_gl_binary(&self) -> [u8; 16] {
        unsafe { std::mem::transmute_copy::<[f32; 4], [u8; 16]>(&self.to_gl()) }
    }

    #[inline(always)]
    fn copy<V: AsVec4<T> + ?Sized>(&mut self, b: &V) -> &mut Self {
        self.set_x(b.x()).set_y(b.y()).set_z(b.z()).set_w(b.w())
    }

    #[inline(always)]
    fn set(&mut self, x: T, y: T, z: T, w: T) -> &mut Self {
        self.set_x(x).set_y(y).set_z(z).set_w(w)
    }

    #[inline(always)]
    fn set_slice(&mut self, [x, y, z, w]: &[T; 4]) -> &mut Self {
        self.set_x(*x).set_y(*y).set_z(*z).set_w(*w)
    }

    #[inline(always)]
    fn set_zero(&mut self) -> &mut Self {
        self.set_x(T::zero())
            .set_y(T::zero())
            .set_z(T::zero())
            .set_w(T::zero())
    }

    #[inline(always)]
    fn ceil(&self) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();
        let z = self.z();
        let w = self.w();

        Self::from_values(x.ceil(), y.ceil(), z.ceil(), w.ceil())
    }

    #[inline(always)]
    fn floor(&self) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();
        let z = self.z();
        let w = self.w();

        Self::from_values(x.floor(), y.floor(), z.floor(), w.floor())
    }

    #[inline(always)]
    fn round(&self) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();
        let z = self.z();
        let w = self.w();

        Self::from_values(x.round(), y.round(), z.round(), w.round())
    }

    #[inline(always)]
    fn min<V: AsVec4<T> + ?Sized>(&self, b: &V) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();
        let z = self.z();
        let w = self.w();

        Self::from_values(x.min(b.x()), y.min(b.y()), z.min(b.z()), w.min(b.w()))
    }

    #[inline(always)]
    fn max<V: AsVec4<T> + ?Sized>(&self, b: &V) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();
        let z = self.z();
        let w = self.w();

        Self::from_values(x.max(b.x()), y.max(b.y()), z.max(b.z()), w.max(b.w()))
    }

    #[inline(always)]
    fn scale(&self, scale: T) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();
        let z = self.z();
        let w = self.w();

        Self::from_values(x * scale, y * scale, z * scale, w * scale)
    }

    #[inline(always)]
    fn squared_distance<V: AsVec4<T> + ?Sized>(&self, b: &V) -> T {
        let x = b.x() - self.x();
        let y = b.y() - self.y();
        let z = b.z() - self.z();
        let w = b.w() - self.w();

        x * x + y * y + z * z + w * w
    }

    #[inline(always)]
    fn distance<V: AsVec4<T> + ?Sized>(&self, b: &V) -> T {
        self.squared_distance(b).sqrt()
    }

    #[inline(always)]
    fn squared_length(&self) -> T {
        let x = self.x();
        let y = self.y();
        let z = self.z();
        let w = self.w();
        x * x + y * y + z * z + w * w
    }

    #[inline(always)]
    fn length(&self) -> T {
        self.squared_length().sqrt()
    }

    #[inline(always)]
    fn negate(&self) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();
        let z = self.z();
        let w = self.w();

        Self::from_values(x.neg(), y.neg(), z.neg(), w.neg())
    }

    #[inline(always)]
    fn inverse(&self) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();
        let z = self.z();
        let w = self.w();

        Self::from_values(T::one() / x, T::one() / y, T::one() / z, T::one() / w)
    }

    #[inline(always)]
    fn normalize(&self) -> Self
    where
        Self: Sized,
    {
        let mut len = self.squared_length();
        if len > T::zero() {
            len = T::one() / len.sqrt();
        }

        let x = self.x();
        let y = self.y();
        let z = self.z();
        let w = self.w();

        Self::from_values(x * len, y * len, z * len, w * len)
    }

    #[inline(always)]
    fn dot<V: AsVec4<T> + ?Sized>(&self, b: &V) -> T {
        self.x() * b.x() + self.y() * b.y() + self.z() * b.z() + self.w() * b.w()
    }

    #[inline(always)]
    fn cross<V: AsVec4<T> + ?Sized, W: AsVec4<T> + ?Sized>(&self, v: &V, w: &W) -> Self
    where
        Self: Sized,
    {
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

        Self::from_values(
            h * f - i * e + j * d,
            -(g * f) + i * c - j * b,
            g * e - h * c + j * a,
            -(g * d) + h * b - i * a,
        )
    }

    #[inline(always)]
    fn lerp<V: AsVec4<T> + ?Sized>(&self, b: &V, t: T) -> Self
    where
        Self: Sized,
    {
        let ax = self.x();
        let ay = self.y();
        let az = self.z();
        let aw = self.w();

        Self::from_values(
            ax + t * (b.x() - ax),
            ay + t * (b.y() - ay),
            az + t * (b.z() - az),
            aw + t * (b.w() - aw),
        )
    }

    #[inline(always)]
    fn transform_mat4<M: AsMat4<T> + ?Sized>(&self, m: &M) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();
        let z = self.z();
        let w = self.w();

        let a00 = m.m00();
        let a01 = m.m01();
        let a02 = m.m02();
        let a03 = m.m03();
        let a10 = m.m10();
        let a11 = m.m11();
        let a12 = m.m12();
        let a13 = m.m13();
        let a20 = m.m20();
        let a21 = m.m21();
        let a22 = m.m22();
        let a23 = m.m23();
        let a30 = m.m30();
        let a31 = m.m31();
        let a32 = m.m32();
        let a33 = m.m33();

        Self::from_values(
            a00 * x + a10 * y + a20 * z + a30 * w,
            a01 * x + a11 * y + a21 * z + a31 * w,
            a02 * x + a12 * y + a22 * z + a32 * w,
            a03 * x + a13 * y + a23 * z + a33 * w,
        )
    }

    #[inline(always)]
    fn transform_quat<Q: AsQuat<T> + ?Sized>(&self, q: &Q) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();
        let z = self.z();
        let w = self.w();
        let qx = q.x();
        let qy = q.y();
        let qz = q.z();
        let qw = q.w();

        // calculate quat * vec
        let ix = qw * x + qy * z - qz * y;
        let iy = qw * y + qz * x - qx * z;
        let iz = qw * z + qx * y - qy * x;
        let iw = -qx * x - qy * y - qz * z;

        Self::from_values(
            ix * qw + iw * -qx + iy * -qz - iz * -qy,
            iy * qw + iw * -qy + iz * -qx - ix * -qz,
            iz * qw + iw * -qz + ix * -qy - iy * -qx,
            w,
        )
    }

    #[inline(always)]
    fn approximate_eq<V: AsVec4<T> + ?Sized>(&self, b: &V) -> bool {
        let a0 = self.x();
        let a1 = self.y();
        let a2 = self.z();
        let a3 = self.w();
        let b0 = b.x();
        let b1 = b.y();
        let b2 = b.z();
        let b3 = b.w();

        (a0 - b0).abs() <= epsilon::<T>() * T::one().max(a0.abs()).max(b0.abs())
            && (a1 - b1).abs() <= epsilon::<T>() * T::one().max(a1.abs()).max(b1.abs())
            && (a2 - b2).abs() <= epsilon::<T>() * T::one().max(a2.abs()).max(b2.abs())
            && (a3 - b3).abs() <= epsilon::<T>() * T::one().max(a3.abs()).max(b3.abs())
    }
}

impl<T: Float> AsVec4<T> for [T; 4] {
    #[inline(always)]
    fn from_values(x: T, y: T, z: T, w: T) -> Self {
        [x, y, z, w]
    }

    #[inline(always)]
    fn x(&self) -> T {
        self[0]
    }

    #[inline(always)]
    fn y(&self) -> T {
        self[1]
    }

    #[inline(always)]
    fn z(&self) -> T {
        self[2]
    }

    #[inline(always)]
    fn w(&self) -> T {
        self[3]
    }

    #[inline(always)]
    fn set_x(&mut self, x: T) -> &mut Self {
        self[0] = x;
        self
    }

    #[inline(always)]
    fn set_y(&mut self, y: T) -> &mut Self {
        self[1] = y;
        self
    }

    #[inline(always)]
    fn set_z(&mut self, z: T) -> &mut Self {
        self[2] = z;
        self
    }

    #[inline(always)]
    fn set_w(&mut self, w: T) -> &mut Self {
        self[3] = w;
        self
    }
}

impl<T: Float> AsVec4<T> for (T, T, T, T) {
    #[inline(always)]
    fn from_values(x: T, y: T, z: T, w: T) -> Self {
        (x, y, z, w)
    }

    #[inline(always)]
    fn x(&self) -> T {
        self.0
    }

    #[inline(always)]
    fn y(&self) -> T {
        self.1
    }

    #[inline(always)]
    fn z(&self) -> T {
        self.2
    }

    #[inline(always)]
    fn w(&self) -> T {
        self.3
    }

    #[inline(always)]
    fn set_x(&mut self, x: T) -> &mut Self {
        self.0 = x;
        self
    }

    #[inline(always)]
    fn set_y(&mut self, y: T) -> &mut Self {
        self.1 = y;
        self
    }

    #[inline(always)]
    fn set_z(&mut self, z: T) -> &mut Self {
        self.2 = z;
        self
    }

    #[inline(always)]
    fn set_w(&mut self, w: T) -> &mut Self {
        self.3 = w;
        self
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Vec4<T = f64>(pub [T; 4]);

impl<T: Float> Vec4<T> {
    #[inline(always)]
    pub const fn from_values(x: T, y: T, z: T, w: T) -> Self {
        Self([x, y, z, w])
    }

    #[inline(always)]
    pub fn new() -> Self {
        Self([T::zero(); 4])
    }

    #[inline(always)]
    pub fn from_slice(slice: [T; 4]) -> Self {
        Self(slice)
    }

    #[inline(always)]
    pub fn from_as_vec4<V: AsVec4<T>>(v: V) -> Self {
        Self(v.to_raw())
    }

    #[inline(always)]
    pub fn raw(&self) -> &[T; 4] {
        &self.0
    }
}

impl<T: Float> AsVec4<T> for Vec4<T> {
    #[inline(always)]
    fn from_values(x: T, y: T, z: T, w: T) -> Self {
        Self([x, y, z, w])
    }

    #[inline(always)]
    fn x(&self) -> T {
        self.0[0]
    }

    #[inline(always)]
    fn y(&self) -> T {
        self.0[1]
    }

    #[inline(always)]
    fn z(&self) -> T {
        self.0[2]
    }

    #[inline(always)]
    fn w(&self) -> T {
        self.0[3]
    }

    #[inline(always)]
    fn set_x(&mut self, x: T) -> &mut Self {
        self.0[0] = x;
        self
    }

    #[inline(always)]
    fn set_y(&mut self, y: T) -> &mut Self {
        self.0[1] = y;
        self
    }

    #[inline(always)]
    fn set_z(&mut self, z: T) -> &mut Self {
        self.0[2] = z;
        self
    }

    #[inline(always)]
    fn set_w(&mut self, w: T) -> &mut Self {
        self.0[3] = w;
        self
    }
}

impl Vec4<f64> {
    #[inline(always)]
    pub fn random(&self, scale: Option<f64>) -> Self {
        let scale = match scale {
            Some(scale) => scale,
            None => 1.0,
        };

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

        rand = rand::random::<f64>();
        v1 = rand * 2.0 - 1.0;
        v2 = (4.0 * rand::random::<f64>() - 2.0) * (rand * -rand + rand).sqrt();
        s1 = v1 * v1 + v2 * v2;

        rand = rand::random::<f64>();
        v3 = rand * 2.0 - 1.0;
        v4 = (4.0 * rand::random::<f64>() - 2.0) * (rand * -rand + rand).sqrt();
        s2 = v3 * v3 + v4 * v4;

        let d = ((1.0 - s1) / s2).sqrt();

        Self([scale * v1, scale * v2, scale * v3 * d, scale * v4 * d])
    }
}

impl Vec4<f32> {
    #[inline(always)]
    pub fn random(&self, scale: Option<f32>) -> Self {
        let scale = match scale {
            Some(scale) => scale,
            None => 1.0,
        };

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

        rand = rand::random::<f32>();
        v1 = rand * 2.0 - 1.0;
        v2 = (4.0 * rand::random::<f32>() - 2.0) * (rand * -rand + rand).sqrt();
        s1 = v1 * v1 + v2 * v2;

        rand = rand::random::<f32>();
        v3 = rand * 2.0 - 1.0;
        v4 = (4.0 * rand::random::<f32>() - 2.0) * (rand * -rand + rand).sqrt();
        s2 = v3 * v3 + v4 * v4;

        let d = ((1.0 - s1) / s2).sqrt();

        Self([scale * v1, scale * v2, scale * v3 * d, scale * v4 * d])
    }
}

impl Vec4<f16> {
    #[inline(always)]
    pub fn random(&self, scale: Option<f16>) -> Self {
        let scale = match scale {
            Some(scale) => scale,
            None => f16::from_f32_const(1.0),
        };

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

        rand = rand::random::<f16>();
        v1 = rand * f16::from_f32_const(2.0) - f16::from_f32_const(1.0);
        v2 = (f16::from_f32_const(4.0) * rand::random::<f16>() - f16::from_f32_const(2.0))
            * (rand * -rand + rand).sqrt();
        s1 = v1 * v1 + v2 * v2;

        rand = rand::random::<f16>();
        v3 = rand * f16::from_f32_const(2.0) - f16::from_f32_const(1.0);
        v4 = (f16::from_f32_const(4.0) * rand::random::<f16>() - f16::from_f32_const(2.0))
            * (rand * -rand + rand).sqrt();
        s2 = v3 * v3 + v4 * v4;

        let d = ((f16::from_f32_const(1.0) - s1) / s2).sqrt();

        Self([scale * v1, scale * v2, scale * v3 * d, scale * v4 * d])
    }
}

impl<T: Float> Default for Vec4<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Float, V: AsVec4<T>> Add<V> for Vec4<T> {
    type Output = Self;

    #[inline(always)]
    fn add(self, b: V) -> Self::Output {
        Self([
            self.0[0] + b.x(),
            self.0[1] + b.y(),
            self.0[2] + b.z(),
            self.0[3] + b.w(),
        ])
    }
}

impl<T: Float, V: AsVec4<T>> Sub<V> for Vec4<T> {
    type Output = Self;

    #[inline(always)]
    fn sub(self, b: V) -> Self::Output {
        Self([
            self.0[0] - b.x(),
            self.0[1] - b.y(),
            self.0[2] - b.z(),
            self.0[3] - b.w(),
        ])
    }
}

impl<T: Float, V: AsVec4<T>> Mul<V> for Vec4<T> {
    type Output = Self;

    #[inline(always)]
    fn mul(self, b: V) -> Self::Output {
        Self([
            self.0[0] * b.x(),
            self.0[1] * b.y(),
            self.0[2] * b.z(),
            self.0[3] * b.w(),
        ])
    }
}

impl<T: Float, V: AsVec4<T>> Div<V> for Vec4<T> {
    type Output = Self;

    #[inline(always)]
    fn div(self, b: V) -> Self::Output {
        Self([
            self.0[0] / b.x(),
            self.0[1] / b.y(),
            self.0[2] / b.z(),
            self.0[3] / b.w(),
        ])
    }
}

impl<T: Float + AddAssign, V: AsVec4<T>> AddAssign<V> for Vec4<T> {
    fn add_assign(&mut self, rhs: V) {
        self.0[0] += rhs.x();
        self.0[1] += rhs.y();
        self.0[2] += rhs.z();
        self.0[3] += rhs.w();
    }
}

impl<T: Float + SubAssign, V: AsVec4<T>> SubAssign<V> for Vec4<T> {
    fn sub_assign(&mut self, rhs: V) {
        self.0[0] -= rhs.x();
        self.0[1] -= rhs.y();
        self.0[2] -= rhs.z();
        self.0[3] -= rhs.w();
    }
}

impl<T: Float + MulAssign, V: AsVec4<T>> MulAssign<V> for Vec4<T> {
    fn mul_assign(&mut self, rhs: V) {
        self.0[0] *= rhs.x();
        self.0[1] *= rhs.y();
        self.0[2] *= rhs.z();
        self.0[3] *= rhs.w();
    }
}

impl<T: Float + DivAssign, V: AsVec4<T>> DivAssign<V> for Vec4<T> {
    fn div_assign(&mut self, rhs: V) {
        self.0[0] /= rhs.x();
        self.0[1] /= rhs.y();
        self.0[2] /= rhs.z();
        self.0[3] /= rhs.w();
    }
}

macro_rules! float_implementations {
    ($($float: tt),+) => {
        $(
            impl Add<$float> for Vec4<$float> {
                type Output = Self;

                #[inline(always)]
                fn add(self, b: $float) -> Self::Output {
                    Self([self.0[0] + b, self.0[1] + b, self.0[2] + b, self.0[3] + b])
                }
            }

            impl Sub<$float> for Vec4<$float> {
                type Output = Self;

                #[inline(always)]
                fn sub(self, b: $float) -> Self::Output {
                    Self([self.0[0] - b, self.0[1] - b, self.0[2] - b, self.0[3] - b])
                }
            }

            impl Mul<$float> for Vec4<$float> {
                type Output = Self;

                #[inline(always)]
                fn mul(self, b: $float) -> Self::Output {
                    Self([self.0[0] * b, self.0[1] * b, self.0[2] * b, self.0[3] * b])
                }
            }

            impl Div<$float> for Vec4<$float> {
                type Output = Self;

                #[inline(always)]
                fn div(self, b: $float) -> Self::Output {
                    Self([self.0[0] / b, self.0[1] / b, self.0[2] / b, self.0[3] / b])
                }
            }

            impl AddAssign<$float> for Vec4<$float> {
                fn add_assign(&mut self, rhs: $float) {
                    self.0[0] = self.0[0] + rhs;
                    self.0[1] = self.0[1] + rhs;
                    self.0[2] = self.0[2] + rhs;
                    self.0[3] = self.0[3] + rhs;
                }
            }

            impl SubAssign<$float> for Vec4<$float> {
                fn sub_assign(&mut self, rhs: $float) {
                    self.0[0] = self.0[0] - rhs;
                    self.0[1] = self.0[1] - rhs;
                    self.0[2] = self.0[2] - rhs;
                    self.0[3] = self.0[3] - rhs;
                }
            }

            impl MulAssign<$float> for Vec4<$float> {
                fn mul_assign(&mut self, rhs: $float) {
                    self.0[0] = self.0[0] * rhs;
                    self.0[1] = self.0[1] * rhs;
                    self.0[2] = self.0[2] * rhs;
                    self.0[3] = self.0[3] * rhs;
                }
            }

            impl DivAssign<$float> for Vec4<$float> {
                fn div_assign(&mut self, rhs: $float) {
                    self.0[0] = self.0[0] / rhs;
                    self.0[1] = self.0[1] / rhs;
                    self.0[2] = self.0[2] / rhs;
                    self.0[3] = self.0[3] / rhs;
                }
            }
        )+
    };
}

float_implementations!(f16, f32, f64);

impl<T: Float> From<[T; 4]> for Vec4<T> {
    fn from(value: [T; 4]) -> Self {
        Self(value)
    }
}

impl<T: Float> From<(T, T, T, T)> for Vec4<T> {
    fn from(value: (T, T, T, T)) -> Self {
        Self(AsVec4::to_raw(&value))
    }
}

impl<T> AsRef<Self> for Vec4<T> {
    fn as_ref(&self) -> &Self {
        self
    }
}

impl<T> AsRef<[T]> for Vec4<T> {
    fn as_ref(&self) -> &[T] {
        &self.0
    }
}

impl<T: Display> Display for Vec4<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let value = self
            .0
            .iter()
            .map(|v| v.to_string())
            .collect::<Vec<_>>()
            .join(", ");
        f.write_fmt(format_args!("vec4({})", value))
    }
}

#[cfg(test)]
mod tests {
    use crate::vec4::AsVec4;

    use super::Vec4;

    fn vec_a() -> Vec4 {
        Vec4::from_values(1.0, 2.0, 3.0, 4.0)
    }

    fn vec_b() -> Vec4 {
        Vec4::from_values(5.0, 6.0, 7.0, 8.0)
    }

    #[test]
    fn new() {
        assert_eq!(Vec4::<f64>::new().to_raw(), [0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn from_slice() {
        assert_eq!(
            Vec4::from_slice([3.0, 4.0, 5.0, 6.0]).to_raw(),
            [3.0, 4.0, 5.0, 6.0]
        );
    }

    #[test]
    fn from_values() {
        assert_eq!(
            Vec4::from_values(3.0, 4.0, 5.0, 6.0).to_raw(),
            [3.0, 4.0, 5.0, 6.0]
        );
    }

    #[test]
    fn ceil() {
        assert_eq!(
            Vec4::from_values(
                std::f32::consts::E,
                std::f32::consts::PI,
                std::f32::consts::SQRT_2,
                0.5f32.sqrt()
            )
            .ceil()
            .to_raw(),
            [3.0, 4.0, 2.0, 1.0]
        );
    }

    #[test]
    fn floor() {
        assert_eq!(
            Vec4::from_values(
                std::f32::consts::E,
                std::f32::consts::PI,
                std::f32::consts::SQRT_2,
                0.5f32.sqrt()
            )
            .floor()
            .to_raw(),
            [2.0, 3.0, 1.0, 0.0]
        );
    }

    #[test]
    fn min() {
        assert_eq!(
            Vec4::from_values(1.0, 3.0, 1.0, 3.0)
                .min(&[3.0, 1.0, 3.0, 1.0])
                .to_raw(),
            [1.0, 1.0, 1.0, 1.0]
        );
    }

    #[test]
    fn max() {
        assert_eq!(
            Vec4::from_values(1.0, 3.0, 1.0, 3.0)
                .max(&[3.0, 1.0, 3.0, 1.0])
                .to_raw(),
            [3.0, 3.0, 3.0, 3.0]
        );
    }

    #[test]
    fn round() {
        let vec = Vec4::from_values(
            std::f32::consts::E,
            std::f32::consts::PI,
            std::f32::consts::SQRT_2,
            0.5f32.sqrt(),
        )
        .round();
        assert_eq!(vec.to_raw(), [3.0, 3.0, 1.0, 1.0]);
    }

    #[test]
    fn scale() {
        assert_eq!((vec_a() * 2.0).to_raw(), [2.0, 4.0, 6.0, 8.0]);
    }

    #[test]
    fn scale_add() {
        assert_eq!((vec_a() + vec_b() * 0.5).to_raw(), [3.5, 5.0, 6.5, 8.0]);
    }

    #[test]
    fn squared_distance() {
        assert_eq!(vec_a().squared_distance(&vec_b()), 64.0);
    }

    #[test]
    fn distance() {
        assert_eq!(vec_a().distance(&vec_b()), 8.0);
    }

    #[test]
    fn squared_length() {
        assert_eq!(vec_a().squared_length(), 30.0);
    }

    #[test]
    fn length() {
        assert_eq!(vec_a().length(), 5.477225575051661);
    }

    #[test]
    fn negate() {
        assert_eq!(vec_a().negate().to_raw(), [-1.0, -2.0, -3.0, -4.0]);
    }

    #[test]
    fn normalize() {
        assert_eq!(
            Vec4::from_values(5.0, 0.0, 0.0, 0.0).normalize().to_raw(),
            [1.0, 0.0, 0.0, 0.0]
        );
    }

    #[test]
    fn dot() {
        assert_eq!(vec_a().dot(&vec_b()), 70.0);
    }

    #[test]
    fn cross() {
        let vec_a = Vec4::from_values(1.0, 0.0, 0.0, 0.0)
            .cross(&[0.0, 1.0, 0.0, 0.0], &[0.0, 1.0, 1.0, 0.0]);
        assert_eq!(vec_a.to_raw(), [0.0, 0.0, 0.0, -1.0]);
    }

    #[test]
    fn lerp() {
        assert_eq!(vec_a().lerp(&vec_b(), 0.5).to_raw(), [3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn set() {
        let mut mat = Vec4::new();
        mat.set(3.0, 4.0, 5.0, 6.0);

        assert_eq!(mat.to_raw(), [3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn set_slice() {
        let mut mat = Vec4::new();
        mat.set_slice(&[3.0, 4.0, 5.0, 6.0]);

        assert_eq!(mat.raw(), &[3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn add() {
        assert_eq!((vec_a() + vec_b()).to_raw(), [6.0, 8.0, 10.0, 12.0]);
    }

    #[test]
    fn sub() {
        assert_eq!((vec_a() - vec_b()).to_raw(), [-4.0, -4.0, -4.0, -4.0]);
    }

    #[test]
    fn mul() {
        assert_eq!((vec_a() * vec_b()).to_raw(), [5.0, 12.0, 21.0, 32.0]);
    }

    #[test]
    fn mul_scalar() {
        assert_eq!((vec_a() * 2.0).to_raw(), [2.0, 4.0, 6.0, 8.0]);
    }

    #[test]
    fn mul_scalar_add() {
        assert_eq!((vec_a() + vec_b() * 0.5).to_raw(), [3.5, 5.0, 6.5, 8.0]);
    }

    #[test]
    fn div() {
        assert_eq!(
            (vec_a() / vec_b()).to_raw(),
            [0.2, 0.3333333333333333, 0.42857142857142855, 0.5]
        );
    }

    #[test]
    fn div_scalar() {
        assert_eq!((vec_a() / 2.0).to_raw(), [0.5, 1.0, 1.5, 2.0]);
    }

    #[test]
    fn div_scalar_add() {
        assert_eq!((vec_a() + vec_b() / 0.5).to_raw(), [11.0, 14.0, 17.0, 20.0]);
    }

    #[test]
    fn approximate_eq() {
        let vec_a = Vec4::from_values(0.0, 1.0, 2.0, 3.0);
        let vec_b = Vec4::from_values(0.0, 1.0, 2.0, 3.0);
        let vec_c = Vec4::from_values(1.0, 2.0, 3.0, 4.0);
        let vec_d = Vec4::from_values(1e-16, 1.0, 2.0, 3.0);

        assert_eq!(true, vec_a.approximate_eq(&vec_b));
        assert_eq!(false, vec_a.approximate_eq(&vec_c));
        assert_eq!(true, vec_a.approximate_eq(&vec_d));
    }

    #[test]
    fn display() {
        let out = vec_a().to_string();
        assert_eq!(out, "vec4(1, 2, 3, 4)");
    }
}
