use std::{
    fmt::Display,
    ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub, SubAssign},
};

use half::f16;
use num_traits::Float;

use crate::{epsilon, mat4::Mat4, quat::Quat};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Vec4<T = f64>(pub [T; 4]);

impl<T: Float> Vec4<T> {
    #[inline(always)]
    pub fn new() -> Self {
        Self([T::zero(); 4])
    }

    #[inline(always)]
    pub fn from_values(x: T, y: T, z: T, w: T) -> Self {
        Self([x, y, z, w])
    }

    #[inline(always)]
    pub fn from_slice([x, y, z, w]: &[T; 4]) -> Self {
        Self([*x, *y, *z, *w])
    }
}

impl<T: Float> Vec4<T> {
    #[inline(always)]
    pub fn into_gl(&self) -> [f32; 4] {
        [
            T::to_f32(&self.0[0]).unwrap(),
            T::to_f32(&self.0[1]).unwrap(),
            T::to_f32(&self.0[2]).unwrap(),
            T::to_f32(&self.0[3]).unwrap(),
        ]
    }

    #[inline(always)]
    pub fn into_gl_binary(&self) -> [u8; 16] {
        unsafe { std::mem::transmute_copy::<[f32; 4], [u8; 16]>(&self.into_gl()) }
    }

    #[inline(always)]
    pub fn raw(&self) -> &[T; 4] {
        &self.0
    }

    #[inline(always)]
    pub fn set(&mut self, x: T, y: T, z: T, w: T) -> &mut Self {
        self.0[0] = x;
        self.0[1] = y;
        self.0[2] = z;
        self.0[3] = w;
        self
    }

    #[inline(always)]
    pub fn set_slice(&mut self, [x, y, z, w]: &[T; 4]) -> &mut Self {
        self.0[0] = *x;
        self.0[1] = *y;
        self.0[2] = *z;
        self.0[3] = *w;
        self
    }

    #[inline(always)]
    pub fn set_zero(&mut self) -> &mut Self {
        self.0[0] = T::zero();
        self.0[1] = T::zero();
        self.0[2] = T::zero();
        self.0[3] = T::zero();
        self
    }

    #[inline(always)]
    pub fn ceil(&self) -> Self {
        Self([
            self.0[0].ceil(),
            self.0[1].ceil(),
            self.0[2].ceil(),
            self.0[3].ceil(),
        ])
    }

    #[inline(always)]
    pub fn floor(&self) -> Self {
        Self([
            self.0[0].floor(),
            self.0[1].floor(),
            self.0[2].floor(),
            self.0[3].floor(),
        ])
    }

    #[inline(always)]
    pub fn min(&self, b: impl AsRef<Self>) -> Self {
        let b = b.as_ref();

        Self([
            self.0[0].min(b.0[0]),
            self.0[1].min(b.0[1]),
            self.0[2].min(b.0[2]),
            self.0[3].min(b.0[3]),
        ])
    }

    #[inline(always)]
    pub fn max(&self, b: impl AsRef<Self>) -> Self {
        let b = b.as_ref();

        Self([
            self.0[0].max(b.0[0]),
            self.0[1].max(b.0[1]),
            self.0[2].max(b.0[2]),
            self.0[3].max(b.0[3]),
        ])
    }

    #[inline(always)]
    pub fn round(&self) -> Self {
        Self([
            self.0[0].round(),
            self.0[1].round(),
            self.0[2].round(),
            self.0[3].round(),
        ])
    }

    #[inline(always)]
    pub fn scale(&self, scale: T) -> Self {
        self.mul(scale)
    }

    #[inline(always)]
    pub fn squared_distance(&self, b: impl AsRef<Self>) -> T {
        let b = b.as_ref();

        let x = b.0[0] - self.0[0];
        let y = b.0[1] - self.0[1];
        let z = b.0[2] - self.0[2];
        let w = b.0[3] - self.0[3];
        x * x + y * y + z * z + w * w
    }

    #[inline(always)]
    pub fn distance(&self, b: impl AsRef<Self>) -> T {
        let b = b.as_ref();

        self.squared_distance(b).sqrt()
    }

    #[inline(always)]
    pub fn squared_length(&self) -> T {
        let x = self.0[0];
        let y = self.0[1];
        let z = self.0[2];
        let w = self.0[3];
        x * x + y * y + z * z + w * w
    }

    #[inline(always)]
    pub fn length(&self) -> T {
        self.squared_length().sqrt()
    }

    #[inline(always)]
    pub fn negate(&self) -> Self {
        Self([-self.0[0], -self.0[1], -self.0[2], -self.0[3]])
    }

    #[inline(always)]
    pub fn inverse(&self) -> Self {
        Self([
            T::one() / self.0[0],
            T::one() / self.0[1],
            T::one() / self.0[2],
            T::one() / self.0[3],
        ])
    }

    #[inline(always)]
    pub fn normalize(&self) -> Self {
        let mut len = self.squared_length();
        if len > T::zero() {
            len = T::one() / len.sqrt();
        }

        Self([
            self.0[0] * len,
            self.0[1] * len,
            self.0[2] * len,
            self.0[3] * len,
        ])
    }

    #[inline(always)]
    pub fn dot(&self, b: impl AsRef<Self>) -> T {
        let b = b.as_ref();

        self.0[0] * b.0[0] + self.0[1] * b.0[1] + self.0[2] * b.0[2] + self.0[3] * b.0[3]
    }

    #[inline(always)]
    pub fn cross(&self, v: impl AsRef<Self>, w: impl AsRef<Self>) -> Self {
        let v = v.as_ref();
        let w = w.as_ref();

        let a = v.0[0] * w.0[1] - v.0[1] * w.0[0];
        let b = v.0[0] * w.0[2] - v.0[2] * w.0[0];
        let c = v.0[0] * w.0[3] - v.0[3] * w.0[0];
        let d = v.0[1] * w.0[2] - v.0[2] * w.0[1];
        let e = v.0[1] * w.0[3] - v.0[3] * w.0[1];
        let f = v.0[2] * w.0[3] - v.0[3] * w.0[2];
        let g = self.0[0];
        let h = self.0[1];
        let i = self.0[2];
        let j = self.0[3];

        Self([
            h * f - i * e + j * d,
            -(g * f) + i * c - j * b,
            g * e - h * c + j * a,
            -(g * d) + h * b - i * a,
        ])
    }

    #[inline(always)]
    pub fn lerp(&self, b: impl AsRef<Self>, t: T) -> Self {
        let b = b.as_ref();

        let ax = self.0[0];
        let ay = self.0[1];
        let az = self.0[2];
        let aw = self.0[3];

        Self([
            ax + t * (b.0[0] - ax),
            ay + t * (b.0[1] - ay),
            az + t * (b.0[2] - az),
            aw + t * (b.0[3] - aw),
        ])
    }

    #[inline(always)]
    pub fn transform_mat4(&self, m: impl AsRef<Mat4<T>>) -> Self {
        let m = m.as_ref();

        let x = self.0[0];
        let y = self.0[1];
        let z = self.0[2];
        let w = self.0[3];

        Self([
            m.0[0] * x + m.0[4] * y + m.0[8] * z + m.0[12] * w,
            m.0[1] * x + m.0[5] * y + m.0[9] * z + m.0[13] * w,
            m.0[2] * x + m.0[6] * y + m.0[10] * z + m.0[14] * w,
            m.0[3] * x + m.0[7] * y + m.0[11] * z + m.0[15] * w,
        ])
    }

    #[inline(always)]
    pub fn transform_quat(&self, q: impl AsRef<Quat<T>>) -> Self {
        let q = q.as_ref();

        let x = self.0[0];
        let y = self.0[1];
        let z = self.0[2];
        let qx = q.0[0];
        let qy = q.0[1];
        let qz = q.0[2];
        let qw = q.0[3];

        // calculate quat * vec
        let ix = qw * x + qy * z - qz * y;
        let iy = qw * y + qz * x - qx * z;
        let iz = qw * z + qx * y - qy * x;
        let iw = -qx * x - qy * y - qz * z;

        Self([
            ix * qw + iw * -qx + iy * -qz - iz * -qy,
            iy * qw + iw * -qy + iz * -qx - ix * -qz,
            iz * qw + iw * -qz + ix * -qy - iy * -qx,
            self.0[3],
        ])
    }

    #[inline(always)]
    pub fn approximate_eq(&self, b: impl AsRef<Self>) -> bool {
        let b = b.as_ref();

        let a0 = self.0[0];
        let a1 = self.0[1];
        let a2 = self.0[2];
        let a3 = self.0[3];
        let b0 = b.0[0];
        let b1 = b.0[1];
        let b2 = b.0[2];
        let b3 = b.0[3];

        (a0 - b0).abs() <= epsilon::<T>() * T::one().max(a0.abs()).max(b0.abs())
            && (a1 - b1).abs() <= epsilon::<T>() * T::one().max(a1.abs()).max(b1.abs())
            && (a2 - b2).abs() <= epsilon::<T>() * T::one().max(a2.abs()).max(b2.abs())
            && (a3 - b3).abs() <= epsilon::<T>() * T::one().max(a3.abs()).max(b3.abs())
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

impl<T: Float> Add<Vec4<T>> for Vec4<T> {
    type Output = Self;

    #[inline(always)]
    fn add(self, b: Self) -> Self {
        Self([
            self.0[0] + b.0[0],
            self.0[1] + b.0[1],
            self.0[2] + b.0[2],
            self.0[3] + b.0[3],
        ])
    }
}

impl<T: Float> Add<T> for Vec4<T> {
    type Output = Self;

    #[inline(always)]
    fn add(self, b: T) -> Self {
        Self([self.0[0] + b, self.0[1] + b, self.0[2] + b, self.0[3] + b])
    }
}

impl<T: Float> Sub<Vec4<T>> for Vec4<T> {
    type Output = Self;

    #[inline(always)]
    fn sub(self, b: Self) -> Self {
        Self([
            self.0[0] - b.0[0],
            self.0[1] - b.0[1],
            self.0[2] - b.0[2],
            self.0[3] - b.0[3],
        ])
    }
}

impl<T: Float> Sub<T> for Vec4<T> {
    type Output = Self;

    #[inline(always)]
    fn sub(self, b: T) -> Self {
        Self([self.0[0] - b, self.0[1] - b, self.0[2] - b, self.0[3] - b])
    }
}

impl<T: Float> Mul<Vec4<T>> for Vec4<T> {
    type Output = Self;

    #[inline(always)]
    fn mul(self, b: Self) -> Self {
        Self([
            self.0[0] * b.0[0],
            self.0[1] * b.0[1],
            self.0[2] * b.0[2],
            self.0[3] * b.0[3],
        ])
    }
}

impl<T: Float> Mul<T> for Vec4<T> {
    type Output = Self;

    #[inline(always)]
    fn mul(self, b: T) -> Self {
        Self([self.0[0] * b, self.0[1] * b, self.0[2] * b, self.0[3] * b])
    }
}

impl<T: Float> Div<Vec4<T>> for Vec4<T> {
    type Output = Self;

    #[inline(always)]
    fn div(self, b: Self) -> Self {
        Self([
            self.0[0] / b.0[0],
            self.0[1] / b.0[1],
            self.0[2] / b.0[2],
            self.0[3] / b.0[3],
        ])
    }
}

impl<T: Float> Div<T> for Vec4<T> {
    type Output = Self;

    #[inline(always)]
    fn div(self, b: T) -> Self {
        Self([self.0[0] / b, self.0[1] / b, self.0[2] / b, self.0[3] / b])
    }
}

impl<T: Float> AddAssign<Self> for Vec4<T> {
    fn add_assign(&mut self, rhs: Self) {
        self.0[0] = self.0[0] + rhs.0[0];
        self.0[1] = self.0[1] + rhs.0[1];
        self.0[2] = self.0[2] + rhs.0[2];
        self.0[3] = self.0[3] + rhs.0[3];
    }
}

impl<T: Float> AddAssign<T> for Vec4<T> {
    fn add_assign(&mut self, rhs: T) {
        self.0[0] = self.0[0] + rhs;
        self.0[1] = self.0[1] + rhs;
        self.0[2] = self.0[2] + rhs;
        self.0[3] = self.0[3] + rhs;
    }
}

impl<T: Float> SubAssign<Self> for Vec4<T> {
    fn sub_assign(&mut self, rhs: Self) {
        self.0[0] = self.0[0] - rhs.0[0];
        self.0[1] = self.0[1] - rhs.0[1];
        self.0[2] = self.0[2] - rhs.0[2];
        self.0[3] = self.0[3] - rhs.0[2];
    }
}

impl<T: Float> SubAssign<T> for Vec4<T> {
    fn sub_assign(&mut self, rhs: T) {
        self.0[0] = self.0[0] - rhs;
        self.0[1] = self.0[1] - rhs;
        self.0[2] = self.0[2] - rhs;
        self.0[3] = self.0[3] - rhs;
    }
}

impl<T: Float> MulAssign<Self> for Vec4<T> {
    fn mul_assign(&mut self, rhs: Self) {
        self.0[0] = self.0[0] * rhs.0[0];
        self.0[1] = self.0[1] * rhs.0[1];
        self.0[2] = self.0[2] * rhs.0[2];
        self.0[3] = self.0[3] * rhs.0[3];
    }
}

impl<T: Float> MulAssign<T> for Vec4<T> {
    fn mul_assign(&mut self, rhs: T) {
        self.0[0] = self.0[0] * rhs;
        self.0[1] = self.0[1] * rhs;
        self.0[2] = self.0[2] * rhs;
        self.0[3] = self.0[3] * rhs;
    }
}

impl<T: Float> DivAssign<Self> for Vec4<T> {
    fn div_assign(&mut self, rhs: Self) {
        self.0[0] = self.0[0] / rhs.0[0];
        self.0[1] = self.0[1] / rhs.0[1];
        self.0[2] = self.0[2] / rhs.0[2];
        self.0[3] = self.0[3] / rhs.0[3];
    }
}

impl<T: Float> DivAssign<T> for Vec4<T> {
    fn div_assign(&mut self, rhs: T) {
        self.0[0] = self.0[0] / rhs;
        self.0[1] = self.0[1] / rhs;
        self.0[2] = self.0[2] / rhs;
        self.0[3] = self.0[3] / rhs;
    }
}

impl<T> AsRef<Vec4<T>> for Vec4<T> {
    fn as_ref(&self) -> &Self {
        self
    }
}

impl<T> AsRef<[T]> for Vec4<T> {
    fn as_ref(&self) -> &[T] {
        &self.0
    }
}

impl AsRef<[u8]> for Vec4<f64> {
    fn as_ref(&self) -> &[u8] {
        unsafe { std::mem::transmute::<&[f64; 4], &[u8; 32]>(&self.0) }
    }
}

impl AsRef<[u8]> for Vec4<f32> {
    fn as_ref(&self) -> &[u8] {
        unsafe { std::mem::transmute::<&[f32; 4], &[u8; 16]>(&self.0) }
    }
}

impl AsRef<[u8]> for Vec4<f16> {
    fn as_ref(&self) -> &[u8] {
        unsafe { std::mem::transmute::<&[f16; 4], &[u8; 8]>(&self.0) }
    }
}

impl<T: Float> Default for Vec4<T> {
    fn default() -> Self {
        Self::new()
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

/// tests only for f32
#[cfg(test)]
mod tests {
    use std::sync::OnceLock;

    use super::Vec4;

    static VEC_A_RAW: [f32; 4] = [1.0, 2.0, 3.0, 4.0];
    static VEC_B_RAW: [f32; 4] = [5.0, 6.0, 7.0, 8.0];

    static VEC_A: OnceLock<Vec4<f32>> = OnceLock::new();
    static VEC_B: OnceLock<Vec4<f32>> = OnceLock::new();

    fn vec_a() -> &'static Vec4<f32> {
        VEC_A.get_or_init(|| Vec4::from_slice(&VEC_A_RAW))
    }

    fn vec_b() -> &'static Vec4<f32> {
        VEC_B.get_or_init(|| Vec4::from_slice(&VEC_B_RAW))
    }

    #[test]
    fn new() {
        assert_eq!(Vec4::<f32>::new().raw(), &[0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn from_slice() {
        assert_eq!(
            Vec4::from_slice(&[3.0, 4.0, 5.0, 6.0]).raw(),
            &[3.0, 4.0, 5.0, 6.0]
        );
    }

    #[test]
    fn from_values() {
        assert_eq!(
            Vec4::from_values(3.0, 4.0, 5.0, 6.0).raw(),
            &[3.0, 4.0, 5.0, 6.0]
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
            .raw(),
            &[3.0, 4.0, 2.0, 1.0]
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
            .raw(),
            &[2.0, 3.0, 1.0, 0.0]
        );
    }

    #[test]
    fn min() {
        let vec_a = Vec4::from_values(1.0, 3.0, 1.0, 3.0);
        let vec_b = Vec4::from_values(3.0, 1.0, 3.0, 1.0);
        assert_eq!(vec_a.min(vec_b).raw(), &[1.0, 1.0, 1.0, 1.0]);
    }

    #[test]
    fn max() {
        let vec_a = Vec4::from_values(1.0, 3.0, 1.0, 3.0);
        let vec_b = Vec4::from_values(3.0, 1.0, 3.0, 1.0);
        assert_eq!(vec_a.max(vec_b).raw(), &[3.0, 3.0, 3.0, 3.0]);
    }

    #[test]
    fn round() {
        let vec = Vec4::from_values(
            std::f32::consts::E,
            std::f32::consts::PI,
            std::f32::consts::SQRT_2,
            0.5f32.sqrt(),
        );
        assert_eq!(vec.round().raw(), &[3.0, 3.0, 1.0, 1.0]);
    }

    #[test]
    fn scale() {
        assert_eq!((*vec_a() * 2.0).raw(), &[2.0, 4.0, 6.0, 8.0]);
    }

    #[test]
    fn scale_add() {
        assert_eq!((*vec_a() + *vec_b() * 0.5).raw(), &[3.5, 5.0, 6.5, 8.0]);
    }

    #[test]
    fn squared_distance() {
        assert_eq!(vec_a().squared_distance(vec_b()), 64.0);
    }

    #[test]
    fn distance() {
        assert_eq!(vec_a().distance(vec_b()), 8.0);
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
        assert_eq!(vec_a().negate().raw(), &[-1.0, -2.0, -3.0, -4.0]);
    }

    #[test]
    fn normalize() {
        assert_eq!(
            Vec4::from_values(5.0, 0.0, 0.0, 0.0).normalize().raw(),
            &[1.0, 0.0, 0.0, 0.0]
        );
    }

    #[test]
    fn dot() {
        assert_eq!(vec_a().dot(vec_b()), 70.0);
    }

    #[test]
    fn cross() {
        let vec_a = Vec4::from_values(1.0, 0.0, 0.0, 0.0);
        let vec_b = Vec4::from_values(0.0, 1.0, 0.0, 0.0);
        let vec_c = Vec4::from_values(0.0, 1.0, 1.0, 0.0);
        assert_eq!(vec_a.cross(vec_b, vec_c).raw(), &[0.0, 0.0, 0.0, -1.0]);
    }

    #[test]
    fn lerp() {
        assert_eq!(vec_a().lerp(vec_b(), 0.5).raw(), &[3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn set() {
        let mut mat = Vec4::new();
        mat.set(3.0, 4.0, 5.0, 6.0);

        assert_eq!(mat.raw(), &[3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn set_slice() {
        let mut mat = Vec4::new();
        mat.set_slice(&[3.0, 4.0, 5.0, 6.0]);

        assert_eq!(mat.raw(), &[3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn add() {
        assert_eq!((*vec_a() + *vec_b()).raw(), &[6.0, 8.0, 10.0, 12.0]);
    }

    #[test]
    fn sub() {
        assert_eq!((*vec_a() - *vec_b()).raw(), &[-4.0, -4.0, -4.0, -4.0]);
    }

    #[test]
    fn mul() {
        assert_eq!((*vec_a() * *vec_b()).raw(), &[5.0, 12.0, 21.0, 32.0]);
    }

    #[test]
    fn mul_scalar() {
        assert_eq!((*vec_a() * 2.0).raw(), &[2.0, 4.0, 6.0, 8.0]);
    }

    #[test]
    fn mul_scalar_add() {
        assert_eq!((*vec_a() + *vec_b() * 0.5).raw(), &[3.5, 5.0, 6.5, 8.0]);
    }

    #[test]
    fn div() {
        assert_eq!(
            (*vec_a() / *vec_b()).raw(),
            &[0.2, 0.3333333333333333, 0.42857142857142855, 0.5]
        );
    }

    #[test]
    fn div_scalar() {
        assert_eq!((*vec_a() / 2.0).raw(), &[0.5, 1.0, 1.5, 2.0]);
    }

    #[test]
    fn div_scalar_add() {
        assert_eq!((*vec_a() + *vec_b() / 0.5).raw(), &[11.0, 14.0, 17.0, 20.0]);
    }

    #[test]
    fn approximate_eq() {
        let vec_a = Vec4::from_values(0.0, 1.0, 2.0, 3.0);
        let vec_b = Vec4::from_values(0.0, 1.0, 2.0, 3.0);
        let vec_c = Vec4::from_values(1.0, 2.0, 3.0, 4.0);
        let vec_d = Vec4::from_values(1e-16, 1.0, 2.0, 3.0);

        assert_eq!(true, vec_a.approximate_eq(vec_b));
        assert_eq!(false, vec_a.approximate_eq(vec_c));
        assert_eq!(true, vec_a.approximate_eq(vec_d));
    }

    #[test]
    fn display() {
        let out = vec_a().to_string();
        assert_eq!(out, "vec4(1, 2, 3, 4)");
    }

    #[test]
    fn test_u8_slice() {
        let bin: &[u8] = vec_a().as_ref();
        bin.chunks(4).enumerate().for_each(|(index, bin)| {
            let value = f32::from_ne_bytes(bin.try_into().unwrap());
            assert_eq!(vec_a().0[index], value);
        });
    }
}
