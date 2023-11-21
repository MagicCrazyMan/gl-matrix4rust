use std::{
    fmt::Display,
    ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub, SubAssign},
};

use half::f16;
use num_traits::Float;

use crate::{
    epsilon,
    mat2::AsMat2,
    mat2d::AsMat2d,
    mat3::AsMat3,
    mat4::AsMat4,
    vec3::{AsVec3, Vec3},
};

pub trait AsVec2<T: Float> {
    fn from_values(x: T, y: T) -> Self;

    fn x(&self) -> T;

    fn y(&self) -> T;

    fn set_x(&mut self, x: T) -> &mut Self;

    fn set_y(&mut self, y: T) -> &mut Self;

    #[inline(always)]
    fn to_raw(&self) -> [T; 2] {
        [self.x(), self.y()]
    }

    #[inline(always)]
    fn to_gl(&self) -> [f32; 2] {
        [T::to_f32(&self.x()).unwrap(), T::to_f32(&self.y()).unwrap()]
    }

    #[inline(always)]
    fn to_gl_binary(&self) -> [u8; 8] {
        unsafe { std::mem::transmute_copy::<[f32; 2], [u8; 8]>(&self.to_gl()) }
    }

    #[inline(always)]
    fn copy<V: AsVec2<T> + ?Sized>(&mut self, b: &V) -> &mut Self {
        self.set_x(b.x()).set_y(b.y())
    }

    #[inline(always)]
    fn set(&mut self, x: T, y: T) -> &mut Self {
        self.set_x(x).set_y(y)
    }

    #[inline(always)]
    fn set_slice(&mut self, [x, y]: &[T; 2]) -> &mut Self {
        self.set_x(*x).set_y(*y)
    }

    #[inline(always)]
    fn set_zero(&mut self) -> &mut Self {
        self.set_x(T::zero()).set_y(T::zero())
    }

    #[inline(always)]
    fn ceil(&self) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();

        Self::from_values(x.ceil(), y.ceil())
    }

    #[inline(always)]
    fn floor(&self) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();

        Self::from_values(x.floor(), y.floor())
    }

    #[inline(always)]
    fn round(&self) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();

        Self::from_values(x.round(), y.round())
    }

    #[inline(always)]
    fn min<V: AsVec2<T> + ?Sized>(&self, b: &V) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();

        Self::from_values(x.min(b.x()), y.min(b.y()))
    }

    #[inline(always)]
    fn max<V: AsVec2<T> + ?Sized>(&self, b: &V) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();

        Self::from_values(x.max(b.x()), y.max(b.y()))
    }

    #[inline(always)]
    fn scale(&self, scale: T) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();

        Self::from_values(x * scale, y * scale)
    }

    #[inline(always)]
    fn squared_distance<V: AsVec2<T> + ?Sized>(&self, b: &V) -> T {
        let x = b.x() - self.x();
        let y = b.y() - self.y();
        x * x + y * y
    }

    #[inline(always)]
    fn distance<V: AsVec2<T> + ?Sized>(&self, b: &V) -> T {
        self.squared_distance(b).sqrt()
    }

    #[inline(always)]
    fn squared_length(&self) -> T {
        let x = self.x();
        let y = self.y();
        x * x + y * y
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

        Self::from_values(x.neg(), y.neg())
    }

    #[inline(always)]
    fn inverse(&self) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();

        Self::from_values(T::one() / x, T::one() / y)
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

        Self::from_values(x * len, y * len)
    }

    #[inline(always)]
    fn dot<V: AsVec2<T> + ?Sized>(&self, b: &V) -> T {
        let x = self.x();
        let y = self.y();

        x * b.x() + y * b.y()
    }

    #[inline(always)]
    fn cross<V: AsVec2<T> + ?Sized>(&self, b: &V) -> Vec3<T> {
        let x = self.x();
        let y = self.y();

        Vec3::<T>::from_values(T::zero(), T::zero(), x * b.y() - y * b.x())
    }

    #[inline(always)]
    fn lerp<V: AsVec2<T> + ?Sized>(&self, b: &V, t: T) -> Self
    where
        Self: Sized,
    {
        let ax = self.x();
        let ay = self.y();

        Self::from_values(ax + t * (b.x() - ax), ay + t * (b.y() - ay))
    }

    #[inline(always)]
    fn transform_mat2<M: AsMat2<T> + ?Sized>(&self, m: &M) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();

        Self::from_values(m.m00() * x + m.m10() * y, m.m01() * x + m.m11() * y)
    }

    #[inline(always)]
    fn transform_mat2d<M: AsMat2d<T> + ?Sized>(&self, m: &M) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();

        Self::from_values(
            m.a() * x + m.c() * y + m.tx(),
            m.b() * x + m.d() * y + m.ty(),
        )
    }

    #[inline(always)]
    fn transform_mat3<M: AsMat3<T> + ?Sized>(&self, m: &M) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();

        Self::from_values(
            m.m00() * x + m.m10() * y + m.m20(),
            m.m01() * x + m.m11() * y + m.m21(),
        )
    }

    #[inline(always)]
    fn transform_mat4<M: AsMat4<T> + ?Sized>(&self, m: &M) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();

        Self::from_values(
            m.m00() * x + m.m10() * y + m.m30(),
            m.m01() * x + m.m11() * y + m.m31(),
        )
    }

    #[inline(always)]
    fn rotate<V: AsVec2<T> + ?Sized>(&self, b: &V, rad: T) -> Self
    where
        Self: Sized,
    {
        //Translate point to the origin
        let p0 = self.x() - b.x();
        let p1 = self.y() - b.y();
        let sin_c = rad.sin();
        let cos_c = rad.cos();

        //perform rotation and translate to correct position
        Self::from_values(
            p0 * cos_c - p1 * sin_c + b.x(),
            p0 * sin_c + p1 * cos_c + b.y(),
        )
    }

    #[inline(always)]
    fn angle<V: AsVec2<T> + ?Sized>(&self, b: &V) -> T {
        let x1 = self.x();
        let y1 = self.y();
        let x2 = b.x();
        let y2 = b.y();
        // mag is the product of the magnitudes of a and b
        let mag = ((x1 * x1 + y1 * y1) * (x2 * x2 + y2 * y2)).sqrt();
        // mag &&.. short circuits if mag == 0
        let cosine = if mag == T::zero() {
            mag
        } else {
            (x1 * x2 + y1 * y2) / mag
        };
        // Math.min(Math.max(cosine, -1), 1) clamps the cosine between -1 and 1
        cosine.max(-T::one()).min(T::one()).acos()
    }

    #[inline(always)]
    fn approximate_eq<V: AsVec2<T> + ?Sized>(&self, b: &V) -> bool {
        let a0 = self.x();
        let a1 = self.y();
        let b0 = b.x();
        let b1 = b.y();

        (a0 - b0).abs() <= epsilon::<T>() * T::one().max(a0.abs()).max(b0.abs())
            && (a1 - b1).abs() <= epsilon::<T>() * T::one().max(a1.abs()).max(b1.abs())
    }
}

impl<T: Float> AsVec2<T> for [T; 2] {
    #[inline(always)]
    fn from_values(x: T, y: T) -> Self {
        [x, y]
    }

    #[inline]
    fn x(&self) -> T {
        self[0]
    }

    #[inline]
    fn y(&self) -> T {
        self[1]
    }

    #[inline]
    fn set_x(&mut self, x: T) -> &mut Self {
        self[0] = x;
        self
    }

    #[inline]
    fn set_y(&mut self, y: T) -> &mut Self {
        self[1] = y;
        self
    }
}

impl<T: Float> AsVec2<T> for (T, T) {
    #[inline(always)]
    fn from_values(x: T, y: T) -> Self {
        (x, y)
    }

    #[inline]
    fn x(&self) -> T {
        self.0
    }

    #[inline]
    fn y(&self) -> T {
        self.1
    }

    #[inline]
    fn set_x(&mut self, x: T) -> &mut Self {
        self.0 = x;
        self
    }

    #[inline]
    fn set_y(&mut self, y: T) -> &mut Self {
        self.1 = y;
        self
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Vec2<T = f64>(pub [T; 2]);

impl<T: Float> Vec2<T> {
    #[inline(always)]
    pub fn new() -> Self {
        Self([T::zero(); 2])
    }

    #[inline(always)]
    pub fn from_slice(slice: &[T; 2]) -> Self {
        Self(slice.clone())
    }

    #[inline(always)]
    pub fn raw(&self) -> &[T; 2] {
        &self.0
    }
}

impl<T: Float> AsVec2<T> for Vec2<T> {
    #[inline(always)]
    fn from_values(x: T, y: T) -> Self {
        Self([x, y])
    }

    #[inline]
    fn x(&self) -> T {
        self.0[0]
    }

    #[inline]
    fn y(&self) -> T {
        self.0[1]
    }

    #[inline]
    fn set_x(&mut self, x: T) -> &mut Self {
        self.0[0] = x;
        self
    }

    #[inline]
    fn set_y(&mut self, y: T) -> &mut Self {
        self.0[1] = y;
        self
    }
}

impl Vec2<f64> {
    #[inline(always)]
    pub fn random(&self, scale: Option<f64>) -> Self {
        let scale = match scale {
            Some(scale) => scale,
            None => 1.0,
        };

        let r = rand::random::<f64>() * 2.0 * std::f64::consts::PI;

        Self([r.cos() * scale, r.sin() * scale])
    }
}

impl Vec2<f32> {
    #[inline(always)]
    pub fn random(&self, scale: Option<f32>) -> Self {
        let scale = match scale {
            Some(scale) => scale,
            None => 1.0,
        };

        let r = rand::random::<f32>() * 2.0 * std::f32::consts::PI;

        Self([r.cos() * scale, r.sin() * scale])
    }
}

impl Vec2<f16> {
    #[inline(always)]
    pub fn random(&self, scale: Option<f16>) -> Self {
        let scale = match scale {
            Some(scale) => scale,
            None => f16::from_f32_const(1.0),
        };

        let r = rand::random::<f16>() * f16::from_f32_const(2.0) * f16::PI;

        Self([r.cos() * scale, r.sin() * scale])
    }
}

impl<T: Float> Default for Vec2<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Float> Add<Self> for Vec2<T> {
    type Output = Self;

    #[inline(always)]
    fn add(self, b: Self) -> Self {
        Self([self.0[0] + b.0[0], self.0[1] + b.0[1]])
    }
}

impl<T: Float> Add<T> for Vec2<T> {
    type Output = Self;

    #[inline(always)]
    fn add(self, b: T) -> Self {
        Self([self.0[0] + b, self.0[1] + b])
    }
}

impl<T: Float> Sub<Vec2<T>> for Vec2<T> {
    type Output = Self;

    #[inline(always)]
    fn sub(self, b: Self) -> Self {
        Self([self.0[0] - b.0[0], self.0[1] - b.0[1]])
    }
}

impl<T: Float> Sub<T> for Vec2<T> {
    type Output = Self;

    #[inline(always)]
    fn sub(self, b: T) -> Self {
        Self([self.0[0] - b, self.0[1] - b])
    }
}

impl<T: Float> Mul<Vec2<T>> for Vec2<T> {
    type Output = Self;

    #[inline(always)]
    fn mul(self, b: Self) -> Self {
        Self([self.0[0] * b.0[0], self.0[1] * b.0[1]])
    }
}

impl<T: Float> Mul<T> for Vec2<T> {
    type Output = Self;

    #[inline(always)]
    fn mul(self, b: T) -> Self {
        Self([self.0[0] * b, self.0[1] * b])
    }
}

impl<T: Float> Div<Vec2<T>> for Vec2<T> {
    type Output = Self;

    #[inline(always)]
    fn div(self, b: Self) -> Self {
        Self([self.0[0] / b.0[0], self.0[1] / b.0[1]])
    }
}

impl<T: Float> Div<T> for Vec2<T> {
    type Output = Self;

    #[inline(always)]
    fn div(self, b: T) -> Self {
        Self([self.0[0] / b, self.0[1] / b])
    }
}

impl<T: Float> AddAssign<Self> for Vec2<T> {
    #[inline(always)]
    fn add_assign(&mut self, rhs: Self) {
        self.0[0] = self.0[0] + rhs.0[0];
        self.0[1] = self.0[1] + rhs.0[1];
    }
}

impl<T: Float> AddAssign<T> for Vec2<T> {
    #[inline(always)]
    fn add_assign(&mut self, rhs: T) {
        self.0[0] = self.0[0] + rhs;
        self.0[1] = self.0[1] + rhs;
    }
}

impl<T: Float> SubAssign<Self> for Vec2<T> {
    #[inline(always)]
    fn sub_assign(&mut self, rhs: Self) {
        self.0[0] = self.0[0] - rhs.0[0];
        self.0[1] = self.0[1] - rhs.0[1];
    }
}

impl<T: Float> SubAssign<T> for Vec2<T> {
    #[inline(always)]
    fn sub_assign(&mut self, rhs: T) {
        self.0[0] = self.0[0] - rhs;
        self.0[1] = self.0[1] - rhs;
    }
}

impl<T: Float> MulAssign<Self> for Vec2<T> {
    #[inline(always)]
    fn mul_assign(&mut self, rhs: Self) {
        self.0[0] = self.0[0] * rhs.0[0];
        self.0[1] = self.0[1] * rhs.0[1];
    }
}

impl<T: Float> MulAssign<T> for Vec2<T> {
    #[inline(always)]
    fn mul_assign(&mut self, rhs: T) {
        self.0[0] = self.0[0] * rhs;
        self.0[1] = self.0[1] * rhs;
    }
}

impl<T: Float> DivAssign<Self> for Vec2<T> {
    #[inline(always)]
    fn div_assign(&mut self, rhs: Self) {
        self.0[0] = self.0[0] / rhs.0[0];
        self.0[1] = self.0[1] / rhs.0[1];
    }
}

impl<T: Float> DivAssign<T> for Vec2<T> {
    #[inline(always)]
    fn div_assign(&mut self, rhs: T) {
        self.0[0] = self.0[0] / rhs;
        self.0[1] = self.0[1] / rhs;
    }
}

impl<T> AsRef<Vec2<T>> for Vec2<T> {
    fn as_ref(&self) -> &Self {
        self
    }
}

impl<T> AsRef<[T]> for Vec2<T> {
    fn as_ref(&self) -> &[T] {
        &self.0
    }
}

impl<T: Display> Display for Vec2<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let value = self
            .0
            .iter()
            .map(|v| v.to_string())
            .collect::<Vec<_>>()
            .join(", ");
        f.write_fmt(format_args!("vec2({})", value))
    }
}

#[cfg(test)]
mod tests {
    use std::sync::OnceLock;

    use crate::{
        error::Error,
        mat2::{Mat2, AsMat2},
        mat2d::{AsMat2d, Mat2d},
        vec2::AsVec2,
    };

    use super::Vec2;

    static VEC_A_RAW: [f64; 2] = [1.0, 2.0];
    static VEC_B_RAW: [f64; 2] = [3.0, 4.0];

    static VEC_A: OnceLock<Vec2> = OnceLock::new();
    static VEC_B: OnceLock<Vec2> = OnceLock::new();

    fn vec_a() -> &'static Vec2 {
        VEC_A.get_or_init(|| Vec2::from_slice(&VEC_A_RAW))
    }

    fn vec_b() -> &'static Vec2 {
        VEC_B.get_or_init(|| Vec2::from_slice(&VEC_B_RAW))
    }

    #[test]
    fn new() {
        assert_eq!(Vec2::<f64>::new().to_raw(), [0.0, 0.0]);
    }

    #[test]
    fn from_slice() {
        assert_eq!(Vec2::from_slice(&[3.0, 4.0]).to_raw(), [3.0, 4.0]);
    }

    #[test]
    fn from_values() {
        assert_eq!(Vec2::from_values(3.0, 4.0).to_raw(), [3.0, 4.0]);
    }

    #[test]
    fn ceil() {
        assert_eq!(
            Vec2::from_values(std::f64::consts::E, std::f64::consts::PI)
                .ceil()
                .to_raw(),
            [3.0, 4.0]
        );
    }

    #[test]
    fn floor() -> Result<(), Error> {
        assert_eq!(
            Vec2::from_values(std::f64::consts::E, std::f64::consts::PI)
                .floor()
                .to_raw(),
            [2.0, 3.0]
        );

        Ok(())
    }

    #[test]
    fn min() -> Result<(), Error> {
        let vec_a = Vec2::from_values(1.0, 4.0);
        let vec_b = Vec2::from_values(3.0, 2.0);
        assert_eq!(vec_a.clone().min(&vec_b).to_raw(), [1.0, 2.0]);

        Ok(())
    }

    #[test]
    fn max() -> Result<(), Error> {
        let vec_a = Vec2::from_values(1.0, 4.0);
        let vec_b = Vec2::from_values(3.0, 2.0);
        assert_eq!(vec_a.clone().max(&vec_b).to_raw(), [3.0, 4.0]);

        Ok(())
    }

    #[test]
    fn round() -> Result<(), Error> {
        assert_eq!(
            Vec2::from_values(std::f64::consts::E, std::f64::consts::PI)
                .round()
                .to_raw(),
            [3.0, 3.0]
        );

        Ok(())
    }

    #[test]
    fn scale() {
        assert_eq!((*vec_a() * 2.0).to_raw(), [2.0, 4.0]);
    }

    #[test]
    fn scale_add() {
        assert_eq!((*vec_a() + *vec_b() * 2.0).to_raw(), [7.0, 10.0]);
    }

    #[test]
    fn squared_distance() {
        assert_eq!(vec_a().squared_distance(vec_b()), 8.0);
    }

    #[test]
    fn distance() {
        assert_eq!(vec_a().distance(vec_b()), 2.8284271247461903);
    }

    #[test]
    fn squared_length() {
        assert_eq!(vec_a().squared_length(), 5.0);
    }

    #[test]
    fn length() {
        assert_eq!(vec_a().length(), 2.23606797749979);
    }

    #[test]
    fn negate() {
        assert_eq!(vec_a().clone().negate().raw(), &[-1.0, -2.0]);
    }

    #[test]
    fn normalize() {
        assert_eq!(Vec2::from_values(5.0, 0.0).normalize().raw(), &[1.0, 0.0]);
    }

    #[test]
    fn dot() {
        assert_eq!(vec_a().dot(vec_b()), 11.0);
    }

    #[test]
    fn cross() {
        assert_eq!(vec_a().cross(vec_b()).raw(), &[0.0, 0.0, -2.0]);
    }

    #[test]
    fn lerp() {
        assert_eq!(vec_a().clone().lerp(vec_b(), 0.5).raw(), &[2.0, 3.0]);
    }

    #[test]
    fn angle() {
        let vec_a = Vec2::from_values(1.0, 0.0);
        let vec_b = Vec2::from_values(1.0, 2.0);
        assert_eq!(vec_a.angle(&vec_b), 1.1071487177940904);
    }

    #[test]
    fn transform_mat2() {
        let mat = Mat2::from_values(1.0, 2.0, 3.0, 4.0);
        assert_eq!(vec_a().clone().transform_mat2(&mat).raw(), &[7.0, 10.0]);
    }

    #[test]
    fn transform_mat2d() {
        let mat = Mat2d::from_values(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        assert_eq!(vec_a().clone().transform_mat2d(&mat).raw(), &[12.0, 16.0]);
    }

    #[test]
    fn set() {
        let mut mat = Vec2::new();
        mat.set(3.0, 4.0);

        assert_eq!(mat.raw(), &[3.0, 4.0]);
    }

    #[test]
    fn set_slice() {
        let mut mat = Vec2::new();
        mat.set_slice(&[3.0, 4.0]);

        assert_eq!(mat.raw(), &[3.0, 4.0]);
    }

    #[test]
    fn add() {
        assert_eq!((*vec_a() + *vec_b()).raw(), &[4.0, 6.0]);
    }

    #[test]
    fn sub() {
        assert_eq!((*vec_a() - *vec_b()).raw(), &[-2.0, -2.0]);
    }

    #[test]
    fn mul() {
        assert_eq!((*vec_a() * *vec_b()).raw(), &[3.0, 8.0]);
    }

    #[test]
    fn mul_scalar() {
        assert_eq!((*vec_a() * 2.0).raw(), &[2.0, 4.0]);
    }

    #[test]
    fn mul_scalar_add() {
        assert_eq!((*vec_a() + *vec_b() * 0.5).raw(), &[2.5, 4.0]);
    }

    #[test]
    fn div() {
        assert_eq!((*vec_a() / *vec_b()).raw(), &[0.3333333333333333, 0.5]);
    }

    #[test]
    fn div_scalar() {
        assert_eq!((*vec_a() / 2.0).raw(), &[0.5, 1.0]);
    }

    #[test]
    fn div_scalar_add() {
        assert_eq!((*vec_a() + *vec_b() / 0.5).raw(), &[7.0, 10.0]);
    }

    #[test]
    fn approximate_eq() {
        let vec_a = Vec2::from_values(0.0, 1.0);
        let vec_b = Vec2::from_values(0.0, 1.0);
        let vec_c = Vec2::from_values(1.0, 2.0);
        let vec_d = Vec2::from_values(1e-16, 1.0);

        assert_eq!(true, vec_a.approximate_eq(&vec_b));
        assert_eq!(false, vec_a.approximate_eq(&vec_c));
        assert_eq!(true, vec_a.approximate_eq(&vec_d));
    }

    #[test]
    fn display() {
        let out = vec_a().to_string();
        assert_eq!(out, "vec2(1, 2)");
    }

    #[test]
    fn rotate() {
        let vec_a = Vec2::from_values(0.0, 1.0);
        let vec_b = Vec2::from_values(0.0, 0.0);
        assert_eq!(
            vec_a.clone().rotate(&vec_b, std::f64::consts::PI).to_raw(),
            [-1.2246467991473532e-16, -1.0]
        );

        let vec_a = Vec2::from_values(6.0, -5.0);
        let vec_b = Vec2::from_values(0.0, -5.0);
        assert_eq!(
            vec_a.clone().rotate(&vec_b, std::f64::consts::PI).to_raw(),
            [-6.0, -4.999999999999999]
        );
    }
}
