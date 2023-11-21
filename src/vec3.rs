use std::{
    fmt::Display,
    ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub, SubAssign},
};

use half::f16;
use num_traits::Float;

use crate::{epsilon, mat3::AsMat3, mat4::AsMat4, quat::AsQuat};

pub trait AsVec3<T: Float> {
    fn from_values(x: T, y: T, z: T) -> Self;

    fn x(&self) -> T;

    fn y(&self) -> T;

    fn z(&self) -> T;

    fn set_x(&mut self, x: T) -> &mut Self;

    fn set_y(&mut self, y: T) -> &mut Self;

    fn set_z(&mut self, z: T) -> &mut Self;

    #[inline(always)]
    fn to_raw(&self) -> [T; 3] {
        [self.x(), self.y(), self.z()]
    }

    #[inline(always)]
    fn to_gl(&self) -> [f32; 3] {
        [
            T::to_f32(&self.x()).unwrap(),
            T::to_f32(&self.y()).unwrap(),
            T::to_f32(&self.z()).unwrap(),
        ]
    }

    #[inline(always)]
    fn to_gl_binary(&self) -> [u8; 12] {
        unsafe { std::mem::transmute_copy::<[f32; 3], [u8; 12]>(&self.to_gl()) }
    }

    #[inline(always)]
    fn copy<V: AsVec3<T> + ?Sized>(&mut self, b: &V) -> &mut Self {
        self.set_x(b.x()).set_y(b.y()).set_z(b.z())
    }

    #[inline(always)]
    fn set(&mut self, x: T, y: T, z: T) -> &mut Self {
        self.set_x(x).set_y(y).set_z(z)
    }

    #[inline(always)]
    fn set_slice(&mut self, [x, y, z]: &[T; 3]) -> &mut Self {
        self.set_x(*x).set_y(*y).set_z(*z)
    }

    #[inline(always)]
    fn set_zero(&mut self) -> &mut Self {
        self.set_x(T::zero()).set_y(T::zero()).set_z(T::zero())
    }

    #[inline(always)]
    fn ceil(&self) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();
        let z = self.z();

        Self::from_values(x.ceil(), y.ceil(), z.ceil())
    }

    #[inline(always)]
    fn floor(&self) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();
        let z = self.z();

        Self::from_values(x.floor(), y.floor(), z.floor())
    }

    #[inline(always)]
    fn round(&self) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();
        let z = self.z();

        Self::from_values(x.round(), y.round(), z.round())
    }

    #[inline(always)]
    fn min<V: AsVec3<T> + ?Sized>(&self, b: &V) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();
        let z = self.z();

        Self::from_values(x.min(b.x()), y.min(b.y()), z.min(b.z()))
    }

    #[inline(always)]
    fn max<V: AsVec3<T> + ?Sized>(&self, b: &V) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();
        let z = self.z();

        Self::from_values(x.max(b.x()), y.max(b.y()), z.max(b.z()))
    }

    #[inline(always)]
    fn scale(&self, scale: T) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();
        let z = self.z();

        Self::from_values(x * scale, y * scale, z * scale)
    }

    #[inline(always)]
    fn squared_distance<V: AsVec3<T> + ?Sized>(&self, b: &V) -> T {
        let x = b.x() - self.x();
        let y = b.y() - self.y();
        let z = b.z() - self.z();
        x * x + y * y + z * z
    }

    #[inline(always)]
    fn distance<V: AsVec3<T> + ?Sized>(&self, b: &V) -> T {
        self.squared_distance(b).sqrt()
    }

    #[inline(always)]
    fn squared_length(&self) -> T {
        let x = self.x();
        let y = self.y();
        let z = self.z();
        x * x + y * y + z * z
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

        Self::from_values(x.neg(), y.neg(), z.neg())
    }

    #[inline(always)]
    fn inverse(&self) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();
        let z = self.z();

        Self::from_values(T::one() / x, T::one() / y, T::one() / z)
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

        Self::from_values(x * len, y * len, z * len)
    }

    #[inline(always)]
    fn dot<V: AsVec3<T> + ?Sized>(&self, b: &V) -> T {
        self.x() * b.x() + self.y() * b.y() + self.z() * b.z()
    }

    #[inline(always)]
    fn cross<V: AsVec3<T> + ?Sized>(&self, b: &V) -> Self
    where
        Self: Sized,
    {
        let ax = self.x();
        let ay = self.y();
        let az = self.z();
        let bx = b.x();
        let by = b.y();
        let bz = b.z();

        Self::from_values(ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx)
    }

    #[inline(always)]
    fn lerp<V: AsVec3<T> + ?Sized>(&self, b: &V, t: T) -> Self
    where
        Self: Sized,
    {
        let ax = self.x();
        let ay = self.y();
        let az = self.z();

        Self::from_values(
            ax + t * (b.x() - ax),
            ay + t * (b.y() - ay),
            az + t * (b.z() - az),
        )
    }

    #[inline(always)]
    fn slerp<V: AsVec3<T> + ?Sized>(&self, b: &V, t: T) -> Self
    where
        Self: Sized,
    {
        let angle = self.dot(b).max(-T::one()).min(T::one()).acos();
        let sin_total = angle.sin();

        let ratio_a = ((T::one() - t) * angle).sin() / sin_total;
        let ratio_b = (t * angle).sin() / sin_total;

        let ax = self.x();
        let ay = self.y();
        let az = self.z();

        Self::from_values(
            ratio_a * ax + ratio_b * b.x(),
            ratio_a * ay + ratio_b * b.y(),
            ratio_a * az + ratio_b * b.z(),
        )
    }

    #[inline(always)]
    fn hermite<V1, V2, V3>(&self, b: &V1, c: &V2, d: &V3, t: T) -> Self
    where
        Self: Sized,

        V1: AsVec3<T> + ?Sized,
        V2: AsVec3<T> + ?Sized,
        V3: AsVec3<T> + ?Sized,
    {
        let factor_times2 = t * t;
        let factor1 =
            factor_times2 * (T::from(2.0).unwrap() * t - T::from(3.0).unwrap()) + T::one();
        let factor2 = factor_times2 * (t - T::from(2.0).unwrap()) + t;
        let factor3 = factor_times2 * (t - T::one());
        let factor4 = factor_times2 * (T::from(3.0).unwrap() - T::from(2.0).unwrap() * t);

        let ax = self.x();
        let ay = self.y();
        let az = self.z();

        Self::from_values(
            ax * factor1 + b.x() * factor2 + c.x() * factor3 + d.x() * factor4,
            ay * factor1 + b.y() * factor2 + c.y() * factor3 + d.y() * factor4,
            az * factor1 + b.z() * factor2 + c.z() * factor3 + d.z() * factor4,
        )
    }

    #[inline(always)]
    fn bezier<V1, V2, V3>(&self, b: &V1, c: &V2, d: &V3, t: T) -> Self
    where
        Self: Sized,
        V1: AsVec3<T> + ?Sized,
        V2: AsVec3<T> + ?Sized,
        V3: AsVec3<T> + ?Sized,
    {
        let inverse_factor = T::one() - t;
        let inverse_factor_times_two = inverse_factor * inverse_factor;
        let factor_times2 = t * t;
        let factor1 = inverse_factor_times_two * inverse_factor;
        let factor2 = T::from(3.0).unwrap() * t * inverse_factor_times_two;
        let factor3 = T::from(3.0).unwrap() * factor_times2 * inverse_factor;
        let factor4 = factor_times2 * t;

        let ax = self.x();
        let ay = self.y();
        let az = self.z();

        Self::from_values(
            ax * factor1 + b.x() * factor2 + c.x() * factor3 + d.x() * factor4,
            ay * factor1 + b.y() * factor2 + c.y() * factor3 + d.y() * factor4,
            az * factor1 + b.z() * factor2 + c.z() * factor3 + d.z() * factor4,
        )
    }

    #[inline(always)]
    fn transform_mat3<M: AsMat3<T> + ?Sized>(&self, m: &M) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();
        let z = self.z();

        Self::from_values(
            x * m.m00() + y * m.m10() + z * m.m20(),
            x * m.m01() + y * m.m11() + z * m.m21(),
            x * m.m02() + y * m.m12() + z * m.m22(),
        )
    }

    #[inline(always)]
    fn transform_quat<Q: AsQuat<T> + ?Sized>(&self, q: &Q) -> Self
    where
        Self: Sized,
    {
        // benchmarks: https://jsperf.com/quaternion-transform-vec3-implementations-fixed
        let qx = q.x();
        let qy = q.y();
        let qz = q.z();
        let qw = q.w();
        let x = self.x();
        let y = self.y();
        let z = self.z();
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
        let w2 = qw * T::from(2.0).unwrap();
        uvx = uvx * w2;
        uvy = uvy * w2;
        uvz = uvz * w2;
        // vec3.scale(uuv, uuv, 2);
        uuvx = uuvx * T::from(2.0).unwrap();
        uuvy = uuvy * T::from(2.0).unwrap();
        uuvz = uuvz * T::from(2.0).unwrap();
        // return vec3.add(out, a, vec3.add(out, uv, uuv));

        Self::from_values(x + uvx + uuvx, y + uvy + uuvy, z + uvz + uuvz)
    }

    #[inline(always)]
    fn transform_mat4<M: AsMat4<T> + ?Sized>(&self, m: &M) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();
        let z = self.z();

        let [m00, m01, m02, m03, m04, m05, m06, m07, m08, m09, m10, m11, m12, m13, m14, m15] =
            m.to_raw();

        let mut w = m03 * x + m07 * y + m11 * z + m15;
        w = if w == T::zero() { T::one() } else { w };

        Self::from_values(
            (m00 * x + m04 * y + m08 * z + m12) / w,
            (m01 * x + m05 * y + m09 * z + m13) / w,
            (m02 * x + m06 * y + m10 * z + m14) / w,
        )
    }

    #[inline(always)]
    fn rotate_x<V: AsVec3<T> + ?Sized>(&self, b: &V, rad: T) -> Self
    where
        Self: Sized,
    {
        let mut p = [T::zero(); 3];
        let mut r = [T::zero(); 3];

        //Translate point to the origin
        p[0] = self.x() - b.x();
        p[1] = self.y() - b.y();
        p[2] = self.z() - b.z();

        //perform rotation
        r[0] = p[0];
        r[1] = p[1] * rad.cos() - p[2] * rad.sin();
        r[2] = p[1] * rad.sin() + p[2] * rad.cos();

        Self::from_values(r[0] + b.x(), r[1] + b.y(), r[2] + b.z())
    }

    #[inline(always)]
    fn rotate_y<V: AsVec3<T> + ?Sized>(&self, b: &V, rad: T) -> Self
    where
        Self: Sized,
    {
        let mut p = [T::zero(); 3];
        let mut r = [T::zero(); 3];

        //Translate point to the origin
        p[0] = self.x() - b.x();
        p[1] = self.y() - b.y();
        p[2] = self.z() - b.z();

        //perform rotation
        r[0] = p[2] * rad.sin() + p[0] * rad.cos();
        r[1] = p[1];
        r[2] = p[2] * rad.cos() - p[0] * rad.sin();

        Self::from_values(r[0] + b.x(), r[1] + b.y(), r[2] + b.z())
    }

    #[inline(always)]
    fn rotate_z<V: AsVec3<T> + ?Sized>(&self, b: &V, rad: T) -> Self
    where
        Self: Sized,
    {
        let mut p = [T::zero(); 3];
        let mut r = [T::zero(); 3];

        //Translate point to the origin
        p[0] = self.x() - b.x();
        p[1] = self.y() - b.y();
        p[2] = self.z() - b.z();

        //perform rotation
        r[0] = p[0] * rad.cos() - p[1] * rad.sin();
        r[1] = p[0] * rad.sin() + p[1] * rad.cos();
        r[2] = p[2];

        Self::from_values(r[0] + b.x(), r[1] + b.y(), r[2] + b.z())
    }

    #[inline(always)]
    fn angle<V: AsVec3<T> + ?Sized>(&self, b: &V) -> T {
        let ax = self.x();
        let ay = self.y();
        let az = self.z();
        let bx = b.x();
        let by = b.y();
        let bz = b.z();
        let mag = ((ax * ax + ay * ay + az * az) * (bx * bx + by * by + bz * bz)).sqrt();
        let cosine = if mag == T::zero() {
            mag
        } else {
            self.dot(b) / mag
        };
        // Math.min(Math.max(cosine, -1), 1) clamps the cosine between -1 and 1
        cosine.max(-T::one()).min(T::one()).acos()
    }

    #[inline(always)]
    fn approximate_eq<V: AsVec3<T> + ?Sized>(&self, b: &V) -> bool {
        let a0 = self.x();
        let a1 = self.y();
        let a2 = self.z();
        let b0 = b.x();
        let b1 = b.y();
        let b2 = b.z();

        (a0 - b0).abs() <= epsilon::<T>() * T::one().max(a0.abs()).max(b0.abs())
            && (a1 - b1).abs() <= epsilon::<T>() * T::one().max(a1.abs()).max(b1.abs())
            && (a2 - b2).abs() <= epsilon::<T>() * T::one().max(a2.abs()).max(b2.abs())
    }
}

impl<T: Float> AsVec3<T> for [T; 3] {
    #[inline(always)]
    fn from_values(x: T, y: T, z: T) -> Self {
        [x, y, z]
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
}

impl<T: Float> AsVec3<T> for (T, T, T) {
    #[inline(always)]
    fn from_values(x: T, y: T, z: T) -> Self {
        (x, y, z)
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
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Vec3<T = f64>(pub [T; 3]);

impl<T: Float> Vec3<T> {
    #[inline(always)]
    pub fn new() -> Self {
        Self([T::zero(); 3])
    }

    #[inline(always)]
    pub fn from_slice(slice: &[T; 3]) -> Self {
        Self(slice.clone())
    }

    #[inline(always)]
    pub fn raw(&self) -> &[T; 3] {
        &self.0
    }
}

impl<T: Float> AsVec3<T> for Vec3<T> {
    #[inline(always)]
    fn from_values(x: T, y: T, z: T) -> Self {
        Self([x, y, z])
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
}

impl Vec3<f64> {
    #[inline(always)]
    pub fn random(&self, scale: Option<f64>) -> Self {
        let scale = match scale {
            Some(scale) => scale,
            None => 1.0,
        };

        let r = rand::random::<f64>() * 2.0 * std::f64::consts::PI;
        let z = rand::random::<f64>() * 2.0 - 1.0;
        let z_scale = (1.0 - z * z).sqrt() * scale;

        Self([r.cos() * z_scale, r.sin() * z_scale, z * scale])
    }
}

impl Vec3<f32> {
    #[inline(always)]
    pub fn random(&self, scale: Option<f32>) -> Self {
        let scale = match scale {
            Some(scale) => scale,
            None => 1.0,
        };

        let r = rand::random::<f32>() * 2.0 * std::f32::consts::PI;
        let z = rand::random::<f32>() * 2.0 - 1.0;
        let z_scale = (1.0 - z * z).sqrt() * scale;

        Self([r.cos() * z_scale, r.sin() * z_scale, z * scale])
    }
}

impl Vec3<f16> {
    #[inline(always)]
    pub fn random(&self, scale: Option<f16>) -> Self {
        let scale = match scale {
            Some(scale) => scale,
            None => f16::from_f32_const(1.0),
        };

        let r = rand::random::<f16>() * f16::from_f32_const(2.0) * f16::PI;
        let z = rand::random::<f16>() * f16::from_f32_const(2.0) - f16::from_f32_const(1.0);
        let z_scale = (f16::from_f32_const(1.0) - z * z).sqrt() * scale;

        Self([r.cos() * z_scale, r.sin() * z_scale, z * scale])
    }
}

impl<T: Float> Default for Vec3<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Float> Add<Vec3<T>> for Vec3<T> {
    type Output = Self;

    #[inline(always)]
    fn add(self, b: Self) -> Self {
        Self([self.0[0] + b.0[0], self.0[1] + b.0[1], self.0[2] + b.0[2]])
    }
}

impl<T: Float> Add<T> for Vec3<T> {
    type Output = Self;

    #[inline(always)]
    fn add(self, b: T) -> Self {
        Self([self.0[0] + b, self.0[1] + b, self.0[2] + b])
    }
}

impl<T: Float> Sub<Vec3<T>> for Vec3<T> {
    type Output = Self;

    #[inline(always)]
    fn sub(self, b: Self) -> Self {
        Self([self.0[0] - b.0[0], self.0[1] - b.0[1], self.0[2] - b.0[2]])
    }
}

impl<T: Float> Sub<T> for Vec3<T> {
    type Output = Self;

    #[inline(always)]
    fn sub(self, b: T) -> Self {
        Self([self.0[0] - b, self.0[1] - b, self.0[2] - b])
    }
}

impl<T: Float> Mul<Vec3<T>> for Vec3<T> {
    type Output = Self;

    #[inline(always)]
    fn mul(self, b: Self) -> Self {
        Self([self.0[0] * b.0[0], self.0[1] * b.0[1], self.0[2] * b.0[2]])
    }
}

impl<T: Float> Mul<T> for Vec3<T> {
    type Output = Self;

    #[inline(always)]
    fn mul(self, b: T) -> Self {
        Self([self.0[0] * b, self.0[1] * b, self.0[2] * b])
    }
}

impl<T: Float> Div<Vec3<T>> for Vec3<T> {
    type Output = Self;

    #[inline(always)]
    fn div(self, b: Self) -> Self {
        Self([self.0[0] / b.0[0], self.0[1] / b.0[1], self.0[2] / b.0[2]])
    }
}

impl<T: Float> Div<T> for Vec3<T> {
    type Output = Self;

    #[inline(always)]
    fn div(self, b: T) -> Self {
        Self([self.0[0] / b, self.0[1] / b, self.0[2] / b])
    }
}

impl<T: Float> AddAssign<Self> for Vec3<T> {
    fn add_assign(&mut self, rhs: Self) {
        self.0[0] = self.0[0] + rhs.0[0];
        self.0[1] = self.0[1] + rhs.0[1];
        self.0[2] = self.0[2] + rhs.0[2];
    }
}

impl<T: Float> AddAssign<T> for Vec3<T> {
    fn add_assign(&mut self, rhs: T) {
        self.0[0] = self.0[0] + rhs;
        self.0[1] = self.0[1] + rhs;
        self.0[2] = self.0[2] + rhs;
    }
}

impl<T: Float> SubAssign<Self> for Vec3<T> {
    fn sub_assign(&mut self, rhs: Self) {
        self.0[0] = self.0[0] - rhs.0[0];
        self.0[1] = self.0[1] - rhs.0[1];
        self.0[2] = self.0[2] - rhs.0[2];
    }
}

impl<T: Float> SubAssign<T> for Vec3<T> {
    fn sub_assign(&mut self, rhs: T) {
        self.0[0] = self.0[0] - rhs;
        self.0[1] = self.0[1] - rhs;
        self.0[2] = self.0[2] - rhs;
    }
}

impl<T: Float> MulAssign<Self> for Vec3<T> {
    fn mul_assign(&mut self, rhs: Self) {
        self.0[0] = self.0[0] * rhs.0[0];
        self.0[1] = self.0[1] * rhs.0[1];
        self.0[2] = self.0[2] * rhs.0[2];
    }
}

impl<T: Float> MulAssign<T> for Vec3<T> {
    fn mul_assign(&mut self, rhs: T) {
        self.0[0] = self.0[0] * rhs;
        self.0[1] = self.0[1] * rhs;
        self.0[2] = self.0[2] * rhs;
    }
}

impl<T: Float> DivAssign<Self> for Vec3<T> {
    fn div_assign(&mut self, rhs: Self) {
        self.0[0] = self.0[0] / rhs.0[0];
        self.0[1] = self.0[1] / rhs.0[1];
        self.0[2] = self.0[2] / rhs.0[2];
    }
}

impl<T: Float> DivAssign<T> for Vec3<T> {
    fn div_assign(&mut self, rhs: T) {
        self.0[0] = self.0[0] / rhs;
        self.0[1] = self.0[1] / rhs;
        self.0[2] = self.0[2] / rhs;
    }
}

impl<T> AsRef<Vec3<T>> for Vec3<T> {
    fn as_ref(&self) -> &Self {
        self
    }
}

impl<T> AsRef<[T]> for Vec3<T> {
    fn as_ref(&self) -> &[T] {
        &self.0
    }
}

impl<T: Display> Display for Vec3<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let value = self
            .0
            .iter()
            .map(|v| v.to_string())
            .collect::<Vec<_>>()
            .join(", ");
        f.write_fmt(format_args!("vec3({})", value))
    }
}

#[cfg(test)]
mod tests {
    use std::sync::OnceLock;

    use crate::{
        error::Error,
        mat3::{AsMat3, Mat3},
        mat4::{AsMat4, Mat4},
        quat::{AsQuat, Quat},
        vec3::{AsVec3, Vec3},
    };

    static VEC_A_RAW: [f64; 3] = [1.0, 2.0, 3.0];
    static VEC_B_RAW: [f64; 3] = [4.0, 5.0, 6.0];

    static VEC_A: OnceLock<Vec3> = OnceLock::new();
    static VEC_B: OnceLock<Vec3> = OnceLock::new();

    fn vec_a() -> &'static Vec3 {
        VEC_A.get_or_init(|| Vec3::from_slice(&VEC_A_RAW))
    }

    fn vec_b() -> &'static Vec3 {
        VEC_B.get_or_init(|| Vec3::from_slice(&VEC_B_RAW))
    }

    #[test]
    fn new() {
        assert_eq!(Vec3::<f64>::new().to_raw(), [0.0, 0.0, 0.0]);
    }

    #[test]
    fn from_slice() {
        assert_eq!(Vec3::from_slice(&[3.0, 4.0, 5.0]).to_raw(), [3.0, 4.0, 5.0]);
    }

    #[test]
    fn from_values() {
        assert_eq!(Vec3::from_values(3.0, 4.0, 5.0).to_raw(), [3.0, 4.0, 5.0]);
    }

    #[test]
    fn ceil() {
        assert_eq!(
            Vec3::from_values(
                std::f64::consts::E,
                std::f64::consts::PI,
                std::f64::consts::SQRT_2
            )
            .ceil()
            .to_raw(),
            [3.0, 4.0, 2.0]
        );
    }

    #[test]
    fn floor() {
        assert_eq!(
            Vec3::from_values(
                std::f64::consts::E,
                std::f64::consts::PI,
                std::f64::consts::SQRT_2
            )
            .floor()
            .to_raw(),
            [2.0, 3.0, 1.0]
        );
    }

    #[test]
    fn min() -> Result<(), Error> {
        assert_eq!(
            Vec3::from_values(1.0, 3.0, 1.0)
                .min(&[3.0, 1.0, 3.0])
                .to_raw(),
            [1.0, 1.0, 1.0]
        );

        Ok(())
    }

    #[test]
    fn max() -> Result<(), Error> {
        assert_eq!(
            Vec3::from_values(1.0, 3.0, 1.0)
                .max(&[3.0, 1.0, 3.0])
                .to_raw(),
            [3.0, 3.0, 3.0]
        );

        Ok(())
    }

    #[test]
    fn round() -> Result<(), Error> {
        let vec = Vec3::from_values(
            std::f64::consts::E,
            std::f64::consts::PI,
            std::f64::consts::SQRT_2,
        )
        .round();
        assert_eq!(vec.to_raw(), [3.0, 3.0, 1.0]);

        Ok(())
    }

    #[test]
    fn scale() {
        assert_eq!((*vec_a() * 2.0).to_raw(), [2.0, 4.0, 6.0]);
    }

    #[test]
    fn scale_add() {
        assert_eq!((*vec_a() + *vec_b() * 0.5).to_raw(), [3.0, 4.5, 6.0]);
    }

    #[test]
    fn squared_distance() {
        assert_eq!(vec_a().squared_distance(vec_b()), 27.0);
    }

    #[test]
    fn distance() {
        assert_eq!(vec_a().distance(vec_b()), 5.196152422706632);
    }

    #[test]
    fn squared_length() {
        assert_eq!(vec_a().squared_length(), 14.0);
    }

    #[test]
    fn length() {
        assert_eq!(vec_a().length(), 3.7416573867739413);
    }

    #[test]
    fn negate() {
        assert_eq!(vec_a().negate().to_raw(), [-1.0, -2.0, -3.0]);
    }

    #[test]
    fn normalize() {
        assert_eq!(
            Vec3::from_values(5.0, 0.0, 0.0).normalize().to_raw(),
            [1.0, 0.0, 0.0]
        );
    }

    #[test]
    fn dot() {
        assert_eq!(vec_a().dot(vec_b()), 32.0);
    }

    #[test]
    fn cross() {
        assert_eq!(vec_a().cross(vec_b()).to_raw(), [-3.0, 6.0, -3.0]);
    }

    #[test]
    fn lerp() {
        assert_eq!(vec_a().lerp(vec_b(), 0.5).to_raw(), [2.5, 3.5, 4.5]);
    }

    #[test]
    fn slerp() {
        let vec_a = Vec3::from_values(1.0, 0.0, 0.0);
        let vec_b = [0.0, 1.0, 0.0];
        assert_eq!(vec_a.slerp(&vec_b, 0.0).to_raw(), [1.0, 0.0, 0.0]);
        assert_eq!(vec_a.slerp(&vec_b, 1.0).to_raw(), [0.0, 1.0, 0.0]);
        assert_eq!(
            vec_a.slerp(&vec_b, 0.5).to_raw(),
            [0.7071067811865475, 0.7071067811865475, 0.0]
        );
    }

    #[test]
    fn transform_mat4() {
        let mat = Mat4::from_values(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        );
        assert_eq!(vec_a().transform_mat4(&mat).to_raw(), [1.0, 2.0, 3.0]);
    }

    #[test]
    fn transform_mat3() {
        let mat = Mat3::from_values(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        assert_eq!(vec_a().transform_mat3(&mat).to_raw(), [1.0, 2.0, 3.0]);
    }

    #[test]
    fn transform_quat() {
        let quat = Quat::from_values(
            0.18257418567011074,
            0.3651483713402215,
            0.5477225570103322,
            0.730296742680443,
        );
        assert_eq!(vec_a().transform_quat(&quat).to_raw(), [1.0, 2.0, 3.0]);
    }

    #[test]
    fn set() {
        let mut mat = Vec3::new();
        mat.set(3.0, 4.0, 5.0);

        assert_eq!(mat.to_raw(), [3.0, 4.0, 5.0]);
    }

    #[test]
    fn set_slice() {
        let mut mat = Vec3::new();
        mat.set_slice(&[3.0, 4.0, 5.0]);

        assert_eq!(mat.to_raw(), [3.0, 4.0, 5.0]);
    }

    #[test]
    fn add() {
        assert_eq!((*vec_a() + *vec_b()).to_raw(), [5.0, 7.0, 9.0]);
    }

    #[test]
    fn sub() {
        assert_eq!((*vec_a() - *vec_b()).to_raw(), [-3.0, -3.0, -3.0]);
    }

    #[test]
    fn mul() {
        assert_eq!((*vec_a() * *vec_b()).to_raw(), [4.0, 10.0, 18.0]);
    }

    #[test]
    fn mul_scalar() {
        assert_eq!((*vec_a() * 2.0).to_raw(), [2.0, 4.0, 6.0]);
    }

    #[test]
    fn mul_scalar_add() {
        assert_eq!((*vec_a() + *vec_b() * 0.5).to_raw(), [3.0, 4.5, 6.0]);
    }

    #[test]
    fn div() {
        assert_eq!((*vec_a() / *vec_b()).to_raw(), [0.25, 0.4, 0.5]);
    }

    #[test]
    fn div_scalar() {
        assert_eq!((*vec_a() / 2.0).to_raw(), [0.5, 1.0, 1.5]);
    }

    #[test]
    fn div_scalar_add() {
        assert_eq!((*vec_a() + *vec_b() / 0.5).to_raw(), [9.0, 12.0, 15.0]);
    }

    #[test]
    fn approximate_eq() {
        let vec_a = Vec3::from_values(0.0, 1.0, 2.0);
        let vec_b = Vec3::from_values(0.0, 1.0, 2.0);
        let vec_c = Vec3::from_values(1.0, 2.0, 3.0);
        let vec_d = Vec3::from_values(1e-16, 1.0, 2.0);

        assert_eq!(true, vec_a.approximate_eq(&vec_b));
        assert_eq!(false, vec_a.approximate_eq(&vec_c));
        assert_eq!(true, vec_a.approximate_eq(&vec_d));
    }

    #[test]
    fn display() {
        let out = vec_a().to_string();
        assert_eq!(out, "vec3(1, 2, 3)");
    }

    #[test]
    fn angle() {
        assert_eq!(vec_a().angle(vec_b()), 0.2257261285527342);
    }

    #[test]
    fn rotate_x() {
        let vec_a = Vec3::from_values(0.0, 1.0, 0.0);
        let vec_b = Vec3::from_values(0.0, 0.0, 0.0);
        assert_eq!(
            vec_a.rotate_x(&vec_b, std::f64::consts::PI).to_raw(),
            [0.0, -1.0, 1.2246467991473532e-16]
        );

        let vec_a = Vec3::from_values(2.0, 7.0, 0.0);
        let vec_b = Vec3::from_values(2.0, 5.0, 0.0);
        assert_eq!(
            vec_a.rotate_x(&vec_b, std::f64::consts::PI).to_raw(),
            [2.0, 3.0, 2.4492935982947064e-16]
        );
    }

    #[test]
    fn rotate_y() {
        let vec_a = Vec3::from_values(1.0, 0.0, 0.0);
        let vec_b = Vec3::from_values(0.0, 0.0, 0.0);
        assert_eq!(
            vec_a.rotate_y(&vec_b, std::f64::consts::PI).to_raw(),
            [-1.0, 0.0, -1.2246467991473532e-16]
        );

        let vec_a = Vec3::from_values(-2.0, 3.0, 10.0);
        let vec_b = Vec3::from_values(-4.0, 3.0, 10.0);
        assert_eq!(
            vec_a.rotate_y(&vec_b, std::f64::consts::PI).to_raw(),
            [-6.0, 3.0, 10.0]
        );
    }

    #[test]
    fn rotate_z() {
        let vec_a = Vec3::from_values(0.0, 1.0, 0.0);
        let vec_b = Vec3::from_values(0.0, 0.0, 0.0);
        assert_eq!(
            vec_a.rotate_z(&vec_b, std::f64::consts::PI).to_raw(),
            [-1.2246467991473532e-16, -1.0, 0.0]
        );

        let vec_a = Vec3::from_values(0.0, 6.0, -5.0);
        let vec_b = Vec3::from_values(0.0, 0.0, -5.0);
        assert_eq!(
            vec_a.rotate_z(&vec_b, std::f64::consts::PI).to_raw(),
            [-7.347880794884119e-16, -6.0, -5.0]
        );
    }
}
