use std::{
    fmt::Display,
    ops::{Add, Div, Mul, Sub},
};

use num_traits::Float;

use crate::{epsilon, mat3::Mat3, mat4::Mat4, quat::Quat};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Vec3<T = f32>(pub [T; 3]);

impl<T: Float> Vec3<T> {
    #[inline(always)]
    pub fn new() -> Self {
        Self([T::zero(); 3])
    }

    #[inline(always)]
    pub fn from_values(x: T, y: T, z: T) -> Self {
        Self([x, y, z])
    }

    #[inline(always)]
    pub fn from_slice(slice: &[T; 3]) -> Self {
        Self(slice.clone())
    }
}

impl<T: Float> Vec3<T> {
    #[inline(always)]
    pub fn raw(&self) -> &[T; 3] {
        &self.0
    }

    #[inline(always)]
    pub fn set(&mut self, x: T, y: T, z: T) -> &mut Self {
        self.0[0] = x;
        self.0[1] = y;
        self.0[2] = z;
        self
    }

    #[inline(always)]
    pub fn set_slice(&mut self, [x, y, z]: &[T; 3]) -> &mut Self {
        self.0[0] = *x;
        self.0[1] = *y;
        self.0[2] = *z;
        self
    }

    #[inline(always)]
    pub fn set_zero(&mut self) -> &mut Self {
        self.0[0] = T::zero();
        self.0[1] = T::zero();
        self.0[2] = T::zero();
        self
    }

    #[inline(always)]
    pub fn ceil(&self) -> Self {
        Self([self.0[0].ceil(), self.0[1].ceil(), self.0[2].ceil()])
    }

    #[inline(always)]
    pub fn floor(&self) -> Self {
        Self([self.0[0].floor(), self.0[1].floor(), self.0[2].floor()])
    }

    #[inline(always)]
    pub fn min(&self, b: &Self) -> Self {
        Self([
            self.0[0].min(b.0[0]),
            self.0[1].min(b.0[1]),
            self.0[2].min(b.0[2]),
        ])
    }

    #[inline(always)]
    pub fn max(&self, b: &Self) -> Self {
        Self([
            self.0[0].max(b.0[0]),
            self.0[1].max(b.0[1]),
            self.0[2].max(b.0[2]),
        ])
    }

    #[inline(always)]
    pub fn round(&self) -> Self {
        Self([self.0[0].round(), self.0[1].round(), self.0[2].round()])
    }

    #[inline(always)]
    pub fn scale(&self, scale: T) -> Self {
        self.mul(scale)
    }

    #[inline(always)]
    pub fn squared_distance(&self, b: &Self) -> T {
        let x = b.0[0] - self.0[0];
        let y = b.0[1] - self.0[1];
        let z = b.0[2] - self.0[2];
        x * x + y * y + z * z
    }

    #[inline(always)]
    pub fn distance(&self, b: &Self) -> T {
        self.squared_distance(b).sqrt()
    }

    #[inline(always)]
    pub fn squared_length(&self) -> T {
        let x = self.0[0];
        let y = self.0[1];
        let z = self.0[2];
        x * x + y * y + z * z
    }

    #[inline(always)]
    pub fn length(&self) -> T {
        self.squared_length().sqrt()
    }

    #[inline(always)]
    pub fn negate(&self) -> Self {
        Self([-self.0[0], -self.0[1], -self.0[2]])
    }

    #[inline(always)]
    pub fn inverse(&self) -> Self {
        Self([
            T::one() / self.0[0],
            T::one() / self.0[1],
            T::one() / self.0[2],
        ])
    }

    #[inline(always)]
    pub fn normalize(&self) -> Self {
        let mut len = self.squared_length();
        if len > T::zero() {
            len = T::one() / len.sqrt();
        }

        Self([self.0[0] * len, self.0[1] * len, self.0[2] * len])
    }

    #[inline(always)]
    pub fn dot(&self, b: &Self) -> T {
        self.0[0] * b.0[0] + self.0[1] * b.0[1] + self.0[2] * b.0[2]
    }

    #[inline(always)]
    pub fn cross(&self, b: &Self) -> Self {
        let ax = self.0[0];
        let ay = self.0[1];
        let az = self.0[2];
        let bx = b.0[0];
        let by = b.0[1];
        let bz = b.0[2];

        Self([ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx])
    }

    #[inline(always)]
    pub fn lerp(&self, b: &Self, t: T) -> Self {
        let ax = self.0[0];
        let ay = self.0[1];
        let az = self.0[2];
        Self([
            ax + t * (b.0[0] - ax),
            ay + t * (b.0[1] - ay),
            az + t * (b.0[2] - az),
        ])
    }

    #[inline(always)]
    pub fn slerp(&self, b: &Self, t: T) -> Self {
        let angle = self.dot(b).max(-T::one()).min(T::one()).acos();
        let sin_total = angle.sin();

        let ratio_a = ((T::one() - t) * angle).sin() / sin_total;
        let ratio_b = (t * angle).sin() / sin_total;
        Self([
            ratio_a * self.0[0] + ratio_b * b.0[0],
            ratio_a * self.0[1] + ratio_b * b.0[1],
            ratio_a * self.0[2] + ratio_b * b.0[2],
        ])
    }

    #[inline(always)]
    pub fn hermite(&self, b: &Self, c: &Self, d: &Self, t: T) -> Self {
        let factor_times2 = t * t;
        let factor1 =
            factor_times2 * (T::from(2.0).unwrap() * t - T::from(3.0).unwrap()) + T::one();
        let factor2 = factor_times2 * (t - T::from(2.0).unwrap()) + t;
        let factor3 = factor_times2 * (t - T::one());
        let factor4 = factor_times2 * (T::from(3.0).unwrap() - T::from(2.0).unwrap() * t);

        Self([
            self.0[0] * factor1 + b.0[0] * factor2 + c.0[0] * factor3 + d.0[0] * factor4,
            self.0[1] * factor1 + b.0[1] * factor2 + c.0[1] * factor3 + d.0[1] * factor4,
            self.0[2] * factor1 + b.0[2] * factor2 + c.0[2] * factor3 + d.0[2] * factor4,
        ])
    }

    #[inline(always)]
    pub fn bezier(&self, b: &Self, c: &Self, d: &Self, t: T) -> Self {
        let inverse_factor = T::one() - t;
        let inverse_factor_times_two = inverse_factor * inverse_factor;
        let factor_times2 = t * t;
        let factor1 = inverse_factor_times_two * inverse_factor;
        let factor2 = T::from(3.0).unwrap() * t * inverse_factor_times_two;
        let factor3 = T::from(3.0).unwrap() * factor_times2 * inverse_factor;
        let factor4 = factor_times2 * t;

        Self([
            self.0[0] * factor1 + b.0[0] * factor2 + c.0[0] * factor3 + d.0[0] * factor4,
            self.0[1] * factor1 + b.0[1] * factor2 + c.0[1] * factor3 + d.0[1] * factor4,
            self.0[2] * factor1 + b.0[2] * factor2 + c.0[2] * factor3 + d.0[2] * factor4,
        ])
    }

    #[inline(always)]
    pub fn transform_mat3(&self, m: &Mat3<T>) -> Self {
        let x = self.0[0];
        let y = self.0[1];
        let z = self.0[2];

        Self([
            x * m.0[0] + y * m.0[3] + z * m.0[6],
            x * m.0[1] + y * m.0[4] + z * m.0[7],
            x * m.0[2] + y * m.0[5] + z * m.0[8],
        ])
    }

    #[inline(always)]
    pub fn transform_quat(&self, q: &Quat<T>) -> Self {
        // benchmarks: https://jsperf.com/quaternion-transform-vec3-implementations-fixed
        let qx = q.0[0];
        let qy = q.0[1];
        let qz = q.0[2];
        let qw = q.0[3];
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
        let w2 = qw * T::from(2.0).unwrap();
        uvx = uvx * w2;
        uvy = uvy * w2;
        uvz = uvz * w2;
        // vec3.scale(uuv, uuv, 2);
        uuvx = uuvx * T::from(2.0).unwrap();
        uuvy = uuvy * T::from(2.0).unwrap();
        uuvz = uuvz * T::from(2.0).unwrap();
        // return vec3.add(out, a, vec3.add(out, uv, uuv));

        Self([x + uvx + uuvx, y + uvy + uuvy, z + uvz + uuvz])
    }

    #[inline(always)]
    pub fn transform_mat4(&self, m: &Mat4<T>) -> Self {
        let x = self.0[0];
        let y = self.0[1];
        let z = self.0[2];
        let mut w = m.0[3] * x + m.0[7] * y + m.0[11] * z + m.0[15];
        w = if w == T::zero() { T::one() } else { w };

        Self([
            (m.0[0] * x + m.0[4] * y + m.0[8] * z + m.0[12]) / w,
            (m.0[1] * x + m.0[5] * y + m.0[9] * z + m.0[13]) / w,
            (m.0[2] * x + m.0[6] * y + m.0[10] * z + m.0[14]) / w,
        ])
    }

    #[inline(always)]
    pub fn rotate_x(&self, b: &Self, rad: T) -> Self {
        let mut p = [T::zero(); 3];
        let mut r = [T::zero(); 3];
        //Translate point to the origin
        p[0] = self.0[0] - b.0[0];
        p[1] = self.0[1] - b.0[1];
        p[2] = self.0[2] - b.0[2];

        //perform rotation
        r[0] = p[0];
        r[1] = p[1] * rad.cos() - p[2] * rad.sin();
        r[2] = p[1] * rad.sin() + p[2] * rad.cos();

        Self([r[0] + b.0[0], r[1] + b.0[1], r[2] + b.0[2]])
    }

    #[inline(always)]
    pub fn rotate_y(&self, b: &Self, rad: T) -> Self {
        let mut p = [T::zero(); 3];
        let mut r = [T::zero(); 3];
        //Translate point to the origin
        p[0] = self.0[0] - b.0[0];
        p[1] = self.0[1] - b.0[1];
        p[2] = self.0[2] - b.0[2];

        //perform rotation
        r[0] = p[2] * rad.sin() + p[0] * rad.cos();
        r[1] = p[1];
        r[2] = p[2] * rad.cos() - p[0] * rad.sin();

        Self([r[0] + b.0[0], r[1] + b.0[1], r[2] + b.0[2]])
    }

    #[inline(always)]
    pub fn rotate_z(&self, b: &Self, rad: T) -> Self {
        let mut p = [T::zero(); 3];
        let mut r = [T::zero(); 3];
        //Translate point to the origin
        p[0] = self.0[0] - b.0[0];
        p[1] = self.0[1] - b.0[1];
        p[2] = self.0[2] - b.0[2];

        //perform rotation
        r[0] = p[0] * rad.cos() - p[1] * rad.sin();
        r[1] = p[0] * rad.sin() + p[1] * rad.cos();
        r[2] = p[2];

        Self([r[0] + b.0[0], r[1] + b.0[1], r[2] + b.0[2]])
    }

    #[inline(always)]
    pub fn angle(&self, b: &Self) -> T {
        let ax = self.0[0];
        let ay = self.0[1];
        let az = self.0[2];
        let bx = b.0[0];
        let by = b.0[1];
        let bz = b.0[2];
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
    pub fn approximate_eq(&self, b: &Self) -> bool {
        let a0 = self.0[0];
        let a1 = self.0[1];
        let a2 = self.0[2];
        let b0 = b.0[0];
        let b1 = b.0[1];
        let b2 = b.0[2];

        (a0 - b0).abs() <= epsilon::<T>() * T::one().max(a0.abs()).max(b0.abs())
            && (a1 - b1).abs() <= epsilon::<T>() * T::one().max(a1.abs()).max(b1.abs())
            && (a2 - b2).abs() <= epsilon::<T>() * T::one().max(a2.abs()).max(b2.abs())
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

/// tests only for f32
#[cfg(test)]
mod tests {
    use std::sync::OnceLock;

    use crate::{error::Error, mat3::Mat3, mat4::Mat4, quat::Quat, vec3::Vec3};

    static VEC_A_RAW: [f32; 3] = [1.0, 2.0, 3.0];
    static VEC_B_RAW: [f32; 3] = [4.0, 5.0, 6.0];

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
        assert_eq!(Vec3::<f32>::new().raw(), &[0.0, 0.0, 0.0]);
    }

    #[test]
    fn from_slice() {
        assert_eq!(Vec3::from_slice(&[3.0, 4.0, 5.0]).raw(), &[3.0, 4.0, 5.0]);
    }

    #[test]
    fn from_values() {
        assert_eq!(Vec3::from_values(3.0, 4.0, 5.0).raw(), &[3.0, 4.0, 5.0]);
    }

    #[test]
    fn ceil() {
        assert_eq!(
            Vec3::from_values(
                std::f32::consts::E,
                std::f32::consts::PI,
                std::f32::consts::SQRT_2
            )
            .ceil()
            .raw(),
            &[3.0, 4.0, 2.0]
        );
    }

    #[test]
    fn floor() -> Result<(), Error> {
        assert_eq!(
            Vec3::from_values(
                std::f32::consts::E,
                std::f32::consts::PI,
                std::f32::consts::SQRT_2
            )
            .floor()
            .raw(),
            &[2.0, 3.0, 1.0]
        );

        Ok(())
    }

    #[test]
    fn min() -> Result<(), Error> {
        let vec_a = Vec3::from_values(1.0, 3.0, 1.0);
        let vec_b = Vec3::from_values(3.0, 1.0, 3.0);
        assert_eq!(vec_a.min(&vec_b).raw(), &[1.0, 1.0, 1.0]);

        Ok(())
    }

    #[test]
    fn max() -> Result<(), Error> {
        let vec_a = Vec3::from_values(1.0, 3.0, 1.0);
        let vec_b = Vec3::from_values(3.0, 1.0, 3.0);
        assert_eq!(vec_a.max(&vec_b).raw(), &[3.0, 3.0, 3.0]);

        Ok(())
    }

    #[test]
    fn round() -> Result<(), Error> {
        let vec = Vec3::from_values(
            std::f32::consts::E,
            std::f32::consts::PI,
            std::f32::consts::SQRT_2,
        );
        assert_eq!(vec.round().raw(), &[3.0, 3.0, 1.0]);

        Ok(())
    }

    #[test]
    fn scale() {
        assert_eq!((*vec_a() * 2.0).raw(), &[2.0, 4.0, 6.0]);
    }

    #[test]
    fn scale_add() {
        assert_eq!((*vec_a() + *vec_b() * 0.5).raw(), &[3.0, 4.5, 6.0]);
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
        assert_eq!(vec_a().negate().raw(), &[-1.0, -2.0, -3.0]);
    }

    #[test]
    fn normalize() {
        assert_eq!(
            Vec3::from_values(5.0, 0.0, 0.0).normalize().raw(),
            &[1.0, 0.0, 0.0]
        );
    }

    #[test]
    fn dot() {
        assert_eq!(vec_a().dot(vec_b()), 32.0);
    }

    #[test]
    fn cross() {
        assert_eq!(vec_a().cross(vec_b()).raw(), &[-3.0, 6.0, -3.0]);
    }

    #[test]
    fn lerp() {
        assert_eq!(vec_a().lerp(vec_b(), 0.5).raw(), &[2.5, 3.5, 4.5]);
    }

    #[test]
    fn slerp() {
        let vec_a = Vec3::from_values(1.0, 0.0, 0.0);
        let vec_b = Vec3::from_values(0.0, 1.0, 0.0);
        assert_eq!(vec_a.slerp(&vec_b, 0.0).raw(), &[1.0, 0.0, 0.0]);
        assert_eq!(vec_a.slerp(&vec_b, 1.0).raw(), &[0.0, 1.0, 0.0]);
        assert_eq!(
            vec_a.slerp(&vec_b, 0.5).raw(),
            &[0.7071067811865475, 0.7071067811865475, 0.0]
        );
    }

    #[test]
    fn transform_mat4() {
        let mat = Mat4::from_values(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        );
        assert_eq!(vec_a().transform_mat4(&mat).raw(), &[1.0, 2.0, 3.0]);
    }

    #[test]
    fn transform_mat3() {
        let mat = Mat3::from_values(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        assert_eq!(vec_a().transform_mat3(&mat).raw(), &[1.0, 2.0, 3.0]);
    }

    #[test]
    fn transform_quat() {
        let quat = Quat::from_values(
            0.18257418567011074,
            0.3651483713402215,
            0.5477225570103322,
            0.730296742680443,
        );
        assert_eq!(vec_a().transform_quat(&quat).raw(), &[1.0, 2.0, 3.0]);
    }

    #[test]
    fn set() {
        let mut mat = Vec3::new();
        mat.set(3.0, 4.0, 5.0);

        assert_eq!(mat.raw(), &[3.0, 4.0, 5.0]);
    }

    #[test]
    fn set_slice() {
        let mut mat = Vec3::new();
        mat.set_slice(&[3.0, 4.0, 5.0]);

        assert_eq!(mat.raw(), &[3.0, 4.0, 5.0]);
    }

    #[test]
    fn add() {
        assert_eq!((*vec_a() + *vec_b()).raw(), &[5.0, 7.0, 9.0]);
    }

    #[test]
    fn sub() {
        assert_eq!((*vec_a() - *vec_b()).raw(), &[-3.0, -3.0, -3.0]);
    }

    #[test]
    fn mul() {
        assert_eq!((*vec_a() * *vec_b()).raw(), &[4.0, 10.0, 18.0]);
    }

    #[test]
    fn mul_scalar() {
        assert_eq!((*vec_a() * 2.0).raw(), &[2.0, 4.0, 6.0]);
    }

    #[test]
    fn mul_scalar_add() {
        assert_eq!((*vec_a() + *vec_b() * 0.5).raw(), &[3.0, 4.5, 6.0]);
    }

    #[test]
    fn div() {
        assert_eq!((*vec_a() / *vec_b()).raw(), &[0.25, 0.4, 0.5]);
    }

    #[test]
    fn div_scalar() {
        assert_eq!((*vec_a() / 2.0).raw(), &[0.5, 1.0, 1.5]);
    }

    #[test]
    fn div_scalar_add() {
        assert_eq!((*vec_a() + *vec_b() / 0.5).raw(), &[9.0, 12.0, 15.0]);
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
        assert_eq!(vec_a().angle(&vec_b()), 0.22572586);
    }

    #[test]
    fn rotate_x() {
        let vec_a = Vec3::<f32>::from_values(0.0, 1.0, 0.0);
        let vec_b = Vec3::<f32>::from_values(0.0, 0.0, 0.0);
        assert_eq!(
            vec_a.rotate_x(&vec_b, std::f32::consts::PI).raw(),
            &[0.0, -1.0, -8.742278e-8]
        );

        let vec_a = Vec3::<f32>::from_values(2.0, 7.0, 0.0);
        let vec_b = Vec3::<f32>::from_values(2.0, 5.0, 0.0);
        assert_eq!(
            vec_a.rotate_x(&vec_b, std::f32::consts::PI).raw(),
            &[2.0, 3.0, -1.7484555e-7]
        );
    }

    #[test]
    fn rotate_y() {
        let vec_a = Vec3::<f32>::from_values(1.0, 0.0, 0.0);
        let vec_b = Vec3::<f32>::from_values(0.0, 0.0, 0.0);
        assert_eq!(
            vec_a.rotate_y(&vec_b, std::f32::consts::PI).raw(),
            &[-1.0, 0.0, 8.742278e-8]
        );

        let vec_a = Vec3::<f32>::from_values(-2.0, 3.0, 10.0);
        let vec_b = Vec3::<f32>::from_values(-4.0, 3.0, 10.0);
        assert_eq!(
            vec_a.rotate_y(&vec_b, std::f32::consts::PI).raw(),
            &[-6.0, 3.0, 10.0]
        );
    }

    #[test]
    fn rotate_z() {
        let vec_a = Vec3::<f32>::from_values(0.0, 1.0, 0.0);
        let vec_b = Vec3::<f32>::from_values(0.0, 0.0, 0.0);
        assert_eq!(
            vec_a.rotate_z(&vec_b, std::f32::consts::PI).raw(),
            &[8.742278e-8, -1.0, 0.0]
        );

        let vec_a = Vec3::<f32>::from_values(0.0, 6.0, -5.0);
        let vec_b = Vec3::<f32>::from_values(0.0, 0.0, -5.0);
        assert_eq!(
            vec_a.rotate_z(&vec_b, std::f32::consts::PI).raw(),
            &[5.2453663e-7, -6.0, -5.0]
        );
    }
}
