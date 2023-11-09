use std::{
    fmt::Display,
    ops::{Add, Div, Mul, Sub},
};

use num_traits::Float;

use crate::{epsilon, mat4::Mat4, quat::Quat};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Vec4<T = f32>(pub [T; 4]);

impl<T: Float> Vec4<T> {
    #[inline(always)]
    pub fn new() -> Vec4<T> {
        Self([T::zero(); 4])
    }

    #[inline(always)]
    pub fn from_values(x: T, y: T, z: T, w: T) -> Vec4<T> {
        Self([x, y, z, w])
    }

    #[inline(always)]
    pub fn from_slice([x, y, z, w]: &[T; 4]) -> Self {
        Self([*x, *y, *z, *w])
    }
}

impl<T: Float> Vec4<T> {
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
        let mut out = Self::new();
        out.0[0] = self.0[0].ceil();
        out.0[1] = self.0[1].ceil();
        out.0[2] = self.0[2].ceil();
        out.0[3] = self.0[3].ceil();
        out
    }

    #[inline(always)]
    pub fn floor(&self) -> Self {
        let mut out = Self::new();
        out.0[0] = self.0[0].floor();
        out.0[1] = self.0[1].floor();
        out.0[2] = self.0[2].floor();
        out.0[3] = self.0[3].floor();
        out
    }

    #[inline(always)]
    pub fn min(&self, b: &Vec4<T>) -> Self {
        let mut out = Self::new();
        out.0[0] = self.0[0].min(b.0[0]);
        out.0[1] = self.0[1].min(b.0[1]);
        out.0[2] = self.0[2].min(b.0[2]);
        out.0[3] = self.0[3].min(b.0[3]);
        out
    }

    #[inline(always)]
    pub fn max(&self, b: &Vec4<T>) -> Self {
        let mut out = Self::new();
        out.0[0] = self.0[0].max(b.0[0]);
        out.0[1] = self.0[1].max(b.0[1]);
        out.0[2] = self.0[2].max(b.0[2]);
        out.0[3] = self.0[3].max(b.0[3]);
        out
    }

    #[inline(always)]
    pub fn round(&self) -> Self {
        let mut out = Self::new();
        out.0[0] = self.0[0].round();
        out.0[1] = self.0[1].round();
        out.0[2] = self.0[2].round();
        out.0[3] = self.0[3].round();
        out
    }

    #[inline(always)]
    pub fn scale(&self, scale: T) -> Self {
        self.mul(scale)
    }

    #[inline(always)]
    pub fn squared_distance(&self, b: &Vec4<T>) -> T {
        let x = b.0[0] - self.0[0];
        let y = b.0[1] - self.0[1];
        let z = b.0[2] - self.0[2];
        let w = b.0[3] - self.0[3];
        x * x + y * y + z * z + w * w
    }

    #[inline(always)]
    pub fn distance(&self, b: &Vec4<T>) -> T {
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
        let mut out = Self::new();
        out.0[0] = -self.0[0];
        out.0[1] = -self.0[1];
        out.0[2] = -self.0[2];
        out.0[3] = -self.0[3];
        out
    }

    #[inline(always)]
    pub fn inverse(&self) -> Self {
        let mut out = Self::new();
        out.0[0] = T::one() / self.0[0];
        out.0[1] = T::one() / self.0[1];
        out.0[2] = T::one() / self.0[2];
        out.0[3] = T::one() / self.0[3];
        out
    }

    #[inline(always)]
    pub fn normalize(&self) -> Self {
        let mut len = self.squared_length();
        if len > T::zero() {
            len = T::one() / len.sqrt();
        }

        Self::from_values(
            self.0[0] * len,
            self.0[1] * len,
            self.0[2] * len,
            self.0[3] * len,
        )
    }

    #[inline(always)]
    pub fn dot(&self, b: &Vec4<T>) -> T {
        self.0[0] * b.0[0] + self.0[1] * b.0[1] + self.0[2] * b.0[2] + self.0[3] * b.0[3]
    }

    #[inline(always)]
    pub fn cross(&self, v: &Vec4<T>, w: &Vec4<T>) -> Self {
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

        Self::from_values(
            h * f - i * e + j * d,
            -(g * f) + i * c - j * b,
            g * e - h * c + j * a,
            -(g * d) + h * b - i * a,
        )
    }

    #[inline(always)]
    pub fn lerp(&self, b: &Vec4<T>, t: T) -> Self {
        let ax = self.0[0];
        let ay = self.0[1];
        let az = self.0[2];
        let aw = self.0[3];

        Self::from_values(
            ax + t * (b.0[0] - ax),
            ay + t * (b.0[1] - ay),
            az + t * (b.0[2] - az),
            aw + t * (b.0[3] - aw),
        )
    }

    #[inline(always)]
    pub fn transform_mat4(&self, m: &Mat4<T>) -> Self {
        let x = self.0[0];
        let y = self.0[1];
        let z = self.0[2];
        let w = self.0[3];

        Self::from_values(
            m.0[0] * x + m.0[4] * y + m.0[8] * z + m.0[12] * w,
            m.0[1] * x + m.0[5] * y + m.0[9] * z + m.0[13] * w,
            m.0[2] * x + m.0[6] * y + m.0[10] * z + m.0[14] * w,
            m.0[3] * x + m.0[7] * y + m.0[11] * z + m.0[15] * w,
        )
    }

    #[inline(always)]
    pub fn transform_quat(&self, q: &Quat<T>) -> Self {
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

        Self::from_values(
            ix * qw + iw * -qx + iy * -qz - iz * -qy,
            iy * qw + iw * -qy + iz * -qx - ix * -qz,
            iz * qw + iw * -qz + ix * -qy - iy * -qx,
            self.0[3],
        )
    }

    #[inline(always)]
    pub fn approximate_eq(&self, b: &Vec4<T>) -> bool {
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

impl<T: Float> Add<Vec4<T>> for Vec4<T> {
    type Output = Vec4<T>;

    #[inline(always)]
    fn add(self, b: Vec4<T>) -> Vec4<T> {
        let mut out = Vec4::<T>::new();
        out.0[0] = self.0[0] + b.0[0];
        out.0[1] = self.0[1] + b.0[1];
        out.0[2] = self.0[2] + b.0[2];
        out.0[3] = self.0[3] + b.0[3];
        out
    }
}

impl<T: Float> Sub<Vec4<T>> for Vec4<T> {
    type Output = Vec4<T>;

    #[inline(always)]
    fn sub(self, b: Vec4<T>) -> Vec4<T> {
        let mut out = Vec4::<T>::new();
        out.0[0] = self.0[0] - b.0[0];
        out.0[1] = self.0[1] - b.0[1];
        out.0[2] = self.0[2] - b.0[2];
        out.0[3] = self.0[3] - b.0[3];
        out
    }
}

impl<T: Float> Mul<Vec4<T>> for Vec4<T> {
    type Output = Vec4<T>;

    #[inline(always)]
    fn mul(self, b: Vec4<T>) -> Vec4<T> {
        let mut out = Vec4::<T>::new();
        out.0[0] = self.0[0] * b.0[0];
        out.0[1] = self.0[1] * b.0[1];
        out.0[2] = self.0[2] * b.0[2];
        out.0[3] = self.0[3] * b.0[3];
        out
    }
}

impl<T: Float> Mul<T> for Vec4<T> {
    type Output = Vec4<T>;

    #[inline(always)]
    fn mul(self, b: T) -> Vec4<T> {
        let mut out = Vec4::<T>::new();
        out.0[0] = self.0[0] * b;
        out.0[1] = self.0[1] * b;
        out.0[2] = self.0[2] * b;
        out.0[3] = self.0[3] * b;
        out
    }
}

impl<T: Float> Div<Vec4<T>> for Vec4<T> {
    type Output = Vec4<T>;

    #[inline(always)]
    fn div(self, b: Vec4<T>) -> Vec4<T> {
        let mut out = Vec4::<T>::new();
        out.0[0] = self.0[0] / b.0[0];
        out.0[1] = self.0[1] / b.0[1];
        out.0[2] = self.0[2] / b.0[2];
        out.0[3] = self.0[3] / b.0[3];
        out
    }
}

impl<T: Float> Div<T> for Vec4<T> {
    type Output = Vec4<T>;

    #[inline(always)]
    fn div(self, b: T) -> Vec4<T> {
        let mut out = Vec4::<T>::new();
        out.0[0] = self.0[0] / b;
        out.0[1] = self.0[1] / b;
        out.0[2] = self.0[2] / b;
        out.0[3] = self.0[3] / b;
        out
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

        Self::from_values(scale * v1, scale * v2, scale * v3 * d, scale * v4 * d)
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

        Self::from_values(scale * v1, scale * v2, scale * v3 * d, scale * v4 * d)
    }
}

// #[cfg(test)]
// #[rustfmt::skip]
// mod tests {
//     macro_rules! float_test {
//         (T:tt, epsilon::<T>():expr, $pi:expr, $e:expr, $sqrt2:expr) => {
//             use std::sync::OnceLock;

//             use crate::vec4::Vec4;

//             static VEC_A_RAW: [T; 4] = [T::one(), 2.0, 3.0, 4.0];
//             static VEC_B_RAW: [T; 4] = [5.0, 6.0, 7.0, 8.0];

//             static VEC_A: OnceLock<Vec4<T>> = OnceLock::new();
//             static VEC_B: OnceLock<Vec4<T>> = OnceLock::new();

//             fn vec_a() -> &'static Vec4<T> {
//                 VEC_A.get_or_init(|| {
//                     Vec4::<T>::from_slice(&VEC_A_RAW)
//                 })
//             }

//             fn vec_b() -> &'static Vec4<T> {
//                 VEC_B.get_or_init(|| {
//                     Vec4::<T>::from_slice(&VEC_B_RAW)
//                 })
//             }

//             #[test]
//             fn new() {
//                 assert_eq!(
//                     Vec4::<T>::new().raw(),
//                     &[T::zero(), T::zero(), T::zero(), T::zero()]
//                 );
//             }

//             #[test]
//             fn from_slice() {
//                 assert_eq!(
//                     Vec4::<T>::from_slice(&[3.0, 4.0, 5.0, 6.0]).raw(),
//                     &[3.0, 4.0, 5.0, 6.0]
//                 );
//             }

//             #[test]
//             fn from_values() {
//                 assert_eq!(
//                     Vec4::<T>::from_values(3.0, 4.0, 5.0, 6.0).raw(),
//                     &[3.0, 4.0, 5.0, 6.0]
//                 );
//             }

//             #[test]
//             fn ceil() {
//                 assert_eq!(
//                     Vec4::<T>::from_values($e, $pi, $sqrt2, (0.5 as T).sqrt()).ceil().raw(),
//                     &[3.0, 4.0, 2.0, T::one()]
//                 );
//             }

//             #[test]
//             fn floor() {
//                 assert_eq!(
//                     Vec4::<T>::from_values($e, $pi, $sqrt2, (0.5 as T).sqrt()).floor().raw(),
//                     &[2.0, 3.0, T::one(), T::zero()]
//                 );
//             }

//             #[test]
//             fn min() {
//                 let vec_a = Vec4::<T>::from_values(T::one(), 3.0, T::one(), 3.0);
//                 let vec_b = Vec4::<T>::from_values(3.0, T::one(), 3.0, T::one());
//                 assert_eq!(
//                     vec_a.min(&vec_b).raw(),
//                     &[T::one(), T::one(), T::one(), T::one()]
//                 );
//             }

//             #[test]
//             fn max() {
//                 let vec_a = Vec4::<T>::from_values(T::one(), 3.0, T::one(), 3.0);
//                 let vec_b = Vec4::<T>::from_values(3.0, T::one(), 3.0, T::one());
//                 assert_eq!(
//                     vec_a.max(&vec_b).raw(),
//                     &[3.0, 3.0, 3.0, 3.0]
//                 );
//             }

//             #[test]
//             fn round() {
//                 let vec = Vec4::<T>::from_values($e, $pi, $sqrt2, (0.5 as T).sqrt());
//                 assert_eq!(
//                     vec.round().raw(),
//                     &[3.0, 3.0, T::one(), T::one()]
//                 );
//             }

//             #[test]
//             fn scale() {
//                 assert_eq!(
//                     (*vec_a() * 2.0).raw(),
//                     &[2.0, 4.0, 6.0, 8.0]
//                 );
//             }

//             #[test]
//             fn scale_add() {
//                 assert_eq!(
//                     (*vec_a() + *vec_b() * 0.5).raw(),
//                     &[3.5, 5.0, 6.5, 8.0]
//                 );
//             }

//             #[test]
//             fn squared_distance() {
//                 assert_eq!(
//                     vec_a().squared_distance(vec_b()),
//                     64.0
//                 );
//             }

//             #[test]
//             fn distance() {
//                 assert_eq!(
//                     vec_a().distance(vec_b()),
//                     8.0
//                 );
//             }

//             #[test]
//             fn squared_length() {
//                 assert_eq!(
//                     vec_a().squared_length(),
//                     3T::zero()
//                 );
//             }

//             #[test]
//             fn length() {
//                 assert_eq!(
//                     vec_a().length(),
//                     5.477225575051661
//                 );
//             }

//             #[test]
//             fn negate() {
//                 assert_eq!(
//                     vec_a().negate().raw(),
//                     &[-T::one() ,-2.0, -3.0, -4.0]
//                 );
//             }

//             #[test]
//             fn normalize() {
//                 assert_eq!(
//                     Vec4::<T>::from_values(5.0, T::zero(), T::zero(), T::zero()).normalize().raw(),
//                     &[T::one() ,T::zero(), T::zero(), T::zero()]
//                 );
//             }

//             #[test]
//             fn dot() {
//                 assert_eq!(
//                     vec_a().dot(vec_b()),
//                     7T::zero()
//                 );
//             }

//             #[test]
//             fn cross() {
//                 let vec_a = Vec4::<T>::from_values(T::one(), T::zero(), T::zero(), T::zero());
//                 let vec_b = Vec4::<T>::from_values(T::zero(), T::one(), T::zero(), T::zero());
//                 let vec_c = Vec4::<T>::from_values(T::zero(), T::one(), T::one(), T::zero());
//                 assert_eq!(
//                     vec_a.cross(&vec_b, &vec_c).raw(),
//                     &[T::zero(), T::zero(), T::zero(), -T::one()]
//                 );
//             }

//             #[test]
//             fn lerp() {
//                 assert_eq!(
//                     vec_a().lerp(vec_b(), 0.5).raw(),
//                     &[3.0, 4.0, 5.0, 6.0]
//                 );
//             }

//             #[test]
//             fn set() {
//                 let mut mat = Vec4::<T>::new();
//                 mat.set(3.0, 4.0, 5.0, 6.0);

//                 assert_eq!(
//                     mat.raw(),
//                     &[3.0, 4.0, 5.0, 6.0]
//                 );
//             }

//             #[test]
//             fn set_slice() {
//                 let mut mat = Vec4::<T>::new();
//                 mat.set_slice(&[3.0, 4.0, 5.0, 6.0]);

//                 assert_eq!(
//                     mat.raw(),
//                     &[3.0, 4.0, 5.0, 6.0]
//                 );
//             }

//             #[test]
//             fn add() {
//                 assert_eq!(
//                     (*vec_a() + *vec_b()).raw(),
//                     &[6.0, 8.0, 1T::zero(), 12.0]
//                 );
//             }

//             #[test]
//             fn sub() {
//                 assert_eq!(
//                     (*vec_a() - *vec_b()).raw(),
//                     &[-4.0, -4.0, -4.0, -4.0]
//                 );
//             }

//             #[test]
//             fn mul() {
//                 assert_eq!(
//                     (*vec_a() * *vec_b()).raw(),
//                     &[5.0, 12.0, 2T::one(), 32.0]
//                 );
//             }

//             #[test]
//             fn mul_scalar() {
//                 assert_eq!(
//                     (*vec_a() * 2.0).raw(),
//                     &[2.0, 4.0, 6.0, 8.0]
//                 );
//             }

//             #[test]
//             fn mul_scalar_add() {
//                 assert_eq!(
//                     (*vec_a() + *vec_b() * 0.5).raw(),
//                     &[3.5, 5.0, 6.5, 8.0]
//                 );
//             }

//             #[test]
//             fn div() {
//                 assert_eq!(
//                     (*vec_a() / *vec_b()).raw(),
//                     &[0.2, 0.3333333333333333, 0.42857142857142855, 0.5]
//                 );
//             }

//             #[test]
//             fn div_scalar() {
//                 assert_eq!(
//                     (*vec_a() / 2.0).raw(),
//                     &[0.5, T::one(), 1.5, 2.0]
//                 );
//             }

//             #[test]
//             fn div_scalar_add() {
//                 assert_eq!(
//                     (*vec_a() + *vec_b() / 0.5).raw(),
//                     &[1T::one(), 14.0, 17.0, 2T::zero()]
//                 );
//             }

//             #[test]
//             fn approximate_eq() {
//                 let vec_a = Vec4::<T>::from_values(T::zero(),  T::one(), 2.0, 3.0);
//                 let vec_b = Vec4::<T>::from_values(T::zero(),  T::one(), 2.0, 3.0);
//                 let vec_c = Vec4::<T>::from_values(T::one(),  2.0, 3.0, 4.0);
//                 let vec_d = Vec4::<T>::from_values(1e-16,  T::one(), 2.0, 3.0);

//                 assert_eq!(
//                     true,
//                     vec_a.approximate_eq(&vec_b)
//                 );
//                 assert_eq!(
//                     false,
//                     vec_a.approximate_eq(&vec_c)
//                 );
//                 assert_eq!(
//                     true,
//                     vec_a.approximate_eq(&vec_d)
//                 );
//             }

//             #[test]
//             fn display() {
//                 let out = vec_a().to_string();
//                 assert_eq!(
//                     out,
//                     "vec4(1, 2, 3, 4)"
//                 );
//             }
//         };
//     }

//     mod f32 {
//         float_test!(
//             f32,
//             crate::EPSILON_F32,
//             std::f32::consts::PI,
//             std::f32::consts::E,
//             std::f32::consts::SQRT_2
//         );
//     }

//     mod f64 {
//         float_test!(
//             f64,
//             crate::EPSILON_F64,
//             std::f64::consts::PI,
//             std::f64::consts::E,
//             std::f64::consts::SQRT_2
//         );
//     }
// }
