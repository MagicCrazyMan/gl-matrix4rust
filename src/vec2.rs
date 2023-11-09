use std::{
    fmt::Display,
    ops::{Add, Div, Mul, Sub},
};

use num_traits::Float;

use crate::{epsilon, mat2::Mat2, mat2d::Mat2d, mat3::Mat3, mat4::Mat4, vec3::Vec3};
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Vec2<T = f32>(pub [T; 2]);

impl<T: Float> Vec2<T> {
    #[inline(always)]
    pub fn new() -> Self {
        Self([T::zero(); 2])
    }

    #[inline(always)]
    pub fn from_values(x: T, y: T) -> Self {
        Self([x, y])
    }

    #[inline(always)]
    pub fn from_slice(slice: &[T; 2]) -> Self {
        Self(slice.clone())
    }
}

impl<T: Float> Vec2<T> {
    #[inline(always)]
    pub fn raw(&self) -> &[T; 2] {
        &self.0
    }

    #[inline(always)]
    pub fn set(&mut self, x: T, y: T) -> &mut Self {
        self.0[0] = x;
        self.0[1] = y;
        self
    }

    #[inline(always)]
    pub fn set_slice(&mut self, [x, y]: &[T; 2]) -> &mut Self {
        self.0[0] = *x;
        self.0[1] = *y;
        self
    }

    #[inline(always)]
    pub fn set_zero(&mut self) -> &mut Self {
        self.0[0] = T::zero();
        self.0[1] = T::zero();
        self
    }

    #[inline(always)]
    pub fn ceil(&self) -> Self {
        Self([self.0[0].ceil(), self.0[1].ceil()])
    }

    #[inline(always)]
    pub fn floor(&self) -> Self {
        Self([self.0[0].floor(), self.0[1].floor()])
    }

    #[inline(always)]
    pub fn min(&self, b: &Self) -> Self {
        Self([self.0[0].min(b.0[0]), self.0[1].min(b.0[0])])
    }

    #[inline(always)]
    pub fn max(&self, b: &Self) -> Self {
        Self([self.0[0].max(b.0[0]), self.0[1].max(b.0[1])])
    }

    #[inline(always)]
    pub fn round(&self) -> Self {
        Self([self.0[0].round(), self.0[1].round()])
    }

    #[inline(always)]
    pub fn scale(&self, scale: T) -> Self {
        self.mul(scale)
    }

    #[inline(always)]
    pub fn squared_distance(&self, b: &Self) -> T {
        let x = b.0[0] - self.0[0];
        let y = b.0[1] - self.0[1];
        x * x + y * y
    }

    #[inline(always)]
    pub fn distance(&self, b: &Self) -> T {
        self.squared_distance(b).sqrt()
    }

    #[inline(always)]
    pub fn squared_length(&self) -> T {
        let x = self.0[0];
        let y = self.0[1];
        x * x + y * y
    }

    #[inline(always)]
    pub fn length(&self) -> T {
        self.squared_length().sqrt()
    }

    #[inline(always)]
    pub fn negate(&self) -> Self {
        Self([-self.0[0], -self.0[1]])
    }

    #[inline(always)]
    pub fn inverse(&self) -> Self {
        Self([T::one() / self.0[0], T::one() / self.0[1]])
    }

    #[inline(always)]
    pub fn normalize(&self) -> Self {
        let mut len = self.squared_length();
        if len > T::zero() {
            len = T::one() / len.sqrt();
        }

        Self([self.0[0] * len, self.0[1] * len])
    }

    #[inline(always)]
    pub fn dot(&self, b: &Self) -> T {
        self.0[0] * b.0[0] + self.0[1] * b.0[1]
    }

    #[inline(always)]
    pub fn cross(&self, b: &Self) -> Vec3<T> {
        Vec3::<T>::from_values(
            T::zero(),
            T::zero(),
            self.0[0] * b.0[1] - self.0[1] * b.0[0],
        )
    }

    #[inline(always)]
    pub fn lerp(&self, b: &Self, t: T) -> Self {
        let ax = self.0[0];
        let ay = self.0[1];
        Self([ax + t * (b.0[0] - ax), ay + t * (b.0[1] - ay)])
    }

    #[inline(always)]
    pub fn transform_mat2(&self, m: &Mat2<T>) -> Self {
        let x = self.0[0];
        let y = self.0[1];
        Self([m.0[0] * x + m.0[2] * y, m.0[1] * x + m.0[3] * y])
    }

    #[inline(always)]
    pub fn transform_mat2d(&self, m: &Mat2d<T>) -> Self {
        let x = self.0[0];
        let y = self.0[1];
        Self([
            m.0[0] * x + m.0[2] * y + m.0[4],
            m.0[1] * x + m.0[3] * y + m.0[5],
        ])
    }

    #[inline(always)]
    pub fn transform_mat3(&self, m: &Mat3<T>) -> Self {
        let x = self.0[0];
        let y = self.0[1];
        Self([
            m.0[0] * x + m.0[3] * y + m.0[6],
            m.0[1] * x + m.0[4] * y + m.0[7],
        ])
    }

    #[inline(always)]
    pub fn transform_mat4(&self, m: &Mat4<T>) -> Self {
        let x = self.0[0];
        let y = self.0[1];
        Self([
            m.0[0] * x + m.0[4] * y + m.0[12],
            m.0[1] * x + m.0[5] * y + m.0[13],
        ])
    }

    #[inline(always)]
    pub fn rotate(&self, b: &Self, rad: T) -> Self {
        //Translate point to the origin
        let p0 = self.0[0] - b.0[0];
        let p1 = self.0[1] - b.0[1];
        let sin_c = rad.sin();
        let cos_c = rad.cos();

        //perform rotation and translate to correct position
        Self([
            p0 * cos_c - p1 * sin_c + b.0[0],
            p0 * sin_c + p1 * cos_c + b.0[1],
        ])
    }

    #[inline(always)]
    pub fn angle(&self, b: &Self) -> T {
        let x1 = self.0[0];
        let y1 = self.0[1];
        let x2 = b.0[0];
        let y2 = b.0[1];
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
    pub fn approximate_eq(&self, b: &Self) -> bool {
        let a0 = self.0[0];
        let a1 = self.0[1];
        let b0 = b.0[0];
        let b1 = b.0[1];

        (a0 - b0).abs() <= epsilon::<T>() * T::one().max(a0.abs()).max(b0.abs())
            && (a1 - b1).abs() <= epsilon::<T>() * T::one().max(a1.abs()).max(b1.abs())
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

// #[cfg(test)]
// #[rustfmt::skip]
// mod tests {
//     macro_rules! float_test {
//         (T:tt, epsilon():expr, $pi:expr, $e:expr) => {
//             use std::sync::OnceLock;

//             use crate::error::Error;
//             use crate::vec2::Vec2;
//             use crate::mat2::Mat2;
//             use crate::mat2d::Mat2d;

//             static VEC_A_RAW: [T; 2] = [T::one(), 2.0];
//             static VEC_B_RAW: [T; 2] = [3.0, 4.0];

//             static VEC_A: OnceLock<Vec2<T>> = OnceLock::new();
//             static VEC_B: OnceLock<Vec2<T>> = OnceLock::new();

//             fn vec_a() -> &'static Vec2<T> {
//                 VEC_A.get_or_init(|| {
//                     Vec2::<T>::from_slice(&VEC_A_RAW)
//                 })
//             }

//             fn vec_b() -> &'static Vec2<T> {
//                 VEC_B.get_or_init(|| {
//                     Vec2::<T>::from_slice(&VEC_B_RAW)
//                 })
//             }

//             #[test]
//             fn new() {
//                 assert_eq!(
//                     Vec2::<T>::new().raw(),
//                     &[T::zero(), T::zero()]
//                 );
//             }

//             #[test]
//             fn from_slice() {
//                 assert_eq!(
//                     Vec2::<T>::from_slice(&[3.0, 4.0]).raw(),
//                     &[3.0, 4.0]
//                 );
//             }

//             #[test]
//             fn from_values() {
//                 assert_eq!(
//                     Vec2::<T>::from_values(3.0, 4.0).raw(),
//                     &[3.0, 4.0]
//                 );
//             }

//             #[test]
//             fn ceil() {
//                 assert_eq!(
//                     Vec2::<T>::from_values($e, $pi).ceil().raw(),
//                     &[3.0, 4.0]
//                 );
//             }

//             #[test]
//             fn floor() -> Result<(), Error> {
//                 assert_eq!(
//                     Vec2::<T>::from_values($e, $pi).floor().raw(),
//                     &[2.0, 3.0]
//                 );

//                 Ok(())
//             }

//             #[test]
//             fn min() -> Result<(), Error> {
//                 let vec_a = Vec2::<T>::from_values(T::one(), 4.0);
//                 let vec_b = Vec2::<T>::from_values(3.0, 2.0);
//                 assert_eq!(
//                     vec_a.min(&vec_b).raw(),
//                     &[T::one(), 2.0]
//                 );

//                 Ok(())
//             }

//             #[test]
//             fn max() -> Result<(), Error> {
//                 let vec_a = Vec2::<T>::from_values(T::one(), 4.0);
//                 let vec_b = Vec2::<T>::from_values(3.0, 2.0);
//                 assert_eq!(
//                     vec_a.max(&vec_b).raw(),
//                     &[3.0, 4.0]
//                 );

//                 Ok(())
//             }

//             #[test]
//             fn round() -> Result<(), Error> {
//                 assert_eq!(
//                     Vec2::<T>::from_values($e, $pi).round().raw(),
//                     &[3.0, 3.0]
//                 );

//                 Ok(())
//             }

//             #[test]
//             fn scale() {
//                 assert_eq!(
//                     (*vec_a() * 2.0).raw(),
//                     &[2.0, 4.0]
//                 );
//             }

//             #[test]
//             fn scale_add() {
//                 assert_eq!(
//                     (*vec_a() + *vec_b() * 2.0).raw(),
//                     &[7.0, 1T::zero()]
//                 );
//             }

//             #[test]
//             fn squared_distance() {
//                 assert_eq!(
//                     vec_a().squared_distance(vec_b()),
//                     8.0
//                 );
//             }

//             #[test]
//             fn distance() {
//                 assert_eq!(
//                     vec_a().distance(vec_b()),
//                     2.8284271247461903
//                 );
//             }

//             #[test]
//             fn squared_length() {
//                 assert_eq!(
//                     vec_a().squared_length(),
//                     5.0
//                 );
//             }

//             #[test]
//             fn length() {
//                 assert_eq!(
//                     vec_a().length(),
//                     2.23606797749979
//                 );
//             }

//             #[test]
//             fn negate() {
//                 assert_eq!(
//                     vec_a().negate().raw(),
//                     &[-T::one() ,-2.0]
//                 );
//             }

//             #[test]
//             fn normalize() {
//                 assert_eq!(
//                     Vec2::<T>::from_values(5.0, T::zero()).normalize().raw(),
//                     &[T::one() ,T::zero()]
//                 );
//             }

//             #[test]
//             fn dot() {
//                 assert_eq!(
//                     vec_a().dot(vec_b()),
//                     1T::one()
//                 );
//             }

//             #[test]
//             fn cross() {
//                 assert_eq!(
//                     vec_a().cross(vec_b()).raw(),
//                     &[T::zero(), T::zero(), -2.0]
//                 );
//             }

//             #[test]
//             fn lerp() {
//                 assert_eq!(
//                     vec_a().lerp(vec_b(), 0.5).raw(),
//                     &[2.0, 3.0]
//                 );
//             }

//             #[test]
//             fn angle() {
//                 let vec_a = Vec2::<T>::from_values(T::one(), T::zero());
//                 let vec_b = Vec2::<T>::from_values(T::one(), 2.0);
//                 assert_eq!(
//                     vec_a.angle(&vec_b),
//                     1.1071487177940904
//                 );
//             }

//             #[test]
//             fn transform_mat2() {
//                 let mat = Mat2::<T>::from_values(T::one(), 2.0, 3.0, 4.0);
//                 assert_eq!(
//                     vec_a().transform_mat2(&mat).raw(),
//                     &[7.0, 1T::zero()]
//                 );
//             }

//             #[test]
//             fn transform_mat2d() {
//                 let mat = Mat2d::<T>::from_values(T::one(), 2.0, 3.0, 4.0, 5.0, 6.0);
//                 assert_eq!(
//                     vec_a().transform_mat2d(&mat).raw(),
//                     &[12.0, 16.0]
//                 );
//             }

//             #[test]
//             fn set() {
//                 let mut mat = Vec2::<T>::new();
//                 mat.set(3.0, 4.0);

//                 assert_eq!(
//                     mat.raw(),
//                     &[3.0, 4.0]
//                 );
//             }

//             #[test]
//             fn set_slice() {
//                 let mut mat = Vec2::<T>::new();
//                 mat.set_slice(&[3.0, 4.0]);

//                 assert_eq!(
//                     mat.raw(),
//                     &[3.0, 4.0]
//                 );
//             }

//             #[test]
//             fn add() {
//                 assert_eq!(
//                     (*vec_a() + *vec_b()).raw(),
//                     &[4.0,  6.0]
//                 );
//             }

//             #[test]
//             fn sub() {
//                 assert_eq!(
//                     (*vec_a() - *vec_b()).raw(),
//                     &[-2.0, -2.0]
//                 );
//             }

//             #[test]
//             fn mul() {
//                 assert_eq!(
//                     (*vec_a() * *vec_b()).raw(),
//                     &[3.0, 8.0]
//                 );
//             }

//             #[test]
//             fn mul_scalar() {
//                 assert_eq!(
//                     (*vec_a() * 2.0).raw(),
//                     &[2.0, 4.0]
//                 );
//             }

//             #[test]
//             fn mul_scalar_add() {
//                 assert_eq!(
//                     (*vec_a() + *vec_b() * 0.5).raw(),
//                     &[2.5, 4.0]
//                 );
//             }

//             #[test]
//             fn div() {
//                 assert_eq!(
//                     (*vec_a() / *vec_b()).raw(),
//                     &[0.3333333333333333, 0.5]
//                 );
//             }

//             #[test]
//             fn div_scalar() {
//                 assert_eq!(
//                     (*vec_a() / 2.0).raw(),
//                     &[0.5, T::one()]
//                 );
//             }

//             #[test]
//             fn div_scalar_add() {
//                 assert_eq!(
//                     (*vec_a() + *vec_b() / 0.5).raw(),
//                     &[7.0, 1T::zero()]
//                 );
//             }

//             #[test]
//             fn approximate_eq() {
//                 let vec_a = Vec2::<T>::from_values(T::zero(),  T::one());
//                 let vec_b = Vec2::<T>::from_values(T::zero(),  T::one());
//                 let vec_c = Vec2::<T>::from_values(T::one(),  2.0);
//                 let vec_d = Vec2::<T>::from_values(1e-16,  T::one());

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
//                     "vec2(1, 2)"
//                 );
//             }
//         };
//     }

//     mod f32 {
//         float_test!(f32, crate::EPSILON_F32, std::f32::consts::PI, std::f32::consts::E);

//         #[test]
//         fn rotate() {
//             let vec_a = Vec2::<f32>::from_values(T::zero(), T::one());
//             let vec_b = Vec2::<f32>::from_values(T::zero(), T::zero());
//             assert_eq!(
//                 vec_a.rotate(&vec_b, std::f32::consts::PI).raw(),
//                 &[8.742278e-8, -T::one()]
//             );

//             let vec_a = Vec2::<f32>::from_values(6.0, -5.0);
//             let vec_b = Vec2::<f32>::from_values(T::zero(), -5.0);
//             assert_eq!(
//                 vec_a.rotate(&vec_b, std::f32::consts::PI).raw(),
//                 &[-6.0, -5.0000005]
//             );
//         }
//     }

//     mod f64 {
//         float_test!(f64, crate::EPSILON_F64, std::f64::consts::PI, std::f64::consts::E);

//         #[test]
//         fn rotate() {
//             let vec_a = Vec2::<f64>::from_values(T::zero(), T::one());
//             let vec_b = Vec2::<f64>::from_values(T::zero(), T::zero());
//             assert_eq!(
//                 vec_a.rotate(&vec_b, std::f64::consts::PI).raw(),
//                 &[-1.2246467991473532e-16, -T::one()]
//             );

//             let vec_a = Vec2::<f64>::from_values(6.0, -5.0);
//             let vec_b = Vec2::<f64>::from_values(T::zero(), -5.0);
//             assert_eq!(
//                 vec_a.rotate(&vec_b, std::f64::consts::PI).raw(),
//                 &[-6.0, -4.999999999999999]
//             );
//         }
//     }
// }
