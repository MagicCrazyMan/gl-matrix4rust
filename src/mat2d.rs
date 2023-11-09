use std::{
    fmt::Display,
    ops::{Add, Mul, Sub},
};

use num_traits::Float;

use crate::{epsilon, error::Error, vec2::Vec2};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mat2d<T = f32>(pub [T; 6]);

impl<T: Float> Mat2d<T> {
    #[inline(always)]
    pub fn new() -> Self {
        Self([T::zero(); 6])
    }

    #[inline(always)]
    pub fn new_identity() -> Self {
        let mut out = Self::new();
        out.0[0] = T::one();
        out.0[3] = T::one();
        out
    }

    #[inline(always)]
    pub fn from_values(a: T, b: T, c: T, d: T, tx: T, ty: T) -> Self {
        let mut out = Self::new();
        out.0[0] = a;
        out.0[1] = b;
        out.0[2] = c;
        out.0[3] = d;
        out.0[4] = tx;
        out.0[5] = ty;
        out
    }

    #[inline(always)]
    pub fn from_slice(slice: &[T; 6]) -> Self {
        let mut out = Self::new();
        out.0[0] = slice[0];
        out.0[1] = slice[1];
        out.0[2] = slice[2];
        out.0[3] = slice[3];
        out.0[4] = slice[4];
        out.0[5] = slice[5];
        out
    }

    #[inline(always)]
    pub fn from_scaling(v: &Vec2<T>) -> Self {
        let mut out = Self::new();
        out.0[0] = v.0[0];
        out.0[1] = T::zero();
        out.0[2] = T::zero();
        out.0[3] = v.0[1];
        out.0[4] = T::zero();
        out.0[5] = T::zero();
        out
    }

    #[inline(always)]
    pub fn from_rotation(rad: T) -> Self {
        let mut out = Self::new();

        let s = rad.sin();
        let c = rad.cos();
        out.0[0] = c;
        out.0[1] = s;
        out.0[2] = -s;
        out.0[3] = c;
        out.0[4] = T::zero();
        out.0[5] = T::zero();

        out
    }

    #[inline(always)]
    pub fn from_translation(v: &Vec2<T>) -> Self {
        let mut out = Self::new();

        out.0[0] = T::one();
        out.0[1] = T::zero();
        out.0[2] = T::zero();
        out.0[3] = T::one();
        out.0[4] = v.0[0];
        out.0[5] = v.0[1];

        out
    }
}

impl<T: Float> Mat2d<T> {
    #[inline(always)]
    pub fn raw(&self) -> &[T; 6] {
        &self.0
    }

    #[inline(always)]
    pub fn set(&mut self, a: T, b: T, c: T, d: T, tx: T, ty: T) -> &mut Self {
        self.0[0] = a;
        self.0[1] = b;
        self.0[2] = c;
        self.0[3] = d;
        self.0[4] = tx;
        self.0[5] = ty;
        self
    }

    #[inline(always)]
    pub fn set_slice(&mut self, slice: &[T; 6]) -> &mut Self {
        self.0 = slice.clone();
        self
    }

    #[inline(always)]
    pub fn set_zero(&mut self) -> &mut Self {
        self.0[0] = T::zero();
        self.0[1] = T::zero();
        self.0[2] = T::zero();
        self.0[3] = T::zero();
        self.0[4] = T::zero();
        self.0[5] = T::zero();
        self
    }

    #[inline(always)]
    pub fn set_identify(&mut self) -> &mut Self {
        self.0[0] = T::one();
        self.0[1] = T::zero();
        self.0[2] = T::zero();
        self.0[3] = T::one();
        self.0[4] = T::zero();
        self.0[5] = T::zero();
        self
    }

    #[inline(always)]
    pub fn invert(&self) -> Result<Self, Error> {
        let mut out = Self::new();

        let aa = self.0[0];
        let ab = self.0[1];
        let ac = self.0[2];
        let ad = self.0[3];
        let atx = self.0[4];
        let aty = self.0[5];

        // Calculate the determinant
        let mut det = aa * ad - ab * ac;

        if det == T::zero() {
            return Err(Error::ZeroDeterminant);
        }
        det = T::one() / det;

        out.0[0] = ad * det;
        out.0[1] = -ab * det;
        out.0[2] = -ac * det;
        out.0[3] = aa * det;
        out.0[4] = (ac * aty - ad * atx) * det;
        out.0[5] = (ab * atx - aa * aty) * det;

        Ok(out)
    }

    #[inline(always)]
    pub fn determinant(&self) -> T {
        self.0[0] * self.0[3] - self.0[1] * self.0[2]
    }

    #[inline(always)]
    pub fn scale(&self, v: &Vec2<T>) -> Self {
        let mut out = Self::new();

        let a0 = self.0[0];
        let a1 = self.0[1];
        let a2 = self.0[2];
        let a3 = self.0[3];
        let a4 = self.0[4];
        let a5 = self.0[5];
        let v0 = v.0[0];
        let v1 = v.0[1];
        out.0[0] = a0 * v0;
        out.0[1] = a1 * v0;
        out.0[2] = a2 * v1;
        out.0[3] = a3 * v1;
        out.0[4] = a4;
        out.0[5] = a5;

        out
    }

    #[inline(always)]
    pub fn rotate(&self, rad: T) -> Self {
        let mut out = Self::new();

        let a0 = self.0[0];
        let a1 = self.0[1];
        let a2 = self.0[2];
        let a3 = self.0[3];
        let a4 = self.0[4];
        let a5 = self.0[5];
        let s = rad.sin();
        let c = rad.cos();
        out.0[0] = a0 * c + a2 * s;
        out.0[1] = a1 * c + a3 * s;
        out.0[2] = a0 * -s + a2 * c;
        out.0[3] = a1 * -s + a3 * c;
        out.0[4] = a4;
        out.0[5] = a5;

        out
    }

    #[inline(always)]
    pub fn translate(&self, v: &Vec2<T>) -> Self {
        let mut out = Self::new();

        let a0 = self.0[0];
        let a1 = self.0[1];
        let a2 = self.0[2];
        let a3 = self.0[3];
        let a4 = self.0[4];
        let a5 = self.0[5];
        let v0 = v.0[0];
        let v1 = v.0[1];
        out.0[0] = a0;
        out.0[1] = a1;
        out.0[2] = a2;
        out.0[3] = a3;
        out.0[4] = a0 * v0 + a2 * v1 + a4;
        out.0[5] = a1 * v0 + a3 * v1 + a5;

        out
    }

    #[inline(always)]
    pub fn frob(&self) -> T {
        (self.0[0] * self.0[0]
            + self.0[1] * self.0[1]
            + self.0[2] * self.0[2]
            + self.0[3] * self.0[3]
            + self.0[4] * self.0[4]
            + self.0[5] * self.0[5]
            + T::one())
        .sqrt()
    }

    /// Returns whether or not the matrices have approximately the same elements in the same position.
    ///
    /// Refers to `equals` function in `glMatrix`. `exactEquals` is impl<T: Float>emented with [`PartialEq`] and [`Eq`],
    #[inline(always)]
    pub fn approximate_eq(&self, b: &Mat2d<T>) -> bool {
        let a0 = self.0[0];
        let a1 = self.0[1];
        let a2 = self.0[2];
        let a3 = self.0[3];
        let a4 = self.0[4];
        let a5 = self.0[5];

        let b0 = b.0[0];
        let b1 = b.0[1];
        let b2 = b.0[2];
        let b3 = b.0[3];
        let b4 = b.0[4];
        let b5 = b.0[5];

        (a0 - b0).abs() <= epsilon::<T>() * (T::one() as T).max(a0.abs()).max(b0.abs())
            && (a1 - b1).abs() <= epsilon::<T>() * (T::one() as T).max(a1.abs()).max(b1.abs())
            && (a2 - b2).abs() <= epsilon::<T>() * (T::one() as T).max(a2.abs()).max(b2.abs())
            && (a3 - b3).abs() <= epsilon::<T>() * (T::one() as T).max(a3.abs()).max(b3.abs())
            && (a4 - b4).abs() <= epsilon::<T>() * (T::one() as T).max(a4.abs()).max(b4.abs())
            && (a5 - b5).abs() <= epsilon::<T>() * (T::one() as T).max(a5.abs()).max(b5.abs())
    }
}

impl<T: Float> Add<Mat2d<T>> for Mat2d<T> {
    type Output = Mat2d<T>;

    #[inline(always)]
    fn add(self, b: Mat2d<T>) -> Mat2d<T> {
        let mut out = Mat2d::<T>::new_identity();
        out.0[0] = self.0[0] + b.0[0];
        out.0[1] = self.0[1] + b.0[1];
        out.0[2] = self.0[2] + b.0[2];
        out.0[3] = self.0[3] + b.0[3];
        out.0[4] = self.0[4] + b.0[4];
        out.0[5] = self.0[5] + b.0[5];
        out
    }
}

impl<T: Float> Sub<Mat2d<T>> for Mat2d<T> {
    type Output = Mat2d<T>;

    #[inline(always)]
    fn sub(self, b: Mat2d<T>) -> Mat2d<T> {
        let mut out = Mat2d::<T>::new_identity();
        out.0[0] = self.0[0] - b.0[0];
        out.0[1] = self.0[1] - b.0[1];
        out.0[2] = self.0[2] - b.0[2];
        out.0[3] = self.0[3] - b.0[3];
        out.0[4] = self.0[4] - b.0[4];
        out.0[5] = self.0[5] - b.0[5];
        out
    }
}

impl<T: Float> Mul<Mat2d<T>> for Mat2d<T> {
    type Output = Mat2d<T>;

    #[inline(always)]
    fn mul(self, b: Mat2d<T>) -> Mat2d<T> {
        let mut out = Mat2d::<T>::new_identity();
        let a0 = self.0[0];
        let a1 = self.0[1];
        let a2 = self.0[2];
        let a3 = self.0[3];
        let a4 = self.0[4];
        let a5 = self.0[5];

        let b0 = b.0[0];
        let b1 = b.0[1];
        let b2 = b.0[2];
        let b3 = b.0[3];
        let b4 = b.0[4];
        let b5 = b.0[5];

        out.0[0] = a0 * b0 + a2 * b1;
        out.0[1] = a1 * b0 + a3 * b1;
        out.0[2] = a0 * b2 + a2 * b3;
        out.0[3] = a1 * b2 + a3 * b3;
        out.0[4] = a0 * b4 + a2 * b5 + a4;
        out.0[5] = a1 * b4 + a3 * b5 + a5;

        out
    }
}

impl<T: Float> Mul<T> for Mat2d<T> {
    type Output = Mat2d<T>;

    #[inline(always)]
    fn mul(self, b: T) -> Mat2d<T> {
        let mut out = Mat2d::<T>::new_identity();
        out.0[0] = self.0[0] * b;
        out.0[1] = self.0[1] * b;
        out.0[2] = self.0[2] * b;
        out.0[3] = self.0[3] * b;
        out.0[4] = self.0[4] * b;
        out.0[5] = self.0[5] * b;
        out
    }
}

impl<T: Display> Display for Mat2d<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let value = self
            .0
            .iter()
            .map(|v| v.to_string())
            .collect::<Vec<_>>()
            .join(", ");
        f.write_fmt(format_args!("mat2d({})", value))
    }
}

// #[cfg(test)]
// #[rustfmt::skip]
// mod tests {
//     macro_rules! float_test {
//         (T:tt, epsilon::<T>():expr) => {
//             use std::sync::OnceLock;

//             use crate::error::Error;
//             use crate::mat2d::Mat2d;
//             use crate::vec2::Vec2;

//             static MAT_A_RAW: [T; 6] = [
//                 T::one(), 2.0,
//                 3.0, 4.0,
//                 5.0, 6.0
//             ];
//             static MAT_B_RAW: [T; 6] = [
//                  7.0,  8.0,
//                  9.0, 1T::zero(),
//                 1T::one(), 12.0
//             ];

//             static MAT_A: OnceLock<Mat2d<T>> = OnceLock::new();
//             static MAT_B: OnceLock<Mat2d<T>> = OnceLock::new();

//             fn mat_a() -> &'static Mat2d<T> {
//                 MAT_A.get_or_init(|| {
//                     Mat2d::<T>::from_slice(&MAT_A_RAW)
//                 })
//             }

//             fn mat_b() -> &'static Mat2d<T> {
//                 MAT_B.get_or_init(|| {
//                     Mat2d::<T>::from_slice(&MAT_B_RAW)
//                 })
//             }

//             #[test]
//             fn new() {
//                 assert_eq!(
//                     Mat2d::<T>::new().raw(),
//                     &[
//                         T::zero(), T::zero(),
//                         T::zero(), T::zero(),
//                         T::zero(), T::zero()
//                     ]
//                 );
//             }

//             #[test]
//             fn new_identity() {
//                 assert_eq!(
//                     Mat2d::<T>::new_identity().raw(),
//                     &[
//                         T::one(), T::zero(),
//                         T::zero(), T::one(),
//                         T::zero(), T::zero()
//                     ]
//                 );
//             }

//             #[test]
//             fn from_slice() {
//                 assert_eq!(
//                     Mat2d::<T>::from_slice(&[
//                         T::one(), 2.0,
//                         3.0, 4.0,
//                         5.0, 6.0
//                     ]).raw(),
//                     &[
//                         T::one(), 2.0,
//                         3.0, 4.0,
//                         5.0, 6.0
//                     ]
//                 );
//             }

//             #[test]
//             fn from_values() {
//                 assert_eq!(
//                     Mat2d::<T>::from_values(
//                         T::one(), 2.0,
//                         3.0, 4.0,
//                         5.0, 6.0
//                     )
//                     .raw(),
//                     &[
//                         T::one(), 2.0,
//                         3.0, 4.0,
//                         5.0, 6.0
//                     ]
//                 );
//             }

//             #[test]
//             fn invert() -> Result<(), Error> {
//                 assert_eq!(
//                     mat_a().invert()?.raw(),
//                     &[
//                         -2.0,  T::one(),
//                          1.5, -0.5,
//                          T::one(), -2.0
//                     ]
//                 );

//                 Ok(())
//             }

//             #[test]
//             fn determinant() {
//                 assert_eq!(
//                     mat_a().determinant(),
//                     -2.0
//                 );
//             }

//             #[test]
//             fn scale() {
//                 assert_eq!(
//                     mat_a().scale(&Vec2::<T>::from_values(2.0, 3.0)).raw(),
//                     &[
//                         2.0,  4.0,
//                         9.0, 12.0,
//                         5.0,  6.0
//                     ]
//                 );
//             }

//             #[test]
//             fn translate() {
//                 assert_eq!(
//                     mat_a().translate(&Vec2::<T>::from_values(2.0, 3.0)).raw(),
//                     &[
//                          T::one(),  2.0,
//                          3.0,  4.0,
//                         16.0, 22.0
//                     ]
//                 );
//             }

//             #[test]
//             fn frob() {
//                 assert_eq!(
//                     mat_a().frob(),
//                     (
//                         (T::one() as T).powi(2) +
//                         (2.0 as T).powi(2) +
//                         (3.0 as T).powi(2) +
//                         (4.0 as T).powi(2) +
//                         (5.0 as T).powi(2) +
//                         (6.0 as T).powi(2) +
//                         T::one()
//                     ).sqrt()
//                 );
//             }

//             #[test]
//             fn set() {
//                 let mut mat = Mat2d::<T>::new();
//                 mat.set(
//                     T::one(), 2.0,
//                     3.0, 4.0,
//                     5.0, 6.0
//                 );

//                 assert_eq!(
//                     mat.raw(),
//                     &[
//                         T::one(), 2.0,
//                         3.0, 4.0,
//                         5.0, 6.0
//                     ]
//                 );
//             }

//             #[test]
//             fn set_slice() {
//                 let mut mat = Mat2d::<T>::new();
//                 mat.set_slice(&[
//                     T::one(), 2.0,
//                     3.0, 4.0,
//                     5.0, 6.0
//                 ]);

//                 assert_eq!(
//                     mat.raw(),
//                     &[
//                         T::one(), 2.0,
//                         3.0, 4.0,
//                         5.0, 6.0
//                     ]
//                 );
//             }

//             #[test]
//             fn add() {
//                 assert_eq!(
//                     (*mat_a() + *mat_b()).raw(),
//                     &[
//                          8.0, 1T::zero(),
//                         12.0, 14.0,
//                         16.0, 18.0
//                     ]
//                 );
//             }

//             #[test]
//             fn sub() {
//                 assert_eq!(
//                     (*mat_a() - *mat_b()).raw(),
//                     &[
//                         -6.0, -6.0,
//                         -6.0, -6.0,
//                         -6.0, -6.0
//                     ]
//                 );
//             }

//             #[test]
//             fn mul() {
//                 assert_eq!(
//                     (*mat_a() * *mat_b()).raw(),
//                     &[
//                         3T::one(), 46.0,
//                         39.0, 58.0,
//                         52.0, 76.0
//                     ]
//                 );
//             }

//             #[test]
//             fn mul_scalar() {
//                 assert_eq!(
//                     (*mat_a() * 2.0).raw(),
//                     &[
//                          2.0,  4.0,
//                          6.0,  8.0,
//                         1T::zero(), 12.0
//                     ]
//                 );
//             }

//             #[test]
//             fn mul_scalar_add() {
//                 assert_eq!(
//                     (*mat_a() + *mat_b() * 0.5).raw(),
//                     &[
//                          4.5,  6.0,
//                          7.5,  9.0,
//                         10.5, 12.0
//                     ]
//                 );
//             }

//             #[test]
//             fn approximate_eq() {
//                 let mat_a = Mat2d::<T>::from_values(
//                     T::zero(),  T::one(),
//                     2.0,  3.0,
//                     4.0,  5.0
//                 );
//                 let mat_b = Mat2d::<T>::from_values(
//                     T::zero(),  T::one(),
//                     2.0,  3.0,
//                     4.0,  5.0
//                 );
//                 let mat_c = Mat2d::<T>::from_values(
//                     T::one(),  2.0,
//                     3.0,  4.0,
//                     5.0,  6.0
//                 );
//                 let mat_d = Mat2d::<T>::from_values(
//                     1e-16,  T::one(),
//                     2.0,  3.0,
//                     4.0,  5.0
//                 );

//                 assert_eq!(
//                     true,
//                     mat_a.approximate_eq(&mat_b)
//                 );
//                 assert_eq!(
//                     false,
//                     mat_a.approximate_eq(&mat_c)
//                 );
//                 assert_eq!(
//                     true,
//                     mat_a.approximate_eq(&mat_d)
//                 );
//             }

//             #[test]
//             fn display() {
//                 let out = mat_a().to_string();
//                 assert_eq!(
//                     out,
//                     "mat2d(1, 2, 3, 4, 5, 6)"
//                 );
//             }
//         };
//     }

//     mod f32 {
//         float_test!(f32, crate::EPSILON_F32);

//         #[test]
//         fn rotate() {
//             assert_eq!(
//                 mat_a().rotate(std::f32::consts::PI * 0.5).raw(),
//                 &[
//                      3.0,  4.0,
//                      -T::one()000001, -2.0000002,
//                      5.0, 6.0
//                 ]
//             );
//         }
//     }

//     mod f64 {
//         float_test!(f64, crate::EPSILON_F64);

//         #[test]
//         fn rotate() {
//             assert_eq!(
//                 mat_a().rotate(std::f64::consts::PI * 0.5).raw(),
//                 &[
//                      3.0,  4.0,
//                      -0.9999999999999998, -1.9999999999999998,
//                      5.0, 6.0
//                 ]
//             );
//         }
//     }
// }
