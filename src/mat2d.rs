use std::{
    fmt::Display,
    ops::{Add, Mul, Sub},
};

use crate::{error::Error, vec2::Vec2, EPSILON_F32, EPSILON_F64};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mat2d<T = f32>(pub [T; 6]);

macro_rules! float {
    ($(($t:tt, $epsilon:expr)),+) => {
        $(
            impl Mat2d<$t> {
                #[inline]
                pub fn new() -> Self {
                    Self([0.0; 6])
                }

                #[inline]
                pub fn new_identity() -> Self {
                    let mut out = Self::new();
                    out.0[0] = 1.0;
                    out.0[3] = 1.0;
                    out
                }

                #[inline]
                pub fn from_values(
                    a: $t,
                    b: $t,
                    c: $t,
                    d: $t,
                    tx: $t,
                    ty: $t,
                ) -> Self {
                    let mut out = Self::new();
                    out.0[0] = a;
                    out.0[1] = b;
                    out.0[2] = c;
                    out.0[3] = d;
                    out.0[4] = tx;
                    out.0[5] = ty;
                    out
                }

                #[inline]
                pub fn from_slice(slice: &[$t; 6]) -> Self {
                    let mut out = Self::new();
                    out.0[0] = slice[0];
                    out.0[1] = slice[1];
                    out.0[2] = slice[2];
                    out.0[3] = slice[3];
                    out.0[4] = slice[4];
                    out.0[5] = slice[5];
                    out
                }

                #[inline]
                pub fn from_scaling(v: &Vec2<$t>) -> Self {
                    let mut out = Self::new();
                    out.0[0] = v.0[0];
                    out.0[1] = 0.0;
                    out.0[2] = 0.0;
                    out.0[3] = v.0[1];
                    out.0[4] = 0.0;
                    out.0[5] = 0.0;
                    out
                }

                #[inline]
                pub fn from_rotation(rad: $t) -> Self {
                    let mut out = Self::new();

                    let s = rad.sin();
                    let c = rad.cos();
                    out.0[0] = c;
                    out.0[1] = s;
                    out.0[2] = -s;
                    out.0[3] = c;
                    out.0[4] = 0.0;
                    out.0[5] = 0.0;

                    out
                }

                #[inline]
                pub fn from_translation(v: &Vec2<$t>) -> Self {
                    let mut out = Self::new();

                    out.0[0] = 1.0;
                    out.0[1] = 0.0;
                    out.0[2] = 0.0;
                    out.0[3] = 1.0;
                    out.0[4] = v.0[0];
                    out.0[5] = v.0[1];

                    out
                }
            }

            impl Mat2d<$t> {
                #[inline]
                pub fn raw(&self) -> &[$t; 6] {
                    &self.0
                }

                #[inline]
                pub fn set(
                    &mut self,
                    a: $t,
                    b: $t,
                    c: $t,
                    d: $t,
                    tx: $t,
                    ty: $t,
                ) -> &mut Self {
                    self.0[0] = a;
                    self.0[1] = b;
                    self.0[2] = c;
                    self.0[3] = d;
                    self.0[4] = tx;
                    self.0[5] = ty;
                    self
                }

                #[inline]
                pub fn set_slice(
                    &mut self,
                    slice: &[$t; 6]
                ) -> &mut Self {
                    self.0 = slice.clone();
                    self
                }

                #[inline]
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

                    if (det == 0.0) {
                        return Err(Error::ZeroDeterminant);
                    }
                    det = 1.0 / det;

                    out.0[0] = ad * det;
                    out.0[1] = -ab * det;
                    out.0[2] = -ac * det;
                    out.0[3] = aa * det;
                    out.0[4] = (ac * aty - ad * atx) * det;
                    out.0[5] = (ab * atx - aa * aty) * det;

                    Ok(out)
                }

                #[inline]
                pub fn determinant(&self) -> $t {
                    self.0[0] * self.0[3] - self.0[1] * self.0[2]
                }

                #[inline]
                pub fn scale(&self, v: &Vec2<$t>) -> Self {
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

                #[inline]
                pub fn rotate(&self, rad: $t) -> Self {
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

                #[inline]
                pub fn translate(&self, v: &Vec2<$t>) -> Self {
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

                #[inline]
                pub fn frob(&self) -> $t {
                    (
                        self.0[0] * self.0[0] +
                        self.0[1] * self.0[1] +
                        self.0[2] * self.0[2] +
                        self.0[3] * self.0[3] +
                        self.0[4] * self.0[4] +
                        self.0[5] * self.0[5] +
                        1.0
                    ).sqrt()
                }

                /// Returns whether or not the matrices have approximately the same elements in the same position.
                ///
                /// Refers to `equals` function in `glMatrix`. `exactEquals` is implemented with [`PartialEq`] and [`Eq`],
                #[inline]
                pub fn approximate_eq(&self, b: &Mat2d<$t>)  -> bool {
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

                    return (
                      (a0 - b0).abs() <= $epsilon * (1.0 as $t).max(a0.abs()).max(b0.abs()) &&
                      (a1 - b1).abs() <= $epsilon * (1.0 as $t).max(a1.abs()).max(b1.abs()) &&
                      (a2 - b2).abs() <= $epsilon * (1.0 as $t).max(a2.abs()).max(b2.abs()) &&
                      (a3 - b3).abs() <= $epsilon * (1.0 as $t).max(a3.abs()).max(b3.abs()) &&
                      (a4 - b4).abs() <= $epsilon * (1.0 as $t).max(a4.abs()).max(b4.abs()) &&
                      (a5 - b5).abs() <= $epsilon * (1.0 as $t).max(a5.abs()).max(b5.abs())
                    );
                }
            }

            impl Add<Mat2d<$t>> for Mat2d<$t> {
                type Output = Mat2d<$t>;

                #[inline]
                fn add(self, b: Mat2d<$t>) -> Mat2d<$t> {
                    let mut out = Mat2d::<$t>::new_identity();
                    out.0[0] = self.0[0] + b.0[0];
                    out.0[1] = self.0[1] + b.0[1];
                    out.0[2] = self.0[2] + b.0[2];
                    out.0[3] = self.0[3] + b.0[3];
                    out.0[4] = self.0[4] + b.0[4];
                    out.0[5] = self.0[5] + b.0[5];
                    out
                }
            }

            impl Sub<Mat2d<$t>> for Mat2d<$t> {
                type Output = Mat2d<$t>;

                #[inline]
                fn sub(self, b: Mat2d<$t>) -> Mat2d<$t> {
                    let mut out = Mat2d::<$t>::new_identity();
                    out.0[0] = self.0[0] - b.0[0];
                    out.0[1] = self.0[1] - b.0[1];
                    out.0[2] = self.0[2] - b.0[2];
                    out.0[3] = self.0[3] - b.0[3];
                    out.0[4] = self.0[4] - b.0[4];
                    out.0[5] = self.0[5] - b.0[5];
                    out
                }
            }

            impl Mul<Mat2d<$t>> for Mat2d<$t> {
                type Output = Mat2d<$t>;

                #[inline]
                fn mul(self, b: Mat2d<$t>) -> Mat2d<$t> {
                    let mut out = Mat2d::<$t>::new_identity();
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

            impl Mul<$t> for Mat2d<$t> {
                type Output = Mat2d<$t>;

                #[inline]
                fn mul(self, b: $t) -> Mat2d<$t> {
                    let mut out = Mat2d::<$t>::new_identity();
                    out.0[0] = self.0[0] * b;
                    out.0[1] = self.0[1] * b;
                    out.0[2] = self.0[2] * b;
                    out.0[3] = self.0[3] * b;
                    out.0[4] = self.0[4] * b;
                    out.0[5] = self.0[5] * b;
                    out
                }
            }

            impl Display for Mat2d<$t> {
                fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                    let value = self.0.iter().map(|v| v.to_string()).collect::<Vec<_>>().join(", ");
                    f.write_fmt(format_args!("mat2d({})", value))
                }
            }
        )+
    };
}

float! {
    (f64, EPSILON_F64),
    (f32, EPSILON_F32)
}

#[cfg(test)]
#[rustfmt::skip]
mod tests {
    macro_rules! float_test {
        ($t:tt, $epsilon:expr) => {
            use std::sync::OnceLock;

            use crate::error::Error;
            use crate::mat2d::Mat2d;
            use crate::vec2::Vec2;

            static MAT_A_RAW: [$t; 6] = [
                1.0, 2.0,
                3.0, 4.0,
                5.0, 6.0
            ];
            static MAT_B_RAW: [$t; 6] = [
                 7.0,  8.0,
                 9.0, 10.0,
                11.0, 12.0
            ];

            static MAT_A: OnceLock<Mat2d<$t>> = OnceLock::new();
            static MAT_B: OnceLock<Mat2d<$t>> = OnceLock::new();

            fn mat_a() -> &'static Mat2d<$t> {
                MAT_A.get_or_init(|| {
                    Mat2d::<$t>::from_slice(&MAT_A_RAW)
                })
            }

            fn mat_b() -> &'static Mat2d<$t> {
                MAT_B.get_or_init(|| {
                    Mat2d::<$t>::from_slice(&MAT_B_RAW)
                })
            }

            #[test]
            fn new() {
                assert_eq!(
                    Mat2d::<$t>::new().raw(),
                    &[
                        0.0, 0.0,
                        0.0, 0.0,
                        0.0, 0.0
                    ]
                );
            }

            #[test]
            fn new_identity() {
                assert_eq!(
                    Mat2d::<$t>::new_identity().raw(),
                    &[
                        1.0, 0.0,
                        0.0, 1.0,
                        0.0, 0.0
                    ]
                );
            }

            #[test]
            fn from_slice() {
                assert_eq!(
                    Mat2d::<$t>::from_slice(&[
                        1.0, 2.0,
                        3.0, 4.0,
                        5.0, 6.0
                    ]).raw(),
                    &[
                        1.0, 2.0,
                        3.0, 4.0,
                        5.0, 6.0
                    ]
                );
            }

            #[test]
            fn from_values() {
                assert_eq!(
                    Mat2d::<$t>::from_values(
                        1.0, 2.0,
                        3.0, 4.0,
                        5.0, 6.0
                    )
                    .raw(),
                    &[
                        1.0, 2.0,
                        3.0, 4.0,
                        5.0, 6.0
                    ]
                );
            }

            #[test]
            fn invert() -> Result<(), Error> {
                assert_eq!(
                    mat_a().invert()?.raw(),
                    &[
                        -2.0,  1.0,
                         1.5, -0.5,
                         1.0, -2.0
                    ]
                );

                Ok(())
            }

            #[test]
            fn determinant() {
                assert_eq!(
                    mat_a().determinant(),
                    -2.0
                );
            }

            #[test]
            fn scale() {
                assert_eq!(
                    mat_a().scale(&Vec2::<$t>::from_values(2.0, 3.0)).raw(),
                    &[
                        2.0,  4.0,
                        9.0, 12.0,
                        5.0,  6.0
                    ]
                );
            }

            #[test]
            fn translate() {
                assert_eq!(
                    mat_a().translate(&Vec2::<$t>::from_values(2.0, 3.0)).raw(),
                    &[
                         1.0,  2.0,
                         3.0,  4.0,
                        16.0, 22.0
                    ]
                );
            }

            #[test]
            fn frob() {
                assert_eq!(
                    mat_a().frob(),
                    (
                        (1.0 as $t).powi(2) +
                        (2.0 as $t).powi(2) +
                        (3.0 as $t).powi(2) +
                        (4.0 as $t).powi(2) +
                        (5.0 as $t).powi(2) +
                        (6.0 as $t).powi(2) +
                        1.0
                    ).sqrt()
                );
            }

            #[test]
            fn set() {
                let mut mat = Mat2d::<$t>::new();
                mat.set(
                    1.0, 2.0,
                    3.0, 4.0,
                    5.0, 6.0
                );

                assert_eq!(
                    mat.raw(),
                    &[
                        1.0, 2.0,
                        3.0, 4.0,
                        5.0, 6.0
                    ]
                );
            }

            #[test]
            fn set_slice() {
                let mut mat = Mat2d::<$t>::new();
                mat.set_slice(&[
                    1.0, 2.0,
                    3.0, 4.0,
                    5.0, 6.0
                ]);

                assert_eq!(
                    mat.raw(),
                    &[
                        1.0, 2.0,
                        3.0, 4.0,
                        5.0, 6.0
                    ]
                );
            }

            #[test]
            fn add() {
                assert_eq!(
                    (*mat_a() + *mat_b()).raw(),
                    &[
                         8.0, 10.0,
                        12.0, 14.0,
                        16.0, 18.0
                    ]
                );
            }

            #[test]
            fn sub() {
                assert_eq!(
                    (*mat_a() - *mat_b()).raw(),
                    &[
                        -6.0, -6.0,
                        -6.0, -6.0,
                        -6.0, -6.0
                    ]
                );
            }

            #[test]
            fn mul() {
                assert_eq!(
                    (*mat_a() * *mat_b()).raw(),
                    &[
                        31.0, 46.0,
                        39.0, 58.0,
                        52.0, 76.0
                    ]
                );
            }

            #[test]
            fn mul_scalar() {
                assert_eq!(
                    (*mat_a() * 2.0).raw(),
                    &[
                         2.0,  4.0,
                         6.0,  8.0,
                        10.0, 12.0
                    ]
                );
            }

            #[test]
            fn mul_scalar_add() {
                assert_eq!(
                    (*mat_a() + *mat_b() * 0.5).raw(),
                    &[
                         4.5,  6.0,
                         7.5,  9.0,
                        10.5, 12.0
                    ]
                );
            }

            #[test]
            fn approximate_eq() {
                let mat_a = Mat2d::<$t>::from_values(
                    0.0,  1.0,
                    2.0,  3.0,
                    4.0,  5.0
                );
                let mat_b = Mat2d::<$t>::from_values(
                    0.0,  1.0,
                    2.0,  3.0,
                    4.0,  5.0
                );
                let mat_c = Mat2d::<$t>::from_values(
                    1.0,  2.0,
                    3.0,  4.0,
                    5.0,  6.0
                );
                let mat_d = Mat2d::<$t>::from_values(
                    1e-16,  1.0,
                    2.0,  3.0,
                    4.0,  5.0
                );

                assert_eq!(
                    true,
                    mat_a.approximate_eq(&mat_b)
                );
                assert_eq!(
                    false,
                    mat_a.approximate_eq(&mat_c)
                );
                assert_eq!(
                    true,
                    mat_a.approximate_eq(&mat_d)
                );
            }

            #[test]
            fn display() {
                let out = mat_a().to_string();
                assert_eq!(
                    out,
                    "mat2d(1, 2, 3, 4, 5, 6)"
                );
            }
        };
    }

    mod f32 {
        float_test!(f32, crate::EPSILON_F32);
        
        #[test]
        fn rotate() {
            assert_eq!(
                mat_a().rotate(std::f32::consts::PI * 0.5).raw(),
                &[
                     3.0,  4.0,
                     -1.0000001, -2.0000002,
                     5.0, 6.0
                ]
            );
        }
    }

    mod f64 {
        float_test!(f64, crate::EPSILON_F64);
        
        #[test]
        fn rotate() {
            assert_eq!(
                mat_a().rotate(std::f64::consts::PI * 0.5).raw(),
                &[
                     3.0,  4.0,
                     -0.9999999999999998, -1.9999999999999998,
                     5.0, 6.0
                ]
            );
        }
    }
}
