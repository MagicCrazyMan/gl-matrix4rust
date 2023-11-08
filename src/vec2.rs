use std::{
    fmt::Display,
    ops::{Add, Div, Mul, Sub},
};

use crate::{
    mat2::Mat2, mat2d::Mat2d, mat3::Mat3, mat4::Mat4, vec3::Vec3, EPSILON_F32, EPSILON_F64,
};
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Vec2<T = f32>(pub [T; 2]);

macro_rules! float {
    ($(($t:tt, $epsilon:expr, $pi:expr)),+) => {
        $(
            impl Vec2<$t> {
                #[inline]
                pub fn new() -> Self {
                    Self([0.0; 2])
                }

                #[inline]
                pub fn from_values(x: $t, y: $t) -> Self {
                    Self([x, y])
                }

                #[inline]
                pub fn from_slice([x, y]: &[$t; 2]) -> Self {
                    Self([*x, *y])
                }
            }

            impl Vec2<$t> {
                #[inline]
                pub fn raw(&self) -> &[$t; 2] {
                    &self.0
                }

                #[inline]
                pub fn set(&mut self, x: $t, y: $t) -> &mut Self {
                    self.0[0] = x;
                    self.0[1] = y;
                    self
                }

                #[inline]
                pub fn set_slice(&mut self, [x, y]: &[$t; 2]) -> &mut Self {
                    self.0[0] = *x;
                    self.0[1] = *y;
                    self
                }

                #[inline]
                pub fn set_zero(&mut self) -> &mut Self {
                    self.0[0] = 0.0;
                    self.0[1] = 0.0;
                    self
                }

                #[inline]
                pub fn ceil(&self) -> Self {
                    let mut out = Self::new();
                    out.0[0] = self.0[0].ceil();
                    out.0[1] = self.0[1].ceil();
                    out
                }

                #[inline]
                pub fn floor(&self) -> Self {
                    let mut out = Self::new();
                    out.0[0] = self.0[0].floor();
                    out.0[1] = self.0[1].floor();
                    out
                }

                #[inline]
                pub fn min(&self, b: &Vec2<$t>) -> Self {
                    let mut out = Self::new();
                    out.0[0] = self.0[0].min(b.0[0]);
                    out.0[1] = self.0[1].min(b.0[1]);
                    out
                }

                #[inline]
                pub fn max(&self, b: &Vec2<$t>) -> Self {
                    let mut out = Self::new();
                    out.0[0] = self.0[0].max(b.0[0]);
                    out.0[1] = self.0[1].max(b.0[1]);
                    out
                }

                #[inline]
                pub fn round(&self) -> Self {
                    Self::from_values(
                        self.0[0].round(),
                        self.0[1].round(),
                    )
                }

                #[inline]
                pub fn scale(&self, scale: $t) -> Self {
                    self.mul(scale)
                }

                #[inline]
                pub fn squared_distance(&self, b: &Vec2<$t>) -> $t {
                    let x = b.0[0] - self.0[0];
                    let y = b.0[1] - self.0[1];
                    x * x + y * y
                }

                #[inline]
                pub fn distance(&self, b: &Vec2<$t>) -> $t {
                    self.squared_distance(b).sqrt()
                }

                #[inline]
                pub fn squared_length(&self) -> $t {
                    let x = self.0[0];
                    let y = self.0[1];
                    x * x + y * y
                }

                #[inline]
                pub fn length(&self) -> $t {
                    self.squared_length().sqrt()
                }

                #[inline]
                pub fn negate(&self) -> Self {
                    let mut out = Self::new();
                    out.0[0] = -self.0[0];
                    out.0[1] = -self.0[1];
                    out
                }

                #[inline]
                pub fn inverse(&self) -> Self {
                    let mut out = Self::new();
                    out.0[0] = 1.0 / self.0[0];
                    out.0[1] = 1.0 / self.0[1];
                    out
                }

                #[inline]
                pub fn normalize(&self) -> Self {
                    let mut len = self.squared_length();
                    if len > 0.0 {
                        len = 1.0 / len.sqrt();
                    }

                    let mut out = Self::new();
                    out.0[0] = self.0[0] * len;
                    out.0[1] = self.0[1] * len;
                    out
                }

                #[inline]
                pub fn dot(&self, b: &Vec2<$t>) -> $t {
                    self.0[0] * b.0[0] + self.0[1] * b.0[1]
                }

                #[inline]
                pub fn cross(&self, b: &Vec2<$t>) -> Vec3::<$t> {
                    Vec3::<$t>::from_values(
                        0.0,
                        0.0,
                        self.0[0] * b.0[1] - self.0[1] * b.0[0]
                    )
                }

                #[inline]
                pub fn lerp(&self, b: &Vec2<$t>, t: $t) -> Self {
                    let mut out = Self::new();
                    let ax = self.0[0];
                    let ay = self.0[1];
                    out.0[0] = ax + t * (b.0[0] - ax);
                    out.0[1] = ay + t * (b.0[1] - ay);
                    out
                }

                #[inline]
                pub fn random(&self, scale: Option<$t>) -> Self {
                    let scale = match scale {
                        Some(scale) => scale,
                        None => 1.0,
                    };

                    let r = rand::random::<$t>() * 2.0 * $pi;

                    Self::from_values(
                        r.cos() * scale,
                        r.sin() * scale
                    )
                }

                #[inline]
                pub fn transform_mat2(&self, m: &Mat2<$t>) -> Self {
                    let mut out = Self::new();
                    let x = self.0[0];
                    let y = self.0[1];
                    out.0[0] = m.0[0] * x + m.0[2] * y;
                    out.0[1] = m.0[1] * x + m.0[3] * y;
                    out
                }

                #[inline]
                pub fn transform_mat2d(&self, m: &Mat2d<$t>) -> Self {
                    let mut out = Self::new();
                    let x = self.0[0];
                    let y = self.0[1];
                    out.0[0] = m.0[0] * x + m.0[2] * y + m.0[4];
                    out.0[1] = m.0[1] * x + m.0[3] * y + m.0[5];
                    out
                }

                #[inline]
                pub fn transform_mat3(&self, m: &Mat3<$t>) -> Self {
                    let mut out = Self::new();
                    let x = self.0[0];
                    let y = self.0[1];
                    out.0[0] = m.0[0] * x + m.0[3] * y + m.0[6];
                    out.0[1] = m.0[1] * x + m.0[4] * y + m.0[7];
                    out
                }

                #[inline]
                pub fn transform_mat4(&self, m: &Mat4<$t>) -> Self {
                    let mut out = Self::new();
                    let x = self.0[0];
                    let y = self.0[1];
                    out.0[0] = m.0[0] * x + m.0[4] * y + m.0[12];
                    out.0[1] = m.0[1] * x + m.0[5] * y + m.0[13];
                    out
                }

                #[inline]
                pub fn rotate(&self, b: &Vec2<$t>, rad: $t) -> Self {
                    let mut out = Self::new();
                    //Translate point to the origin
                    let p0 = self.0[0] - b.0[0];
                    let p1 = self.0[1] - b.0[1];
                    let sin_c = rad.sin();
                    let cos_c = rad.cos();

                    //perform rotation and translate to correct position
                    out.0[0] = p0 * cos_c - p1 * sin_c + b.0[0];
                    out.0[1] = p0 * sin_c + p1 * cos_c + b.0[1];
                    out
                }

                #[inline]
                pub fn angle(&self, b: &Vec2<$t>) -> $t {
                    let x1 = self.0[0];
                    let y1 = self.0[1];
                    let x2 = b.0[0];
                    let y2 = b.0[1];
                    // mag is the product of the magnitudes of a and b
                    let mag = ((x1 * x1 + y1 * y1) * (x2 * x2 + y2 * y2)).sqrt();
                    // mag &&.. short circuits if mag == 0
                    let cosine = if mag == 0.0 {
                        mag
                    } else {
                        (x1 * x2 + y1 * y2) / mag
                    };
                    // Math.min(Math.max(cosine, -1), 1) clamps the cosine between -1 and 1
                    cosine.max(-1.0).min(1.0).acos()
                }

                #[inline]
                pub fn approximate_eq(&self, b: &Vec2<$t>) -> bool {
                    let a0 = self.0[0];
                    let a1 = self.0[1];
                    let b0 = b.0[0];
                    let b1 = b.0[1];
                    (
                      (a0 - b0).abs() <=
                        $epsilon * (1.0 as $t).max(a0.abs()).max(b0.abs()) &&
                      (a1 - b1).abs() <=
                        $epsilon * (1.0 as $t).max(a1.abs()).max(b1.abs())
                    )
                }
            }

            impl Add<Vec2<$t>> for Vec2<$t> {
                type Output = Vec2<$t>;

                #[inline]
                fn add(self, b: Vec2<$t>) -> Vec2<$t> {
                    let mut out = Vec2::<$t>::new();
                    out.0[0] = self.0[0] + b.0[0];
                    out.0[1] = self.0[1] + b.0[1];
                    out
                }
            }

            impl Sub<Vec2<$t>> for Vec2<$t> {
                type Output = Vec2<$t>;

                #[inline]
                fn sub(self, b: Vec2<$t>) -> Vec2<$t> {
                    let mut out = Vec2::<$t>::new();
                    out.0[0] = self.0[0] - b.0[0];
                    out.0[1] = self.0[1] - b.0[1];
                    out
                }
            }

            impl Mul<Vec2<$t>> for Vec2<$t> {
                type Output = Vec2<$t>;

                #[inline]
                fn mul(self, b: Vec2<$t>) -> Vec2<$t> {
                    let mut out = Vec2::<$t>::new();
                    out.0[0] = self.0[0] * b.0[0];
                    out.0[1] = self.0[1] * b.0[1];
                    out
                }
            }

            impl Mul<$t> for Vec2<$t> {
                type Output = Vec2<$t>;

                #[inline]
                fn mul(self, b: $t) -> Vec2<$t> {
                    let mut out = Vec2::<$t>::new();
                    out.0[0] = self.0[0] * b;
                    out.0[1] = self.0[1] * b;
                    out
                }
            }

            impl Div<Vec2<$t>> for Vec2<$t> {
                type Output = Vec2<$t>;

                #[inline]
                fn div(self, b: Vec2<$t>) -> Vec2<$t> {
                    let mut out = Vec2::<$t>::new();
                    out.0[0] = self.0[0] / b.0[0];
                    out.0[1] = self.0[1] / b.0[1];
                    out
                }
            }

            impl Div<$t> for Vec2<$t> {
                type Output = Vec2<$t>;

                #[inline]
                fn div(self, b: $t) -> Vec2<$t> {
                    let mut out = Vec2::<$t>::new();
                    out.0[0] = self.0[0] / b;
                    out.0[1] = self.0[1] / b;
                    out
                }
            }

            impl Display for Vec2<$t> {
                fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                    let value = self.0.iter().map(|v| v.to_string()).collect::<Vec<_>>().join(", ");
                    f.write_fmt(format_args!("vec2({})", value))
                }
            }
        )+
    };
}

float! {
    (f64, EPSILON_F64, std::f64::consts::PI),
    (f32, EPSILON_F32, std::f32::consts::PI)
}

#[cfg(test)]
#[rustfmt::skip]
mod tests {
    macro_rules! float_test {
        ($t:tt, $epsilon:expr, $pi:expr, $e:expr) => {
            use std::sync::OnceLock;

            use crate::error::Error;
            use crate::vec2::Vec2;
            use crate::mat2::Mat2;
            use crate::mat2d::Mat2d;

            static VEC_A_RAW: [$t; 2] = [1.0, 2.0];
            static VEC_B_RAW: [$t; 2] = [3.0, 4.0];

            static VEC_A: OnceLock<Vec2<$t>> = OnceLock::new();
            static VEC_B: OnceLock<Vec2<$t>> = OnceLock::new();

            fn vec_a() -> &'static Vec2<$t> {
                VEC_A.get_or_init(|| {
                    Vec2::<$t>::from_slice(&VEC_A_RAW)
                })
            }

            fn vec_b() -> &'static Vec2<$t> {
                VEC_B.get_or_init(|| {
                    Vec2::<$t>::from_slice(&VEC_B_RAW)
                })
            }

            #[test]
            fn new() {
                assert_eq!(
                    Vec2::<$t>::new().raw(),
                    &[0.0, 0.0]
                );
            }

            #[test]
            fn from_slice() {
                assert_eq!(
                    Vec2::<$t>::from_slice(&[3.0, 4.0]).raw(),
                    &[3.0, 4.0]
                );
            }

            #[test]
            fn from_values() {
                assert_eq!(
                    Vec2::<$t>::from_values(3.0, 4.0).raw(),
                    &[3.0, 4.0]
                );
            }

            #[test]
            fn ceil() {
                assert_eq!(
                    Vec2::<$t>::from_values($e, $pi).ceil().raw(),
                    &[3.0, 4.0]
                );
            }

            #[test]
            fn floor() -> Result<(), Error> {
                assert_eq!(
                    Vec2::<$t>::from_values($e, $pi).floor().raw(),
                    &[2.0, 3.0]
                );

                Ok(())
            }

            #[test]
            fn min() -> Result<(), Error> {
                let vec_a = Vec2::<$t>::from_values(1.0, 4.0);
                let vec_b = Vec2::<$t>::from_values(3.0, 2.0);
                assert_eq!(
                    vec_a.min(&vec_b).raw(),
                    &[1.0, 2.0]
                );

                Ok(())
            }

            #[test]
            fn max() -> Result<(), Error> {
                let vec_a = Vec2::<$t>::from_values(1.0, 4.0);
                let vec_b = Vec2::<$t>::from_values(3.0, 2.0);
                assert_eq!(
                    vec_a.max(&vec_b).raw(),
                    &[3.0, 4.0]
                );

                Ok(())
            }

            #[test]
            fn round() -> Result<(), Error> {
                assert_eq!(
                    Vec2::<$t>::from_values($e, $pi).round().raw(),
                    &[3.0, 3.0]
                );

                Ok(())
            }

            #[test]
            fn scale() {
                assert_eq!(
                    (*vec_a() * 2.0).raw(),
                    &[2.0, 4.0]
                );
            }

            #[test]
            fn scale_add() {
                assert_eq!(
                    (*vec_a() + *vec_b() * 2.0).raw(),
                    &[7.0, 10.0]
                );
            }

            #[test]
            fn squared_distance() {
                assert_eq!(
                    vec_a().squared_distance(vec_b()),
                    8.0
                );
            }

            #[test]
            fn distance() {
                assert_eq!(
                    vec_a().distance(vec_b()),
                    2.8284271247461903
                );
            }

            #[test]
            fn squared_length() {
                assert_eq!(
                    vec_a().squared_length(),
                    5.0
                );
            }

            #[test]
            fn length() {
                assert_eq!(
                    vec_a().length(),
                    2.23606797749979
                );
            }

            #[test]
            fn negate() {
                assert_eq!(
                    vec_a().negate().raw(),
                    &[-1.0 ,-2.0]
                );
            }

            #[test]
            fn normalize() {
                assert_eq!(
                    Vec2::<$t>::from_values(5.0, 0.0).normalize().raw(),
                    &[1.0 ,0.0]
                );
            }

            #[test]
            fn dot() {
                assert_eq!(
                    vec_a().dot(vec_b()),
                    11.0
                );
            }

            #[test]
            fn cross() {
                assert_eq!(
                    vec_a().cross(vec_b()).raw(),
                    &[0.0, 0.0, -2.0]
                );
            }

            #[test]
            fn lerp() {
                assert_eq!(
                    vec_a().lerp(vec_b(), 0.5).raw(),
                    &[2.0, 3.0]
                );
            }

            #[test]
            fn angle() {
                let vec_a = Vec2::<$t>::from_values(1.0, 0.0);
                let vec_b = Vec2::<$t>::from_values(1.0, 2.0);
                assert_eq!(
                    vec_a.angle(&vec_b),
                    1.1071487177940904
                );
            }

            #[test]
            fn transform_mat2() {
                let mat = Mat2::<$t>::from_values(1.0, 2.0, 3.0, 4.0);
                assert_eq!(
                    vec_a().transform_mat2(&mat).raw(),
                    &[7.0, 10.0]
                );
            }

            #[test]
            fn transform_mat2d() {
                let mat = Mat2d::<$t>::from_values(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
                assert_eq!(
                    vec_a().transform_mat2d(&mat).raw(),
                    &[12.0, 16.0]
                );
            }

            #[test]
            fn set() {
                let mut mat = Vec2::<$t>::new();
                mat.set(3.0, 4.0);

                assert_eq!(
                    mat.raw(),
                    &[3.0, 4.0]
                );
            }

            #[test]
            fn set_slice() {
                let mut mat = Vec2::<$t>::new();
                mat.set_slice(&[3.0, 4.0]);

                assert_eq!(
                    mat.raw(),
                    &[3.0, 4.0]
                );
            }

            #[test]
            fn add() {
                assert_eq!(
                    (*vec_a() + *vec_b()).raw(),
                    &[4.0,  6.0]
                );
            }

            #[test]
            fn sub() {
                assert_eq!(
                    (*vec_a() - *vec_b()).raw(),
                    &[-2.0, -2.0]
                );
            }

            #[test]
            fn mul() {
                assert_eq!(
                    (*vec_a() * *vec_b()).raw(),
                    &[3.0, 8.0]
                );
            }

            #[test]
            fn mul_scalar() {
                assert_eq!(
                    (*vec_a() * 2.0).raw(),
                    &[2.0, 4.0]
                );
            }

            #[test]
            fn mul_scalar_add() {
                assert_eq!(
                    (*vec_a() + *vec_b() * 0.5).raw(),
                    &[2.5, 4.0]
                );
            }

            #[test]
            fn div() {
                assert_eq!(
                    (*vec_a() / *vec_b()).raw(),
                    &[0.3333333333333333, 0.5]
                );
            }

            #[test]
            fn div_scalar() {
                assert_eq!(
                    (*vec_a() / 2.0).raw(),
                    &[0.5, 1.0]
                );
            }

            #[test]
            fn div_scalar_add() {
                assert_eq!(
                    (*vec_a() + *vec_b() / 0.5).raw(),
                    &[7.0, 10.0]
                );
            }

            #[test]
            fn approximate_eq() {
                let vec_a = Vec2::<$t>::from_values(0.0,  1.0);
                let vec_b = Vec2::<$t>::from_values(0.0,  1.0);
                let vec_c = Vec2::<$t>::from_values(1.0,  2.0);
                let vec_d = Vec2::<$t>::from_values(1e-16,  1.0);

                assert_eq!(
                    true,
                    vec_a.approximate_eq(&vec_b)
                );
                assert_eq!(
                    false,
                    vec_a.approximate_eq(&vec_c)
                );
                assert_eq!(
                    true,
                    vec_a.approximate_eq(&vec_d)
                );
            }

            #[test]
            fn display() {
                let out = vec_a().to_string();
                assert_eq!(
                    out,
                    "vec2(1, 2)"
                );
            }
        };
    }

    mod f32 {
        float_test!(f32, crate::EPSILON_F32, std::f32::consts::PI, std::f32::consts::E);

        #[test]
        fn rotate() {
            let vec_a = Vec2::<f32>::from_values(0.0, 1.0);
            let vec_b = Vec2::<f32>::from_values(0.0, 0.0);
            assert_eq!(
                vec_a.rotate(&vec_b, std::f32::consts::PI).raw(),
                &[8.742278e-8, -1.0]
            );

            let vec_a = Vec2::<f32>::from_values(6.0, -5.0);
            let vec_b = Vec2::<f32>::from_values(0.0, -5.0);
            assert_eq!(
                vec_a.rotate(&vec_b, std::f32::consts::PI).raw(),
                &[-6.0, -5.0000005]
            );
        }
    }

    mod f64 {
        float_test!(f64, crate::EPSILON_F64, std::f64::consts::PI, std::f64::consts::E);

        #[test]
        fn rotate() {
            let vec_a = Vec2::<f64>::from_values(0.0, 1.0);
            let vec_b = Vec2::<f64>::from_values(0.0, 0.0);
            assert_eq!(
                vec_a.rotate(&vec_b, std::f64::consts::PI).raw(),
                &[-1.2246467991473532e-16, -1.0]
            );

            let vec_a = Vec2::<f64>::from_values(6.0, -5.0);
            let vec_b = Vec2::<f64>::from_values(0.0, -5.0);
            assert_eq!(
                vec_a.rotate(&vec_b, std::f64::consts::PI).raw(),
                &[-6.0, -4.999999999999999]
            );
        }
    }
}
