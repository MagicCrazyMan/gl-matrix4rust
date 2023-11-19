use std::{
    fmt::Display,
    ops::{Add, Mul, Sub},
};

use half::f16;
use num_traits::Float;

use crate::{epsilon, error::Error, vec2::Vec2};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mat2<T = f64>(pub [T; 4]);

impl<T: Float> Mat2<T> {
    #[inline(always)]
    pub fn new() -> Self {
        Self([T::zero(); 4])
    }

    #[inline(always)]
    pub fn new_identity() -> Self {
        Self([T::one(), T::zero(), T::zero(), T::one()])
    }

    #[inline(always)]
    pub fn from_values(m00: T, m01: T, m10: T, m11: T) -> Self {
        Self([m00, m01, m10, m11])
    }

    #[inline(always)]
    pub fn from_slice(slice: &[T; 4]) -> Self {
        Self(slice.clone())
    }

    #[inline(always)]
    pub fn from_scaling(v: impl AsRef<Vec2<T>>) -> Self {
        let v = v.as_ref();
        Self([v.0[0], T::zero(), T::zero(), v.0[1]])
    }

    #[inline(always)]
    pub fn from_rotation(rad: T) -> Self {
        let s = rad.sin();
        let c = rad.cos();
        Self([c, s, -s, c])
    }
}

impl<T: Float> Mat2<T> {
    #[inline(always)]
    pub fn to_gl(&self) -> [f32; 4] {
        [
            T::to_f32(&self.0[0]).unwrap(),
            T::to_f32(&self.0[1]).unwrap(),
            T::to_f32(&self.0[2]).unwrap(),
            T::to_f32(&self.0[3]).unwrap(),
        ]
    }

    #[inline(always)]
    pub fn to_gl_binary(&self) -> [u8; 16] {
        unsafe { std::mem::transmute_copy::<[f32; 4], [u8; 16]>(&self.to_gl()) }
    }

    #[inline(always)]
    pub fn raw(&self) -> &[T; 4] {
        &self.0
    }

    #[inline(always)]
    pub fn set(&mut self, m00: T, m01: T, m10: T, m11: T) -> &mut Self {
        self.0[0] = m00;
        self.0[1] = m01;
        self.0[2] = m10;
        self.0[3] = m11;

        self
    }

    #[inline(always)]
    pub fn set_slice(&mut self, slice: &[T; 4]) -> &mut Self {
        self.0 = slice.clone();
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
    pub fn set_identify(&mut self) -> &mut Self {
        self.0[0] = T::one();
        self.0[1] = T::zero();
        self.0[2] = T::zero();
        self.0[3] = T::one();
        self
    }

    #[inline(always)]
    pub fn transpose(&self) -> Self {
        Self([self.0[0], self.0[2], self.0[1], self.0[3]])
    }

    #[inline(always)]
    pub fn invert(&self) -> Result<Self, Error> {
        let a0 = self.0[0];
        let a1 = self.0[1];
        let a2 = self.0[2];
        let a3 = self.0[3];

        // Calculate the determinant
        let mut det = a0 * a3 - a2 * a1;

        if det == T::zero() {
            return Err(Error::ZeroDeterminant);
        }
        det = T::one() / det;

        Ok(Self([a3 * det, -a1 * det, -a2 * det, a0 * det]))
    }

    #[inline(always)]
    pub fn adjoint(&self) -> Self {
        let a0 = self.0[0];
        Self([self.0[3], -self.0[1], -self.0[2], a0])
    }

    #[inline(always)]
    pub fn determinant(&self) -> T {
        self.0[0] * self.0[3] - self.0[2] * self.0[1]
    }

    #[inline(always)]
    pub fn scale(&self, v: impl AsRef<Vec2<T>>) -> Self {
        let v = v.as_ref();

        let a0 = self.0[0];
        let a1 = self.0[1];
        let a2 = self.0[2];
        let a3 = self.0[3];

        let v0 = v.0[0];
        let v1 = v.0[1];

        Self([a0 * v0, a1 * v0, a2 * v1, a3 * v1])
    }

    #[inline(always)]
    pub fn rotate(&self, rad: T) -> Self {
        let a0 = self.0[0];
        let a1 = self.0[1];
        let a2 = self.0[2];
        let a3 = self.0[3];

        let s = rad.sin();
        let c = rad.cos();

        Self([
            a0 * c + a2 * s,
            a1 * c + a3 * s,
            a0 * -s + a2 * c,
            a1 * -s + a3 * c,
        ])
    }

    #[inline(always)]
    pub fn frob(&self) -> T {
        (self.0[0] * self.0[0]
            + self.0[1] * self.0[1]
            + self.0[2] * self.0[2]
            + self.0[3] * self.0[3])
            .sqrt()
    }

    #[inline(always)]
    pub fn ldu(
        &self,
        l: impl AsRef<Self>,
        d: impl AsRef<Self>,
        u: impl AsRef<Self>,
    ) -> (Self, Self, Self) {
        let l = l.as_ref();
        let d = d.as_ref();
        let u = u.as_ref();

        let mut l = Self::from_slice(l.raw());
        let d = Self::from_slice(d.raw());
        let mut u = Self::from_slice(u.raw());
        l.0[2] = self.0[2] / self.0[0];
        u.0[0] = self.0[0];
        u.0[1] = self.0[1];
        u.0[3] = self.0[3] - l.0[2] * u.0[1];
        (l, d, u)
    }

    /// Returns whether or not the matrices have approximately the same elements in the same position.
    ///
    /// Refers to `equals` function in `glMatrix`. `exactEquals` is impl<T: Float>emented with [`PartialEq`] and [`Eq`],
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

impl<T: Float> Add<Mat2<T>> for Mat2<T> {
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

impl<T: Float> Sub<Mat2<T>> for Mat2<T> {
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

impl<T: Float> Mul<Mat2<T>> for Mat2<T> {
    type Output = Self;

    #[inline(always)]
    fn mul(self, b: Self) -> Self {
        let a0 = self.0[0];
        let a1 = self.0[1];
        let a2 = self.0[2];
        let a3 = self.0[3];

        let b0 = b.0[0];
        let b1 = b.0[1];
        let b2 = b.0[2];
        let b3 = b.0[3];

        Self([
            a0 * b0 + a2 * b1,
            a1 * b0 + a3 * b1,
            a0 * b2 + a2 * b3,
            a1 * b2 + a3 * b3,
        ])
    }
}

impl<T: Float> Mul<T> for Mat2<T> {
    type Output = Self;

    #[inline(always)]
    fn mul(self, b: T) -> Self {
        Self([self.0[0] * b, self.0[1] * b, self.0[2] * b, self.0[3] * b])
    }
}

impl<T> AsRef<Mat2<T>> for Mat2<T> {
    fn as_ref(&self) -> &Self {
        self
    }
}

impl<T> AsRef<[T]> for Mat2<T> {
    fn as_ref(&self) -> &[T] {
        &self.0
    }
}

impl AsRef<[u8]> for Mat2<f64> {
    fn as_ref(&self) -> &[u8] {
        unsafe { std::mem::transmute::<&[f64; 4], &[u8; 32]>(&self.0) }
    }
}

impl AsRef<[u8]> for Mat2<f32> {
    fn as_ref(&self) -> &[u8] {
        unsafe { std::mem::transmute::<&[f32; 4], &[u8; 16]>(&self.0) }
    }
}

impl AsRef<[u8]> for Mat2<f16> {
    fn as_ref(&self) -> &[u8] {
        unsafe { std::mem::transmute::<&[f16; 4], &[u8; 8]>(&self.0) }
    }
}

impl<T: Float> Default for Mat2<T> {
    fn default() -> Self {
        Self::new_identity()
    }
}

impl<T: Display> Display for Mat2<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let value = self
            .0
            .iter()
            .map(|v| v.to_string())
            .collect::<Vec<_>>()
            .join(", ");
        f.write_fmt(format_args!("mat2({})", value))
    }
}

/// tests only for f32
#[cfg(test)]
mod tests {
    use std::sync::OnceLock;

    use crate::{error::Error, vec2::Vec2};

    use super::Mat2;

    static MAT_A_RAW: [f32; 4] = [1.0, 2.0, 3.0, 4.0];
    static MAT_B_RAW: [f32; 4] = [5.0, 6.0, 7.0, 8.0];

    static MAT_A: OnceLock<Mat2<f32>> = OnceLock::new();
    static MAT_B: OnceLock<Mat2<f32>> = OnceLock::new();

    fn mat_a() -> &'static Mat2<f32> {
        MAT_A.get_or_init(|| Mat2::from_slice(&MAT_A_RAW))
    }

    fn mat_b() -> &'static Mat2<f32> {
        MAT_B.get_or_init(|| Mat2::from_slice(&MAT_B_RAW))
    }

    #[test]
    fn new() {
        assert_eq!(Mat2::<f32>::new().raw(), &[0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn new_identity() {
        assert_eq!(Mat2::<f32>::new_identity().raw(), &[1.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn from_slice() {
        assert_eq!(
            Mat2::from_slice(&[1.0, 2.0, 3.0, 4.0]).raw(),
            &[1.0, 2.0, 3.0, 4.0,]
        );
    }

    #[test]
    fn from_values() {
        assert_eq!(
            Mat2::from_values(1.0, 2.0, 3.0, 4.0).raw(),
            &[1.0, 2.0, 3.0, 4.0]
        );
    }

    #[test]
    fn transpose() {
        assert_eq!(mat_a().transpose().raw(), &[1.0, 3.0, 2.0, 4.0]);
    }

    #[test]
    fn invert() -> Result<(), Error> {
        assert_eq!(mat_a().invert()?.raw(), &[-2.0, 1.0, 1.5, -0.5]);
        Ok(())
    }

    #[test]
    fn adjoint() {
        assert_eq!(mat_a().adjoint().raw(), &[4.0, -2.0, -3.0, 1.0]);
    }

    #[test]
    fn determinant() {
        assert_eq!(mat_a().determinant(), -2.0);
    }

    #[test]
    fn scale() {
        assert_eq!(
            mat_a().scale(Vec2::from_values(2.0, 3.0)).raw(),
            &[2.0, 4.0, 9.0, 12.0]
        );
    }

    #[test]
    fn frob() {
        assert_eq!(
            mat_a().frob(),
            (1.0f32.powi(2) + 2.0f32.powi(2) + 3.0f32.powi(2) + 4.0f32.powi(2)).sqrt()
        );
    }

    #[test]
    fn ldu() {
        let l = Mat2::new_identity();
        let d = Mat2::new_identity();
        let u = Mat2::new_identity();
        let mat = Mat2::from_values(4.0, 3.0, 6.0, 3.0);

        let (l, d, u) = mat.ldu(l, d, u);

        assert_eq!(l.raw(), &[1.0, 0.0, 1.5, 1.0]);
        assert_eq!(d.raw(), &[1.0, 0.0, 0.0, 1.0]);
        assert_eq!(u.raw(), &[4.0, 3.0, 0.0, -1.5]);
    }

    #[test]
    fn set() {
        let mut mat = Mat2::new();
        mat.set(1.0, 2.0, 3.0, 4.0);

        assert_eq!(mat.raw(), &[1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn set_slice() {
        let mut mat = Mat2::new();
        mat.set_slice(&[1.0, 2.0, 3.0, 4.0]);

        assert_eq!(mat.raw(), &[1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn add() {
        assert_eq!((*mat_a() + *mat_b()).raw(), &[6.0, 8.0, 10.0, 12.0]);
    }

    #[test]
    fn sub() {
        assert_eq!((*mat_a() - *mat_b()).raw(), &[-4.0, -4.0, -4.0, -4.0]);
    }

    #[test]
    fn mul() {
        let out = *mat_a() * *mat_b();
        assert_eq!(out.raw(), &[23.0, 34.0, 31.0, 46.0]);
    }

    #[test]
    fn mul_scalar() {
        assert_eq!((*mat_a() * 2.0).raw(), &[2.0, 4.0, 6.0, 8.0]);
    }

    #[test]
    fn mul_scalar_add() {
        assert_eq!((*mat_a() + *mat_b() * 0.5).raw(), &[3.5, 5.0, 6.5, 8.0]);
    }

    #[test]
    fn approximate_eq() {
        let mat_a = Mat2::from_values(0.0, 1.0, 2.0, 3.0);
        let mat_b = Mat2::from_values(0.0, 1.0, 2.0, 3.0);
        let mat_c = Mat2::from_values(1.0, 2.0, 3.0, 4.0);
        let mat_d = Mat2::from_values(1e-16, 1.0, 2.0, 3.0);

        assert_eq!(true, mat_a.approximate_eq(mat_b));
        assert_eq!(false, mat_a.approximate_eq(mat_c));
        assert_eq!(true, mat_a.approximate_eq(mat_d));
    }

    #[test]
    fn display() {
        let out = mat_a().to_string();
        assert_eq!(out, "mat2(1, 2, 3, 4)");
    }

    #[test]
    fn rotate() {
        assert_eq!(
            mat_a().rotate(std::f32::consts::PI * 0.5).raw(),
            &[3.0, 4.0, -1.0000001, -2.0000002]
        );
    }

    #[test]
    fn test_u8_slice() {
        let bin: &[u8] = mat_a().as_ref();
        bin.chunks(4).enumerate().for_each(|(index, bin)| {
            let value = f32::from_ne_bytes(bin.try_into().unwrap());
            assert_eq!(mat_a().0[index], value);
        });
    }
}
