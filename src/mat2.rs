use std::{
    fmt::Display,
    ops::{Add, Mul, Sub},
};

use num_traits::Float;

use crate::{epsilon, error::Error, vec2::AsVec2};

pub trait AsMat2<T: Float> {
    fn m00(&self) -> T;

    fn m01(&self) -> T;

    fn m10(&self) -> T;

    fn m11(&self) -> T;

    fn set_m00(&mut self, m00: T) -> &mut Self;

    fn set_m01(&mut self, m01: T) -> &mut Self;

    fn set_m10(&mut self, m10: T) -> &mut Self;

    fn set_m11(&mut self, m11: T) -> &mut Self;

    #[inline(always)]
    fn to_raw(&self) -> [T; 4] {
        [self.m00(), self.m01(), self.m10(), self.m11()]
    }

    #[inline(always)]
    fn to_gl(&self) -> [f32; 4] {
        [
            T::to_f32(&self.m00()).unwrap(),
            T::to_f32(&self.m01()).unwrap(),
            T::to_f32(&self.m10()).unwrap(),
            T::to_f32(&self.m11()).unwrap(),
        ]
    }

    #[inline(always)]
    fn to_gl_binary(&self) -> [u8; 16] {
        unsafe { std::mem::transmute_copy::<[f32; 4], [u8; 16]>(&self.to_gl()) }
    }

    #[inline(always)]
    fn copy<M: AsMat2<T> + ?Sized>(&mut self, b: &M) -> &mut Self {
        self.set_m00(b.m00())
            .set_m01(b.m01())
            .set_m10(b.m10())
            .set_m11(b.m11())
    }

    #[inline(always)]
    fn set(&mut self, m00: T, m01: T, m10: T, m11: T) -> &mut Self {
        self.set_m00(m00).set_m01(m01).set_m10(m10).set_m11(m11)
    }

    #[inline(always)]
    fn set_slice(&mut self, [m00, m01, m10, m11]: &[T; 4]) -> &mut Self {
        self.set_m00(*m00).set_m01(*m01).set_m10(*m10).set_m11(*m11)
    }

    #[inline(always)]
    fn set_zero(&mut self) -> &mut Self {
        self.set_m00(T::zero())
            .set_m01(T::zero())
            .set_m10(T::zero())
            .set_m11(T::zero())
    }

    #[inline(always)]
    fn set_identify(&mut self) -> &mut Self {
        self.set_m00(T::one())
            .set_m01(T::zero())
            .set_m10(T::zero())
            .set_m11(T::one())
    }

    #[inline(always)]
    fn transpose(&mut self) -> &mut Self {
        let a0 = self.m00();
        let a1 = self.m01();
        let a2 = self.m10();
        let a3 = self.m11();

        self.set_m00(a0).set_m01(a2).set_m10(a1).set_m11(a3)
    }

    #[inline(always)]
    fn invert(&mut self) -> Result<&mut Self, Error> {
        let a0 = self.m00();
        let a1 = self.m01();
        let a2 = self.m10();
        let a3 = self.m11();

        // Calculate the determinant
        let mut det = a0 * a3 - a2 * a1;

        if det == T::zero() {
            return Err(Error::ZeroDeterminant);
        }
        det = T::one() / det;

        Ok(self
            .set_m00(a3 * det)
            .set_m01(-a1 * det)
            .set_m10(-a2 * det)
            .set_m11(a0 * det))
    }

    #[inline(always)]
    fn adjoint(&mut self) -> &mut Self {
        let a0 = self.m00();
        let a1 = self.m01();
        let a2 = self.m10();
        let a3 = self.m11();

        self.set_m00(a3).set_m01(-a1).set_m10(-a2).set_m11(a0)
    }

    #[inline(always)]
    fn determinant(&self) -> T {
        let a0 = self.m00();
        let a1 = self.m01();
        let a2 = self.m10();
        let a3 = self.m11();

        a0 * a3 - a2 * a1
    }

    #[inline(always)]
    fn scale<V: AsVec2<T> + ?Sized>(&mut self, v: &V) -> &mut Self {
        let a0 = self.m00();
        let a1 = self.m01();
        let a2 = self.m10();
        let a3 = self.m11();

        let v0 = v.x();
        let v1 = v.y();

        self.set_m00(a0 * v0)
            .set_m01(a1 * v0)
            .set_m10(a2 * v1)
            .set_m11(a3 * v1)
    }

    #[inline(always)]
    fn rotate(&mut self, rad: T) -> &mut Self {
        let a0 = self.m00();
        let a1 = self.m01();
        let a2 = self.m10();
        let a3 = self.m11();

        let s = rad.sin();
        let c = rad.cos();

        self.set_m00(a0 * c + a2 * s)
            .set_m01(a1 * c + a3 * s)
            .set_m10(a0 * -s + a2 * c)
            .set_m11(a1 * -s + a3 * c)
    }

    #[inline(always)]
    fn frob(&self) -> T {
        let a0 = self.m00();
        let a1 = self.m01();
        let a2 = self.m10();
        let a3 = self.m11();

        (a0 * a0 + a1 * a1 + a2 * a2 + a3 * a3).sqrt()
    }

    #[inline(always)]
    fn ldu<M1, M2, M3>(&self, l: &M1, d: &M2, u: &M3) -> (Mat2<T>, Mat2<T>, Mat2<T>)
    where
        M1: AsMat2<T> + ?Sized,
        M2: AsMat2<T> + ?Sized,
        M3: AsMat2<T> + ?Sized,
    {
        let mut l = Mat2::from_values(l.m00(), l.m01(), l.m10(), l.m11());
        let d = Mat2::from_values(d.m00(), d.m01(), d.m10(), d.m11());
        let mut u = Mat2::from_values(u.m00(), u.m01(), u.m10(), u.m11());

        l.0[2] = self.m10() / self.m00();
        u.0[0] = self.m00();
        u.0[1] = self.m01();
        u.0[3] = self.m11() - l.0[2] * u.0[1];
        (l, d, u)
    }

    /// Returns whether or not the matrices have approximately the same elements in the same position.
    ///
    /// Refers to `equals` function in `glMatrix`. `exactEquals` is impl<T: Float>emented with [`PartialEq`] and [`Eq`],
    #[inline(always)]
    fn approximate_eq<V: AsMat2<T> + ?Sized>(&self, b: &V) -> bool {
        let a0 = self.m00();
        let a1 = self.m01();
        let a2 = self.m10();
        let a3 = self.m11();

        let b0 = b.m00();
        let b1 = b.m01();
        let b2 = b.m10();
        let b3 = b.m11();

        (a0 - b0).abs() <= epsilon::<T>() * T::one().max(a0.abs()).max(b0.abs())
            && (a1 - b1).abs() <= epsilon::<T>() * T::one().max(a1.abs()).max(b1.abs())
            && (a2 - b2).abs() <= epsilon::<T>() * T::one().max(a2.abs()).max(b2.abs())
            && (a3 - b3).abs() <= epsilon::<T>() * T::one().max(a3.abs()).max(b3.abs())
    }
}

impl<T: Float> AsMat2<T> for [T; 4] {
    #[inline(always)]
    fn m00(&self) -> T {
        self[0]
    }

    #[inline(always)]
    fn m01(&self) -> T {
        self[1]
    }

    #[inline(always)]
    fn m10(&self) -> T {
        self[2]
    }

    #[inline(always)]
    fn m11(&self) -> T {
        self[3]
    }

    #[inline(always)]
    fn set_m00(&mut self, m00: T) -> &mut Self {
        self[0] = m00;
        self
    }

    #[inline(always)]
    fn set_m01(&mut self, m01: T) -> &mut Self {
        self[1] = m01;
        self
    }

    #[inline(always)]
    fn set_m10(&mut self, m10: T) -> &mut Self {
        self[2] = m10;
        self
    }

    #[inline(always)]
    fn set_m11(&mut self, m11: T) -> &mut Self {
        self[3] = m11;
        self
    }
}

impl<T: Float> AsMat2<T> for (T, T, T, T) {
    #[inline(always)]
    fn m00(&self) -> T {
        self.0
    }

    #[inline(always)]
    fn m01(&self) -> T {
        self.1
    }

    #[inline(always)]
    fn m10(&self) -> T {
        self.2
    }

    #[inline(always)]
    fn m11(&self) -> T {
        self.3
    }

    #[inline(always)]
    fn set_m00(&mut self, m00: T) -> &mut Self {
        self.0 = m00;
        self
    }

    #[inline(always)]
    fn set_m01(&mut self, m01: T) -> &mut Self {
        self.1 = m01;
        self
    }

    #[inline(always)]
    fn set_m10(&mut self, m10: T) -> &mut Self {
        self.2 = m10;
        self
    }

    #[inline(always)]
    fn set_m11(&mut self, m11: T) -> &mut Self {
        self.3 = m11;
        self
    }
}

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
    pub fn from_scaling<V: AsVec2<T> + ?Sized>(v: &V) -> Self {
        Self([v.x(), T::zero(), T::zero(), v.y()])
    }

    #[inline(always)]
    pub fn from_rotation(rad: T) -> Self {
        let s = rad.sin();
        let c = rad.cos();
        Self([c, s, -s, c])
    }

    #[inline(always)]
    pub fn raw(&self) -> &[T; 4] {
        &self.0
    }
}

impl<T: Float> AsMat2<T> for Mat2<T> {
    #[inline(always)]
    fn m00(&self) -> T {
        self.0[0]
    }

    #[inline(always)]
    fn m01(&self) -> T {
        self.0[1]
    }

    #[inline(always)]
    fn m10(&self) -> T {
        self.0[2]
    }

    #[inline(always)]
    fn m11(&self) -> T {
        self.0[3]
    }

    #[inline(always)]
    fn set_m00(&mut self, m00: T) -> &mut Self {
        self.0[0] = m00;
        self
    }

    #[inline(always)]
    fn set_m01(&mut self, m01: T) -> &mut Self {
        self.0[1] = m01;
        self
    }

    #[inline(always)]
    fn set_m10(&mut self, m10: T) -> &mut Self {
        self.0[2] = m10;
        self
    }

    #[inline(always)]
    fn set_m11(&mut self, m11: T) -> &mut Self {
        self.0[3] = m11;
        self
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

#[cfg(test)]
mod tests {
    use std::sync::OnceLock;

    use crate::{error::Error, mat2::AsMat2};

    use super::Mat2;

    static MAT_A_RAW: [f64; 4] = [1.0, 2.0, 3.0, 4.0];
    static MAT_B_RAW: [f64; 4] = [5.0, 6.0, 7.0, 8.0];

    static MAT_A: OnceLock<Mat2> = OnceLock::new();
    static MAT_B: OnceLock<Mat2> = OnceLock::new();

    fn mat_a() -> &'static Mat2 {
        MAT_A.get_or_init(|| Mat2::from_slice(&MAT_A_RAW))
    }

    fn mat_b() -> &'static Mat2 {
        MAT_B.get_or_init(|| Mat2::from_slice(&MAT_B_RAW))
    }

    #[test]
    fn new() {
        assert_eq!(Mat2::<f64>::new().to_raw(), [0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn new_identity() {
        assert_eq!(Mat2::<f64>::new_identity().to_raw(), [1.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn from_slice() {
        assert_eq!(
            Mat2::from_slice(&[1.0, 2.0, 3.0, 4.0]).to_raw(),
            [1.0, 2.0, 3.0, 4.0,]
        );
    }

    #[test]
    fn from_values() {
        assert_eq!(
            Mat2::from_values(1.0, 2.0, 3.0, 4.0).to_raw(),
            [1.0, 2.0, 3.0, 4.0]
        );
    }

    #[test]
    fn transpose() {
        assert_eq!(mat_a().clone().transpose().to_raw(), [1.0, 3.0, 2.0, 4.0]);
    }

    #[test]
    fn invert() -> Result<(), Error> {
        assert_eq!(mat_a().clone().invert()?.to_raw(), [-2.0, 1.0, 1.5, -0.5]);
        Ok(())
    }

    #[test]
    fn adjoint() {
        assert_eq!(mat_a().clone().adjoint().to_raw(), [4.0, -2.0, -3.0, 1.0]);
    }

    #[test]
    fn determinant() {
        assert_eq!(mat_a().determinant(), -2.0);
    }

    #[test]
    fn scale() {
        assert_eq!(
            mat_a().clone().scale(&(2.0, 3.0)).to_raw(),
            [2.0, 4.0, 9.0, 12.0]
        );
    }

    #[test]
    fn frob() {
        assert_eq!(
            mat_a().frob(),
            (1.0f64.powi(2) + 2.0f64.powi(2) + 3.0f64.powi(2) + 4.0f64.powi(2)).sqrt()
        );
    }

    #[test]
    fn ldu() {
        let l = Mat2::new_identity();
        let d = Mat2::new_identity();
        let u = Mat2::new_identity();
        let mat = Mat2::from_values(4.0, 3.0, 6.0, 3.0);

        let (l, d, u) = mat.ldu(&l, &d, &u);

        assert_eq!(l.to_raw(), [1.0, 0.0, 1.5, 1.0]);
        assert_eq!(d.to_raw(), [1.0, 0.0, 0.0, 1.0]);
        assert_eq!(u.to_raw(), [4.0, 3.0, 0.0, -1.5]);
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
        assert_eq!((*mat_a() + *mat_b()).to_raw(), [6.0, 8.0, 10.0, 12.0]);
    }

    #[test]
    fn sub() {
        assert_eq!((*mat_a() - *mat_b()).to_raw(), [-4.0, -4.0, -4.0, -4.0]);
    }

    #[test]
    fn mul() {
        let out = *mat_a() * *mat_b();
        assert_eq!(out.to_raw(), [23.0, 34.0, 31.0, 46.0]);
    }

    #[test]
    fn mul_scalar() {
        assert_eq!((*mat_a() * 2.0).to_raw(), [2.0, 4.0, 6.0, 8.0]);
    }

    #[test]
    fn mul_scalar_add() {
        assert_eq!((*mat_a() + *mat_b() * 0.5).to_raw(), [3.5, 5.0, 6.5, 8.0]);
    }

    #[test]
    fn approximate_eq() {
        let mat_a = Mat2::from_values(0.0, 1.0, 2.0, 3.0);
        let mat_b = Mat2::from_values(0.0, 1.0, 2.0, 3.0);
        let mat_c = Mat2::from_values(1.0, 2.0, 3.0, 4.0);
        let mat_d = Mat2::from_values(1e-16, 1.0, 2.0, 3.0);

        assert_eq!(true, mat_a.approximate_eq(&mat_b));
        assert_eq!(false, mat_a.approximate_eq(&mat_c));
        assert_eq!(true, mat_a.approximate_eq(&mat_d));
    }

    #[test]
    fn display() {
        let out = mat_a().to_string();
        assert_eq!(out, "mat2(1, 2, 3, 4)");
    }

    #[test]
    fn rotate() {
        assert_eq!(
            mat_a().clone().rotate(std::f64::consts::PI * 0.5).to_raw(),
            [3.0, 4.0, -0.9999999999999998, -1.9999999999999998]
        );
    }
}
