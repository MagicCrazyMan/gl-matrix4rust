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
        Self([
            T::one(),
            T::zero(),
            T::zero(),
            T::one(),
            T::zero(),
            T::zero(),
        ])
    }

    #[inline(always)]
    pub fn from_values(a: T, b: T, c: T, d: T, tx: T, ty: T) -> Self {
        Self([a, b, c, d, tx, ty])
    }

    #[inline(always)]
    pub fn from_slice(slice: &[T; 6]) -> Self {
        Self([slice[0], slice[1], slice[2], slice[3], slice[4], slice[5]])
    }

    #[inline(always)]
    pub fn from_scaling(v: &Vec2<T>) -> Self {
        Self([v.0[0], T::zero(), T::zero(), v.0[1], T::zero(), T::zero()])
    }

    #[inline(always)]
    pub fn from_rotation(rad: T) -> Self {
        let s = rad.sin();
        let c = rad.cos();
        Self([c, s, -s, c, T::zero(), T::zero()])
    }

    #[inline(always)]
    pub fn from_translation(v: &Vec2<T>) -> Self {
        Self([T::one(), T::zero(), T::zero(), T::one(), v.0[0], v.0[1]])
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

        Ok(Self([
            ad * det,
            -ab * det,
            -ac * det,
            aa * det,
            (ac * aty - ad * atx) * det,
            (ab * atx - aa * aty) * det,
        ]))
    }

    #[inline(always)]
    pub fn determinant(&self) -> T {
        self.0[0] * self.0[3] - self.0[1] * self.0[2]
    }

    #[inline(always)]
    pub fn scale(&self, v: &Vec2<T>) -> Self {
        let a0 = self.0[0];
        let a1 = self.0[1];
        let a2 = self.0[2];
        let a3 = self.0[3];
        let a4 = self.0[4];
        let a5 = self.0[5];
        let v0 = v.0[0];
        let v1 = v.0[1];

        Self([a0 * v0, a1 * v0, a2 * v1, a3 * v1, a4, a5])
    }

    #[inline(always)]
    pub fn rotate(&self, rad: T) -> Self {
        let a0 = self.0[0];
        let a1 = self.0[1];
        let a2 = self.0[2];
        let a3 = self.0[3];
        let a4 = self.0[4];
        let a5 = self.0[5];
        let s = rad.sin();
        let c = rad.cos();

        Self([
            a0 * c + a2 * s,
            a1 * c + a3 * s,
            a0 * -s + a2 * c,
            a1 * -s + a3 * c,
            a4,
            a5,
        ])
    }

    #[inline(always)]
    pub fn translate(&self, v: &Vec2<T>) -> Self {
        let a0 = self.0[0];
        let a1 = self.0[1];
        let a2 = self.0[2];
        let a3 = self.0[3];
        let a4 = self.0[4];
        let a5 = self.0[5];
        let v0 = v.0[0];
        let v1 = v.0[1];
        Self([
            a0,
            a1,
            a2,
            a3,
            a0 * v0 + a2 * v1 + a4,
            a1 * v0 + a3 * v1 + a5,
        ])
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
    pub fn approximate_eq(&self, b: &Self) -> bool {
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
    type Output = Self;

    #[inline(always)]
    fn add(self, b: Self) -> Self {
        Self([
            self.0[0] + b.0[0],
            self.0[1] + b.0[1],
            self.0[2] + b.0[2],
            self.0[3] + b.0[3],
            self.0[4] + b.0[4],
            self.0[5] + b.0[5],
        ])
    }
}

impl<T: Float> Sub<Mat2d<T>> for Mat2d<T> {
    type Output = Self;

    #[inline(always)]
    fn sub(self, b: Self) -> Self {
        Self([
            self.0[0] - b.0[0],
            self.0[1] - b.0[1],
            self.0[2] - b.0[2],
            self.0[3] - b.0[3],
            self.0[4] - b.0[4],
            self.0[5] - b.0[5],
        ])
    }
}

impl<T: Float> Mul<Mat2d<T>> for Mat2d<T> {
    type Output = Self;

    #[inline(always)]
    fn mul(self, b: Self) -> Self {
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

        Self([
            a0 * b0 + a2 * b1,
            a1 * b0 + a3 * b1,
            a0 * b2 + a2 * b3,
            a1 * b2 + a3 * b3,
            a0 * b4 + a2 * b5 + a4,
            a1 * b4 + a3 * b5 + a5,
        ])
    }
}

impl<T: Float> Mul<T> for Mat2d<T> {
    type Output = Self;

    #[inline(always)]
    fn mul(self, b: T) -> Self {
        Self([
            self.0[0] * b,
            self.0[1] * b,
            self.0[2] * b,
            self.0[3] * b,
            self.0[4] * b,
            self.0[5] * b,
        ])
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

/// tests only for f32
#[cfg(test)]
mod tests {
    use std::sync::OnceLock;

    use crate::{error::Error, vec2::Vec2};

    use super::Mat2d;

    static MAT_A_RAW: [f32; 6] = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
    static MAT_B_RAW: [f32; 6] = [7.0, 8.0, 9.0, 10.0, 11.0, 12.0];

    static MAT_A: OnceLock<Mat2d> = OnceLock::new();
    static MAT_B: OnceLock<Mat2d> = OnceLock::new();

    fn mat_a() -> &'static Mat2d {
        MAT_A.get_or_init(|| Mat2d::from_slice(&MAT_A_RAW))
    }

    fn mat_b() -> &'static Mat2d {
        MAT_B.get_or_init(|| Mat2d::from_slice(&MAT_B_RAW))
    }

    #[test]
    fn new() {
        assert_eq!(Mat2d::<f32>::new().raw(), &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn new_identity() {
        assert_eq!(
            Mat2d::<f32>::new_identity().raw(),
            &[1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        );
    }

    #[test]
    fn from_slice() {
        assert_eq!(
            Mat2d::from_slice(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]).raw(),
            &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        );
    }

    #[test]
    fn from_values() {
        assert_eq!(
            Mat2d::from_values(1.0, 2.0, 3.0, 4.0, 5.0, 6.0).raw(),
            &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        );
    }

    #[test]
    fn invert() -> Result<(), Error> {
        assert_eq!(mat_a().invert()?.raw(), &[-2.0, 1.0, 1.5, -0.5, 1.0, -2.0]);

        Ok(())
    }

    #[test]
    fn determinant() {
        assert_eq!(mat_a().determinant(), -2.0);
    }

    #[test]
    fn scale() {
        assert_eq!(
            mat_a().scale(&Vec2::from_values(2.0, 3.0)).raw(),
            &[2.0, 4.0, 9.0, 12.0, 5.0, 6.0]
        );
    }

    #[test]
    fn translate() {
        assert_eq!(
            mat_a().translate(&Vec2::from_values(2.0, 3.0)).raw(),
            &[1.0, 2.0, 3.0, 4.0, 16.0, 22.0]
        );
    }

    #[test]
    fn frob() {
        assert_eq!(
            mat_a().frob(),
            (1.0f32.powi(2)
                + 2.0f32.powi(2)
                + 3.0f32.powi(2)
                + 4.0f32.powi(2)
                + 5.0f32.powi(2)
                + 6.0f32.powi(2)
                + 1.0)
                .sqrt()
        );
    }

    #[test]
    fn set() {
        let mut mat = Mat2d::new();
        mat.set(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

        assert_eq!(mat.raw(), &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn set_slice() {
        let mut mat = Mat2d::new();
        mat.set_slice(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);

        assert_eq!(mat.raw(), &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn add() {
        assert_eq!(
            (*mat_a() + *mat_b()).raw(),
            &[8.0, 10.0, 12.0, 14.0, 16.0, 18.0]
        );
    }

    #[test]
    fn sub() {
        assert_eq!(
            (*mat_a() - *mat_b()).raw(),
            &[-6.0, -6.0, -6.0, -6.0, -6.0, -6.0]
        );
    }

    #[test]
    fn mul() {
        assert_eq!(
            (*mat_a() * *mat_b()).raw(),
            &[31.0, 46.0, 39.0, 58.0, 52.0, 76.0]
        );
    }

    #[test]
    fn mul_scalar() {
        assert_eq!((*mat_a() * 2.0).raw(), &[2.0, 4.0, 6.0, 8.0, 10.0, 12.0]);
    }

    #[test]
    fn mul_scalar_add() {
        assert_eq!(
            (*mat_a() + *mat_b() * 0.5).raw(),
            &[4.5, 6.0, 7.5, 9.0, 10.5, 12.0]
        );
    }

    #[test]
    fn approximate_eq() {
        let mat_a = Mat2d::from_values(0.0, 1.0, 2.0, 3.0, 4.0, 5.0);
        let mat_b = Mat2d::from_values(0.0, 1.0, 2.0, 3.0, 4.0, 5.0);
        let mat_c = Mat2d::from_values(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let mat_d = Mat2d::from_values(1e-16, 1.0, 2.0, 3.0, 4.0, 5.0);

        assert_eq!(true, mat_a.approximate_eq(&mat_b));
        assert_eq!(false, mat_a.approximate_eq(&mat_c));
        assert_eq!(true, mat_a.approximate_eq(&mat_d));
    }

    #[test]
    fn display() {
        let out = mat_a().to_string();
        assert_eq!(out, "mat2d(1, 2, 3, 4, 5, 6)");
    }

    #[test]
    fn rotate() {
        assert_eq!(
            mat_a().rotate(std::f32::consts::PI * 0.5).raw(),
            &[3.0, 4.0, -1.0000001, -2.0000002, 5.0, 6.0]
        );
    }
}
