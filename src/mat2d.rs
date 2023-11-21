use std::{
    fmt::Display,
    ops::{Add, Mul, Sub},
};

use num_traits::Float;

use crate::{epsilon, error::Error, vec2::AsVec2};

pub trait AsMat2d<T: Float> {
    fn from_values(a: T, b: T, c: T, d: T, tx: T, ty: T) -> Self;

    fn a(&self) -> T;

    fn b(&self) -> T;

    fn c(&self) -> T;

    fn d(&self) -> T;

    fn tx(&self) -> T;

    fn ty(&self) -> T;

    fn set_a(&mut self, a: T) -> &mut Self;

    fn set_b(&mut self, b: T) -> &mut Self;

    fn set_c(&mut self, c: T) -> &mut Self;

    fn set_d(&mut self, d: T) -> &mut Self;

    fn set_tx(&mut self, tx: T) -> &mut Self;

    fn set_ty(&mut self, ty: T) -> &mut Self;

    #[inline(always)]
    fn to_raw(&self) -> [T; 6] {
        [self.a(), self.b(), self.c(), self.d(), self.tx(), self.ty()]
    }

    #[inline(always)]
    fn to_gl(&self) -> [f32; 6] {
        [
            T::to_f32(&self.a()).unwrap(),
            T::to_f32(&self.b()).unwrap(),
            T::to_f32(&self.c()).unwrap(),
            T::to_f32(&self.d()).unwrap(),
            T::to_f32(&self.tx()).unwrap(),
            T::to_f32(&self.ty()).unwrap(),
        ]
    }

    #[inline(always)]
    fn to_gl_binary(&self) -> [u8; 24] {
        unsafe { std::mem::transmute_copy::<[f32; 6], [u8; 24]>(&self.to_gl()) }
    }

    #[inline(always)]
    fn copy<M: AsMat2d<T> + ?Sized>(&mut self, b: &M) -> &mut Self {
        self.set_a(b.a())
            .set_b(b.b())
            .set_c(b.c())
            .set_d(b.d())
            .set_tx(b.tx())
            .set_ty(b.ty())
    }

    #[inline(always)]
    fn set(&mut self, a: T, b: T, c: T, d: T, tx: T, ty: T) -> &mut Self {
        self.set_a(a)
            .set_b(b)
            .set_c(c)
            .set_d(d)
            .set_tx(tx)
            .set_ty(ty)
    }

    #[inline(always)]
    fn set_slice(&mut self, [a, b, c, d, tx, ty]: &[T; 6]) -> &mut Self {
        self.set_a(*a)
            .set_b(*b)
            .set_c(*c)
            .set_d(*d)
            .set_tx(*tx)
            .set_ty(*ty)
    }

    #[inline(always)]
    fn set_zero(&mut self) -> &mut Self {
        self.set_a(T::zero())
            .set_b(T::zero())
            .set_c(T::zero())
            .set_d(T::zero())
            .set_tx(T::zero())
            .set_ty(T::zero())
    }

    #[inline(always)]
    fn set_identify(&mut self) -> &mut Self {
        self.set_a(T::one())
            .set_b(T::zero())
            .set_c(T::zero())
            .set_d(T::one())
            .set_tx(T::zero())
            .set_ty(T::zero())
    }

    #[inline(always)]
    fn invert(&self) -> Result<Self, Error>
    where
        Self: Sized,
    {
        let aa = self.a();
        let ab = self.b();
        let ac = self.c();
        let ad = self.d();
        let atx = self.tx();
        let aty = self.ty();

        // Calculate the determinant
        let mut det = aa * ad - ab * ac;

        if det == T::zero() {
            return Err(Error::ZeroDeterminant);
        }
        det = T::one() / det;

        Ok(Self::from_values(
            ad * det,
            -ab * det,
            -ac * det,
            aa * det,
            (ac * aty - ad * atx) * det,
            (ab * atx - aa * aty) * det,
        ))
    }

    #[inline(always)]
    fn determinant(&self) -> T {
        self.a() * self.d() - self.b() * self.c()
    }

    #[inline(always)]
    fn scale<V: AsVec2<T> + ?Sized>(&self, v: &V) -> Self
    where
        Self: Sized,
    {
        let a0 = self.a();
        let a1 = self.b();
        let a2 = self.c();
        let a3 = self.d();
        let a4 = self.tx();
        let a5 = self.ty();
        let v0 = v.x();
        let v1 = v.y();

        Self::from_values(a0 * v0, a1 * v0, a2 * v1, a3 * v1, a4, a5)
    }

    #[inline(always)]
    fn rotate(&self, rad: T) -> Self
    where
        Self: Sized,
    {
        let a0 = self.a();
        let a1 = self.b();
        let a2 = self.c();
        let a3 = self.d();
        let a4 = self.tx();
        let a5 = self.ty();
        let s = rad.sin();
        let c = rad.cos();

        Self::from_values(
            a0 * c + a2 * s,
            a1 * c + a3 * s,
            a0 * -s + a2 * c,
            a1 * -s + a3 * c,
            a4,
            a5,
        )
    }

    #[inline(always)]
    fn translate<V: AsVec2<T> + ?Sized>(&self, v: &V) -> Self
    where
        Self: Sized,
    {
        let a0 = self.a();
        let a1 = self.b();
        let a2 = self.c();
        let a3 = self.d();
        let a4 = self.tx();
        let a5 = self.ty();
        let v0 = v.x();
        let v1 = v.y();

        Self::from_values(
            a0,
            a1,
            a2,
            a3,
            a0 * v0 + a2 * v1 + a4,
            a1 * v0 + a3 * v1 + a5,
        )
    }

    #[inline(always)]
    fn frob(&self) -> T {
        (self.a() * self.a()
            + self.b() * self.b()
            + self.c() * self.c()
            + self.d() * self.d()
            + self.tx() * self.tx()
            + self.ty() * self.ty()
            + T::one())
        .sqrt()
    }

    /// Returns whether or not the matrices have approximately the same elements in the same position.
    ///
    /// Refers to `equals` function in `glMatrix`. `exactEquals` is impl<T: Float>emented with [`PartialEq`] and [`Eq`],
    #[inline(always)]
    fn approximate_eq<M: AsMat2d<T> + ?Sized>(&self, b: &M) -> bool {
        let a0 = self.a();
        let a1 = self.b();
        let a2 = self.c();
        let a3 = self.d();
        let a4 = self.tx();
        let a5 = self.ty();

        let b0 = b.a();
        let b1 = b.b();
        let b2 = b.c();
        let b3 = b.d();
        let b4 = b.tx();
        let b5 = b.ty();

        (a0 - b0).abs() <= epsilon::<T>() * (T::one() as T).max(a0.abs()).max(b0.abs())
            && (a1 - b1).abs() <= epsilon::<T>() * (T::one() as T).max(a1.abs()).max(b1.abs())
            && (a2 - b2).abs() <= epsilon::<T>() * (T::one() as T).max(a2.abs()).max(b2.abs())
            && (a3 - b3).abs() <= epsilon::<T>() * (T::one() as T).max(a3.abs()).max(b3.abs())
            && (a4 - b4).abs() <= epsilon::<T>() * (T::one() as T).max(a4.abs()).max(b4.abs())
            && (a5 - b5).abs() <= epsilon::<T>() * (T::one() as T).max(a5.abs()).max(b5.abs())
    }
}

impl<T: Float> AsMat2d<T> for [T; 6] {
    #[inline(always)]
    fn from_values(a: T, b: T, c: T, d: T, tx: T, ty: T) -> Self {
        [a, b, c, d, tx, ty]
    }

    #[inline(always)]
    fn a(&self) -> T {
        self[0]
    }

    #[inline(always)]
    fn b(&self) -> T {
        self[1]
    }

    #[inline(always)]
    fn c(&self) -> T {
        self[2]
    }

    #[inline(always)]
    fn d(&self) -> T {
        self[3]
    }

    #[inline(always)]
    fn tx(&self) -> T {
        self[4]
    }

    #[inline(always)]
    fn ty(&self) -> T {
        self[5]
    }

    #[inline(always)]
    fn set_a(&mut self, a: T) -> &mut Self {
        self[0] = a;
        self
    }

    #[inline(always)]
    fn set_b(&mut self, b: T) -> &mut Self {
        self[1] = b;
        self
    }

    #[inline(always)]
    fn set_c(&mut self, c: T) -> &mut Self {
        self[2] = c;
        self
    }

    #[inline(always)]
    fn set_d(&mut self, d: T) -> &mut Self {
        self[3] = d;
        self
    }

    #[inline(always)]
    fn set_tx(&mut self, tx: T) -> &mut Self {
        self[4] = tx;
        self
    }

    #[inline(always)]
    fn set_ty(&mut self, ty: T) -> &mut Self {
        self[5] = ty;
        self
    }
}

impl<T: Float> AsMat2d<T> for (T, T, T, T, T, T) {
    #[inline(always)]
    fn from_values(a: T, b: T, c: T, d: T, tx: T, ty: T) -> Self {
        (a, b, c, d, tx, ty)
    }

    #[inline(always)]
    fn a(&self) -> T {
        self.0
    }

    #[inline(always)]
    fn b(&self) -> T {
        self.1
    }

    #[inline(always)]
    fn c(&self) -> T {
        self.2
    }

    #[inline(always)]
    fn d(&self) -> T {
        self.3
    }

    #[inline(always)]
    fn tx(&self) -> T {
        self.4
    }

    #[inline(always)]
    fn ty(&self) -> T {
        self.5
    }

    #[inline(always)]
    fn set_a(&mut self, a: T) -> &mut Self {
        self.0 = a;
        self
    }

    #[inline(always)]
    fn set_b(&mut self, b: T) -> &mut Self {
        self.1 = b;
        self
    }

    #[inline(always)]
    fn set_c(&mut self, c: T) -> &mut Self {
        self.2 = c;
        self
    }

    #[inline(always)]
    fn set_d(&mut self, d: T) -> &mut Self {
        self.3 = d;
        self
    }

    #[inline(always)]
    fn set_tx(&mut self, tx: T) -> &mut Self {
        self.4 = tx;
        self
    }

    #[inline(always)]
    fn set_ty(&mut self, ty: T) -> &mut Self {
        self.5 = ty;
        self
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mat2d<T = f64>(pub [T; 6]);

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
    pub fn from_slice(slice: &[T; 6]) -> Self {
        Self(slice.clone())
    }

    #[inline(always)]
    pub fn from_scaling<V: AsVec2<T> + ?Sized>(v: &V) -> Self {
        Self([v.x(), T::zero(), T::zero(), v.y(), T::zero(), T::zero()])
    }

    #[inline(always)]
    pub fn from_rotation(rad: T) -> Self {
        let s = rad.sin();
        let c = rad.cos();
        Self([c, s, -s, c, T::zero(), T::zero()])
    }

    #[inline(always)]
    pub fn from_translation<V: AsVec2<T> + ?Sized>(v: &V) -> Self {
        Self([T::one(), T::zero(), T::zero(), T::one(), v.x(), v.y()])
    }

    #[inline(always)]
    pub fn raw(&self) -> &[T; 6] {
        &self.0
    }
}

impl<T: Float> AsMat2d<T> for Mat2d<T> {
    #[inline(always)]
    fn from_values(a: T, b: T, c: T, d: T, tx: T, ty: T) -> Self {
        Self([a, b, c, d, tx, ty])
    }

    #[inline(always)]
    fn a(&self) -> T {
        self.0[0]
    }

    #[inline(always)]
    fn b(&self) -> T {
        self.0[1]
    }

    #[inline(always)]
    fn c(&self) -> T {
        self.0[2]
    }

    #[inline(always)]
    fn d(&self) -> T {
        self.0[3]
    }

    #[inline(always)]
    fn tx(&self) -> T {
        self.0[4]
    }

    #[inline(always)]
    fn ty(&self) -> T {
        self.0[5]
    }

    #[inline(always)]
    fn set_a(&mut self, a: T) -> &mut Self {
        self.0[0] = a;
        self
    }

    #[inline(always)]
    fn set_b(&mut self, b: T) -> &mut Self {
        self.0[1] = b;
        self
    }

    #[inline(always)]
    fn set_c(&mut self, c: T) -> &mut Self {
        self.0[2] = c;
        self
    }

    #[inline(always)]
    fn set_d(&mut self, d: T) -> &mut Self {
        self.0[3] = d;
        self
    }

    #[inline(always)]
    fn set_tx(&mut self, tx: T) -> &mut Self {
        self.0[4] = tx;
        self
    }

    #[inline(always)]
    fn set_ty(&mut self, ty: T) -> &mut Self {
        self.0[5] = ty;
        self
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

impl<T> AsRef<Mat2d<T>> for Mat2d<T> {
    fn as_ref(&self) -> &Self {
        self
    }
}

impl<T> AsRef<[T]> for Mat2d<T> {
    fn as_ref(&self) -> &[T] {
        &self.0
    }
}

impl<T: Float> Default for Mat2d<T> {
    fn default() -> Self {
        Self::new_identity()
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

#[cfg(test)]
mod tests {
    use std::sync::OnceLock;

    use crate::{error::Error, mat2d::AsMat2d};

    use super::Mat2d;

    static MAT_A_RAW: [f64; 6] = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
    static MAT_B_RAW: [f64; 6] = [7.0, 8.0, 9.0, 10.0, 11.0, 12.0];

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
        assert_eq!(Mat2d::<f64>::new().to_raw(), [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn new_identity() {
        assert_eq!(
            Mat2d::<f64>::new_identity().to_raw(),
            [1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        );
    }

    #[test]
    fn from_slice() {
        assert_eq!(
            Mat2d::from_slice(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]).to_raw(),
            [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        );
    }

    #[test]
    fn from_values() {
        assert_eq!(
            Mat2d::from_values(1.0, 2.0, 3.0, 4.0, 5.0, 6.0).to_raw(),
            [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        );
    }

    #[test]
    fn invert() -> Result<(), Error> {
        assert_eq!(
            mat_a().invert()?.to_raw(),
            [-2.0, 1.0, 1.5, -0.5, 1.0, -2.0]
        );

        Ok(())
    }

    #[test]
    fn determinant() {
        assert_eq!(mat_a().determinant(), -2.0);
    }

    #[test]
    fn scale() {
        assert_eq!(
            mat_a().scale(&(2.0, 3.0)).to_raw(),
            [2.0, 4.0, 9.0, 12.0, 5.0, 6.0]
        );
    }

    #[test]
    fn translate() {
        assert_eq!(
            mat_a().translate(&(2.0, 3.0)).to_raw(),
            [1.0, 2.0, 3.0, 4.0, 16.0, 22.0]
        );
    }

    #[test]
    fn frob() {
        assert_eq!(
            mat_a().frob(),
            (1.0f64.powi(2)
                + 2.0f64.powi(2)
                + 3.0f64.powi(2)
                + 4.0f64.powi(2)
                + 5.0f64.powi(2)
                + 6.0f64.powi(2)
                + 1.0)
                .sqrt()
        );
    }

    #[test]
    fn set() {
        let mut mat = Mat2d::new();
        mat.set(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

        assert_eq!(mat.to_raw(), [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn set_slice() {
        let mut mat = Mat2d::new();
        mat.set_slice(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);

        assert_eq!(mat.to_raw(), [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn add() {
        assert_eq!(
            (*mat_a() + *mat_b()).to_raw(),
            [8.0, 10.0, 12.0, 14.0, 16.0, 18.0]
        );
    }

    #[test]
    fn sub() {
        assert_eq!(
            (*mat_a() - *mat_b()).to_raw(),
            [-6.0, -6.0, -6.0, -6.0, -6.0, -6.0]
        );
    }

    #[test]
    fn mul() {
        assert_eq!(
            (*mat_a() * *mat_b()).to_raw(),
            [31.0, 46.0, 39.0, 58.0, 52.0, 76.0]
        );
    }

    #[test]
    fn mul_scalar() {
        assert_eq!((*mat_a() * 2.0).to_raw(), [2.0, 4.0, 6.0, 8.0, 10.0, 12.0]);
    }

    #[test]
    fn mul_scalar_add() {
        assert_eq!(
            (*mat_a() + *mat_b() * 0.5).to_raw(),
            [4.5, 6.0, 7.5, 9.0, 10.5, 12.0]
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
            mat_a().rotate(std::f64::consts::PI * 0.5).to_raw(),
            [3.0, 4.0, -0.9999999999999998, -1.9999999999999998, 5.0, 6.0]
        );
    }
}
