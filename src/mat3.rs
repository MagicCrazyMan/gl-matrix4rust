use std::{
    fmt::Display,
    ops::{Add, Mul, Sub},
};

use num_traits::Float;

use crate::{epsilon, error::Error, mat2d::AsMat2d, mat4::AsMat4, quat::AsQuat, vec2::AsVec2};

pub trait AsMat3<T: Float> {
    fn m00(&self) -> T;

    fn m01(&self) -> T;

    fn m02(&self) -> T;

    fn m10(&self) -> T;

    fn m11(&self) -> T;

    fn m12(&self) -> T;

    fn m20(&self) -> T;

    fn m21(&self) -> T;

    fn m22(&self) -> T;

    fn set_m00(&mut self, m00: T) -> &mut Self;

    fn set_m01(&mut self, m01: T) -> &mut Self;

    fn set_m02(&mut self, m02: T) -> &mut Self;

    fn set_m10(&mut self, m10: T) -> &mut Self;

    fn set_m11(&mut self, m11: T) -> &mut Self;

    fn set_m12(&mut self, m12: T) -> &mut Self;

    fn set_m20(&mut self, m20: T) -> &mut Self;

    fn set_m21(&mut self, m21: T) -> &mut Self;

    fn set_m22(&mut self, m22: T) -> &mut Self;

    #[inline(always)]
    fn to_raw(&self) -> [T; 9] {
        [
            self.m00(),
            self.m01(),
            self.m02(),
            self.m10(),
            self.m11(),
            self.m12(),
            self.m20(),
            self.m21(),
            self.m22(),
        ]
    }

    #[inline(always)]
    fn to_gl(&self) -> [f32; 9] {
        [
            T::to_f32(&self.m00()).unwrap(),
            T::to_f32(&self.m01()).unwrap(),
            T::to_f32(&self.m02()).unwrap(),
            T::to_f32(&self.m10()).unwrap(),
            T::to_f32(&self.m11()).unwrap(),
            T::to_f32(&self.m12()).unwrap(),
            T::to_f32(&self.m20()).unwrap(),
            T::to_f32(&self.m21()).unwrap(),
            T::to_f32(&self.m22()).unwrap(),
        ]
    }

    #[inline(always)]
    fn to_gl_binary(&self) -> [u8; 36] {
        unsafe { std::mem::transmute_copy::<[f32; 9], [u8; 36]>(&self.to_gl()) }
    }

    #[inline(always)]
    fn copy<M: AsMat3<T> + ?Sized>(&mut self, b: &M) -> &mut Self {
        self.set_m00(b.m00())
            .set_m01(b.m01())
            .set_m02(b.m02())
            .set_m10(b.m10())
            .set_m11(b.m11())
            .set_m12(b.m12())
            .set_m20(b.m20())
            .set_m21(b.m21())
            .set_m22(b.m22())
    }

    #[inline(always)]
    fn set(
        &mut self,
        m00: T,
        m01: T,
        m02: T,
        m10: T,
        m11: T,
        m12: T,
        m20: T,
        m21: T,
        m22: T,
    ) -> &mut Self {
        self.set_m00(m00)
            .set_m01(m01)
            .set_m02(m02)
            .set_m10(m10)
            .set_m11(m11)
            .set_m12(m12)
            .set_m20(m20)
            .set_m21(m21)
            .set_m22(m22)
    }

    #[inline(always)]
    fn set_slice(&mut self, slice: &[T; 9]) -> &mut Self {
        let [m00, m01, m02, m10, m11, m12, m20, m21, m22] = slice;
        self.set_m00(*m00)
            .set_m01(*m01)
            .set_m02(*m02)
            .set_m10(*m10)
            .set_m11(*m11)
            .set_m12(*m12)
            .set_m20(*m20)
            .set_m21(*m21)
            .set_m22(*m22)
    }

    #[inline(always)]
    fn set_zero(&mut self) -> &mut Self {
        self.set_m00(T::zero())
            .set_m01(T::zero())
            .set_m02(T::zero())
            .set_m10(T::zero())
            .set_m11(T::zero())
            .set_m12(T::zero())
            .set_m20(T::zero())
            .set_m21(T::zero())
            .set_m22(T::zero())
    }

    #[inline(always)]
    fn set_identify(&mut self) -> &mut Self {
        self.set_m00(T::one())
            .set_m01(T::zero())
            .set_m02(T::zero())
            .set_m10(T::zero())
            .set_m11(T::one())
            .set_m12(T::zero())
            .set_m20(T::zero())
            .set_m21(T::zero())
            .set_m22(T::one())
    }

    #[inline(always)]
    fn transpose(&mut self) -> &mut Self {
        let a00 = self.m00();
        let a01 = self.m01();
        let a02 = self.m02();
        let m10 = self.m10();
        let m11 = self.m11();
        let m12 = self.m12();
        let m20 = self.m20();
        let m21 = self.m21();
        let m22 = self.m22();

        self.set_m00(a00)
            .set_m01(m10)
            .set_m02(m20)
            .set_m10(a01)
            .set_m11(m11)
            .set_m12(m21)
            .set_m20(a02)
            .set_m21(m12)
            .set_m22(m22)
    }

    #[inline(always)]
    fn invert(&mut self) -> Result<&mut Self, Error> {
        let a00 = self.m00();
        let a01 = self.m01();
        let a02 = self.m02();
        let a10 = self.m10();
        let a11 = self.m11();
        let a12 = self.m12();
        let a20 = self.m20();
        let a21 = self.m21();
        let a22 = self.m22();

        let b01 = a22 * a11 - a12 * a21;
        let b11 = -a22 * a10 + a12 * a20;
        let b21 = a21 * a10 - a11 * a20;

        // Calculate the determinant
        let mut det = a00 * b01 + a01 * b11 + a02 * b21;

        if det == T::zero() {
            return Err(Error::ZeroDeterminant);
        }
        det = T::one() / det;

        Ok(self
            .set_m00(b01 * det)
            .set_m01((-a22 * a01 + a02 * a21) * det)
            .set_m02((a12 * a01 - a02 * a11) * det)
            .set_m10(b11 * det)
            .set_m11((a22 * a00 - a02 * a20) * det)
            .set_m12((-a12 * a00 + a02 * a10) * det)
            .set_m20(b21 * det)
            .set_m21((-a21 * a00 + a01 * a20) * det)
            .set_m22((a11 * a00 - a01 * a10) * det))
    }

    #[inline(always)]
    fn adjoint(&mut self) -> &mut Self {
        let a00 = self.m00();
        let a01 = self.m01();
        let a02 = self.m02();
        let a10 = self.m10();
        let a11 = self.m11();
        let a12 = self.m12();
        let a20 = self.m20();
        let a21 = self.m21();
        let a22 = self.m22();

        self.set_m00(a11 * a22 - a12 * a21)
            .set_m01(a02 * a21 - a01 * a22)
            .set_m02(a01 * a12 - a02 * a11)
            .set_m10(a12 * a20 - a10 * a22)
            .set_m11(a00 * a22 - a02 * a20)
            .set_m12(a02 * a10 - a00 * a12)
            .set_m20(a10 * a21 - a11 * a20)
            .set_m21(a01 * a20 - a00 * a21)
            .set_m22(a00 * a11 - a01 * a10)
    }

    #[inline(always)]
    fn determinant(&self) -> T {
        let a00 = self.m00();
        let a01 = self.m01();
        let a02 = self.m02();
        let a10 = self.m10();
        let a11 = self.m11();
        let a12 = self.m12();
        let a20 = self.m20();
        let a21 = self.m21();
        let a22 = self.m22();

        a00 * (a22 * a11 - a12 * a21)
            + a01 * (-a22 * a10 + a12 * a20)
            + a02 * (a21 * a10 - a11 * a20)
    }

    #[inline(always)]
    fn translate<V: AsVec2<T> + ?Sized>(&mut self, v: &V) -> &mut Self {
        let x = v.x();
        let y = v.y();

        let a00 = self.m00();
        let a01 = self.m01();
        let a02 = self.m02();
        let a10 = self.m10();
        let a11 = self.m11();
        let a12 = self.m12();
        let a20 = self.m20();
        let a21 = self.m21();
        let a22 = self.m22();

        self.set_m00(a00)
            .set_m01(a01)
            .set_m02(a02)
            .set_m10(a10)
            .set_m11(a11)
            .set_m12(a12)
            .set_m20(x * a00 + y * a10 + a20)
            .set_m21(x * a01 + y * a11 + a21)
            .set_m22(x * a02 + y * a12 + a22)
    }

    #[inline(always)]
    fn scale<V: AsVec2<T> + ?Sized>(&mut self, v: &V) -> &mut Self {
        let x = v.x();
        let y = v.y();

        let a00 = self.m00();
        let a01 = self.m01();
        let a02 = self.m02();
        let a10 = self.m10();
        let a11 = self.m11();
        let a12 = self.m12();
        let a20 = self.m20();
        let a21 = self.m21();
        let a22 = self.m22();

        self.set_m00(x * a00)
            .set_m01(x * a01)
            .set_m02(x * a02)
            .set_m10(y * a10)
            .set_m11(y * a11)
            .set_m12(y * a12)
            .set_m20(a20)
            .set_m21(a21)
            .set_m22(a22)
    }

    #[inline(always)]
    fn rotate(&mut self, rad: T) -> &mut Self {
        let a00 = self.m00();
        let a01 = self.m01();
        let a02 = self.m02();
        let a10 = self.m10();
        let a11 = self.m11();
        let a12 = self.m12();
        let a20 = self.m20();
        let a21 = self.m21();
        let a22 = self.m22();

        let s = rad.sin();
        let c = rad.cos();

        self.set_m00(c * a00 + s * a10)
            .set_m01(c * a01 + s * a11)
            .set_m02(c * a02 + s * a12)
            .set_m10(c * a10 - s * a00)
            .set_m11(c * a11 - s * a01)
            .set_m12(c * a12 - s * a02)
            .set_m20(a20)
            .set_m21(a21)
            .set_m22(a22)
    }

    #[inline(always)]
    fn frob(&self) -> T {
        let a00 = self.m00();
        let a01 = self.m01();
        let a02 = self.m02();
        let a10 = self.m10();
        let a11 = self.m11();
        let a12 = self.m12();
        let a20 = self.m20();
        let a21 = self.m21();
        let a22 = self.m22();

        (a00 * a00
            + a01 * a01
            + a02 * a02
            + a10 * a10
            + a11 * a11
            + a12 * a12
            + a20 * a20
            + a21 * a21
            + a22 * a22)
            .sqrt()
    }

    /// Returns whether or not the matrices have approximately the same elements in the same position.
    ///
    /// Refers to `equals` function in `glMatrix`. `exactEquals` is implemented with [`PartialEq`] and [`Eq`],
    #[inline(always)]
    fn approximate_eq<M: AsMat3<T> + ?Sized>(&self, b: &M) -> bool {
        let a0 = self.m00();
        let a1 = self.m01();
        let a2 = self.m02();
        let a3 = self.m10();
        let a4 = self.m11();
        let a5 = self.m12();
        let a6 = self.m20();
        let a7 = self.m21();
        let a8 = self.m22();

        let b0 = b.m00();
        let b1 = b.m01();
        let b2 = b.m02();
        let b3 = b.m10();
        let b4 = b.m11();
        let b5 = b.m12();
        let b6 = b.m20();
        let b7 = b.m21();
        let b8 = b.m22();

        (a0 - b0).abs() <= epsilon::<T>() * T::one().max(a0.abs()).max(b0.abs())
            && (a1 - b1).abs() <= epsilon::<T>() * T::one().max(a1.abs()).max(b1.abs())
            && (a2 - b2).abs() <= epsilon::<T>() * T::one().max(a2.abs()).max(b2.abs())
            && (a3 - b3).abs() <= epsilon::<T>() * T::one().max(a3.abs()).max(b3.abs())
            && (a4 - b4).abs() <= epsilon::<T>() * T::one().max(a4.abs()).max(b4.abs())
            && (a5 - b5).abs() <= epsilon::<T>() * T::one().max(a5.abs()).max(b5.abs())
            && (a6 - b6).abs() <= epsilon::<T>() * T::one().max(a6.abs()).max(b6.abs())
            && (a7 - b7).abs() <= epsilon::<T>() * T::one().max(a7.abs()).max(b7.abs())
            && (a8 - b8).abs() <= epsilon::<T>() * T::one().max(a8.abs()).max(b8.abs())
    }
}

impl<T: Float> AsMat3<T> for [T; 9] {
    #[inline(always)]
    fn m00(&self) -> T {
        self[0]
    }

    #[inline(always)]
    fn m01(&self) -> T {
        self[1]
    }

    #[inline(always)]
    fn m02(&self) -> T {
        self[2]
    }

    #[inline(always)]
    fn m10(&self) -> T {
        self[3]
    }

    #[inline(always)]
    fn m11(&self) -> T {
        self[4]
    }

    #[inline(always)]
    fn m12(&self) -> T {
        self[5]
    }

    #[inline(always)]
    fn m20(&self) -> T {
        self[6]
    }

    #[inline(always)]
    fn m21(&self) -> T {
        self[7]
    }

    #[inline(always)]
    fn m22(&self) -> T {
        self[8]
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
    fn set_m02(&mut self, m02: T) -> &mut Self {
        self[2] = m02;
        self
    }

    #[inline(always)]
    fn set_m10(&mut self, m10: T) -> &mut Self {
        self[3] = m10;
        self
    }

    #[inline(always)]
    fn set_m11(&mut self, m11: T) -> &mut Self {
        self[4] = m11;
        self
    }

    #[inline(always)]
    fn set_m12(&mut self, m12: T) -> &mut Self {
        self[5] = m12;
        self
    }

    #[inline(always)]
    fn set_m20(&mut self, m20: T) -> &mut Self {
        self[6] = m20;
        self
    }

    #[inline(always)]
    fn set_m21(&mut self, m21: T) -> &mut Self {
        self[7] = m21;
        self
    }

    #[inline(always)]
    fn set_m22(&mut self, m22: T) -> &mut Self {
        self[8] = m22;
        self
    }
}

impl<T: Float> AsMat3<T> for (T, T, T, T, T, T, T, T, T) {
    #[inline(always)]
    fn m00(&self) -> T {
        self.0
    }

    #[inline(always)]
    fn m01(&self) -> T {
        self.1
    }

    #[inline(always)]
    fn m02(&self) -> T {
        self.2
    }

    #[inline(always)]
    fn m10(&self) -> T {
        self.3
    }

    #[inline(always)]
    fn m11(&self) -> T {
        self.4
    }

    #[inline(always)]
    fn m12(&self) -> T {
        self.5
    }

    #[inline(always)]
    fn m20(&self) -> T {
        self.6
    }

    #[inline(always)]
    fn m21(&self) -> T {
        self.7
    }

    #[inline(always)]
    fn m22(&self) -> T {
        self.8
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
    fn set_m02(&mut self, m02: T) -> &mut Self {
        self.2 = m02;
        self
    }

    #[inline(always)]
    fn set_m10(&mut self, m10: T) -> &mut Self {
        self.3 = m10;
        self
    }

    #[inline(always)]
    fn set_m11(&mut self, m11: T) -> &mut Self {
        self.4 = m11;
        self
    }

    #[inline(always)]
    fn set_m12(&mut self, m12: T) -> &mut Self {
        self.5 = m12;
        self
    }

    #[inline(always)]
    fn set_m20(&mut self, m20: T) -> &mut Self {
        self.6 = m20;
        self
    }

    #[inline(always)]
    fn set_m21(&mut self, m21: T) -> &mut Self {
        self.7 = m21;
        self
    }

    #[inline(always)]
    fn set_m22(&mut self, m22: T) -> &mut Self {
        self.8 = m22;
        self
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mat3<T = f64>(pub [T; 9]);

impl<T: Float> Mat3<T> {
    #[inline(always)]
    pub fn new() -> Self {
        Self([T::zero(); 9])
    }

    #[inline(always)]
    pub fn new_identity() -> Self {
        Self([
            T::one(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_mat4<M: AsMat4<T> + ?Sized>(mat: &M) -> Self {
        Self([
            mat.m00(),
            mat.m01(),
            mat.m02(),
            mat.m10(),
            mat.m11(),
            mat.m12(),
            mat.m20(),
            mat.m21(),
            mat.m22(),
        ])
    }

    #[inline(always)]
    pub fn from_values(
        m00: T,
        m01: T,
        m02: T,
        m10: T,
        m11: T,
        m12: T,
        m20: T,
        m21: T,
        m22: T,
    ) -> Self {
        Self([m00, m01, m02, m10, m11, m12, m20, m21, m22])
    }

    #[inline(always)]
    pub fn from_slice(slice: &[T; 9]) -> Self {
        Self(slice.clone())
    }

    #[inline(always)]
    pub fn from_translation<V: AsVec2<T> + ?Sized>(v: &V) -> Self {
        Self([
            T::one(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
            T::zero(),
            v.x(),
            v.y(),
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_scaling<V: AsVec2<T> + ?Sized>(v: &V) -> Self {
        Self([
            v.x(),
            T::zero(),
            T::zero(),
            T::zero(),
            v.y(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_rotation(rad: T) -> Self {
        let s = rad.sin();
        let c = rad.cos();
        Self([
            c,
            s,
            T::zero(),
            -s,
            c,
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_mat_2d<M: AsMat2d<T> + ?Sized>(a: &M) -> Self {
        Self([
            a.a(),
            a.b(),
            T::zero(),
            a.c(),
            a.d(),
            T::zero(),
            a.tx(),
            a.ty(),
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_quat<Q: AsQuat<T> + ?Sized>(q: &Q) -> Self {
        let x = q.x();
        let y = q.y();
        let z = q.z();
        let w = q.w();
        let x2 = x + x;
        let y2 = y + y;
        let z2 = z + z;

        let xx = x * x2;
        let yx = y * x2;
        let yy = y * y2;
        let zx = z * x2;
        let zy = z * y2;
        let zz = z * z2;
        let wx = w * x2;
        let wy = w * y2;
        let wz = w * z2;

        Self([
            T::one() - yy - zz,
            yx + wz,
            zx - wy,
            yx - wz,
            T::one() - xx - zz,
            zy + wx,
            zx + wy,
            zy - wx,
            T::one() - xx - yy,
        ])
    }

    #[inline(always)]
    pub fn from_projection(width: T, height: T) -> Self {
        Self([
            T::from(2.0).unwrap() / width,
            T::zero(),
            T::zero(),
            T::zero(),
            T::from(-2.0).unwrap() / height,
            T::zero(),
            -T::one(),
            T::one(),
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn normal_matrix_from_mat4<M: AsMat4<T> + ?Sized>(a: &M) -> Result<Self, Error> {
        let a00 = a.m00();
        let a01 = a.m01();
        let a02 = a.m02();
        let a03 = a.m03();
        let a10 = a.m10();
        let a11 = a.m11();
        let a12 = a.m12();
        let a13 = a.m13();
        let a20 = a.m20();
        let a21 = a.m21();
        let a22 = a.m22();
        let a23 = a.m23();
        let a30 = a.m30();
        let a31 = a.m31();
        let a32 = a.m32();
        let a33 = a.m33();

        let b00 = a00 * a11 - a01 * a10;
        let b01 = a00 * a12 - a02 * a10;
        let b02 = a00 * a13 - a03 * a10;
        let b03 = a01 * a12 - a02 * a11;
        let b04 = a01 * a13 - a03 * a11;
        let b05 = a02 * a13 - a03 * a12;
        let b06 = a20 * a31 - a21 * a30;
        let b07 = a20 * a32 - a22 * a30;
        let b08 = a20 * a33 - a23 * a30;
        let b09 = a21 * a32 - a22 * a31;
        let b10 = a21 * a33 - a23 * a31;
        let b11 = a22 * a33 - a23 * a32;

        // Calculate the determinant
        let mut det = b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;

        if det == T::zero() {
            return Err(Error::ZeroDeterminant);
        }
        det = T::one() / det;

        Ok(Self([
            (a11 * b11 - a12 * b10 + a13 * b09) * det,
            (a12 * b08 - a10 * b11 - a13 * b07) * det,
            (a10 * b10 - a11 * b08 + a13 * b06) * det,
            (a02 * b10 - a01 * b11 - a03 * b09) * det,
            (a00 * b11 - a02 * b08 + a03 * b07) * det,
            (a01 * b08 - a00 * b10 - a03 * b06) * det,
            (a31 * b05 - a32 * b04 + a33 * b03) * det,
            (a32 * b02 - a30 * b05 - a33 * b01) * det,
            (a30 * b04 - a31 * b02 + a33 * b00) * det,
        ]))
    }

    #[inline(always)]
    pub fn raw(&self) -> &[T; 9] {
        &self.0
    }
}

impl<T: Float> AsMat3<T> for Mat3<T> {
    #[inline(always)]
    fn m00(&self) -> T {
        self.0[0]
    }

    #[inline(always)]
    fn m01(&self) -> T {
        self.0[1]
    }

    #[inline(always)]
    fn m02(&self) -> T {
        self.0[2]
    }

    #[inline(always)]
    fn m10(&self) -> T {
        self.0[3]
    }

    #[inline(always)]
    fn m11(&self) -> T {
        self.0[4]
    }

    #[inline(always)]
    fn m12(&self) -> T {
        self.0[5]
    }

    #[inline(always)]
    fn m20(&self) -> T {
        self.0[6]
    }

    #[inline(always)]
    fn m21(&self) -> T {
        self.0[7]
    }

    #[inline(always)]
    fn m22(&self) -> T {
        self.0[8]
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
    fn set_m02(&mut self, m02: T) -> &mut Self {
        self.0[2] = m02;
        self
    }

    #[inline(always)]
    fn set_m10(&mut self, m10: T) -> &mut Self {
        self.0[3] = m10;
        self
    }

    #[inline(always)]
    fn set_m11(&mut self, m11: T) -> &mut Self {
        self.0[4] = m11;
        self
    }

    #[inline(always)]
    fn set_m12(&mut self, m12: T) -> &mut Self {
        self.0[5] = m12;
        self
    }

    #[inline(always)]
    fn set_m20(&mut self, m20: T) -> &mut Self {
        self.0[6] = m20;
        self
    }

    #[inline(always)]
    fn set_m21(&mut self, m21: T) -> &mut Self {
        self.0[7] = m21;
        self
    }

    #[inline(always)]
    fn set_m22(&mut self, m22: T) -> &mut Self {
        self.0[8] = m22;
        self
    }
}

impl<T: Float> Add<Mat3<T>> for Mat3<T> {
    type Output = Self;

    #[inline(always)]
    fn add(self, b: Self) -> Self {
        let mut out = Mat3::<T>::new_identity();
        out.0[0] = self.0[0] + b.0[0];
        out.0[1] = self.0[1] + b.0[1];
        out.0[2] = self.0[2] + b.0[2];
        out.0[3] = self.0[3] + b.0[3];
        out.0[4] = self.0[4] + b.0[4];
        out.0[5] = self.0[5] + b.0[5];
        out.0[6] = self.0[6] + b.0[6];
        out.0[7] = self.0[7] + b.0[7];
        out.0[8] = self.0[8] + b.0[8];
        out
    }
}

impl<T: Float> Sub<Mat3<T>> for Mat3<T> {
    type Output = Self;

    #[inline(always)]
    fn sub(self, b: Self) -> Self {
        let mut out = Mat3::<T>::new_identity();
        out.0[0] = self.0[0] - b.0[0];
        out.0[1] = self.0[1] - b.0[1];
        out.0[2] = self.0[2] - b.0[2];
        out.0[3] = self.0[3] - b.0[3];
        out.0[4] = self.0[4] - b.0[4];
        out.0[5] = self.0[5] - b.0[5];
        out.0[6] = self.0[6] - b.0[6];
        out.0[7] = self.0[7] - b.0[7];
        out.0[8] = self.0[8] - b.0[8];
        out
    }
}

impl<T: Float> Mul<Mat3<T>> for Mat3<T> {
    type Output = Self;

    #[inline(always)]
    fn mul(self, b: Self) -> Self {
        let mut out = Mat3::<T>::new_identity();
        let a00 = self.0[0];
        let a01 = self.0[1];
        let a02 = self.0[2];
        let a10 = self.0[3];
        let a11 = self.0[4];
        let a12 = self.0[5];
        let a20 = self.0[6];
        let a21 = self.0[7];
        let a22 = self.0[8];

        let b00 = b.0[0];
        let b01 = b.0[1];
        let b02 = b.0[2];
        let b10 = b.0[3];
        let b11 = b.0[4];
        let b12 = b.0[5];
        let b20 = b.0[6];
        let b21 = b.0[7];
        let b22 = b.0[8];

        out.0[0] = b00 * a00 + b01 * a10 + b02 * a20;
        out.0[1] = b00 * a01 + b01 * a11 + b02 * a21;
        out.0[2] = b00 * a02 + b01 * a12 + b02 * a22;

        out.0[3] = b10 * a00 + b11 * a10 + b12 * a20;
        out.0[4] = b10 * a01 + b11 * a11 + b12 * a21;
        out.0[5] = b10 * a02 + b11 * a12 + b12 * a22;

        out.0[6] = b20 * a00 + b21 * a10 + b22 * a20;
        out.0[7] = b20 * a01 + b21 * a11 + b22 * a21;
        out.0[8] = b20 * a02 + b21 * a12 + b22 * a22;
        out
    }
}

impl<T: Float> Mul<T> for Mat3<T> {
    type Output = Self;

    #[inline(always)]
    fn mul(self, b: T) -> Self {
        let mut out = Mat3::<T>::new_identity();
        out.0[0] = self.0[0] * b;
        out.0[1] = self.0[1] * b;
        out.0[2] = self.0[2] * b;
        out.0[3] = self.0[3] * b;
        out.0[4] = self.0[4] * b;
        out.0[5] = self.0[5] * b;
        out.0[6] = self.0[6] * b;
        out.0[7] = self.0[7] * b;
        out.0[8] = self.0[8] * b;
        out
    }
}

impl<T> AsRef<Mat3<T>> for Mat3<T> {
    fn as_ref(&self) -> &Self {
        self
    }
}

impl<T> AsRef<[T]> for Mat3<T> {
    fn as_ref(&self) -> &[T] {
        &self.0
    }
}

impl<T: Float> Default for Mat3<T> {
    fn default() -> Self {
        Self::new_identity()
    }
}

impl<T: Display> Display for Mat3<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let value = self
            .0
            .iter()
            .map(|v| v.to_string())
            .collect::<Vec<_>>()
            .join(", ");
        f.write_fmt(format_args!("mat3({})", value))
    }
}

#[cfg(test)]
mod tests {
    use std::sync::OnceLock;

    use crate::{
        error::Error,
        mat3::AsMat3,
        mat4::{AsMat4, Mat4},
        quat::Quat, vec3::AsVec3,
    };

    use super::Mat3;

    static MAT_A_RAW: [f64; 9] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 1.0];
    static MAT_B_RAW: [f64; 9] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 3.0, 4.0, 1.0];

    static MAT_A: OnceLock<Mat3> = OnceLock::new();
    static MAT_B: OnceLock<Mat3> = OnceLock::new();

    fn mat_a() -> &'static Mat3 {
        MAT_A.get_or_init(|| Mat3::from_slice(&MAT_A_RAW))
    }

    fn mat_b() -> &'static Mat3 {
        MAT_B.get_or_init(|| Mat3::from_slice(&MAT_B_RAW))
    }

    #[test]
    fn new() {
        assert_eq!(
            Mat3::<f64>::new().to_raw(),
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        );
    }

    #[test]
    fn new_identity() {
        assert_eq!(
            Mat3::<f64>::new_identity().to_raw(),
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        );
    }

    #[test]
    fn from_slice() {
        assert_eq!(
            Mat3::from_slice(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0,]).to_raw(),
            [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0,]
        );
    }

    #[test]
    fn from_values() {
        assert_eq!(
            Mat3::from_values(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0,).to_raw(),
            [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0,]
        );
    }

    #[test]
    fn from_quat() {
        let q = Quat::from_values(0.0, -0.7071067811865475, 0.0, 0.7071067811865475);
        let m = Mat3::from_quat(&q);

        assert_eq!(
            [0.0, 0.0, -1.0].transform_mat3(&m).to_raw(),
            [0.0, 0.0, -1.0].transform_quat(&q).to_raw(),
        );

        assert_eq!(
            [0.0, 0.0, -1.0].transform_mat3(&m).to_raw(),
            [0.9999999999999998, 0.0, -2.220446049250313e-16],
        );
    }

    #[test]
    fn from_mat4() {
        let mat4 = Mat4::from_values(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );

        assert_eq!(
            Mat3::from_mat4(&mat4).to_raw(),
            [1.0, 2.0, 3.0, 5.0, 6.0, 7.0, 9.0, 10.0, 11.0,]
        );
    }

    #[test]
    fn transpose() {
        assert_eq!(
            mat_a().clone().transpose().to_raw(),
            [1.0, 0.0, 1.0, 0.0, 1.0, 2.0, 0.0, 0.0, 1.0]
        );
    }

    #[test]
    fn invert() -> Result<(), Error> {
        assert_eq!(
            mat_a().clone().invert()?.to_raw(),
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, -2.0, 1.0]
        );

        Ok(())
    }

    #[test]
    fn adjoint() {
        assert_eq!(
            mat_a().clone().adjoint().to_raw(),
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, -2.0, 1.0]
        );
    }

    #[test]
    fn determinant() {
        assert_eq!(mat_a().determinant(), 1.0);
    }

    #[test]
    fn frob() {
        assert_eq!(
            mat_a().frob(),
            (1.0f64.powi(2)
                + 0.0f64.powi(2)
                + 0.0f64.powi(2)
                + 0.0f64.powi(2)
                + 0.0f64.powi(2)
                + 1.0f64.powi(2)
                + 0.0f64.powi(2)
                + 1.0f64.powi(2)
                + 2.0f64.powi(2)
                + 1.0f64.powi(2))
            .sqrt()
        );
    }

    #[test]
    fn from_projection() {
        assert_eq!(
            Mat3::from_projection(100.0, 200.0).to_raw(),
            [0.02, 0.0, 0.0, 0.0, -0.01, 0.0, -1.0, 1.0, 1.0,]
        );
    }

    #[test]
    fn set() {
        let mut mat = Mat3::new();
        mat.set(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);

        assert_eq!(mat.to_raw(), [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
    }

    #[test]
    fn set_slice() {
        let mut mat = Mat3::new();
        mat.set_slice(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);

        assert_eq!(mat.to_raw(), [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
    }

    #[test]
    fn add() {
        let mat_a = Mat3::from_values(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let mat_b = Mat3::from_values(10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0);

        assert_eq!(
            (mat_a + mat_b).to_raw(),
            [11.0, 13.0, 15.0, 17.0, 19.0, 21.0, 23.0, 25.0, 27.0]
        );
    }

    #[test]
    fn sub() {
        let mat_a = Mat3::from_values(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let mat_b = Mat3::from_values(10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0);

        assert_eq!(
            (mat_a - mat_b).to_raw(),
            [-9.0, -9.0, -9.0, -9.0, -9.0, -9.0, -9.0, -9.0, -9.0]
        );
    }

    #[test]
    fn mul() {
        let out = *mat_a() * *mat_b();
        assert_eq!(out.to_raw(), [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 4.0, 6.0, 1.0]);
    }

    #[test]
    fn mul_scalar() {
        let mat = Mat3::from_values(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);

        assert_eq!(
            (mat * 2.0).to_raw(),
            [2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0]
        );
    }

    #[test]
    fn mul_scalar_add() {
        let mat_a = Mat3::from_values(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let mat_b = Mat3::from_values(10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0);

        assert_eq!(
            (mat_a + mat_b * 0.5).to_raw(),
            [6.0, 7.5, 9.0, 10.5, 12.0, 13.5, 15.0, 16.5, 18.0]
        );
    }

    #[test]
    fn approximate_eq() {
        let mat_a = Mat3::from_values(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0);
        let mat_b = Mat3::from_values(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0);
        let mat_c = Mat3::from_values(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let mat_d = Mat3::from_values(1e-16, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0);

        assert_eq!(true, mat_a.approximate_eq(&mat_b));
        assert_eq!(false, mat_a.approximate_eq(&mat_c));
        assert_eq!(true, mat_a.approximate_eq(&mat_d));
    }

    #[test]
    fn display() {
        let out = mat_a().to_string();
        assert_eq!(out, "mat3(1, 0, 0, 0, 1, 0, 1, 2, 1)");
    }
    #[test]
    fn normal_matrix_from_mat4() -> Result<(), Error> {
        let mut mat4 = Mat4::new_identity();
        mat4.translate(&(2.0, 4.0, 6.0))
            .rotate_x(std::f64::consts::PI / 2.0);

        assert_eq!(
            Mat3::normal_matrix_from_mat4(&mat4)?.to_raw(),
            [
                1.0,
                0.0,
                0.0,
                0.0,
                6.123233995736766e-17,
                1.0,
                0.0,
                -1.0,
                6.123233995736766e-17,
            ]
        );

        mat4.scale(&(2.0, 3.0, 4.0));

        assert_eq!(
            Mat3::normal_matrix_from_mat4(&mat4)?.to_raw(),
            [
                0.5,
                0.0,
                0.0,
                0.0,
                2.041077998578922e-17,
                0.3333333333333333,
                0.0,
                -0.25,
                1.5308084989341912e-17,
            ]
        );

        Ok(())
    }
}
