use std::{
    fmt::Display,
    ops::{Add, Mul, Sub},
};

use num_traits::{Float, FloatConst};

use crate::{
    error::Error,
    quat::{AsQuat, Quat},
    quat2::AsQuat2,
    vec3::{AsVec3, Vec3},
};

pub trait AsMat4<T: Float> {
    fn from_values(
        m00: T,
        m01: T,
        m02: T,
        m03: T,
        m10: T,
        m11: T,
        m12: T,
        m13: T,
        m20: T,
        m21: T,
        m22: T,
        m23: T,
        m30: T,
        m31: T,
        m32: T,
        m33: T,
    ) -> Self;

    fn m00(&self) -> T;

    fn m01(&self) -> T;

    fn m02(&self) -> T;

    fn m03(&self) -> T;

    fn m10(&self) -> T;

    fn m11(&self) -> T;

    fn m12(&self) -> T;

    fn m13(&self) -> T;

    fn m20(&self) -> T;

    fn m21(&self) -> T;

    fn m22(&self) -> T;

    fn m23(&self) -> T;

    fn m30(&self) -> T;

    fn m31(&self) -> T;

    fn m32(&self) -> T;

    fn m33(&self) -> T;

    fn set_m00(&mut self, m00: T) -> &mut Self;

    fn set_m01(&mut self, m01: T) -> &mut Self;

    fn set_m02(&mut self, m02: T) -> &mut Self;

    fn set_m03(&mut self, m03: T) -> &mut Self;

    fn set_m10(&mut self, m10: T) -> &mut Self;

    fn set_m11(&mut self, m11: T) -> &mut Self;

    fn set_m12(&mut self, m12: T) -> &mut Self;

    fn set_m13(&mut self, m13: T) -> &mut Self;

    fn set_m20(&mut self, m20: T) -> &mut Self;

    fn set_m21(&mut self, m21: T) -> &mut Self;

    fn set_m22(&mut self, m22: T) -> &mut Self;

    fn set_m23(&mut self, m23: T) -> &mut Self;

    fn set_m30(&mut self, m30: T) -> &mut Self;

    fn set_m31(&mut self, m31: T) -> &mut Self;

    fn set_m32(&mut self, m32: T) -> &mut Self;

    fn set_m33(&mut self, m33: T) -> &mut Self;

    #[inline(always)]
    fn to_raw(&self) -> [T; 16] {
        [
            self.m00(),
            self.m01(),
            self.m02(),
            self.m03(),
            self.m10(),
            self.m11(),
            self.m12(),
            self.m13(),
            self.m20(),
            self.m21(),
            self.m22(),
            self.m23(),
            self.m30(),
            self.m31(),
            self.m32(),
            self.m33(),
        ]
    }

    #[inline(always)]
    fn to_gl(&self) -> [f32; 16] {
        [
            T::to_f32(&self.m00()).unwrap(),
            T::to_f32(&self.m01()).unwrap(),
            T::to_f32(&self.m02()).unwrap(),
            T::to_f32(&self.m03()).unwrap(),
            T::to_f32(&self.m10()).unwrap(),
            T::to_f32(&self.m11()).unwrap(),
            T::to_f32(&self.m12()).unwrap(),
            T::to_f32(&self.m13()).unwrap(),
            T::to_f32(&self.m20()).unwrap(),
            T::to_f32(&self.m21()).unwrap(),
            T::to_f32(&self.m22()).unwrap(),
            T::to_f32(&self.m23()).unwrap(),
            T::to_f32(&self.m30()).unwrap(),
            T::to_f32(&self.m31()).unwrap(),
            T::to_f32(&self.m32()).unwrap(),
            T::to_f32(&self.m33()).unwrap(),
        ]
    }

    #[inline(always)]
    fn to_gl_binary(&self) -> [u8; 64] {
        unsafe { std::mem::transmute_copy::<[f32; 16], [u8; 64]>(&self.to_gl()) }
    }

    #[inline(always)]
    fn copy<M: AsMat4<T> + ?Sized>(&mut self, b: &M) -> &mut Self {
        self.set_m00(b.m00())
            .set_m01(b.m01())
            .set_m02(b.m02())
            .set_m03(b.m03())
            .set_m10(b.m10())
            .set_m11(b.m11())
            .set_m12(b.m12())
            .set_m13(b.m13())
            .set_m20(b.m20())
            .set_m21(b.m21())
            .set_m22(b.m22())
            .set_m23(b.m23())
            .set_m30(b.m30())
            .set_m31(b.m31())
            .set_m32(b.m32())
            .set_m33(b.m33())
    }

    #[inline(always)]
    fn set(
        &mut self,
        m00: T,
        m01: T,
        m02: T,
        m03: T,
        m10: T,
        m11: T,
        m12: T,
        m13: T,
        m20: T,
        m21: T,
        m22: T,
        m23: T,
        m30: T,
        m31: T,
        m32: T,
        m33: T,
    ) -> &mut Self {
        self.set_m00(m00)
            .set_m01(m01)
            .set_m02(m02)
            .set_m03(m03)
            .set_m10(m10)
            .set_m11(m11)
            .set_m12(m12)
            .set_m13(m13)
            .set_m20(m20)
            .set_m21(m21)
            .set_m22(m22)
            .set_m23(m23)
            .set_m30(m30)
            .set_m31(m31)
            .set_m32(m32)
            .set_m33(m33)
    }

    #[inline(always)]
    fn set_slice(&mut self, slice: &[T; 16]) -> &mut Self {
        let [m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33] =
            slice;

        self.set_m00(*m00)
            .set_m01(*m01)
            .set_m02(*m02)
            .set_m03(*m03)
            .set_m10(*m10)
            .set_m11(*m11)
            .set_m12(*m12)
            .set_m13(*m13)
            .set_m20(*m20)
            .set_m21(*m21)
            .set_m22(*m22)
            .set_m23(*m23)
            .set_m30(*m30)
            .set_m31(*m31)
            .set_m32(*m32)
            .set_m33(*m33)
    }

    #[inline(always)]
    fn set_zero(&mut self) -> &mut Self {
        self.set_m00(T::zero())
            .set_m01(T::zero())
            .set_m02(T::zero())
            .set_m03(T::zero())
            .set_m10(T::zero())
            .set_m11(T::zero())
            .set_m12(T::zero())
            .set_m13(T::zero())
            .set_m20(T::zero())
            .set_m21(T::zero())
            .set_m22(T::zero())
            .set_m23(T::zero())
            .set_m30(T::zero())
            .set_m31(T::zero())
            .set_m32(T::zero())
            .set_m33(T::zero())
    }

    #[inline(always)]
    fn set_identify(&mut self) -> &mut Self {
        self.set_m00(T::one())
            .set_m01(T::zero())
            .set_m02(T::zero())
            .set_m03(T::zero())
            .set_m10(T::zero())
            .set_m11(T::one())
            .set_m12(T::zero())
            .set_m13(T::zero())
            .set_m20(T::zero())
            .set_m21(T::zero())
            .set_m22(T::one())
            .set_m23(T::zero())
            .set_m30(T::zero())
            .set_m31(T::zero())
            .set_m32(T::zero())
            .set_m33(T::one())
    }

    #[inline(always)]
    fn transpose(&self) -> Self
    where
        Self: Sized,
    {
        let m00 = self.m00();
        let m01 = self.m01();
        let m02 = self.m02();
        let m03 = self.m03();
        let m10 = self.m10();
        let m11 = self.m11();
        let m12 = self.m12();
        let m13 = self.m13();
        let m20 = self.m20();
        let m21 = self.m21();
        let m22 = self.m22();
        let m23 = self.m23();
        let m30 = self.m30();
        let m31 = self.m31();
        let m32 = self.m32();
        let m33 = self.m33();

        Self::from_values(
            m00, m10, m20, m30, m01, m11, m21, m31, m02, m12, m22, m32, m03, m13, m23, m33,
        )
    }

    #[inline(always)]
    fn invert(&self) -> Result<Self, Error>
    where
        Self: Sized,
    {
        let a00 = self.m00();
        let a01 = self.m01();
        let a02 = self.m02();
        let a03 = self.m03();
        let a10 = self.m10();
        let a11 = self.m11();
        let a12 = self.m12();
        let a13 = self.m13();
        let a20 = self.m20();
        let a21 = self.m21();
        let a22 = self.m22();
        let a23 = self.m23();
        let a30 = self.m30();
        let a31 = self.m31();
        let a32 = self.m32();
        let a33 = self.m33();

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

        Ok(Self::from_values(
            (a11 * b11 - a12 * b10 + a13 * b09) * det,
            (a02 * b10 - a01 * b11 - a03 * b09) * det,
            (a31 * b05 - a32 * b04 + a33 * b03) * det,
            (a22 * b04 - a21 * b05 - a23 * b03) * det,
            (a12 * b08 - a10 * b11 - a13 * b07) * det,
            (a00 * b11 - a02 * b08 + a03 * b07) * det,
            (a32 * b02 - a30 * b05 - a33 * b01) * det,
            (a20 * b05 - a22 * b02 + a23 * b01) * det,
            (a10 * b10 - a11 * b08 + a13 * b06) * det,
            (a01 * b08 - a00 * b10 - a03 * b06) * det,
            (a30 * b04 - a31 * b02 + a33 * b00) * det,
            (a21 * b02 - a20 * b04 - a23 * b00) * det,
            (a11 * b07 - a10 * b09 - a12 * b06) * det,
            (a00 * b09 - a01 * b07 + a02 * b06) * det,
            (a31 * b01 - a30 * b03 - a32 * b00) * det,
            (a20 * b03 - a21 * b01 + a22 * b00) * det,
        ))
    }

    #[inline(always)]
    fn adjoint(&self) -> Self
    where
        Self: Sized,
    {
        let a00 = self.m00();
        let a01 = self.m01();
        let a02 = self.m02();
        let a03 = self.m03();
        let a10 = self.m10();
        let a11 = self.m11();
        let a12 = self.m12();
        let a13 = self.m13();
        let a20 = self.m20();
        let a21 = self.m21();
        let a22 = self.m22();
        let a23 = self.m23();
        let a30 = self.m30();
        let a31 = self.m31();
        let a32 = self.m32();
        let a33 = self.m33();

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

        Self::from_values(
            a11 * b11 - a12 * b10 + a13 * b09,
            a02 * b10 - a01 * b11 - a03 * b09,
            a31 * b05 - a32 * b04 + a33 * b03,
            a22 * b04 - a21 * b05 - a23 * b03,
            a12 * b08 - a10 * b11 - a13 * b07,
            a00 * b11 - a02 * b08 + a03 * b07,
            a32 * b02 - a30 * b05 - a33 * b01,
            a20 * b05 - a22 * b02 + a23 * b01,
            a10 * b10 - a11 * b08 + a13 * b06,
            a01 * b08 - a00 * b10 - a03 * b06,
            a30 * b04 - a31 * b02 + a33 * b00,
            a21 * b02 - a20 * b04 - a23 * b00,
            a11 * b07 - a10 * b09 - a12 * b06,
            a00 * b09 - a01 * b07 + a02 * b06,
            a31 * b01 - a30 * b03 - a32 * b00,
            a20 * b03 - a21 * b01 + a22 * b00,
        )
    }

    #[inline(always)]
    fn determinant(&self) -> T {
        let a00 = self.m00();
        let a01 = self.m01();
        let a02 = self.m02();
        let a03 = self.m03();
        let a10 = self.m10();
        let a11 = self.m11();
        let a12 = self.m12();
        let a13 = self.m13();
        let a20 = self.m20();
        let a21 = self.m21();
        let a22 = self.m22();
        let a23 = self.m23();
        let a30 = self.m30();
        let a31 = self.m31();
        let a32 = self.m32();
        let a33 = self.m33();

        let b0 = a00 * a11 - a01 * a10;
        let b1 = a00 * a12 - a02 * a10;
        let b2 = a01 * a12 - a02 * a11;
        let b3 = a20 * a31 - a21 * a30;
        let b4 = a20 * a32 - a22 * a30;
        let b5 = a21 * a32 - a22 * a31;
        let b6 = a00 * b5 - a01 * b4 + a02 * b3;
        let b7 = a10 * b5 - a11 * b4 + a12 * b3;
        let b8 = a20 * b2 - a21 * b1 + a22 * b0;
        let b9 = a30 * b2 - a31 * b1 + a32 * b0;

        // Calculate the determinant
        a13 * b6 - a03 * b7 + a33 * b8 - a23 * b9
    }

    #[inline(always)]
    fn translate<V: AsVec3<T> + ?Sized>(&self, v: &V) -> Self
    where
        Self: Sized,
    {
        let x = v.x();
        let y = v.y();
        let z = v.z();
        let a00 = self.m00();
        let a01 = self.m01();
        let a02 = self.m02();
        let a03 = self.m03();
        let a10 = self.m10();
        let a11 = self.m11();
        let a12 = self.m12();
        let a13 = self.m13();
        let a20 = self.m20();
        let a21 = self.m21();
        let a22 = self.m22();
        let a23 = self.m23();
        let a30 = self.m30();
        let a31 = self.m31();
        let a32 = self.m32();
        let a33 = self.m33();

        Self::from_values(
            a00,
            a01,
            a02,
            a03,
            a10,
            a11,
            a12,
            a13,
            a20,
            a21,
            a22,
            a23,
            a00 * x + a10 * y + a20 * z + a30,
            a01 * x + a11 * y + a21 * z + a31,
            a02 * x + a12 * y + a22 * z + a32,
            a03 * x + a13 * y + a23 * z + a33,
        )
    }

    #[inline(always)]
    fn scale<V: AsVec3<T> + ?Sized>(&self, v: &V) -> Self
    where
        Self: Sized,
    {
        let x = v.x();
        let y = v.y();
        let z = v.z();

        let a00 = self.m00();
        let a01 = self.m01();
        let a02 = self.m02();
        let a03 = self.m03();
        let a10 = self.m10();
        let a11 = self.m11();
        let a12 = self.m12();
        let a13 = self.m13();
        let a20 = self.m20();
        let a21 = self.m21();
        let a22 = self.m22();
        let a23 = self.m23();
        let a30 = self.m30();
        let a31 = self.m31();
        let a32 = self.m32();
        let a33 = self.m33();

        Self::from_values(
            a00 * x,
            a01 * x,
            a02 * x,
            a03 * x,
            a10 * y,
            a11 * y,
            a12 * y,
            a13 * y,
            a20 * z,
            a21 * z,
            a22 * z,
            a23 * z,
            a30,
            a31,
            a32,
            a33,
        )
    }

    #[inline(always)]
    fn rotate<V: AsVec3<T> + ?Sized>(&self, axis: &V, rad: T) -> Result<Self, Error>
    where
        Self: Sized,
    {
        let mut x = axis.x();
        let mut y = axis.y();
        let mut z = axis.z();
        let mut len = (x * x + y * y + z * z).sqrt();
        let s;
        let c;
        let t;

        let a00 = self.m00();
        let a01 = self.m01();
        let a02 = self.m02();
        let a03 = self.m03();
        let a10 = self.m10();
        let a11 = self.m11();
        let a12 = self.m12();
        let a13 = self.m13();
        let a20 = self.m20();
        let a21 = self.m21();
        let a22 = self.m22();
        let a23 = self.m23();
        let a30 = self.m30();
        let a31 = self.m31();
        let a32 = self.m32();
        let a33 = self.m33();

        let b00;
        let b01;
        let b02;
        let b10;
        let b11;
        let b12;
        let b20;
        let b21;
        let b22;

        if len < T::from(0.000001).unwrap() {
            return Err(Error::LengthSmallerThanEpsilon);
        }

        len = T::one() / len;
        x = x * len;
        y = y * len;
        z = z * len;

        s = rad.sin();
        c = rad.cos();
        t = T::one() - c;

        // Construct the elements of the rotation matrix
        b00 = x * x * t + c;
        b01 = y * x * t + z * s;
        b02 = z * x * t - y * s;
        b10 = x * y * t - z * s;
        b11 = y * y * t + c;
        b12 = z * y * t + x * s;
        b20 = x * z * t + y * s;
        b21 = y * z * t - x * s;
        b22 = z * z * t + c;

        Ok(Self::from_values(
            a00 * b00 + a10 * b01 + a20 * b02,
            a01 * b00 + a11 * b01 + a21 * b02,
            a02 * b00 + a12 * b01 + a22 * b02,
            a03 * b00 + a13 * b01 + a23 * b02,
            a00 * b10 + a10 * b11 + a20 * b12,
            a01 * b10 + a11 * b11 + a21 * b12,
            a02 * b10 + a12 * b11 + a22 * b12,
            a03 * b10 + a13 * b11 + a23 * b12,
            a00 * b20 + a10 * b21 + a20 * b22,
            a01 * b20 + a11 * b21 + a21 * b22,
            a02 * b20 + a12 * b21 + a22 * b22,
            a03 * b20 + a13 * b21 + a23 * b22,
            // If the source and destination differ, copy the unchanged last row
            a30,
            a31,
            a32,
            a33,
        ))
    }

    #[inline(always)]
    fn rotate_x(&self, rad: T) -> Self
    where
        Self: Sized,
    {
        let s = rad.sin();
        let c = rad.cos();

        let a00 = self.m00();
        let a01 = self.m01();
        let a02 = self.m02();
        let a03 = self.m03();
        let a10 = self.m10();
        let a11 = self.m11();
        let a12 = self.m12();
        let a13 = self.m13();
        let a20 = self.m20();
        let a21 = self.m21();
        let a22 = self.m22();
        let a23 = self.m23();
        let a30 = self.m30();
        let a31 = self.m31();
        let a32 = self.m32();
        let a33 = self.m33();

        // Perform axis-specific matrix multiplication
        Self::from_values(
            a00,
            a01,
            a02,
            a03,
            a10 * c + a20 * s,
            a11 * c + a21 * s,
            a12 * c + a22 * s,
            a13 * c + a23 * s,
            a20 * c - a10 * s,
            a21 * c - a11 * s,
            a22 * c - a12 * s,
            a23 * c - a13 * s,
            a30,
            a31,
            a32,
            a33,
        )
    }

    #[inline(always)]
    fn rotate_y(&self, rad: T) -> Self
    where
        Self: Sized,
    {
        let s = rad.sin();
        let c = rad.cos();

        let a00 = self.m00();
        let a01 = self.m01();
        let a02 = self.m02();
        let a03 = self.m03();
        let a10 = self.m10();
        let a11 = self.m11();
        let a12 = self.m12();
        let a13 = self.m13();
        let a20 = self.m20();
        let a21 = self.m21();
        let a22 = self.m22();
        let a23 = self.m23();
        let a30 = self.m30();
        let a31 = self.m31();
        let a32 = self.m32();
        let a33 = self.m33();

        // Perform axis-specific matrix multiplication
        Self::from_values(
            a00 * c - a20 * s,
            a01 * c - a21 * s,
            a02 * c - a22 * s,
            a03 * c - a23 * s,
            a10,
            a11,
            a12,
            a13,
            a00 * s + a20 * c,
            a01 * s + a21 * c,
            a02 * s + a22 * c,
            a03 * s + a23 * c,
            a30,
            a31,
            a32,
            a33,
        )
    }

    #[inline(always)]
    fn rotate_z(&self, rad: T) -> Self
    where
        Self: Sized,
    {
        let s = rad.sin();
        let c = rad.cos();

        let a00 = self.m00();
        let a01 = self.m01();
        let a02 = self.m02();
        let a03 = self.m03();
        let a10 = self.m10();
        let a11 = self.m11();
        let a12 = self.m12();
        let a13 = self.m13();
        let a20 = self.m20();
        let a21 = self.m21();
        let a22 = self.m22();
        let a23 = self.m23();
        let a30 = self.m30();
        let a31 = self.m31();
        let a32 = self.m32();
        let a33 = self.m33();

        // Perform axis-specific matrix multiplication
        Self::from_values(
            a00 * c + a10 * s,
            a01 * c + a11 * s,
            a02 * c + a12 * s,
            a03 * c + a13 * s,
            a10 * c - a00 * s,
            a11 * c - a01 * s,
            a12 * c - a02 * s,
            a13 * c - a03 * s,
            a20,
            a21,
            a22,
            a23,
            a30,
            a31,
            a32,
            a33,
        )
    }

    #[inline(always)]
    fn decompose(&self) -> (Quat<T>, Vec3<T>, Vec3<T>) {
        let mut out_r = Quat::<T>::new();
        let mut out_t = Vec3::<T>::new();
        let mut out_s = Vec3::<T>::new();

        let a00 = self.m00();
        let a01 = self.m01();
        let a02 = self.m02();
        let a10 = self.m10();
        let a11 = self.m11();
        let a12 = self.m12();
        let a20 = self.m20();
        let a21 = self.m21();
        let a22 = self.m22();
        let a30 = self.m30();
        let a31 = self.m31();
        let a32 = self.m32();

        out_t.0[0] = a30;
        out_t.0[1] = a31;
        out_t.0[2] = a32;

        let m11 = a00;
        let m12 = a01;
        let m13 = a02;
        let m21 = a10;
        let m22 = a11;
        let m23 = a12;
        let m31 = a20;
        let m32 = a21;
        let m33 = a22;

        out_s.0[0] = (m11 * m11 + m12 * m12 + m13 * m13).sqrt();
        out_s.0[1] = (m21 * m21 + m22 * m22 + m23 * m23).sqrt();
        out_s.0[2] = (m31 * m31 + m32 * m32 + m33 * m33).sqrt();

        let is1 = T::one() / out_s.0[0];
        let is2 = T::one() / out_s.0[1];
        let is3 = T::one() / out_s.0[2];

        let sm11 = m11 * is1;
        let sm12 = m12 * is2;
        let sm13 = m13 * is3;
        let sm21 = m21 * is1;
        let sm22 = m22 * is2;
        let sm23 = m23 * is3;
        let sm31 = m31 * is1;
        let sm32 = m32 * is2;
        let sm33 = m33 * is3;

        let trace = sm11 + sm22 + sm33;

        if trace > T::zero() {
            let s = (trace + T::one()).sqrt() * T::from(2.0).unwrap();
            out_r.0[3] = T::from(0.25).unwrap() * s;
            out_r.0[0] = (sm23 - sm32) / s;
            out_r.0[1] = (sm31 - sm13) / s;
            out_r.0[2] = (sm12 - sm21) / s;
        } else if sm11 > sm22 && sm11 > sm33 {
            let s = (T::one() + sm11 - sm22 - sm33).sqrt() * T::from(2.0).unwrap();
            out_r.0[3] = (sm23 - sm32) / s;
            out_r.0[0] = T::from(0.25).unwrap() * s;
            out_r.0[1] = (sm12 + sm21) / s;
            out_r.0[2] = (sm31 + sm13) / s;
        } else if sm22 > sm33 {
            let s = (T::one() + sm22 - sm11 - sm33).sqrt() * T::from(2.0).unwrap();
            out_r.0[3] = (sm31 - sm13) / s;
            out_r.0[0] = (sm12 + sm21) / s;
            out_r.0[1] = T::from(0.25).unwrap() * s;
            out_r.0[2] = (sm23 + sm32) / s;
        } else {
            let s = (T::one() + sm33 - sm11 - sm22).sqrt() * T::from(2.0).unwrap();
            out_r.0[3] = (sm12 - sm21) / s;
            out_r.0[0] = (sm31 + sm13) / s;
            out_r.0[1] = (sm23 + sm32) / s;
            out_r.0[2] = T::from(0.25).unwrap() * s;
        }

        (out_r, out_t, out_s)
    }

    #[inline(always)]
    fn frob(&self) -> T {
        let a00 = self.m00();
        let a01 = self.m01();
        let a02 = self.m02();
        let a03 = self.m03();
        let a10 = self.m10();
        let a11 = self.m11();
        let a12 = self.m12();
        let a13 = self.m13();
        let a20 = self.m20();
        let a21 = self.m21();
        let a22 = self.m22();
        let a23 = self.m23();
        let a30 = self.m30();
        let a31 = self.m31();
        let a32 = self.m32();
        let a33 = self.m33();

        (a00 * a00
            + a01 * a01
            + a02 * a02
            + a03 * a03
            + a10 * a10
            + a11 * a11
            + a12 * a12
            + a13 * a13
            + a20 * a20
            + a21 * a21
            + a22 * a22
            + a23 * a23
            + a30 * a30
            + a31 * a31
            + a32 * a32
            + a33 * a33)
            .sqrt()
    }

    #[inline(always)]
    fn translation(&self) -> Vec3<T> {
        Vec3::<T>::from_values(self.m30(), self.m31(), self.m32())
    }

    #[inline(always)]
    fn scaling(&self) -> Vec3<T> {
        let m00 = self.m00();
        let m01 = self.m01();
        let m02 = self.m02();
        let m10 = self.m10();
        let m11 = self.m11();
        let m12 = self.m12();
        let m20 = self.m20();
        let m21 = self.m21();
        let m22 = self.m22();

        Vec3::<T>::from_values(
            (m00 * m00 + m01 * m01 + m02 * m02).sqrt(),
            (m10 * m10 + m11 * m11 + m12 * m12).sqrt(),
            (m20 * m20 + m21 * m21 + m22 * m22).sqrt(),
        )
    }

    #[inline(always)]
    fn rotation(&self) -> Quat<T> {
        let mut out = Quat::<T>::new();

        let scaling = self.scaling();

        let is1 = T::one() / scaling.0[0];
        let is2 = T::one() / scaling.0[1];
        let is3 = T::one() / scaling.0[2];

        let sm11 = self.m11() * is1;
        let sm12 = self.m12() * is2;
        let sm13 = self.m13() * is3;
        let sm21 = self.m21() * is1;
        let sm22 = self.m22() * is2;
        let sm23 = self.m23() * is3;
        let sm31 = self.m31() * is1;
        let sm32 = self.m32() * is2;
        let sm33 = self.m33() * is3;

        let trace = sm11 + sm22 + sm33;

        if trace > T::zero() {
            let s = (trace + T::one()).sqrt() * T::from(2.0).unwrap();
            out.0[3] = T::from(0.25).unwrap() * s;
            out.0[0] = (sm23 - sm32) / s;
            out.0[1] = (sm31 - sm13) / s;
            out.0[2] = (sm12 - sm21) / s;
        } else if sm11 > sm22 && sm11 > sm33 {
            let s = (T::one() + sm11 - sm22 - sm33).sqrt() * T::from(2.0).unwrap();
            out.0[3] = (sm23 - sm32) / s;
            out.0[0] = T::from(0.25).unwrap() * s;
            out.0[1] = (sm12 + sm21) / s;
            out.0[2] = (sm31 + sm13) / s;
        } else if sm22 > sm33 {
            let s = (T::one() + sm22 - sm11 - sm33).sqrt() * T::from(2.0).unwrap();
            out.0[3] = (sm31 - sm13) / s;
            out.0[0] = (sm12 + sm21) / s;
            out.0[1] = T::from(0.25).unwrap() * s;
            out.0[2] = (sm23 + sm32) / s;
        } else {
            let s = (T::one() + sm33 - sm11 - sm22).sqrt() * T::from(2.0).unwrap();
            out.0[3] = (sm12 - sm21) / s;
            out.0[0] = (sm31 + sm13) / s;
            out.0[1] = (sm23 + sm32) / s;
            out.0[2] = T::from(0.25).unwrap() * s;
        }

        out
    }

    /// Returns whether or not the matrices have approximately the same elements in the same position.
    ///
    /// Refers to `equals` function in `glMatrix`. `exactEquals` is implemented with [`PartialEq`] and [`Eq`],
    #[inline(always)]
    fn approximate_eq<V: AsMat4<T> + ?Sized>(&self, b: &V) -> bool {
        let a0 = self.m00();
        let a1 = self.m01();
        let a2 = self.m02();
        let a3 = self.m03();
        let a4 = self.m10();
        let a5 = self.m11();
        let a6 = self.m12();
        let a7 = self.m13();
        let a8 = self.m20();
        let a9 = self.m21();
        let a10 = self.m22();
        let a11 = self.m23();
        let a12 = self.m30();
        let a13 = self.m31();
        let a14 = self.m32();
        let a15 = self.m33();

        let b0 = b.m00();
        let b1 = b.m01();
        let b2 = b.m02();
        let b3 = b.m03();
        let b4 = b.m10();
        let b5 = b.m11();
        let b6 = b.m12();
        let b7 = b.m13();
        let b8 = b.m20();
        let b9 = b.m21();
        let b10 = b.m22();
        let b11 = b.m23();
        let b12 = b.m30();
        let b13 = b.m31();
        let b14 = b.m32();
        let b15 = b.m33();

        (a0 - b0).abs() <= T::from(0.000001).unwrap() * T::one().max(a0.abs()).max(b0.abs())
            && (a1 - b1).abs() <= T::from(0.000001).unwrap() * T::one().max(a1.abs()).max(b1.abs())
            && (a2 - b2).abs() <= T::from(0.000001).unwrap() * T::one().max(a2.abs()).max(b2.abs())
            && (a3 - b3).abs() <= T::from(0.000001).unwrap() * T::one().max(a3.abs()).max(b3.abs())
            && (a4 - b4).abs() <= T::from(0.000001).unwrap() * T::one().max(a4.abs()).max(b4.abs())
            && (a5 - b5).abs() <= T::from(0.000001).unwrap() * T::one().max(a5.abs()).max(b5.abs())
            && (a6 - b6).abs() <= T::from(0.000001).unwrap() * T::one().max(a6.abs()).max(b6.abs())
            && (a7 - b7).abs() <= T::from(0.000001).unwrap() * T::one().max(a7.abs()).max(b7.abs())
            && (a8 - b8).abs() <= T::from(0.000001).unwrap() * T::one().max(a8.abs()).max(b8.abs())
            && (a9 - b9).abs() <= T::from(0.000001).unwrap() * T::one().max(a9.abs()).max(b9.abs())
            && (a10 - b10).abs()
                <= T::from(0.000001).unwrap() * T::one().max(a10.abs()).max(b10.abs())
            && (a11 - b11).abs()
                <= T::from(0.000001).unwrap() * T::one().max(a11.abs()).max(b11.abs())
            && (a12 - b12).abs()
                <= T::from(0.000001).unwrap() * T::one().max(a12.abs()).max(b12.abs())
            && (a13 - b13).abs()
                <= T::from(0.000001).unwrap() * T::one().max(a13.abs()).max(b13.abs())
            && (a14 - b14).abs()
                <= T::from(0.000001).unwrap() * T::one().max(a14.abs()).max(b14.abs())
            && (a15 - b15).abs()
                <= T::from(0.000001).unwrap() * T::one().max(a15.abs()).max(b15.abs())
    }
}

impl<T: Float> AsMat4<T> for [T; 16] {
    #[inline(always)]
    fn from_values(
        m00: T,
        m01: T,
        m02: T,
        m03: T,
        m10: T,
        m11: T,
        m12: T,
        m13: T,
        m20: T,
        m21: T,
        m22: T,
        m23: T,
        m30: T,
        m31: T,
        m32: T,
        m33: T,
    ) -> Self {
        [
            m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33,
        ]
    }

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
    fn m03(&self) -> T {
        self[3]
    }

    #[inline(always)]
    fn m10(&self) -> T {
        self[4]
    }

    #[inline(always)]
    fn m11(&self) -> T {
        self[5]
    }

    #[inline(always)]
    fn m12(&self) -> T {
        self[6]
    }

    #[inline(always)]
    fn m13(&self) -> T {
        self[7]
    }

    #[inline(always)]
    fn m20(&self) -> T {
        self[8]
    }

    #[inline(always)]
    fn m21(&self) -> T {
        self[9]
    }

    #[inline(always)]
    fn m22(&self) -> T {
        self[10]
    }

    #[inline(always)]
    fn m23(&self) -> T {
        self[11]
    }

    #[inline(always)]
    fn m30(&self) -> T {
        self[12]
    }

    #[inline(always)]
    fn m31(&self) -> T {
        self[13]
    }

    #[inline(always)]
    fn m32(&self) -> T {
        self[14]
    }

    #[inline(always)]
    fn m33(&self) -> T {
        self[15]
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
    fn set_m03(&mut self, m03: T) -> &mut Self {
        self[3] = m03;
        self
    }

    #[inline(always)]
    fn set_m10(&mut self, m10: T) -> &mut Self {
        self[4] = m10;
        self
    }

    #[inline(always)]
    fn set_m11(&mut self, m11: T) -> &mut Self {
        self[5] = m11;
        self
    }

    #[inline(always)]
    fn set_m12(&mut self, m12: T) -> &mut Self {
        self[6] = m12;
        self
    }

    #[inline(always)]
    fn set_m13(&mut self, m13: T) -> &mut Self {
        self[7] = m13;
        self
    }

    #[inline(always)]
    fn set_m20(&mut self, m20: T) -> &mut Self {
        self[8] = m20;
        self
    }

    #[inline(always)]
    fn set_m21(&mut self, m21: T) -> &mut Self {
        self[9] = m21;
        self
    }

    #[inline(always)]
    fn set_m22(&mut self, m22: T) -> &mut Self {
        self[10] = m22;
        self
    }

    #[inline(always)]
    fn set_m23(&mut self, m23: T) -> &mut Self {
        self[11] = m23;
        self
    }

    #[inline(always)]
    fn set_m30(&mut self, m30: T) -> &mut Self {
        self[12] = m30;
        self
    }

    #[inline(always)]
    fn set_m31(&mut self, m31: T) -> &mut Self {
        self[13] = m31;
        self
    }

    #[inline(always)]
    fn set_m32(&mut self, m32: T) -> &mut Self {
        self[14] = m32;
        self
    }

    #[inline(always)]
    fn set_m33(&mut self, m33: T) -> &mut Self {
        self[15] = m33;
        self
    }
}

impl<T: Float> AsMat4<T> for (T, T, T, T, T, T, T, T, T, T, T, T, T, T, T, T) {
    #[inline(always)]
    fn from_values(
        m00: T,
        m01: T,
        m02: T,
        m03: T,
        m10: T,
        m11: T,
        m12: T,
        m13: T,
        m20: T,
        m21: T,
        m22: T,
        m23: T,
        m30: T,
        m31: T,
        m32: T,
        m33: T,
    ) -> Self {
        (
            m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33,
        )
    }

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
    fn m03(&self) -> T {
        self.3
    }

    #[inline(always)]
    fn m10(&self) -> T {
        self.4
    }

    #[inline(always)]
    fn m11(&self) -> T {
        self.5
    }

    #[inline(always)]
    fn m12(&self) -> T {
        self.6
    }

    #[inline(always)]
    fn m13(&self) -> T {
        self.7
    }

    #[inline(always)]
    fn m20(&self) -> T {
        self.8
    }

    #[inline(always)]
    fn m21(&self) -> T {
        self.9
    }

    #[inline(always)]
    fn m22(&self) -> T {
        self.10
    }

    #[inline(always)]
    fn m23(&self) -> T {
        self.11
    }

    #[inline(always)]
    fn m30(&self) -> T {
        self.12
    }

    #[inline(always)]
    fn m31(&self) -> T {
        self.13
    }

    #[inline(always)]
    fn m32(&self) -> T {
        self.14
    }

    #[inline(always)]
    fn m33(&self) -> T {
        self.15
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
    fn set_m03(&mut self, m03: T) -> &mut Self {
        self.3 = m03;
        self
    }

    #[inline(always)]
    fn set_m10(&mut self, m10: T) -> &mut Self {
        self.4 = m10;
        self
    }

    #[inline(always)]
    fn set_m11(&mut self, m11: T) -> &mut Self {
        self.5 = m11;
        self
    }

    #[inline(always)]
    fn set_m12(&mut self, m12: T) -> &mut Self {
        self.6 = m12;
        self
    }

    #[inline(always)]
    fn set_m13(&mut self, m13: T) -> &mut Self {
        self.7 = m13;
        self
    }

    #[inline(always)]
    fn set_m20(&mut self, m20: T) -> &mut Self {
        self.8 = m20;
        self
    }

    #[inline(always)]
    fn set_m21(&mut self, m21: T) -> &mut Self {
        self.9 = m21;
        self
    }

    #[inline(always)]
    fn set_m22(&mut self, m22: T) -> &mut Self {
        self.10 = m22;
        self
    }

    #[inline(always)]
    fn set_m23(&mut self, m23: T) -> &mut Self {
        self.11 = m23;
        self
    }

    #[inline(always)]
    fn set_m30(&mut self, m30: T) -> &mut Self {
        self.12 = m30;
        self
    }

    #[inline(always)]
    fn set_m31(&mut self, m31: T) -> &mut Self {
        self.13 = m31;
        self
    }

    #[inline(always)]
    fn set_m32(&mut self, m32: T) -> &mut Self {
        self.14 = m32;
        self
    }

    #[inline(always)]
    fn set_m33(&mut self, m33: T) -> &mut Self {
        self.15 = m33;
        self
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mat4<T = f64>(pub [T; 16]);

impl<T: Float> Mat4<T> {
    #[inline(always)]
    pub fn new() -> Self {
        Self([T::zero(); 16])
    }

    #[inline(always)]
    pub fn new_identity() -> Self {
        Self([
            T::one(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_slice(slice: &[T; 16]) -> Self {
        Self(slice.clone())
    }

    #[inline(always)]
    pub fn from_translation<V: AsVec3<T> + ?Sized>(v: &V) -> Self {
        Self([
            T::one(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
            T::zero(),
            v.x(),
            v.y(),
            v.z(),
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_scaling<V: AsVec3<T> + ?Sized>(v: &V) -> Self {
        Self([
            v.x(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            v.y(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            v.z(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_rotation<V: AsVec3<T> + ?Sized>(rad: T, axis: &V) -> Result<Self, Error> {
        let mut x = axis.x();
        let mut y = axis.y();
        let mut z = axis.z();
        let mut len = (x * x + y * y + z * z).sqrt();
        let s;
        let c;
        let t;

        if len < T::from(0.000001).unwrap() {
            return Err(Error::LengthSmallerThanEpsilon);
        }

        len = T::one() / len;
        x = x * len;
        y = y * len;
        z = z * len;

        s = rad.sin();
        c = rad.cos();
        t = T::one() - c;

        // Perform rotation-specific matrix multiplication
        Ok(Self([
            x * x * t + c,
            y * x * t + z * s,
            z * x * t - y * s,
            T::zero(),
            x * y * t - z * s,
            y * y * t + c,
            z * y * t + x * s,
            T::zero(),
            x * z * t + y * s,
            y * z * t - x * s,
            z * z * t + c,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
        ]))
    }

    #[inline(always)]
    pub fn from_x_rotation(rad: T) -> Self {
        let s = rad.sin();
        let c = rad.cos();

        // Perform axis-specific matrix multiplication
        Self([
            T::one(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            c,
            s,
            T::zero(),
            T::zero(),
            -s,
            c,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_y_rotation(rad: T) -> Self {
        let s = rad.sin();
        let c = rad.cos();

        // Perform axis-specific matrix multiplication
        Self([
            c,
            T::zero(),
            -s,
            T::zero(),
            T::zero(),
            T::one(),
            T::zero(),
            T::zero(),
            s,
            T::zero(),
            c,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_z_rotation(rad: T) -> Self {
        let s = rad.sin();
        let c = rad.cos();

        // Perform axis-specific matrix multiplication
        Self([
            c,
            s,
            T::zero(),
            T::zero(),
            -s,
            c,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_rotation_translation<Q, V>(q: &Q, v: &V) -> Self
    where
        Q: AsQuat2<T> + ?Sized,
        V: AsVec3<T> + ?Sized,
    {
        // Quaternion math
        let x = q.x1();
        let y = q.y1();
        let z = q.z1();
        let w = q.w1();
        let x2 = x + x;
        let y2 = y + y;
        let z2 = z + z;

        let xx = x * x2;
        let xy = x * y2;
        let xz = x * z2;
        let yy = y * y2;
        let yz = y * z2;
        let zz = z * z2;
        let wx = w * x2;
        let wy = w * y2;
        let wz = w * z2;

        Self([
            T::one() - (yy + zz),
            xy + wz,
            xz - wy,
            T::zero(),
            xy - wz,
            T::one() - (xx + zz),
            yz + wx,
            T::zero(),
            xz + wy,
            yz - wx,
            T::one() - (xx + yy),
            T::zero(),
            v.x(),
            v.y(),
            v.z(),
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_quat2<Q: AsQuat2<T> + ?Sized>(a: &Q) -> Self {
        let mut translation = Vec3::<T>::new();
        let bx = -a.x1();
        let by = -a.y1();
        let bz = -a.z1();
        let bw = a.w1();
        let ax = a.x2();
        let ay = a.y2();
        let az = a.z2();
        let aw = a.w2();

        let magnitude = bx * bx + by * by + bz * bz + bw * bw;
        //Only scale if it makes sense
        if magnitude > T::zero() {
            translation.0[0] =
                ((ax * bw + aw * bx + ay * bz - az * by) * T::from(2.0).unwrap()) / magnitude;
            translation.0[1] =
                ((ay * bw + aw * by + az * bx - ax * bz) * T::from(2.0).unwrap()) / magnitude;
            translation.0[2] =
                ((az * bw + aw * bz + ax * by - ay * bx) * T::from(2.0).unwrap()) / magnitude;
        } else {
            translation.0[0] = (ax * bw + aw * bx + ay * bz - az * by) * T::from(2.0).unwrap();
            translation.0[1] = (ay * bw + aw * by + az * bx - ax * bz) * T::from(2.0).unwrap();
            translation.0[2] = (az * bw + aw * bz + ax * by - ay * bx) * T::from(2.0).unwrap();
        }
        Self::from_rotation_translation(a, &translation)
    }

    #[inline(always)]
    pub fn from_rotation_translation_scale<Q, V1, V2>(q: &Q, v: &V1, s: &V2) -> Self
    where
        Q: AsQuat2<T> + ?Sized,
        V1: AsVec3<T> + ?Sized,
        V2: AsVec3<T> + ?Sized,
    {
        // Quaternion math
        let x = q.x1();
        let y = q.y1();
        let z = q.z1();
        let w = q.w1();
        let x2 = x + x;
        let y2 = y + y;
        let z2 = z + z;

        let xx = x * x2;
        let xy = x * y2;
        let xz = x * z2;
        let yy = y * y2;
        let yz = y * z2;
        let zz = z * z2;
        let wx = w * x2;
        let wy = w * y2;
        let wz = w * z2;
        let sx = s.x();
        let sy = s.y();
        let sz = s.z();

        Self([
            (T::one() - (yy + zz)) * sx,
            (xy + wz) * sx,
            (xz - wy) * sx,
            T::zero(),
            (xy - wz) * sy,
            (T::one() - (xx + zz)) * sy,
            (yz + wx) * sy,
            T::zero(),
            (xz + wy) * sz,
            (yz - wx) * sz,
            (T::one() - (xx + yy)) * sz,
            T::zero(),
            v.x(),
            v.y(),
            v.z(),
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_rotation_translation_scale_origin<Q, V1, V2, V3>(
        q: &Q,
        v: &V1,
        s: &V2,
        o: &V3,
    ) -> Self
    where
        Q: AsQuat2<T> + ?Sized,
        V1: AsVec3<T> + ?Sized,
        V2: AsVec3<T> + ?Sized,
        V3: AsVec3<T> + ?Sized,
    {
        // Quaternion math
        let x = q.x1();
        let y = q.y1();
        let z = q.z1();
        let w = q.w1();
        let x2 = x + x;
        let y2 = y + y;
        let z2 = z + z;

        let xx = x * x2;
        let xy = x * y2;
        let xz = x * z2;
        let yy = y * y2;
        let yz = y * z2;
        let zz = z * z2;
        let wx = w * x2;
        let wy = w * y2;
        let wz = w * z2;

        let sx = s.x();
        let sy = s.y();
        let sz = s.z();

        let ox = o.x();
        let oy = o.y();
        let oz = o.z();

        let out0 = (T::one() - (yy + zz)) * sx;
        let out1 = (xy + wz) * sx;
        let out2 = (xz - wy) * sx;
        let out4 = (xy - wz) * sy;
        let out5 = (T::one() - (xx + zz)) * sy;
        let out6 = (yz + wx) * sy;
        let out8 = (xz + wy) * sz;
        let out9 = (yz - wx) * sz;
        let out10 = (T::one() - (xx + yy)) * sz;

        Self([
            out0,
            out1,
            out2,
            T::zero(),
            out4,
            out5,
            out6,
            T::zero(),
            out8,
            out9,
            out10,
            T::zero(),
            v.x() + ox - (out0 * ox + out4 * oy + out8 * oz),
            v.y() + oy - (out1 * ox + out5 * oy + out9 * oz),
            v.z() + oz - (out2 * ox + out6 * oy + out10 * oz),
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_quat<Q: AsQuat<T> + ?Sized>(q: &Q) -> Self {
        // Quaternion math
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
            T::zero(),
            yx - wz,
            T::one() - xx - zz,
            zy + wx,
            T::zero(),
            zx + wy,
            zy - wx,
            T::one() - xx - yy,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_frustum(left: T, right: T, bottom: T, top: T, near: T, far: T) -> Self {
        let rl = T::one() / (right - left);
        let tb = T::one() / (top - bottom);
        let nf = T::one() / (near - far);

        Self([
            near * T::from(2.0).unwrap() * rl,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            near * T::from(2.0).unwrap() * tb,
            T::zero(),
            T::zero(),
            (right + left) * rl,
            (top + bottom) * tb,
            (far + near) * nf,
            -T::one(),
            T::zero(),
            T::zero(),
            far * near * T::from(2.0).unwrap() * nf,
            T::zero(),
        ])
    }

    #[inline(always)]
    pub fn from_perspective_no(fovy: T, aspect: T, near: T, far: Option<T>) -> Self {
        let mut out = Self::new();

        let f = T::one() / (fovy / T::from(2.0).unwrap()).tan();
        out.0[0] = f / aspect;
        out.0[1] = T::zero();
        out.0[2] = T::zero();
        out.0[3] = T::zero();
        out.0[4] = T::zero();
        out.0[5] = f;
        out.0[6] = T::zero();
        out.0[7] = T::zero();
        out.0[8] = T::zero();
        out.0[9] = T::zero();
        out.0[11] = -T::one();
        out.0[12] = T::zero();
        out.0[13] = T::zero();
        out.0[15] = T::zero();

        match far {
            Some(far) => {
                if far.is_infinite() {
                    out.0[10] = -T::one();
                    out.0[14] = -T::from(2.0).unwrap() * near;
                    out
                } else {
                    let nf = T::one() / (near - far);
                    out.0[10] = (far + near) * nf;
                    out.0[14] = T::from(2.0).unwrap() * far * near * nf;
                    out
                }
            }
            None => {
                out.0[10] = -T::one();
                out.0[14] = -T::from(2.0).unwrap() * near;
                out
            }
        }
    }

    #[inline(always)]
    pub fn from_perspective(fovy: T, aspect: T, near: T, far: Option<T>) -> Self {
        Self::from_perspective_no(fovy, aspect, near, far)
    }

    #[inline(always)]
    pub fn from_perspective_zo(fovy: T, aspect: T, near: T, far: Option<T>) -> Self {
        let mut out = Self::new();

        let f = T::one() / (fovy / T::from(2.0).unwrap()).tan();
        out.0[0] = f / aspect;
        out.0[1] = T::zero();
        out.0[2] = T::zero();
        out.0[3] = T::zero();
        out.0[4] = T::zero();
        out.0[5] = f;
        out.0[6] = T::zero();
        out.0[7] = T::zero();
        out.0[8] = T::zero();
        out.0[9] = T::zero();
        out.0[11] = -T::one();
        out.0[12] = T::zero();
        out.0[13] = T::zero();
        out.0[15] = T::zero();

        match far {
            Some(far) => {
                if far.is_infinite() {
                    out.0[10] = -T::one();
                    out.0[14] = -near;
                    out
                } else {
                    let nf = T::one() / (near - far);
                    out.0[10] = far * nf;
                    out.0[14] = far * near * nf;
                    out
                }
            }
            None => {
                out.0[10] = -T::one();
                out.0[14] = -near;
                out
            }
        }
    }

    #[inline(always)]
    pub fn from_ortho_no(left: T, right: T, bottom: T, top: T, near: T, far: T) -> Self {
        let lr = T::one() / (left - right);
        let bt = T::one() / (bottom - top);
        let nf = T::one() / (near - far);

        Self([
            -T::from(2.0).unwrap() * lr,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            -T::from(2.0).unwrap() * bt,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::from(2.0).unwrap() * nf,
            T::zero(),
            (left + right) * lr,
            (top + bottom) * bt,
            (far + near) * nf,
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_ortho(left: T, right: T, bottom: T, top: T, near: T, far: T) -> Self {
        Self::from_ortho_no(left, right, bottom, top, near, far)
    }

    #[inline(always)]
    pub fn from_ortho_zo(left: T, right: T, bottom: T, top: T, near: T, far: T) -> Self {
        let lr = T::one() / (left - right);
        let bt = T::one() / (bottom - top);
        let nf = T::one() / (near - far);

        Self([
            -T::from(2.0).unwrap() * lr,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            -T::from(2.0).unwrap() * bt,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            nf,
            T::zero(),
            (left + right) * lr,
            (top + bottom) * bt,
            near * nf,
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_look_at<V1, V2, V3>(eye: &V1, center: &V2, up: &V3) -> Self
    where
        V1: AsVec3<T> + ?Sized,
        V2: AsVec3<T> + ?Sized,
        V3: AsVec3<T> + ?Sized,
    {
        let eye_x = eye.x();
        let eye_y = eye.y();
        let eye_z = eye.z();
        let up_x = up.x();
        let up_y = up.y();
        let up_z = up.z();
        let center_x = center.x();
        let center_y = center.y();
        let center_z = center.z();

        if (eye_x - center_x).abs() < T::from(0.000001).unwrap()
            && (eye_y - center_y).abs() < T::from(0.000001).unwrap()
            && (eye_z - center_z).abs() < T::from(0.000001).unwrap()
        {
            return Self::new();
        }

        let mut z0 = eye_x - center_x;
        let mut z1 = eye_y - center_y;
        let mut z2 = eye_z - center_z;

        let mut len = T::one() / (z0 * z0 + z1 * z1 + z2 * z2).sqrt();
        z0 = z0 * len;
        z1 = z1 * len;
        z2 = z2 * len;

        let mut x0 = up_y * z2 - up_z * z1;
        let mut x1 = up_z * z0 - up_x * z2;
        let mut x2 = up_x * z1 - up_y * z0;
        len = (x0 * x0 + x1 * x1 + x2 * x2).sqrt();
        if len == T::zero() {
            x0 = T::zero();
            x1 = T::zero();
            x2 = T::zero();
        } else {
            len = T::one() / len;
            x0 = x0 * len;
            x1 = x1 * len;
            x2 = x2 * len;
        }

        let mut y0 = z1 * x2 - z2 * x1;
        let mut y1 = z2 * x0 - z0 * x2;
        let mut y2 = z0 * x1 - z1 * x0;

        len = (y0 * y0 + y1 * y1 + y2 * y2).sqrt();
        if len == T::zero() {
            y0 = T::zero();
            y1 = T::zero();
            y2 = T::zero();
        } else {
            len = T::one() / len;
            y0 = y0 * len;
            y1 = y1 * len;
            y2 = y2 * len;
        }

        Self([
            x0,
            y0,
            z0,
            T::zero(),
            x1,
            y1,
            z1,
            T::zero(),
            x2,
            y2,
            z2,
            T::zero(),
            -(x0 * eye_x + x1 * eye_y + x2 * eye_z),
            -(y0 * eye_x + y1 * eye_y + y2 * eye_z),
            -(z0 * eye_x + z1 * eye_y + z2 * eye_z),
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_target_to<V1, V2, V3>(eye: &V1, target: &V2, up: &V3) -> Self
    where
        V1: AsVec3<T> + ?Sized,
        V2: AsVec3<T> + ?Sized,
        V3: AsVec3<T> + ?Sized,
    {
        let eye_x = eye.x();
        let eye_y = eye.y();
        let eye_z = eye.z();
        let up_x = up.x();
        let up_y = up.y();
        let up_z = up.z();

        let mut z0 = eye_x - target.x();
        let mut z1 = eye_y - target.y();
        let mut z2 = eye_z - target.z();

        let mut len = z0 * z0 + z1 * z1 + z2 * z2;
        if len > T::zero() {
            len = T::one() / len.sqrt();
            z0 = z0 * len;
            z1 = z1 * len;
            z2 = z2 * len;
        }

        let mut x0 = up_y * z2 - up_z * z1;
        let mut x1 = up_z * z0 - up_x * z2;
        let mut x2 = up_x * z1 - up_y * z0;

        len = x0 * x0 + x1 * x1 + x2 * x2;
        if len > T::zero() {
            len = T::one() / len.sqrt();
            x0 = x0 * len;
            x1 = x1 * len;
            x2 = x2 * len;
        }

        Self([
            x0,
            x1,
            x2,
            T::zero(),
            z1 * x2 - z2 * x1,
            z2 * x0 - z0 * x2,
            z0 * x1 - z1 * x0,
            T::zero(),
            z0,
            z1,
            z2,
            T::zero(),
            eye_x,
            eye_y,
            eye_z,
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn raw(&self) -> &[T; 16] {
        &self.0
    }
}

impl<T: Float + FloatConst> Mat4<T> {
    #[inline(always)]
    pub fn from_perspective_from_field_of_view(
        fov_left_degrees: T,
        fov_right_degrees: T,
        fov_down_degrees: T,
        fov_up_degrees: T,
        near: T,
        far: T,
    ) -> Self {
        let up_tan = ((fov_up_degrees * T::PI()) / T::from(180.0).unwrap()).tan();
        let down_tan = ((fov_down_degrees * T::PI()) / T::from(180.0).unwrap()).tan();
        let left_tan = ((fov_left_degrees * T::PI()) / T::from(180.0).unwrap()).tan();
        let right_tan = ((fov_right_degrees * T::PI()) / T::from(180.0).unwrap()).tan();
        let x_scale = T::from(2.0).unwrap() / (left_tan + right_tan);
        let y_scale = T::from(2.0).unwrap() / (up_tan + down_tan);

        Self([
            x_scale,
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
            y_scale,
            T::zero(),
            T::zero(),
            -((left_tan - right_tan) * x_scale * T::from(0.5).unwrap()),
            (up_tan - down_tan) * y_scale * T::from(0.5).unwrap(),
            far / (near - far),
            -T::one(),
            T::zero(),
            T::zero(),
            (far * near) / (near - far),
            T::zero(),
        ])
    }
}

impl<T: Float> AsMat4<T> for Mat4<T> {
    #[inline(always)]
    fn from_values(
        m00: T,
        m01: T,
        m02: T,
        m03: T,
        m10: T,
        m11: T,
        m12: T,
        m13: T,
        m20: T,
        m21: T,
        m22: T,
        m23: T,
        m30: T,
        m31: T,
        m32: T,
        m33: T,
    ) -> Self {
        Self([
            m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33,
        ])
    }

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
    fn m03(&self) -> T {
        self.0[3]
    }

    #[inline(always)]
    fn m10(&self) -> T {
        self.0[4]
    }

    #[inline(always)]
    fn m11(&self) -> T {
        self.0[5]
    }

    #[inline(always)]
    fn m12(&self) -> T {
        self.0[6]
    }

    #[inline(always)]
    fn m13(&self) -> T {
        self.0[7]
    }

    #[inline(always)]
    fn m20(&self) -> T {
        self.0[8]
    }

    #[inline(always)]
    fn m21(&self) -> T {
        self.0[9]
    }

    #[inline(always)]
    fn m22(&self) -> T {
        self.0[10]
    }

    #[inline(always)]
    fn m23(&self) -> T {
        self.0[11]
    }

    #[inline(always)]
    fn m30(&self) -> T {
        self.0[12]
    }

    #[inline(always)]
    fn m31(&self) -> T {
        self.0[13]
    }

    #[inline(always)]
    fn m32(&self) -> T {
        self.0[14]
    }

    #[inline(always)]
    fn m33(&self) -> T {
        self.0[15]
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
    fn set_m03(&mut self, m03: T) -> &mut Self {
        self.0[3] = m03;
        self
    }

    #[inline(always)]
    fn set_m10(&mut self, m10: T) -> &mut Self {
        self.0[4] = m10;
        self
    }

    #[inline(always)]
    fn set_m11(&mut self, m11: T) -> &mut Self {
        self.0[5] = m11;
        self
    }

    #[inline(always)]
    fn set_m12(&mut self, m12: T) -> &mut Self {
        self.0[6] = m12;
        self
    }

    #[inline(always)]
    fn set_m13(&mut self, m13: T) -> &mut Self {
        self.0[7] = m13;
        self
    }

    #[inline(always)]
    fn set_m20(&mut self, m20: T) -> &mut Self {
        self.0[8] = m20;
        self
    }

    #[inline(always)]
    fn set_m21(&mut self, m21: T) -> &mut Self {
        self.0[9] = m21;
        self
    }

    #[inline(always)]
    fn set_m22(&mut self, m22: T) -> &mut Self {
        self.0[10] = m22;
        self
    }

    #[inline(always)]
    fn set_m23(&mut self, m23: T) -> &mut Self {
        self.0[11] = m23;
        self
    }

    #[inline(always)]
    fn set_m30(&mut self, m30: T) -> &mut Self {
        self.0[12] = m30;
        self
    }

    #[inline(always)]
    fn set_m31(&mut self, m31: T) -> &mut Self {
        self.0[13] = m31;
        self
    }

    #[inline(always)]
    fn set_m32(&mut self, m32: T) -> &mut Self {
        self.0[14] = m32;
        self
    }

    #[inline(always)]
    fn set_m33(&mut self, m33: T) -> &mut Self {
        self.0[15] = m33;
        self
    }
}

impl<T: Float> Add<Mat4<T>> for Mat4<T> {
    type Output = Self;

    #[inline(always)]
    fn add(self, b: Self) -> Self {
        let mut out = Mat4::<T>::new_identity();
        out.0[0] = self.0[0] + b.0[0];
        out.0[1] = self.0[1] + b.0[1];
        out.0[2] = self.0[2] + b.0[2];
        out.0[3] = self.0[3] + b.0[3];
        out.0[4] = self.0[4] + b.0[4];
        out.0[5] = self.0[5] + b.0[5];
        out.0[6] = self.0[6] + b.0[6];
        out.0[7] = self.0[7] + b.0[7];
        out.0[8] = self.0[8] + b.0[8];
        out.0[9] = self.0[9] + b.0[9];
        out.0[10] = self.0[10] + b.0[10];
        out.0[11] = self.0[11] + b.0[11];
        out.0[12] = self.0[12] + b.0[12];
        out.0[13] = self.0[13] + b.0[13];
        out.0[14] = self.0[14] + b.0[14];
        out.0[15] = self.0[15] + b.0[15];
        out
    }
}

impl<T: Float> Sub<Mat4<T>> for Mat4<T> {
    type Output = Self;

    #[inline(always)]
    fn sub(self, b: Self) -> Self {
        let mut out = Mat4::<T>::new_identity();
        out.0[0] = self.0[0] - b.0[0];
        out.0[1] = self.0[1] - b.0[1];
        out.0[2] = self.0[2] - b.0[2];
        out.0[3] = self.0[3] - b.0[3];
        out.0[4] = self.0[4] - b.0[4];
        out.0[5] = self.0[5] - b.0[5];
        out.0[6] = self.0[6] - b.0[6];
        out.0[7] = self.0[7] - b.0[7];
        out.0[8] = self.0[8] - b.0[8];
        out.0[9] = self.0[9] - b.0[9];
        out.0[10] = self.0[10] - b.0[10];
        out.0[11] = self.0[11] - b.0[11];
        out.0[12] = self.0[12] - b.0[12];
        out.0[13] = self.0[13] - b.0[13];
        out.0[14] = self.0[14] - b.0[14];
        out.0[15] = self.0[15] - b.0[15];
        out
    }
}

impl<T: Float> Mul<Mat4<T>> for Mat4<T> {
    type Output = Self;

    #[inline(always)]
    fn mul(self, b: Self) -> Self {
        let mut out = Mat4::<T>::new_identity();
        let a00 = self.0[0];
        let a01 = self.0[1];
        let a02 = self.0[2];
        let a03 = self.0[3];
        let a10 = self.0[4];
        let a11 = self.0[5];
        let a12 = self.0[6];
        let a13 = self.0[7];
        let a20 = self.0[8];
        let a21 = self.0[9];
        let a22 = self.0[10];
        let a23 = self.0[11];
        let a30 = self.0[12];
        let a31 = self.0[13];
        let a32 = self.0[14];
        let a33 = self.0[15];

        // Cache only the current line of the second matrix
        let mut b0 = b.0[0];
        let mut b1 = b.0[1];
        let mut b2 = b.0[2];
        let mut b3 = b.0[3];
        out.0[0] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
        out.0[1] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
        out.0[2] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
        out.0[3] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;

        b0 = b.0[4];
        b1 = b.0[5];
        b2 = b.0[6];
        b3 = b.0[7];
        out.0[4] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
        out.0[5] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
        out.0[6] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
        out.0[7] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;

        b0 = b.0[8];
        b1 = b.0[9];
        b2 = b.0[10];
        b3 = b.0[11];
        out.0[8] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
        out.0[9] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
        out.0[10] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
        out.0[11] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;

        b0 = b.0[12];
        b1 = b.0[13];
        b2 = b.0[14];
        b3 = b.0[15];
        out.0[12] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
        out.0[13] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
        out.0[14] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
        out.0[15] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;
        out
    }
}

impl<T: Float> Mul<T> for Mat4<T> {
    type Output = Self;

    #[inline(always)]
    fn mul(self, b: T) -> Self {
        let mut out = Mat4::<T>::new_identity();
        out.0[0] = self.0[0] * b;
        out.0[1] = self.0[1] * b;
        out.0[2] = self.0[2] * b;
        out.0[3] = self.0[3] * b;
        out.0[4] = self.0[4] * b;
        out.0[5] = self.0[5] * b;
        out.0[6] = self.0[6] * b;
        out.0[7] = self.0[7] * b;
        out.0[8] = self.0[8] * b;
        out.0[9] = self.0[9] * b;
        out.0[10] = self.0[10] * b;
        out.0[11] = self.0[11] * b;
        out.0[12] = self.0[12] * b;
        out.0[13] = self.0[13] * b;
        out.0[14] = self.0[14] * b;
        out.0[15] = self.0[15] * b;
        out
    }
}

impl<T> AsRef<Mat4<T>> for Mat4<T> {
    fn as_ref(&self) -> &Self {
        self
    }
}

impl<T> AsRef<[T]> for Mat4<T> {
    fn as_ref(&self) -> &[T] {
        &self.0
    }
}

impl<T: Float> Default for Mat4<T> {
    fn default() -> Self {
        Self::new_identity()
    }
}

impl<T: Display> Display for Mat4<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let value = self
            .0
            .iter()
            .map(|v| v.to_string())
            .collect::<Vec<_>>()
            .join(", ");
        f.write_fmt(format_args!("mat4({})", value))
    }
}

#[cfg(test)]
mod tests {
    use std::sync::OnceLock;

    use crate::{
        error::Error,
        mat4::AsMat4,
        vec3::{AsVec3, Vec3},
    };

    use super::Mat4;

    const MAT_A_RAW: [f64; 16] = [
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
    ];
    const MAT_B_RAW: [f64; 16] = [
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 4.0, 5.0, 6.0, 1.0,
    ];
    const MAT_IDENTITY_RAW: [f64; 16] = [
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ];

    static MAT_A: OnceLock<Mat4> = OnceLock::new();
    static MAT_B: OnceLock<Mat4> = OnceLock::new();
    static MAT_IDENTITY: OnceLock<Mat4> = OnceLock::new();

    fn mat_a() -> &'static Mat4 {
        MAT_A.get_or_init(|| Mat4::from_slice(&MAT_A_RAW))
    }

    fn mat_b() -> &'static Mat4 {
        MAT_B.get_or_init(|| Mat4::from_slice(&MAT_B_RAW))
    }

    fn mat_identity() -> &'static Mat4 {
        MAT_IDENTITY.get_or_init(|| Mat4::from_slice(&MAT_IDENTITY_RAW))
    }

    #[test]
    fn new() {
        assert_eq!(
            Mat4::<f64>::new().to_raw(),
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        );
    }

    #[test]
    fn new_identity() {
        assert_eq!(Mat4::<f64>::new_identity().to_raw(), MAT_IDENTITY_RAW);
    }

    #[test]
    fn from_slice() {
        assert_eq!(
            Mat4::from_slice(&[
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ])
            .to_raw(),
            [
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn from_values() {
        assert_eq!(
            Mat4::from_values(
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0,
            )
            .to_raw(),
            [
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn from_frustum() {
        assert_eq!(
            Mat4::from_frustum(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0).to_raw(),
            [-1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0]
        );
    }

    #[test]
    fn from_look_at() {
        let out = Mat4::from_look_at(&(0.0, 0.0, 1.0), &(0.0, 0.0, -1.0), &(0.0, 1.0, 0.0));

        assert_eq!(
            out.to_raw(),
            [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0, 1.0]
        );
    }

    #[test]
    fn from_target_to() {
        let out = Mat4::from_target_to(&(0.0, 0.0, 1.0), &(0.0, 0.0, -1.0), &(0.0, 1.0, 0.0));

        assert_eq!(
            out.to_raw(),
            [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0]
        );

        let scaling = out.scaling();
        assert_eq!(scaling.raw(), &[1.0, 1.0, 1.0]);
    }

    #[test]
    fn transpose() {
        assert_eq!(
            mat_a().clone().transpose().to_raw(),
            [1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 2.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0, 1.0]
        );
    }

    #[test]
    fn invert() -> Result<(), Error> {
        assert_eq!(
            mat_a().clone().invert()?.to_raw(),
            [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, -2.0, -3.0, 1.0]
        );

        Ok(())
    }

    #[test]
    fn adjoint() {
        assert_eq!(
            mat_a().clone().adjoint().to_gl(),
            [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, -2.0, -3.0, 1.0]
        );
    }

    #[test]
    fn determinant() {
        assert_eq!(mat_a().determinant(), 1.0);
    }

    #[test]
    fn translate() {
        assert_eq!(
            mat_a().clone().translate(&(4.0, 5.0, 6.0)).to_raw(),
            [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 5.0, 7.0, 9.0, 1.0]
        );
    }

    #[test]
    fn scale() {
        assert_eq!(
            mat_a().clone().scale(&(4.0, 5.0, 6.0)).to_raw(),
            [4.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 1.0, 2.0, 3.0, 1.0]
        );
    }

    #[test]
    fn rotate_x() {
        let rad = std::f64::consts::PI * 0.5;

        assert_eq!(
            mat_a().clone().rotate_x(rad).to_raw(),
            [
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                rad.cos(),
                rad.sin(),
                0.0,
                0.0,
                -rad.sin(),
                rad.cos(),
                0.0,
                1.0,
                2.0,
                3.0,
                1.0
            ]
        );
    }

    #[test]
    fn rotate_y() {
        let rad = std::f64::consts::PI * 0.5;

        assert_eq!(
            mat_a().clone().rotate_y(rad).to_raw(),
            [
                rad.cos(),
                0.0,
                -rad.sin(),
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                rad.sin(),
                0.0,
                rad.cos(),
                0.0,
                1.0,
                2.0,
                3.0,
                1.0
            ]
        );
    }

    #[test]
    fn rotate_z() {
        let rad = std::f64::consts::PI * 0.5;

        assert_eq!(
            mat_a().clone().rotate_z(rad).to_raw(),
            [
                rad.cos(),
                rad.sin(),
                0.0,
                0.0,
                -rad.sin(),
                rad.cos(),
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                1.0,
                2.0,
                3.0,
                1.0
            ]
        );
    }

    #[test]
    fn translation() {
        let translation = mat_identity().translation();
        assert_eq!(translation.raw(), &[0.0, 0.0, 0.0]);
    }

    #[test]
    fn scaling() {
        let scaling = mat_identity().scaling();
        assert_eq!(scaling.raw(), &[1.0, 1.0, 1.0]);
    }

    #[test]
    fn rotation() {
        let rotation = mat_identity().rotation();
        assert_eq!(rotation.raw(), &[0.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn decompose() {
        let (out_r, out_t, out_s) = mat_identity().decompose();
        assert_eq!(out_r.raw(), &[0.0, 0.0, 0.0, 1.0]);
        assert_eq!(out_t.raw(), &[0.0, 0.0, 0.0]);
        assert_eq!(out_s.raw(), &[1.0, 1.0, 1.0]);
    }

    #[test]
    fn frob() {
        let out = mat_a().frob();
        assert_eq!(
            out,
            (1.0f64.powi(2)
                + 1.0f64.powi(2)
                + 1.0f64.powi(2)
                + 1.0f64.powi(2)
                + 1.0f64.powi(2)
                + 2.0f64.powi(2)
                + 3.0f64.powi(2))
            .sqrt()
        );
    }

    #[test]
    fn set() {
        let mut mat = Mat4::new();
        mat.set(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );

        assert_eq!(
            mat.to_raw(),
            [
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn set_slice() {
        let mut mat = Mat4::<f64>::new();
        mat.set_slice(&[
            1.0f64, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
            16.0,
        ]);

        assert_eq!(
            mat.to_raw(),
            [
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn add() {
        let mat_a = Mat4::from_values(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        let mat_b = Mat4::from_values(
            17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0,
            31.0, 32.0,
        );

        assert_eq!(
            (mat_a + mat_b).to_raw(),
            [
                18.0, 20.0, 22.0, 24.0, 26.0, 28.0, 30.0, 32.0, 34.0, 36.0, 38.0, 40.0, 42.0, 44.0,
                46.0, 48.0
            ]
        );
    }

    #[test]
    fn sub() {
        let mat_a = Mat4::from_values(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        let mat_b = Mat4::from_values(
            17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0,
            31.0, 32.0,
        );

        assert_eq!(
            (mat_a - mat_b).to_raw(),
            [
                -16.0, -16.0, -16.0, -16.0, -16.0, -16.0, -16.0, -16.0, -16.0, -16.0, -16.0, -16.0,
                -16.0, -16.0, -16.0, -16.0
            ]
        );
    }

    #[test]
    fn mul() {
        let out = *mat_a() * *mat_b();
        assert_eq!(
            out.to_raw(),
            [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 5.0, 7.0, 9.0, 1.0]
        );
    }

    #[test]
    fn mul_scalar() {
        let mat = Mat4::from_values(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );

        assert_eq!(
            (mat * 2.0).to_raw(),
            [
                2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0, 22.0, 24.0, 26.0, 28.0,
                30.0, 32.0
            ]
        );
    }

    #[test]
    fn mul_scalar_add() {
        let mat_a = Mat4::from_values(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        let mat_b = Mat4::from_values(
            17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0,
            31.0, 32.0,
        );

        assert_eq!(
            (mat_a + mat_b * 0.5).to_raw(),
            [
                9.5, 11.0, 12.5, 14.0, 15.5, 17.0, 18.5, 20.0, 21.5, 23.0, 24.5, 26.0, 27.5, 29.0,
                30.5, 32.0
            ]
        );
    }

    #[test]
    fn approximate_eq() {
        let mat_a = Mat4::from_values(
            0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
        );
        let mat_b = Mat4::from_values(
            0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
        );
        let mat_c = Mat4::from_values(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        let mat_d = Mat4::from_values(
            1e-16, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
        );

        assert_eq!(true, mat_a.approximate_eq(&mat_b));
        assert_eq!(false, mat_a.approximate_eq(&mat_c));
        assert_eq!(true, mat_a.approximate_eq(&mat_d));
    }

    #[test]
    fn display() {
        let out = mat_a().to_string();
        assert_eq!(out, "mat4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 2, 3, 1)");
    }

    #[test]
    fn rotate() -> Result<(), Error> {
        let rad = std::f64::consts::PI * 0.5;
        let axis = Vec3::from_values(1.0, 0.0, 0.0);

        assert_eq!(
            mat_a().clone().rotate(&axis, rad)?.to_raw(),
            [
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                rad.cos(),
                rad.sin(),
                0.0,
                0.0,
                -rad.sin(),
                rad.cos(),
                0.0,
                1.0,
                2.0,
                3.0,
                1.0
            ]
        );

        Ok(())
    }

    #[test]
    fn from_perspective_no() {
        assert_eq!(
            Mat4::from_perspective_no(std::f32::consts::PI * 0.5, 1.0, 0.0, Some(1.0)).to_raw(),
            [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0]
        );

        assert_eq!(
            Mat4::from_perspective_no(
                std::f32::consts::PI * 45.0 / 180.0,
                640.0 / 480.0,
                0.1,
                Some(200.0)
            )
            .to_raw(),
            [
                1.81066,
                0.0,
                0.0,
                0.0,
                0.0,
                2.4142134,
                0.0,
                0.0,
                0.0,
                0.0,
                -1.0010005,
                -1.0,
                0.0,
                0.0,
                -0.20010006,
                0.0
            ]
        );

        assert_eq!(
            Mat4::from_perspective_no(
                std::f32::consts::PI * 45.0 / 180.0,
                640.0 / 480.0,
                0.1,
                None
            )
            .to_raw(),
            [
                1.81066, 0.0, 0.0, 0.0, 0.0, 2.4142134, 0.0, 0.0, 0.0, 0.0, -1.0, -1.0, 0.0, 0.0,
                -0.2, 0.0
            ]
        );

        assert_eq!(
            Mat4::from_perspective_no(
                std::f32::consts::PI * 45.0 / 180.0,
                640.0 / 480.0,
                0.1,
                Some(f32::INFINITY)
            )
            .to_raw(),
            [
                1.81066, 0.0, 0.0, 0.0, 0.0, 2.4142134, 0.0, 0.0, 0.0, 0.0, -1.0, -1.0, 0.0, 0.0,
                -0.2, 0.0
            ]
        );
    }
}
