use std::{
    fmt::Display,
    ops::{Add, Mul, Sub},
};

use num_traits::Float;

use crate::{epsilon, error::Error, mat2d::Mat2d, mat4::Mat4, quat::Quat, vec2::Vec2, vec3::Vec3};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mat3<T = f32>(pub [T; 9]);

impl<T> AsRef<Mat3<T>> for Mat3<T> {
    fn as_ref(&self) -> &Self {
        self
    }
}

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
    pub fn from_mat4(mat: impl AsRef<Mat4<T>>) -> Self {
        let mat = mat.as_ref();
        Self([
            mat.0[0], mat.0[1], mat.0[2], mat.0[4], mat.0[5], mat.0[6], mat.0[8], mat.0[9],
            mat.0[10],
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
    pub fn from_translation(v: impl AsRef<Vec2<T>>) -> Self {
        let v = v.as_ref();
        Self([
            T::one(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
            T::zero(),
            v.0[0],
            v.0[1],
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_scaling(v: impl AsRef<Vec2<T>>) -> Self {
        let v = v.as_ref();
        Self([
            v.0[0],
            T::zero(),
            T::zero(),
            T::zero(),
            v.0[1],
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
    pub fn from_mat_2d(a: impl AsRef<Mat2d<T>>) -> Self {
        let a = a.as_ref();
        Self([
            a.0[0],
            a.0[1],
            T::zero(),
            a.0[2],
            a.0[3],
            T::zero(),
            a.0[4],
            a.0[5],
            T::one(),
        ])
    }

    #[inline(always)]
    pub fn from_quat(q: impl AsRef<Quat<T>>) -> Self {
        let q = q.as_ref();
        let x = q.0[0];
        let y = q.0[1];
        let z = q.0[2];
        let w = q.0[3];
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
    pub fn normal_matrix_from_mat4(a: impl AsRef<Mat4<T>>) -> Result<Self, Error> {
        let a = a.as_ref();

        let a00 = a.0[0];
        let a01 = a.0[1];
        let a02 = a.0[2];
        let a03 = a.0[3];
        let a10 = a.0[4];
        let a11 = a.0[5];
        let a12 = a.0[6];
        let a13 = a.0[7];
        let a20 = a.0[8];
        let a21 = a.0[9];
        let a22 = a.0[10];
        let a23 = a.0[11];
        let a30 = a.0[12];
        let a31 = a.0[13];
        let a32 = a.0[14];
        let a33 = a.0[15];

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
}

impl<T: Float> Mat3<T> {
    #[inline(always)]
    pub fn raw(&self) -> &[T; 9] {
        &self.0
    }

    #[inline(always)]
    pub fn set(
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
        self.0[0] = m00;
        self.0[1] = m01;
        self.0[2] = m02;
        self.0[3] = m10;
        self.0[4] = m11;
        self.0[5] = m12;
        self.0[6] = m20;
        self.0[7] = m21;
        self.0[8] = m22;

        self
    }

    #[inline(always)]
    pub fn set_slice(&mut self, slice: &[T; 9]) -> &mut Self {
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
        self.0[6] = T::zero();
        self.0[7] = T::zero();
        self.0[8] = T::zero();
        self
    }

    #[inline(always)]
    pub fn set_identify(&mut self) -> &mut Self {
        self.0[0] = T::one();
        self.0[1] = T::zero();
        self.0[2] = T::zero();
        self.0[3] = T::zero();
        self.0[4] = T::one();
        self.0[5] = T::zero();
        self.0[6] = T::zero();
        self.0[7] = T::zero();
        self.0[8] = T::one();
        self
    }

    #[inline(always)]
    pub fn transpose(&self) -> Self {
        Self([
            self.0[0], self.0[3], self.0[6], self.0[1], self.0[4], self.0[7], self.0[2], self.0[5],
            self.0[8],
        ])
    }

    #[inline(always)]
    pub fn invert(&self) -> Result<Self, Error> {
        let a00 = self.0[0];
        let a01 = self.0[1];
        let a02 = self.0[2];
        let a10 = self.0[3];
        let a11 = self.0[4];
        let a12 = self.0[5];
        let a20 = self.0[6];
        let a21 = self.0[7];
        let a22 = self.0[8];

        let b01 = a22 * a11 - a12 * a21;
        let b11 = -a22 * a10 + a12 * a20;
        let b21 = a21 * a10 - a11 * a20;

        // Calculate the determinant
        let mut det = a00 * b01 + a01 * b11 + a02 * b21;

        if det == T::zero() {
            return Err(Error::ZeroDeterminant);
        }
        det = T::one() / det;

        Ok(Self([
            b01 * det,
            (-a22 * a01 + a02 * a21) * det,
            (a12 * a01 - a02 * a11) * det,
            b11 * det,
            (a22 * a00 - a02 * a20) * det,
            (-a12 * a00 + a02 * a10) * det,
            b21 * det,
            (-a21 * a00 + a01 * a20) * det,
            (a11 * a00 - a01 * a10) * det,
        ]))
    }

    #[inline(always)]
    pub fn adjoint(&self) -> Self {
        let a00 = self.0[0];
        let a01 = self.0[1];
        let a02 = self.0[2];
        let a10 = self.0[3];
        let a11 = self.0[4];
        let a12 = self.0[5];
        let a20 = self.0[6];
        let a21 = self.0[7];
        let a22 = self.0[8];

        Self([
            a11 * a22 - a12 * a21,
            a02 * a21 - a01 * a22,
            a01 * a12 - a02 * a11,
            a12 * a20 - a10 * a22,
            a00 * a22 - a02 * a20,
            a02 * a10 - a00 * a12,
            a10 * a21 - a11 * a20,
            a01 * a20 - a00 * a21,
            a00 * a11 - a01 * a10,
        ])
    }

    #[inline(always)]
    pub fn determinant(&self) -> T {
        let a00 = self.0[0];
        let a01 = self.0[1];
        let a02 = self.0[2];
        let a10 = self.0[3];
        let a11 = self.0[4];
        let a12 = self.0[5];
        let a20 = self.0[6];
        let a21 = self.0[7];
        let a22 = self.0[8];

        a00 * (a22 * a11 - a12 * a21)
            + a01 * (-a22 * a10 + a12 * a20)
            + a02 * (a21 * a10 - a11 * a20)
    }

    #[inline(always)]
    pub fn translate(&self, v: impl AsRef<Vec3<T>>) -> Self {
        let v = v.as_ref();

        let a00 = self.0[0];
        let a01 = self.0[1];
        let a02 = self.0[2];
        let a10 = self.0[3];
        let a11 = self.0[4];
        let a12 = self.0[5];
        let a20 = self.0[6];
        let a21 = self.0[7];
        let a22 = self.0[8];
        let x = v.0[0];
        let y = v.0[1];

        Self([
            a00,
            a01,
            a02,
            a10,
            a11,
            a12,
            x * a00 + y * a10 + a20,
            x * a01 + y * a11 + a21,
            x * a02 + y * a12 + a22,
        ])
    }

    #[inline(always)]
    pub fn scale(&self, v: impl AsRef<Vec2<T>>) -> Self {
        let v = v.as_ref();

        let x = v.0[0];
        let y = v.0[1];

        Self([
            x * self.0[0],
            x * self.0[1],
            x * self.0[2],
            y * self.0[3],
            y * self.0[4],
            y * self.0[5],
            self.0[6],
            self.0[7],
            self.0[8],
        ])
    }

    #[inline(always)]
    pub fn rotate(&self, rad: T) -> Self {
        let a00 = self.0[0];
        let a01 = self.0[1];
        let a02 = self.0[2];
        let a10 = self.0[3];
        let a11 = self.0[4];
        let a12 = self.0[5];
        let a20 = self.0[6];
        let a21 = self.0[7];
        let a22 = self.0[8];
        let s = rad.sin();
        let c = rad.cos();

        Self([
            c * a00 + s * a10,
            c * a01 + s * a11,
            c * a02 + s * a12,
            c * a10 - s * a00,
            c * a11 - s * a01,
            c * a12 - s * a02,
            a20,
            a21,
            a22,
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
            + self.0[6] * self.0[6]
            + self.0[7] * self.0[7]
            + self.0[8] * self.0[8])
            .sqrt()
    }

    /// Returns whether or not the matrices have approximately the same elements in the same position.
    ///
    /// Refers to `equals` function in `glMatrix`. `exactEquals` is implemented with [`PartialEq`] and [`Eq`],
    #[inline(always)]
    pub fn approximate_eq(&self, b: impl AsRef<Mat3<T>>) -> bool {
        let b = b.as_ref();

        let a0 = self.0[0];
        let a1 = self.0[1];
        let a2 = self.0[2];
        let a3 = self.0[3];
        let a4 = self.0[4];
        let a5 = self.0[5];
        let a6 = self.0[6];
        let a7 = self.0[7];
        let a8 = self.0[8];

        let b0 = b.0[0];
        let b1 = b.0[1];
        let b2 = b.0[2];
        let b3 = b.0[3];
        let b4 = b.0[4];
        let b5 = b.0[5];
        let b6 = b.0[6];
        let b7 = b.0[7];
        let b8 = b.0[8];

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

/// tests only for f32
#[cfg(test)]
mod tests {
    use std::sync::OnceLock;

    use crate::{error::Error, mat4::Mat4, vec3::Vec3};

    use super::Mat3;

    static MAT_A_RAW: [f32; 9] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 1.0];
    static MAT_B_RAW: [f32; 9] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 3.0, 4.0, 1.0];

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
            Mat3::<f32>::new().raw(),
            &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        );
    }

    #[test]
    fn new_identity() {
        assert_eq!(
            Mat3::<f32>::new_identity().raw(),
            &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        );
    }

    #[test]
    fn from_slice() {
        assert_eq!(
            Mat3::from_slice(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0,]).raw(),
            &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0,]
        );
    }

    #[test]
    fn from_values() {
        assert_eq!(
            Mat3::from_values(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0,).raw(),
            &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0,]
        );
    }

    // #[test]
    // fn from_quat() {
    //     let q = Quat::<T>::from_values(0.0, -0.7071067811865475, 0.0, 0.7071067811865475);

    //     assert_eq!(
    //         Mat3::from_quat(&q).raw(),
    //         &[
    //             1.0, 2.0, 3.0,
    //             4.0, 5.0, 6.0,
    //             7.0, 8.0, 9.0,
    //         ]
    //     );
    // }

    #[test]
    fn from_mat4() {
        let mat4 = Mat4::from_values(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );

        assert_eq!(
            Mat3::from_mat4(mat4).raw(),
            &[1.0, 2.0, 3.0, 5.0, 6.0, 7.0, 9.0, 10.0, 11.0,]
        );
    }

    #[test]
    fn transpose() {
        assert_eq!(
            mat_a().transpose().raw(),
            &[1.0, 0.0, 1.0, 0.0, 1.0, 2.0, 0.0, 0.0, 1.0]
        );
    }

    #[test]
    fn invert() -> Result<(), Error> {
        assert_eq!(
            mat_a().invert()?.raw(),
            &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, -2.0, 1.0]
        );

        Ok(())
    }

    #[test]
    fn adjoint() {
        assert_eq!(
            mat_a().adjoint().raw(),
            &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, -2.0, 1.0]
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
            (1.0f32.powi(2)
                + 0.0f32.powi(2)
                + 0.0f32.powi(2)
                + 0.0f32.powi(2)
                + 0.0f32.powi(2)
                + 1.0f32.powi(2)
                + 0.0f32.powi(2)
                + 1.0f32.powi(2)
                + 2.0f32.powi(2)
                + 1.0f32.powi(2))
            .sqrt()
        );
    }

    #[test]
    fn from_projection() {
        assert_eq!(
            Mat3::from_projection(100.0, 200.0).raw(),
            &[0.02, 0.0, 0.0, 0.0, -0.01, 0.0, -1.0, 1.0, 1.0,]
        );
    }

    #[test]
    fn set() {
        let mut mat = Mat3::new();
        mat.set(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);

        assert_eq!(mat.raw(), &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
    }

    #[test]
    fn set_slice() {
        let mut mat = Mat3::new();
        mat.set_slice(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);

        assert_eq!(mat.raw(), &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
    }

    #[test]
    fn add() {
        let mat_a = Mat3::from_values(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let mat_b = Mat3::from_values(10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0);

        assert_eq!(
            (mat_a + mat_b).raw(),
            &[11.0, 13.0, 15.0, 17.0, 19.0, 21.0, 23.0, 25.0, 27.0]
        );
    }

    #[test]
    fn sub() {
        let mat_a = Mat3::from_values(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let mat_b = Mat3::from_values(10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0);

        assert_eq!(
            (mat_a - mat_b).raw(),
            &[-9.0, -9.0, -9.0, -9.0, -9.0, -9.0, -9.0, -9.0, -9.0]
        );
    }

    #[test]
    fn mul() {
        let out = *mat_a() * *mat_b();
        assert_eq!(out.raw(), &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 4.0, 6.0, 1.0]);
    }

    #[test]
    fn mul_scalar() {
        let mat = Mat3::from_values(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);

        assert_eq!(
            (mat * 2.0).raw(),
            &[2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0]
        );
    }

    #[test]
    fn mul_scalar_add() {
        let mat_a = Mat3::from_values(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let mat_b = Mat3::from_values(10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0);

        assert_eq!(
            (mat_a + mat_b * 0.5).raw(),
            &[6.0, 7.5, 9.0, 10.5, 12.0, 13.5, 15.0, 16.5, 18.0]
        );
    }

    #[test]
    fn approximate_eq() {
        let mat_a = Mat3::from_values(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0);
        let mat_b = Mat3::from_values(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0);
        let mat_c = Mat3::from_values(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let mat_d = Mat3::from_values(1e-16, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0);

        assert_eq!(true, mat_a.approximate_eq(mat_b));
        assert_eq!(false, mat_a.approximate_eq(mat_c));
        assert_eq!(true, mat_a.approximate_eq(mat_d));
    }

    #[test]
    fn display() {
        let out = mat_a().to_string();
        assert_eq!(out, "mat3(1, 0, 0, 0, 1, 0, 1, 2, 1)");
    }
    #[test]
    fn normal_matrix_from_mat4() -> Result<(), Error> {
        let mut mat4 = Mat4::new_identity();
        mat4 = mat4.translate(Vec3::from_values(2.0, 4.0, 6.0));
        mat4 = mat4.rotate_x(std::f32::consts::PI / 2.0);

        assert_eq!(
            Mat3::normal_matrix_from_mat4(&mat4)?.raw(),
            &[
                1.0,
                0.0,
                0.0,
                0.0,
                -4.371139e-8,
                1.0,
                0.0,
                -1.0,
                -4.371139e-8,
            ]
        );

        mat4 = mat4.scale(Vec3::from_values(2.0, 3.0, 4.0));

        assert_eq!(
            Mat3::normal_matrix_from_mat4(mat4)?.raw(),
            &[
                0.5,
                0.0,
                0.0,
                0.0,
                -1.4570463e-8,
                0.33333334,
                0.0,
                -0.25,
                -1.0927847e-8,
            ]
        );

        Ok(())
    }
}
