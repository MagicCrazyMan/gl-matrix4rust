pub mod error;
pub mod mat2;
pub mod mat2d;
pub mod mat3;
pub mod mat4;
pub mod quat;
pub mod quat2;
pub mod vec2;
pub mod vec3;
pub mod vec4;

pub const EPSILON_F32: f32 = 0.000001f32;
pub const EPSILON_F64: f64 = 0.000001f64;

pub trait ApproximateEq {
    fn approximate_eq(&self, other: &Self) -> bool;
}

impl ApproximateEq for f32 {
    #[inline]
    fn approximate_eq(&self, other: &Self) -> bool {
        (self - other).abs() < EPSILON_F32
    }
}

impl ApproximateEq for f64 {
    #[inline]
    fn approximate_eq(&self, other: &Self) -> bool {
        (self - other).abs() < EPSILON_F64
    }
}

#[cfg(feature = "gl")]
pub trait GLF32<const N: usize> {
    fn gl_f32(&self) -> [f32; N];
}

#[cfg(feature = "gl")]
pub trait GLU8<const N: usize> {
    fn gl_u8(&self) -> [u8; N];
}

macro_rules! gl_f32_to_u8 {
    ($(($f32size: tt, $u8size: tt)),+) => {
        $(
            impl<T: GLF32<$f32size>> GLU8<$u8size> for T {
                #[inline(always)]
                fn gl_u8(&self) -> [u8; $u8size] {
                    unsafe { std::mem::transmute::<[f32; $f32size], [u8; $u8size]>(self.gl_f32()) }
                }
            }
        )+
    };
}

#[cfg(feature = "gl")]
gl_f32_to_u8! {
    (2, 8),
    (3, 12),
    (4, 16),
    (6, 24),
    (8, 32),
    (9, 36),
    (16, 64)
}

#[cfg(feature = "gl")]
pub trait GLF32Borrowed<const N: usize> {
    fn gl_f32_borrowed(&self) -> &[f32; N];
}

#[cfg(feature = "gl")]
pub trait GLU8Borrowed<const N: usize> {
    fn gl_u8_borrowed(&self) -> &[u8; N];
}

macro_rules! gl_f32_borrowed_to_u8_borrowed {
    ($(($f32size: tt, $u8size: tt)),+) => {
        $(
            impl<T: GLF32Borrowed<$f32size>> GLU8Borrowed<$u8size> for T {
                #[inline(always)]
                fn gl_u8_borrowed(&self) -> &[u8; $u8size] {
                    unsafe { std::mem::transmute::<&[f32; $f32size], &[u8; $u8size]>(self.gl_f32_borrowed()) }
                }
            }
        )+
    };
}

#[cfg(feature = "gl")]
gl_f32_borrowed_to_u8_borrowed! {
    (2, 8),
    (3, 12),
    (4, 16),
    (6, 24),
    (8, 32),
    (9, 36),
    (16, 64)
}