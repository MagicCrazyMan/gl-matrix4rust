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

// pub trait ToWebGLData<const N: usize, const C: usize> {
//     fn to_gl(&self) -> [f32; N];

//     fn to_gl_binary(&self) -> [u8; C];
// }
