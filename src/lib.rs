use num_traits::Float;

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

#[inline(always)]
pub fn epsilon<T: Float>() -> T {
    T::from(0.000001).unwrap()
}
