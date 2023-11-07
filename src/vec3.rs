#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Vec3<T = f64>(pub [T; 3]);

macro_rules! float {
    ($(($t:tt, $epsilon:expr)),+) => {
        $(
            impl Vec3<$t> {
                pub fn new() -> Vec3<$t> {
                    Self([0.0; 3])
                }
            }
        )+
    };
}

float! {
    (f64, EPSILON_F64),
    (f32, EPSILON_F32)
}
