#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Vec4<T = f64>(pub [T; 4]);

macro_rules! float {
    ($(($t:tt, $epsilon:expr)),+) => {
        $(
            impl Vec4<$t> {
                pub fn new() -> Vec4<$t> {
                    Self([0.0; 4])
                }

                pub fn from_values(x: $t, y: $t, z: $t, w: $t) -> Vec4<$t> {
                    Self([x, y, z, w])
                }
            }

            impl Vec4<$t> {
                pub fn raw(&self) -> &[$t; 4] {
                    &self.0
                }
            }
        )+
    };
}

float! {
    (f64, EPSILON_F64),
    (f32, EPSILON_F32)
}
