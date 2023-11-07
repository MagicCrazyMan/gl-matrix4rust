#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Vec2<T = f64>(pub [T; 2]);

macro_rules! float {
    ($(($t:tt, $epsilon:expr)),+) => {
        $(
            impl Vec2<$t> {
                pub fn new() -> Vec2<$t> {
                    Self([0.0; 2])
                }

                pub fn from_values(x: $t, y: $t) -> Vec2<$t> {
                    Self([x, y])
                }
            }

            impl Vec2<$t> {
                pub fn raw(&self) -> &[$t; 2] {
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
