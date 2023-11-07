#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Quat<T = f64>(pub [T; 4]);

macro_rules! float {
    ($(($t:tt, $epsilon:expr)),+) => {
        $(
            impl Quat<$t> {
                pub fn new() -> Quat<$t> {
                    Self([0.0; 4])
                }
            }

            impl Quat<$t> {
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
