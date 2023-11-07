use crate::EPSILON_F64;

#[derive(Debug, Clone, Copy)]
pub enum Error {
    ZeroDeterminant,
    LengthSmallerThanEpsilon,
}

impl std::error::Error for Error {}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::ZeroDeterminant => f.write_str("determinant of the matrix is 0.0"),
            Error::LengthSmallerThanEpsilon => f.write_fmt(format_args!(
                "length of the vector smaller than epsilon({})",
                EPSILON_F64
            )),
        }
    }
}
