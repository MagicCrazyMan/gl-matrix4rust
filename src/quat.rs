use std::{
    fmt::Display,
    ops::{Add, Mul},
};

use num_traits::{Float, FloatConst};

use crate::{epsilon, mat3::Mat3, vec3::Vec3};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EulerOrder {
    XYZ,
    XZY,
    YZX,
    YXZ,
    ZXY,
    ZYX,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Quat<T = f32>(pub [T; 4]);

impl<T: Float> Quat<T> {
    #[inline(always)]
    pub fn new() -> Self {
        Self([T::zero(); 4])
    }

    #[inline(always)]
    pub fn new_identity() -> Self {
        Self([T::zero(), T::zero(), T::zero(), T::one()])
    }

    #[inline(always)]
    pub fn from_values(x: T, y: T, z: T, w: T) -> Self {
        Self([x, y, z, w])
    }

    #[inline(always)]
    pub fn from_slice(slice: &[T; 4]) -> Self {
        Self(slice.clone())
    }

    #[inline(always)]
    pub fn from_axis_angle(axis: &Vec3<T>, rad: T) -> Self {
        let rad = rad * T::from(0.5).unwrap();
        let s = rad.sin();

        Self([s * axis.0[0], s * axis.0[1], s * axis.0[2], rad.cos()])
    }

    #[inline(always)]
    pub fn from_axes(view: &Vec3<T>, right: &Vec3<T>, up: &Vec3<T>) -> Self {
        let matr = Mat3::<T>::from_values(
            right.0[0], up.0[0], -view.0[0], right.0[1], up.0[1], -view.0[1], right.0[2], up.0[2],
            -view.0[2],
        );

        Self::from_mat3(&matr).normalize()
    }

    #[inline(always)]
    pub fn from_mat3(m: &Mat3<T>) -> Self {
        let mut out = Self::new();

        // Algorithm in Ken Shoemake's article in 1987 SIGGRAPH course notes
        // article "Quaternion Calculus and Fast Animation".
        let f_trace = m.0[0] + m.0[4] + m.0[8];
        let mut f_root;

        if f_trace > T::zero() {
            // |w| > 1/2, may as well choose w > 1/2
            f_root = (f_trace + T::one()).sqrt(); // 2w
            out.0[3] = T::from(0.5).unwrap() * f_root;
            f_root = T::from(0.5).unwrap() / f_root; // 1/(4w)
            out.0[0] = (m.0[5] - m.0[7]) * f_root;
            out.0[1] = (m.0[6] - m.0[2]) * f_root;
            out.0[2] = (m.0[1] - m.0[3]) * f_root;
        } else {
            // |w| <= 1/2
            let mut i = 0;
            if m.0[4] > m.0[0] {
                i = 1
            };
            if m.0[8] > m.0[i * 3 + i] {
                i = 2
            };
            let j = (i + 1) % 3;
            let k = (i + 2) % 3;

            f_root = (m.0[i * 3 + i] - m.0[j * 3 + j] - m.0[k * 3 + k] + T::one()).sqrt();
            out.0[i] = T::from(0.5).unwrap() * f_root;
            f_root = T::from(0.5).unwrap() / f_root;
            out.0[3] = (m.0[j * 3 + k] - m.0[k * 3 + j]) * f_root;
            out.0[j] = (m.0[j * 3 + i] + m.0[i * 3 + j]) * f_root;
            out.0[k] = (m.0[k * 3 + i] + m.0[i * 3 + k]) * f_root;
        }

        out
    }
}

impl<T: Float + FloatConst> Quat<T> {
    #[inline(always)]
    pub fn from_euler_with_order(x: T, y: T, z: T, order: EulerOrder) -> Self {
        let half_to_rad = T::PI() / T::from(360.0).unwrap();
        let x = x * half_to_rad;
        let z = z * half_to_rad;
        let y = y * half_to_rad;

        let sx = x.sin();
        let cx = x.cos();
        let sy = y.sin();
        let cy = y.cos();
        let sz = z.sin();
        let cz = z.cos();

        match order {
            EulerOrder::XYZ => Self([
                sx * cy * cz + cx * sy * sz,
                cx * sy * cz - sx * cy * sz,
                cx * cy * sz + sx * sy * cz,
                cx * cy * cz - sx * sy * sz,
            ]),
            EulerOrder::XZY => Self([
                sx * cy * cz - cx * sy * sz,
                cx * sy * cz - sx * cy * sz,
                cx * cy * sz + sx * sy * cz,
                cx * cy * cz + sx * sy * sz,
            ]),
            EulerOrder::YXZ => Self([
                sx * cy * cz + cx * sy * sz,
                cx * sy * cz - sx * cy * sz,
                cx * cy * sz - sx * sy * cz,
                cx * cy * cz + sx * sy * sz,
            ]),
            EulerOrder::YZX => Self([
                sx * cy * cz + cx * sy * sz,
                cx * sy * cz + sx * cy * sz,
                cx * cy * sz - sx * sy * cz,
                cx * cy * cz - sx * sy * sz,
            ]),
            EulerOrder::ZXY => Self([
                sx * cy * cz - cx * sy * sz,
                cx * sy * cz + sx * cy * sz,
                cx * cy * sz + sx * sy * cz,
                cx * cy * cz - sx * sy * sz,
            ]),
            EulerOrder::ZYX => Self([
                sx * cy * cz - cx * sy * sz,
                cx * sy * cz + sx * cy * sz,
                cx * cy * sz - sx * sy * cz,
                cx * cy * cz + sx * sy * sz,
            ]),
        }
    }

    #[inline(always)]
    pub fn from_euler(x: T, y: T, z: T) -> Self {
        Self::from_euler_with_order(x, y, z, EulerOrder::ZYX)
    }

    #[inline(always)]
    pub fn from_rotate_to(a: &Vec3<T>, b: &Vec3<T>) -> Self {
        let dot = a.dot(b);
        if dot < T::from(-0.999999).unwrap() {
            let x_unit = Vec3::<T>::from_values(T::one(), T::zero(), T::zero());
            let y_unit = Vec3::<T>::from_values(T::zero(), T::one(), T::zero());
            let mut tmp = x_unit.cross(a);
            if tmp.length() < T::from(0.00001).unwrap() {
                tmp = y_unit.cross(a)
            };
            tmp = tmp.normalize();
            Self::from_axis_angle(&tmp, T::PI())
        } else if dot > T::from(0.999999).unwrap() {
            Self([T::zero(), T::zero(), T::zero(), T::one()])
        } else {
            let tmp = a.cross(b);
            let out = Self([tmp.0[0], tmp.0[1], tmp.0[2], T::one() + dot]);
            out.normalize()
        }
    }
}

impl<T: Float> Quat<T> {
    #[inline(always)]
    pub fn raw(&self) -> &[T; 4] {
        &self.0
    }

    #[inline(always)]
    pub fn axis_angle(&self) -> (Vec3<T>, T) {
        let rad = self.0[3].acos() * T::from(2.0).unwrap();
        let s = (rad / T::from(2.0).unwrap()).sin();
        if s > epsilon() {
            (
                Vec3::<T>::from_values(self.0[0] / s, self.0[1] / s, self.0[2] / s),
                rad,
            )
        } else {
            // If s is zero, return any axis (no rotation - axis does not matter)
            (Vec3::<T>::from_values(T::one(), T::zero(), T::zero()), rad)
        }
    }

    #[inline(always)]
    pub fn angle(&self, b: &Self) -> T {
        let dotproduct = self.dot(b);
        (T::from(2.0).unwrap() * dotproduct * dotproduct - T::one()).acos()
    }

    #[inline(always)]
    pub fn set(&mut self, x: T, y: T, z: T, w: T) -> &mut Self {
        self.0[0] = x;
        self.0[1] = y;
        self.0[2] = z;
        self.0[3] = w;
        self
    }

    #[inline(always)]
    pub fn set_slice(&mut self, [x, y, z, w]: &[T; 4]) -> &mut Self {
        self.0[0] = *x;
        self.0[1] = *y;
        self.0[2] = *z;
        self.0[3] = *w;
        self
    }

    #[inline(always)]
    pub fn set_zero(&mut self) -> &mut Self {
        self.0[0] = T::zero();
        self.0[1] = T::zero();
        self.0[2] = T::zero();
        self.0[3] = T::zero();
        self
    }

    #[inline(always)]
    pub fn set_identify(&mut self) -> &mut Self {
        self.0[0] = T::zero();
        self.0[1] = T::zero();
        self.0[2] = T::zero();
        self.0[3] = T::one();
        self
    }

    #[inline(always)]
    pub fn rotate_x(&self, rad: T) -> Self {
        let rad = rad * T::from(0.5).unwrap();

        let ax = self.0[0];
        let ay = self.0[1];
        let az = self.0[2];
        let aw = self.0[3];
        let bx = rad.sin();
        let bw = rad.cos();

        Self([
            ax * bw + aw * bx,
            ay * bw + az * bx,
            az * bw - ay * bx,
            aw * bw - ax * bx,
        ])
    }

    #[inline(always)]
    pub fn rotate_y(&self, rad: T) -> Self {
        let rad = rad * T::from(0.5).unwrap();

        let ax = self.0[0];
        let ay = self.0[1];
        let az = self.0[2];
        let aw = self.0[3];
        let by = rad.sin();
        let bw = rad.cos();

        Self([
            ax * bw - az * by,
            ay * bw + aw * by,
            az * bw + ax * by,
            aw * bw - ay * by,
        ])
    }

    #[inline(always)]
    pub fn rotate_z(&self, rad: T) -> Self {
        let rad = rad * T::from(0.5).unwrap();

        let ax = self.0[0];
        let ay = self.0[1];
        let az = self.0[2];
        let aw = self.0[3];
        let bz = rad.sin();
        let bw = rad.cos();

        Self([
            ax * bw + ay * bz,
            ay * bw - ax * bz,
            az * bw + aw * bz,
            aw * bw - az * bz,
        ])
    }

    #[inline(always)]
    pub fn calculate_w(&self) -> Self {
        let x = self.0[0];
        let y = self.0[1];
        let z = self.0[2];

        Self([x, y, z, (T::one() - x * x - y * y - z * z).abs().sqrt()])
    }

    #[inline(always)]
    pub fn exp(&self) -> Self {
        let x = self.0[0];
        let y = self.0[1];
        let z = self.0[2];
        let w = self.0[3];

        let r = (x * x + y * y + z * z).sqrt();
        let et = w.exp();
        let s = if r > T::zero() {
            (et * r.sin()) / r
        } else {
            T::zero()
        };

        Self([x * s, y * s, z * s, et * r.cos()])
    }

    #[inline(always)]
    pub fn ln(&self) -> Self {
        let x = self.0[0];
        let y = self.0[1];
        let z = self.0[2];
        let w = self.0[3];

        let r = (x * x + y * y + z * z).sqrt();
        let t = if r > T::zero() {
            r.atan2(w) / r
        } else {
            T::zero()
        };

        Self([
            x * t,
            y * t,
            z * t,
            T::from(0.5).unwrap() * (x * x + y * y + z * z + w * w).ln(),
        ])
    }

    #[inline(always)]
    pub fn scale(&self, scale: T) -> Self {
        self.mul(scale)
    }

    #[inline(always)]
    pub fn pow(&self, b: T) -> Self {
        self.ln().scale(b).exp()
    }

    #[inline(always)]
    pub fn dot(&self, b: &Self) -> T {
        self.0[0] * b.0[0] + self.0[1] * b.0[1] + self.0[2] * b.0[2] + self.0[3] * b.0[3]
    }

    #[inline(always)]
    pub fn lerp(&self, b: &Self, t: T) -> Self {
        let ax = self.0[0];
        let ay = self.0[1];
        let az = self.0[2];
        let aw = self.0[3];

        Self([
            ax + t * (b.0[0] - ax),
            ay + t * (b.0[1] - ay),
            az + t * (b.0[2] - az),
            aw + t * (b.0[3] - aw),
        ])
    }

    #[inline(always)]
    pub fn slerp(&self, b: &Self, t: T) -> Self {
        let ax = self.0[0];
        let ay = self.0[1];
        let az = self.0[2];
        let aw = self.0[3];
        let mut bx = b.0[0];
        let mut by = b.0[1];
        let mut bz = b.0[2];
        let mut bw = b.0[3];

        let omega;
        let mut cosom;
        let sinom;
        let scale0;
        let scale1;

        // calc cosine
        cosom = ax * bx + ay * by + az * bz + aw * bw;
        // adjust signs (if necessary)
        if cosom < T::zero() {
            cosom = -cosom;
            bx = -bx;
            by = -by;
            bz = -bz;
            bw = -bw;
        }
        // calculate coefficients
        if T::one() - cosom > epsilon() {
            // standard case (slerp)
            omega = cosom.acos();
            sinom = omega.sin();
            scale0 = (((T::one() - t) * omega) / sinom).sin();
            scale1 = ((t * omega) / sinom).sin();
        } else {
            // "from" and "to" quaternions are very close
            //  ... so we can do a linear interpolation
            scale0 = T::one() - t;
            scale1 = t;
        }

        Self([
            scale0 * ax + scale1 * bx,
            scale0 * ay + scale1 * by,
            scale0 * az + scale1 * bz,
            scale0 * aw + scale1 * bw,
        ])
    }

    #[inline(always)]
    pub fn sqlerp(&self, b: &Self, c: &Self, d: &Self, t: T) -> Self {
        let tmp1 = self.slerp(d, t);
        let tmp2 = b.slerp(c, t);
        tmp1.slerp(&tmp2, T::from(2.0).unwrap() * t * (T::one() - t))
    }

    #[inline(always)]
    pub fn squared_length(&self) -> T {
        let x = self.0[0];
        let y = self.0[1];
        let z = self.0[2];
        let w = self.0[3];
        x * x + y * y + z * z + w * w
    }

    #[inline(always)]
    pub fn length(&self) -> T {
        self.squared_length().sqrt()
    }

    #[inline(always)]
    pub fn normalize(&self) -> Self {
        let mut len = self.squared_length();
        if len > T::zero() {
            len = T::one() / len.sqrt();
        }

        Self([
            self.0[0] * len,
            self.0[1] * len,
            self.0[2] * len,
            self.0[3] * len,
        ])
    }

    #[inline(always)]
    pub fn invert(&self) -> Self {
        let a0 = self.0[0];
        let a1 = self.0[1];
        let a2 = self.0[2];
        let a3 = self.0[3];

        let dot = a0 * a0 + a1 * a1 + a2 * a2 + a3 * a3;
        let inv_dot = if dot != T::zero() {
            T::one() / dot
        } else {
            T::zero()
        };

        Self([-a0 * inv_dot, -a1 * inv_dot, -a2 * inv_dot, a3 * inv_dot])
    }

    #[inline(always)]
    pub fn conjugate(&self) -> Self {
        Self([-self.0[0], -self.0[1], -self.0[2], self.0[3]])
    }

    #[inline(always)]
    pub fn approximate_eq(&self, b: &Self) -> bool {
        self.dot(b).abs() >= T::one() - epsilon()
    }
}

impl<T: Float> Add<Quat<T>> for Quat<T> {
    type Output = Self;

    #[inline(always)]
    fn add(self, b: Self) -> Self {
        let mut out = Quat::<T>::new();
        out.0[0] = self.0[0] + b.0[0];
        out.0[1] = self.0[1] + b.0[1];
        out.0[2] = self.0[2] + b.0[2];
        out.0[3] = self.0[3] + b.0[3];
        out
    }
}

impl<T: Float> Mul<Quat<T>> for Quat<T> {
    type Output = Self;

    #[inline(always)]
    fn mul(self, b: Self) -> Self {
        let ax = self.0[0];
        let ay = self.0[1];
        let az = self.0[2];
        let aw = self.0[3];
        let bx = b.0[0];
        let by = b.0[1];
        let bz = b.0[2];
        let bw = b.0[3];

        Quat::<T>::from_values(
            ax * bw + aw * bx + ay * bz - az * by,
            ay * bw + aw * by + az * bx - ax * bz,
            az * bw + aw * bz + ax * by - ay * bx,
            aw * bw - ax * bx - ay * by - az * bz,
        )
    }
}

impl<T: Float> Mul<T> for Quat<T> {
    type Output = Self;

    #[inline(always)]
    fn mul(self, b: T) -> Self {
        Self([self.0[0] * b, self.0[1] * b, self.0[2] * b, self.0[3] * b])
    }
}

impl<T: Display> Display for Quat<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let value = self
            .0
            .iter()
            .map(|v| v.to_string())
            .collect::<Vec<_>>()
            .join(", ");
        f.write_fmt(format_args!("quat({})", value))
    }
}

impl Quat<f32> {
    #[inline(always)]
    pub fn random(&self) -> Self {
        let u1 = rand::random::<f32>();
        let u2 = rand::random::<f32>();
        let u3 = rand::random::<f32>();
        let sqrt1_minus_u1 = (1.0 - u1).sqrt();
        let sqrt_u1 = u1.sqrt();
        Self([
            sqrt1_minus_u1 * (2.0 * std::f32::consts::PI * u2).sin(),
            sqrt1_minus_u1 * (2.0 * std::f32::consts::PI * u2).cos(),
            sqrt_u1 * (2.0 * std::f32::consts::PI * u3).sin(),
            sqrt_u1 * (2.0 * std::f32::consts::PI * u3).cos(),
        ])
    }
}

impl Quat<f64> {
    #[inline(always)]
    pub fn random(&self) -> Self {
        let u1 = rand::random::<f64>();
        let u2 = rand::random::<f64>();
        let u3 = rand::random::<f64>();
        let sqrt1_minus_u1 = (1.0 - u1).sqrt();
        let sqrt_u1 = u1.sqrt();
        Self([
            sqrt1_minus_u1 * (2.0 * std::f64::consts::PI * u2).sin(),
            sqrt1_minus_u1 * (2.0 * std::f64::consts::PI * u2).cos(),
            sqrt_u1 * (2.0 * std::f64::consts::PI * u3).sin(),
            sqrt_u1 * (2.0 * std::f64::consts::PI * u3).cos(),
        ])
    }
}

// #[cfg(test)]
// #[rustfmt::skip]
// mod tests {
//     macro_rules! float_test {
//         (T:tt, epsilon():expr, T::PI():expr, $e:expr, $sqrt2:expr) => {
//             use std::sync::OnceLock;

//             use crate::error::Error;
//             use crate::quat::Quat;
//             use crate::vec3::Vec3;
//             use crate::mat3::Mat3;
//             use crate::mat4::Mat4;

//             static QUAT_A_RAW: [T; 4] = [T::one(), T::from(2.0).unwrap(), 3.0, 4.0];
//             static QUAT_B_RAW: [T; 4] = [5.0, 6.0, 7.0, 8.0];
//             static QUAT_IDENTITY_RAW: [T; 4] = [T::zero(), T::zero(), T::zero(), T::one()];

//             static QUAT_A: OnceLock<Quat<T>> = OnceLock::new();
//             static QUAT_B: OnceLock<Quat<T>> = OnceLock::new();
//             static QUAT_IDENTITY: OnceLock<Quat<T>> = OnceLock::new();

//             fn quat_a() -> &'static Quat<T> {
//                 QUAT_A.get_or_init(|| {
//                     Quat::<T>::from_slice(&QUAT_A_RAW)
//                 })
//             }

//             fn quat_b() -> &'static Quat<T> {
//                 QUAT_B.get_or_init(|| {
//                     Quat::<T>::from_slice(&QUAT_B_RAW)
//                 })
//             }

//             fn quat_identity() -> &'static Quat<T> {
//                 QUAT_IDENTITY.get_or_init(|| {
//                     Quat::<T>::from_slice(&QUAT_IDENTITY_RAW)
//                 })
//             }

//             #[test]
//             fn new() {
//                 assert_eq!(
//                     Quat::<T>::new().raw(),
//                     &[T::zero(), T::zero(), T::zero(), T::zero()]
//                 );
//             }

//             #[test]
//             fn new_identity() {
//                 assert_eq!(
//                     Quat::<T>::new_identity().raw(),
//                     &[T::zero(), T::zero(), T::zero(), T::one()]
//                 );
//             }

//             #[test]
//             fn from_slice() {
//                 assert_eq!(
//                     Quat::<T>::from_slice(&[3.0, 4.0, 5.0, 6.0]).raw(),
//                     &[3.0, 4.0, 5.0, 6.0]
//                 );
//             }

//             #[test]
//             fn from_values() {
//                 assert_eq!(
//                     Quat::<T>::from_values(3.0, 4.0, 5.0, 6.0).raw(),
//                     &[3.0, 4.0, 5.0, 6.0]
//                 );
//             }

//             #[test]
//             fn scale() {
//                 assert_eq!(
//                     (*quat_a() * T::from(2.0).unwrap()).raw(),
//                     &[T::from(2.0).unwrap(), 4.0, 6.0, 8.0]
//                 );
//             }

//             #[test]
//             fn scale_add() {
//                 assert_eq!(
//                     (*quat_a() + *quat_b() * T::from(0.5).unwrap()).raw(),
//                     &[3.5, 5.0, 6.5, 8.0]
//                 );
//             }

//             #[test]
//             fn squared_length() {
//                 assert_eq!(
//                     quat_a().squared_length(),
//                     3T::zero()
//                 );
//             }

//             #[test]
//             fn length() {
//                 assert_eq!(
//                     quat_a().length(),
//                     5.477225575051661
//                 );
//             }

//             #[test]
//             fn normalize() {
//                 assert_eq!(
//                     Quat::<T>::from_values(5.0, T::zero(), T::zero(), T::zero()).normalize().raw(),
//                     &[T::one() ,T::zero(), T::zero(), T::zero()]
//                 );
//             }

//             #[test]
//             fn dot() {
//                 assert_eq!(
//                     quat_a().dot(quat_b()),
//                     7T::zero()
//                 );
//             }

//             #[test]
//             fn lerp() {
//                 assert_eq!(
//                     quat_a().lerp(quat_b(), T::from(0.5).unwrap()).raw(),
//                     &[3.0, 4.0, 5.0, 6.0]
//                 );
//             }

//             #[test]
//             fn slerp() {
//                 assert_eq!(
//                     quat_a().slerp(quat_b(), T::from(0.5).unwrap()).raw(),
//                     &[3.0, 4.0, 5.0, 6.0]
//                 );
//             }

//             #[test]
//             fn conjugate() {
//                 assert_eq!(
//                     quat_a().conjugate().raw(),
//                     &[-T::one(), -T::from(2.0).unwrap(), -3.0, 4.0]
//                 );
//             }

//             #[test]
//             fn set() {
//                 let mut mat = Quat::<T>::new();
//                 mat.set(3.0, 4.0, 5.0, 6.0);

//                 assert_eq!(
//                     mat.raw(),
//                     &[3.0, 4.0, 5.0, 6.0]
//                 );
//             }

//             #[test]
//             fn set_slice() {
//                 let mut mat = Quat::<T>::new();
//                 mat.set_slice(&[3.0, 4.0, 5.0, 6.0]);

//                 assert_eq!(
//                     mat.raw(),
//                     &[3.0, 4.0, 5.0, 6.0]
//                 );
//             }

//             #[test]
//             fn add() {
//                 assert_eq!(
//                     (*quat_a() + *quat_b()).raw(),
//                     &[6.0, 8.0, 1T::zero(), 1T::from(2.0).unwrap()]
//                 );
//             }

//             #[test]
//             fn mul() {
//                 assert_eq!(
//                     (*quat_a() * *quat_b()).raw(),
//                     &[24.0, 48.0, 48.0, -6.0]
//                 );
//             }

//             #[test]
//             fn mul_scalar() {
//                 assert_eq!(
//                     (*quat_a() * T::from(2.0).unwrap()).raw(),
//                     &[T::from(2.0).unwrap(), 4.0, 6.0, 8.0]
//                 );
//             }

//             #[test]
//             fn mul_scalar_add() {
//                 assert_eq!(
//                     (*quat_a() + *quat_b() * T::from(0.5).unwrap()).raw(),
//                     &[3.5, 5.0, 6.5, 8.0]
//                 );
//             }

//             #[test]
//             fn approximate_eq() {
//                 let vec_a = Quat::<T>::from_values(T::zero(), T::zero(), T::zero(), T::one());
//                 let vec_b = Quat::<T>::from_values(T::zero(), T::zero(), T::zero(), T::one());
//                 let vec_c = Quat::<T>::from_values(T::zero(), T::one(), T::zero(), T::zero());
//                 let vec_d = Quat::<T>::from_values(1e-16, T::one(), T::from(2.0).unwrap(), 3.0);
//                 let vec_e = Quat::<T>::from_values(T::zero(), -T::one(), T::zero(), T::zero());

//                 assert_eq!(
//                     true,
//                     vec_a.approximate_eq(&vec_b)
//                 );
//                 assert_eq!(
//                     false,
//                     vec_a.approximate_eq(&vec_c)
//                 );
//                 assert_eq!(
//                     true,
//                     vec_a.approximate_eq(&vec_d)
//                 );
//                 assert_eq!(
//                     true,
//                     vec_c.approximate_eq(&vec_e)
//                 );
//             }

//             #[test]
//             fn display() {
//                 let out = quat_a().to_string();
//                 assert_eq!(
//                     out,
//                     "quat(1, 2, 3, 4)"
//                 );
//             }
//         };
//     }

//     mod f32 {
//         float_test!(
//             f32,
//             crate::EPSILON_F32,
//             std::f32::consts::PI,
//             std::f32::consts::E,
//             std::f32::consts::SQRT_2
//         );

//         #[test]
//         fn from_mat3() -> Result<(), Error> {
//             let mat = Mat3::<f32>::from_values(
//                 T::one(), T::zero(),  T::zero(),
//                 T::zero(), T::zero(), -T::one(),
//                 T::zero(), T::one(),  T::zero(),
//             );
//             let quat = Quat::<f32>::from_mat3(&mat);
//             assert_eq!(
//                 quat.raw(),
//                 &[-0.70710677, T::zero(), T::zero(), 0.70710677]
//             );

//             assert_eq!(
//                 Vec3::<f32>::from_values(T::zero(), T::one(), T::zero()).transform_quat(&quat).raw(),
//                 &[T::zero(), 5.9604645e-8, -T::from(0.999999).unwrap()94]
//             );

//             let mat = Mat3::<f32>::from_mat4(&Mat4::<f32>::from_look_at(
//                 &Vec3::<f32>::from_values(T::zero(), T::zero(), T::zero()),
//                 &Vec3::<f32>::from_values(T::zero(), T::zero(), T::one()),
//                 &Vec3::<f32>::from_values(T::zero(), T::one(), T::zero()),
//             ))
//                 .invert()?
//                 .transpose();
//             let quat = Quat::<f32>::from_mat3(&mat);
//             assert_eq!(
//                 Vec3::<f32>::from_values(3.0, T::from(2.0).unwrap(), -T::one()).transform_quat(&quat.normalize()).raw(),
//                 Vec3::<f32>::from_values(3.0, T::from(2.0).unwrap(), -T::one()).transform_mat3(&mat).raw()
//             );

//             let mat = Mat3::<f32>::from_mat4(&Mat4::<f32>::from_look_at(
//                 &Vec3::<f32>::from_values(T::zero(), T::zero(), T::zero()),
//                 &Vec3::<f32>::from_values(-T::one(), T::zero(), T::zero()),
//                 &Vec3::<f32>::from_values(T::zero(), -T::one(), T::zero()),
//             ))
//                 .invert()?
//                 .transpose();
//             let quat = Quat::<f32>::from_mat3(&mat);
//             assert_eq!(
//                 Vec3::<f32>::from_values(3.0, T::from(2.0).unwrap(), -T::one()).transform_quat(&quat.normalize()).raw(),
//                 &[-T::one()000005, -T::from(2.0).unwrap()000005, 3.0000005]
//             );

//             let mat = Mat3::<f32>::from_mat4(&Mat4::<f32>::from_look_at(
//                 &Vec3::<f32>::from_values(T::zero(), T::zero(), T::zero()),
//                 &Vec3::<f32>::from_values(T::zero(), T::zero(), -T::one()),
//                 &Vec3::<f32>::from_values(T::zero(), -T::one(), T::zero()),
//             ))
//                 .invert()?
//                 .transpose();
//             let quat = Quat::<f32>::from_mat3(&mat);
//             assert_eq!(
//                 Vec3::<f32>::from_values(3.0, T::from(2.0).unwrap(), -T::one()).transform_quat(&quat.normalize()).raw(),
//                 Vec3::<f32>::from_values(3.0, T::from(2.0).unwrap(), -T::one()).transform_mat3(&mat).raw()
//             );

//             Ok(())
//         }

//         #[test]
//         fn from_rotate_to() {
//             assert_eq!(
//                 Quat::<f32>::from_rotate_to(
//                     &Vec3::<f32>::from_values(T::zero(), T::one(), T::zero()),
//                     &Vec3::<f32>::from_values(T::one(), T::zero(), T::zero()),
//                 ).raw(),
//                 &[T::zero(), T::zero(), -0.70710677, 0.70710677]
//             );

//             let quat = Quat::<f32>::from_rotate_to(
//                 &Vec3::<f32>::from_values(T::zero(), T::one(), T::zero()),
//                 &Vec3::<f32>::from_values(T::zero(), T::one(), T::zero()),
//             );
//             assert_eq!(
//                 Vec3::<f32>::from_values(T::zero(), T::one(), T::zero()).transform_quat(&quat).raw(),
//                 &[T::zero(), T::one(), T::zero()]
//             );

//             let quat = Quat::<f32>::from_rotate_to(
//                 &Vec3::<f32>::from_values(T::one(), T::zero(), T::zero()),
//                 &Vec3::<f32>::from_values(-T::one(), T::zero(), T::zero()),
//             );
//             assert_eq!(
//                 Vec3::<f32>::from_values(T::one(), T::zero(), T::zero()).transform_quat(&quat).raw(),
//                 &[-T::one(), 8.742278e-8, T::zero()]
//             );

//             let quat = Quat::<f32>::from_rotate_to(
//                 &Vec3::<f32>::from_values(T::zero(), T::one(), T::zero()),
//                 &Vec3::<f32>::from_values(T::zero(), -T::one(), T::zero()),
//             );
//             assert_eq!(
//                 Vec3::<f32>::from_values(T::zero(), T::one(), T::zero()).transform_quat(&quat).raw(),
//                 &[8.742278e-8, -T::one(), T::zero()]
//             );

//             let quat = Quat::<f32>::from_rotate_to(
//                 &Vec3::<f32>::from_values(T::zero(), T::zero(), T::one()),
//                 &Vec3::<f32>::from_values(T::zero(), T::zero(), -T::one()),
//             );
//             assert_eq!(
//                 Vec3::<f32>::from_values(T::zero(), T::zero(), T::one()).transform_quat(&quat).raw(),
//                 &[8.742278e-8, T::zero(), -T::one()]
//             );
//         }

//         #[test]
//         fn from_axes() {
//             let quat = Quat::<f32>::from_axes(
//                 &Vec3::<f32>::from_values(-T::one(), T::zero(), T::zero()),
//                 &Vec3::<f32>::from_values(T::zero(), T::zero(), -T::one()),
//                 &Vec3::<f32>::from_values(T::zero(), T::one(), T::zero()),
//             );
//             assert_eq!(
//                 Vec3::<f32>::from_values(T::zero(), T::zero(), -T::one()).transform_quat(&quat).raw(),
//                 &[T::one()000001, T::zero(), 1.1920929e-7]
//             );
//             assert_eq!(
//                 Vec3::<f32>::from_values(T::one(), T::zero(), T::zero()).transform_quat(&quat).raw(),
//                 &[-1.1920929e-7, T::zero(), T::one()000001]
//             );

//             let quat = Quat::<f32>::from_axes(
//                 &Vec3::<f32>::from_values(T::zero(), T::zero(), -T::one()),
//                 &Vec3::<f32>::from_values(T::one(), T::zero(), T::zero()),
//                 &Vec3::<f32>::from_values(T::zero(), T::one(), T::zero()),
//             );
//             assert_eq!(
//                 quat.raw(),
//                 &[-T::zero(), T::zero(), T::zero(), T::one()]
//             );
//         }

//         #[test]
//         fn angle() {
//             assert_eq!(
//                 quat_a().normalize().angle(&quat_a().normalize()),
//                 T::zero()
//             );

//             // precision consider
//             assert_eq!(
//                 quat_a().normalize().angle(&quat_a().normalize().rotate_x(std::f32::consts::PI / 4.0)),
//                 0.7853986
//             );

//             // precision consider
//             let quat_a = quat_a().normalize();
//             let quat_b = quat_b().normalize();
//             assert_eq!(
//                 quat_a.angle(&quat_b),
//                 T::from(0.5).unwrap()003915
//             );
//         }

//         #[test]
//         fn axis_angle() {
//             let quat = Quat::<f32>::from_axis_angle(&Vec3::<f32>::from_values(T::zero(), T::one(), T::zero()), T::zero());
//             let (_, rad) = quat.axis_angle();
//             assert_eq!(
//                 rad % (std::f32::consts::PI * T::from(2.0).unwrap()),
//                 T::zero()
//             );

//             let quat = Quat::<f32>::from_axis_angle(&Vec3::<f32>::from_values(T::one(), T::zero(), T::zero()), 0.7778);
//             let (vec3, rad) = quat.axis_angle();
//             assert_eq!(
//                 vec3.raw(),
//                 &[T::one(), T::zero(), T::zero()]
//             );
//             assert_eq!(
//                 rad,
//                 0.7778
//             );

//             let quat = Quat::<f32>::from_axis_angle(&Vec3::<f32>::from_values(T::zero(), T::one(), T::zero()), 0.879546);
//             let (vec3, rad) = quat.axis_angle();
//             assert_eq!(
//                 vec3.raw(),
//                 &[T::zero(), T::one(), T::zero()]
//             );
//             assert_eq!(
//                 rad,
//                 0.879546
//             );

//             let quat = Quat::<f32>::from_axis_angle(&Vec3::<f32>::from_values(T::zero(), T::zero(), T::one()), 0.123456);
//             let (vec3, rad) = quat.axis_angle();
//             assert_eq!(
//                 vec3.raw(),
//                 &[T::zero(), T::zero(), T::one()000055]
//             );
//             assert_eq!(
//                 rad,
//                 0.12345532
//             );

//             let quat = Quat::<f32>::from_axis_angle(&Vec3::<f32>::from_values(0.707106, T::zero(), 0.707106), std::f32::consts::PI * T::from(0.5).unwrap());
//             let (vec3, rad) = quat.axis_angle();
//             assert_eq!(
//                 vec3.raw(),
//                 &[0.707106, T::zero(), 0.707106]
//             );
//             assert_eq!(
//                 rad,
//                 std::f32::consts::PI * T::from(0.5).unwrap()
//             );

//             let quat = Quat::<f32>::from_axis_angle(&Vec3::<f32>::from_values(0.65538555, 0.49153915, T::from(0.5).unwrap()7346237), 8.8888);
//             let (_, rad) = quat.axis_angle();
//             assert_eq!(
//                 quat.raw(),
//                 &[-0.63199174, -0.4739938, -T::from(0.5).unwrap()5299276, -0.2647929]
//             );
//             assert_eq!(
//                 rad > T::zero(),
//                 true
//             );
//             assert_eq!(
//                 rad < std::f32::consts::PI * T::from(2.0).unwrap(),
//                 true
//             );
//         }

//         #[test]
//         fn invert() {
//             assert_eq!(
//                 quat_a().invert().raw(),
//                 &[-T::zero()33333335, -T::zero()6666667, -0.10000001, 0.13333334]
//             );
//         }

//         #[test]
//         fn pow() {
//             // identity quat
//             assert_eq!(
//                 quat_identity().pow(2.1).raw(),
//                 &QUAT_IDENTITY_RAW
//             );

//             // power of one
//             assert_eq!(
//                 quat_a().normalize().pow(T::one()).length(),
//                 T::one()
//             );

//             // squared
//             assert_eq!(
//                 quat_a().normalize().pow(T::from(2.0).unwrap()).raw(),
//                 &[0.26666668, T::from(0.5).unwrap()3333336, 0.8, T::zero()6666667]
//             );
//             assert_eq!(
//                 quat_a().normalize().pow(T::from(2.0).unwrap()).length(),
//                 T::one()
//             );

//             // conjugate
//             assert_eq!(
//                 quat_a().normalize().pow(-T::one()).raw(),
//                 quat_a().normalize().conjugate().raw()
//             );
//             assert_eq!(
//                 quat_a().normalize().pow(-T::one()).length(),
//                 T::one()
//             );

//             // reversible
//             assert_eq!(
//                 quat_a().normalize().pow(2.1).pow(T::one() / 2.1).raw(),
//                 quat_a().normalize().raw()
//             );
//             assert_eq!(
//                 quat_a().normalize().pow(2.1).pow(T::one() / 2.1).length(),
//                 T::one()
//             );
//         }

//         #[test]
//         fn rotate_x() {
//             let quat = quat_identity().rotate_x((9T::zero()f32).to_radians());
//             assert_eq!(
//                 Vec3::<f32>::from_values(T::zero(), T::zero(), -T::one()).transform_quat(&quat).raw(),
//                 &[T::zero(), T::from(0.999999).unwrap()94, -5.9604645e-8]
//             );
//         }

//         #[test]
//         fn rotate_y() {
//             let quat = quat_identity().rotate_y((9T::zero()f32).to_radians());
//             assert_eq!(
//                 Vec3::<f32>::from_values(T::zero(), T::zero(), -T::one()).transform_quat(&quat).raw(),
//                 &[-T::from(0.999999).unwrap()94, T::zero(), -5.9604645e-8]
//             );
//         }

//         #[test]
//         fn rotate_z() {
//             let quat = quat_identity().rotate_z((9T::zero()f32).to_radians());
//             assert_eq!(
//                 Vec3::<f32>::from_values(T::zero(), T::one(), T::zero()).transform_quat(&quat).raw(),
//                 &[-T::from(0.999999).unwrap()94, 5.9604645e-8, T::zero()]
//             );
//         }

//         #[test]
//         fn from_euler() {
//             assert_eq!(
//                 Quat::<f32>::from_euler(-3T::zero(), 3T::zero(), 3T::zero()).raw(),
//                 &[-0.3061862, 0.17677669, 0.3061862, 0.8838835]
//             );

//             let quat = Quat::<f32>::from_euler(-9T::zero(), T::zero(), T::zero());
//             assert_eq!(
//                 Vec3::<f32>::from_values(T::zero(), T::one(), T::zero()).transform_quat(&quat).raw(),
//                 &[T::zero(), 5.9604645e-8, -T::from(0.999999).unwrap()94]
//             );
//         }

//         #[test]
//         fn from_axis_angle() {
//             assert_eq!(
//                 Quat::<f32>::from_axis_angle(
//                     &Vec3::<f32>::from_values(T::one(), T::zero(), T::zero()),
//                     std::f32::consts::PI * T::from(0.5).unwrap(),
//                 ).raw(),
//                 &[0.70710677, T::zero(), T::zero(), 0.70710677]
//             );
//         }
//     }

//     mod f64 {
//         float_test!(
//             f64,
//             crate::EPSILON_F64,
//             std::f64::consts::PI,
//             std::f64::consts::E,
//             std::f64::consts::SQRT_2
//         );

//         #[test]
//         fn from_mat3() -> Result<(), Error> {
//             let mat = Mat3::<f64>::from_values(
//                 T::one(), T::zero(),  T::zero(),
//                 T::zero(), T::zero(), -T::one(),
//                 T::zero(), T::one(),  T::zero(),
//             );
//             let quat = Quat::<f64>::from_mat3(&mat);
//             assert_eq!(
//                 quat.raw(),
//                 &[-0.7071067811865475, T::zero(), T::zero(), 0.7071067811865476]
//             );

//             assert_eq!(
//                 Vec3::<f64>::from_values(T::zero(), T::one(), T::zero()).transform_quat(&quat).raw(),
//                 &[T::zero(), 2.220446049250313e-16, -T::one()]
//             );

//             let mat = Mat3::<f64>::from_mat4(&Mat4::<f64>::from_look_at(
//                 &Vec3::<f64>::from_values(T::zero(), T::zero(), T::zero()),
//                 &Vec3::<f64>::from_values(T::zero(), T::zero(), T::one()),
//                 &Vec3::<f64>::from_values(T::zero(), T::one(), T::zero()),
//             ))
//                 .invert()?
//                 .transpose();
//             let quat = Quat::<f64>::from_mat3(&mat);
//             assert_eq!(
//                 Vec3::<f64>::from_values(3.0, T::from(2.0).unwrap(), -T::one()).transform_quat(&quat.normalize()).raw(),
//                 Vec3::<f64>::from_values(3.0, T::from(2.0).unwrap(), -T::one()).transform_mat3(&mat).raw()
//             );

//             let mat = Mat3::<f64>::from_mat4(&Mat4::<f64>::from_look_at(
//                 &Vec3::<f64>::from_values(T::zero(), T::zero(), T::zero()),
//                 &Vec3::<f64>::from_values(-T::one(), T::zero(), T::zero()),
//                 &Vec3::<f64>::from_values(T::zero(), -T::one(), T::zero()),
//             ))
//                 .invert()?
//                 .transpose();
//             let quat = Quat::<f64>::from_mat3(&mat);
//             assert_eq!(
//                 Vec3::<f64>::from_values(3.0, T::from(2.0).unwrap(), -T::one()).transform_quat(&quat.normalize()).raw(),
//                 &[-T::from(0.999999).unwrap()9999999991, -T::from(2.0).unwrap(), 3.0]
//             );

//             let mat = Mat3::<f64>::from_mat4(&Mat4::<f64>::from_look_at(
//                 &Vec3::<f64>::from_values(T::zero(), T::zero(), T::zero()),
//                 &Vec3::<f64>::from_values(T::zero(), T::zero(), -T::one()),
//                 &Vec3::<f64>::from_values(T::zero(), -T::one(), T::zero()),
//             ))
//                 .invert()?
//                 .transpose();
//             let quat = Quat::<f64>::from_mat3(&mat);
//             assert_eq!(
//                 Vec3::<f64>::from_values(3.0, T::from(2.0).unwrap(), -T::one()).transform_quat(&quat.normalize()).raw(),
//                 Vec3::<f64>::from_values(3.0, T::from(2.0).unwrap(), -T::one()).transform_mat3(&mat).raw()
//             );

//             Ok(())
//         }

//         #[test]
//         fn from_rotate_to() {
//             assert_eq!(
//                 Quat::<f64>::from_rotate_to(
//                     &Vec3::<f64>::from_values(T::zero(), T::one(), T::zero()),
//                     &Vec3::<f64>::from_values(T::one(), T::zero(), T::zero()),
//                 ).raw(),
//                 &[T::zero(), T::zero(), -0.7071067811865475, 0.7071067811865475]
//             );

//             let quat = Quat::<f64>::from_rotate_to(
//                 &Vec3::<f64>::from_values(T::zero(), T::one(), T::zero()),
//                 &Vec3::<f64>::from_values(T::zero(), T::one(), T::zero()),
//             );
//             assert_eq!(
//                 Vec3::<f64>::from_values(T::zero(), T::one(), T::zero()).transform_quat(&quat).raw(),
//                 &[T::zero(), T::one(), T::zero()]
//             );

//             let quat = Quat::<f64>::from_rotate_to(
//                 &Vec3::<f64>::from_values(T::one(), T::zero(), T::zero()),
//                 &Vec3::<f64>::from_values(-T::one(), T::zero(), T::zero()),
//             );
//             assert_eq!(
//                 Vec3::<f64>::from_values(T::one(), T::zero(), T::zero()).transform_quat(&quat).raw(),
//                 &[-T::one(), -1.2246467991473532e-16, T::zero()]
//             );

//             let quat = Quat::<f64>::from_rotate_to(
//                 &Vec3::<f64>::from_values(T::zero(), T::one(), T::zero()),
//                 &Vec3::<f64>::from_values(T::zero(), -T::one(), T::zero()),
//             );
//             assert_eq!(
//                 Vec3::<f64>::from_values(T::zero(), T::one(), T::zero()).transform_quat(&quat).raw(),
//                 &[-1.2246467991473532e-16, -T::one(), T::zero()]
//             );

//             let quat = Quat::<f64>::from_rotate_to(
//                 &Vec3::<f64>::from_values(T::zero(), T::zero(), T::one()),
//                 &Vec3::<f64>::from_values(T::zero(), T::zero(), -T::one()),
//             );
//             assert_eq!(
//                 Vec3::<f64>::from_values(T::zero(), T::zero(), T::one()).transform_quat(&quat).raw(),
//                 &[-1.2246467991473532e-16, T::zero(), -T::one()]
//             );
//         }

//         #[test]
//         fn from_axes() {
//             let quat = Quat::<f64>::from_axes(
//                 &Vec3::<f64>::from_values(-T::one(), T::zero(), T::zero()),
//                 &Vec3::<f64>::from_values(T::zero(), T::zero(), -T::one()),
//                 &Vec3::<f64>::from_values(T::zero(), T::one(), T::zero()),
//             );
//             assert_eq!(
//                 Vec3::<f64>::from_values(T::zero(), T::zero(), -T::one()).transform_quat(&quat).raw(),
//                 &[T::one(), T::zero(), -2.220446049250313e-16]
//             );
//             assert_eq!(
//                 Vec3::<f64>::from_values(T::one(), T::zero(), T::zero()).transform_quat(&quat).raw(),
//                 &[2.220446049250313e-16, T::zero(), T::one()]
//             );

//             let quat = Quat::<f64>::from_axes(
//                 &Vec3::<f64>::from_values(T::zero(), T::zero(), -T::one()),
//                 &Vec3::<f64>::from_values(T::one(), T::zero(), T::zero()),
//                 &Vec3::<f64>::from_values(T::zero(), T::one(), T::zero()),
//             );
//             assert_eq!(
//                 quat.raw(),
//                 &[T::zero(), T::zero(), T::zero(), T::one()]
//             );
//         }

//         #[test]
//         fn angle() {
//             assert_eq!(
//                 quat_a().normalize().angle(&quat_a().normalize()),
//                 4.2146848510894035e-8
//             );

//             // precision consider
//             assert_eq!(
//                 quat_a().normalize().angle(&quat_a().normalize().rotate_x(std::f64::consts::PI / 4.0)),
//                 0.7853981633974491
//             );

//             let quat_a = quat_a().normalize();
//             let quat_b = quat_b().normalize();
//             let dummy = (quat_a.conjugate() * quat_b).axis_angle();
//             assert_eq!(
//                 quat_a.angle(&quat_b),
//                 dummy.1
//             );
//         }

//         #[test]
//         fn axis_angle() {
//             let quat = Quat::<f64>::from_axis_angle(&Vec3::<f64>::from_values(T::zero(), T::one(), T::zero()), T::zero());
//             let (_, rad) = quat.axis_angle();
//             assert_eq!(
//                 rad % (std::f64::consts::PI * T::from(2.0).unwrap()),
//                 T::zero()
//             );

//             let quat = Quat::<f64>::from_axis_angle(&Vec3::<f64>::from_values(T::one(), T::zero(), T::zero()), 0.7778);
//             let (vec3, rad) = quat.axis_angle();
//             assert_eq!(
//                 vec3.raw(),
//                 &[T::one(), T::zero(), T::zero()]
//             );
//             assert_eq!(
//                 rad,
//                 0.7778
//             );

//             let quat = Quat::<f64>::from_axis_angle(&Vec3::<f64>::from_values(T::zero(), T::one(), T::zero()), 0.879546);
//             let (vec3, rad) = quat.axis_angle();
//             assert_eq!(
//                 vec3.raw(),
//                 &[T::zero(), T::one()000000000000002, T::zero()]
//             );
//             assert_eq!(
//                 rad,
//                 0.8795459999999998
//             );

//             let quat = Quat::<f64>::from_axis_angle(&Vec3::<f64>::from_values(T::zero(), T::zero(), T::one()), 0.123456);
//             let (vec3, rad) = quat.axis_angle();
//             assert_eq!(
//                 vec3.raw(),
//                 &[T::zero(), T::zero(), T::one()000000000000138]
//             );
//             assert_eq!(
//                 rad,
//                 0.1234559999999983
//             );

//             let quat = Quat::<f64>::from_axis_angle(&Vec3::<f64>::from_values(0.707106, T::zero(), 0.707106), std::f64::consts::PI * T::from(0.5).unwrap());
//             let (vec3, rad) = quat.axis_angle();
//             assert_eq!(
//                 vec3.raw(),
//                 &[0.707106, T::zero(), 0.707106]
//             );
//             assert_eq!(
//                 rad,
//                 std::f64::consts::PI * T::from(0.5).unwrap()
//             );

//             let quat = Quat::<f64>::from_axis_angle(&Vec3::<f64>::from_values(0.65538555, 0.49153915, T::from(0.5).unwrap()7346237), 8.8888);
//             let (_, rad) = quat.axis_angle();
//             assert_eq!(
//                 quat.raw(),
//                 &[-0.6319917917288304, -0.4739938317428059, -T::from(0.5).unwrap()529928310219251, -0.2647927364605179]
//             );
//             assert_eq!(
//                 rad > T::zero(),
//                 true
//             );
//             assert_eq!(
//                 rad < std::f64::consts::PI * T::from(2.0).unwrap(),
//                 true
//             );
//         }

//         #[test]
//         fn invert() {
//             assert_eq!(
//                 quat_a().invert().raw(),
//                 &[-T::zero()3333333333333333, -T::zero()6666666666666667, -0.1, 0.13333333333333333]
//             );
//         }

//         #[test]
//         fn pow() {
//             // identity quat
//             assert_eq!(
//                 quat_identity().pow(2.1).raw(),
//                 &QUAT_IDENTITY_RAW
//             );

//             // power of one
//             assert_eq!(
//                 quat_a().normalize().pow(T::one()).length(),
//                 T::from(0.999999).unwrap()9999999999
//             );

//             // squared
//             assert_eq!(
//                 quat_a().normalize().pow(T::from(2.0).unwrap()).raw(),
//                 &[0.26666666666666666, T::from(0.5).unwrap()333333333333333, 0.7999999999999999, T::zero()6666666666666689]
//             );
//             assert_eq!(
//                 quat_a().normalize().pow(T::from(2.0).unwrap()).length(),
//                 T::one()
//             );

//             // conjugate
//             assert_eq!(
//                 quat_a().normalize().pow(-T::one()).raw(),
//                 &[-0.1825741858350554, -0.3651483716701108, -T::from(0.5).unwrap()477225575051663, 0.7302967433402217]
//             );
//             assert_eq!(
//                 quat_a().normalize().pow(-T::one()).length(),
//                 T::one()000000000000002
//             );

//             // reversible
//             assert_eq!(
//                 quat_a().normalize().pow(2.1).pow(T::one() / 2.1).raw(),
//                 quat_a().normalize().raw()
//             );
//             assert_eq!(
//                 quat_a().normalize().pow(2.1).pow(T::one() / 2.1).length(),
//                 T::from(0.999999).unwrap()9999999999
//             );
//         }

//         #[test]
//         fn rotate_x() {
//             let quat = quat_identity().rotate_x((9T::zero()f64).to_radians());
//             assert_eq!(
//                 Vec3::<f64>::from_values(T::zero(), T::zero(), -T::one()).transform_quat(&quat).raw(),
//                 &[T::zero(), T::one(), -2.220446049250313e-16]
//             );
//         }

//         #[test]
//         fn rotate_y() {
//             let quat = quat_identity().rotate_y((9T::zero()f64).to_radians());
//             assert_eq!(
//                 Vec3::<f64>::from_values(T::zero(), T::zero(), -T::one()).transform_quat(&quat).raw(),
//                 &[-T::one(), T::zero(), -2.220446049250313e-16]
//             );
//         }

//         #[test]
//         fn rotate_z() {
//             let quat = quat_identity().rotate_z((9T::zero()f64).to_radians());
//             assert_eq!(
//                 Vec3::<f64>::from_values(T::zero(), T::one(), T::zero()).transform_quat(&quat).raw(),
//                 &[-T::one(), 2.220446049250313e-16, T::zero()]
//             );
//         }

//         #[test]
//         fn from_euler() {
//             assert_eq!(
//                 Quat::<f64>::from_euler(-3T::zero(), 3T::zero(), 3T::zero()).raw(),
//                 &[-0.30618621784789724, 0.17677669529663687, 0.30618621784789724, 0.8838834764831845]
//             );

//             let quat = Quat::<f64>::from_euler(-9T::zero(), T::zero(), T::zero());
//             assert_eq!(
//                 Vec3::<f64>::from_values(T::zero(), T::one(), T::zero()).transform_quat(&quat).raw(),
//                 &[T::zero(), 2.220446049250313e-16, -T::one()]
//             );
//         }

//         #[test]
//         fn from_axis_angle() {
//             assert_eq!(
//                 Quat::<f64>::from_axis_angle(
//                     &Vec3::<f64>::from_values(T::one(), T::zero(), T::zero()),
//                     std::f64::consts::PI * T::from(0.5).unwrap(),
//                 ).raw(),
//                 &[0.7071067811865475, T::zero(), T::zero(), 0.7071067811865476]
//             );
//         }
//     }
// }
