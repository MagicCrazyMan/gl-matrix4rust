use std::{
    fmt::Display,
    ops::{Add, Mul},
};

use half::f16;
use num_traits::{Float, FloatConst};

use crate::{
    epsilon,
    mat3::AsMat3,
    vec3::{AsVec3, Vec3},
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EulerOrder {
    XYZ,
    XZY,
    YZX,
    YXZ,
    ZXY,
    ZYX,
}

pub trait AsQuat<T: Float> {
    fn from_values(x: T, y: T, z: T, w: T) -> Self;

    fn x(&self) -> T;

    fn y(&self) -> T;

    fn z(&self) -> T;

    fn w(&self) -> T;

    fn set_x(&mut self, x: T) -> &mut Self;

    fn set_y(&mut self, y: T) -> &mut Self;

    fn set_z(&mut self, z: T) -> &mut Self;

    fn set_w(&mut self, w: T) -> &mut Self;

    #[inline(always)]
    fn to_raw(&self) -> [T; 4] {
        [self.x(), self.y(), self.z(), self.w()]
    }

    #[inline(always)]
    fn to_gl(&self) -> [f32; 4] {
        [
            T::to_f32(&self.x()).unwrap(),
            T::to_f32(&self.y()).unwrap(),
            T::to_f32(&self.z()).unwrap(),
            T::to_f32(&self.w()).unwrap(),
        ]
    }

    #[inline(always)]
    fn to_gl_binary(&self) -> [u8; 16] {
        unsafe { std::mem::transmute_copy::<[f32; 4], [u8; 16]>(&self.to_gl()) }
    }

    #[inline(always)]
    fn copy<Q: AsQuat<T> + ?Sized>(&mut self, b: &Q) -> &mut Self {
        self.set_x(b.x()).set_y(b.y()).set_z(b.z()).set_w(b.w())
    }

    #[inline(always)]
    fn set(&mut self, x: T, y: T, z: T, w: T) -> &mut Self {
        self.set_x(x).set_y(y).set_z(z).set_w(w)
    }

    #[inline(always)]
    fn set_slice(&mut self, [x, y, z, w]: &[T; 4]) -> &mut Self {
        self.set_x(*x).set_y(*y).set_z(*z).set_w(*w)
    }

    #[inline(always)]
    fn set_zero(&mut self) -> &mut Self {
        self.set_x(T::zero())
            .set_y(T::zero())
            .set_z(T::zero())
            .set_w(T::zero())
    }

    #[inline(always)]
    fn set_identify(&mut self) -> &mut Self {
        self.set_x(T::zero())
            .set_y(T::zero())
            .set_z(T::zero())
            .set_w(T::one())
    }

    #[inline(always)]
    fn axis_angle(&self) -> (Vec3<T>, T) {
        let rad = self.w().acos() * T::from(2.0).unwrap();
        let s = (rad / T::from(2.0).unwrap()).sin();
        if s > epsilon() {
            (
                Vec3::<T>::from_values(self.x() / s, self.y() / s, self.z() / s),
                rad,
            )
        } else {
            // If s is zero, return any axis (no rotation - axis does not matter)
            (Vec3::<T>::from_values(T::one(), T::zero(), T::zero()), rad)
        }
    }

    #[inline(always)]
    fn angle<Q: AsQuat<T> + ?Sized>(&self, b: &Q) -> T {
        let dotproduct = self.dot(b);
        (T::from(2.0).unwrap() * dotproduct * dotproduct - T::one()).acos()
    }

    #[inline(always)]
    fn rotate_x(&self, rad: T) -> Self
    where
        Self: Sized,
    {
        let rad = rad * T::from(0.5).unwrap();

        let ax = self.x();
        let ay = self.y();
        let az = self.z();
        let aw = self.w();
        let bx = rad.sin();
        let bw = rad.cos();

        Self::from_values(
            ax * bw + aw * bx,
            ay * bw + az * bx,
            az * bw - ay * bx,
            aw * bw - ax * bx,
        )
    }

    #[inline(always)]
    fn rotate_y(&self, rad: T) -> Self
    where
        Self: Sized,
    {
        let rad = rad * T::from(0.5).unwrap();

        let ax = self.x();
        let ay = self.y();
        let az = self.z();
        let aw = self.w();
        let by = rad.sin();
        let bw = rad.cos();

        Self::from_values(
            ax * bw - az * by,
            ay * bw + aw * by,
            az * bw + ax * by,
            aw * bw - ay * by,
        )
    }

    #[inline(always)]
    fn rotate_z(&self, rad: T) -> Self
    where
        Self: Sized,
    {
        let rad = rad * T::from(0.5).unwrap();

        let ax = self.x();
        let ay = self.y();
        let az = self.z();
        let aw = self.w();
        let bz = rad.sin();
        let bw = rad.cos();

        Self::from_values(
            ax * bw + ay * bz,
            ay * bw - ax * bz,
            az * bw + aw * bz,
            aw * bw - az * bz,
        )
    }

    #[inline(always)]
    fn calculate_w(&self) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();
        let z = self.z();

        Self::from_values(x, y, z, (T::one() - x * x - y * y - z * z).abs().sqrt())
    }

    #[inline(always)]
    fn exp(&self) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();
        let z = self.z();
        let w = self.w();

        let r = (x * x + y * y + z * z).sqrt();
        let et = w.exp();
        let s = if r > T::zero() {
            (et * r.sin()) / r
        } else {
            T::zero()
        };

        Self::from_values(x * s, y * s, z * s, et * r.cos())
    }

    #[inline(always)]
    fn ln(&self) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();
        let z = self.z();
        let w = self.w();

        let r = (x * x + y * y + z * z).sqrt();
        let t = if r > T::zero() {
            r.atan2(w) / r
        } else {
            T::zero()
        };

        Self::from_values(
            x * t,
            y * t,
            z * t,
            T::from(0.5).unwrap() * (x * x + y * y + z * z + w * w).ln(),
        )
    }

    #[inline(always)]
    fn scale(&self, scale: T) -> Self
    where
        Self: Sized,
    {
        let x = self.x();
        let y = self.y();
        let z = self.z();
        let w = self.w();

        Self::from_values(x * scale, y * scale, z * scale, w * scale)
    }

    #[inline(always)]
    fn pow(&self, b: T) -> Self
    where
        Self: Sized,
    {
        self.ln().scale(b).exp()
    }

    #[inline(always)]
    fn dot<Q: AsQuat<T> + ?Sized>(&self, b: &Q) -> T {
        let ax = self.x();
        let ay = self.y();
        let az = self.z();
        let aw = self.w();
        let bx = b.x();
        let by = b.y();
        let bz = b.z();
        let bw = b.w();

        ax * bx + ay * by + az * bz + aw * bw
    }

    #[inline(always)]
    fn lerp<Q: AsQuat<T> + ?Sized>(&self, b: &Q, t: T) -> Self
    where
        Self: Sized,
    {
        let ax = self.x();
        let ay = self.y();
        let az = self.z();
        let aw = self.w();
        let bx = b.x();
        let by = b.y();
        let bz = b.z();
        let bw = b.w();

        Self::from_values(
            ax + t * (bx - ax),
            ay + t * (by - ay),
            az + t * (bz - az),
            aw + t * (bw - aw),
        )
    }

    #[inline(always)]
    fn slerp<Q: AsQuat<T> + ?Sized>(&self, b: &Q, t: T) -> Self
    where
        Self: Sized,
    {
        let ax = self.x();
        let ay = self.y();
        let az = self.z();
        let aw = self.w();
        let mut bx = b.x();
        let mut by = b.y();
        let mut bz = b.z();
        let mut bw = b.w();

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

        Self::from_values(
            scale0 * ax + scale1 * bx,
            scale0 * ay + scale1 * by,
            scale0 * az + scale1 * bz,
            scale0 * aw + scale1 * bw,
        )
    }

    #[inline(always)]
    fn sqlerp<Q1, Q2, Q3>(&self, b: &Q1, c: &Q2, d: &Q3, t: T) -> Self
    where
        Self: Sized,
        Q1: AsQuat<T> + ?Sized,
        Q2: AsQuat<T> + ?Sized,
        Q3: AsQuat<T> + ?Sized,
    {
        let tmp1 = self.slerp(d, t);
        let mut tmp2 = [T::zero(); 4];
        tmp2.copy(b).slerp(c, t);
        tmp1.slerp(&tmp2, T::from(2.0).unwrap() * t * (T::one() - t))
    }

    #[inline(always)]
    fn squared_length(&self) -> T {
        let x = self.x();
        let y = self.y();
        let z = self.z();
        let w = self.w();
        x * x + y * y + z * z + w * w
    }

    #[inline(always)]
    fn length(&self) -> T {
        self.squared_length().sqrt()
    }

    #[inline(always)]
    fn normalize(&self) -> Self
    where
        Self: Sized,
    {
        let mut len = self.squared_length();
        if len > T::zero() {
            len = T::one() / len.sqrt();
        }

        let x = self.x();
        let y = self.y();
        let z = self.z();
        let w = self.w();

        Self::from_values(x * len, y * len, z * len, w * len)
    }

    #[inline(always)]
    fn invert(&self) -> Self
    where
        Self: Sized,
    {
        let a0 = self.x();
        let a1 = self.y();
        let a2 = self.z();
        let a3 = self.w();

        let dot = a0 * a0 + a1 * a1 + a2 * a2 + a3 * a3;
        let inv_dot = if dot != T::zero() {
            T::one() / dot
        } else {
            T::zero()
        };

        Self::from_values(-a0 * inv_dot, -a1 * inv_dot, -a2 * inv_dot, a3 * inv_dot)
    }

    #[inline(always)]
    fn conjugate(&self) -> Self
    where
        Self: Sized,
    {
        let a0 = self.x();
        let a1 = self.y();
        let a2 = self.z();
        let a3 = self.w();

        Self::from_values(-a0, -a1, -a2, a3)
    }

    #[inline(always)]
    fn approximate_eq<Q: AsQuat<T> + ?Sized>(&self, b: &Q) -> bool {
        self.dot(b).abs() >= T::one() - epsilon()
    }
}

impl<T: Float> AsQuat<T> for [T; 4] {
    #[inline(always)]
    fn from_values(x: T, y: T, z: T, w: T) -> Self {
        [x, y, z, w]
    }

    #[inline(always)]
    fn x(&self) -> T {
        self[0]
    }

    #[inline(always)]
    fn y(&self) -> T {
        self[1]
    }

    #[inline(always)]
    fn z(&self) -> T {
        self[2]
    }

    #[inline(always)]
    fn w(&self) -> T {
        self[3]
    }

    #[inline(always)]
    fn set_x(&mut self, x: T) -> &mut Self {
        self[0] = x;
        self
    }

    #[inline(always)]
    fn set_y(&mut self, y: T) -> &mut Self {
        self[1] = y;
        self
    }

    #[inline(always)]
    fn set_z(&mut self, z: T) -> &mut Self {
        self[2] = z;
        self
    }

    #[inline(always)]
    fn set_w(&mut self, w: T) -> &mut Self {
        self[3] = w;
        self
    }
}

impl<T: Float> AsQuat<T> for (T, T, T, T) {
    #[inline(always)]
    fn from_values(x: T, y: T, z: T, w: T) -> Self {
        (x, y, z, w)
    }

    #[inline(always)]
    fn x(&self) -> T {
        self.0
    }

    #[inline(always)]
    fn y(&self) -> T {
        self.1
    }

    #[inline(always)]
    fn z(&self) -> T {
        self.2
    }

    #[inline(always)]
    fn w(&self) -> T {
        self.3
    }

    #[inline(always)]
    fn set_x(&mut self, x: T) -> &mut Self {
        self.0 = x;
        self
    }

    #[inline(always)]
    fn set_y(&mut self, y: T) -> &mut Self {
        self.1 = y;
        self
    }

    #[inline(always)]
    fn set_z(&mut self, z: T) -> &mut Self {
        self.2 = z;
        self
    }

    #[inline(always)]
    fn set_w(&mut self, w: T) -> &mut Self {
        self.3 = w;
        self
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Quat<T = f64>(pub [T; 4]);

impl<T: Float> Quat<T> {
    #[inline(always)]
    pub fn raw(&self) -> &[T; 4] {
        &self.0
    }

    #[inline(always)]
    pub fn new() -> Self {
        Self([T::zero(); 4])
    }

    #[inline(always)]
    pub fn new_identity() -> Self {
        Self([T::zero(), T::zero(), T::zero(), T::one()])
    }

    #[inline(always)]
    pub fn from_slice(slice: &[T; 4]) -> Self {
        Self(slice.clone())
    }

    #[inline(always)]
    pub fn from_axis_angle<V: AsVec3<T> + ?Sized>(axis: &V, rad: T) -> Self {
        let rad = rad * T::from(0.5).unwrap();
        let s = rad.sin();

        Self([s * axis.x(), s * axis.y(), s * axis.z(), rad.cos()])
    }

    #[inline(always)]
    pub fn from_axes<V1, V2, V3>(view: &V1, right: &V2, up: &V3) -> Self
    where
        V1: AsVec3<T> + ?Sized,
        V2: AsVec3<T> + ?Sized,
        V3: AsVec3<T> + ?Sized,
    {
        let matr = (
            right.x(),
            up.x(),
            -view.x(),
            right.y(),
            up.y(),
            -view.y(),
            right.z(),
            up.z(),
            -view.z(),
        );

        Self::from_mat3(&matr).normalize()
    }

    #[inline(always)]
    pub fn from_mat3<M: AsMat3<T> + ?Sized>(m: &M) -> Self {
        let mut out = Self::new();

        let m = m.to_raw();

        // Algorithm in Ken Shoemake's article in 1987 SIGGRAPH course notes
        // article "Quaternion Calculus and Fast Animation".
        let f_trace = m[0] + m[4] + m[8];
        let mut f_root;

        if f_trace > T::zero() {
            // |w| > 1/2, may as well choose w > 1/2
            f_root = (f_trace + T::one()).sqrt(); // 2w
            out.0[3] = T::from(0.5).unwrap() * f_root;
            f_root = T::from(0.5).unwrap() / f_root; // 1/(4w)
            out.0[0] = (m[5] - m[7]) * f_root;
            out.0[1] = (m[6] - m[2]) * f_root;
            out.0[2] = (m[1] - m[3]) * f_root;
        } else {
            // |w| <= 1/2
            let mut i = 0;
            if m[4] > m[0] {
                i = 1
            };
            if m[8] > m[i * 3 + i] {
                i = 2
            };
            let j = (i + 1) % 3;
            let k = (i + 2) % 3;

            f_root = (m[i * 3 + i] - m[j * 3 + j] - m[k * 3 + k] + T::one()).sqrt();
            out.0[i] = T::from(0.5).unwrap() * f_root;
            f_root = T::from(0.5).unwrap() / f_root;
            out.0[3] = (m[j * 3 + k] - m[k * 3 + j]) * f_root;
            out.0[j] = (m[j * 3 + i] + m[i * 3 + j]) * f_root;
            out.0[k] = (m[k * 3 + i] + m[i * 3 + k]) * f_root;
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
    pub fn from_rotation_to<V1, V2>(a: &V1, b: &V2) -> Self
    where
        V1: AsVec3<T> + ?Sized,
        V2: AsVec3<T> + ?Sized,
    {
        let dot = a.dot(b);
        if dot < T::from(-0.999999).unwrap() {
            let x_unit = [T::one(), T::zero(), T::zero()];

            let mut tmp = x_unit.cross(a);
            if tmp.length() < T::from(0.00001).unwrap() {
                let y_unit = [T::zero(), T::one(), T::zero()];
                tmp = y_unit.cross(a);
            };

            Self::from_axis_angle(&tmp.normalize(), T::PI())
        } else if dot > T::from(0.999999).unwrap() {
            Self([T::zero(), T::zero(), T::zero(), T::one()])
        } else {
            let tmp = a.to_raw().cross(b);
            Self([tmp.x(), tmp.y(), tmp.z(), T::one() + dot]).normalize()
        }
    }
}

impl<T: Float> AsQuat<T> for Quat<T> {
    #[inline(always)]
    fn from_values(x: T, y: T, z: T, w: T) -> Self {
        Self([x, y, z, w])
    }

    #[inline(always)]
    fn x(&self) -> T {
        self.0[0]
    }

    #[inline(always)]
    fn y(&self) -> T {
        self.0[1]
    }

    #[inline(always)]
    fn z(&self) -> T {
        self.0[2]
    }

    #[inline(always)]
    fn w(&self) -> T {
        self.0[3]
    }

    #[inline(always)]
    fn set_x(&mut self, x: T) -> &mut Self {
        self.0[0] = x;
        self
    }

    #[inline(always)]
    fn set_y(&mut self, y: T) -> &mut Self {
        self.0[1] = y;
        self
    }

    #[inline(always)]
    fn set_z(&mut self, z: T) -> &mut Self {
        self.0[2] = z;
        self
    }

    #[inline(always)]
    fn set_w(&mut self, w: T) -> &mut Self {
        self.0[3] = w;
        self
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

impl Quat<f16> {
    #[inline(always)]
    pub fn random(&self) -> Self {
        let u1 = rand::random::<f16>();
        let u2 = rand::random::<f16>();
        let u3 = rand::random::<f16>();
        let sqrt1_minus_u1 = (f16::from_f32_const(1.0) - u1).sqrt();
        let sqrt_u1 = u1.sqrt();
        Self([
            sqrt1_minus_u1 * (f16::from_f32_const(2.0) * f16::PI * u2).sin(),
            sqrt1_minus_u1 * (f16::from_f32_const(2.0) * f16::PI * u2).cos(),
            sqrt_u1 * (f16::from_f32_const(2.0) * f16::PI * u3).sin(),
            sqrt_u1 * (f16::from_f32_const(2.0) * f16::PI * u3).cos(),
        ])
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

impl<T> AsRef<Quat<T>> for Quat<T> {
    fn as_ref(&self) -> &Self {
        self
    }
}

impl<T> AsRef<[T]> for Quat<T> {
    fn as_ref(&self) -> &[T] {
        &self.0
    }
}

impl<T: Float> Default for Quat<T> {
    fn default() -> Self {
        Self::new_identity()
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

#[cfg(test)]
mod tests {
    use std::sync::OnceLock;

    use crate::{
        error::Error,
        mat3::{AsMat3, Mat3},
        mat4::Mat4,
        quat::AsQuat,
        vec3::{AsVec3, Vec3},
    };

    use super::Quat;

    static QUAT_A_RAW: [f64; 4] = [1.0, 2.0, 3.0, 4.0];
    static QUAT_B_RAW: [f64; 4] = [5.0, 6.0, 7.0, 8.0];
    static QUAT_IDENTITY_RAW: [f64; 4] = [0.0, 0.0, 0.0, 1.0];

    static QUAT_A: OnceLock<Quat> = OnceLock::new();
    static QUAT_B: OnceLock<Quat> = OnceLock::new();
    static QUAT_IDENTITY: OnceLock<Quat> = OnceLock::new();

    fn quat_a() -> &'static Quat {
        QUAT_A.get_or_init(|| Quat::from_slice(&QUAT_A_RAW))
    }

    fn quat_b() -> &'static Quat {
        QUAT_B.get_or_init(|| Quat::from_slice(&QUAT_B_RAW))
    }

    fn quat_identity() -> &'static Quat {
        QUAT_IDENTITY.get_or_init(|| Quat::from_slice(&QUAT_IDENTITY_RAW))
    }

    #[test]
    fn new() {
        assert_eq!(Quat::<f64>::new().to_raw(), [0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn new_identity() {
        assert_eq!(Quat::<f64>::new_identity().to_raw(), [0.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn from_slice() {
        assert_eq!(
            Quat::from_slice(&[3.0, 4.0, 5.0, 6.0]).to_raw(),
            [3.0, 4.0, 5.0, 6.0]
        );
    }

    #[test]
    fn from_values() {
        assert_eq!(
            Quat::from_values(3.0, 4.0, 5.0, 6.0).to_raw(),
            [3.0, 4.0, 5.0, 6.0]
        );
    }

    #[test]
    fn scale() {
        assert_eq!((*quat_a() * 2.0).to_raw(), [2.0, 4.0, 6.0, 8.0]);
    }

    #[test]
    fn scale_add() {
        assert_eq!((*quat_a() + *quat_b() * 0.5).to_raw(), [3.5, 5.0, 6.5, 8.0]);
    }

    #[test]
    fn squared_length() {
        assert_eq!(quat_a().squared_length(), 30.0);
    }

    #[test]
    fn length() {
        assert_eq!(quat_a().length(), 5.477225575051661);
    }

    #[test]
    fn normalize() {
        assert_eq!(
            Quat::from_values(5.0, 0.0, 0.0, 0.0).normalize().to_raw(),
            [1.0, 0.0, 0.0, 0.0]
        );
    }

    #[test]
    fn dot() {
        assert_eq!(quat_a().dot(quat_b()), 70.0);
    }

    #[test]
    fn lerp() {
        assert_eq!(quat_a().lerp(quat_b(), 0.5).to_raw(), [3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn slerp() {
        assert_eq!(quat_a().slerp(quat_b(), 0.5).to_raw(), [3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn conjugate() {
        assert_eq!(quat_a().conjugate().to_raw(), [-1.0, -2.0, -3.0, 4.0]);
    }

    #[test]
    fn set() {
        let mut mat = Quat::new();
        mat.set(3.0, 4.0, 5.0, 6.0);

        assert_eq!(mat.to_raw(), [3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn set_slice() {
        let mut mat = Quat::new();
        mat.set_slice(&[3.0, 4.0, 5.0, 6.0]);

        assert_eq!(mat.to_raw(), [3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn add() {
        assert_eq!((*quat_a() + *quat_b()).to_raw(), [6.0, 8.0, 10.0, 12.0]);
    }

    #[test]
    fn mul() {
        assert_eq!((*quat_a() * *quat_b()).to_raw(), [24.0, 48.0, 48.0, -6.0]);
    }

    #[test]
    fn mul_scalar() {
        assert_eq!((*quat_a() * 2.0).to_raw(), [2.0, 4.0, 6.0, 8.0]);
    }

    #[test]
    fn mul_scalar_add() {
        assert_eq!((*quat_a() + *quat_b() * 0.5).to_raw(), [3.5, 5.0, 6.5, 8.0]);
    }

    #[test]
    fn approximate_eq() {
        let vec_a = Quat::from_values(0.0, 0.0, 0.0, 1.0);
        let vec_b = Quat::from_values(0.0, 0.0, 0.0, 1.0);
        let vec_c = Quat::from_values(0.0, 1.0, 0.0, 0.0);
        let vec_d = Quat::from_values(1e-16, 1.0, 2.0, 3.0);
        let vec_e = Quat::from_values(0.0, -1.0, 0.0, 0.0);

        assert_eq!(true, vec_a.approximate_eq(&vec_b));
        assert_eq!(false, vec_a.approximate_eq(&vec_c));
        assert_eq!(true, vec_a.approximate_eq(&vec_d));
        assert_eq!(true, vec_c.approximate_eq(&vec_e));
    }

    #[test]
    fn display() {
        let out = quat_a().to_string();
        assert_eq!(out, "quat(1, 2, 3, 4)");
    }

    #[test]
    fn from_mat3() -> Result<(), Error> {
        let mat = Mat3::from_values(1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0);
        let quat = Quat::from_mat3(&mat);
        assert_eq!(
            quat.to_raw(),
            [-0.7071067811865475, 0.0, 0.0, 0.7071067811865476]
        );

        assert_eq!(
            Vec3::from_values(0.0, 1.0, 0.0)
                .transform_quat(&quat)
                .to_raw(),
            [0.0, 2.220446049250313e-16, -1.0]
        );

        let mat = Mat3::from_mat4(&Mat4::from_look_at(
            &(0.0, 0.0, 0.0),
            &(0.0, 0.0, 1.0),
            &(0.0, 1.0, 0.0),
        ))
        .invert()?
        .transpose();
        let quat = Quat::from_mat3(&mat).normalize();
        assert_eq!(
            Vec3::from_values(3.0, 2.0, -1.0)
                .transform_quat(&quat)
                .to_raw(),
            Vec3::from_values(3.0, 2.0, -1.0)
                .transform_mat3(&mat)
                .to_raw()
        );

        let mat = Mat3::from_mat4(&Mat4::from_look_at(
            &(0.0, 0.0, 0.0),
            &(-1.0, 0.0, 0.0),
            &(0.0, -1.0, 0.0),
        ))
        .invert()?
        .transpose();
        let quat = Quat::from_mat3(&mat).normalize();
        assert_eq!(
            Vec3::from_values(3.0, 2.0, -1.0)
                .transform_quat(&quat)
                .to_raw(),
            [-0.9999999999999991, -2.0, 3.0]
        );

        let mat = Mat3::from_mat4(&Mat4::from_look_at(
            &(0.0, 0.0, 0.0),
            &(0.0, 0.0, -1.0),
            &(0.0, -1.0, 0.0),
        ))
        .invert()?
        .transpose();
        let quat = Quat::from_mat3(&mat).normalize();
        assert_eq!(
            Vec3::from_values(3.0, 2.0, -1.0)
                .transform_quat(&quat)
                .to_raw(),
            Vec3::from_values(3.0, 2.0, -1.0)
                .transform_mat3(&mat)
                .to_raw()
        );

        Ok(())
    }

    #[test]
    fn from_rotate_to() {
        assert_eq!(
            Quat::from_rotation_to(&(0.0, 1.0, 0.0), &(1.0, 0.0, 0.0),).to_raw(),
            [0.0, 0.0, -0.7071067811865475, 0.7071067811865475]
        );

        let quat = Quat::from_rotation_to(&(0.0, 1.0, 0.0), &(0.0, 1.0, 0.0));
        assert_eq!(
            Vec3::from_values(0.0, 1.0, 0.0)
                .transform_quat(&quat)
                .to_raw(),
            [0.0, 1.0, 0.0]
        );

        let quat = Quat::from_rotation_to(&(1.0, 0.0, 0.0), &(-1.0, 0.0, 0.0));
        assert_eq!(
            Vec3::from_values(1.0, 0.0, 0.0)
                .transform_quat(&quat)
                .to_raw(),
            [-1.0, -1.2246467991473532e-16, 0.0]
        );

        let quat = Quat::from_rotation_to(&(0.0, 1.0, 0.0), &(0.0, -1.0, 0.0));
        assert_eq!(
            Vec3::from_values(0.0, 1.0, 0.0)
                .transform_quat(&quat)
                .to_raw(),
            [-1.2246467991473532e-16, -1.0, 0.0]
        );

        let quat = Quat::from_rotation_to(&(0.0, 0.0, 1.0), &(0.0, 0.0, -1.0));
        assert_eq!(
            Vec3::from_values(0.0, 0.0, 1.0)
                .transform_quat(&quat)
                .to_raw(),
            [-1.2246467991473532e-16, 0.0, -1.0]
        );
    }

    #[test]
    fn from_axes() {
        let quat = Quat::from_axes(&(-1.0, 0.0, 0.0), &(0.0, 0.0, -1.0), &(0.0, 1.0, 0.0));
        assert_eq!(
            Vec3::from_values(0.0, 0.0, -1.0)
                .transform_quat(&quat)
                .to_raw(),
            [1.0, 0.0, -2.220446049250313e-16]
        );
        assert_eq!(
            Vec3::from_values(1.0, 0.0, 0.0)
                .transform_quat(&quat)
                .to_raw(),
            [2.220446049250313e-16, 0.0, 1.0]
        );

        let quat = Quat::from_axes(&(0.0, 0.0, -1.0), &(1.0, 0.0, 0.0), &(0.0, 1.0, 0.0));
        assert_eq!(quat.to_raw(), [-0.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn angle() {
        assert_eq!(
            quat_a().normalize().angle(&quat_a().normalize()),
            4.2146848510894035e-8
        );

        // precision consider
        assert_eq!(
            quat_a()
                .normalize()
                .angle(&quat_a().normalize().rotate_x(std::f64::consts::PI / 4.0)),
            0.7853981633974491
        );

        // precision consider
        let quat_a = quat_a().normalize();
        let quat_b = quat_b().normalize();
        assert_eq!(quat_a.angle(&quat_b), 0.5003918408450226);
    }

    #[test]
    fn axis_angle() {
        let quat = Quat::from_axis_angle(&(0.0, 1.0, 0.0), 0.0);
        let (_, rad) = quat.axis_angle();
        assert_eq!(rad % (std::f64::consts::PI * 2.0), 0.0);

        let quat = Quat::from_axis_angle(&(1.0, 0.0, 0.0), 0.7778);
        let (vec3, rad) = quat.axis_angle();
        assert_eq!(vec3.to_raw(), [1.0, 0.0, 0.0]);
        assert_eq!(rad, 0.7778);

        let quat = Quat::from_axis_angle(&(0.0, 1.0, 0.0), 0.879546);
        let (vec3, rad) = quat.axis_angle();
        assert_eq!(vec3.to_raw(), [0.0, 1.0000000000000002, 0.0]);
        assert_eq!(rad, 0.8795459999999998);

        let quat = Quat::from_axis_angle(&(0.0, 0.0, 1.0), 0.123456);
        let (vec3, rad) = quat.axis_angle();
        assert_eq!(vec3.to_raw(), [0.0, 0.0, 1.0000000000000138]);
        assert_eq!(rad, 0.1234559999999983);

        let quat = Quat::from_axis_angle(&(0.707106, 0.0, 0.707106), std::f64::consts::PI * 0.5);
        let (vec3, rad) = quat.axis_angle();
        assert_eq!(vec3.to_raw(), [0.707106, 0.0, 0.707106]);
        assert_eq!(rad, std::f64::consts::PI * 0.5);

        let quat = Quat::from_axis_angle(&(0.65538555, 0.49153915, 0.57346237), 8.8888);
        let (_, rad) = quat.axis_angle();
        assert_eq!(
            quat.to_raw(),
            [-0.63199174, -0.4739938, -0.55299276, -0.2647929]
        );
        assert_eq!(rad > 0.0, true);
        assert_eq!(rad < std::f32::consts::PI * 2.0, true);
    }

    #[test]
    fn invert() {
        assert_eq!(
            quat_a().invert().to_raw(),
            [
                -0.03333333333333333,
                -0.06666666666666667,
                -0.1,
                0.13333333333333333
            ]
        );
    }

    #[test]
    fn pow() {
        // identity quat
        assert_eq!(quat_identity().pow(2.1).to_raw(), QUAT_IDENTITY_RAW);

        // power of one
        assert_eq!(quat_a().normalize().pow(1.0).length(), 0.9999999999999999);

        // squared
        assert_eq!(
            quat_a().normalize().pow(2.0).to_raw(),
            [
                0.26666666666666666,
                0.5333333333333333,
                0.7999999999999999,
                0.06666666666666689
            ]
        );
        assert_eq!(quat_a().normalize().pow(2.0).length(), 1.0);

        // conjugate
        assert_eq!(
            quat_a()
                .normalize()
                .pow(-1.0)
                .to_raw()
                .approximate_eq(quat_a().normalize().conjugate().raw()),
            true
        );
        assert_eq!(quat_a().normalize().pow(-1.0).length(), 1.0000000000000002);

        // reversible
        assert_eq!(
            quat_a().normalize().pow(2.1).pow(1.0 / 2.1).to_raw(),
            quat_a().normalize().to_raw()
        );
        assert_eq!(
            quat_a().normalize().pow(2.1).pow(1.0 / 2.1).length(),
            0.9999999999999999
        );
    }

    #[test]
    fn rotate_x() {
        let quat = quat_identity().rotate_x(90.0f64.to_radians());

        assert_eq!(
            Vec3::from_values(0.0, 0.0, -1.0)
                .transform_quat(&quat)
                .to_raw(),
            [0.0, 1.0, -2.220446049250313e-16]
        );
    }

    #[test]
    fn rotate_y() {
        let quat = quat_identity().rotate_y(90.0f64.to_radians());

        assert_eq!(
            Vec3::from_values(0.0, 0.0, -1.0)
                .transform_quat(&quat)
                .to_raw(),
            [-1.0, 0.0, -2.220446049250313e-16]
        );
    }

    #[test]
    fn rotate_z() {
        let quat = quat_identity().rotate_z(90.0f64.to_radians());

        assert_eq!(
            Vec3::from_values(0.0, 1.0, 0.0)
                .transform_quat(&quat)
                .to_raw(),
            [-1.0, 2.220446049250313e-16, 0.0]
        );
    }

    #[test]
    fn from_euler() {
        assert_eq!(
            Quat::from_euler(-30.0, 30.0, 30.0).to_raw(),
            [
                -0.30618621784789724,
                0.17677669529663687,
                0.30618621784789724,
                0.8838834764831845
            ]
        );

        let quat = Quat::from_euler(-90.0, 0.0, 0.0);
        assert_eq!(
            Vec3::from_values(0.0, 1.0, 0.0)
                .transform_quat(&quat)
                .to_raw(),
            [0.0, 2.220446049250313e-16, -1.0]
        );
    }

    #[test]
    fn from_axis_angle() {
        assert_eq!(
            Quat::from_axis_angle(&(1.0, 0.0, 0.0), std::f64::consts::PI * 0.5,).to_raw(),
            [0.7071067811865475, 0.0, 0.0, 0.7071067811865476]
        );
    }
}
