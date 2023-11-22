use std::{
    fmt::Display,
    ops::{Add, Mul, Sub},
};

use half::f16;
use num_traits::Float;

use crate::{
    epsilon,
    mat4::AsMat4,
    quat::{AsQuat, Quat},
    vec3::{AsVec3, Vec3},
};

pub trait AsQuat2<T: Float> {
    fn from_values(x1: T, y1: T, z1: T, w1: T, x2: T, y2: T, z2: T, w2: T) -> Self;

    fn x1(&self) -> T;

    fn y1(&self) -> T;

    fn z1(&self) -> T;

    fn w1(&self) -> T;

    fn x2(&self) -> T;

    fn y2(&self) -> T;

    fn z2(&self) -> T;

    fn w2(&self) -> T;

    fn set_x1(&mut self, x1: T) -> &mut Self;

    fn set_y1(&mut self, y1: T) -> &mut Self;

    fn set_z1(&mut self, z1: T) -> &mut Self;

    fn set_w1(&mut self, w1: T) -> &mut Self;

    fn set_x2(&mut self, x2: T) -> &mut Self;

    fn set_y2(&mut self, y2: T) -> &mut Self;

    fn set_z2(&mut self, z2: T) -> &mut Self;

    fn set_w2(&mut self, w2: T) -> &mut Self;

    #[inline(always)]
    fn to_raw(&self) -> [T; 8] {
        [
            self.x1(),
            self.y1(),
            self.z1(),
            self.w1(),
            self.x2(),
            self.y2(),
            self.z2(),
            self.w2(),
        ]
    }

    #[inline(always)]
    fn to_gl(&self) -> [f32; 8] {
        [
            T::to_f32(&self.x1()).unwrap(),
            T::to_f32(&self.y1()).unwrap(),
            T::to_f32(&self.z1()).unwrap(),
            T::to_f32(&self.w1()).unwrap(),
            T::to_f32(&self.x2()).unwrap(),
            T::to_f32(&self.y2()).unwrap(),
            T::to_f32(&self.z2()).unwrap(),
            T::to_f32(&self.w2()).unwrap(),
        ]
    }

    #[inline(always)]
    fn to_gl_binary(&self) -> [u8; 32] {
        unsafe { std::mem::transmute_copy::<[f32; 8], [u8; 32]>(&self.to_gl()) }
    }

    #[inline(always)]
    fn copy<Q: AsQuat2<T> + ?Sized>(&mut self, b: &Q) -> &mut Self {
        self.set_x1(b.x1())
            .set_y1(b.y1())
            .set_z1(b.z1())
            .set_w1(b.w1())
            .set_x2(b.x2())
            .set_y2(b.y2())
            .set_z2(b.z2())
            .set_w2(b.w2())
    }

    #[inline(always)]
    fn real(&self) -> Quat<T> {
        Quat::<T>::from_values(self.x1(), self.y1(), self.z1(), self.w1())
    }

    #[inline(always)]
    fn dual(&self) -> Quat<T> {
        Quat::<T>::from_values(self.x2(), self.y2(), self.z2(), self.w2())
    }

    #[inline(always)]
    fn translation(&self) -> Vec3<T> {
        let bx = -self.x1();
        let by = -self.y1();
        let bz = -self.z1();
        let bw = self.w1();
        let ax = self.x2();
        let ay = self.y2();
        let az = self.z2();
        let aw = self.w2();
        Vec3::<T>::from_values(
            (ax * bw + aw * bx + ay * bz - az * by) * T::from(2.0).unwrap(),
            (ay * bw + aw * by + az * bx - ax * bz) * T::from(2.0).unwrap(),
            (az * bw + aw * bz + ax * by - ay * bx) * T::from(2.0).unwrap(),
        )
    }

    #[inline(always)]
    fn set(&mut self, x1: T, y1: T, z1: T, w1: T, x2: T, y2: T, z2: T, w2: T) -> &mut Self {
        self.set_x1(x1)
            .set_y1(y1)
            .set_z1(z1)
            .set_w1(w1)
            .set_x2(x2)
            .set_y2(y2)
            .set_z2(z2)
            .set_w2(w2)
    }

    #[inline(always)]
    fn set_slice(&mut self, [x1, y1, z1, w1, x2, y2, z2, w2]: &[T; 8]) -> &mut Self {
        self.set_x1(*x1)
            .set_y1(*y1)
            .set_z1(*z1)
            .set_w1(*w1)
            .set_x2(*x2)
            .set_y2(*y2)
            .set_z2(*z2)
            .set_w2(*w2)
    }

    #[inline(always)]
    fn set_zero(&mut self) -> &mut Self {
        self.set_x1(T::zero())
            .set_y1(T::zero())
            .set_z1(T::zero())
            .set_w1(T::zero())
            .set_x2(T::zero())
            .set_y2(T::zero())
            .set_z2(T::zero())
            .set_w2(T::zero())
    }

    #[inline(always)]
    fn set_identify(&mut self) -> &mut Self {
        self.set_x1(T::zero())
            .set_y1(T::zero())
            .set_z1(T::zero())
            .set_w1(T::one())
            .set_x2(T::zero())
            .set_y2(T::zero())
            .set_z2(T::zero())
            .set_w2(T::zero())
    }

    #[inline(always)]
    fn set_real<Q: AsQuat<T> + ?Sized>(&mut self, q: &Q) -> &mut Self {
        self.set_x1(q.x()).set_y1(q.y()).set_z1(q.z()).set_w1(q.w())
    }

    #[inline(always)]
    fn set_dual<Q: AsQuat<T> + ?Sized>(&mut self, q: &Q) -> &mut Self {
        self.set_x2(q.x()).set_y2(q.y()).set_z2(q.z()).set_w2(q.w())
    }

    #[inline(always)]
    fn translate<V: AsVec3<T> + ?Sized>(&self, v: &V) -> Self
    where
        Self: Sized,
    {
        let ax1 = self.x1();
        let ay1 = self.y1();
        let az1 = self.z1();
        let aw1 = self.w1();
        let bx1 = v.x() * T::from(0.5).unwrap();
        let by1 = v.y() * T::from(0.5).unwrap();
        let bz1 = v.z() * T::from(0.5).unwrap();
        let ax2 = self.x2();
        let ay2 = self.y2();
        let az2 = self.z2();
        let aw2 = self.w2();

        Self::from_values(
            ax1,
            ay1,
            az1,
            aw1,
            aw1 * bx1 + ay1 * bz1 - az1 * by1 + ax2,
            aw1 * by1 + az1 * bx1 - ax1 * bz1 + ay2,
            aw1 * bz1 + ax1 * by1 - ay1 * bx1 + az2,
            -ax1 * bx1 - ay1 * by1 - az1 * bz1 + aw2,
        )
    }

    #[inline(always)]
    fn rotate_x(&self, rad: T) -> Self
    where
        Self: Sized,
    {
        let bx = -self.x1();
        let by = -self.y1();
        let bz = -self.z1();
        let bw = self.w1();
        let ax = self.x2();
        let ay = self.y2();
        let az = self.z2();
        let aw = self.w2();
        let ax1 = ax * bw + aw * bx + ay * bz - az * by;
        let ay1 = ay * bw + aw * by + az * bx - ax * bz;
        let az1 = az * bw + aw * bz + ax * by - ay * bx;
        let aw1 = aw * bw - ax * bx - ay * by - az * bz;
        let quat = Quat::from_values(self.x1(), self.y1(), self.z1(), self.w1()).rotate_x(rad);
        let bx = quat.x();
        let by = quat.y();
        let bz = quat.z();
        let bw = quat.w();

        Self::from_values(
            bx,
            by,
            bz,
            bw,
            ax1 * bw + aw1 * bx + ay1 * bz - az1 * by,
            ay1 * bw + aw1 * by + az1 * bx - ax1 * bz,
            az1 * bw + aw1 * bz + ax1 * by - ay1 * bx,
            aw1 * bw - ax1 * bx - ay1 * by - az1 * bz,
        )
    }

    #[inline(always)]
    fn rotate_y(&self, rad: T) -> Self
    where
        Self: Sized,
    {
        let bx = -self.x1();
        let by = -self.y1();
        let bz = -self.z1();
        let bw = self.w1();
        let ax = self.x2();
        let ay = self.y2();
        let az = self.z2();
        let aw = self.w2();
        let ax1 = ax * bw + aw * bx + ay * bz - az * by;
        let ay1 = ay * bw + aw * by + az * bx - ax * bz;
        let az1 = az * bw + aw * bz + ax * by - ay * bx;
        let aw1 = aw * bw - ax * bx - ay * by - az * bz;
        let quat = Quat::from_values(self.x1(), self.y1(), self.z1(), self.w1()).rotate_y(rad);
        let bx = quat.x();
        let by = quat.y();
        let bz = quat.z();
        let bw = quat.w();

        Self::from_values(
            bx,
            by,
            bz,
            bw,
            ax1 * bw + aw1 * bx + ay1 * bz - az1 * by,
            ay1 * bw + aw1 * by + az1 * bx - ax1 * bz,
            az1 * bw + aw1 * bz + ax1 * by - ay1 * bx,
            aw1 * bw - ax1 * bx - ay1 * by - az1 * bz,
        )
    }

    #[inline(always)]
    fn rotate_z(&self, rad: T) -> Self
    where
        Self: Sized,
    {
        let bx = -self.x1();
        let by = -self.y1();
        let bz = -self.z1();
        let bw = self.w1();
        let ax = self.x2();
        let ay = self.y2();
        let az = self.z2();
        let aw = self.w2();
        let ax1 = ax * bw + aw * bx + ay * bz - az * by;
        let ay1 = ay * bw + aw * by + az * bx - ax * bz;
        let az1 = az * bw + aw * bz + ax * by - ay * bx;
        let aw1 = aw * bw - ax * bx - ay * by - az * bz;
        let quat = Quat::from_values(self.x1(), self.y1(), self.z1(), self.w1()).rotate_z(rad);
        let bx = quat.x();
        let by = quat.y();
        let bz = quat.z();
        let bw = quat.w();

        Self::from_values(
            bx,
            by,
            bz,
            bw,
            ax1 * bw + aw1 * bx + ay1 * bz - az1 * by,
            ay1 * bw + aw1 * by + az1 * bx - ax1 * bz,
            az1 * bw + aw1 * bz + ax1 * by - ay1 * bx,
            aw1 * bw - ax1 * bx - ay1 * by - az1 * bz,
        )
    }

    #[inline(always)]
    fn rotate_by_quat_append<Q: AsQuat<T> + ?Sized>(&self, q: &Q) -> Self
    where
        Self: Sized,
    {
        let qx = q.x();
        let qy = q.y();
        let qz = q.z();
        let qw = q.w();
        let mut ax = self.x1();
        let mut ay = self.y1();
        let mut az = self.z1();
        let mut aw = self.w1();
        let x1 = ax * qw + aw * qx + ay * qz - az * qy;
        let y1 = ay * qw + aw * qy + az * qx - ax * qz;
        let z1 = az * qw + aw * qz + ax * qy - ay * qx;
        let w1 = aw * qw - ax * qx - ay * qy - az * qz;

        ax = self.x2();
        ay = self.y2();
        az = self.z2();
        aw = self.w2();
        let x2 = ax * qw + aw * qx + ay * qz - az * qy;
        let y2 = ay * qw + aw * qy + az * qx - ax * qz;
        let z2 = az * qw + aw * qz + ax * qy - ay * qx;
        let w2 = aw * qw - ax * qx - ay * qy - az * qz;

        Self::from_values(x1, y1, z1, w1, x2, y2, z2, w2)
    }

    #[inline(always)]
    fn rotate_by_quat_prepend<Q: AsQuat<T> + ?Sized>(&self, q: &Q) -> Self
    where
        Self: Sized,
    {
        let qx = q.x();
        let qy = q.y();
        let qz = q.z();
        let qw = q.w();
        let mut bx = self.x1();
        let mut by = self.y1();
        let mut bz = self.z1();
        let mut bw = self.w1();
        let x1 = qx * bw + qw * bx + qy * bz - qz * by;
        let y1 = qy * bw + qw * by + qz * bx - qx * bz;
        let z1 = qz * bw + qw * bz + qx * by - qy * bx;
        let w1 = qw * bw - qx * bx - qy * by - qz * bz;

        bx = self.x2();
        by = self.y2();
        bz = self.z2();
        bw = self.w2();
        let x2 = qx * bw + qw * bx + qy * bz - qz * by;
        let y2 = qy * bw + qw * by + qz * bx - qx * bz;
        let z2 = qz * bw + qw * bz + qx * by - qy * bx;
        let w2 = qw * bw - qx * bx - qy * by - qz * bz;

        Self::from_values(x1, y1, z1, w1, x2, y2, z2, w2)
    }

    #[inline(always)]
    fn rotate_around_axis<V: AsVec3<T> + ?Sized>(&self, axis: &V, rad: T) -> Self
    where
        Self: Sized,
    {
        if rad.abs() < epsilon() {
            return Self::from_values(
                self.x1(),
                self.y1(),
                self.z1(),
                self.w1(),
                self.x2(),
                self.y2(),
                self.z2(),
                self.w2(),
            );
        }

        let axis_length = (axis.x() * axis.x() + axis.y() * axis.y() + axis.z() * axis.z()).sqrt();
        let rad = rad * T::from(0.5).unwrap();
        let s = rad.sin();
        let bx = (s * axis.x()) / axis_length;
        let by = (s * axis.y()) / axis_length;
        let bz = (s * axis.z()) / axis_length;
        let bw = rad.cos();

        let ax1 = self.x1();
        let ay1 = self.y1();
        let az1 = self.z1();
        let aw1 = self.w1();
        let x1 = ax1 * bw + aw1 * bx + ay1 * bz - az1 * by;
        let y1 = ay1 * bw + aw1 * by + az1 * bx - ax1 * bz;
        let z1 = az1 * bw + aw1 * bz + ax1 * by - ay1 * bx;
        let w1 = aw1 * bw - ax1 * bx - ay1 * by - az1 * bz;

        let ax = self.x2();
        let ay = self.y2();
        let az = self.z2();
        let aw = self.w2();
        let x2 = ax * bw + aw * bx + ay * bz - az * by;
        let y2 = ay * bw + aw * by + az * bx - ax * bz;
        let z2 = az * bw + aw * bz + ax * by - ay * bx;
        let w2 = aw * bw - ax * bx - ay * by - az * bz;

        Self::from_values(x1, y1, z1, w1, x2, y2, z2, w2)
    }

    #[inline(always)]
    fn scale(&self, scale: T) -> Self
    where
        Self: Sized,
    {
        let x1 = self.x1();
        let y1 = self.y1();
        let z1 = self.z1();
        let w1 = self.w1();
        let x2 = self.x2();
        let y2 = self.y2();
        let z2 = self.z2();
        let w2 = self.w2();

        Self::from_values(
            x1 * scale,
            y1 * scale,
            z1 * scale,
            w1 * scale,
            x2 * scale,
            y2 * scale,
            z2 * scale,
            w2 * scale,
        )
    }

    #[inline(always)]
    fn dot<Q: AsQuat2<T> + ?Sized>(&self, b: &Q) -> T {
        self.x1() * b.x1() + self.y1() * b.y1() + self.z1() * b.z1() + self.w1() * b.w1()
    }

    #[inline(always)]
    fn lerp(&self, b: &Self, t: T) -> Self
    where
        Self: Sized,
    {
        let mt = T::one() - t;
        let t = if self.dot(b) < T::zero() { -t } else { t };

        let bx = self.x1();
        let by = self.y1();
        let bz = self.z1();
        let bw = self.w1();
        let ax = self.x2();
        let ay = self.y2();
        let az = self.z2();
        let aw = self.w2();

        Self::from_values(
            bx * mt + b.x1() * t,
            by * mt + b.y1() * t,
            bz * mt + b.z1() * t,
            bw * mt + b.w1() * t,
            ax * mt + b.x2() * t,
            ay * mt + b.y2() * t,
            az * mt + b.z2() * t,
            aw * mt + b.w2() * t,
        )
    }

    #[inline(always)]
    fn squared_length(&self) -> T {
        let x = self.x1();
        let y = self.y1();
        let z = self.z1();
        let w = self.w1();
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
        let mut magnitude = self.squared_length();
        if magnitude > T::zero() {
            magnitude = magnitude.sqrt();
        }

        let a0 = self.x1() / magnitude;
        let a1 = self.y1() / magnitude;
        let a2 = self.z1() / magnitude;
        let a3 = self.w1() / magnitude;

        let b0 = self.x2();
        let b1 = self.y2();
        let b2 = self.z2();
        let b3 = self.w2();

        let a_dot_b = a0 * b0 + a1 * b1 + a2 * b2 + a3 * b3;

        Self::from_values(
            a0,
            a1,
            a2,
            a3,
            (b0 - a0 * a_dot_b) / magnitude,
            (b1 - a1 * a_dot_b) / magnitude,
            (b2 - a2 * a_dot_b) / magnitude,
            (b3 - a3 * a_dot_b) / magnitude,
        )
    }

    #[inline(always)]
    fn invert(&self) -> Self
    where
        Self: Sized,
    {
        let sqlen = self.squared_length();

        let bx = self.x1();
        let by = self.y1();
        let bz = self.z1();
        let bw = self.w1();
        let ax = self.x2();
        let ay = self.y2();
        let az = self.z2();
        let aw = self.w2();

        Self::from_values(
            -bx / sqlen,
            -by / sqlen,
            -bz / sqlen,
            bw / sqlen,
            -ax / sqlen,
            -ay / sqlen,
            -az / sqlen,
            aw / sqlen,
        )
    }

    #[inline(always)]
    fn conjugate(&self) -> Self
    where
        Self: Sized,
    {
        let bx = self.x1();
        let by = self.y1();
        let bz = self.z1();
        let bw = self.w1();
        let ax = self.x2();
        let ay = self.y2();
        let az = self.z2();
        let aw = self.w2();

        Self::from_values(-bx, -by, -bz, bw, -ax, -ay, -az, aw)
    }

    #[inline(always)]
    fn approximate_eq<Q: AsQuat2<T> + ?Sized>(&self, b: &Q) -> bool {
        let a0 = self.x1();
        let a1 = self.y1();
        let a2 = self.z1();
        let a3 = self.w1();
        let a4 = self.x2();
        let a5 = self.y2();
        let a6 = self.z2();
        let a7 = self.w2();
        let b0 = b.x1();
        let b1 = b.y1();
        let b2 = b.z1();
        let b3 = b.w1();
        let b4 = b.x2();
        let b5 = b.y2();
        let b6 = b.z2();
        let b7 = b.w2();

        (a0 - b0).abs() <= epsilon::<T>() * T::one().max(a0.abs()).max(b0.abs())
            && (a1 - b1).abs() <= epsilon::<T>() * T::one().max(a1.abs()).max(b1.abs())
            && (a2 - b2).abs() <= epsilon::<T>() * T::one().max(a2.abs()).max(b2.abs())
            && (a3 - b3).abs() <= epsilon::<T>() * T::one().max(a3.abs()).max(b3.abs())
            && (a4 - b4).abs() <= epsilon::<T>() * T::one().max(a4.abs()).max(b4.abs())
            && (a5 - b5).abs() <= epsilon::<T>() * T::one().max(a5.abs()).max(b5.abs())
            && (a6 - b6).abs() <= epsilon::<T>() * T::one().max(a6.abs()).max(b6.abs())
            && (a7 - b7).abs() <= epsilon::<T>() * T::one().max(a7.abs()).max(b7.abs())
    }
}

impl<T: Float> AsQuat2<T> for [T; 8] {
    #[inline(always)]
    fn from_values(x1: T, y1: T, z1: T, w1: T, x2: T, y2: T, z2: T, w2: T) -> Self {
        [x1, y1, z1, w1, x2, y2, z2, w2]
    }

    #[inline(always)]
    fn x1(&self) -> T {
        self[0]
    }

    #[inline(always)]
    fn y1(&self) -> T {
        self[1]
    }

    #[inline(always)]
    fn z1(&self) -> T {
        self[2]
    }

    #[inline(always)]
    fn w1(&self) -> T {
        self[3]
    }

    #[inline(always)]
    fn x2(&self) -> T {
        self[4]
    }

    #[inline(always)]
    fn y2(&self) -> T {
        self[5]
    }

    #[inline(always)]
    fn z2(&self) -> T {
        self[6]
    }

    #[inline(always)]
    fn w2(&self) -> T {
        self[7]
    }

    #[inline(always)]
    fn set_x1(&mut self, x1: T) -> &mut Self {
        self[0] = x1;
        self
    }

    #[inline(always)]
    fn set_y1(&mut self, y1: T) -> &mut Self {
        self[1] = y1;
        self
    }

    #[inline(always)]
    fn set_z1(&mut self, z1: T) -> &mut Self {
        self[2] = z1;
        self
    }

    #[inline(always)]
    fn set_w1(&mut self, w1: T) -> &mut Self {
        self[3] = w1;
        self
    }

    #[inline(always)]
    fn set_x2(&mut self, x2: T) -> &mut Self {
        self[4] = x2;
        self
    }

    #[inline(always)]
    fn set_y2(&mut self, y2: T) -> &mut Self {
        self[5] = y2;
        self
    }

    #[inline(always)]
    fn set_z2(&mut self, z2: T) -> &mut Self {
        self[6] = z2;
        self
    }

    #[inline(always)]
    fn set_w2(&mut self, w2: T) -> &mut Self {
        self[7] = w2;
        self
    }
}

impl<T: Float> AsQuat2<T> for (T, T, T, T, T, T, T, T) {
    #[inline(always)]
    fn from_values(x1: T, y1: T, z1: T, w1: T, x2: T, y2: T, z2: T, w2: T) -> Self {
        (x1, y1, z1, w1, x2, y2, z2, w2)
    }

    #[inline(always)]
    fn x1(&self) -> T {
        self.0
    }

    #[inline(always)]
    fn y1(&self) -> T {
        self.1
    }

    #[inline(always)]
    fn z1(&self) -> T {
        self.2
    }

    #[inline(always)]
    fn w1(&self) -> T {
        self.3
    }

    #[inline(always)]
    fn x2(&self) -> T {
        self.4
    }

    #[inline(always)]
    fn y2(&self) -> T {
        self.5
    }

    #[inline(always)]
    fn z2(&self) -> T {
        self.6
    }

    #[inline(always)]
    fn w2(&self) -> T {
        self.7
    }

    #[inline(always)]
    fn set_x1(&mut self, x1: T) -> &mut Self {
        self.0 = x1;
        self
    }

    #[inline(always)]
    fn set_y1(&mut self, y1: T) -> &mut Self {
        self.1 = y1;
        self
    }

    #[inline(always)]
    fn set_z1(&mut self, z1: T) -> &mut Self {
        self.2 = z1;
        self
    }

    #[inline(always)]
    fn set_w1(&mut self, w1: T) -> &mut Self {
        self.3 = w1;
        self
    }

    #[inline(always)]
    fn set_x2(&mut self, x2: T) -> &mut Self {
        self.4 = x2;
        self
    }

    #[inline(always)]
    fn set_y2(&mut self, y2: T) -> &mut Self {
        self.5 = y2;
        self
    }

    #[inline(always)]
    fn set_z2(&mut self, z2: T) -> &mut Self {
        self.6 = z2;
        self
    }

    #[inline(always)]
    fn set_w2(&mut self, w2: T) -> &mut Self {
        self.7 = w2;
        self
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Quat2<T = f64>(pub [T; 8]);

impl<T: Float> Quat2<T> {
    #[inline(always)]
    pub fn raw(&self) -> &[T; 8] {
        &self.0
    }

    #[inline(always)]
    pub fn new() -> Self {
        Self([T::zero(); 8])
    }

    #[inline(always)]
    pub fn new_identity() -> Self {
        Self([
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
        ])
    }

    #[inline(always)]
    pub fn from_slice(slice: &[T; 8]) -> Self {
        Self(slice.clone())
    }

    #[inline(always)]
    pub fn from_rotation_translation_values(
        x1: T,
        y1: T,
        z1: T,
        w1: T,
        x2: T,
        y2: T,
        z2: T,
    ) -> Self {
        let ax = x2 * T::from(0.5).unwrap();
        let ay = y2 * T::from(0.5).unwrap();
        let az = z2 * T::from(0.5).unwrap();
        Self([
            x1,
            y1,
            z1,
            w1,
            ax * w1 + ay * z1 - az * y1,
            ay * w1 + az * x1 - ax * z1,
            az * w1 + ax * y1 - ay * x1,
            -ax * x1 - ay * y1 - az * z1,
        ])
    }

    #[inline(always)]
    pub fn from_rotation_translation<Q, V>(q: &Q, t: &V) -> Self
    where
        Q: AsQuat<T> + ?Sized,
        V: AsVec3<T> + ?Sized,
    {
        let ax = t.x() * T::from(0.5).unwrap();
        let ay = t.y() * T::from(0.5).unwrap();
        let az = t.z() * T::from(0.5).unwrap();
        let bx = q.x();
        let by = q.y();
        let bz = q.z();
        let bw = q.w();
        Self([
            bx,
            by,
            bz,
            bw,
            ax * bw + ay * bz - az * by,
            ay * bw + az * bx - ax * bz,
            az * bw + ax * by - ay * bx,
            -ax * bx - ay * by - az * bz,
        ])
    }

    #[inline(always)]
    pub fn from_translation<V: AsVec3<T> + ?Sized>(t: &V) -> Self {
        Self([
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
            t.x() * T::from(0.5).unwrap(),
            t.y() * T::from(0.5).unwrap(),
            t.z() * T::from(0.5).unwrap(),
            T::zero(),
        ])
    }

    #[inline(always)]
    pub fn from_rotation<Q: AsQuat<T> + ?Sized>(q: &Q) -> Self {
        Self([
            q.x(),
            q.y(),
            q.z(),
            q.w(),
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
        ])
    }

    #[inline(always)]
    pub fn from_mat4<M: AsMat4<T> + ?Sized>(a: &M) -> Self {
        let outer = a.rotation();
        let t = a.translation();
        Self::from_rotation_translation(&outer, &t)
    }
}

impl<T: Float> AsQuat2<T> for Quat2<T> {
    #[inline(always)]
    fn from_values(x1: T, y1: T, z1: T, w1: T, x2: T, y2: T, z2: T, w2: T) -> Self {
        Self([x1, y1, z1, w1, x2, y2, z2, w2])
    }

    #[inline(always)]
    fn x1(&self) -> T {
        self.0[0]
    }

    #[inline(always)]
    fn y1(&self) -> T {
        self.0[1]
    }

    #[inline(always)]
    fn z1(&self) -> T {
        self.0[2]
    }

    #[inline(always)]
    fn w1(&self) -> T {
        self.0[3]
    }

    #[inline(always)]
    fn x2(&self) -> T {
        self.0[4]
    }

    #[inline(always)]
    fn y2(&self) -> T {
        self.0[5]
    }

    #[inline(always)]
    fn z2(&self) -> T {
        self.0[6]
    }

    #[inline(always)]
    fn w2(&self) -> T {
        self.0[7]
    }

    #[inline(always)]
    fn set_x1(&mut self, x1: T) -> &mut Self {
        self.0[0] = x1;
        self
    }

    #[inline(always)]
    fn set_y1(&mut self, y1: T) -> &mut Self {
        self.0[1] = y1;
        self
    }

    #[inline(always)]
    fn set_z1(&mut self, z1: T) -> &mut Self {
        self.0[2] = z1;
        self
    }

    #[inline(always)]
    fn set_w1(&mut self, w1: T) -> &mut Self {
        self.0[3] = w1;
        self
    }

    #[inline(always)]
    fn set_x2(&mut self, x2: T) -> &mut Self {
        self.0[4] = x2;
        self
    }

    #[inline(always)]
    fn set_y2(&mut self, y2: T) -> &mut Self {
        self.0[5] = y2;
        self
    }

    #[inline(always)]
    fn set_z2(&mut self, z2: T) -> &mut Self {
        self.0[6] = z2;
        self
    }

    #[inline(always)]
    fn set_w2(&mut self, w2: T) -> &mut Self {
        self.0[7] = w2;
        self
    }
}

impl<T: Float, Q: AsQuat2<T>> Add<Q> for Quat2<T> {
    type Output = Self;

    #[inline(always)]
    fn add(self, b: Q) -> Self::Output {
        Self([
            self.0[0] + b.x1(),
            self.0[1] + b.y1(),
            self.0[2] + b.z1(),
            self.0[3] + b.w1(),
            self.0[4] + b.x2(),
            self.0[5] + b.y2(),
            self.0[6] + b.z2(),
            self.0[7] + b.w2(),
        ])
    }
}

impl<T: Float, Q: AsQuat2<T>> Sub<Q> for Quat2<T> {
    type Output = Self;

    #[inline(always)]
    fn sub(self, b: Q) -> Self::Output {
        Self([
            self.0[0] - b.x1(),
            self.0[1] - b.y1(),
            self.0[2] - b.z1(),
            self.0[3] - b.w1(),
            self.0[4] - b.x2(),
            self.0[5] - b.y2(),
            self.0[6] - b.z2(),
            self.0[7] - b.w2(),
        ])
    }
}

impl<T: Float, Q: AsQuat2<T>> Mul<Q> for Quat2<T> {
    type Output = Self;

    #[inline(always)]
    fn mul(self, b: Q) -> Self::Output {
        let ax0 = self.0[0];
        let ay0 = self.0[1];
        let az0 = self.0[2];
        let aw0 = self.0[3];
        let bx1 = b.x2();
        let by1 = b.y2();
        let bz1 = b.z2();
        let bw1 = b.w2();
        let ax1 = self.0[4];
        let ay1 = self.0[5];
        let az1 = self.0[6];
        let aw1 = self.0[7];
        let bx0 = b.x1();
        let by0 = b.y1();
        let bz0 = b.z1();
        let bw0 = b.w1();

        Self([
            ax0 * bw0 + aw0 * bx0 + ay0 * bz0 - az0 * by0,
            ay0 * bw0 + aw0 * by0 + az0 * bx0 - ax0 * bz0,
            az0 * bw0 + aw0 * bz0 + ax0 * by0 - ay0 * bx0,
            aw0 * bw0 - ax0 * bx0 - ay0 * by0 - az0 * bz0,
            ax0 * bw1 + aw0 * bx1 + ay0 * bz1 - az0 * by1 + ax1 * bw0 + aw1 * bx0 + ay1 * bz0
                - az1 * by0,
            ay0 * bw1 + aw0 * by1 + az0 * bx1 - ax0 * bz1 + ay1 * bw0 + aw1 * by0 + az1 * bx0
                - ax1 * bz0,
            az0 * bw1 + aw0 * bz1 + ax0 * by1 - ay0 * bx1 + az1 * bw0 + aw1 * bz0 + ax1 * by0
                - ay1 * bx0,
            aw0 * bw1 - ax0 * bx1 - ay0 * by1 - az0 * bz1 + aw1 * bw0
                - ax1 * bx0
                - ay1 * by0
                - az1 * bz0,
        ])
    }
}

macro_rules! float_implementations {
    ($($float: tt),+) => {
        $(
            impl Mul<$float> for Quat2<$float> {
                type Output = Self;

                #[inline(always)]
                fn mul(self, b: $float) -> Self::Output {
                    Self([
                        self.0[0] * b,
                        self.0[1] * b,
                        self.0[2] * b,
                        self.0[3] * b,
                        self.0[4] * b,
                        self.0[5] * b,
                        self.0[6] * b,
                        self.0[7] * b,
                    ])
                }
            }
        )+
    };
}

float_implementations!(f16, f32, f64);

impl<T> AsRef<Quat2<T>> for Quat2<T> {
    fn as_ref(&self) -> &Self {
        self
    }
}

impl<T> AsRef<[T]> for Quat2<T> {
    fn as_ref(&self) -> &[T] {
        &self.0
    }
}

impl<T: Float> Default for Quat2<T> {
    fn default() -> Self {
        Self::new_identity()
    }
}

impl<T: Display> Display for Quat2<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let value = self
            .0
            .iter()
            .map(|v| v.to_string())
            .collect::<Vec<_>>()
            .join(", ");
        f.write_fmt(format_args!("quat2({})", value))
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        error::Error,
        quat::{AsQuat, Quat},
        quat2::AsQuat2,
        vec3::{AsVec3, Vec3},
    };

    use super::Quat2;

    fn quat2_a() -> Quat2 {
        Quat2::from_values(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0)
    }

    fn quat2_b() -> Quat2 {
        Quat2::from_values(5.0, 6.0, 7.0, 8.0, 9.0, 8.0, 6.0, -4.0)
    }

    #[test]
    fn new() {
        assert_eq!(
            Quat2::<f64>::new().to_raw(),
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        );
    }

    #[test]
    fn from_slice() {
        assert_eq!(
            Quat2::from_slice(&[3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]).to_raw(),
            [3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
        );
    }

    #[test]
    fn from_values() {
        assert_eq!(
            Quat2::from_values(3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0).to_raw(),
            [3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
        );
    }

    #[test]
    fn from_rotation() {
        assert_eq!(
            Quat2::from_rotation(&(1.0, 2.0, 3.0, 4.0)).to_raw(),
            [1.0, 2.0, 3.0, 4.0, 0.0, 0.0, 0.0, 0.0]
        );
    }

    #[test]
    fn from_translation() {
        assert_eq!(
            Quat2::from_translation(&(1.0, 2.0, 3.0)).to_raw(),
            [0.0, 0.0, 0.0, 1.0, 0.5, 1.0, 1.5, 0.0]
        );
    }

    #[test]
    fn from_rotation_translation() {
        assert_eq!(
            Quat2::from_rotation_translation(&(1.0, 2.0, 3.0, 4.0), &(1.0, 2.0, 3.0)).to_raw(),
            [1.0, 2.0, 3.0, 4.0, 2.0, 4.0, 6.0, -7.0]
        );
    }

    #[test]
    fn from_rotation_translation_values() {
        assert_eq!(
            Quat2::from_rotation_translation_values(1.0, 2.0, 3.0, 4.0, 1.0, 2.0, 3.0).to_raw(),
            [1.0, 2.0, 3.0, 4.0, 2.0, 4.0, 6.0, -7.0]
        );
    }

    #[test]
    fn scale() {
        assert_eq!(
            (quat2_a() * 2.0).to_raw(),
            [2.0, 4.0, 6.0, 8.0, 4.0, 10.0, 12.0, -4.0]
        );
    }

    #[test]
    fn scale_add() {
        assert_eq!(
            (quat2_a() + quat2_b() * 0.5).to_raw(),
            [3.5, 5.0, 6.5, 8.0, 6.5, 9.0, 9.0, -4.0]
        );
    }

    #[test]
    fn rotate_by_quat_append() {
        let quat2_rotation = Quat2::from_values(2.0, 5.0, 2.0, -10.0, 0.0, 0.0, 0.0, 0.0);

        assert_eq!(
            quat2_a()
                .rotate_by_quat_append(&(2.0, 5.0, 2.0, -10.0))
                .to_raw(),
            (quat2_a() * quat2_rotation).to_raw()
        );
    }

    #[test]
    fn rotate_by_quat_prepend() {
        let quat2_rotation = Quat2::from_values(2.0, 5.0, 2.0, -10.0, 0.0, 0.0, 0.0, 0.0);

        assert_eq!(
            quat2_a()
                .rotate_by_quat_prepend(&quat2_rotation.real())
                .to_raw(),
            (quat2_rotation * quat2_a()).to_raw()
        );
    }

    #[test]
    fn squared_length() {
        assert_eq!(quat2_a().squared_length(), 30.0);
    }

    #[test]
    fn length() {
        assert_eq!(quat2_a().length(), 5.477225575051661);
    }

    #[test]
    fn dot() {
        assert_eq!(quat2_a().dot(&quat2_b()), 70.0);
    }

    #[test]
    fn conjugate() {
        assert_eq!(
            quat2_a().conjugate().to_raw(),
            [-1.0, -2.0, -3.0, 4.0, -2.0, -5.0, -6.0, -2.0]
        );
    }

    #[test]
    fn real() {
        assert_eq!(quat2_a().real().to_raw(), [1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn dual() {
        assert_eq!(quat2_a().dual().to_raw(), [2.0, 5.0, 6.0, -2.0]);
    }

    #[test]
    fn set() {
        let mut quat = Quat::new();
        quat.set(3.0, 4.0, 5.0, 6.0);

        assert_eq!(quat.to_raw(), [3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn set_slice() {
        let mut quat = Quat::new();
        quat.set_slice(&[3.0, 4.0, 5.0, 6.0]);

        assert_eq!(quat.to_raw(), [3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn set_real() {
        assert_eq!(
            quat2_a()
                .clone()
                .set_real(&(4.0, 6.0, 8.0, -100.0))
                .to_raw(),
            [4.0, 6.0, 8.0, -100.0, 2.0, 5.0, 6.0, -2.0]
        );
    }

    #[test]
    fn set_dual() {
        let mut quat = Quat::new();
        quat.set_slice(&[3.0, 4.0, 5.0, 6.0]);

        assert_eq!(
            quat2_a()
                .clone()
                .set_dual(&(4.3, 6.0, 8.0, -100.0))
                .to_raw(),
            [1.0, 2.0, 3.0, 4.0, 4.3, 6.0, 8.0, -100.0]
        );
    }

    #[test]
    fn add() {
        assert_eq!(
            (quat2_a() + quat2_b()).to_raw(),
            [6.0, 8.0, 10.0, 12.0, 11.0, 13.0, 12.0, -6.0]
        );
    }

    #[test]
    fn mul() {
        assert_eq!(
            (quat2_a() * quat2_b()).to_raw(),
            [24.0, 48.0, 48.0, -6.0, 25.0, 89.0, 23.0, -157.0]
        );
    }

    #[test]
    fn mul_scalar() {
        assert_eq!(
            (quat2_a() * 2.0).to_raw(),
            [2.0, 4.0, 6.0, 8.0, 4.0, 10.0, 12.0, -4.0]
        );
    }

    #[test]
    fn mul_scalar_add() {
        assert_eq!(
            (quat2_a() + quat2_b() * 0.5).to_raw(),
            [3.5, 5.0, 6.5, 8.0, 6.5, 9.0, 9.0, -4.0]
        );
    }

    #[test]
    fn approximate_eq() {
        let quat2_a = Quat2::from_values(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0);
        let quat2_b = Quat2::from_values(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0);
        let quat2_c = Quat2::from_values(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0);
        let quat2_d = Quat2::from_values(1e-16, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0);

        assert_eq!(true, quat2_a.approximate_eq(&quat2_b));
        assert_eq!(false, quat2_a.approximate_eq(&quat2_c));
        assert_eq!(true, quat2_a.approximate_eq(&quat2_d));
    }

    #[test]
    fn display() {
        let out = quat2_a().to_string();
        assert_eq!(out, "quat2(1, 2, 3, 4, 2, 5, 6, -2)");
    }

    #[test]
    fn from_mat4() {
        let quat_rotation = Quat::from_values(1.0, 2.0, 3.0, 4.0).normalize();
        let quat2_a =
            Quat2::from_rotation_translation(&quat_rotation, &(1.0, -5.0, 3.0)).normalize();

        assert_eq!(
            quat2_a.to_raw(),
            [
                0.18257418583505539,
                0.36514837167011077,
                0.5477225575051662,
                0.7302967433402215,
                -1.5518805795979707,
                -1.8257418583505538,
                1.734454765433026,
                0.0
            ],
        );
    }

    #[test]
    fn invert() {
        assert_eq!(
            quat2_a().invert().to_raw(),
            [
                -0.03333333333333333,
                -0.06666666666666667,
                -0.1,
                0.13333333333333333,
                -0.06666666666666667,
                -0.16666666666666666,
                -0.2,
                -0.06666666666666667
            ]
        );
        assert_eq!(
            quat2_a().invert().real().to_raw(),
            [
                -0.03333333333333333,
                -0.06666666666666667,
                -0.1,
                0.13333333333333333
            ],
        );
    }

    #[test]
    fn lerp() {
        assert_eq!(
            quat2_a().lerp(&quat2_b(), 0.7).to_raw(),
            [3.8, 4.799999999999999, 5.8, 6.8, 6.9, 7.1, 6.0, -3.4]
        );
    }

    #[test]
    fn normalize() {
        assert_eq!(
            Quat2::from_values(1.0, 2.0, 3.0, 4.0, 2.0, 5.0, 6.0, -2.0)
                .normalize()
                .to_raw(),
            [
                0.18257418583505536,
                0.3651483716701107,
                0.5477225575051661,
                0.7302967433402214,
                0.23126063539107017,
                0.6450954566171957,
                0.6937819061732106,
                -0.900699316786273
            ]
        );
        assert_eq!(
            Quat2::from_values(5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                .normalize()
                .to_raw(),
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        );
        assert_eq!(
            Quat2::from_values(5.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 5.0)
                .normalize()
                .to_raw(),
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.4, 0.6, 1.0]
        );
    }

    #[test]
    fn rotate_around_axis() -> Result<(), Error> {
        let quat2_a =
            Quat2::from_rotation_translation(&(1.0, 2.0, 3.0, 4.0), &(-5.0, 4.0, 10.0)).normalize();

        let result = quat2_a.rotate_around_axis(&(1.0, 4.0, 2.0), 5.0);

        assert_eq!(
            result.to_raw(),
            [
                -0.24164294714846665,
                0.11280662947202075,
                -0.20036742052872042,
                -0.9427280876431084,
                1.3920522306902268,
                -3.594589462350351,
                -4.512371117598661,
                0.17211647582839384
            ]
        );

        Ok(())
    }

    #[test]
    fn rotate_x() {
        let quat2_a =
            Quat2::from_rotation_translation(&(1.0, 2.0, 3.0, 4.0), &(-5.0, 4.0, 10.0)).normalize();

        let result = quat2_a.rotate_x(5.0);

        assert_eq!(
            result.to_raw(),
            [
                0.2907941144735252,
                0.0352604066733514,
                -0.6573355589457817,
                -0.6943381378364758,
                0.2448721933328691,
                -1.5780446006697788,
                -4.141429934812809,
                3.943142267566019
            ]
        );
    }

    #[test]
    fn rotate_y() {
        let quat2_a =
            Quat2::from_rotation_translation(&(1.0, 2.0, 3.0, 4.0), &(-5.0, 4.0, 10.0)).normalize();

        let result = quat2_a.rotate_y(5.0);

        assert_eq!(
            result.to_raw(),
            [
                -0.4740648367096534,
                0.14452597112809118,
                -0.32953886558156226,
                -0.8036037022912156,
                0.6273016689244582,
                -4.801378752084603,
                -3.431203765857,
                0.17348029387749575
            ]
        );
    }

    #[test]
    fn rotate_z() {
        let quat2_a =
            Quat2::from_rotation_translation(&(1.0, 2.0, 3.0, 4.0), &(-5.0, 4.0, 10.0)).normalize();

        let result = quat2_a.rotate_y(5.0);

        assert_eq!(
            result.to_raw(),
            [
                -0.4740648367096534,
                0.14452597112809118,
                -0.32953886558156226,
                -0.8036037022912156,
                0.6273016689244582,
                -4.801378752084603,
                -3.431203765857,
                0.17348029387749575
            ]
        );
    }

    #[test]
    fn translation() {
        assert_eq!(
            Quat2::from_translation(&Vec3::from_values(1.0, 2.0, 3.0))
                .translation()
                .to_raw(),
            [1.0, 2.0, 3.0]
        );
        assert_eq!(
            Quat2::from_translation(&Vec3::from_values(1.0, 2.0, 3.0))
                .translation()
                .normalize()
                .to_raw(),
            [0.2672612419124244, 0.5345224838248488, 0.8017837257372732]
        );
        assert_ne!(
            Quat2::from_rotation_translation(&(2.0, 4.0, 6.0, 2.0), &(1.0, 2.0, 3.0))
                .translation()
                .to_raw(),
            [1.0, 2.0, 3.0]
        );
        assert_eq!(
            Quat2::from_rotation_translation(&(2.0, 4.0, 6.0, 2.0), &(1.0, 2.0, 3.0))
                .normalize()
                .translation()
                .to_raw(),
            [0.9999999999999998, 1.9999999999999998, 3.0]
        );
    }

    #[test]
    fn translate() {
        let quat2_a =
            Quat2::from_rotation_translation(&(1.0, 2.0, 3.0, 4.0), &(-5.0, 4.0, 10.0)).normalize();

        let result = quat2_a.translate(&(1.0, 1.0, -1.0));

        assert_eq!(
            result.to_raw(),
            [
                0.18257418583505536,
                0.3651483716701107,
                0.5477225575051661,
                0.7302967433402214,
                -2.647325694608303,
                4.473067552958856,
                1.9170289512680818,
                -3.0124740662784135
            ]
        );
    }
}
