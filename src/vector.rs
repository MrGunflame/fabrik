use core::ops::{Add, Mul, Sub};

use glam::{Vec2, Vec3};

/// A vector type.
pub trait Vector:
    Sized
    + Copy
    + Add<Self, Output = Self>
    + Sub<Self, Output = Self>
    + Mul<f32, Output = Self>
    + PartialEq
{
    const NAN: Self;

    fn normalize(self) -> Self;
    fn length(self) -> f32;
    fn distance(self, rhs: Self) -> f32;
}

impl Vector for Vec2 {
    const NAN: Self = Self::NAN;

    fn normalize(self) -> Self {
        self.normalize()
    }

    fn length(self) -> f32 {
        self.length()
    }

    fn distance(self, rhs: Self) -> f32 {
        self.distance(rhs)
    }
}

impl Vector for Vec3 {
    const NAN: Self = Self::NAN;

    fn normalize(self) -> Self {
        self.normalize()
    }

    fn length(self) -> f32 {
        self.length()
    }

    fn distance(self, rhs: Self) -> f32 {
        self.distance(rhs)
    }
}
