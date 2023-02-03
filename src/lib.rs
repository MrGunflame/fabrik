use std::ops::{Deref, DerefMut};

use glam::Vec2;

#[derive(Clone, Debug)]
pub struct Solver {
    segments: Segments,
    tolerance: f32,
}

impl Solver {
    pub fn new<T>(segments: T) -> Self
    where
        T: IntoIterator<Item = Joint>,
    {
        let mut segments: Vec<_> = segments
            .into_iter()
            .map(|j| Segment {
                is_edge: false,
                length: j.length,
                translation: j.translation,
            })
            .collect();

        if let Some(seg) = segments.get_mut(0) {
            seg.is_edge = true;
        }

        Self {
            segments: Segments(segments),
            tolerance: 0.01,
        }
    }

    pub fn solve(&mut self, goal: Vec2) -> Option<Vec2> {
        // Distance is greater than the length of all bones combined.
        // The point is unreachable.
        if Vec2::length(goal - self.segments[0].translation).abs() > self.segments.length() {
            return None;
        }

        let mut actual = Vec2::NAN;

        // Assign the last point to the end effector.
        let last = self.segments.last_mut().unwrap();
        last.is_edge = true;
        last.translation = goal;

        let mut iteration = 0;

        loop {
            // Backwards
            for (b, a) in self.segments.backwards() {
                let delta = Vec2::normalize(a.translation - b.translation) * a.length;

                let new_a = b.translation + delta;

                // Don't overwrite the starting point.
                if !a.is_edge {
                    a.translation = new_a;
                }
            }

            // Forwards
            for (a, b) in self.segments.forwards() {
                let delta = Vec2::normalize(b.translation - a.translation) * a.length;

                let new_b = a.translation + delta;

                if !b.is_edge {
                    b.translation = new_b;
                } else {
                    // Catch potential deadlocks when the point doesn't change.
                    debug_assert_ne!(actual, new_b);

                    // Calculate offset from goal.
                    actual = new_b;
                }
            }

            if Vec2::distance(actual, goal) <= self.tolerance {
                break Some(actual);
            }

            iteration += 1;
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Joint {
    pub translation: Vec2,
    pub length: f32,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Segment {
    pub is_edge: bool,
    pub length: f32,
    pub translation: Vec2,
}

#[derive(Clone, Debug)]
pub struct Segments(Vec<Segment>);

impl Segments {
    fn length(&self) -> f32 {
        self.0.iter().fold(0.0, |acc, seg| acc + seg.length)
    }

    fn backwards(&mut self) -> Backwards<'_, Segment> {
        Backwards::new(self)
    }

    fn forwards(&mut self) -> Forwards<'_, Segment> {
        Forwards::new(self)
    }
}

impl Deref for Segments {
    type Target = Vec<Segment>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for Segments {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

///
///
/// Points `p0, p1, p2, p3` returns `(p3, p2), (p2, p1) (p1, p0)`.
struct Backwards<'a, T> {
    inner: &'a mut [T],
    next: usize,
}

impl<'a, T> Backwards<'a, T> {
    fn new(inner: &'a mut [T]) -> Self {
        Self {
            next: inner.len().saturating_sub(1),
            inner,
        }
    }
}

impl<'a, T> Iterator for Backwards<'a, T> {
    type Item = (&'a mut T, &'a mut T);

    fn next(&mut self) -> Option<Self::Item> {
        if self.next == 0 {
            return None;
        }

        self.inner.get(self.next)?;
        self.inner.get(self.next - 1)?;

        let ptr = self.inner.as_mut_ptr();
        let a = unsafe { &mut *ptr.add(self.next) };
        let b = unsafe { &mut *ptr.add(self.next - 1) };

        self.next -= 1;

        Some((a, b))
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.len(), Some(self.len()))
    }
}

impl<'a, T> ExactSizeIterator for Backwards<'a, T> {
    fn len(&self) -> usize {
        self.inner.len().saturating_sub(1)
    }
}

struct Forwards<'a, T> {
    inner: &'a mut [T],
    next: usize,
}

impl<'a, T> Forwards<'a, T> {
    fn new(inner: &'a mut [T]) -> Self {
        Self { inner, next: 0 }
    }
}

impl<'a, T> Iterator for Forwards<'a, T> {
    type Item = (&'a mut T, &'a mut T);

    fn next(&mut self) -> Option<Self::Item> {
        // Assets
        self.inner.get(self.next)?;
        self.inner.get(self.next + 1)?;

        let ptr = self.inner.as_mut_ptr();
        let a = unsafe { &mut *ptr.add(self.next) };
        let b = unsafe { &mut *ptr.add(self.next + 1) };

        self.next += 1;

        Some((a, b))
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.len(), Some(self.len()))
    }
}

impl<'a, T> ExactSizeIterator for Forwards<'a, T> {
    fn len(&self) -> usize {
        self.inner.len().saturating_sub(1)
    }
}

#[cfg(test)]
mod tests {
    use glam::Vec2;

    use super::{Backwards, Forwards, Joint, Solver};

    macro_rules! assert_tolerance {
        ($lhs:expr, $rhs:expr) => {
            if !($lhs >= $rhs - 0.01 && $lhs <= 10.0 + 0.01) {
                assert_eq!($lhs, $rhs);
            }
        };
    }

    #[test]
    fn test_unreachable() {
        let mut body = Solver::new([
            Joint {
                translation: Vec2::new(0.0, 0.0),
                length: 5.0,
            },
            Joint {
                translation: Vec2::new(0.0, 5.0),
                length: 4.0,
            },
        ]);

        assert_eq!(body.solve(Vec2::new(0.0, 10.0)), None);
    }

    #[test]
    fn test_solve() {
        let mut body = Solver::new([
            Joint {
                translation: Vec2::new(0.0, 0.0),
                length: 5.0,
            },
            Joint {
                translation: Vec2::new(0.0, 5.0),
                length: 5.0,
            },
            Joint {
                translation: Vec2::new(0.0, 10.0),
                length: 5.0,
            },
        ]);

        let res = body.solve(Vec2::new(10.0, 0.0)).unwrap();
        assert_tolerance!(res.x, 10.0);
        assert_tolerance!(res.y, 0.0);
    }

    #[test]
    fn test_backwards() {
        let mut slice = [0, 1, 2, 3, 4];
        let mut iter = Backwards::new(&mut slice);

        assert_eq!(iter.next(), Some((&mut 4, &mut 3)));
        assert_eq!(iter.next(), Some((&mut 3, &mut 2)));
        assert_eq!(iter.next(), Some((&mut 2, &mut 1)));
        assert_eq!(iter.next(), Some((&mut 1, &mut 0)));
        assert_eq!(iter.next(), None);
    }

    #[test]
    fn test_forwards() {
        let mut slice = [0, 1, 2, 3, 4];
        let mut iter = Forwards::new(&mut slice);

        assert_eq!(iter.next(), Some((&mut 0, &mut 1)));
        assert_eq!(iter.next(), Some((&mut 1, &mut 2)));
        assert_eq!(iter.next(), Some((&mut 2, &mut 3)));
        assert_eq!(iter.next(), Some((&mut 3, &mut 4)));
        assert_eq!(iter.next(), None);
    }
}
