#![feature(test)]

extern crate test;

use fabrik::{Joint, Solver};
use glam::Vec2;
use test::Bencher;

#[bench]
fn bench_solver2d(b: &mut Bencher) {
    let body = Solver::new([
        Joint {
            translation: Vec2::new(0.0, 0.0),
            length: 1.0,
        },
        Joint {
            translation: Vec2::new(1.0, 0.0),
            length: 1.0,
        },
        Joint {
            translation: Vec2::new(2.0, 0.0),
            length: 1.0,
        },
        Joint {
            translation: Vec2::new(3.0, 0.0),
            length: 1.0,
        },
        Joint {
            translation: Vec2::new(4.0, 0.0),
            length: 1.0,
        },
        Joint {
            translation: Vec2::new(5.0, 0.0),
            length: 1.0,
        },
        Joint {
            translation: Vec2::new(7.0, 0.0),
            length: 1.0,
        },
        Joint {
            translation: Vec2::new(8.0, 0.0),
            length: 1.0,
        },
        Joint {
            translation: Vec2::new(9.0, 0.0),
            length: 1.0,
        },
    ]);

    b.iter(|| {
        let mut body = body.clone();

        body.solve(Vec2::new(5.0, 5.0));
    });
}
