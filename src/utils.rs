use rlalg::{v, v2f};

// Helpers

pub fn rotate(v: v2f, angle: f32) -> v2f {
    let (sin, cos) = angle.sin_cos();
    v!(v.x * cos - v.y * sin, v.x * sin + v.y * cos)
}
