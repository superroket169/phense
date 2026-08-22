use crate::GRAVITY;
use crate::utils::rotate;
use rlalg::{v, v2f};
use serde::{Deserialize, Serialize};

#[derive(serde::Serialize, serde::Deserialize, Clone)]
pub struct Edge {
    pub begin: v2f,
    pub end: v2f,
}

#[derive(serde::Serialize, serde::Deserialize, Clone)]
pub struct RigidBody {
    pub pos: v2f,
    pub pre_pos: v2f,
    pub rot: f32,
    pub pre_rot: f32,
    pub accel: v2f,
    pub rot_accel: f32,
    inv_mass: f32,
    inv_inertia: f32,
    pub edges: Vec<Edge>,
    pub is_static: bool,
}

impl RigidBody {
    fn calc_inv_inert(mass: f32, edges: &[Edge]) -> f32 {
        if mass <= 0.0 || edges.is_empty() {
            return 0.0;
        }

        let mut min_x = 0.0;
        let mut max_x = 0.0;
        let mut min_y = 0.0;
        let mut max_y = 0.0;

        for e in edges {
            for p in [e.begin, e.end] {
                if p.x < min_x {
                    min_x = p.x;
                }
                if p.x > max_x {
                    max_x = p.x;
                }
                if p.y < min_y {
                    min_y = p.y;
                }
                if p.y > max_y {
                    max_y = p.y;
                }
            }
        }

        let w = max_x - min_x;
        let h = max_y - min_y;

        let inertia = (mass * (w * w + h * h)) / 12.0;
        1.0 / inertia * 100.0
    }

    pub fn new(m: f32, p: (f32, f32), r: f32, e: Vec<Edge>) -> Self {
        let im = if m == 0.0 { 0.0 } else { 1.0 / m };
        let ii = Self::calc_inv_inert(m, &e);

        Self {
            pos: v!(p.0, p.1),
            pre_pos: v!(p.0, p.1),
            rot: r,
            pre_rot: r,
            accel: v!(0.0, 0.0),
            rot_accel: 0.0,
            inv_mass: im,
            inv_inertia: ii,
            edges: e,
            is_static: if ii == 0.0 { true } else { false }, // so warningly
        }
    }

    pub fn add_force(&mut self, f: v2f) {
        self.accel += f * self.inv_mass;
    }

    pub fn set_static(&mut self, is: bool) {
        self.is_static = is;
    }

    pub fn add_gravity(&mut self) {
        if self.is_static == false {
            self.accel += v!(0.0, GRAVITY);
        }
    }

    pub fn add_torque(&mut self, t: f32) {
        self.rot_accel += t * self.inv_inertia;
    }

    pub fn apply_impulse(&mut self, i: v2f) {
        if self.inv_mass > 0.0 {
            self.pos += i * self.inv_mass;
        }
    }

    pub fn update(&mut self, dt: f32) {
        let dt_sq = dt * dt;

        let velocity = self.pos - self.pre_pos;
        let rot_velocity = self.rot - self.pre_rot;

        self.pre_pos = self.pos;
        self.pre_rot = self.rot;

        self.pos = self.pos + velocity + self.accel * dt_sq;
        self.rot = self.rot + rot_velocity + self.rot_accel * dt_sq;

        self.accel = v!(0.0, 0.0);
        self.rot_accel = 0.0;
    }

    // --- Getter / Setters ---

    pub fn get_inv_mass(&self) -> f32 {
        self.inv_mass
    }

    pub fn get_inv_inertia(&self) -> f32 {
        self.inv_inertia
    }

    pub fn get_world_edges(&self) -> Vec<(v2f, v2f)> {
        self.edges
            .iter()
            .map(|e| {
                (
                    rotate(e.begin, self.rot) + self.pos,
                    rotate(e.end, self.rot) + self.pos,
                )
            })
            .collect()
    }

    pub fn get_world_points(&self) -> Vec<v2f> {
        self.edges
            .iter()
            .map(|e| rotate(e.begin, self.rot) + self.pos)
            .collect()
    }

    pub fn set_mass(&mut self, m: f32) {
        if m <= 0.0 {
            self.inv_mass = 0.0;
            self.inv_inertia = 0.0;
        } else {
            self.inv_mass = 1.0 / m;
            self.inv_inertia = Self::calc_inv_inert(m, &self.edges);
        }
    }

    pub fn set_edges(&mut self, e: Vec<Edge>) {
        self.edges = e;
        if self.inv_mass > 0.0 {
            let m = 1.0 / self.inv_mass;
            self.inv_inertia = Self::calc_inv_inert(m, &self.edges);
        }
    }
}
