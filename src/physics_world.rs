use crate::rigid_body::RigidBody;
use rlalg::Vector;
use rlalg::dot;
use rlalg::{v, v2f};

#[derive(serde::Serialize, serde::Deserialize)]
pub struct PhysicsWorld {
    pub bodies: Vec<RigidBody>,
    pub boddie_corrections: Vec<v2f>,
    pub boddie_friction: Vec<v2f>,
    pub boddie_rot_corrections: Vec<f32>,
}

impl PhysicsWorld {
    pub fn new() -> PhysicsWorld {
        PhysicsWorld {
            bodies: Vec::new(),
            boddie_corrections: Vec::new(),
            boddie_friction: Vec::new(),
            boddie_rot_corrections: Vec::new(),
        }
    }

    pub fn add_body(&mut self, body: RigidBody) {
        self.bodies.push(body);
    }

    pub fn step(&mut self, dt: f32) {
        for body in &mut self.bodies {
            // body.add_gravity();
            body.update(dt);
        }

        for _ in 0..50 {
            self.solve_collisions();
            self.apply_corrections();
        }
    }

    pub fn solve_collisions(&mut self) {
        self.boddie_corrections = vec![v!(0.0, 0.0); self.bodies.len()];
        self.boddie_rot_corrections = vec![0.0; self.bodies.len()];

        let all_points: Vec<Vec<v2f>> = self.bodies.iter().map(|b| b.get_world_points()).collect();
        let all_edges: Vec<Vec<(v2f, v2f)>> =
            self.bodies.iter().map(|b| b.get_world_edges()).collect();

        for i in 0..self.bodies.len() {
            for j in 0..self.bodies.len() {
                if j == i {
                    continue;
                }

                for &target_p in &all_points[j] {
                    let mut best_dist = f32::MIN;
                    let mut best_normal = v!(0.0, 0.0);
                    let mut is_inside = true;

                    for (p1, p2) in &all_edges[i] {
                        let e_vec = *p2 - *p1;
                        let normal = v!(e_vec.y, -e_vec.x).norm();
                        let dist = dot(target_p - *p1, normal);

                        if dist > 0.0 {
                            is_inside = false;
                            break;
                        }

                        if dist > best_dist {
                            best_dist = dist;
                            best_normal = normal;
                        }
                    }

                    if is_inside {
                        let r_i = target_p - self.bodies[i].pos;
                        let r_j = target_p - self.bodies[j].pos;

                        let cross_2d = |a: v2f, b: v2f| a.x * b.y - a.y * b.x;

                        let rn_i = cross_2d(r_i, best_normal);
                        let rn_j = cross_2d(r_j, best_normal);

                        let inv_m_i = self.bodies[i].get_inv_mass();
                        let inv_m_j = self.bodies[j].get_inv_mass();
                        let inv_i_i = self.bodies[i].get_inv_inertia();
                        let inv_i_j = self.bodies[j].get_inv_inertia();

                        let k =
                            inv_m_i + inv_m_j + (rn_i * rn_i * inv_i_i) + (rn_j * rn_j * inv_i_j);

                        if k > 0.0 {
                            let penetration = -best_dist;
                            let p_mag = penetration / k;

                            self.boddie_corrections[i] -= best_normal * (p_mag * inv_m_i);
                            self.boddie_corrections[j] += best_normal * (p_mag * inv_m_j);

                            self.boddie_rot_corrections[i] -= p_mag * rn_i * inv_i_i;
                            self.boddie_rot_corrections[j] += p_mag * rn_j * inv_i_j;
                        }
                    }
                }
            }
        }
    }

    pub fn apply_corrections(&mut self) {
        for (i, body) in self.bodies.iter_mut().enumerate() {
            body.pos += self.boddie_corrections[i];
            body.rot += self.boddie_rot_corrections[i];
        }
    }
}
