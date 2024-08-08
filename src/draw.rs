use crate::elements::to_principal_direction;
use crate::elements::PrincipalDirection;
use crate::elements::Side;
use crate::system::Configuration;
use crate::MeshProperties;
use crate::SolutionResource;
use bevy::prelude::*;
use douconel::douconel::Douconel;
use douconel::douconel::EdgeID;
use douconel::douconel::FaceID;
use douconel::douconel::VertID;
use douconel::douconel_embedded::HasPosition;
use hutspot::draw::DrawableLine;
use hutspot::geom::Vector3D;

#[derive(Default, Resource)]
pub struct GizmosCache {
    pub wireframe: Vec<Line>,
    pub vertices: Vec<Line>,
    pub normals: Vec<Line>,
    pub loops: Vec<Line>,
    pub paths: Vec<Line>,

    pub raycaster: Vec<Line>,
}
type Line = (Vec3, Vec3, Color);

// Draws the gizmos. This includes the wireframe, vertices, normals, and raycasts, etc.
pub fn draw_gizmos(mut gizmos: Gizmos, gizmos_cache: Res<GizmosCache>, solution: Res<SolutionResource>, configuration: Res<Configuration>) {
    if configuration.draw_wireframe {
        for &(u, v, c) in &gizmos_cache.wireframe {
            gizmos.line(u, v, c);
        }
    }
    if configuration.draw_vertices {
        for &(u, v, c) in &gizmos_cache.vertices {
            gizmos.line(u, v, c);
        }
    }
    if configuration.draw_normals {
        for &(u, v, c) in &gizmos_cache.normals {
            gizmos.line(u, v, c);
        }
    }
    if configuration.interactive {
        for &(u, v, c) in &gizmos_cache.raycaster {
            gizmos.line(u, v, c);
        }
    }

    // Draw loops if `black` is set to `true`.
    if configuration.black {
        for &(u, v, c) in &gizmos_cache.loops {
            gizmos.line(u, v, c);
        }
    // Draw paths if `black` is set to `false`.
    } else {
        for &(u, v, c) in &gizmos_cache.paths {
            gizmos.line(u, v, c);
        }
    }

    // Polycube wireframe, does not need to be cached, since it is so simple.
    if let Ok(primal) = &solution.primal {
        if configuration.black {
            // Draw all loop segments / faces axis aligned.
            for (face_id, (original_id, _)) in &primal.faces {
                let this_centroid = primal.centroid(face_id);

                let normal = (primal.normal(face_id) as Vector3D).normalize();
                let orientation = to_principal_direction(normal).0;

                for &neighbor_id in &primal.fneighbors(face_id) {
                    let next_original_id = &primal.faces[neighbor_id].0;

                    let edge_between = primal.edge_between_faces(face_id, neighbor_id).unwrap().0;
                    let root = primal.root(edge_between);
                    let root_pos = primal.position(root);

                    let segment = solution
                        .dual
                        .get_loop_structure()
                        .edge_between_verts(*original_id, *next_original_id)
                        .unwrap()
                        .0;

                    let direction = solution.dual.segment_to_direction(segment);

                    for side in [Side::Upper, Side::Lower] {
                        let segment_direction = match (orientation, direction) {
                            (PrincipalDirection::X, PrincipalDirection::Y) | (PrincipalDirection::Y, PrincipalDirection::X) => PrincipalDirection::Z,
                            (PrincipalDirection::X, PrincipalDirection::Z) | (PrincipalDirection::Z, PrincipalDirection::X) => PrincipalDirection::Y,
                            (PrincipalDirection::Y, PrincipalDirection::Z) | (PrincipalDirection::Z, PrincipalDirection::Y) => PrincipalDirection::X,
                            _ => unreachable!(),
                        };

                        let mut direction_vector = this_centroid;
                        direction_vector[segment_direction as usize] = root_pos[segment_direction as usize];

                        let mut offset = Vector3D::new(0., 0., 0.);

                        let dist = 0.001 * f64::from(solution.properties.scale);

                        match side {
                            Side::Upper => match direction {
                                PrincipalDirection::X => offset[0] += dist,
                                PrincipalDirection::Y => offset[1] += dist,
                                PrincipalDirection::Z => offset[2] += dist,
                            },
                            Side::Lower => match direction {
                                PrincipalDirection::X => offset[0] -= dist,
                                PrincipalDirection::Y => offset[1] -= dist,
                                PrincipalDirection::Z => offset[2] -= dist,
                            },
                        };

                        let line = DrawableLine::from_line(
                            this_centroid,
                            direction_vector,
                            offset + normal * 0.01,
                            solution.properties.translation,
                            solution.properties.scale,
                        );

                        gizmos.line(line.u, line.v, direction.to_dual_color_sided(side));
                    }
                }
            }
        }

        // Draw the edges of the polycube.
        for edge_id in primal.edge_ids() {
            let endpoints = primal.endpoints(edge_id);
            let u = primal.position(endpoints.0);
            let v = primal.position(endpoints.1);
            let line = DrawableLine::from_line(
                u,
                v,
                primal.normal(primal.face(edge_id)) * 0.001,
                solution.properties.translation,
                solution.properties.scale,
            );
            gizmos.line(line.u, line.v, hutspot::color::BLACK.into());
        }
    }
}

pub fn add_line(lines: &mut Vec<Line>, position: Vector3D, normal: Vector3D, length: f32, color: Color, props: &MeshProperties) {
    let line = DrawableLine::from_vertex(position, normal, length, props.translation, props.scale);
    lines.push((line.u, line.v, color));
}

pub fn add_line2(lines: &mut Vec<Line>, position_a: Vector3D, position_b: Vector3D, offset: Vector3D, color: Color, props: &MeshProperties) {
    let line = DrawableLine::from_line(position_a, position_b, offset, props.translation, props.scale);
    lines.push((line.u, line.v, color));
}
