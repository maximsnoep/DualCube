use douconel::douconel::{Douconel, FaceID, VertID};
use douconel::douconel_embedded::HasPosition;
use hutspot::geom::Vector3D;
use serde::{Deserialize, Serialize};
use std::collections::HashSet;
use std::sync::Arc;

use crate::EmbeddedMesh;

pub type Polycube = Douconel<PolycubeVertex, Vec<VertID>, (VertID, Patch)>;

// Embedded vertices (have a position)
#[derive(Default, Copy, Clone, Debug, Serialize, Deserialize)]
pub struct PolycubeVertex {
    // real space
    pub position: Vector3D,
    // pointer to the face of the loop structure
    pub pointer_loop_region: FaceID,
    // pointer to the corresponding vertex on the mesh
    pub pointer_primal_vertex: VertID,
}

impl HasPosition for PolycubeVertex {
    fn position(&self) -> Vector3D {
        self.position
    }
    fn set_position(&mut self, position: Vector3D) {
        self.position = position;
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Patch {
    // A patch is defined by a set of faces
    pub faces: HashSet<FaceID>,
}
