use crate::dual::LoopID;
use bevy::render::color::Color;
use douconel::douconel::EdgeID;
use douconel::douconel::FaceID;
use douconel::douconel::VertID;
use hutspot::geom::Vector3D;
use serde::Deserialize;
use serde::Serialize;
use std::collections::HashSet;
use std::fmt::Display;

#[derive(Copy, Clone, PartialEq, Eq, Debug, Hash, Serialize, Deserialize)]
pub enum PrincipalDirection {
    X,
    Y,
    Z,
}

impl Display for PrincipalDirection {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PrincipalDirection::X => write!(f, "X-axis"),
            PrincipalDirection::Y => write!(f, "Y-axis"),
            PrincipalDirection::Z => write!(f, "Z-axis"),
        }
    }
}

impl From<PrincipalDirection> for Vector3D {
    fn from(dir: PrincipalDirection) -> Self {
        match dir {
            PrincipalDirection::X => Vector3D::new(1., 0., 0.),
            PrincipalDirection::Y => Vector3D::new(0., 1., 0.),
            PrincipalDirection::Z => Vector3D::new(0., 0., 1.),
        }
    }
}

impl PrincipalDirection {
    pub fn to_primal_color(&self) -> Color {
        match self {
            PrincipalDirection::X => hutspot::color::ROODT.into(),
            PrincipalDirection::Y => hutspot::color::BLAUW.into(),
            PrincipalDirection::Z => hutspot::color::YELLO.into(),
        }
    }

    pub fn to_dual_color(&self) -> Color {
        match self {
            PrincipalDirection::X => hutspot::color::GREEN.into(),
            PrincipalDirection::Y => hutspot::color::ORANG.into(),
            PrincipalDirection::Z => hutspot::color::PURPL.into(),
        }
    }

    pub fn to_dual_color_sided(&self, s: Side) -> Color {
        match (self, s) {
            (PrincipalDirection::X, Side::Upper) => hutspot::color::GREEN.into(),
            (PrincipalDirection::X, Side::Lower) => hutspot::color::GREEN_L.into(),
            (PrincipalDirection::Y, Side::Upper) => hutspot::color::ORANG.into(),
            (PrincipalDirection::Y, Side::Lower) => hutspot::color::ORANG_L.into(),
            (PrincipalDirection::Z, Side::Upper) => hutspot::color::PURPL.into(),
            (PrincipalDirection::Z, Side::Lower) => hutspot::color::PURPL_L.into(),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Loop {
    // A loop is defined by a sequence of half-edges.
    pub edges: Vec<EdgeID>,
    // the direction or labeling associated with the loop
    pub direction: PrincipalDirection,
}

impl Loop {
    pub fn contains_pair(&self, needle: (EdgeID, EdgeID)) -> bool {
        hutspot::math::wrap_pairs(&self.edges)
            .into_iter()
            .any(|(a, b)| a == needle.0 && b == needle.1)
    }

    fn find_edge(&self, needle: EdgeID) -> usize {
        self.edges.iter().position(|&e| e == needle).unwrap()
    }

    pub fn between(&self, start: EdgeID, end: EdgeID) -> Vec<EdgeID> {
        let start_pos = self.find_edge(start);
        let end_pos = self.find_edge(end);

        let mut seq = vec![];
        if start_pos < end_pos {
            // if start_pos < end_pos, we return [start...end]
            seq.extend(self.edges[start_pos..=end_pos].iter());
        } else {
            // if start_pos > end_pos, we return [start...MAX] + [0...end]
            seq.extend(self.edges[start_pos..].iter());
            seq.extend(self.edges[..=end_pos].iter());
        }
        seq
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq, Hash)]
pub enum Side {
    Upper,
    Lower,
}

impl Side {
    pub fn flip(&self) -> Self {
        match self {
            Side::Upper => Side::Lower,
            Side::Lower => Side::Upper,
        }
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq, Eq, Hash)]
pub struct LoopSegment {
    // A loop segment is defined by a reference to a loop (id)
    pub loop_id: LoopID,
    // The start and end of the segment (intersections)
    pub start: EdgeID,
    pub end: EdgeID,
    // And a sequence of half-edges that define the segment of the loop (inbetween start and end)
    pub between: Vec<EdgeID>,
    // Side
    pub side: Option<Side>,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Subsurface {
    // A subsurface is defined by a set of vertices
    pub verts: Vec<VertID>,
    // Random color for visualization
    pub color: Color,
}

#[derive(Clone, Debug)]
pub struct Zone {
    // A zone is defined by a direction
    pub direction: PrincipalDirection,
    // All subsurfaces that are part of the zone (identified through face ids)
    pub subsurfaces: HashSet<FaceID>,
    // Coordinate
    pub coordinate: Option<f64>,
}
