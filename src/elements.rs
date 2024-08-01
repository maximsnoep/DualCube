use crate::dual::LoopID;
use bevy::render::color::Color;
use douconel::douconel::{EdgeID, FaceID, VertID};
use hutspot::geom::Vector3D;
use serde::{Deserialize, Serialize};
use std::collections::HashSet;
use std::fmt::Display;

#[derive(Copy, Clone, Default, PartialEq, Eq, Debug, Hash, Serialize, Deserialize)]
pub enum PrincipalDirection {
    #[default]
    X,
    Y,
    Z,
}

impl Display for PrincipalDirection {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::X => write!(f, "X-axis"),
            Self::Y => write!(f, "Y-axis"),
            Self::Z => write!(f, "Z-axis"),
        }
    }
}

impl PrincipalDirection {
    pub fn to_primal_color(self) -> Color {
        match self {
            Self::X => hutspot::color::ROODT.into(),
            Self::Y => hutspot::color::BLAUW.into(),
            Self::Z => hutspot::color::YELLO.into(),
        }
    }

    pub fn to_primal_color_sided(self, s: Side) -> Color {
        match (self, s) {
            (Self::X, Side::Upper) => hutspot::color::ROODT.into(),
            (Self::X, Side::Lower) => hutspot::color::ROODT_L.into(),
            (Self::Y, Side::Upper) => hutspot::color::BLAUW.into(),
            (Self::Y, Side::Lower) => hutspot::color::BLAUW_L.into(),
            (Self::Z, Side::Upper) => hutspot::color::YELLO.into(),
            (Self::Z, Side::Lower) => hutspot::color::YELLO_L.into(),
        }
    }

    pub fn to_dual_color(self) -> Color {
        match self {
            Self::X => hutspot::color::GREEN.into(),
            Self::Y => hutspot::color::ORANG.into(),
            Self::Z => hutspot::color::PURPL.into(),
        }
    }

    pub fn to_dual_color_sided(self, s: Side) -> Color {
        match (self, s) {
            (Self::X, Side::Upper) => hutspot::color::GREEN.into(),
            (Self::X, Side::Lower) => hutspot::color::GREEN_L.into(),
            (Self::Y, Side::Upper) => hutspot::color::ORANG.into(),
            (Self::Y, Side::Lower) => hutspot::color::ORANG_L.into(),
            (Self::Z, Side::Upper) => hutspot::color::PURPL.into(),
            (Self::Z, Side::Lower) => hutspot::color::PURPL_L.into(),
        }
    }
}

impl From<PrincipalDirection> for Vector3D {
    fn from(dir: PrincipalDirection) -> Self {
        match dir {
            PrincipalDirection::X => Self::new(1., 0., 0.),
            PrincipalDirection::Y => Self::new(0., 1., 0.),
            PrincipalDirection::Z => Self::new(0., 0., 1.),
        }
    }
}

pub fn to_principal_direction(v: Vector3D) -> (PrincipalDirection, Side) {
    let x_is_max = v.x.abs() > v.y.abs() && v.x.abs() > v.z.abs();
    let y_is_max = v.y.abs() > v.x.abs() && v.y.abs() > v.z.abs();
    let z_is_max = v.z.abs() > v.x.abs() && v.z.abs() > v.y.abs();
    assert!(x_is_max ^ y_is_max ^ z_is_max);

    if x_is_max {
        if v.x > 0. {
            (PrincipalDirection::X, Side::Upper)
        } else {
            (PrincipalDirection::X, Side::Lower)
        }
    } else if y_is_max {
        if v.y > 0. {
            (PrincipalDirection::Y, Side::Upper)
        } else {
            (PrincipalDirection::Y, Side::Lower)
        }
    } else if z_is_max {
        if v.z > 0. {
            (PrincipalDirection::Z, Side::Upper)
        } else {
            (PrincipalDirection::Z, Side::Lower)
        }
    } else {
        unreachable!()
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
        hutspot::math::wrap_pairs(&self.edges).into_iter().any(|(a, b)| a == needle.0 && b == needle.1)
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
    pub const fn flip(&self) -> Self {
        match self {
            Self::Upper => Self::Lower,
            Self::Lower => Self::Upper,
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
