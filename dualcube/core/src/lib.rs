#[path = "201-dual/mod.rs"]
pub mod dual;
#[path = "100-flowfield/mod.rs"]
pub mod flowfield;
#[path = "101-flowgraph/mod.rs"]
pub mod flowgraph;
#[path = "600-hex/mod.rs"]
pub mod hex;
#[path = "300-layout/mod.rs"]
pub mod layout;
#[path = "200-loops/mod.rs"]
pub mod loops;
#[path = "301-polycube/mod.rs"]
pub mod polycube;
#[path = "500-quad/mod.rs"]
pub mod quad;

pub mod solution;

pub mod prelude {
    use mehsh::prelude::*;
    use serde::{Deserialize, Serialize};
    use std::fmt::Display;

    pub use crate::dual::*;
    pub use crate::flowfield::*;
    pub use crate::flowgraph::*;
    // pub use crate::hex::*;
    pub use crate::layout::*;
    pub use crate::loops::*;
    pub use crate::polycube::*;
    pub use crate::quad::*;
    pub use crate::solution::*;

    #[derive(Default, Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
    pub struct INPUT;
    pub type VertID = VertKey<INPUT>;
    pub type EdgeID = EdgeKey<INPUT>;
    pub type FaceID = FaceKey<INPUT>;

    // The three coordinate directions X, Y and Z
    #[derive(Copy, Clone, Default, PartialEq, Eq, Debug, Hash, Serialize, Deserialize)]
    pub enum Direction {
        #[default]
        X,
        Y,
        Z,
    }
    impl Display for Direction {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            match self {
                Self::X => write!(f, "x-direction"),
                Self::Y => write!(f, "y-direction"),
                Self::Z => write!(f, "z-direction"),
            }
        }
    }

    pub const DIRECTIONS: [Direction; 3] = [Direction::X, Direction::Y, Direction::Z];

    #[derive(Default, Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
    pub enum Sign {
        #[default]
        Positive,
        Negative,
    }

    #[derive(Copy, Clone, Default, Debug, Serialize, Deserialize)]
    pub enum Perspective {
        Primal,
        #[default]
        Dual,
    }

    impl From<Direction> for Vector3D {
        fn from(dir: Direction) -> Self {
            match dir {
                Direction::X => Self::new(1., 0., 0.),
                Direction::Y => Self::new(0., 1., 0.),
                Direction::Z => Self::new(0., 0., 1.),
            }
        }
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
    pub enum Side {
        Upper,
        Lower,
    }

    pub fn to_principal_direction(v: Vector3D) -> (Direction, Sign) {
        let x_is_max = v.x.abs() >= v.y.abs() && v.x.abs() >= v.z.abs();
        let y_is_max = v.y.abs() > v.x.abs() && v.y.abs() >= v.z.abs();
        let z_is_max = v.z.abs() > v.x.abs() && v.z.abs() > v.y.abs();
        assert!(x_is_max ^ y_is_max ^ z_is_max, "{v:?}");

        if x_is_max {
            if v.x > 0. {
                (Direction::X, Sign::Positive)
            } else {
                (Direction::X, Sign::Negative)
            }
        } else if y_is_max {
            if v.y > 0. {
                (Direction::Y, Sign::Positive)
            } else {
                (Direction::Y, Sign::Negative)
            }
        } else if z_is_max {
            if v.z > 0. {
                (Direction::Z, Sign::Positive)
            } else {
                (Direction::Z, Sign::Negative)
            }
        } else {
            unreachable!()
        }
    }

    pub fn to_vector(dir: Direction, sign: Sign) -> Vector3D {
        let v = Vector3D::from(dir);
        match sign {
            Sign::Positive => v,
            Sign::Negative => -v,
        }
    }
}
