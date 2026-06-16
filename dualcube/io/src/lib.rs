pub mod formats {
    pub mod apg;
    pub mod dc;
    pub mod flag;
    pub mod hex;
    pub mod loops;
    pub mod nlr;
    pub mod obj;
}

pub trait Export {
    fn export(solution: &dualcube::prelude::Solution, path: &std::path::Path)
        -> anyhow::Result<()>;
}

pub trait Import {
    fn import(path: &std::path::Path) -> anyhow::Result<dualcube::prelude::Solution>;
}

pub use crate::formats::{
    apg::APG, dc::Dc, flag::Flag, hex::HEX, loops::Loops, nlr::NLR, obj::OBJ,
};
use dualcube::prelude::Solution;
use std::{path::PathBuf, sync::Arc};

pub fn import_solution(path: PathBuf) -> Solution {
    match path.extension().unwrap().to_str() {
        Some("obj") => {
            let mesh = match mehsh::mesh::connectivity::Mesh::from_obj(&path) {
                Ok(res) => Arc::new(res.0),
                Err(err) => {
                    panic!("Error while parsing OBJ file {path:?}: {err:?}");
                }
            };
            Solution::new(mesh.clone())
        }
        Some("stl") => {
            let mesh = match mehsh::mesh::connectivity::Mesh::from_stl(&path) {
                Ok(res) => Arc::new(res.0),
                Err(err) => {
                    panic!("Error while parsing STL file {path:?}: {err:?}");
                }
            };
            Solution::new(mesh.clone())
        }
        Some("dc") | Some("dsol") => {
            if let Ok(sol) = Dc::import(&path) {
                sol
            } else {
                panic!("Error while parsing Dc file {path:?}");
            }
        }
        Some("loops") => {
            if let Ok(sol) = Loops::import(&path) {
                sol
            } else {
                panic!("Error while parsing loops file {path:?}");
            }
        }
        _ => {
            panic!("Unsupported file extension for {path:?}");
        }
    }
}
