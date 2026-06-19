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
    fn export(solution: &Solution, path: &Path) -> anyhow::Result<()>;
}

pub trait Import {
    fn import(path: &Path) -> anyhow::Result<Solution>;
}

pub use crate::formats::{
    apg::APG, dc::Dc, flag::Flag, hex::HEX, loops::Loops, nlr::NLR, obj::OBJ,
};
use dualcube::prelude::*;
use std::{path::Path, sync::Arc};

pub fn import_solution(path: &Path) -> Solution {
    match path.extension().unwrap().to_str() {
        Some("obj") => {
            let mesh = match Mesh::from_obj(path) {
                Ok(res) => Arc::new(res.0),
                Err(err) => {
                    panic!("Error while parsing OBJ file {path:?}: {err:?}");
                }
            };
            Solution::new(mesh.clone())
        }
        Some("stl") => {
            let mesh = match Mesh::from_stl(path) {
                Ok(res) => Arc::new(res.0),
                Err(err) => {
                    panic!("Error while parsing STL file {path:?}: {err:?}");
                }
            };
            Solution::new(mesh.clone())
        }
        Some("dc") => {
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

pub fn export_solution(sol: &Solution, path: &Path) -> anyhow::Result<()> {
    match path.extension().and_then(|e| e.to_str()) {
        Some("obj") => OBJ::export(sol, path),
        Some("dc") => Dc::export(sol, path),
        Some("loops") => Loops::export(sol, path),
        Some("nlr") => NLR::export(sol, path),
        Some("apg") => APG::export(sol, path),
        _ => {
            panic!("Unsupported file extension for {path:?}");
        }
    }
}
