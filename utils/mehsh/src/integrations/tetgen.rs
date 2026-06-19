use crate::prelude::*;
use crate::utils::ids::IdMap;
use log::info;

use std::{
    collections::HashMap,
    fmt::Write as FmtWrite,
    fs::{self, File},
    io::{self, Write as IoWrite},
    path::{Path, PathBuf},
    process::Command,
};

#[derive(Debug, Clone)]
pub struct TetExport<M: Tag>
where
    M: Default + Eq + std::hash::Hash + Copy + Clone,
{
    pub boundary: Mesh<M>,
    pub tet_vertex_to_input_vertex: IdMap<VERT, M>,
    pub tet_vertex_to_boundary_vertex: IdMap<VERT, M>,
    pub path: PathBuf,
}

#[derive(Debug, Clone)]
pub struct TetgenCliParams {
    pub command: String,
    pub args: Vec<String>,
}

#[derive(Debug, Clone, Copy)]
struct TetNode {
    position: Vector3D,
    marker: i32,
}

#[derive(Debug, Clone)]
struct TetMeshData {
    nodes: Vec<TetNode>,
    tets: Vec<[usize; 4]>,
}

impl<M: Tag> Mesh<M>
where
    M: Default + Eq + std::hash::Hash + Copy + Clone,
{
    pub fn to_tet(
        &self,
        vtk_path1: &Path,
        vtk_path2: &Path,
        params: &TetgenCliParams,
    ) -> io::Result<TetExport<M>> {
        let smesh_path1 = vtk_path1.with_extension("smesh");
        let smesh_path2 = vtk_path2.with_extension("smesh");

        let node_path = tetgen_output_path(&smesh_path1, "node")?;
        let ele_path = tetgen_output_path(&smesh_path1, "ele")?;

        remove_if_exists(&node_path)?;
        remove_if_exists(&ele_path)?;

        info!(
            "running TetGen: command={} args={:?} smesh={}",
            params.command,
            params.args,
            smesh_path2.display()
        );

        let marker_to_vert = self.write_smesh(&smesh_path1)?;
        run_tetgen(params, &smesh_path2)?;
        let tet_mesh = read_tetgen_output(&node_path, &ele_path)?;

        info!("extracting boundary mesh from TetGen output");

        let tet_vertex_to_input_vertex = recover_original_vertex_map(&tet_mesh, &marker_to_vert);
        let (boundary, tet_vertex_to_boundary_vertex) = extract_boundary_mesh(&tet_mesh)?;

        info!("writing VTK output");

        write_vtk(vtk_path1, &tet_mesh)?;

        info!("done");

        Ok(TetExport {
            boundary,
            tet_vertex_to_input_vertex,
            tet_vertex_to_boundary_vertex,
            path: vtk_path1.to_owned(),
        })
    }

    fn write_smesh(&self, path: &Path) -> io::Result<HashMap<i32, VertKey<M>>> {
        let vert_ids = self.vert_ids();

        if vert_ids.len() > (i32::MAX as usize - 2) {
            return Err(io::Error::other(
                "too many vertices to assign unique i32 TetGen markers",
            ));
        }

        let mut input_vert_to_tet_id = HashMap::new();
        let mut marker_to_vert = HashMap::new();
        let mut text = String::new();

        writeln!(text, "{} 3 0 1", vert_ids.len()).unwrap();

        for (i, vert_id) in vert_ids.iter().copied().enumerate() {
            input_vert_to_tet_id.insert(vert_id, i);

            let marker = -((i as i32) + 2);
            marker_to_vert.insert(marker, vert_id);

            let p = self.position(vert_id);
            writeln!(text, "{} {:?} {:?} {:?} {}", i, p.x, p.y, p.z, marker).unwrap();
        }

        let face_ids = self.face_ids();

        // Part 2: facet list.
        // Format: <# facets> <# boundary markers>
        writeln!(text, "{} 1", face_ids.len()).unwrap();

        for face_id in face_ids {
            let f = self
                .vertices(face_id)
                .map(|v| input_vert_to_tet_id[&v])
                .collect::<Vec<_>>();

            if f.len() != 3 {
                return Err(io::Error::other(format!(
                    "TetGen .smesh export expects triangles, found {} vertices",
                    f.len()
                )));
            }

            writeln!(text, "3 {} {} {} 0", f[0], f[1], f[2]).unwrap();
        }

        // Part 3: holes.
        writeln!(text, "0").unwrap();

        // Part 4: regions.
        writeln!(text, "0").unwrap();

        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent)?;
        }

        let mut file = File::create(path)?;
        file.write_all(text.as_bytes())?;
        file.sync_all()?;

        Ok(marker_to_vert)
    }
}

pub fn boundary_loop_to_disk_scissors_loop<M: Tag>(
    export: &TetExport<M>,
    loop_vertices: &[VertKey<M>],
) -> io::Result<Vec<usize>>
where
    M: Default + Eq + std::hash::Hash + Copy + Clone,
{
    loop_vertices
        .iter()
        .map(|v| {
            export
                .tet_vertex_to_boundary_vertex
                .id(v)
                .copied()
                .ok_or_else(|| io::Error::other("boundary vertex has no VTK/TetGen vertex id"))
        })
        .collect()
}

fn run_tetgen(params: &TetgenCliParams, smesh_path: &Path) -> io::Result<()> {
    let mut command_parts = params.command.split_whitespace();
    let executable = command_parts
        .next()
        .ok_or_else(|| io::Error::other("TetGen command is empty"))?;

    let mut command = Command::new(executable);
    command.args(command_parts);
    command.args(&params.args);
    command.arg(smesh_path);

    info!(
        "Running TetGen command without path conversion: {:?}",
        command
    );

    let output = command.output()?;

    info!(
        "TetGen finished: status={} stdout_bytes={} stderr_bytes={}",
        output.status,
        output.stdout.len(),
        output.stderr.len()
    );
    info!("TetGen stdout: {}", String::from_utf8_lossy(&output.stdout));
    info!("TetGen stderr: {}", String::from_utf8_lossy(&output.stderr));

    if output.status.success() {
        return Ok(());
    }

    Err(io::Error::other(format!(
        "TetGen command failed\nstdout:\n{}\nstderr:\n{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr),
    )))
}

fn read_tetgen_output(node_path: &Path, ele_path: &Path) -> io::Result<TetMeshData> {
    let (nodes, raw_to_local) = read_node_file(node_path)?;
    let tets = read_ele_file(ele_path, &raw_to_local)?;
    Ok(TetMeshData { nodes, tets })
}

fn read_node_file(path: &Path) -> io::Result<(Vec<TetNode>, HashMap<usize, usize>)> {
    let lines = cleaned_lines(path)?;
    let header = parse_usizes(lines.first(), path, "node header")?;

    if header.len() < 4 || header[1] != 3 {
        return Err(io::Error::other(format!(
            "invalid .node header in '{}'",
            path.display()
        )));
    }

    let npoint = header[0];
    let nattr = header[2];
    let has_marker = header[3] != 0;

    if lines.len() < 1 + npoint {
        return Err(io::Error::other(format!(
            "invalid .node file '{}': expected {} points",
            path.display(),
            npoint
        )));
    }

    let mut nodes = Vec::with_capacity(npoint);
    let mut raw_to_local = HashMap::new();

    for local in 0..npoint {
        let tokens = lines[1 + local].split_whitespace().collect::<Vec<_>>();
        let min_len = 4 + nattr + usize::from(has_marker);

        if tokens.len() < min_len {
            return Err(io::Error::other(format!(
                "invalid .node line in '{}': '{}'",
                path.display(),
                lines[1 + local]
            )));
        }

        let raw_id = parse_token(tokens[0], path, "node id")?;
        let marker = if has_marker {
            parse_token(tokens[4 + nattr], path, "node marker")?
        } else {
            0
        };

        raw_to_local.insert(raw_id, local);
        nodes.push(TetNode {
            position: Vector3D::new(
                parse_token(tokens[1], path, "x")?,
                parse_token(tokens[2], path, "y")?,
                parse_token(tokens[3], path, "z")?,
            ),
            marker,
        });
    }

    Ok((nodes, raw_to_local))
}

fn read_ele_file(path: &Path, raw_to_local: &HashMap<usize, usize>) -> io::Result<Vec<[usize; 4]>> {
    let lines = cleaned_lines(path)?;
    let header = parse_usizes(lines.first(), path, "ele header")?;

    if header.len() < 3 || header[1] != 4 {
        return Err(io::Error::other(format!(
            "only tet4 .ele files are supported: '{}'",
            path.display()
        )));
    }

    let ncell = header[0];

    if lines.len() < 1 + ncell {
        return Err(io::Error::other(format!(
            "invalid .ele file '{}': expected {} cells",
            path.display(),
            ncell
        )));
    }

    (0..ncell)
        .map(|i| {
            let tokens = lines[1 + i].split_whitespace().collect::<Vec<_>>();

            if tokens.len() < 5 {
                return Err(io::Error::other(format!(
                    "invalid .ele line in '{}': '{}'",
                    path.display(),
                    lines[1 + i]
                )));
            }

            Ok([
                raw_to_local_id(parse_token(tokens[1], path, "tet v0")?, raw_to_local, path)?,
                raw_to_local_id(parse_token(tokens[2], path, "tet v1")?, raw_to_local, path)?,
                raw_to_local_id(parse_token(tokens[3], path, "tet v2")?, raw_to_local, path)?,
                raw_to_local_id(parse_token(tokens[4], path, "tet v3")?, raw_to_local, path)?,
            ])
        })
        .collect()
}

fn recover_original_vertex_map<M: Tag>(
    tet_mesh: &TetMeshData,
    marker_to_vert: &HashMap<i32, VertKey<M>>,
) -> IdMap<VERT, M>
where
    M: Default + Eq + std::hash::Hash + Copy + Clone,
{
    let mut map = IdMap::<VERT, M>::new();

    for (tet_vertex, node) in tet_mesh.nodes.iter().enumerate() {
        if let Some(&vert_id) = marker_to_vert.get(&node.marker) {
            map.insert(tet_vertex, vert_id);
        }
    }

    map
}

fn extract_boundary_mesh<M: Tag>(tet_mesh: &TetMeshData) -> io::Result<(Mesh<M>, IdMap<VERT, M>)>
where
    M: Default + Eq + std::hash::Hash + Copy + Clone,
{
    let mut face_count = HashMap::<[usize; 3], usize>::new();
    let mut face_original = HashMap::<[usize; 3], [usize; 3]>::new();

    for &[a, b, c, d] in &tet_mesh.tets {
        for face in [[a, b, c], [a, d, b], [a, c, d], [b, d, c]] {
            let key = sorted_face(face);
            *face_count.entry(key).or_insert(0) += 1;
            face_original.entry(key).or_insert(face);
        }
    }

    let faces_tet = orient_triangle_soup(
        face_count
            .into_iter()
            .filter_map(|(key, count)| (count == 1).then_some(face_original[&key]))
            .collect(),
    )?;

    if faces_tet.is_empty() {
        return Err(io::Error::other("TetGen output has no boundary faces"));
    }

    let mut tet_to_local = HashMap::<usize, usize>::new();
    let mut local_to_tet = Vec::<usize>::new();

    for face in &faces_tet {
        for &v in face {
            if !tet_to_local.contains_key(&v) {
                tet_to_local.insert(v, local_to_tet.len());
                local_to_tet.push(v);
            }
        }
    }

    let positions = local_to_tet
        .iter()
        .map(|&v| tet_mesh.nodes[v].position)
        .collect::<Vec<_>>();

    let faces = faces_tet
        .iter()
        .map(|face| {
            face.iter()
                .map(|v| tet_to_local[v])
                .rev()
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();

    let (boundary, boundary_vert_map, _) = Mesh::<M>::from(&faces, &positions)
        .map_err(|err| io::Error::other(format!("failed to build boundary mesh: {err:?}")))?;

    let mut tet_vertex_to_boundary_vertex = IdMap::<VERT, M>::new();

    for boundary_vert in boundary.vert_ids() {
        let local = boundary_vert_map
            .id(&boundary_vert)
            .copied()
            .ok_or_else(|| io::Error::other("missing boundary vertex in builder id map"))?;

        tet_vertex_to_boundary_vertex.insert(local_to_tet[local], boundary_vert);
    }

    Ok((boundary, tet_vertex_to_boundary_vertex))
}

fn write_vtk(path: &Path, tet_mesh: &TetMeshData) -> io::Result<()> {
    let mut text = String::new();

    writeln!(text, "# vtk DataFile Version 3.0").unwrap();
    writeln!(text, "tet mesh").unwrap();
    writeln!(text, "ASCII").unwrap();
    writeln!(text, "DATASET UNSTRUCTURED_GRID").unwrap();

    writeln!(text, "POINTS {} double", tet_mesh.nodes.len()).unwrap();
    for node in &tet_mesh.nodes {
        let p = node.position;
        writeln!(text, "{:?} {:?} {:?}", p.x, p.y, p.z).unwrap();
    }

    writeln!(
        text,
        "CELLS {} {}",
        tet_mesh.tets.len(),
        tet_mesh.tets.len() * 5
    )
    .unwrap();
    for [a, b, c, d] in &tet_mesh.tets {
        writeln!(text, "4 {} {} {} {}", a, b, c, d).unwrap();
    }

    writeln!(text, "CELL_TYPES {}", tet_mesh.tets.len()).unwrap();
    for _ in &tet_mesh.tets {
        writeln!(text, "10").unwrap();
    }

    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }

    let mut file = File::create(path)?;
    file.write_all(text.as_bytes())?;
    file.sync_all()?;

    Ok(())
}

fn tetgen_output_path(smesh_path: &Path, extension: &str) -> io::Result<PathBuf> {
    let stem = smesh_path
        .file_stem()
        .ok_or_else(|| io::Error::other("smesh path has no file stem"))?
        .to_string_lossy();

    Ok(smesh_path
        .parent()
        .unwrap_or_else(|| Path::new(""))
        .join(format!("{stem}.1.{extension}")))
}

fn cleaned_lines(path: &Path) -> io::Result<Vec<String>> {
    Ok(fs::read_to_string(path)?
        .lines()
        .filter_map(|line| {
            let line = line.split('#').next().unwrap_or("").trim();
            (!line.is_empty()).then_some(line.to_owned())
        })
        .collect())
}

fn parse_usizes(line: Option<&String>, path: &Path, context: &str) -> io::Result<Vec<usize>> {
    let line = line.ok_or_else(|| {
        io::Error::other(format!(
            "missing {context} while reading '{}'",
            path.display()
        ))
    })?;

    line.split_whitespace()
        .map(|token| parse_token(token, path, context))
        .collect()
}

fn parse_token<T>(token: &str, path: &Path, context: &str) -> io::Result<T>
where
    T: std::str::FromStr,
    T::Err: std::fmt::Display,
{
    token.parse().map_err(|err| {
        io::Error::other(format!(
            "failed to parse {context} token '{token}' in '{}': {err}",
            path.display()
        ))
    })
}

fn raw_to_local_id(
    raw_id: usize,
    raw_to_local: &HashMap<usize, usize>,
    path: &Path,
) -> io::Result<usize> {
    raw_to_local.get(&raw_id).copied().ok_or_else(|| {
        io::Error::other(format!(
            "cell references missing node id {raw_id} while reading '{}'",
            path.display()
        ))
    })
}

fn remove_if_exists(path: &Path) -> io::Result<()> {
    match fs::remove_file(path) {
        Ok(()) => Ok(()),
        Err(err) if err.kind() == io::ErrorKind::NotFound => Ok(()),
        Err(err) => Err(err),
    }
}

fn sorted_face(mut face: [usize; 3]) -> [usize; 3] {
    face.sort();
    face
}

fn sorted_edge(a: usize, b: usize) -> (usize, usize) {
    if a < b { (a, b) } else { (b, a) }
}

fn directed_edges(face: [usize; 3]) -> [(usize, usize); 3] {
    [(face[0], face[1]), (face[1], face[2]), (face[2], face[0])]
}

fn has_directed_edge(face: [usize; 3], a: usize, b: usize) -> bool {
    directed_edges(face).iter().any(|&(x, y)| x == a && y == b)
}

fn flip_face(face: [usize; 3]) -> [usize; 3] {
    [face[0], face[2], face[1]]
}

fn orient_triangle_soup(faces: Vec<[usize; 3]>) -> io::Result<Vec<[usize; 3]>> {
    let mut edge_to_faces = HashMap::<(usize, usize), Vec<usize>>::new();

    for (face_id, face) in faces.iter().copied().enumerate() {
        for (a, b) in directed_edges(face) {
            edge_to_faces
                .entry(sorted_edge(a, b))
                .or_default()
                .push(face_id);
        }
    }

    for (edge, incident) in &edge_to_faces {
        if incident.len() != 2 {
            return Err(io::Error::other(format!(
                "boundary is not closed/manifold at edge {edge:?}: {} incident faces",
                incident.len()
            )));
        }
    }

    let mut oriented = vec![None::<[usize; 3]>; faces.len()];

    for seed in 0..faces.len() {
        if oriented[seed].is_some() {
            continue;
        }

        oriented[seed] = Some(faces[seed]);
        let mut stack = vec![seed];

        while let Some(face_id) = stack.pop() {
            let face = oriented[face_id].unwrap();

            for (a, b) in directed_edges(face) {
                let incident = &edge_to_faces[&sorted_edge(a, b)];
                let other_id = if incident[0] == face_id {
                    incident[1]
                } else {
                    incident[0]
                };

                if let Some(other) = oriented[other_id] {
                    if !has_directed_edge(other, b, a) {
                        return Err(io::Error::other(
                            "boundary face orientation is inconsistent",
                        ));
                    }
                    continue;
                }

                let candidate = faces[other_id];
                let other = if has_directed_edge(candidate, b, a) {
                    candidate
                } else if has_directed_edge(candidate, a, b) {
                    flip_face(candidate)
                } else {
                    return Err(io::Error::other(
                        "adjacent faces do not share expected edge",
                    ));
                };

                oriented[other_id] = Some(other);
                stack.push(other_id);
            }
        }
    }

    oriented
        .into_iter()
        .map(|face| face.ok_or_else(|| io::Error::other("failed to orient boundary face")))
        .collect()
}
