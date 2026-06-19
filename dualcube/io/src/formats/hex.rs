use log::info;
use std::io::Error;
use std::process::Command;

pub struct HEX;

const HEXMESH_PIPELINE: &str = "~/polycube-to-hexmesh/pipeline.sh";

impl crate::Export for HEX {
    fn export(
        solution: &dualcube::prelude::Solution,
        path: &std::path::Path,
    ) -> anyhow::Result<()> {
        let Some(layout) = solution.layout.as_ref() else {
            anyhow::bail!("No layout available");
        };

        let path_hex = path.with_extension("hex.mesh");
        let path_obj = path.with_extension("obj");
        let path_flag = path.with_extension("flag");

        info!(
            "HEX export paths: requested={} obj={} flag={} hex={}",
            path.display(),
            path_obj.display(),
            path_flag.display(),
            path_hex.display()
        );

        info!("Exporting OBJ file to {}", path_obj.display());
        crate::OBJ::export(solution, &path_obj)?;
        info!("Exporting FLAG file to {}", path_flag.display());
        crate::Flag::export(solution, &path_flag)?;

        let obj_arg = path_obj.display().to_string();
        let hex_arg = path_hex.display().to_string();
        let flag_arg = path_flag.display().to_string();

        info!(
            "Running hexmesh pipeline with raw paths: script={} obj={} out={} flag={}",
            HEXMESH_PIPELINE, obj_arg, hex_arg, flag_arg
        );
        let out = run(
            HEXMESH_PIPELINE,
            &[&obj_arg, "-out", &hex_arg, "-algo", &flag_arg],
        )?;

        info!(
            "hex pipeline finished: status={} stdout_bytes={} stderr_bytes={}",
            out.status,
            out.stdout.len(),
            out.stderr.len()
        );
        info!(
            "hex pipeline stdout: {}",
            String::from_utf8_lossy(&out.stdout)
        );
        info!(
            "hex pipeline stderr: {}",
            String::from_utf8_lossy(&out.stderr)
        );

        anyhow::Context::with_context(layout.granulated_mesh.to_obj(&path_obj), || {
            format!("writing HEX to {}", path_obj.display())
        })?;

        Ok(())
    }
}

fn run(command: &str, args: &[&str]) -> Result<std::process::Output, Error> {
    let mut command_parts = command.split_whitespace();
    let executable = command_parts
        .next()
        .ok_or_else(|| Error::other("hexmesh pipeline command is empty"))?;

    let mut process = Command::new(executable);
    process.args(command_parts);
    process.args(args);

    info!("Running command without path conversion: {:?}", process);
    process.output()
}
