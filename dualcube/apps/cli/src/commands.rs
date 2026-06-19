use crate::parser::{Args, Command};
use dualcube::prelude::*;
use io::Export;
use std::path::{Path, PathBuf};

pub fn run(args: anyhow::Result<Args>) -> anyhow::Result<()> {
    let args = args?;

    match args.command {
        Command::Help => {
            print_help();
        }
        Command::Import { input } => {
            let solution = io::import_solution(&input);
            println!(
                "Imported `{}`: verts={}, faces={}, loops={}",
                input.display(),
                solution.mesh_ref.nr_verts(),
                solution.mesh_ref.nr_faces(),
                solution.loops.len()
            );
        }
        Command::Initialize {
            input,
            output,
            samples,
            reconstruct,
            unit,
            omega,
        } => {
            let mut solution = io::import_solution(&input);

            if samples != 3 {
                println!(
                    "warning: initialize currently uses the library default sampling count internally; requested samples={samples}"
                );
            }

            solution.initialize();

            if reconstruct {
                solution.reconstruct_solution(unit, omega)?;
            }

            let output = output.unwrap_or_else(|| default_output(&input, "dc"));
            export_solution(&solution, &output)?;
            println!("Wrote `{}`", output.display());
            print_solution_summary("initialize", &input, &output, &solution);
        }
        Command::Evolve {
            input,
            output,
            iterations,
            pool1,
            pool2,
            reconstruct,
            unit,
            omega,
        } => {
            let solution = io::import_solution(&input);

            let Ok(mut evolved) = solution.evolve(iterations, pool1, pool2) else {
                anyhow::bail!("evolution produced no valid solution");
            };

            if reconstruct {
                evolved.reconstruct_solution(unit, omega)?;
            }

            let output = output.unwrap_or_else(|| default_output(&input, "dc"));
            export_solution(&evolved, &output)?;
            println!("Wrote `{}`", output.display());
            print_solution_summary("evolve", &input, &output, &evolved);
        }
        Command::Reconstruct {
            input,
            output,
            unit,
            omega,
        } => {
            let mut solution = io::import_solution(&input);
            solution.reconstruct_solution(unit, omega)?;

            let output = output.unwrap_or_else(|| default_output(&input, "dc"));
            export_solution(&solution, &output)?;
            println!("Wrote `{}`", output.display());
            print_solution_summary("reconstruct", &input, &output, &solution);
        }
        Command::Export {
            input,
            output,
            format,
        } => {
            let solution = io::import_solution(&input);
            let output = output.unwrap_or_else(|| PathBuf::from("output"));
            export_solution_as(&solution, &output, &format)?;
            println!("Wrote `{}`", output.display());
            print_solution_summary("export", &input, &output, &solution);
        }
    }

    Ok(())
}

fn default_output(input: &Path, extension: &str) -> PathBuf {
    input.with_extension(extension)
}

fn export_solution(solution: &Solution, output: &Path) -> anyhow::Result<()> {
    let ext = output
        .extension()
        .and_then(|x| x.to_str())
        .ok_or_else(|| anyhow::anyhow!("output path must have an extension"))?;
    export_solution_as(solution, output, ext)
}

fn export_solution_as(solution: &Solution, output: &Path, format: &str) -> anyhow::Result<()> {
    match format {
        "dc" | "dsol" => io::Dc::export(solution, output),
        "obj" => io::OBJ::export(solution, output),
        "flag" => io::Flag::export(solution, output),
        "apg" => io::APG::export(solution, output),
        "loops" => io::Loops::export(solution, output),
        "nlr" => io::NLR::export(solution, output),
        "hex" | "hex.mesh" => io::HEX::export(solution, output),
        other => anyhow::bail!("unsupported export format: {other}"),
    }
}

fn print_solution_summary(command: &str, input: &Path, output: &Path, solution: &Solution) {
    println!(
        "summary command={} input={} output={} quality={} loops={} verts={} faces={} dual={} layout={} polycube={} quad={}",
        command,
        input.display(),
        output.display(),
        solution
            .get_quality()
            .map_or_else(|| "none".to_owned(), |q| format!("{q:.6}")),
        solution.loops.len(),
        solution.mesh_ref.nr_verts(),
        solution.mesh_ref.nr_faces(),
        solution.dual.is_ok(),
        solution.layout.is_some(),
        solution.polycube.is_some(),
        solution.quad.is_some()
    );
}

fn print_help() {
    println!(
        "\
DualCube CLI

Commands:
  help
      Show this help text.

  import <input>
      Import a mesh/solution and print a short summary.

  initialize --input <path> [--output <path>] [--samples <n>] [--reconstruct] [--unit|--no-unit] [--omega <n>]
      Initialize loop structure from an input mesh or solution.
      Writes the result to --output, or defaults to changing the extension to .dc.

  evolve --input <path> [--output <path>] [--iterations <n>] [--pool1 <n>] [--pool2 <n>] [--reconstruct] [--unit|--no-unit] [--omega <n>]
      Evolve an existing solution.

  reconstruct --input <path> [--output <path>] [--unit|--no-unit] [--omega <n>]
      Reconstruct dual/layout/polyclube/quad state from the current loops.

  export --input <path> --format <dc|dsol|obj|flag|apg|loops|nlr|hex> [--output <path>]
      Export a solution in the requested format.

Examples:
  cli import bunny.obj
  cli initialize --input bunny.obj --output bunny.dc --reconstruct --omega 5
  cli evolve --input bunny.dc --iterations 10 --pool1 10 --pool2 30 --output evolved.dc
  cli reconstruct --input loops.loops --output result.dc --unit --omega 5
  cli export --input result.dc --format obj --output result.obj
"
    );
}
