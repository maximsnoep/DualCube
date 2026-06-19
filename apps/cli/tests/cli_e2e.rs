use cli::cli_main;
use std::fs;
use std::path::PathBuf;

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .and_then(|p| p.parent())
        .unwrap()
        .to_path_buf()
}

fn test_output_dir(name: &str) -> PathBuf {
    let dir = repo_root().join("target").join("cli-e2e-tests").join(name);
    let _ = fs::remove_dir_all(&dir);
    fs::create_dir_all(&dir).unwrap();
    dir
}

fn example_model(name: &str) -> PathBuf {
    repo_root().join("out").join("examples").join(name)
}

fn maybe_models() -> Vec<PathBuf> {
    ["cube.obj", "plane.obj", "bunny.obj"]
        .into_iter()
        .map(example_model)
        .filter(|p| p.exists())
        .collect()
}

fn run_cli(args: &[&str]) {
    let argv = std::iter::once("cli".to_owned())
        .chain(args.iter().map(|s| (*s).to_owned()))
        .collect::<Vec<_>>();
    cli_main(argv).unwrap();
}

#[test]
fn import_command_runs_on_example_model() {
    let input = example_model("cube.obj");
    assert!(input.exists(), "missing test model: {}", input.display());

    run_cli(&["import", &input.display().to_string()]);
}

#[test]
fn initialize_then_export_dsol_end_to_end() {
    let input = example_model("cube.obj");
    assert!(input.exists(), "missing test model: {}", input.display());

    let out_dir = test_output_dir("initialize_then_export_dsol_end_to_end");
    let output_base = out_dir.join("cube_initialized");

    run_cli(&[
        "initialize",
        "--input",
        &input.display().to_string(),
        "--output",
        &output_base.display().to_string(),
    ]);

    let output_file = output_base.with_extension("dsol");
    assert!(
        output_file.exists(),
        "expected output file to exist: {}",
        output_file.display()
    );
}

#[test]
fn reconstruct_command_runs_from_initialized_solution() {
    let input = example_model("cube.obj");
    assert!(input.exists(), "missing test model: {}", input.display());

    let out_dir = test_output_dir("reconstruct_command_runs_from_initialized_solution");
    let initialized_base = out_dir.join("cube_initialized");
    let reconstructed_base = out_dir.join("cube_reconstructed");

    run_cli(&[
        "initialize",
        "--input",
        &input.display().to_string(),
        "--output",
        &initialized_base.display().to_string(),
    ]);

    let initialized_file = initialized_base.with_extension("dsol");
    assert!(
        initialized_file.exists(),
        "expected initialized file to exist: {}",
        initialized_file.display()
    );

    run_cli(&[
        "reconstruct",
        "--input",
        &initialized_file.display().to_string(),
        "--output",
        &reconstructed_base.display().to_string(),
        "--unit",
        "--omega",
        "5",
    ]);

    let reconstructed_file = reconstructed_base.with_extension("dsol");
    assert!(
        reconstructed_file.exists(),
        "expected reconstructed file to exist: {}",
        reconstructed_file.display()
    );
}

#[test]
fn initialize_smoke_runs_across_available_models() {
    let inputs = maybe_models();
    assert!(
        !inputs.is_empty(),
        "expected at least one example model to exist"
    );

    let out_dir = test_output_dir("initialize_smoke_runs_across_available_models");

    for input in inputs {
        let output_base = out_dir.join(input.file_stem().unwrap());
        run_cli(&[
            "initialize",
            "--input",
            &input.display().to_string(),
            "--output",
            &output_base.display().to_string(),
        ]);

        let output_file = output_base.with_extension("dsol");
        assert!(
            output_file.exists(),
            "expected output file to exist: {}",
            output_file.display()
        );
    }
}
