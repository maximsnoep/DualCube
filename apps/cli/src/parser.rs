use std::path::PathBuf;

#[derive(Debug, Clone)]
pub struct Args {
    pub command: Command,
}

#[derive(Debug, Clone)]
pub enum Command {
    Help,
    Import {
        input: PathBuf,
    },
    Initialize {
        input: PathBuf,
        output: Option<PathBuf>,
        samples: usize,
        reconstruct: bool,
        unit: bool,
        omega: usize,
    },
    Evolve {
        input: PathBuf,
        output: Option<PathBuf>,
        iterations: usize,
        pool1: usize,
        pool2: usize,
        reconstruct: bool,
        unit: bool,
        omega: usize,
    },
    Reconstruct {
        input: PathBuf,
        output: Option<PathBuf>,
        unit: bool,
        omega: usize,
    },
    Export {
        input: PathBuf,
        output: Option<PathBuf>,
        format: String,
    },
}

impl Args {
    pub fn parse(argv: Vec<String>) -> anyhow::Result<Self> {
        if argv.len() <= 1 {
            return Ok(Self {
                command: Command::Help,
            });
        }

        let mut it = argv.into_iter();
        let _bin = it.next();
        let cmd = it.next().unwrap_or_else(|| "help".to_owned());

        let command = match cmd.as_str() {
            "help" | "--help" | "-h" => Command::Help,
            "import" => {
                let input = take_positional_path(&mut it, "input")?;
                Command::Import { input }
            }
            "initialize" | "init" => {
                let mut input = None;
                let mut output = None;
                let mut samples = 3usize;
                let mut reconstruct = false;
                let mut unit = true;
                let mut omega = 1usize;

                parse_kv_flags(it.collect(), |flag, value| {
                    match flag {
                        "--input" | "-i" => {
                            input = Some(PathBuf::from(require_value(flag, value)?))
                        }
                        "--output" | "-o" => {
                            output = Some(PathBuf::from(require_value(flag, value)?))
                        }
                        "--samples" | "-s" => {
                            samples = require_value(flag, value)?.parse::<usize>()?;
                        }
                        "--reconstruct" => reconstruct = true,
                        "--no-unit" => unit = false,
                        "--unit" => unit = true,
                        "--omega" => {
                            omega = require_value(flag, value)?.parse::<usize>()?;
                        }
                        other => anyhow::bail!("unknown flag for initialize: {other}"),
                    }
                    Ok(())
                })?;

                let input = input.ok_or_else(|| anyhow::anyhow!("missing required --input"))?;
                Command::Initialize {
                    input,
                    output,
                    samples,
                    reconstruct,
                    unit,
                    omega,
                }
            }
            "evolve" => {
                let mut input = None;
                let mut output = None;
                let mut iterations = 10usize;
                let mut pool1 = 10usize;
                let mut pool2 = 30usize;
                let mut reconstruct = false;
                let mut unit = true;
                let mut omega = 1usize;

                parse_kv_flags(it.collect(), |flag, value| {
                    match flag {
                        "--input" | "-i" => {
                            input = Some(PathBuf::from(require_value(flag, value)?))
                        }
                        "--output" | "-o" => {
                            output = Some(PathBuf::from(require_value(flag, value)?))
                        }
                        "--iterations" => {
                            iterations = require_value(flag, value)?.parse::<usize>()?;
                        }
                        "--pool1" => {
                            pool1 = require_value(flag, value)?.parse::<usize>()?;
                        }
                        "--pool2" => {
                            pool2 = require_value(flag, value)?.parse::<usize>()?;
                        }
                        "--reconstruct" => reconstruct = true,
                        "--no-unit" => unit = false,
                        "--unit" => unit = true,
                        "--omega" => {
                            omega = require_value(flag, value)?.parse::<usize>()?;
                        }
                        other => anyhow::bail!("unknown flag for evolve: {other}"),
                    }
                    Ok(())
                })?;

                let input = input.ok_or_else(|| anyhow::anyhow!("missing required --input"))?;
                Command::Evolve {
                    input,
                    output,
                    iterations,
                    pool1,
                    pool2,
                    reconstruct,
                    unit,
                    omega,
                }
            }
            "reconstruct" => {
                let mut input = None;
                let mut output = None;
                let mut unit = true;
                let mut omega = 1usize;

                parse_kv_flags(it.collect(), |flag, value| {
                    match flag {
                        "--input" | "-i" => {
                            input = Some(PathBuf::from(require_value(flag, value)?))
                        }
                        "--output" | "-o" => {
                            output = Some(PathBuf::from(require_value(flag, value)?))
                        }
                        "--no-unit" => unit = false,
                        "--unit" => unit = true,
                        "--omega" => {
                            omega = require_value(flag, value)?.parse::<usize>()?;
                        }
                        other => anyhow::bail!("unknown flag for reconstruct: {other}"),
                    }
                    Ok(())
                })?;

                let input = input.ok_or_else(|| anyhow::anyhow!("missing required --input"))?;
                Command::Reconstruct {
                    input,
                    output,
                    unit,
                    omega,
                }
            }
            "export" => {
                let mut input = None;
                let mut output = None;
                let mut format = None;

                parse_kv_flags(it.collect(), |flag, value| {
                    match flag {
                        "--input" | "-i" => {
                            input = Some(PathBuf::from(require_value(flag, value)?))
                        }
                        "--output" | "-o" => {
                            output = Some(PathBuf::from(require_value(flag, value)?))
                        }
                        "--format" | "-f" => {
                            format = Some(require_value(flag, value)?.to_owned());
                        }
                        other => anyhow::bail!("unknown flag for export: {other}"),
                    }
                    Ok(())
                })?;

                let input = input.ok_or_else(|| anyhow::anyhow!("missing required --input"))?;
                let format = format.ok_or_else(|| anyhow::anyhow!("missing required --format"))?;
                Command::Export {
                    input,
                    output,
                    format,
                }
            }
            other => anyhow::bail!("unknown command: {other}"),
        };

        Ok(Self { command })
    }
}

fn require_value<'a>(flag: &str, value: Option<&'a str>) -> anyhow::Result<&'a str> {
    value.ok_or_else(|| anyhow::anyhow!("missing value for {flag}"))
}

fn take_positional_path<I>(it: &mut I, name: &str) -> anyhow::Result<PathBuf>
where
    I: Iterator<Item = String>,
{
    it.next()
        .map(PathBuf::from)
        .ok_or_else(|| anyhow::anyhow!("missing required positional argument `{name}`"))
}

fn parse_kv_flags<F>(args: Vec<String>, mut f: F) -> anyhow::Result<()>
where
    F: FnMut(&str, Option<&str>) -> anyhow::Result<()>,
{
    let mut i = 0usize;
    while i < args.len() {
        let flag = &args[i];
        if !flag.starts_with('-') {
            anyhow::bail!("unexpected positional argument: {}", args[i]);
        }

        let takes_value = matches!(
            flag.as_str(),
            "--input"
                | "-i"
                | "--output"
                | "-o"
                | "--samples"
                | "-s"
                | "--iterations"
                | "--pool1"
                | "--pool2"
                | "--omega"
                | "--format"
                | "-f"
        );

        let value = if takes_value {
            i += 1;
            Some(
                args.get(i)
                    .ok_or_else(|| anyhow::anyhow!("missing value for {}", flag))?
                    .as_str(),
            )
        } else {
            None
        };

        f(flag, value)?;
        i += 1;
    }
    Ok(())
}
