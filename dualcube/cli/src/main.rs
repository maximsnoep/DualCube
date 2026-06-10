use std::env;

fn main() {
    if let Err(err) = cli::cli_main(env::args().collect()) {
        eprintln!("error: {err}");
        std::process::exit(1);
    }
}
