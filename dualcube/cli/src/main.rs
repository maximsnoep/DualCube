use std::env;

struct CliLogger;

impl log::Log for CliLogger {
    fn enabled(&self, metadata: &log::Metadata<'_>) -> bool {
        metadata.level() <= log::Level::Info
    }

    fn log(&self, record: &log::Record<'_>) {
        if self.enabled(record.metadata()) {
            eprintln!("{} {}: {}", record.level(), record.target(), record.args());
        }
    }

    fn flush(&self) {}
}

static LOGGER: CliLogger = CliLogger;

fn main() {
    let _ = log::set_logger(&LOGGER).map(|()| log::set_max_level(log::LevelFilter::Info));

    if let Err(err) = cli::cli_main(env::args().collect()) {
        eprintln!("error: {err}");
        std::process::exit(1);
    }
}
