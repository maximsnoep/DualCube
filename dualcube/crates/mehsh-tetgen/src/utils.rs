use std::{fs, io, path::Path};

pub(crate) fn remove_if_exists(path: &Path) -> io::Result<()> {
    match fs::remove_file(path) {
        Ok(()) => Ok(()),
        Err(err) if err.kind() == io::ErrorKind::NotFound => Ok(()),
        Err(err) => Err(err),
    }
}

pub(crate) fn cleaned_lines(path: &Path) -> io::Result<Vec<String>> {
    Ok(fs::read_to_string(path)?
        .lines()
        .filter_map(|line| {
            let line = line.split('#').next()?.trim();
            (!line.is_empty()).then(|| line.to_owned())
        })
        .collect())
}

pub(crate) fn write_text(path: &Path, text: String) -> io::Result<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    fs::write(path, text)
}

pub(crate) fn parse_usizes(
    line: Option<&String>,
    path: &Path,
    context: &str,
) -> io::Result<Vec<usize>> {
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

pub(crate) fn parse_token<T>(token: &str, path: &Path, context: &str) -> io::Result<T>
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
