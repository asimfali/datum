//! Whether a newer release is out (D96): the latest release of the repository the program is
//! built from, asked of GitHub once, in the background, through `curl` (in Windows since 10, in
//! nearly every Linux): no HTTP client of our own. No network, no `curl`, or turned off
//! (`DATUM_NO_UPDATE_CHECK`, or a file `no-update-check` beside the program): nothing is said.

use std::process::Command;

/// The latest release, if newer than this program: its version and its page.
pub fn newer() -> Option<(String, String)> {
    let off_file = std::env::current_exe().ok().and_then(|e| e.parent().map(|d| d.join("no-update-check")));
    if std::env::var_os("DATUM_NO_UPDATE_CHECK").is_some() || off_file.is_some_and(|f| f.exists()) {
        return None;
    }
    let api = env!("CARGO_PKG_REPOSITORY").replacen("https://github.com/", "https://api.github.com/repos/", 1) + "/releases/latest";
    let mut curl = Command::new("curl");
    curl.args(["-sSfL", "--max-time", "8", "-H", "Accept: application/vnd.github+json", "-H", "User-Agent: datum", &api]);
    #[cfg(windows)]
    {
        // No console window flashing up from the window's own process.
        use std::os::windows::process::CommandExt;
        curl.creation_flags(0x0800_0000);
    }
    let out = curl.output().ok().filter(|o| o.status.success())?;
    let release: serde_json::Value = serde_json::from_slice(&out.stdout).ok()?;
    let tag = release["tag_name"].as_str()?;
    let url = release["html_url"].as_str()?;
    is_newer(tag, env!("CARGO_PKG_VERSION")).then(|| (tag.trim_start_matches('v').to_string(), url.to_string()))
}

/// Whether version `tag` (`v1.2.3` or `1.2.3`) is later than `current`, part by part.
fn is_newer(tag: &str, current: &str) -> bool {
    let parts = |v: &str| -> Option<Vec<u64>> { v.trim_start_matches('v').split('.').map(|p| p.parse().ok()).collect() };
    match (parts(tag), parts(current)) {
        (Some(a), Some(b)) => a > b,
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn versions_compare_part_by_part() {
        assert!(is_newer("v0.2.0", "0.1.0"));
        assert!(is_newer("v0.10.0", "0.9.3"));
        assert!(!is_newer("v0.1.0", "0.1.0"));
        assert!(!is_newer("v0.1.0", "0.2.0"));
        assert!(!is_newer("nightly", "0.1.0"));
    }
}
