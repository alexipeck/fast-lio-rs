//! Which CPUs `lio`'s threads run on: CPU lists, the CPUs the process may
//! use, core capacities, and pinning the calling thread. Pinning needs
//! Linux; elsewhere `lio` warns and leaves placement to the OS.

/// Parses a CPU list in Linux's notation, such as `0-3,6`.
pub fn parse_list(s: &str) -> Result<Vec<usize>, String> {
    let num = |x: &str| {
        x.trim()
            .parse::<usize>()
            .map_err(|e| format!("CPU list {s:?}: {e}"))
    };
    let mut cpus = Vec::new();
    for part in s.split(',').filter(|p| !p.trim().is_empty()) {
        match part.split_once('-') {
            Some((a, b)) => {
                let (a, b) = (num(a)?, num(b)?);
                if a > b {
                    return Err(format!("CPU list {s:?}: range {a}-{b} runs backwards"));
                }
                cpus.extend(a..=b);
            }
            None => cpus.push(num(part)?),
        }
    }
    cpus.sort_unstable();
    cpus.dedup();
    if cpus.is_empty() {
        return Err(format!("CPU list {s:?} is empty"));
    }
    Ok(cpus)
}

/// Writes a CPU list in Linux's notation, ranges collapsed.
pub fn format_list(cpus: &[usize]) -> String {
    let mut out: Vec<String> = Vec::new();
    let mut i = 0;
    while i < cpus.len() {
        let mut j = i;
        while j + 1 < cpus.len() && cpus[j + 1] == cpus[j] + 1 {
            j += 1;
        }
        out.push(if j == i {
            cpus[i].to_string()
        } else {
            format!("{}-{}", cpus[i], cpus[j])
        });
        i = j + 1;
    }
    out.join(",")
}

/// Whether this platform can pin threads.
pub const CAN_PIN: bool = cfg!(target_os = "linux");

/// The CPUs the calling thread may run on.
#[cfg(target_os = "linux")]
pub fn allowed() -> Vec<usize> {
    use nix::sched::{CpuSet, sched_getaffinity};
    use nix::unistd::Pid;
    match sched_getaffinity(Pid::from_raw(0)) {
        Ok(set) => (0..CpuSet::count())
            .filter(|&c| set.is_set(c).unwrap_or(false))
            .collect(),
        Err(_) => all_cpus(),
    }
}

#[cfg(not(target_os = "linux"))]
pub fn allowed() -> Vec<usize> {
    all_cpus()
}

fn all_cpus() -> Vec<usize> {
    (0..std::thread::available_parallelism().map_or(1, usize::from)).collect()
}

/// Restricts the calling thread, and any thread it starts afterwards, to `cpus`.
#[cfg(target_os = "linux")]
pub fn pin(cpus: &[usize]) -> Result<(), String> {
    use nix::sched::{CpuSet, sched_setaffinity};
    use nix::unistd::Pid;
    let mut set = CpuSet::new();
    for &c in cpus {
        set.set(c).map_err(|e| format!("CPU {c}: {e}"))?;
    }
    sched_setaffinity(Pid::from_raw(0), &set)
        .map_err(|e| format!("pinning to CPUs {}: {e}", format_list(cpus)))
}

#[cfg(not(target_os = "linux"))]
pub fn pin(_cpus: &[usize]) -> Result<(), String> {
    Ok(())
}

/// The fastest of `cpus`, when Linux reports core capacities and they
/// differ, as on ARM big.LITTLE chips.
pub fn fastest(cpus: &[usize]) -> Option<Vec<usize>> {
    let capacity = cpus
        .iter()
        .map(|c| {
            std::fs::read_to_string(format!("/sys/devices/system/cpu/cpu{c}/cpu_capacity"))
                .ok()?
                .trim()
                .parse::<u32>()
                .ok()
        })
        .collect::<Option<Vec<u32>>>()?;
    let max = *capacity.iter().max()?;
    let fast: Vec<usize> = cpus
        .iter()
        .zip(&capacity)
        .filter(|&(_, &c)| c == max)
        .map(|(&cpu, _)| cpu)
        .collect();
    (fast.len() < cpus.len()).then_some(fast)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cpu_lists_round_trip() {
        assert_eq!(parse_list("0-3,6").unwrap(), [0, 1, 2, 3, 6]);
        assert_eq!(parse_list(" 5, 2-3 ,2").unwrap(), [2, 3, 5]);
        assert_eq!(format_list(&[0, 1, 2, 3, 6, 8, 9]), "0-3,6,8-9");
        assert_eq!(format_list(&[4]), "4");
        assert!(parse_list("3-1").is_err());
        assert!(parse_list("").is_err());
        assert!(parse_list("a").is_err());
    }
}
