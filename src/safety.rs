//! goal shaping (clamp to joint limits, cap motion per tick)

pub fn shape(target: u16, prev: u16, min: u16, max: u16, max_step: u16) -> u16 {
    let t = target.clamp(min, max) as i32;
    let p = prev as i32;
    let step = max_step as i32;
    (p + (t - p).clamp(-step, step)) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn within_step() {
        assert_eq!(shape(2100, 2090, 0, 4095, 20), 2100);
    }

    #[test]
    fn step_capped_up() {
        assert_eq!(shape(2200, 2090, 0, 4095, 20), 2110);
    }

    #[test]
    fn step_capped_down() {
        assert_eq!(shape(1000, 2090, 0, 4095, 20), 2070);
    }

    #[test]
    fn clamped_to_max() {
        assert_eq!(shape(5000, 4090, 0, 4095, 20), 4095);
    }

    #[test]
    fn clamped_to_min() {
        assert_eq!(shape(50, 110, 100, 4000, 20), 100);
    }
}
