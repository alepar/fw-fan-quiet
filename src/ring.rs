// Fixed-capacity history buffer backing the UI's scrolling charts (used from task 6).
#![allow(dead_code)]

use std::collections::VecDeque;

pub struct Ring {
    buf: VecDeque<f64>,
    cap: usize,
}

impl Ring {
    pub fn new(cap: usize) -> Self {
        Self {
            buf: VecDeque::with_capacity(cap),
            cap,
        }
    }

    pub fn push(&mut self, v: f64) {
        if self.buf.len() == self.cap {
            self.buf.pop_front();
        }
        self.buf.push_back(v);
    }

    pub fn last(&self) -> Option<f64> {
        self.buf.back().copied()
    }

    pub fn iter(&self) -> impl Iterator<Item = &f64> {
        self.buf.iter()
    }

    pub fn len(&self) -> usize {
        self.buf.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ring_keeps_last_n() {
        let mut r = Ring::new(3);
        for i in 0..5 {
            r.push(i as f64);
        }
        assert_eq!(r.iter().copied().collect::<Vec<_>>(), vec![2.0, 3.0, 4.0]);
        assert_eq!(r.last(), Some(4.0));
    }

    #[test]
    fn ring_handles_empty() {
        let r = Ring::new(3);
        assert_eq!(r.last(), None);
        assert_eq!(r.iter().count(), 0);
    }
}
