use serde::{Deserialize, Serialize};
use std::collections::HashSet;
use wasm_bindgen::prelude::JsValue;

use serde_wasm_bindgen;

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ParameterEntry {
    pub name: String,
    pub value: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ParameterSet {
    entries: Vec<ParameterEntry>,
}

impl ParameterSet {
    pub fn new(entries: Vec<ParameterEntry>) -> Result<Self, String> {
        let mut seen = HashSet::new();
        let mut normalized = Vec::with_capacity(entries.len());

        for entry in entries {
            let name = entry.name.trim().to_string();
            if name.is_empty() {
                return Err("Parameter name cannot be empty".to_string());
            }
            if is_reserved_param(&name) {
                return Err(format!("Parameter name '{}' is reserved", name));
            }
            if !is_valid_identifier(&name) {
                return Err(format!(
                    "Invalid parameter name '{}'. Use letters, digits, and underscore, starting with a letter or underscore",
                    name
                ));
            }
            if !entry.value.is_finite() {
                return Err(format!("Parameter '{}' must be finite", name));
            }
            if !seen.insert(name.clone()) {
                return Err(format!("Duplicate parameter name '{}'", name));
            }

            normalized.push(ParameterEntry {
                name,
                value: entry.value,
            });
        }

        Ok(Self {
            entries: normalized,
        })
    }

    pub fn empty() -> Self {
        Self {
            entries: Vec::new(),
        }
    }

    pub fn entries(&self) -> &[ParameterEntry] {
        &self.entries
    }
}

pub fn parameter_set_from_js(params: JsValue) -> Result<ParameterSet, String> {
    let entries: Vec<ParameterEntry> = serde_wasm_bindgen::from_value(params)
        .map_err(|e| format!("Failed to parse parameters: {:?}", e))?;
    ParameterSet::new(entries)
}

fn is_valid_identifier(name: &str) -> bool {
    let mut chars = name.chars();
    let Some(first) = chars.next() else {
        return false;
    };
    if !(first.is_ascii_alphabetic() || first == '_') {
        return false;
    }
    chars.all(|c| c.is_ascii_alphanumeric() || c == '_')
}

fn is_reserved_param(name: &str) -> bool {
    matches!(
        name,
        "x" | "y" | "sin" | "cos" | "tan" | "abs" | "sqrt" | "exp" | "ln"
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parameter_set_accepts_valid_names() {
        let params = ParameterSet::new(vec![
            ParameterEntry {
                name: "alpha".to_string(),
                value: 1.0,
            },
            ParameterEntry {
                name: "beta_2".to_string(),
                value: -0.5,
            },
        ])
        .unwrap();

        assert_eq!(params.entries().len(), 2);
    }

    #[test]
    fn test_parameter_set_rejects_reserved() {
        let err = ParameterSet::new(vec![ParameterEntry {
            name: "x".to_string(),
            value: 1.0,
        }])
        .unwrap_err();
        assert!(err.contains("reserved"));
    }

    #[test]
    fn test_parameter_set_rejects_duplicates() {
        let err = ParameterSet::new(vec![
            ParameterEntry {
                name: "a".to_string(),
                value: 1.0,
            },
            ParameterEntry {
                name: "a".to_string(),
                value: 2.0,
            },
        ])
        .unwrap_err();
        assert!(err.contains("Duplicate"));
    }

    #[test]
    fn test_parameter_set_rejects_invalid_identifier() {
        let err = ParameterSet::new(vec![ParameterEntry {
            name: "1bad".to_string(),
            value: 1.0,
        }])
        .unwrap_err();
        assert!(err.contains("Invalid"));
    }

    #[test]
    fn test_parameter_set_rejects_non_finite() {
        let err = ParameterSet::new(vec![ParameterEntry {
            name: "a".to_string(),
            value: f64::NAN,
        }])
        .unwrap_err();
        assert!(err.contains("finite"));
    }
}
