//! Basin approximation for the determinstic Hénon extended boundary map 
//! 
//! The independent state is `(x, y, theta)`, where `n = (cos(theta), sin(theta))` 
//! is the unit boundary normal.
//! 
//! The implementation constructs conservative box-image enclosures and a reverse 
//! transtion graph. It returns:
//! 
//! - an inner approximation: every enclosed successor reaches the target;
//! - an outer approximation: at least one enclosed successor can reach the target;
//! - an unresolved band between the two approximations.
//! 
//! 
//!  