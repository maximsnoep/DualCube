# DualCube, robust polycube-maps

Implementation of [Polycubes via Dual Loops](https://arxiv.org/abs/2410.16865) (IMR 2025), [Robust Construction of Polycube Segmentations via Dual Loops](https://arxiv.org/abs/2402.00652) (preprint), and more.



## Prerequisites
Ensure Rust and Cargo are installed on your system. 

Use `rustup` for installation:
- **Unix Systems**:
  `curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`
- **Windows Systems**: 
  Download and run [rustup-init.exe](https://static.rust-lang.org/rustup/dist/i686-pc-windows-msvc/rustup-init.exe)

To update Rust and Cargo to the latest version:
```bash
rustup update
```

## Download the project
Clone the repository from GitHub:
```bash
git clone https://www.github.com/maximsnoep/DualCube
```
If you already have the project, update it to the latest version:
```bash
git pull
```

## Run the project
Compile and run the project using Cargo:
```bash
cargo run
```
For a faster compilation time with a less optimized build, use:
```bash
cargo run --profile dev
```
