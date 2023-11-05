# NUSTARS Payload Autopilot Code

## Contents

- [About the Project](#about-the-project)
- [Dependencies and Installation](#dependencies-and-installation)
- [Team Management Protocols](#team-management-protocols)
- [Pseudocode and Organization](#pseudocode-and-organization)

## About the Project

This project contains the codebase for the entire payload system for the 2023-2024 NASA Student Launch Competition.

## Dependencies and Installation
#### Downloading Code

```bash
cd PROJECT_ROOTDIRECTORY
git clone https://github.com/EnriqueM12/NUSTARS-Payload_Electrical-Autopilot NUSTARS-Autopilot
cd NUSTARS-Autopilot
```

The payload scripts should work directy if you open them up in the Arduino IDE. Bellow are instructions in how to build the client in a local system

#### Setting up Locally

Make sure you have the following dependencies installed in your system
- [cmake][]
- [gcc][]
- [make][]
- [python3][]

**Installing Dependencies on Mac using [Homebrew][]**

```bash
brew install cmake gcc make python
```

**Installing Dependencies on Ubuntu**

```bash
sudo apt install cmake gcc make python3
```

**Installing Dependencies on Arch**

```bash
sudo pacman -S cmake gcc make python3
```

**Building**
```bash
mkdir build
cd build
cmake ..
cmake --build .
```


#### Setting up in [Docker][]

## Team Management Protocols

## Pseudocode and Organization

[cmake]: https://www.cmake.org
[gcc]: https://gcc.gnu.org/
[make]: https://www.gnu.org/software/make/
[python3]: https://www.python.org/
[Homebrew]: https://brew.sh/
[Docker]: https://www.docker.com/
