# NORA (Navigation using Optitrack for Robot Autonomy)

## Description

NORA is a simple, configurable and resuable module capable of retreiving rigid body state data from Optitrack and publishing it over DDS via ROS2. It provides a custom message interface (nora_idl) to allow for

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [Features](#features)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgments](#acknowledgments)

## Installation

1. **Prerequisites**:

   - ROS2 Humble
   - eigen3
   - yaml-cpp
   - nora_idl

2. **Steps**:

   Create a workspace and clone this repo in the src directory of that workspace such that the folder structure is

```sh
nora_ws
   |── src
        │── nora
        │── nora_idl
```

Change directories to the nora package and run the script to build and run NORA

```sh
$ cd nora
$ chmod +x ./nora.sh
$ ./nora.sh
```

## Usage

### Example

To build and run NORA

```bash
$ chmod +x ./nora.sh
$ ./nora.sh
```

To build debians for NORA and NORA_IDL, first ensure the CHANGELOG file is up to date with the latest version number.
Then, change directories to the nora package and run

```sh
$ chmod +x ./debians.sh
$ ./debians.sh 0.0.2
```

where 0.0.2 needs to match the latest version number in the CHANGELOG.rst file. NOTE: this requires that the nora_idl package maintain the same version
as the main nora pkg.
