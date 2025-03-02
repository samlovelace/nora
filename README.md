# NORA (Navigation using Optitrack for Robot Autonomy)

## Description

NORA is a simple, configurable and resuable software module capable of retreiving rigid body state data from Optitrack and publishing it over DDS via ROS2. It provides a custom message interface (nora_idl) to allow for

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [Features](#features)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgments](#acknowledgments)

## Installation

NOTE: these are the manual steps for compiling from source. I am looking at how to package things nicely into a debian.

1. **Prerequisites**:

   - ROS2 Humble
   - eigen3
   - yaml-cpp
   - nora_idl

2. **Steps**:
   - Step-by-step instructions to install and set up.

## Usage

### Example

To build and run NORA

```bash
$ chmod +x ./nora.sh
$ ./nora.sh
```
