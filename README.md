# Overview
The work presented in this repository was carried out at the Chair of Real-Time Systems @ RPTU Kaiserslautern during my time there as a HiWi.

Zephyr was first evaluated through a series of small, exploratory applications, and once its suitability was confirmed, the work culminated in the development of a self‑balancing robot.

In parallel with the software development, a modular hardware architecture
was designed for the robot. This includes a complete schematic and a set of
interchangeable sensor modules that can be attached to the self-balancing
chassis for experimentation and extension.

The repository also includes several additional applications that, while not directly part of the main project, were developed as exploratory experiments for potential future work, along with a few self‑learning projects.


## Initializing a Zephyr Workspace Using This Repository
It is assumed that you have separately installed the Zephyr SDK. If not, you can follow the steps described [here](https://docs.zephyrproject.org/latest/develop/toolchains/zephyr_sdk.html#toolchain-zephyr-sdk). 


This repository contains a `west.yml` manifest and some Zephyr
applications located in the `applications/` directory.

It can be used as a west manifest repository to initialize a Zephyr
workspace, or cloned into an existing workspace as an application
collection.
This repository can be used as a west manifest repository to initialize a Zephyr workspace. In order to do so, follow the steps below:
1. Create a directory which you intend to use as the workspace and navigate to it
    ```bash
    mkdir <workspace> && cd <workspace>
    ```
2. Initialize and activate a virtual environment inside this workspace
    ```bash
    python3 -m venv ~/<workspace>/.venv
    ```
    ```bash
    source .venv/bin/activate
    ```
3. Initialize your West Workspace using this repository as follows:
    ```bash
    west init -m <this_repo_url> <workspace>
    ```
4. Run `west update` inside the workspace folder to get all the dependencies
    ```bash
    west update
    ```
5. Export a Zephyr CMake package. This allows CMake to automatically load boilerplate code required for building Zephyr applications.
    ```bash
    west zephyr-export
    ```
6. Install Python dependencies using `west packages`
    ```bash
    west packages pip --install
    ```
7. Fetch the blobs specified in the manifest
    ```bash
    west blobs fetch
    ```
