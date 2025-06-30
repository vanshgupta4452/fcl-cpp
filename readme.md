# FCL + Open3D URDF Collision Project (C++)

This project demonstrates collision detection using the **Flexible Collision Library (FCL)** and visualization using **Open3D** in pure C++. It loads robot models from URDF, converts them into FCL shapes, and performs:

-  Collision checking with environment objects  
-  Self-collision detection  
-  Visualization of robot and collisions using Open3D  

---

##  Dependencies

###  Install via vcpkg (Recommended for FCL + Eigen + Assimp + ConsoleBridge)

```bash
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.sh

# Install required libraries
./vcpkg install fcl eigen3 assimp urdfdom console-bridge
```

Then integrate with CMake using:

```bash
./vcpkg integrate install
```


### Install using apt:

```bash
sudo apt install \
  libfcl-dev \
  libeigen3-dev \
  libassimp-dev \
  liburdfdom-dev \
  libconsole-bridge-dev \
  libopen3d-dev
```

> If `libopen3d-dev` is not available in your distro, install Open3D from source:  
> https://github.com/isl-org/Open3D

---

## Clone & Build

```bash
git clone git@github.com:vanshgupta4452/fcl-cpp.git
cd fcl-cpp
mkdir build && cd build
cmake ..
make
```



---

##  Executables

### 1. `env_coll`

```bash
./env_coll
```

- Loads URDF
- Parses into FCL collision objects
- Performs collision checking with environment (no visualization)

---

### 2. `open3d_env`

```bash
./open3d_env
```

- Same as `env_coll`
- Adds 3D visualization of robot and environment using Open3D

---

### 3. `self_open3d`

```bash
./self_open3d
```

- Loads URDF robot
- Checks for **self-collision**
- Visualizes robot and collisions using Open3D

---




## 📁 File Structure

```
├── CMakeLists.txt
├── env_coll.cpp        # Collision with environment
├── env_open3d.cpp      # Collision + Open3D
├── self_open3d.cpp     # Self-collision + Open3D
├── urdf/               # (Optional) URDF and mesh files
└── build/              # CMake build output
```

---



