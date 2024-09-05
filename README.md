
<!-- PROJECT LOGO -->
<br />
<div align="center">
    <h3 align ="center" One coil microrheometer>
    <p align ="center">
    Authors: Arttu Lehtonen
    <br />
    PokkiLab
    <br />
</p>
</div>

<!-- Table of contents -->
<details>
    <summary> Table of contents </summary>
    <ol>
        <li><a href = "#Introduction"> Introduction</a> </li>
        <ul>
        <li><a href="#Theory">Theory</a></li>
        <li><a href="#Hardware">Hardware</a></li>
        <li><a href="#Parts">Parts</a></li>
        <li><a href="#Hardware">Software</a></li>
        </ul>
        <li><a href = "#Pixel Size log"> Pixel Size log</a> </li>
        <li><a href = "#Plans"> Plans</a> </li>
    </ol>
</details>

# Introduction

This code presents necessary background, hardware, and software to build a magnetic microrheometer for biomaterial microrheology purposes. In short, the system consist of one electromagnet with sharp-ended core. Currents are supplied from power source via operational amplifier and controlled using computer software. The software is built around pyQt library. It controls coil currents, and microscope camera in parallel. Objects from the video can be tracked in real-time and actively tune the currents based on the object position.


## Hardware

Camera: Basler ace 3.2MP
DAQ: NI PCle-6341
Amplifier: APEX PA51a + IC without safety circuit to minimize effects from the inductive load
Coil: Copper coil estimated 10000 rounds
Core: Cobalt-iron core, vacuflux 50

Circuits can be found from <a href="https://github.com/ArttuD/microrheology/tree/17cb960497b8aadbfc182e9a16a7169b9e020cec/Hardware/Circuit"><strong> Microrheology reprository </strong></a>. Circuits were designed with Autodesk Eagle software and ordered from PCBway.

<p align="center">
<img src="./images/amps.png" width="400" height="225" />
<p>

## Parts

The coil was mounted on Olympus ck21 microcope. All the necessary adapters and microscopy parts were 3D printed from PLA except manual manipulator that was purhaced with microscope. 3D models of the parts can be found under <strong> ./parts </strong> folder. 3D design were drawn with Fusion 365 and printed using Creality Ender 3 pro. From request the parts can be conveted to stl files.

<p align="center">
<img src="./images/parts.png" width="400" height="225" />
<p>

### Software

1) Hardware
    - Main thread: 
        - Qthreading and management
        - interface
        - Signaling with Qt slot-signal
        - Video saving (ffmpeg)
    - Sub-thread: Camera (pyPylon)
        - Acquire images
        - tracking: otsu-binarization and centroid tracking
    - Sub-thread: Ni-DAQ unit (niDaqmx)
        - sampling voltage (coil currents) and magnetic field sensor
        - Output current contol via PID (B and I feedback separate)
        - Option to include Kalman filter (did not improve performance and was slow)
    - Sub-thread: Model 
        - Aquire coordinates from the tracker, locate the bead, and calculate current input adjustment value

### Feedback model


Please, see presentation.ipynb and section "Theory"

$
\begin{align}
F_{\text{bead}}(t) &= V \cdot M(B) \cdot \nabla B(x,y) \\
M_{\text{volumetric}} &= M_{\text{mass}} \cdot \rho \\
f_{\text{bead}} (t) &= 6 \pi r \eta \frac{\partial x}{\partial t} + 6 \pi r \mu x \\
\end{align}
$

$\nabla B(x,y)$ is derived from Comsol FEM simulation (see ./simulation/). 


#### Simulated Bead displacement in magnetic field

<p align="center">
<img src="./images/SimBead.png" width="400" height="225" />
<p>


#### Measured Bead displacement in magnetic field

<p align="center">
<img src="./images/MeasBead.png" width="400" height="300" />
<p>

# Pixel Size Logbook

Day    | Camera | Objective
-------|--------|----------
2305117| 3.45   | x10      






