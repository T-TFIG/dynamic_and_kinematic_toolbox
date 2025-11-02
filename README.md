# 🤖 Robotic Toolbox

Welcome to my **Robotic Toolbox** — a personal side project developed as part of a larger **Robot Manipulator Project**, which you can also find on my GitHub.  

This toolbox is designed to provide a **clean**, **modular**, and **extensible** implementation of robotic **kinematics** and **dynamics**, built with mathematical clarity and practical usability in mind.

---

## 🧩 Overview

Traditional robotic toolboxes often rely on:
- Homogeneous Transformation Matrices  
- **Denavit–Hartenberg (DH)** Parameters  

While these classical methods work well, they can become **difficult to extend** or **interpret geometrically**, especially for complex or redundant manipulators.  

This toolbox, instead, adopts **Screw Theory** — a more general and elegant framework for representing rigid-body motion.

---

## ⚙️ Why Screw Theory?

The main motivation for using screw theory lies in its:

- 🔹 **Intuitive geometric meaning** — directly relates rotation and translation  
- 🔹 **Elegant mathematical form** — compact and expressive  
- 🔹 **High practicality** — especially effective for multi-DOF or spatial robots  

> 💡 Screw theory expresses motion as a single exponential of a twist, making it far easier to visualize and compute than chaining multiple transformation matrices.

---

## 🧠 What’s Inside

This toolbox currently includes:

- `FORWARD_KINEMATIC` class — forward kinematics using the **product of exponentials (PoE)** formulation  
- Support for **symbolic computation** with Sympy  
- Extendable structure for future **dynamics**, **Jacobian**, and **control** modules  

Each function is written with readability and extensibility in mind, allowing you to adapt it easily to your robot model.

---

## 🔬 Comparison to Traditional Methods

| Concept | Traditional (DH) | Screw Theory (This Toolbox) |
|----------|------------------|-----------------------------|
| Representation | Frame-by-frame | Continuous twist motion |
| Mathematical Form | Matrix chain | Exponential coordinates |
| Intuition | Limited | High geometric meaning |
| Scalability | Tedious for complex robots | Naturally extensible |

---

## 📘 Example (Preview)

Here’s how screw theory fits in compared to the traditional approach:

<img width="272" height="318" alt="image" src="https://github.com/user-attachments/assets/9192e78b-4e12-418f-a686-4ea0ab9145b3" />

source from: https://www.physicsandbox.com/projects/double-pendulum.html

this image is the example of the double pendulum and we gonna use it for visualize both forward kinematic (FK) algorithm

### DH-parameter

the dh-parameter we use for finding the Forward kinematic (FK) here is the **modified DH-parameter**

| Link i | $a_{i-1}$ | $\alpha_{i-1}$ | $d_{i}$ | $\theta_{i}$ |
|--------|------------|------------|------------|------------|
|   1    |      0     |      0     |      0     |   $\theta_{1}$  |
|   2    |      $l1$     |      0     |      0     |   $\theta_{2}$  |
|   eff    |      0     |      0     |      0     |   0  |
