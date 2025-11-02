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

## 📘 Example: Double Pendulum Forward Kinematics

Here’s an example of how **screw theory** fits in compared to the traditional DH method for a **planar double pendulum**:

<img width="272" height="318" alt="Double Pendulum" src="https://github.com/user-attachments/assets/9192e78b-4e12-418f-a686-4ea0ab9145b3" />

*Source: [Physics Sandbox](https://www.physicsandbox.com/projects/double-pendulum.html)*  

This image shows a **double pendulum**, which we will use to **visualize forward kinematics (FK)** using both the analytic method and the modified DH method.

---

### DH Parameters

For this example, we are using the **modified DH convention (link-focused)**. The rules for placing frames are:

1. **z-axis** along the rotation axis of the joint.  
2. **x-axis** along the link (common normal to the next joint axis).  
3. **y-axis** follows the right-hand rule: \(y = z \times x\).  
4. **Origin** at the intersection of x and z axes for each link.  

The **modified DH table** for the planar double pendulum is:

| Link i | $a_{i-1}$ | $\alpha_{i-1}$ | $d_{i}$ | $\theta_{i}$ |
|--------|------------|----------------|---------|---------------|
| 1      | 0          | 0              | 0       | $\pi + \theta_1$ |
| 2      | $l_1$      | 0              | 0       | $\theta_2$ |
| End-effector | $l_2$ | 0           | 0       | 0 |

and each frame can be fit in this matrix to do the transformation matrix

<img width="635" height="99" alt="image" src="https://github.com/user-attachments/assets/54ea7723-6629-46b5-bbd2-0c63bac1a8c9" />

at the end we will get this transformation matrix

$H^0_{End-effector} = T^0_1 * T^1_2 * T^2_{End-effector}$

> **Notes:**  
> - $l_1, l_2$ are the lengths of the first and second links.  
> - The first joint angle is offset by $\pi$ to align the frame with the positive x-axis at the home pose.  
> - All rotations occur in the XY-plane about the z-axis.

---
