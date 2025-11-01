This Robotic Toolbox is one of my personal side projects, developed as part of a larger Robot Manipulator Project, which you can also find on my GitHub.

The purpose of this toolbox is to provide a clean and modular implementation of robotic kinematics and dynamics — with a particular focus on screw theory.

Unlike traditional approaches that rely on homogeneous transformation matrices or Denavit–Hartenberg (DH) parameters, this toolbox uses screw theory for forward kinematics.

The main reason for adopting screw theory is that it is:

More intuitive and geometrically meaningful,

Elegant in formulation, and

Highly practical when dealing with complex or redundant robot structures.

In the following sections, you will find a comparison between the screw theory formulation and the traditional approach, along with examples of how this toolbox implements them.
