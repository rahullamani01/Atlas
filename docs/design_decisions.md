# Design decisions

- Angles are in radians everywhere inside the code. Degrees only at the edges.
- Headings are wrapped to [-pi, pi).
- Pose is q = (x, y, theta); theta = 0 points along +x, counter-clockwise positive.
- Source layout: src/atlas/<component>/ with tests in tests/.
