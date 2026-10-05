# Input Vector
- 3 sums of F
- 3 sums of M
- dt
	- step size?

**Frames?**
- position point?
- CoR?

### Attitude representation
Quaternions or Euler Numbers
https://ahrs.readthedocs.io/en/latest/filters.html
https://www.mathworks.com/help/aeroblks/6dofquaternion.html

### Time variants 
- mass
- CoM / CoR
- air resistance
- step size?

---
# Numerical Integration
> Probably Fixed step RK4
- step size? 

---
# Program Structure
**Units**
- unit safe type?
- strict naming / double alias?

**Testing Suite**
- lowk, claude got this

---
# Simple Helpers:

**Euler to Quaternion**
**Quaternion to Euler**

**Variable Rocket Mass**
- should resolve CoM as well


---
- Translational: m·dv/dt = F (plus ω × mv if using body-frame velocity).
- Rotational: Euler's equation, I·dω/dt + ω × (Iω) = M. 
- Quaternion kinematics: q̇ = ½ q ⊗ [0, ω].
- Use the full inertia tensor (including products of inertia) if vehicle is not well-aligned with principal axes.
