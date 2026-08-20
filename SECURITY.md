# Security policy

## Reporting a vulnerability

Do not open a public issue for a suspected vulnerability. Use GitHub's private
vulnerability reporting for this repository. Include affected versions,
reproduction steps, impact, and any suggested mitigation.

## Deployment boundary

Forge Motion provides command-line Dora motion nodes. It performs no
authentication, and its action server commands physical actuators.

- Run motion nodes only on trusted hosts within an isolated robot network.
- Treat motion YAML configuration as safety-relevant input; validate joint
  limits, velocities, and kinematics parameters before deployment to hardware.
- This package does not provide hard real-time motor control. Hardware
  firmware remains responsible for the low-level servo loop and safety limits.
- Do not expose motion processes or their configuration to untrusted networks.
- Validate safety controls independently before connecting a physical robot,
  and keep an operator-accessible emergency stop available.

Security fixes are supported on the latest released version.
