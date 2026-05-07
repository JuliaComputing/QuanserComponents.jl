# Qube STL meshes — origin and license

The STL files in this directory were retrieved from
[Data-Science-in-Mechanical-Engineering/vision-based-furuta-pendulum](https://github.com/Data-Science-in-Mechanical-Engineering/vision-based-furuta-pendulum)
(`gym_brt/data/meshes/`), which is released under the MIT License (Copyright
2021 Max Planck Institute for Intelligent Systems).

That repository's README states the meshes were originally *"provided by
Quanser"*. They depict the Quanser Qube-Servo 2 / Qube-Servo 3 (the two share
the same physical form factor); the upstream README also notes the model is
geometrically approximate.

The meshes are used here for visualization only — mass and inertia in the
`QubePendulum` component come from the dyad parameters, not from the mesh.

## Files in use

- `qube_arm.stl` — rotating arm (`upper_arm.shapefile`)
- `qube_pole.stl` — pendulum rod (`lower_arm.shapefile`)
- `qube_block.stl` — main housing / base, attached statically (`base_box`)
- `qube_motor_drive.stl` — motor shaft hub, rotates with shoulder (`shoulder_cylinder`)
- `qube_motor_main.stl` — motor body, static (`motor_main_mesh`)
- `qube_motor_front.stl` — motor front piece, static (`motor_front_mesh`)
- `qube_motor_part.stl` — small motor mounting piece, static (`motor_part_mesh`)

## Upstream license

```
MIT License — Copyright (c) 2021 Max Planck Institute for Intelligent Systems
```
See <https://github.com/Data-Science-in-Mechanical-Engineering/vision-based-furuta-pendulum/blob/master/LICENSE>
for the full text.
