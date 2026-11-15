# lamp-robot

A 6-axis expressive lamp robot. It notices people, tries to get their attention, remembers objects it sees, and answers questions about them by voice.

Work in progress.

- [Build journal](https://docs.google.com/document/d/1FafV6sYWJsIyEPJD50v6SspsQr1C0WBp2hjmtgFsbPM/edit?usp=sharing)
- [Bill of materials (BOM)](https://docs.google.com/spreadsheets/d/1V-kWyB_abHdd5QoXr_BD3az193OaC49rbLtoEylhK_0/edit?usp=sharing)
- [Build photos](images/build)
- [Schematic images](images/schematic)
- Link to the challenge brief: [REFERENCES.md](REFERENCES.md)
- License: [MIT](LICENSE)

## Next steps

- Write the URDF and a PyBullet sim to test joint limits and motion
- Re-run the torque check on the final dimensions
- Order parts and print the arms and base
- Wire the servo bus, LED matrix and audio, then bring up the Pi
- Build the perception and voice pipeline (face tracking, object memory, Q&A)
