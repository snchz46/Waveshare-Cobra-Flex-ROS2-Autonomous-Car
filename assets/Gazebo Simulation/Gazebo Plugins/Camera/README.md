# Camera Plugin Basics

Use this folder for RGB/depth camera simulation notes and snippets.

## Core Configuration Items

- Image width / height
- Field of view and clipping planes
- Update rate (Hz)
- Pixel format and optional depth stream
- Camera info and image topic names

## Validation Checks

- Image orientation matches robot frame.
- Effective FOV is plausible for placement.
- Topic frequency is stable under motion.
- Camera frame is connected in TF tree.
