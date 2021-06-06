# Slab Robot
Work in progress... upload picture

## Mechanical Design
[OnShape CAD Document](https://cad.onshape.com/documents/01b7bf47ac0296002ad3efc0/w/656ec7f127d0c64f6943908d/e/5454fd1cd4994d1435836e10)

## URDF Generation
[Onshape to Robot](https://onshape-to-robot.readthedocs.io/) is used to generate robot simulation files based on the concept assembly CAD.

```bash
sudo apt install openscad meshlab # install system dependencies
pip install onshape-to-robot # install python application

# generate and output to urdf directory based on urdf/config.json
onshape-to-robot urdf 
```