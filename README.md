# pyrvo

pyrvo is a Python binding for [ORCA (Optimal Reciprocal Collision Avoidance) library](https://github.com/snape/RVO2). It provides a Python interface with similar API as original ORCA and render with [ir-sim](https://github.com/hanruihua/ir-sim).

## Installation

### Build from source

```
git clone https://github.com/hanruihua/pyrvo.git
cd pyrvo
pip install -e .
```

### Install from PyPI

```
pip install pyrvo
```

## ORCA Example

```
python test/test_python/Circle_render.py
```

<div align="center">
<img src="https://github.com/user-attachments/assets/da948a48-3b2d-4aed-abab-497d992bb9ba" width="400"/>
</div>

```
python test/test_python/Blocks_render.py
```

<div align="center">
<img src="https://github.com/user-attachments/assets/035e0f54-7b9a-4b0a-9d83-a44e91ef9184" width="400"/>
</div>

```
python test/test_python/Roadmap_render.py
```

<div align="center">
<img src="https://github.com/user-attachments/assets/a3682144-58d6-4df5-97b4-d67291b1f2eb" width="400"/>
</div>



