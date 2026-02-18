# PyBullet

[PyBullet Quickstart Guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit?tab=t.0#heading=h.btdfuxtf2f72)

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia 
python demo.py
```


ERROR: Run without nvidia support

```python
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)

cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
```

- [pybullet example](https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/examples)
- [good examples github](https://github.com/yijiangh/pybullet_planning_tutorials)
- [Gym-Medium-Post](https://github.com/GerardMaggiolino/Gym-Medium-Post)
- [Cartpole: The “Hello World” of Reinforcement Learning](https://www.codeproject.com/articles/Cartpole-The-Hello-World-of-Reinforcement-Learning#comments-section)
- [OpenAI Gym Environments with PyBullet (Part 1)](https://www.etedal.net/2020/04/pybullet-panda.html)


```
sudo cp pybullet.pyi /usr/local/lib/python3.12/dist-packages/
```

```
cp pybullet.pyi /home/user/projects/pybullet_tutorial/venv/lib/python3.12/site-packages/
```


| file | Description |
| ---- | ----------  |
| basic.py | simple simulation loop with safe exit |
