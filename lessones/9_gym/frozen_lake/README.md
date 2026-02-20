# Frozen lake
![alt text](images/frozen_lake.png)
[gymnasium](https://gymnasium.farama.org/environments/toy_text/frozen_lake/)


|   |   |
|---|---|
| Action Space  | Discrete(4)  |
| Observation Space  | Discrete(16)  |
| Rewards | - Reach goal: +1 - hole: 0 - frozen: 0 |


	
```python
gymnasium.make("FrozenLake-v1")
```


## Demo:
Run []() first with `is_training=true`  and `render=False` then to test  
run `is_training=false`  and `render=True`


---

> [!NOTE]
> ### Exercise
> Watch [ Q Learning From Scratch – Using Q Tables ](https://youtu.be/L-kQSpCJH4Q)
> explain line 44 and the variables in lines 22-25