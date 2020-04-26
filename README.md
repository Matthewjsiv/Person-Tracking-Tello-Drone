# Person-Tracking-Tello-Drone
Python script for using a Tello drone to track and follow a person
###FlowChart

```flow
st=>start: Login
op=>operation: Login operation
cond=>condition: Successful Yes or No?
e=>end: To admin

st->op->cond
cond(yes)->e
cond(no)->op
```
