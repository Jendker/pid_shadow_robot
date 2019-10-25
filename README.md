# pid_shadow_robot

Test of PID control for shadow robot like hand.  

Simplified hand model taken from mj_envs repository:
https://github.com/vikashplus/mj_envs

__To check each of the control types uncomment required model in 'pid_test.py' and run it.__

### Requested behaviours
- All the `ctrl` inputs are set to 0 - hand base should be stable, fingers fully stretched
- Only `ctrl` input of index finger is set to finger - index finger should be fully bent
 
### Observed behaviour
- Control works only with position control. The index finger is bent and hand is kept high.
- With PID and P control the hand is not controlled at all. Hand base is falling down, finger is not bent at all. Resulting pose is the same for PID and P control.
