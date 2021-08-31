# TODO:
- get more props, try 8x4.7 or 8x4, get thrust data from APC's website and choose correct props
- add automated tests
- remove wdog reboot flag
- add logic to disable main motors if loop is taking too long
- add altitude feedback control
- add python control simulation for noise analysis
- add some counter so 10 cycles pass before wait flag clear after boot
- logs show a lot of noise. Change d_filter response? say current attitude is average of last 5?
    - noise source could be derivative of target angle
- reduce max error in rate mode?
- change rate mode so that the target angle is only is constrained in the direction of change
    - this should allow the max error to be lowered without the current angle messing with the target
- should there be yaw stabilization in forward flight? perhaps just very small gains
- flight mode should not change if transition failed. constrain tstate based on current angle?
- create requirements.txt
- improve directory structure; put the scripts in folders

# NICE TO HAVE:
- Add const qualifiers to pointer in mpu6050 and ar610 libs, and flight controller
- Move static vars in fc functions to fc_state, add fc_init method
- make status led that flashes codes based on fc_flags

# TUNE PID:
## AFTER FLIGHT 1, 8/14
Check CG. Needs to be as far back as possible for vertical control.  
Try: tuning pid controller with pivot closer to cg. each axis individually  
roll logs show steady state error. try: lower trim and add I  
pitch logs show it is coupled with throttle  
yaw logs show oscillations. try: increase D  
## AFTER FLIGHT 2 8/20
Roll looks good enough. Logs and math show it could be slightly over damped  
Yaw looks good enough for now.  
Altitude control is important for preventing stalls by limiting decent velocity.  

# BUGS:
- flight controller does not stay in waiting state if transmitter is not connected at boot
    - boot with transmitter off. turn transmitter on with gear and aux1 centered. fc will not wait.
