# S.T.A.L.K.E.R.: Call of Pripyat with HL2 Movement

This modification for [OpenXRay](https://github.com/OpenXRay/xray-16) transfers the physics of movement directly from the HL2.

## Features
* Bunnyhopping
* Ground/air strafing
* Surfing
* Ramp boosting
* Ducking
* Climbing ladders
* Accelerated Back Hopping (ABH)

## Console commands
* sv_frametime - Time between physics simulations
* cl_forwardspeed - The speed of the actor when moving forward/back
* cl_sidespeed - The speed of the actor when moving sideways
* sv_walkspeed - Actor's normal walking speed
* sv_slowwalkspeed - Actor's slow walking speed
* sv_sprintspeed - Actor's sprint speed
* sv_maxcontrolspeed - Maximum speed that is not controlled by the actor
* sv_jumppower - Velocity that will be applied to the actor when they jump
* sv_gravity - Actor's gravity
* sv_maxvelocity - Maximum speed is allowed to attain per axis
* sv_nonjumpvel - Consider that the actor is not on the ground if the vertical velocity is greater than this
* sv_bounce - Bounce multiplier for when actor collide with other objects
* sv_stepsize - Sets the max height that the actor will automatically snap up to when moving into a wall
* sv_friction - Actor's ground friction
* sv_stopspeed - Minimum stopping speed when on ground
* sv_standable_normal - Determine how steep of a slope the actor can stand on
* sv_walkable_normal - Determine how steep of a slope the actor can walk on
* sv_air_max_wishspeed - Wish speed cap in air
* sv_accelerate - Linear acceleration amount
* sv_airaccelerate - Linear acceleration amount in air
* sv_ladder_dist - The distance an actor can be from a ladder and still attach to it
* sv_ladder_leavespeed - The speed that is given to an actor when jumping down a ladder
* sv_climbspeed - Actor's ladder climbing speed
* sv_duckspeed - How quickly actor ducks
* sv_unduckspeed - How quickly actor un-ducks
* sv_pushaway_force - How hard physics objects are pushed away from the actor
* sv_pushaway_max_force - Maximum amount of force applied to physics objects by actor
* cl_speedometer - Draw actor's speed
* sv_sticktoground - Prevents checking for sv_nonjumpvel
* sv_enable_bhop - Make you automatically bunny hop by holding down the space bar
* sv_enable_abh - Allows Accelerated Back Hopping (ABH)
