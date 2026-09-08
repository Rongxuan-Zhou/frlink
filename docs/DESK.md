# Franka Desk: the robot's own console

Desk is the web interface built into the FR3 control cabinet. It is the only place where the robot
is unlocked, activated, put into hand-guiding mode, or recovered from an error. Nothing in this
repository replaces it; the client tools assume that somebody has done the Desk steps first. This page
explains what the Desk states mean, how they map to the lights on the arm, how to guide the arm by
hand, and how to clear the errors you will meet.

The lab's robot runs system image 5.7.2. Menu names below are from that version; if your Desk looks
different, the state machine is the same even when a button has moved.

## 1. Reaching Desk

Desk is served by the control cabinet at `https://172.16.0.2/desk/`, on the robot network that
terminates on the RT host's built-in NIC. That address is not routed to the client link, so:

- on the RT host: open the URL in a browser (the certificate is self-signed; accept it once);
- from another machine with a normal ssh account on the host:
  `ssh -L 8443:172.16.0.2:443 <user>@10.10.0.2` and then `https://localhost:8443/desk/`;
- a client that only has the forced-command key cannot reach Desk at all. Ask the host admin.

Log in with the robot's admin account. Desk hands **control** to one browser session at a time. A
second session opens read-only and shows a "request control" button; the first session then sees a
request it can grant or refuse. When Desk seems to ignore your clicks, check the top bar for the
read-only banner first. The same control token is what the FCI uses: `curl -k
https://172.16.0.2/admin/api/control-token` on the host prints who holds it and whether FCI is active.

## 2. States and lights

The arm's base ring lights show the robot state. The colours below are the ones we see on the lab's FR3; the
sidebar status text in Desk is the authoritative source when the two disagree:

| light | state | what it means | how you get out |
|---|---|---|---|
| yellow, pulsing | booting | cabinet is starting (about 1-2 min) | wait until it stops pulsing |
| yellow | locked or not activated | brakes closed after boot, or the user stop is pressed, or the enabling device was not pressed | unlock joints (Desk sidebar), release the user stop, press the enabling device |
| blue | ready ("Idle") | brakes open, activated, no error; the robot accepts FCI control, Desk apps and guiding | this is the state every session starts from |
| white | guiding | hand-guiding is active (you are holding the Pilot button) | release the button |
| green | executing | a Desk app or task is running | wait or stop the app |
| red | error | a reflex or safety violation stopped the arm; needs acknowledgement | see section 5 |

FCI control does not change the colour: with the servo running the light stays blue. If it turns red
while the servo runs, the servo's own recovery is already working on it (section 5.3).

Two physical inputs sit next to the lights in importance:

- The user stop (mushroom button on a cable): pressing it opens the brakes' safety circuit and drops
  the robot to yellow. Twist to release. Keep it in your hand for every motion you did not plan
  yourself.
- The enabling device (the "Enable" button on the external activation box): after a boot or a user
  stop, the robot is only "activated" once this has been pressed. Pressing it again deactivates. The
  Desk sidebar shows the current state next to "Robot".

## 3. Bringing the robot up (every power cycle)

1. Switch the cabinet on. Wait for the yellow pulsing to stop and for Desk to load.
2. In the Desk sidebar click **Unlock joints**. You hear the brakes click, one joint at a time. The
   sidebar status changes from "locked" to "Idle". If the button is grey, the user stop is pressed.
3. Press the **enabling device**. The light turns blue.
4. If you are going to use the client tools: top-right menu (three lines) -> **Activate FCI**. Desk shows
   a banner that FCI is active. While it is, Desk apps and the "move" widgets are disabled, and guiding
   still works only if the servo is not running.
5. Only now do the client commands work: `franka-remote status` shows the FCI link up, `franka-remote
   goto home` moves the arm.

To finish: stop the servo from the client (`franka-teleop stop`), then in Desk **Deactivate FCI** if
somebody else will use Desk apps, **Lock joints** if the arm will be left alone, and shut the cabinet
down through the top-right menu (**Shut down**), not the power switch. The switch comes after the LEDs
go dark.

## 4. Guiding the arm by hand ("dragging")

Hand guiding is the fastest way to put the arm somewhere: to set up a new start pose, to move it out of
the way, or to push a joint back inside its limit after a violation.

Prerequisites: light blue, user stop released, **no FCI client running** (a live servo owns the joints;
stop it with `franka-teleop stop` first; activating FCI by itself does not block guiding).

1. In the Desk top bar switch the operating mode from **Execution** to **Programming**. Programming mode
   runs the arm at reduced speed and enables the Pilot buttons for guiding.
2. Choose the guiding mode in the sidebar widget: **Free** (all six degrees of freedom), **Translation**
   only, **Rotation** only, or **User** (the axes you tick). Elbow motion can be locked or freed
   separately.
3. Hold the guiding button on the Pilot (the disc at the wrist; on the FR3 it is the pair of buttons
   under your fingers when you grip the disc). The light goes white and the arm becomes weightless.
   Move it. Release to lock it in place; the light goes back to blue.
4. Read the pose you reached: `franka-remote echo` prints `O_T_EE` and the joint angles as JSON (the
   servo must be stopped, which it is if you were guiding). Save it wherever your workflow keeps poses.
5. Switch back to **Execution** before starting the servo again.

Rules of thumb: guide with one hand on the Pilot and the other hand free; do not guide a joint through
its limit; the 30 N / 30 Nm collision thresholds are set by the servo and are not active during guiding.

## 5. Clearing errors

### 5.1 What Desk shows

An error appears as a red banner in the sidebar with the error name (for example `cartesian_reflex`,
`joint_position_limits_violation`, `communication_constraints_violation`) and an **Acknowledge** (or
**Recover**) button. Clicking it runs the same automatic recovery that libfranka exposes as
`automaticErrorRecovery`: brakes stay open, the controller state is reset, the light returns to blue.
The error text stays in the log (sidebar -> Logs) so you can see what tripped.

### 5.2 The common ones

| error / symptom | cause | fix |
|---|---|---|
| `cartesian_reflex`, `joint_reflex` | the arm hit something or was pushed harder than the collision threshold | remove the obstacle, Acknowledge in Desk (or let the servo recover, 5.3), then `franka-remote goto home` |
| `joint_position_limits_violation`, `cartesian_position_limits_violation` | a joint or the end effector was driven to a hard limit | Acknowledge; if the arm is still outside the limit, guide it back by hand (section 4) before starting anything |
| `joint_velocity_violation`, `cartesian_velocity_violation` | a target jumped too far in one step | Acknowledge; on the client, reduce the step size (`FRANKA_MAX_STEP`) or fix the sender that sent the jump |
| `communication_constraints_violation` | the FCI client missed too many 1 ms cycles | the servo host had a stall: check `franka_link.txt` (`missed_cycles_total`), the host's thermal state (`franka-remote status`), then restart the servo |
| light yellow, Desk says "not activated" | user stop pressed, or the enabling device was toggled | release the stop, press the enabling device |
| light yellow, "joints locked" after boot | brakes closed | Unlock joints |
| Desk read-only / clicks ignored | another browser holds control | Request control in your session, or close the other one |
| "FCI already in use" / servo exits at start with a connection error | another FCI client (an `echo`, a `goto`, a second servo) owns the channel | `franka-remote status` on the client, `franka-ctl status` on the host; stop it |
| Desk unreachable | cabinet off, cable to the host unplugged, host NIC down | on the host: `ping 172.16.0.2`, `ip -br addr show enp110s0`; if the cabinet is on and the link is up, power-cycle the cabinet |

### 5.3 What the servo does on its own

When the arm reflexes during FCI control, `robot.control()` throws inside the servo. The servo does not
exit: it freezes the target, calls `automaticErrorRecovery`, waits 500 ms, re-reads the pose, and
re-enters control after a further 2 s, up to 50 times. From the client you see `reflex_count` go up and
`recovering=1` for a moment in `/tmp/franka_link.txt`, and the arm resumes where it is. You only need
Desk when the recovery itself fails (the light stays red and `franka-remote status` shows the unit
inactive), or when the error is a limit violation that requires guiding the arm back.

### 5.4 When everything looks fine but nothing moves

- Light blue, `franka-remote status` shows the servo active, `franka_link.txt` has `cmd_pkts_last_s`
  at your rate, and `cmd_age_ms` is small: the servo is holding on purpose because of a force/torque
  freeze (`freeze=1`) or your targets equal the current pose. Wait 1.5 s or move the target.
- `cmd_pkts_last_s=0`: your sender is not reaching the host (wrong IP, lock, bridge dead).
- `cmd_drop_allow` or `cmd_drop_latch` rising: wrong source address, or another sender is latched.
- FCI active in Desk but the servo fails to start with a Desk-side message: somebody pressed the user
  stop or deactivated the robot after you activated FCI. Redo section 3, steps 2-4.

## 6. Things not to do

- Do not run a Desk app, a `goto`, an `echo`, or hand guiding while the servo is active. The FCI allows
  one client; the newcomer wins and the servo dies mid-motion.
- Do not close the browser tab as a way of stopping FCI. Use **Deactivate FCI** or leave it; the token
  survives the tab either way.
- Do not switch the cabinet off with the power switch while the light is on. Shut down from Desk first.
- Do not leave the arm unlocked and unattended. Lock joints or shut down.
