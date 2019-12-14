# Sourced from Olympe docs

import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged

drone = olympe.Drone("192.168.42.1")
drone.connection()
drone(
    TakeOff()
    >> FlyingStateChanged(state="hovering", _timeout=5)
).wait()
drone(
    moveBy(0, 0, 0, 0)
    >> FlyingStateChanged(state="hovering", _timeout=5)
).wait()
drone(Landing()).wait()
drone.disconnection()
