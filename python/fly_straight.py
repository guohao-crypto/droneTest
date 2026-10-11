"""Baseline: fly in a straight line at the human, ignoring everything else.

It gets there, but only by bumping into things on the way. Use it to check
that the Godot <-> Python link works, and as the result to beat.

    python3 fly_straight.py
"""
import math

from drone_sim import DroneSim

SPEED = 6.0          # m/s
STOP_DISTANCE = 2.5  # m from the human's centre
TIME_LIMIT = 40.0    # s


def main():
    with DroneSim() as sim:
        goal = sim.world["human"]["center"]
        sim.reset()
        start_time = sim.read_state()["time"]
        touched = set()

        while True:
            state = sim.read_state()
            elapsed = state["time"] - start_time
            touched.update(state["contacts"])

            if state["human_distance"] <= STOP_DISTANCE:
                print("Reached the human in %.1f s." % elapsed)
                break
            if elapsed > TIME_LIMIT:
                print("Gave up after %.0f s, still %.1f m away." % (elapsed, state["human_distance"]))
                break

            offset = [g - p for g, p in zip(goal, state["position"])]
            length = math.sqrt(sum(c * c for c in offset))
            sim.send_velocity(*[c / length * SPEED for c in offset])

        sim.hover()
        print("Touched on the way:", ", ".join(sorted(touched)) or "nothing")


if __name__ == "__main__":
    main()
