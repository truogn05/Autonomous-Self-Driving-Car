"""One clock controls all four approaches, including two all-red clearances."""
from controller import Robot
import json

PHASES = [(10, 'EW', 'green'), (2, 'EW', 'amber'), (1, None, None),
          (10, 'NS', 'green'), (2, 'NS', 'amber'), (1, None, None)]


def phase_at(t):
    t %= sum(p[0] for p in PHASES)
    for duration, group, color in PHASES:
        if t < duration:
            return group, color
        t -= duration
    return None, None


def run():
    robot = Robot()
    devices = [robot.getDeviceByIndex(i) for i in range(robot.getNumberOfDevices())]
    previous = None
    while robot.step(int(robot.getBasicTimeStep())) != -1:
        group, color = phase_at(robot.getTime())
        if (group, color) == previous:
            continue
        previous = group, color
        states = {g: color if g == group else 'red' for g in ('EW', 'NS')}
        for led in devices:
            g, _, c = led.getName().split('_')
            led.set(int(states[g] == c))
        robot.setCustomData(json.dumps({'time': robot.getTime(), **states}))


if __name__ == '__main__':
    run()
