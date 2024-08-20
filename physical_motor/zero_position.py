import trajectory
from aiosv2.Exoskeleton import Exoskeleton
from aiosv2.ControlLoop import setup_teardown_motor_combination


if __name__ == "__main__":
    def func(exoskeleton: Exoskeleton, runningTime: float):
        trajectory.slow_move_to_pos(exoskeleton.rightAbductor, 0)
        trajectory.slow_move_to_pos(exoskeleton.leftAbductor, 0)
        trajectory.slow_move_to_pos(exoskeleton.rightExtensor, 0)
        trajectory.slow_move_to_pos(exoskeleton.leftExtensor, 0)
        trajectory.slow_move_to_pos(exoskeleton.leftKnee, -0.05)
        if runningTime < 5:
            trajectory.slow_move_to_pos(exoskeleton.rightKnee, 0.05)
        else:
            trajectory.slow_move_to_pos(exoskeleton.rightKnee, 1)


    setup_teardown_motor_combination(Exoskeleton(), func, 10)