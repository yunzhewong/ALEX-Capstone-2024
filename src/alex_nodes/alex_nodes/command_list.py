from enum import Enum
import math
from commands import CommandType


def stable_exo(state, t: float):
  state.types = [CommandType.Position.value for i in range(6)]
  state.values = [0.0 for i in range(6)]

class State(Enum):
    Collecting = 1,
    Paused = 2,
    Resetting = 3

def step_analysis(state, t: float):
  MAX_ANGLE = 3 * math.pi
  MAX_TIME = 10
  STARTING_ANGLE = -3 * math.pi
  RESET_TIME = 5
  PAUSE_TIME = 1
  START_CURRENT = 0.2
  INCREMENT = 0.02

  if state.state == State.Collecting:
      command = START_CURRENT + INCREMENT * state.collect_index
      if not state.state_initialised:
          state.collecting_start = t
          state.state_initialised = True
          state.log.open(f"step{format(round(command, 2), '.2f')}A.csv")
          print(command)

      state.log.write(t, command, state.velocities[1], state.positions[1])

      exceeded_max_angle = state.positions[1] >= MAX_ANGLE
      exceeded_max_time = (t - state.collecting_start) >= MAX_TIME

      should_stop = exceeded_max_angle or exceeded_max_time

      if should_stop:
          state.state = State.Paused
          state.post_pause_state = State.Resetting
          state.collect_index += 1
          state.log.close()
          state.state_initialised = False

      state.types = [CommandType.Position.value, CommandType.Current.value] 
      state.values = [0.0, command]
  elif state.state == State.Paused:
      if not state.state_initialised:
          state.pause_end_time = t + PAUSE_TIME
          state.state_initialised = True

      if t > state.pause_end_time:
          state.state = state.post_pause_state
          state.state_initialised = False

      state.types = [CommandType.Position.value, CommandType.Current.value] 
      state.values = [0.0, 0.0]
  elif state.state == State.Resetting:
      if not state.state_initialised:
          state.reset_start = t
          state.reset_end = t + RESET_TIME
          state.reset_angle = state.positions[1]
          state.state_initialised = True

      expected_pos = state.reset_angle - ((state.reset_angle - STARTING_ANGLE) / RESET_TIME) * (t - state.reset_start)

      if t > state.reset_end:
          state.post_pause_state = State.Collecting
          state.state = State.Paused
          state.state_initialised = False
      
      state.types = [CommandType.Position.value, CommandType.Position.value] 
      state.values = [0.0, float(expected_pos)]

def position_test(state, t: float):
    remainder = t % 10

    state.types = [CommandType.Position.value, CommandType.Position.value]
    if remainder < 5:
        state.values = [0.0, math.pi]
    else:
        state.values = [math.pi, 0.0]