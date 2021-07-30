from gym.envs.registration import register

register(
    id='f1tenth-v0',
    entry_point='gym_f1tenth.envs:F1tenthEnv',
)