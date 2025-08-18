import numpy as np
from gymnasium import utils
from gymnasium.envs.mujoco import MujocoEnv
from gymnasium.spaces import Box
import os


DEFAULT_CAMERA_CONFIG = {"trackbodyid": 0}


class ReacherEnv(MujocoEnv, utils.EzPickle):

    def __init__(
        self,
        xml_file: str = os.path.abspath("assets/scene_reacher.xml"),
        frame_skip: int = 2,
        default_camera_config: dict[str, float | int] = DEFAULT_CAMERA_CONFIG,
        reward_dist_weight: float = 1,
        reward_control_weight: float = 0.2,
        **kwargs,
    ):
        utils.EzPickle.__init__(
            self,
            xml_file,
            frame_skip,
            default_camera_config,
            reward_dist_weight,
            reward_control_weight,
            **kwargs,
        )

        self._reward_dist_weight = reward_dist_weight
        self._reward_control_weight = reward_control_weight

        observation_space = Box(low=-np.inf, high=np.inf, shape=(10,), dtype=np.float64)

        MujocoEnv.__init__(
            self,
            xml_file,
            frame_skip,
            observation_space=observation_space,
            default_camera_config=default_camera_config,
            **kwargs,
        )

        self.metadata = {
            "render_modes": [
                "human",
                "rgb_array",
                "depth_array",
                "rgbd_tuple",
            ],
            "render_fps": int(np.round(1.0 / self.dt)),
        }

        self.step_number = 0
        self.episode_len = 150

    def step(self, action):
        action[6] = 0.0   # keep gripper closed

        self.do_simulation(action, self.frame_skip)
        self.step_number += 1

        observation = self._get_obs()
        reward, reward_info = self._get_rew(action)
        info = reward_info
        truncated = self.step_number > self.episode_len

        if self.render_mode == "human":
            self.render()
        
        return observation, reward, False, truncated, info

    def _get_rew(self, action):
        vec = self.get_body_com("gripperframe") - self.get_body_com("target")
        reward_dist = -np.linalg.norm(vec) * self._reward_dist_weight
        reward_ctrl = -np.square(action).sum() * self._reward_control_weight

        reward = reward_dist + reward_ctrl
        reward = reward_dist

        reward_info = {
            "reward_dist": reward_dist,
            "reward_ctrl": reward_ctrl,
        }

        return reward, reward_info

    def reset_model(self):
        self.step_number = 0

        qpos = self.init_qpos + self.np_random.uniform(
            size=self.model.nq, low=-0.01, high=0.01
        )
        # Set random new goal position
        while True:
            self.goal = self.np_random.uniform(low=-0.15, high=0.15, size=3)

            if np.linalg.norm(self.goal) < 0.15:
                break
        qpos[-3:] = self.goal

        qvel = self.init_qvel + self.np_random.uniform(
            size=self.model.nv, low=-0.01, high=0.01
        )
        qvel[-3:] = 0

        self.set_state(qpos, qvel)

        return self._get_obs()

    def _get_obs(self):
        observation = np.concatenate((
            np.array(self.data.joint("1").qpos),
            np.array(self.data.joint("2").qpos),
            np.array(self.data.joint("3").qpos),
            np.array(self.data.joint("4").qpos),
            np.array(self.data.joint("5").qpos),
            np.array(self.data.joint("6").qpos),
            np.array(self.data.joint("gripper").qpos),
            np.array([self.get_body_com("target")[0]]), # target_x
            np.array([self.get_body_com("target")[1]]), # target_y
            np.array([self.get_body_com("target")[2]])  # target_z
        ), axis=0)

        return observation