#To train Cara using Reinforcement Learning (RL), you need a Python script that defines the Environment, Observations (what #Cara "sees"), and Rewards (what Cara "wants").

 
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.utils import configclass

@configclass
class CaraLocomotionCfg(ManagerBasedRLEnvCfg):
    def __post_init__(self):
        # 1. THE SCENE: Spawn Cara (your USD file) and the floor
        self.scene.robot = CARA_CFG.replace(prim_path="{ENV_REGEX}/Robot")
        self.scene.terrain = TerrainImporterCfg(prim_path="/World/ground")

        # 2. OBSERVATIONS: What the Orin Nano "perceives" in sim
        self.observations.policy.joint_pos = ObsTerm(func=get_joint_pos) # 20 angles
        self.observations.policy.joint_vel = ObsTerm(func=get_joint_vel) # 20 speeds
        self.observations.policy.base_lin_vel = ObsTerm(func=get_base_lin_vel) # For balance

        # 3. ACTIONS: The 20 target positions for your PCA9685 boards
        self.actions.joint_pos = JointPositionActionCfg(asset_name="robot", joint_names=[".*"])

        # 4. REWARDS: The "Brain" of the operation
        self.rewards.track_lin_vel = RewTerm(func=track_lin_vel_exp, weight=1.0) # Walk forward!
        self.rewards.homeostasis = RewTerm(func=thermal_penalty, weight=-0.5) # Don't overheat!
        
        
        
