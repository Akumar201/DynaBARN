"""Learned (RL) navigation policy for DynaBARN II: shared observation, action and reward code.

Used by the ROS 2 policy node (scripts/learned_policy.py, numpy only) and by the learner
(tools/learned/learner.py, PyTorch), so both always agree on what the policy sees and does.
"""
