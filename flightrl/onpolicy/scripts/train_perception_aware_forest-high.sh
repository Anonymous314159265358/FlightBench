#! /usr/bin/bash

##### use this for training

python ./train_perception_aware.py \
    --n_rollout_threads 128 \
    --n_training_threads 8 \
    --num_mini_batch 1 \
    --algorithm_name mappo \
    --episode_length 850 \
    --stage_two_episode_length 240 \
    --env_name LearningPA \
    --experiment_name forest-high-pa \
    --use_recurrent_policy \
    --mlp_hidden_size 256 \
    --layer_N 1 \
    --log_interval 2 \
    --save_interval 2 \
    --num_env_steps 75000000 \
    --max_grad_norm 8.0 \
    --ppo_epoch 10 \
    --seed 3 \
    --device cuda:0 \
    --scene_path scene/forest-high \
    --vec_env_config flightlib/configs/vec_env.yaml \
    --env_config flightlib/configs/quadrotor_env.yaml \
    --dyn_config flightlib/configs/dyn_param.yaml \
    --gate_radius 0.4 \
    --user_name <user_name> \
    --wandb_name <wandb_name> \
    --gate_num 1 \
    --gate0_x 14.0 \
    --gate0_y -14.0 \
    --gate0_z 2.0 \
    --gate0_angle -45.0 \