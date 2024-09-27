#! /usr/bin/bash
##### use this for training
python ./train_min_time.py \
    --n_rollout_threads 128 \
    --n_training_threads 8 \
    --num_mini_batch 1 \
    --algorithm_name mappo \
    --episode_length 690 \
    --stage_two_episode_length 240 \
    --env_name LearningMinTime \
    --experiment_name racing-mid \
    --use_recurrent_policy \
    --mlp_hidden_size 256 \
    --layer_N 1 \
    --log_interval 2 \
    --save_interval 2 \
    --num_env_steps 160000000 \
    --ppo_epoch 10 \
    --max_grad_norm 8.0 \
    --seed 3 \
    --device cuda:0 \
    --scene_path scene/racing-mid \
    --vec_env_config flightlib/configs/vec_env.yaml \
    --env_config flightlib/configs/quadrotor_env.yaml \
    --dyn_config flightlib/configs/dyn_param.yaml \
    --gate_radius 0.4 \
    --user_name <user_name> \
    --wandb_name <wandb_name> \
    --gate_num 4 \
    --gate0_x 7.0 \
    --gate0_y -6.0 \
    --gate0_z 2.0 \
    --gate0_angle 90 \
    --gate1_x 7.0 \
    --gate1_y 1.71 \
    --gate1_z 2.0 \
    --gate1_angle 90 \
    --gate2_x -1.0 \
    --gate2_y 4.7 \
    --gate2_z 2.0 \
    --gate2_angle 180 \
    --gate3_x -6.0 \
    --gate3_y -6.0 \
    --gate3_z 1.5 \
    --gate3_angle -90