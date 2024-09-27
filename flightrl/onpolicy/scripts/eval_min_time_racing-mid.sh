#! /usr/bin/bash

##### use this for training

python ./eval_min_time.py \
    --n_rollout_threads 1 \
    --n_training_threads 1 \
    --num_mini_batch 32 \
    --algorithm_name mappo \
    --episode_length 210 \
    --env_name LearningMinTime \
    --experiment_name eval \
    --use_recurrent_policy \
    --mlp_hidden_size 256 \
    --layer_N 1 \
    --log_interval 2 \
    --save_interval 2 \
    --num_env_steps 40960000 \
    --ppo_epoch 10 \
    --seed 1 \
    --device cuda:0 \
    --scene_path scene/racing-mid \
    --vec_env_config flightlib/configs/vec_env.yaml \
    --env_config flightlib/configs/quadrotor_env.yaml \
    --dyn_config flightlib/configs/dyn_param.yaml \
    --gate_radius 0.5 \
    --user_name <user_name> \
    --wandb_name <wandb_name> \
    --use_wandb \
    --gate_num 4\
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
    --gate3_angle -90 \
    --model_dir /home/<user_name>/workspace/flightmare_ws/src/flightmare/flightrl/runs/LearningMinTime/mappo/racing-mid/wandb/run-20240410_195400-s3ibu40k/files
