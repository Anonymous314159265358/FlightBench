#! /usr/bin/bash
python ./train_min_time.py \
    --n_rollout_threads 2 \
    --n_training_threads 8 \
    --num_mini_batch 8 \
    --algorithm_name mappo \
    --episode_length 400 \
    --env_name LearningMinTime \
    --experiment_name test_train_pipeline \
    --use_recurrent_policy \
    --mlp_hidden_size 256 \
    --layer_N 1 \
    --log_interval 1 \
    --save_interval 5 \
    --num_env_steps 50000000 \
    --max_grad_norm 6.0 \
    --ppo_epoch 16 \
    --seed 1 \
    --device cuda:0 \
    --scene_path scene/office \
    --vec_env_config flightlib/configs/vec_env.yaml \
    --env_config flightlib/configs/quadrotor_env.yaml \
    --dyn_config flightlib/configs/dyn_param.yaml \
    --gate_radius 0.5 \
    --user_name <user_name> \
    --wandb_name <wandb_name> \