import os
import sys
import h5py
import glob
import numpy as np
import tensorflow as tf
import tensorflow_datasets as tfds
import numpy as np
import threading

# Functions to pre-process teleop data for fine-tuning

def resize_images(images, target_size=(256, 256)):
    resized_images = []
    for img in images:
        resized_img = tf.image.resize(img, target_size)
        resized_images.append(resized_img)
    return np.array(resized_images)

def array_to_hdf5(array, hdf5_file, episode_name):
    with h5py.File(hdf5_file, 'a') as f:
        if episode_name in f:
            del f[episode_name]  # Remove existing dataset
        f.create_dataset(episode_name, data=array, compression='gzip')

def hdf5_to_tfds(hdf5_file, episode_name):
    with h5py.File(hdf5_file, 'r') as f:
        data = f[episode_name][:]
    return tf.data.Dataset.from_tensor_slices(data)

def parse_hdf5_episode(episode_path): # adapted from github.com/moojink/rlds_dataset_builder
    #  Load raw data 
    # (assumes filepaths look like: "/PATH/TO/ARX5/PREPROCESSED/DATASETS/<dataset_name>/train/episode_0.hdf5")
        with h5py.File(episode_path, "r") as F:
            actions = F["/act"][()]
            states_pos = F["/obs/q_pos"][()]
            states_pos_vel = F["/obs/q_vel"][()]
            top_images = F["/obs/images/top_view"][()]  # Primary camera (top-down view)
            front_images = F["/obs/images/front_view"][()]  # Front camera (45 degree view)
            left_wrist_images = F["/obs/images/wrist_view_left"][()]  # Left wrist camera
            right_wrist_images = F["/obs/images/wrist_view_right"][()]  # Right wrist camera

        # Get language instruction
        raw_file_string = episode_path.split('/')[-3]  # e.g. '/data/arx5_preprocessed/put_green_pepper_into_pot/train/episode_0.hdf5' -> put_green_pepper_into_pot
        command = " ".join(raw_file_string.split("_"))

        # Assemble episode: here we're assuming demos so we set reward to 1 at the end
        episode = []
        for i in range(actions.shape[0]):
            episode.append({
                'observation': {
                    'top_image': top_images[i],
                    'front_image': front_images[i],
                    'left_wrist_image': left_wrist_images[i],
                    'right_wrist_image': right_wrist_images[i],
                    'state_pos': np.asarray(states_pos[i], np.float32),
                    'state_vel': np.asarray(states_pos_vel[i], np.float32),
                },
                'action': np.asarray(actions[i], dtype=np.float32),
                'discount': 1.0,
                'reward': float(i == (actions.shape[0] - 1)),
                'is_first': i == 0,
                'is_last': i == (actions.shape[0] - 1),
                'is_terminal': i == (actions.shape[0] - 1),
                'language_instruction': command,
            })

        # Create output data sample
        sample = {
            'steps': episode,
            'episode_metadata': {
                'file_path': episode_path
            }
        }

        # If you want to skip an example for whatever reason, simply return None
        return episode_path, sample

