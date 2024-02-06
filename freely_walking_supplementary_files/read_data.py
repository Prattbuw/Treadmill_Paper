import h5py
import os
import math

#matplotlib inline
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import pandas as pd
import seaborn as sea
import scipy.signal

from scipy import signal

def read_data(curr_file):
    # Open the HDF5 file using h5py.
    f = h5py.File(curr_file, "r")

  # Print a list of the keys available.
    print("Keys in the HDF5 file:", list(f.keys()))

  # Load all the datasets into a dictionary.
#     data = {k: v[:] for k, v in f.items()}
    data = {k: v[()] for k, v in f.items()}

  # Here we're just converting string arrays into regular Python strings.
    data["node_names"] = [s.decode() for s in data["node_names"].tolist()]
    data["track_names"] = [s.decode() for s in data["track_names"].tolist()]

  # And we just flip the order of the tracks axes for convenience.
    data["tracks"] = np.transpose(data["tracks"])

  # And finally convert the data type of the track occupancy array to boolean.
  # We'll see what this array is used for further down.
    data["track_occupancy"] = data["track_occupancy"].astype(bool)


# Describe the values in the data dictionary we just created.
    for key, value in data.items():
        if isinstance(value, np.ndarray):
            print(f"{key}: {value.dtype} array of shape {value.shape}")
    else:
        print(f"{key}: {value}")
        
    valid_frame_idxs = np.argwhere(data["track_occupancy"].any(axis=1)).flatten()
    
    tracks = []
    for frame_idx in valid_frame_idxs:
      # Get the tracking data for the current frame.
      frame_tracks = data["tracks"][frame_idx]

      # Loop over the animals in the current frame.
      for i in range(frame_tracks.shape[-1]):
        pts = frame_tracks[..., i]

        if np.isnan(pts).all():
          # Skip this animal if all of its points are missing (i.e., it wasn't
          # detected in the current frame).
          continue

        # Let's initialize our row with some metadata.
        detection = {"track": data["track_names"][i], "frame_idx": frame_idx}

        # Now let's fill in the coordinates for each body part.
        for node_name, (x, y) in zip(data["node_names"], pts):
          detection[f"{node_name}.x"] = x
          detection[f"{node_name}.y"] = y

        # Add the row to the list and move on to the next detection.
        tracks.append(detection)

    # Once we're done, we can convert this list of rows into a table using Pandas.
    tracks = pd.DataFrame(tracks)

    tracks.head()
    
    numflies=tracks.track.unique()
        
    return [data, valid_frame_idxs,tracks, numflies]