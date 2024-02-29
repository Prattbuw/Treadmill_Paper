# Code Repository for Pratt et al., 2024 Publication
Code for analyzing and visualizing walking kinematics of treadmill (linear and split-belt), freely, and tethered walking fruit flies.
Data can be downloaded from the Dryad repository: https://doi.org/10.5061/dryad.mpg4f4r73

To analyze the walking kinematics of the datasets that contain either 2D (freely walking) or 3D positions (treadmill and tethered) of the body and leg tip key points, please specify the directory that contains the data when running the following analysis scripts:
- "freely_walking_analysis_visualization.ipynb"
- "linear_treadmill_walking_analysis.ipynb"
- "splitbelt_walking_analysis.ipynb"
- "tethered_walking_analysis_visualization.ipynb"

Data visualization for freely walking and tethered flies is done within the analysis script, whereas visualizing the results for linear and split-belt treadmill walking is done using the following scripts, respectively:
- "linear_treadmill_visualization_walking_comparisons.ipynb"
- "splitbelt_walking_visualization.ipynb"

Note that the inputs to these scripts are the numpy files generated from running the respective treadmill analysis script.

The following Python code was used to control the linear and split-belt treadmill systems (i.e. belt speed(s) and high-speed videography):
- "linear_treadmill_belt_stim_videography.py"
- "splitbelt_treadmill_belt_stim_videography.py"

The .stl file for the chamber used in treadmill experiments is called, "Treadmill_Chamber.stl".
