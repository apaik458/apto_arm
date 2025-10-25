# API

This directory contains a Python API for controlling the robot arm, using the STServo SDK from Waveshare. Based on the LEAP Hand API.

### Usage

(Optional) Create a conda virtual environment

```
conda create -n apto_env python=3.12
conda activate apto_env
```

Install required Python packages

```
pip install -r requirements.txt
```

The main.py file shows an example usage of the API
```
python main.py
```

Note: If getting "Permission denied: '/dev/ttyACM0'" error on Linux, change port permission with ```sudo chmod a+rw /dev/ttyACM0``` command