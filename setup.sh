. venv/bin/activate
# Ensure access to the virtual environment packages
export PYTHONPATH="$PYTHONPATH:/home/der/projects/home/ros_host/venv/lib/python3.12/site-packages"
# Add project src to PYTHONPATH
export PYTHONPATH="/home/der/projects/home/ros_host/src:$PYTHONPATH"
