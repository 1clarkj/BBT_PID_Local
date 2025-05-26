This is the repo responsible for the PID controller of the BBT.

The raspberry pi has PEP 668 imposed, thus all code must be run through a virtual machine. This should already be initialised on the pi, if not here are the steps: 
1. create the virtual environment: python3 -m venv venv
2. activate the virtual environment: source venv/bin/activate
3. install necessary packages if needed: pip install -r requirements.txt
4. before running pid_control.py, must initiate pigpiod: sudo pigpiod
5. then run the code : python3 pid_control.py
6. deactivate the virtual environment: deactivate
7. shut down the pi : sudo shutdown now

