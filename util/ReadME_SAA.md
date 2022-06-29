# Sense and Avoid
# Changing from SITL to Real Vehicle.
The code written is not robust to change in run environment. Thus there needs to be some changes when we consider changing the environment. I have compiled a list of parameters to change when we change the environment.

To track the changes:

| SITL | Real System |
|------|-------------|
|self.lidar = driver.SensorDriver('SITL')|self.lidar = driver.SensorDriver('RPLidar')|
|estimation.Sensor(1,0.03098,40,1,-0.976)|estimation.Sensor(1,1*math.pi/180,12,0.01,0)|
|max_obs=35|max_obs=8|
|update_sitl_sensor in scheduler|t1,t2 threads start|
|SAAController mag check = 0|SAAController mag check non zero|


# The credentials for accessing the EP03 vehicle raspberry pi.
```sh
ssh pi@192.168.168.1
Password: GenAero2016
```

# Navigating to the code
```sh
cd Desktop\ga_companion_computer
python3 pymavlink_main.py --veh Test
```

Check the USB port before running. Sometimes it changes.