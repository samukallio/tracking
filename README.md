# Orientation Tracking Using a 9DOF IMU

![UI](https://github.com/samukallio/tracking/blob/main/media/screenshot.png?raw=true)

This repository contains my B.Sc. thesis on sensor fusion titled "Orientation Tracking with Non-Linear Kalman Filters" that I wrote in spring 2018. It includes:
* Source code for the implemented algorithms,
* Firmware designed to run on a sensor board comprised of an ESP8266 microcontroller and an MPU-9025 MEMS IMU,
* A GUI program (see the screenshot) that communicates with the sensor board, computes the orientation estimate, and visualizes the result.

## Overview

A 9 degree of freedom (9DOF) inertial measurement unit (IMU) combines an accelerometer, a gyroscope, and a magnetometer into a single sensor that continuously measures linear acceleration, angular velocity, and the ambient magnetic field direction. The task of an orientation tracking algorithm is to use these measurements to compute the orientation of the sensor in a reference frame that is fixed relative to Earth.

The accelerometer is used to measure Earth's gravitational acceleration, yielding a reference "down" direction, and the magnetometer is used to measure the Earth's magnetic field direction, yielding a second reference direction. In principle, the orientation of the sensor can be deduced from these two measurements alone. However, each sensor is subject to unwanted disturbances: the accelerometer measures all acceleration, not just gravity; and the magnetometer measurement is disturbed by local magnetic effects that can easily overwhelm the Earth's magnetic field. In addition, each sensor is subject to some degree of measurement noise.

On the other hand, the gyroscope produces a relatively accurate and disturbance-free measurement of the angular velocity. If the initial orientation of the sensor is known, then its orientation can (again, in principle) be tracked by integrating the angular velocity measurements to produce the orientation angles. However, the initial orientation is typically not known. Furthermore, any bias or noise in the gyroscope measurement is integrated, causing the resulting orientation estimate to drift over time. Even a perfectly calibrated gyroscope has bias due to the rotation of the Earth (15 degrees/hour on the equator).

The idea behind sensor fusion is to combine the strengths of both types of sensors to avoid the weaknesses of either. The gyroscope provides good short-term accuracy, and the accelerometer and the magnetometer provide long-term accuracy by providing a stable reference that can be used to cancel the gyroscopic drift. Many algorithms exist for sensor fusion in general, and for orientation tracking in particular. The algorithms studied in my thesis, are:

* The Multiplicative Extended Kalman Filter (MEKF), a quaternion-based esimator that uses an Extended Kalman Filter (EKF) to track the error between a nominal quaternion (current orientation estimate) and the actual orientation.
* The Vector Kalman Filter (VKF), an original result that estimates the gravity and magnetic field direction using separate (ordinary) Kalman filters and uses these filtered direction vectors to compute the orientation estimate.
