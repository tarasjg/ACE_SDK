# ACE_SDK

Automatic Concussion Evaluation

This repository contains necesarry hardware libraries and system utilities for the mini EEG developed as part of the ACE project at the Georgia Institute of Technology.


## The System

The ACE mini EEG features an ADS1299 analog front-end, a STM32L452 microcontroller, an ADXL372 accelerometer, and a MX25R6435FM2IL0 QSPI flash memory chip.  In terms of connectivity, the device features a 20 position single row header for the front-end inputs, an STDC14 header for debugging, and a Micro-USB Type B port for external digital interfacing.  The device is powered by three CR2032 coin cells, which constitute the bipolar analog supply rails and the digital supply rail.  Estimates forecast that the device consumes roughly 11mA while in full acquisition mode, and consumes only 3uA while in deep sleep.

<p align="center">
  <img width="294" height="383" src="https://raw.githubusercontent.com/tarasjg/ACE_SDK/main/Wiki%20Photos/Render_Front.PNG">
</p>
  
  ## Hardware Libraries
  
 accel.c - provides direct access to the accelerometer (ADXL372)
 
 afe.c - provides direct access to the analog front-end (ADS1299)
 
 mem.c - provides direct access to the QSPI external flash memory (MX25R64)
 
 
 ## System Utilities
 
 dump.c - allows the user to dump the flash either via USART or USB, this is to analyze recorded data on an external computer
 
 stream.c - allows the user to stream live data from the analog front-end or the accelerometer via USART or USB
 
 modes.c - allows the user to set different acquisition modes of the system
 
 filters.c - provides useful DSP filters for preprocessing
 
 metrics.c - provides several anaylsis tools for quantifying recorded data
  
