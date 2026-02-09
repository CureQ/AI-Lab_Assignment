# Fall Detection with Arduino Nano 33 BLE and Edge Impulse

This project aims to detect human falls using the **Arduino Nano 33 BLE Sense** and **Edge Impulse**. We used motion sensor data collected from the device, trained a machine learning model on Edge Impulse, and deployed it to the microcontroller for real-time fall detection.

## 📘 Project Guide

A step-by-step tutorial on how we built this project can be found in the guide file:  
➡️ **[TinyML_Arduino_Edge_Impulse_Guide.md](TinyML_Arduino_Edge_Impulse_Guide.md)**

## 🌐 Edge Impulse Project

You can access the Edge Impulse project we used via the following public link:  
🔗 [Edge Impulse Live Project](https://studio.edgeimpulse.com/public/657907/live)

## 📁 Dataset

We have also included the dataset used to train the model. It was exported from Edge Impulse and can be found in this repository:  
📦 **[Dataset.zip](Dataset.zip)**

## 📝 Code (Arduino Library)

The [`fall-detection-ardino_lib.zip`](fall-detection-ardino_lib.zip) file contains a library created through Edge Impulse. This library can be added to the Arduino IDE, and the example code inside can be run by following the steps in the tutorial.


## 📄 Project Report

The detailed report of the project is available in this repository. It explains the problem, methodology, data collection, model training, and results.

> **Project Report:**  
> This project investigates fall detection using an embedded machine learning approach. Accelerometer and gyroscope data were collected from the Arduino Nano 33 BLE Sense board under different motion scenarios. A classification model was trained using Edge Impulse’s platform, and then deployed to the device. The system aims to detect falls in real time and trigger appropriate alerts, demonstrating the feasibility of TinyML solutions on low-power hardware.

