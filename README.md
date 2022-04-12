# Table of Context <!-- omit in toc -->
- [1. Introduction](#1-introduction)
- [2. Overview of Functionallities](#2-overview-of-functionallities)
  - [2.1. Limcheck](#21-limcheck)
    - [2.1.1. Executionables](#211-executionables)

# 1. Introduction
The colcon package for which the dolle-iot software is build uppon. 
ROS2 Foxy is at the moment the framework uppon the solution is made.
## Installation <!-- omit in toc -->
To install the package we use snap along with github. 
First we download the package.
```bash
git clone https://github.com/pbtorrild/dolle-iot.git
```
Then we install the provided snap.
```bash
cd dolle-iot && snap install dolle-iot_0.1_arm64.snap --devmode
```
# 2. Overview of Functionallities
Bellow is the implemented functionalities provided. To install the dolle-iot package use snap. 

## 2.1. Limcheck 
How do we make sure that each step on the ladder is glued in place? 
We use computer vision and big data to provide the opporator with realtime information on the oporation state as if someone was looking on every step.

### 2.1.1. Executionables

- dolle-iot.limcheck | *Boots camera and begin gluedetection*



