# carla moduration for contesting
<!-- GETTING STARTED -->
## Getting Started

This is an example of how you may give instructions on setting up your project locally.
To get a local copy up and running follow these simple example steps.

### Prerequisites

#### Base version 9.5.0
Please download base version and carla engine on [CARLA 0.9.5 (development)](http://carla-assets-internal.s3.amazonaws.com/Releases/Windows/CARLA_0.9.5.zip)

#### This project only support python 3.7, if you need other version please following [Carla Github](https://github.com/carla-simulator/carla) instrution to build your own python or other language support API

FYI, if you have more than 1 python version installed. Please use 
```sh
py -3.7 filename.py
```
to run the script and use
```sh
py -3.7 -m pip install SomePackage
```
to install package for that python version.

Install package for working on this project.
#### install package according to requirement
```sh
py -3.7 -m pip install -r requirement.txt
```

#### Run manual control
Move root folder to example folder and run
```sh
py -3.7 -m manual_control.py
```
Check ip, port, and type of actor do you prefer.

#### Run free_camera.py to observer actor
```sh
py -3.7 -m free_camera.py
```

#### Exit
Ctrl + C
