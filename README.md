# mujoco_kuka

*Simulation of KUKA arm with MuJoCo into Python*
---

This repository contains files for simulating the robotic arm KUKA LWR4+ with MuJoCo into Python.

* *MuJoCo* is a physics engine and doesn't need _installation_ but _configuration_. To see it working, run the `./simulate` program from bin.

* *mujoco-py* is the wrapper of MuJoCo for python and _needs_ installation.


## Folders
--------------

* *mujoco-py* This is the original pkg, the wrapper for bringing mujoco into python terms
* *mujoco_lwr* Sample provided by team15, to test mujoco and linear weighted regression
* *ROBOT_model_and_meshes*  files for bringing kuka arm into simulation
* *robot_documentation* docs for Kuka Arm


## Configuration & Installation
--------------
## [MuJoCo](http://www.mujoco.org/book/programming.html#inStart)
MuCoCo Pro is a dynamic library with C/C++ API and therefore doesn't need installation, but configuration. The placement of the library can be anywhere! but for the requirements of the wrapper mujoco_py, it needs to be placed in `~./mujoco`.

The library is a commercial product and therefore requires a license. To get the library and use it:
* Download the [libraries](https://www.roboti.us/index.html) and extract it to `~/.mujoco/mjpro150`
* Get the [license](https://www.roboti.us/license.html) by registering (it will require to run a executable to get our computer's id)
* Copy the license to `~/.mujoco/mjpro150/bin/mjkey.txt`

## To see MuJoCo working
* Have the LD_LIBRARY_PATH (dynamic linker) point to the
.so files everytime by (or copy them to a directory that is already in the linker path)
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.mujoco/mjpro150/bin
sudo ldconfig
```
* execute `./simulate` from bin

Export of the path has to be done in every bash where compilation is required. To make a permanent link,
* either write the export line in the startup files `~/.bashrc)` , or
* if the library is not conflicting with any other library then put into one of standard library path (e.g. /lib,/usr/lib)

--- 


## mujoco-py: the wrapper for python
### regular instructions, not for my Asus Ubuntu
In my installation (Asus Ubuntu 16.04), when tried the regular instructions from the [webpage](https://github.com/openai/mujoco-py):

```sh
pip3 install -U 'mujoco-py<1.50.2,>=1.50.1'
```

it gave me several problems. Particularly one related to the licensing:

```
    [    [V: not found
    sh: 2: Syntax error: "(" unexpected
    [    [V: not found
    sh: 2: Syntax error: "(" unexpected
    ERROR: Invalid activation key

    Press Enter to exit ...   

 failed with error code 1 in /tmp/pip-build-q_ljnipo/mujoco-py/
```

### Tailored for Asus Ubuntu
 Therefore, from this [recommendation](https://github.com/openai/mujoco-py/issues/66), my customized instructions are:

```sh
cd /home/adrianna/Progr/Thesis_Aalto/mujoco_kuka/  #repo where I store everything for this proj
git clone https://github.com/openai/mujoco-py.git
cd mujoco-py
sudo apt-get update
sudo apt-get install libgl1-mesa-dev libgl1-mesa-glx libosmesa6-dev python3-pip python3-numpy python3-scipy
pip3 install -r requirements.txt
# instead of sudo python3 setup.py install  , do

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.mujoco/mjpro150/bin
sudo python3 setup.py install
```

The last line is to add the link to the library, it is to solve the *error*
```
Failed to load GLFW3 shared library.
```

### mujoco-py accesible

copy mujoco-py to ~/.mujoco so the library is accesible from any other location and install it from there


```sh
pip3 install -r requirements.txt
sudo python3 setup.py install
```


### Test mujoco-py

If the variable is not loaded automatically (with the instruction in .bashrc and then `sudo ldconfig` to update the system with the libs), run

```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.mujoco/mjpro150/bin
```

(Although the location `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/Progr/Thesis_Aalto/mujoco_kuka/mjpro150/bin` is technically correct for MuJoCo,  mujoco_py doesn't like this location and therefore requires the mjpro to be stored in `~/.mujoco`. That's why the folder is stored there, else, we can place mjpro where we want -as in this repo-) before anything else. And afterwards,

In `mujoco_kuka/mujoco-py`

```
$ python3
import mujoco_py
from os.path import dirname
model = mujoco_py.load_model_from_path(dirname(dirname(mujoco_py.__file__))  +"/xmls/claw.xml")
sim = mujoco_py.MjSim(model)

print(sim.data.qpos)
# [ 0.  0.  0.  0.  0.  0.  0.  0.  0.  0.  0.  0.]

sim.step()
print(sim.data.qpos)
# [  2.09217903e-06  -1.82329050e-12  -1.16711384e-07  -4.69613872e-11
#   -1.43931860e-05   4.73350204e-10  -3.23749942e-05  -1.19854057e-13
#   -2.39251380e-08  -4.46750545e-07   1.78771599e-09  -1.04232280e-08]
```




### If errors with mujoco-py  ¯\\_(ツ)_/¯

When `pip3 install -U 'mujoco-py<1.50.2,>=1.50.1'` is not nice, [check](https://github.com/openai/mujoco-py/issues/66) and/or..

* *Problem*:
```sh
...mujoco_py/gl/eglshim.c:4:21: fatal error: GL/glew.h: No such file or directory. Compilation terminated.
error: command 'x86_64-linux-gnu-gcc' failed with exit status 1
```
**Solution**: ` sudo apt-get install libglew-dev`

* *Problem*:
```
error: [Errno 2] No such file or directory: 'patchelf'
Failed building wheel for mujoco-py
```

**[Solution](https://github.com/openai/mujoco-py/issues/47)**:
```
sudo apt-get update -q
	DEBIAN_FRONTEND=noninteractive sudo apt-get install -y \
    curl \
    libgl1-mesa-dev \
    libgl1-mesa-glx \
    libosmesa6-dev \
    python3-pip \
    python3-numpy \
    python3-scipy \
    xpra \

sudo apt-get clean
sudo rm -rf /var/lib/apt/lists/*

# install patch
sudo curl -o /usr/local/bin/patchelf https://s3-us-west-2.amazonaws.com/openai-sci-artifacts/manual-builds/patchelf_0.9_amd64.elf
sudo chmod +x /usr/local/bin/patchelf
```


* *Problem*:
```sh
File "/usr/local/lib/python3.5/dist-packages/glfw/__init__.py", line 200, in <module>
  raise ImportError("Failed to load GLFW3 shared library.")
ImportError: Failed to load GLFW3 shared library.
Failed building wheel for mujoco-py
Running setup.py clean for mujoco-py
...
ImportError: Failed to load GLFW3 shared library.
```
**[Solution](https://github.com/openai/mujoco-py#missing-glfw)**:

Which happens when the *glfw* python package fails to find a GLFW dynamic library.

MuJoCo ships with its own copy of this library, which can be used during installation.

Add the path to the mujoco bin directory to your dynamic loader:

```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.mujoco/mjpro150/bin
```

**Another [solution](https://github.com/openai/mujoco-py/issues/103#issuecomment-326717426)** that I didnt try is to modify the activate function


### Next, try with OpenAI gym

From the [thread](https://github.com/openai/mujoco-py/issues/66), seems it has issues with OpenAI

```
Try cloning the repo instead:
git clone https://github.com/openai/mujoco-py.git
go to the directory mujoco-py
python3 setup.py install
as suggested by:
#47
should then be able to
python3
import mujoco_py
which will compile the first time.
But remember, this version doesn't work with gym, so gym.make('environment') won't work!
But the examples should run (eg. examples/tosser.py)
Use an older version if you want to use with gym.
```





## Kuka LWR 4+ Simulation Enviroments
--------------
Provided intially by Murtaza and his colleagues (January 2018)
- **xmls** : models used by the engine
- **src** : source files, controllers, api and such
- **lwrsim.py** : test simulation, changes pose to a new random pose every 2 seconds



## From the Mujoco Pro Version 150, doc/README
--------------
Here we provide brief notes to get you started:

* Once you have mjkey.txt in the bin directory, run:
  ```
  simulate ../model/humanoid.xml  (or ./simulate on Linux and OSX)
  ```
to see MuJoCo Pro in action.

* On Linux, you can use LD_LIBRARY_PATH to point the dynamic linker to the .so files, or copy them to a directory that is already in the linker path.
	* On OSX, the MuJoCo Pro dynamic library is compiled with @executable_path/ to avoid the need for installation in a predefined directory.

* In general, the directory structure we have provided is merely a suggestion; feel free to re-organize it if needed. MuJoCo Pro does not have an installer and does not write any files outside the executable directory.

* The makefile in the sample directory generates binaries in the bin directory. These binaries are pre-compiled and included in the software distribution.

* While the software distribution contains only one model (humanoid.xml), additional models are available at http://www.mujoco.org/forum under Resources.





**Remember to always download the project as SSH to authenticate with the SSH key for every commit**
