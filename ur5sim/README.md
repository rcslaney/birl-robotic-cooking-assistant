# UR5 simulation in pybullet, with URSim Synchronization

### Author: Luca scimeca ls769@cam.ac.uk (luca.scimeca@live.com)

![](/media/test_run.gif)


On windows systems

1. install virtual box
2. install ur-sim https://www.universal-robots.com/download/?option=71470#section16597
	and run the program like explained in the page
3. set-up a tcp connection as detailed here: https://www.universal-robots.com/articles/ur-articles/connecting-to-client-interfaces-within-ursim/					
   HOWEVER: make sure you set the network adapter of the VM to "host-only adapter" or you might encur in connection problems
4. install python and library dependencies, including:
	- pybullet
	- numpy
	- attrdict
	- matplotlib
	- NOTE: If pubullet won't install you might need to dowload VS build tools here https://visualstudio.microsoft.com/thank-you-downloading-visual-studio/?sku=BuildTools&rel=16 , then make sure to install/select "C++ build tools", "desktop .NET build tools", and "universal windows build tools". Afterwards, pybullet should be able to build and install.

5. When instantiating objects of the Simulation classes (i.e. UR5Sim or UR5SimSync defined in 'robot_utiles.py') make sure the IP address passes to create the object is correct
6. Examples can be found in 'ur5_debug_example.py' and 'ur5_control_example.py'. The first is a simple example demonstrating syncronization between URSim and PyBullet. The second is an example of how to control a UR5 robot in simulation in real time. if the "UR5" class is instantiated, rather than the UR5SimSync class, then a real robot can be controlled in the same way.  		