clc
close all
clear all
mover6=importrobot('CPMOVER6.urdf')
mover6.show()
tform=getTransform(mover6,mover6.homeConfiguration,'link6','base_link')
randConfig=mover6.randomConfiguration
tform=getTransform(mover6,randConfig,'link6','base_link')
show(mover6, randConfig)
ik=inverseKinematics('RigidBodyTree', mover6)
weights=[0.25 0.25 0.25 1 1 1];
initialguess=mover6.homeConfiguration;
[configSoln,solnInfo]=ik('link6',tform,weights,initialguess)

%% section
conf
chatpub = rospublisher('/joint_demand','std_msgs/Float32MultiArray');
msg = rosmessage(chatpub);
msg.Data = [0.0,0,0,0,-0.09,0];
send(chatpub,msg);