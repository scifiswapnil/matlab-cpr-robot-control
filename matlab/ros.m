clc;
close all;
clear all;
% rosinit;

jointAngles = CartesianMover6(0.35,0.0,0.35,0.0,1.57,0.0);
chatpub = rospublisher('/joint_demand','std_msgs/Float32MultiArray');
msg = rosmessage(chatpub);
msg.Data = jointAngles.';
send(chatpub,msg);

function [configSoln] = CartesianMover6(X,Y,Z,A,B,C)
    mover6=importrobot('CPMOVER6.urdf');
    mover6.DataFormat = "column";
    ca = cos(A);
    sa = sin(A);
    cb = cos(B);
    sb = sin(B);
    cc = cos(C);
    sc = sin(C);
    M1 = [1.0, 0.0, 0.0,   X; 0.0, 1.0, 0.0,   Y; 0.0, 0.0, 1.0,   Z; 1.0, 0.0, 0.0, 1.0];
    M2 = [ca , -sa, 0.0, 0.0;  sa,  ca, 0.0, 0.0; 0.0, 0.0, 1.0, 0.0; 0.0, 0.0, 0.0, 1.0];
    M3 = [1.0, 0.0, 0.0, 0.0; 0.0,  cc, -sc, 0.0; 0.0,  sc,  cc, 0.0; 0.0, 0.0, 0.0, 1.0];
    M4 = [cb , 0.0,  sb, 0.0; 0.0, 1.0, 0.0, 0.0; -sb, 0.0,  cb, 0.0; 0.0, 0.0, 0.0, 1.0];
    tform = M1*M2*M3*M4;
    ik=inverseKinematics('RigidBodyTree', mover6);
    weights=[0.25 0.25 0.25 1 1 1];
    initialguess=mover6.homeConfiguration;
    [configSoln,~]=ik('link6',tform,weights,initialguess);
    show(mover6, configSoln);
end

