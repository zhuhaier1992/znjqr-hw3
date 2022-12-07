% according file is pd_plus_gravity_2022.ttt
clear
close all
clc

%% Robot Parameters
lbr = importrobot('iiwa14.urdf');
lbr.DataFormat = 'column';
lbr.Gravity = [0 0 -9.81];
forceLimit = 100;
jointNum = 7;
disp(lbr.BodyNames)
path_to_urdf = 'iiwa14.urdf';
kuka = parse_urdf(path_to_urdf);

log_q=[]; log_qdot=[]; log_tau=[];log_t = [];log_ee_pos = [];
log_r=[];log_rd=[];

%% pd controller parameters
% ================= your code here==================%
Ks = diag([1,0.1,1,0.05,1,0.05,0.1]);
alpha=diag([3,3,3]);


%% Connect to the Vrep
% load api library
addpath('./libs');
% using the prototype file (remoteApiProto.m)
vrep=remApi('remoteApi');
% close all the potential link
vrep.simxFinish(-1);   
% wait for connecting vrep, detect every 0.2s
while true
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    if clientID>-1 
        break;
    else
        pause(0.2);
        disp('please run the simulation on vrep...')
    end
end
disp('Connection success!')
% set the simulation time step
tstep = 0.005;  % 5ms per simulation step
vrep.simxSetFloatingParameter(clientID,vrep.sim_floatparam_simulation_time_step,tstep,vrep.simx_opmode_oneshot);
% open the synchronous mode to control the objects in vrep
vrep.simxSynchronous(clientID,true);

%% Simulation Initialization
vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);

% Now try to retrieve data in a blocking fashion (i.e. a service call):
[res,objs]=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking);
if (res==vrep.simx_return_ok)
    fprintf('Number of objects in the scene: %d\n',length(objs));
else
    fprintf('Remote API function call returned with error code: %d\n',res);
end

% get the joint handles
[res,jointHandles]=vrep.simxGetObjects(clientID,vrep.sim_object_joint_type,vrep.simx_opmode_blocking);
if (res==vrep.simx_return_ok)
    fprintf('get %d joint handles\n',length(jointHandles));
else
    fprintf('Remote API function call returned with error code: %d\n',res);
end

% get the force sensor handles, view iiwa_link_ee_visual as the ee
[res,forceSensorHandles] = vrep.simxGetObjects(clientID,vrep.sim_object_forcesensor_type,vrep.simx_opmode_blocking);
if res==0
    fprintf('get %d force sensor handles\n',length(forceSensorHandles));
else
    fprintf('can not get the handle of ee %d!!!\n',i);
end

eeHandle = forceSensorHandles(2);
disp(eeHandle)

vrep.simxSynchronousTrigger(clientID);
disp('Handles available!')

% first call to read the joints' configuration, the mode has to be
% simx_opmode_streaming, afterwards simx_opmode_buffer
jointConfig=zeros(jointNum,1); jointVeloc = zeros(jointNum,1); jointTorque = zeros(jointNum,1); 
for i=1:jointNum
    [~,jointConfig(i)]=vrep.simxGetJointPosition(clientID,jointHandles(i),vrep.simx_opmode_streaming);
    [~,jointVeloc(i)]=vrep.simxGetObjectFloatParameter(clientID,jointHandles(i),2012,vrep.simx_opmode_streaming);% joint velocity 
    [~,jointTorque(i)]=vrep.simxGetJointForce(clientID,jointHandles(i),vrep.simx_opmode_streaming);
end

% end effector position
[~,eePos]=vrep.simxGetObjectPosition(clientID,eeHandle,-1,vrep.simx_opmode_streaming);
r0 = eePos';
disp(r0)


% initialize last
if ~exist('q_last','var') 
    q_last = jointConfig; 
end
if ~exist('qdot_last','var') 
    qdot_last = jointVeloc; 
end
if ~exist('rd_last','var') 
    rd_last = r0; 
end
if ~exist('qrdot_last','var') 
    qrdot_last = 0; % t=0时，r=rd,rddot=0
end

% get simulation time
currCmdTime=vrep.simxGetLastCmdTime(clientID);
lastCmdTime=currCmdTime;
vrep.simxSynchronousTrigger(clientID);         % every time we call this function, verp is triggered

%% Simulation Start
disp('being in loop!');
t = vrep.simxGetLastCmdTime(clientID)/1000;
tInit = t;
while (vrep.simxGetConnectionId(clientID) ~= -1)  % vrep connection is still active
    
    % 0. time update
    currCmdTime=vrep.simxGetLastCmdTime(clientID);
    dt=(currCmdTime-lastCmdTime)/1000;              % simulation step, unit: s 
    
    % 1. read the joints' configuration (position and velocity)
    for i=1:jointNum
        [~,jang]=vrep.simxGetJointPosition(clientID,jointHandles(i),vrep.simx_opmode_buffer);
        [~,jvel] = vrep.simxGetObjectFloatParameter(clientID,jointHandles(i),2012,vrep.simx_opmode_buffer);
        jointConfig(i)=jang;
        jointVeloc(i)=jvel;
    end
    
    q=jointConfig;
    qdot=jointVeloc;    
    qdotdot = (qdot-qdot_last)./dt;% column vector

    [~,eePos]=vrep.simxGetObjectPosition(clientID,eeHandle,-1,vrep.simx_opmode_streaming);
    r = eePos';
    rd=[r0(1);r0(2)+0.1*sin(2*pi*(currCmdTime/1000-tInit));r0(3)];
    disp(rd)
    rddot=(rd-rd_last)./dt;
    disp(r-rd)
    disp(rddot)
    

    % 3. calculate tau
    % ================= your code here==================%
    M=massMatrix(lbr,q);
    g=gravityTorque(lbr,q);
    %C = C_mtrx_fcn(q, q_d, kuka.pi(:))*q_d;
    C=velocityProduct(lbr,q,qdot);

    jacobian=geometricJacobian(lbr,jointConfig,lbr.BodyNames{end});
    jacobian=jacobian(4:6,:);
    %disp(pinv(jacobian))
    qrdot=pinv(jacobian)*(rddot-alpha*(r-rd));
    qrdotdot=(qrdot-qrdot_last)./dt;
    %disp(C)

    s=qdot-qrdot;
    disp(qdot)
    disp(qrdot)
    tau_s = -Ks*s;
    tau_d = M*qrdotdot+C'*qrdot+g;
    tau = tau_d+tau_s;

    %%
    % 4. set the torque in vrep way
    for i=1:jointNum
        if tau(i)<-forceLimit
            setForce=-forceLimit;
        elseif tau(i)>forceLimit
            setForce=+forceLimit;
        else
            setForce=tau(i); 
        end
        vrep.simxSetJointTargetVelocity(clientID, jointHandles(i), sign(setForce)*1e10, vrep.simx_opmode_oneshot);% set a trememdous large velocity for the screwy operation of the vrep torque control implementaion
        tau(i)=setForce;
        if setForce<0
            setForce = -setForce;% decide the direction of the force
        end
        vrep.simxSetJointForce(clientID, jointHandles(i),abs(setForce), vrep.simx_opmode_oneshot);           
    end

    % data recording for plotting
    log_tau = [log_tau tau];
    log_q=[log_q q];  
    log_qdot=[log_qdot qdot]; 
    log_r=[log_r r];
    log_rd=[log_rd rd];

    % 4. update vrep(the server side) matlab is client

    lastCmdTime=currCmdTime;
    q_last=q;    
    qdot_last = qdot;
    rd_last=rd;
    qrdot_last=qrdot;

    vrep.simxSynchronousTrigger(clientID);
    vrep.simxGetPingTime(clientID);
    t=t+dt; % updata simulation time
    log_t = [log_t t];
    disptime = sprintf('the time is %f s', t);
    disp(disptime);
    figure(1)
    for j=1:3
        subplot(3,1,j);
        plot(log_t,log_r(j,:),log_t,log_rd(j,:));hold on;
        scatter(log_t(end),max(log_rd(j,:)));
        grid;
    end
end
vrep.simxFinish(-1);  % close the link
vrep.delete();        % destroy the 'vrep' class

%% plot
%================= your code here==================%
figure(1)
for j=1:3
    subplot(3,1,j);
    plot(log_t,log_r(j,:),log_t,log_rd(j,:),'b');hold on;
    scatter(log_t(end),max(log_rd(j,:)));
    grid;
end
