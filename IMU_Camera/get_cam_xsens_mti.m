% get_cam_xsens_mti
%
% Capture images from USB or firewire camera, store as bmp 
% Capture corresponding IMU data from Xsens MTi,
% store as text file with  ax ay az .... on each line, minimum 10 lines
% load_imu will read 1st 3 values on from the 1st 10 lines as the x, y,z acceleration in m.s^2
%
% jlobo Jun 2008 update for xsens mti sensor and R14 matlab with imagging toolbox grab
% jlobo Mar 2004

imaqreset;
%imaqhwinfo
%imaqhwinfo('winvideo',1)
vid=videoinput('winvideo', 1);
preview(vid);

base_name=input('File base name: ','s');

% xsens

% set up instance of MTObj in MATLAB
h=actxserver('MotionTracker.FilterComponent');

%St=input(['What COM-port is your MTi or MTx connected to? <1>'],'s');
COMport = 5 ; %str2num(St);
% assume default value for baud rate, but this can also be set
baudrate = 115200;

% call MT_SetCOMPort is required, unless using COM 1
h.MT_SetCOMPort(COMport,baudrate);

% request device information from MTi or MTx
[DeviceID] = h.MT_QueryMotionTrackerB

% request calibrated inertial and magnetic data along with orientation data
h.MT_SetCalibratedOutput(1);
% request orientation data in Euler-angles
h.MT_SetOutputMode(1);

% That's it!

	
i=1;
escolha=menu('Capture Images',sprintf('Capture #%03d',i),'Quit');
while(escolha~=2)
 imagem=getsnapshot(vid);
 


 %get IMU
 imu=[];
 for j=1:10

 % MTObj is ready to start processing the data stream from the MTi or MTx
h.MT_StartProcess; % start processing data

% wait short moment for object to read data from COM-port
pause(0.05);
 
% retrieve the data
[arg1,inertialData] = MT_GetCalibratedData(h,1); % get latest calibrated data from buffer
[arg1,eulerAngle] = MT_GetOrientationData(h,1); % get latest orientation data from buffer

% if data retrieved succesfully (arg1=1)
if arg1==1,
    status= double(arg1) % MTObj status (can be converted to double for easy use in Matlab)
    inertialData = double(inertialData) % data values (can be converted to double for easy use in Matlab)
    eulerAngle = double(eulerAngle) % data values (can be converted to double for easy use in Matlab)
else
  display('Error reading xsens');
end


 % stop processing before removing object
h.MT_StopProcess;

    gx = inertialData(4);
    gy = inertialData(5);
    gz = inertialData(6);
    ax = inertialData(1);
    ay = inertialData(2);
    az = inertialData(3);
    imu=[ imu [ax ay az gx gy gz]'];
 end   

 name=sprintf('%s_imu_%03d.txt',base_name,i);
 eval([ 'save ' name ' imu -ASCII; ']);

 imshow(imagem);
 img_name=sprintf('%s_%03d',base_name,i);
 imwrite(imagem,strcat(img_name,'.bmp'),'bmp')
 i=i+1;
escolha=menu('Capture Images',sprintf('Capture #%03d',i),'Quit');
end


% stop processing before removing object
h.MT_StopProcess;

% when finished with MTObj, release it from the MATLAB workspace
delete(h); % release MTObj COM-object
clear h;

closepreview(vid);
delete(vid);
imaqreset;

