%% Path to load the image frames and detections

% Image_address=[pwd,'\Data\PETS\PETS09\Images\',S_Name,filesep,L_Name];
Image_address=fullfile(pwd,'Data','PETS','PETS09','Images',S_Name,filesep,L_Name);
% Address path to the image frames 
Detection_address=fullfile(pwd,'Data','PETS','PETS09','Detections_and_GT',['PETS09-',S_Name,L_Name]);

% Detection_address=[pwd,'\Data\PETS\PETS09\Detections_and_GT\PETS09-',S_Name,L_Name]; % Address path to the detection and ground truth file

% Image info
file = dir(Image_address);
num_file = numel(file);

info = imfinfo([Image_address,filesep,file(3).name]);
u_image=info(1).Height;
v_image=info(1).Width;
cl_image=class(imread([Image_address,filesep,file(3).name]));
