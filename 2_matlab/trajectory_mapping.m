clear
close all

% Import the sequence 00-10
seq = input('Please input the index of sequence that you want to visualize (0-10):  ');

if isempty(seq)
    seq = 0;
end

pose_path = strcat(sprintf('/media/robin/Harddisk_thesis/Odometry_Benchmark/dataset/poses/%02d.txt',seq));
time_path = strcat(sprintf('/media/robin/Harddisk_thesis/Odometry_Benchmark/dataset/sequences/%02d/times.txt',seq));
tra_mat = importdata(pose_path);
times = importdata(time_path);
num_frame = size(tra_mat,1);
frequence = 1/mean(diff(times));

% Initilize the position matrix for later calculation
fprintf('Please input the 3D initial position with the form [x_0, y_0, z_0].\n');
pos_init = input('The initial position will be set to [0 0 0] if there is no input.\npos_init = ');

if isempty(pos_init)
    pos_init = [0 0 0];
end

% Calculate the positions
mat_pos = zeros(12,3);
mat_pos(1:3,1) = pos_init';
mat_pos(5:7,2) = pos_init';
mat_pos(9:11,3) = pos_init';
index = [4 20 36];
mat_pos(index) = 1;

pos = tra_mat * mat_pos;

% Plot the odometry map
figure();
plot3(pos(:,1),pos(:,2),pos(:,3));
title(sprintf('Ground truth of odometry for sequence %02d', seq));
grid on
view(0,0);
% Set the margin for plot
x_margin = (max(pos(:,1)) - min(pos(:,1)))/10;
x_min = min(pos(:,1)) - x_margin;
x_max = max(pos(:,1)) + x_margin;
x_cen = (x_min + x_max) / 2;
y_margin = (max(pos(:,2)) - min(pos(:,2)))/10;
y_min = min(pos(:,2)) - y_margin;
y_max = max(pos(:,2)) + y_margin;
y_cen = (y_min + y_max) / 2;
z_margin = (max(pos(:,3)) - min(pos(:,3)))/10;
z_min = min(pos(:,3)) - z_margin;
z_max = max(pos(:,3)) + z_margin;
axis([x_min x_max y_min y_max z_min z_max]);

fprintf('Press any key to exit.\n');
pause
fprintf('The frequence of the sequence is %f Hz.\n', frequence);
fprintf('The central point of the sequence is [%f, %f].\n', x_cen, y_cen);
fprintf('The RPM of the sequence is %f.\n', 60 * frequence);
close

% The folder 'poses' contains the ground truth poses (trajectory) for the
% first 11 sequences. This information can be used for training/tuning your
% method. Each file xx.txt contains a N x 12 table, where N is the number of
% frames of this sequence. Row i represents the i'th pose of the left camera
% coordinate system (i.e., z pointing forwards) via a 3x4 transformation
% matrix. The matrices are stored in row aligned order (the first entries
% correspond to the first row), and take a point in the i'th coordinate
% system and project it into the first (=0th) coordinate system. Hence, the
% translational part (3x1 vector of column 4) corresponds to the pose of the
% left camera coordinate system in the i'th frame with respect to the first
% (=0th) frame. Your submission results must be provided using the same data
% format.
