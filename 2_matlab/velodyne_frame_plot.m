clear
close all

PI = 3.1415926;

% Choose a sequence
seq = input('Please input the index of sequence that you want to visualize (0-21):  ');

% Set seq 00 as the init seq if no input
if isempty(seq)
    seq = 0;
end

seq_path = strcat(sprintf('/media/robin/Harddisk_thesis/Odometry_Benchmark/dataset/sequences/%02d',seq));
bin_path = dir([seq_path '/velodyne/*.bin']);
num_binary = size(bin_path, 1);

% % Choose a frame index
vis_string = strcat(sprintf('This sequence includes %d frames.', num_binary));
disp(vis_string);
idx = input('Please input the frame index that you want to visualize:  ');

% Set the first frame as the init frame if no input
if isempty(idx)
    idx = 0;
end

disp(strcat(sprintf('Visualizing the %d-th frame in sequence %2d ...', idx, seq)));

% Read the points in the cloud
frame_path = strcat(seq_path, sprintf('/velodyne/%06d.bin', idx));
stream = fopen(frame_path, 'r');
tmp = fread(stream, 'float');
num_pts = size(tmp ,1) / 4;
Pts = reshape(tmp, [4, num_pts])';



% Compute the index of laser
if (0)
    rowIdn = zeros(num_pts,1);
    verticalAngle = zeros(num_pts,1);
    for i = 1:num_pts
        x = Pts(i,1);
        y = Pts(i,2);
        z = Pts(i,3);
        verticalAngle(i,1) = atan2(z, sqrt(x*x+y*y)) * 180 / PI;
        rowIdn(i,1) = floor((verticalAngle(i,1) + 24.8) / (26.8/63));
    end
%     plot(1:num_pts, rowIdn);
    count = zeros(64, 1);
    for i = 1:64
        for j = 1:num_pts
            if rowIdn(j,1) == i
                count(i) = count(i)+1;
            end
        end
    end
end

% Compute the column index
if (0)
    columnIdx = zeros(num_pts,1);
    for i = 1:num_pts
        x = Pts(i,1);
        y = Pts(i,2);
%         z = Pts(i,3);
        horizonAngle = atan2(x, y) * 180 / PI;
        columnIdx(i,1) = floor(-floor((horizonAngle - 90.0) / 0.1728) + 2083/2);
        if (columnIdx(i,1) >= 2081)
            columnIdx(i,1) = columnIdx(i,1)-2083;
        end
    end
%     plot(1:num_pts, rowIdn);

end

% Plot the each scan of the point cloud seprately
% The division of laser scan is roughly divided by point index (each 2000 points)
if (1)
    for i = 1:30
       begin_idx = 2083 * (i - 1) + 1;
       end_idx = 2083 * i;
       if end_idx > num_pts
           end_idx = num_pts;
       end
        X = Pts(begin_idx:end_idx, 1);
        Y = Pts(begin_idx:end_idx, 2);
        Z = Pts(begin_idx:end_idx, 3);
        I = Pts(begin_idx:end_idx, 4);

        figure
        scatter3(X, Y, Z, [], I, '.');
        hold on
        plot(0,0,'or')
        title(sprintf('Point the %d-th laser from %d to %d', i, begin_idx, end_idx));
        axis([min(Pts(:, 1)) max(Pts(:, 1)) min(Pts(:, 2)) max(Pts(:, 2)) min(Pts(:, 3)) max(Pts(:, 3))]);
%         view(0,0); % right view
        view(2); % top view
    end
end

% Plot the whole point cloud with itensity
if (0)
    begin_idx = 1;
    end_idx = num_pts;
    if end_idx > num_pts
       end_idx = num_pts;
    end
    X = Pts(begin_idx:end_idx, 1);
    Y = Pts(begin_idx:end_idx, 2);
    Z = Pts(begin_idx:end_idx, 3);
    I = Pts(begin_idx:end_idx, 4);

    figure
    scatter3(X, Y, Z, [], I, '.');
    hold on
    plot(0,0,'or');
    title(sprintf('Point from %d to %d', begin_idx, end_idx));
    axis([min(Pts(:, 1)) max(Pts(:, 1)) min(Pts(:, 2)) max(Pts(:, 2)) -10 5]);
    view(0,0); % right view
%     view(2); % top view
end

if(0)    
    Angles = zeros(num_pts, 2);
    for i = 1:num_pts
        radius = sqrt(Pts(i,1)^2 + Pts(i,2)^2);
        Angles(i,1) = atan2(Pts(i,3), radius) * 180 / PI;
        Angles(i,2) = atan2(Pts(i,1), Pts(i,2)) * 180 / PI;
    end

    figure;
    plot(1:num_pts, Angles(:,1));
    hold on
    plot(1:num_pts, Angles(:,2));
end


