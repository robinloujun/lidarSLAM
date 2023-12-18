clear
close all

for seq = 0:10
    pose_path = strcat(sprintf('/home/robin/eva/visual-odometry/dataset/poses/%02d.txt',seq));
    target_path = strcat(sprintf('/home/robin/eva/visual-odometry/dataset/%02d.txt',seq));
    tra_mat = importdata(pose_path);
    
    result_mat = zeros(size(tra_mat));
    
    for i = 1:size(tra_mat,1)
        trans_mat = zeros(4,4);
        trans_mat(1,1:4) = tra_mat(i,1:4);
        trans_mat(2,1:4) = tra_mat(i,5:8);
        trans_mat(3,1:4) = tra_mat(i,9:12);
        trans_mat(4,4) = 1;
        
        delta = [0,0,1,0;-1,0,0,0;0,-1,0,0;0,0,0,1];
        result = delta * trans_mat;
        
        result_mat(i,1:4) = result(1,1:4);
        result_mat(i,5:8) = result(2,1:4);
        result_mat(i,9:12) = result(3,1:4);
    end
    
    dlmwrite(target_path,result_mat,'delimiter',' ','precision',9)
end
