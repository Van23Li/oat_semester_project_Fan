% 写入图片帧生成视频的文件
% by lfl

close all;
clear,clc;

fps = 10; % 视频帧率
paperTitle = 'scale'; % 针对的会议或期刊名称和作者
framesPath = ['results_fig/']; % 图像序列所在路径
videosPath = ['videos/']; % 创建视频文件夹
if ~exist(videosPath, 'dir')
    mkdir(videosPath);
end

t_video_make_st = clock;

video = [videosPath 'fig2.avi']; % 即将制作的视频

video_object = VideoWriter(video);  % 创建avi视频文件对象
video_object.FrameRate = fps;
open(video_object); % 打开文件等待写入
for seq_count = 1 : 157 % 607  157
    video_name = ['fig2_', num2str(seq_count), '.jpg'];
    frames = imread([framesPath video_name]);
    writeVideo(video_object, frames); % 写入内容
end
close(video_object); % 关闭文件
fprintf(' End!\n');

t_video_make_end = clock;
t_video_make = etime(t_video_make_end, t_video_make_st);
fprintf('End making videos, total time spent: %.2fs. All videos are in: %s\n', t_video_make, videosPath);