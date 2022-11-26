close all
clear

max_sample = 2000;

for Obs_int = 1:5
    for State_int = 1:5
        for RRT_star = [0, 1]
            for grad_heuristic = [0, 1]
                Name_result = ['results/Obs', num2str(Obs_int), '_State', num2str(State_int), ...
                        '_RRTStar', num2str(RRT_star), '_Heuristic', num2str(grad_heuristic), '.mat'];
                cload(Name_result);
                
                Name = {'Time', 'Counter_rand', 'NumOfNodes', 'PathLength'};
                Time_list = [];
                Counter_rand_list = [];
                NumOfNodes_list = [];
                PathLength_list = [];
                Success_rate_list = [];
                Ave_length = [];
                for i = 1:length(result)
                    Time_list = [Time_list; result(i).Time];
                    Counter_rand_list = [Counter_rand_list; result(i).Counter_rand];
                    NumOfNodes_list = [NumOfNodes_list; result(i).NumOfNodes];
                    Counter_rand_list = [Counter_rand_list; result(i).Counter_rand];
                    PathLength_list = [PathLength_list; result(i).PathLength];
                end
                
                success_rate = sum(Counter_rand_list <= max_sample) / length(Counter_rand_list);
                avg_time = mean(Time_list);
                sigma = std(Time_list, 0);
                min_time = min(Time_list);
                max_time = max(Time_list);
                avg_length = mean(PathLength_list);
                
                
                if exist('results/Count_result.mat')
                    load('results/Count_result.mat');
                    Count_result(end+1,:) = [Obs_int, State_int, RRT_star, grad_heuristic, ...
                        success_rate, avg_time, sigma, min_time, max_time, avg_length];
                else
                    Count_result = [Obs_int, State_int, RRT_star, grad_heuristic, ...
                        success_rate, avg_time, sigma, min_time, max_time, avg_length];
                end
                save('results/Count_result.mat', 'Count_result');
            end
        end
    end
end