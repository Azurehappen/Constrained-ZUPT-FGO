%% generate results;
posTrigger = 0;
stepTrigger = 0;
ekf_function(posTrigger, stepTrigger, "../results/results_ekf_zupt.csv");

posTrigger = 0;
stepTrigger = 1;
ekf_function(posTrigger, stepTrigger, "../results/results_ekf_zupt_step.csv");

posTrigger = 1;
stepTrigger = 0;
ekf_function(posTrigger, stepTrigger, "../results/results_ekf_zupt_pos.csv");

posTrigger = 1;
stepTrigger = 1;
ekf_function(posTrigger, stepTrigger, "../results/results_ekf_zupt_pos_step.csv");

close all;
