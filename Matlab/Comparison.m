Franka_Initial = 22500;
Franka_Hourly = 0.5956;

Human_Hourly = 24;

Human_Array = zeros(2800,1);
Franka_Array = zeros(2800,1);
Franka_Array(1) = Franka_Initial;
Human_Array(1) = Human_Hourly;

for i=2:length(Franka_Array)
Franka_Array(i) = Franka_Array(i-1)+Franka_Hourly;
Human_Array(i) = Human_Hourly*i;
end

k = linspace(1,length(Franka_Array),length(Franka_Array));
figure(1)
plot(k,Human_Array);
hold on
plot(k,Franka_Array);
hold off
ylabel('Cost [$]');
xlabel('Time [Hr]');
legend('Cost for Human Worker','Cost for Robotic Worker',Location='northwest');
title("Human vs Robot Working Cost");

Human_Stock_Times = [7.03 6.90 8.35 8.36 9.21 6.85 6.95 6.24 5.86 6.99];
Avg_Human_Item = mean(Human_Stock_Times)/4;
Robot_Stock_Times = [121.54 114.6 127.3 123.28 119.07 121.158];
Avg_Robot_Item = mean(Robot_Stock_Times)/4;
% disp(Avg_Robot_Item)
% disp(Avg_Human_Item)

Robot_Item_Hour = 3600/Avg_Robot_Item;
Human_Item_Hour = 3600/Avg_Human_Item;

k = linspace(1,168,168);
RobotGraph = [];
for i=1:168
RobotGraph(i) = Robot_Item_Hour*i;

end

runningTotal = 1;
HumanGraph = [];
for j =1:7
for w = 1:8
if runningTotal == 1
    HumanGraph(runningTotal) = Human_Item_Hour;
else

HumanGraph(runningTotal) = HumanGraph(runningTotal-1) + Human_Item_Hour;
end
runningTotal = runningTotal +  1;
end
for m = 1:16
HumanGraph(runningTotal) = HumanGraph(runningTotal - 1)
runningTotal = runningTotal +  1;

end
end

figure(2)
plot(k,HumanGraph);
hold on 
plot(k,RobotGraph);
hold off
title("Robot vs Human Stocking Efficiency");
legend('Human','Robot',Location='northwest');
ylabel('Items')
xlabel('Time [Hr]');
