clearvars;
%%
mqttClient = mqttclient("mqtt://rasticvm.lan"); %connect to mqtt broker
%%
SubscrTable = subscribe(mqttClient,"rb/limo780/rot");
SubscrTable = subscribe(mqttClient,"rb/limo780/pos");
%SubscrTable = subscribe(mqttClient, mqttTopic, Name=values);

%%
Ts = 0.01;
N = 1000;
dataLog = [];   % must be numeric

for k = 1:N

    pause(Ts);
    t = Ts*k;
    
    posTable = read(mqttClient, Topic="rb/limo780/pos");
    
    if ~isempty(posTable)
        rawData = posTable.Data{end};           % e.g. "[-1.4629, 0.1572, 0.1370]"
        rawData = erase(rawData, ["[", "]"]);   % remove brackets
        numericPos = str2double(split(rawData, ","))';  % numeric row
    else
        numericPos = [NaN NaN NaN];
    end
    
    numericPos = numericPos(:)';  % ensure row
    dataLog = [dataLog; t, numericPos];  % append safely

end

disp(dataLog)

% rot = read(mqttClient, Topic= "rb/limo780/rot");
% pos = read(mqttClient, Topic= "rb/limo780/pos");
% peek(mqttClient)

%peek for the latest one 
%transmitted at 120 Hz

%%
% figure
% scatter3(dataLog(:,2), dataLog(:,4), dataLog(:,3), 50, dataLog(:,1), 'filled')
% grid on
% xlabel('Col 2')
% ylabel('Col 3')
% zlabel('Col 4')
% title('3D scatter of MoCap data colored by time')
% colorbar

figure
scatter3(dataLog(:,2), dataLog(:,3), dataLog(:,4), 50, dataLog(:,1), 'filled')
grid on
xlabel('Col 2')
ylabel('Col 3')
zlabel('Col 4')
title('3D scatter of MoCap data colored by time')
colorbar

%plot in live time later

    
