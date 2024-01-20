function handles= plot_results(model,meas,est)

% [X_track,k_birth,k_death]= extract_tracks(truth.X,truth.track_list,truth.total_tracks);

labelcount= countestlabels();
colorarray= makecolorarray(labelcount);
est.total_tracks= labelcount;
% est.track_list= cell(truth.K,1);
est.track_list= cell(meas.K,1);
for k=1:meas.K  %meas.K ? 
    for eidx=1:size(est.X{k},2)
        est.track_list{k} = [est.track_list{k} assigncolor(est.L{k}(:,eidx))];
    end
end
[Y_track,l_birth,l_death]= extract_tracks(est.X,est.track_list,est.total_tracks)

%plot x tracks and measurements in x/y
figure; tracking= gcf; hold on;

%plot x measurement
subplot(211); box on; 


%plot x estimate
for t=1:size(Y_track,3) %darle como input el numero de mediciones , este sirve
    hline2= line(1:meas.K,Y_track(1,:,t),'LineStyle','none','Marker','.','Markersize',8,'Color',colorarray.rgb(t,:));
end

%plot y measurement
subplot(212); box on;
    

%plot y estimate
for t=1:size(Y_track,3)   %Y_track 
    hline2= line(1:meas.K,Y_track(3,:,t),'LineStyle','none','Marker','.','Markersize',8,'Color',colorarray.rgb(t,:));
end

 subplot(211); xlabel('Tiempo'); ylabel('Coordenada x (m)');
% set(gca, 'XLim',[1 truth.K]); set(gca, 'YLim',[-model.range_c(2,2) model.range_c(2,2)]);
% legend([hline2 hline1 hlined],'Estimates          ','True tracks','Measurements');
grid on
% 
 subplot(212); xlabel('Tiempo'); ylabel('Coordenada y (m)');
% set(gca, 'XLim',[1 truth.K]); set(gca, 'YLim',[ model.range_c(1,2) model.range_c(2,2)] );
%legend([yhline2 yhline1 yhlined],'Estimates          ','True tracks','Measurements');
grid on
hold off

figure;
angles_fig = gcf;
hold on;

% Plot angles
for t = 1:size(Y_track, 3)
    x = Y_track(1, :, t);
    y = Y_track(3, :, t);
    angles = atan2d(y, x);
    hline3 = line(1:meas.K, angles, 'LineStyle', 'none', 'Marker', '.', 'Markersize', 8, 'Color', colorarray.rgb(t,:));
end

xlabel('Tiempo');
ylabel('Angulos (grados)');
title('Angulos en el tiempo');
xlim([0, meas.K])
grid on
hold off
% Ajusta el espacio entre subgráficos




function ca= makecolorarray(nlabels)
    lower= 0.1;
    upper= 0.9;
    rrr= rand(1,nlabels)*(upper-lower)+lower;
    ggg= rand(1,nlabels)*(upper-lower)+lower;
    bbb= rand(1,nlabels)*(upper-lower)+lower;
    ca.rgb= [rrr; ggg; bbb]';
    ca.lab= cell(nlabels,1);
    ca.cnt= 0;   
end

function idx= assigncolor(label)
    str= sprintf('%i*',label);
    tmp= strcmp(str,colorarray.lab);
    if any(tmp)
        idx= find(tmp);
    else
        colorarray.cnt= colorarray.cnt + 1;
        colorarray.lab{colorarray.cnt}= str;
        idx= colorarray.cnt;
    end
end

function count= countestlabels
    labelstack= [];
    for k=1:meas.K
        labelstack= [labelstack est.L{k}];
    end
    [c,~,~]= unique(labelstack','rows');
    count=size(c,1);
end

end


function [X_track,k_birth,k_death]= extract_tracks(X,track_list,total_tracks)

K= size(X,1); 
x_dim= size(X{K},1); 
k=K-1; while x_dim==0, x_dim= size(X{k},1); k= k-1; end
X_track= NaN(x_dim,K,total_tracks);
k_birth= zeros(total_tracks,1);
k_death= zeros(total_tracks,1);

max_idx= 0;
for k=1:K
    if ~isempty(X{k})
        X_track(:,k,track_list{k})= X{k};
    end
    if max(track_list{k})> max_idx %new target born?
        idx= find(track_list{k}> max_idx);
        k_birth(track_list{k}(idx))= k;
    end
    if ~isempty(track_list{k}), max_idx= max(track_list{k}); end
    k_death(track_list{k})= k;
end
end


function Xc= get_comps(X,c)

if isempty(X)
    Xc= [];
else
    Xc= X(c,:);
end
end