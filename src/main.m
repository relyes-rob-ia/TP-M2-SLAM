% M2-EST-TP1 
% M2 Bloc 3 : TP d'Estimation
% Kalman Filter
% Date of creation 10/02/2020
close all;

affichage = 0;
M = 5;
[N, T, Z, F, Hfull, mX0, PX0, Qw, Rv, X] = simulationDonnees(affichage);

if affichage == 1
    return;
end

%Quality analyse of the filter
Qw = Qw * 1;
Rv = Rv * 1;

%mX0 = 10*[1; 2; 3; 4; 5; 6];
%PX0 = 10* [1, 0, 0, 0, 0, 1 ;
       %0, 1, 0, 0, 1, 0 ;
       %0, 0, 1, 1, 0, 0 ;
       %0, 0, 1, 1, 0, 0 ;
       %0, 1, 0, 0, 1, 0 ;
       %1, 0, 0, 0, 0, 1 ]
%Qw = [0, 0, 0, 0, 0, 0 ;
       %0, 0, 0, 0, 0, 0 ;
       %0, 0, 0, 0, 0, 0 ;
       %0, 0, 0, 0, 0, 0 ;
       %0, 0, 0, 0, 0, 0 ;
       %0, 0, 0, 0, 0, 0 ]
%Rv = [0, 0, 0, 0 ;
       %0, 0, 0, 0 ;
       %0, 0, 0, 0 ;
       %0, 0, 0, 0 ]
       
%Others examples for the 10th question
       
%mX0 += (rand(size(mX0)) * 2 - 1);
%PX0 += (rand(size(PX0)) * 2 - 1);
%Qw += (rand(size(Qw)) - 0.5);
%Rv += (rand(size(Rv)) - 0.5);
       

%Pre calculation       
[z_list, h_list, r_list] = prepare(N, Z, Hfull, Rv);

kalman = cell(N, M);
kalman(1, :) = {mX0, PX0, NaN, NaN, NaN};

for k = 1:N    
    % Prediction    
    Xpred = F * kalman{k,1};
    Ppred = F * kalman{k,2} * F' + Qw;
    
    % Update
    z_k1 = z_list{k};
    H_k1 = h_list{k}; 
    R_k1 = r_list{k};
    
    if not(isnan(z_k1(1)))
        Gain = Ppred * H_k1' * inv(R_k1 + H_k1 * Ppred * H_k1');
        Xest = Xpred + Gain * (z_k1 - H_k1 * Xpred);
        Pest = Ppred - Gain * H_k1 * Ppred;
    else
        Gain = zeros(6, 0);
        Xest = Xpred;
        Pest = Ppred;
    end
    
    
    %Domain state (hide) vector
    ellipse(Xest(1:2),Pest(1:2,1:2),'r');
    hold on;
    
    %Position of the robot predicted
    if k<50
        plot(X(1,k+1),X(2,k+1),'xg');
    end
    plot(Xest(1),Xest(2),'xk');
    
    kalman(k+1, :) = {Xpred, Ppred, Xest, Pest, Gain};
end

all_Xest = kalman(2:end, 3);
tmp = zeros(50, 6);
for i = 1:size(all_Xest, 1)
    
    tmp(i, :) = all_Xest{i, :};
end
all_Xest = tmp;
error = rmse(all_Xest, X');
fprintf("Global error = %f\n", error);



