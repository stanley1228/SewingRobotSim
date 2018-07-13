close all

DEF_X=1;
DEF_Y=2;
DEF_Z=3;
DEF_alpha=4;
DEF_beta=5;
DEF_gama=6;
DEF_rednt_alpha=7;

DEF_TIME_SEG=[3,12,18,28];
DEF_Z_LIM=80;
DEF_Z_TICK=40;


%% ========Cartesian space reference and simulated feedback ========%%
%right hand
figure;
for i=DEF_X:1:DEF_Z
    subplot(3,1,i),plot(PathPlanPointRec_R.time,PathPlanPointRec_R.data(:,i),'--r','LineWidth',3); 
    hold on;
    subplot(3,1,i),plot(PathIFKPointRec_R.time,PathIFKPointRec_R.data(:,i),'-b','LineWidth',2); 
    xlabel('t (sec)');
   
    grid on;
    legend('reference','simulated');
    
    if i==DEF_X
        title('t versus x of right arm'); 
        ylabel('x (mm)');
    elseif i==DEF_Y
        title('t versus y of right arm'); 
        ylabel('y (mm)');
    elseif i==DEF_Z
        %ylim([0,DEF_Z_LIM]);
        %set(gca,'ytick',[0:DEF_Z_TICK:DEF_Z_LIM]);
        title('t versus z of right arm') ; 
        ylabel('z (mm)');
        
    end
    
     %draw segment dash
    for time_seg=1:1:3
         ylim_value=get(gca,'Ylim');%get the max of the y limit
        subplot(3,1,i),plot([DEF_TIME_SEG(time_seg),DEF_TIME_SEG(time_seg)],ylim_value,'k--'); 
    end 
end

figure;
for i=DEF_alpha:1:DEF_rednt_alpha
    subplot(4,1,i-3),plot(PathPlanPointRec_R.time,PathPlanPointRec_R.data(:,i),'-r','LineWidth',2); 
    hold on;
%     subplot(3,1,i),plot(PathIFKPointRec_R.time,PathIFKPointRec_R.data(:,i),'-b','LineWidth',2); 
    xlabel('t (sec)');
   
    grid on;
    legend('reference');
    
    if i==DEF_alpha
        title('t versus alpha of right arm'); 
        ylabel('theta');
    elseif i==DEF_beta
        title('t versus beta of right arm'); 
        ylabel('theta');
    elseif i==DEF_gama
        title('t versus gamma of right arm') ; 
        ylabel('theta');
     elseif i==DEF_rednt_alpha
        title('t versus rednt alpha of right arm') ; 
        ylabel('theta');    
    end
end


%left hand
figure;
for i=DEF_X:1:DEF_Z
    subplot(3,1,i),plot(PathPlanPointRec_L.time,PathPlanPointRec_L.data(:,i),'--r','LineWidth',3); 
    hold on;
    subplot(3,1,i),plot(PathIFKPointRec_L.time,PathIFKPointRec_L.data(:,i),'-b','LineWidth',2); 
    xlabel('t (sec)');
   
    grid on;
    legend('reference','simulated');
    
    if i==DEF_X
        title('t versus x of left arm'); 
        ylabel('x (mm)');
    elseif i==DEF_Y
        title('t versus y of left arm'); 
        ylabel('y (mm)');
    elseif i==DEF_Z
        %ylim([29,31]);
        %set(gca,'ytick',[29:1:31]);
        title('t versus z of left arm') ; 
        ylabel('z (mm)');
    end
    
    %draw segment dash
    for time_seg=1:1:3
         ylim_value=get(gca,'Ylim');%get the max of the y limit
        subplot(3,1,i),plot([DEF_TIME_SEG(time_seg),DEF_TIME_SEG(time_seg)],ylim_value,'k--'); 
    end 
end

figure;
for i=DEF_alpha:1:DEF_rednt_alpha
    subplot(4,1,i-3),plot(PathPlanPointRec_L.time,PathPlanPointRec_L.data(:,i),'-r','LineWidth',2); 
    hold on;
%     subplot(3,1,i),plot(PathIFKPointRec_L.time,PathIFKPointRec_L.data(:,i),'-b','LineWidth',2); 
    xlabel('t (sec)');
   
    grid on;
    legend('reference');
    
    if i==DEF_alpha
        title('t versus alpha of left arm'); 
        ylabel('theta');
    elseif i==DEF_beta
        title('t versus beta of left arm'); 
        ylabel('theta');
    elseif i==DEF_gama
        title('t versus gamma of left arm') ; 
        ylabel('theta');
     elseif i==DEF_rednt_alpha
        title('t versus rednt alpha of left arm') ; 
        ylabel('theta');    
    end
end

%% ========motor reference input  ========%%
%right hand
figure;
for i=1:7
    plot(IK_out_R.time,IK_out_R.Data(:,i),'LineWidth',2);
    hold on;
end
legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');
xlabel('t (sec)');
ylabel('angle (degree)');
grid on;
title('IK out Joint angles of the right arm') ; 

%left hand
figure;
for i=1:7
    plot(IK_out_L.time,IK_out_L.Data(:,i),'LineWidth',2);
    hold on;
end
legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');
xlabel('t (sec)');
ylabel('angle (degree)');
grid on;
title('IK out Joint angles of the left arm') ; 

%% ========motor output  ========%%
%right hand
figure;
for i=1:7
    plot(motor_out_R.time,motor_out_R.Data(:,i),'LineWidth',2);
    hold on;
end
  
for time_seg=1:1:3%draw segment dash
    ylim_value=get(gca,'Ylim');%get the max of the y limit
    plot([DEF_TIME_SEG(time_seg),DEF_TIME_SEG(time_seg)],ylim_value,'k--'); 
end 
legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');
xlabel('t (sec)');
ylabel('angle (degree)');
grid on;
title('Joint angles of the right arm') ; 

%left hand
figure;
for i=1:7
    plot(motor_out_L.time,motor_out_L.Data(:,i),'LineWidth',2);
    hold on;
end

for time_seg=1:1:3%draw segment dash
    ylim_value=get(gca,'Ylim');%get the max of the y limit
    plot([DEF_TIME_SEG(time_seg),DEF_TIME_SEG(time_seg)],ylim_value,'k--'); 
end 
legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');
xlabel('t (sec)');
ylabel('angle (degree)');
grid on;
title('Joint angles of the left arm') ; 

%% ========error in joint space  ========%%
%right hand
figure;
for i=1:7
    plot(err_joint_R.time,err_joint_R.Data(:,i),'LineWidth',2);
    hold on;
end
legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');
xlabel('t (sec)');
ylabel('angle (degree)');
grid on;
title('Tracking errors in joint space of right arm') ; 

%left hand
figure;
for i=1:7
    plot(err_joint_L.time,err_joint_L.Data(:,i),'LineWidth',2);
    hold on;
end
legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');
xlabel('t (sec)');
ylabel('angle (degree)');
grid on;
title('Tracking errors in joint space of left arm') ; 

%% ========error  in the Cartesian space ========%%
%right hand
figure;
for i=1:3
    plot(err_R.time,err_R.Data(:,i),'LineWidth',2);
    hold on;
end

for time_seg=1:1:3%draw segment dash
    ylim_value=get(gca,'Ylim');%get the max of the y limit
    plot([DEF_TIME_SEG(time_seg),DEF_TIME_SEG(time_seg)],ylim_value,'k--'); 
end 

legend('x','y','z');
xlabel('t (sec)');
ylabel('tracking errors (mm)');
grid on;
title('Tracking errors in the Cartesian space of the right arm') ; 

%left hand
figure;
for i=1:3
    plot(err_L.time,err_L.Data(:,i),'LineWidth',2);
    hold on;
end

for time_seg=1:1:3%draw segment dash
    ylim_value=get(gca,'Ylim');%get the max of the y limit
    plot([DEF_TIME_SEG(time_seg),DEF_TIME_SEG(time_seg)],ylim_value,'k--'); 
end 

legend('x','y','z');
xlabel('t (sec)');
ylabel('tracking errors (mm)');
grid on;
title('Tracking errors in the Cartesian space of the left arm') ; 

% %================================================
% %== compare to experiment result
% %================================================
%right hand
figure;
Joint_R = csvread('D://SewJoint_FeedBack_R_rec10ms_cmd40ms_sewing_load.csv');
abst=Joint_R(:,1);  
 
Cartesian_FB_R=timeseries;
for i=1:1:size(Joint_R,1)
    theta_R=Joint_R(i,2:8); theta_R=theta_R*pi/180;
    [out_x_end_R,out_y_end_R,out_z_end_R,out_alpha_R,out_beta_R,out_gamma_R,P_R,RotationM_R] = FK_7DOF_FB7roll(DEF_RIGHT_HAND,L0,L1,L2,L3,L4,L5,x_base_R,y_base_R,z_base_R,theta_R);
    Cartesian_FB_R=addsample(Cartesian_FB_R,'Time',Joint_R(i,1),'Data',[out_x_end_R,out_y_end_R,out_z_end_R]);
end 

for i=DEF_X:1:DEF_Z
    subplot(3,1,i),plot(PathPlanPointRec_R.time,PathPlanPointRec_R.data(:,i),'--r','LineWidth',3); 
    hold on;
    subplot(3,1,i),plot(Cartesian_FB_R.time,Cartesian_FB_R.data(:,i),'-b','LineWidth',2); 
    xlabel('t (sec)');
   
    grid on;
    legend('reference','experiment');
    
    if i==DEF_X
        title('t versus x of right arm'); 
        ylabel('x (mm)');
    elseif i==DEF_Y
        title('t versus y of right arm'); 
        ylabel('y (mm)');
    elseif i==DEF_Z
        ylim([0,80]);
        set(gca,'ytick',[0:40:80]);
        title('t versus z of right arm') ; 
        ylabel('z (mm)');
    end
    
    %draw segment dash
    for time_seg=1:1:3
         ylim_value=get(gca,'Ylim');%get the max of the y limit
        subplot(3,1,i),plot([DEF_TIME_SEG(time_seg),DEF_TIME_SEG(time_seg)],ylim_value,'k--'); 
    end 
end

%left hand
figure;
Joint_L = csvread('D://SewJoint_FeedBack_L_rec10ms_cmd40ms_sewing_load.csv');
abst=Joint_L(:,1);  

Cartesian_FB_L=timeseries;

for i=1:1:size(Joint_L,1)
    theta_L=Joint_L(i,2:8);
    theta_L=theta_L*pi/180;
    [out_x_end_L,out_y_end_L,out_z_end_L,out_alpha_L,out_beta_L,out_gamma_L,P_L,RotationM_L] = FK_7DOF_FB7roll(DEF_LEFT_HAND,L0,L1,L2,L3,L4,L5,x_base_L,y_base_L,z_base_L,theta_L);
    Cartesian_FB_L=addsample(Cartesian_FB_L,'Time',Joint_L(i,1),'Data',[out_x_end_L,out_y_end_L,out_z_end_L]);
end 

for i=DEF_X:1:DEF_Z
    subplot(3,1,i),plot(PathPlanPointRec_L.time,PathPlanPointRec_L.data(:,i),'--r','LineWidth',3); 
    hold on;
    subplot(3,1,i),plot(Cartesian_FB_L.time,Cartesian_FB_L.data(:,i),'-b','LineWidth',2); 
    xlabel('t (sec)');
   
    grid on;
    legend('reference','experiment');
    
    if i==DEF_X
        title('t versus x of left arm'); 
        ylabel('x (mm)');
    elseif i==DEF_Y
        title('t versus y of left arm'), 
        ylabel('y (mm)');
    elseif i==DEF_Z
        ylim([0,80]);
        set(gca,'ytick',[0:40:80]);
        title('t versus z of left arm') ; 
        ylabel('z (mm)');
    end
    
    %draw segment dash
    for time_seg=1:1:3
         ylim_value=get(gca,'Ylim');%get the max of the y limit
        subplot(3,1,i),plot([DEF_TIME_SEG(time_seg),DEF_TIME_SEG(time_seg)],ylim_value,'k--'); 
    end 
end

% % %===============================================================
% % %==error in Cartesian space experiment result
% % %=========================Cartesian_FB=======================
% %right hand
% figure;
% 
% err_Cartesian_FB_R=timeseries(PathPlanPointRec_R.data(:,1:3)-Cartesian_FB_R.data(:,1:3),PathPlanPointRec_R.time);
% for i=1:3
%     plot(err_Cartesian_FB_R.time,err_Cartesian_FB_R.Data(:,i),'LineWidth',2);
%     hold on;
% end
% legend('x','y','z');
% xlabel('t (sec)');
% ylabel('tracking errors (mm)');
% grid on;
% title('Experimental tracking errors in the Cartesian space of the right arm') ; 
% 
% for time_seg=1:1:3%draw segment dash
%     ylim_value=get(gca,'Ylim');%get the max of the y limit
%     plot([DEF_TIME_SEG(time_seg),DEF_TIME_SEG(time_seg)],ylim_value,'k--'); 
% end 
% 
% %left hand
% figure;
% err_Cartesian_FB_L=timeseries(PathPlanPointRec_L.data(:,1:3)-Cartesian_FB_L.data(:,1:3),PathPlanPointRec_L.time);
% for i=1:3
%     plot(err_Cartesian_FB_L.time,err_Cartesian_FB_L.Data(:,i),'LineWidth',2);
%     hold on;
% end
% legend('x','y','z');
% xlabel('t (sec)');
% ylabel('tracking errors (mm)');
% grid on;
% title('Experimental tracking errors in the Cartesian space of the left arm') ; 
% 
% for time_seg=1:1:3%draw segment dash
%     ylim_value=get(gca,'Ylim');%get the max of the y limit
%     plot([DEF_TIME_SEG(time_seg),DEF_TIME_SEG(time_seg)],ylim_value,'k--'); 
% end 