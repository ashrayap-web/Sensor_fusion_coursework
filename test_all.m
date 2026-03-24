datasets = {'task1_1 1.mat','task1_2 1.mat','task1_3.mat','task2_1 1.mat','task2_2 1.mat','task2_3 1.mat','task2_4.mat'};
for di = 1:numel(datasets)
    clear myEKF
    d = load(datasets{di}); o = d.out;
    acc_all=squeeze(o.Sensor_ACCEL.signals.values)';
    gyro_all=squeeze(o.Sensor_GYRO.signals.values)';
    mag_all=squeeze(o.Sensor_MAG.signals.values)';
    tof1_all=o.Sensor_ToF1.signals.values;
    tof2_all=o.Sensor_ToF2.signals.values;
    tof3_all=o.Sensor_ToF3.signals.values;
    temp_all=o.Sensor_Temp.signals.values;
    lp_acc_all=o.Sensor_LP_ACCEL.signals.values;
    gt_pos=o.GT_position.signals.values;
    gt_quat=o.GT_rotation.signals.values;
    N=size(acc_all,1);
    gt_yaw=zeros(N,1);
    for i=1:N
        W_=gt_quat(i,1);X_=gt_quat(i,2);Y_=gt_quat(i,3);Z_=gt_quat(i,4);
        gt_yaw(i)=atan2(2*(W_*Z_+X_*Y_),1-2*(Y_^2+Z_^2));
    end
    gt_init=[gt_pos(1,1),gt_pos(1,2),gt_yaw(1)];
    X_History=zeros(8,N);
    for k=1:N
        mg=get_dat(mag_all,k);t1=get_dat(tof1_all,k);
        t2=get_dat(tof2_all,k);t3=get_dat(tof3_all,k);
        tmp=get_dat(temp_all,k);lp=get_dat(lp_acc_all,k);
        if k==1
            [X_Est,~]=myEKF(acc_all(k,:),gyro_all(k,:),mg,t1,t2,t3,tmp,lp,gt_init);
        else
            [X_Est,~]=myEKF(acc_all(k,:),gyro_all(k,:),mg,t1,t2,t3,tmp,lp);
        end
        X_History(:,k)=X_Est;
    end
    ekf_xy=X_History([1,4],1:N)';
    gt_xy=gt_pos(1:N,1:2);
    rmse=sqrt(mean(sum((ekf_xy-gt_xy).^2,2)));
    heading_err=rad2deg(wrapToPi(X_History(7,:)'-gt_yaw));
    fprintf('%-18s  RMSE=%.4fm  heading=%.1fdeg\n', datasets{di}, rmse, sqrt(mean(heading_err.^2)));
end
function v=get_dat(arr,k); if k<=size(arr,1);v=arr(k,:);else;v=NaN(1,size(arr,2));end; end
