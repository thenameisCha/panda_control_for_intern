close all; clear all; clc;
 p_smooth = 0.9;

    fileID = fopen('build/ETank_data.txt', 'r');
    formatspec = '%f %f %f %f %f';
    sizeB = [6, inf];
    B = fscanf(fileID, formatspec, sizeB);
    fclose(fileID);
    B = B';
    figure(1);
    plot(B(:,1), B(:,2));
    hold on;
    xdot_smooth = csaps(B(:,1),B(:,3),p_smooth,B(:,1));
    plot(B(:,1), xdot_smooth);
    legend("x desired dot", "x dot");
    xlabel("play time, [s]");
    
    ylabel("task space Z velocity, [m/s]");    

    figure(2);
    ylim([-0.2, 1.2]);
    plot(B(:,1), B(:,4));
    hold on;
    plot(B(:,1), B(:,5));
    ylim([-0.2 1.2])
    xlabel("play time, [s]");
    ylabel("Gamma");
    legend("x_(dot) gamma", "x_(desired, dot) gamma");
    

    figure(3)
    plot(B(:,1), B(:,6));
    xlabel("play time, [s]");
    ylabel("S_ur [J]");

    fileID = fopen('build/Power_data.txt', 'r');
    formatspec = '%f %f %f %f %f';
    sizeC = [5, inf];
    C = fscanf(fileID, formatspec, sizeC);
    fclose(fileID);
    C = C';
    figure(4);
    power0_smooth = csaps(C(:,1)', C(:,2)', p_smooth, C(:,1));
    plot(C(:,1), power0_smooth);
    hold on;
    power1_smooth = csaps(C(:,1)', C(:,3)', p_smooth, C(:,1));
    plot(C(:,1), power1_smooth);
    hold on;
    power2_smooth = csaps(C(:,1)', C(:,4)', p_smooth, C(:,1));
    plot(C(:,1), power2_smooth);
    legend("x dot*F_F", "x desired dot*F_F", "x desired dot*F ext");
    xlabel("play time, [s]");
    ylabel("Port Power, [J/s]");

    figure(5);
    plot(C(:,1), C(:,5));
    xlabel("play time, [s]");
    ylabel("Tank Energy, [J]");
    ylim([8,13]);
    
    fileID = fopen('build/Position_data.txt', 'r');
    formatspec = '%f %f %f %f';
    sizeA = [4, inf];
    A = fscanf(fileID, formatspec, sizeA);
    fclose(fileID);
    A = A';
    
    figure(6);
    plot(A(:,1), A(:, 2));
    hold on;
    plot(A(:,1), A(:, 3));
    xlabel("play time, [s]");
    ylabel("position [m]");
    legend("x desired", "x current");
    
    figure(7);
    f_ext_smooth = csaps(A(:,1)', A(:,4)', p_smooth, A(:,1));
    plot(A(:,1), f_ext_smooth);
    xlabel("play time, [s]");
    ylabel("f ext y-axis[N]");

    
  